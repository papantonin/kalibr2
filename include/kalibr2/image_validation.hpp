#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace kalibr2 {
// Check dimensions before invoking a codec, whose allocation precedes imdecode's return.
// Only PNG and JPEG are accepted. The decoder subsequently checks the full stream.
inline void validate_encoded_dimensions(const unsigned char* data, std::size_t size,
                                       int width, int height) {
  if (!data || width <= 0 || height <= 0)
    throw std::runtime_error("Invalid encoded image or configured dimensions");
  auto be16 = [](const unsigned char* p) -> std::uint32_t {
    return (std::uint32_t{p[0]} << 8) | p[1];
  };
  auto be32 = [](const unsigned char* p) -> std::uint32_t {
    return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) |
           (std::uint32_t{p[2]} << 8) | p[3];
  };
  auto check = [&](std::uint32_t actual_width, std::uint32_t actual_height) {
    if (actual_width != static_cast<std::uint32_t>(width) ||
        actual_height != static_cast<std::uint32_t>(height))
      throw std::runtime_error("Image dimensions " + std::to_string(actual_width) + "x" +
                               std::to_string(actual_height) + " differ from configuration " +
                               std::to_string(width) + "x" + std::to_string(height));
  };
  constexpr unsigned char png_signature[]{137, 80, 78, 71, 13, 10, 26, 10};
  bool png = size >= sizeof(png_signature);
  for (std::size_t i = 0; png && i < sizeof(png_signature); ++i)
    png = data[i] == png_signature[i];
  if (png) {
    if (size < 33 || be32(data + 8) != 13 || data[12] != 'I' || data[13] != 'H' ||
        data[14] != 'D' || data[15] != 'R')
      throw std::runtime_error("Malformed PNG image header");
    check(be32(data + 16), be32(data + 20));
    return;
  }
  if (size < 2 || data[0] != 0xff || data[1] != 0xd8)
    throw std::runtime_error("Only PNG and JPEG encoded images are supported");
  std::size_t pos = 2;
  while (pos < size) {
    if (data[pos] != 0xff) throw std::runtime_error("Malformed JPEG marker");
    while (pos < size && data[pos] == 0xff) ++pos;
    if (pos == size) break;
    const unsigned char marker = data[pos++];
    if (marker == 0xda || marker == 0xd9) break; // Scan/end before frame header.
    if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    if (marker == 0 || marker == 0xd8 || size - pos < 2)
      throw std::runtime_error("Malformed JPEG segment");
    const std::size_t length = be16(data + pos);
    if (length < 2 || length > size - pos)
      throw std::runtime_error("Truncated JPEG segment");
    const bool frame_header = marker >= 0xc0 && marker <= 0xcf &&
                              marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
    if (frame_header) {
      if (length < 8) throw std::runtime_error("Malformed JPEG frame header");
      check(be16(data + pos + 5), be16(data + pos + 3));
      return;
    }
    pos += length;
  }
  throw std::runtime_error("JPEG dimensions not found before image scan");
}
} // namespace kalibr2
