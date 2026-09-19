#pragma once
#include "kalibr2/detector.hpp"
#include "kalibr2/source.hpp"
#include <cstddef>
#include <string>

namespace kalibr2 {
struct PipelineOptions {
  int threads{1};
  std::size_t image_memory_bytes{};
};
struct ExtractedData {
  std::vector<Observation> observations;
  std::vector<ImuSample> imu;
  std::size_t frames{}, detected_frames{}, max_in_flight{};
  double elapsed_seconds{};
  // V1 caches predate backend metadata and are interpreted as AprilTag 3.
  DetectorBackend detector_backend{DetectorBackend::AprilTag3};
  int tag_border{2};
  double decimate{1.0};
  bool from_cache{};
};
// By default, extraction keeps up to `threads` images in flight. When
// image_memory_bytes is non-zero, it reserves decoded + encoded pixel buffers
// and can reduce in-flight images; codec/detector workspace, bag storage
// buffers, observations, IMU and solver allocations are additional.
ExtractedData extract(FrameSource& source, const CameraConfig& camera,
                      const GridConfig& grid, const DetectorOptions& detector,
                      const PipelineOptions& options);
void write_cache(const std::string& path, const ExtractedData& data,
                 const CameraConfig& camera, const GridConfig& grid);
ExtractedData read_cache(const std::string& path, const CameraConfig& camera,
                         const GridConfig& grid);
} // namespace kalibr2
