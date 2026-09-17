#include "kalibr2/detector.hpp"

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace kalibr2 {
namespace {
using Family = std::unique_ptr<apriltag_family_t, decltype(&tag36h11_destroy)>;
using Detector = std::unique_ptr<apriltag_detector_t, decltype(&apriltag_detector_destroy)>;
using Detections = std::unique_ptr<zarray_t, decltype(&apriltag_detections_destroy)>;

void validate(const GridConfig& grid, const DetectorOptions& options) {
  if (grid.rows <= 0 || grid.cols <= 0 ||
      static_cast<std::int64_t>(grid.rows) * grid.cols > 587 ||
      !std::isfinite(grid.tag_size) || grid.tag_size <= 0.0 ||
      !std::isfinite(grid.tag_spacing) || grid.tag_spacing <= 0.0) {
    throw std::invalid_argument("AprilGrid requires positive dimensions, size and spacing, and at most 587 tag36h11 IDs");
  }
  if ((options.tag_border != 1 && options.tag_border != 2) ||
      !std::isfinite(options.quad_decimate) || options.quad_decimate < 1.0 || options.quad_decimate > 8.0 ||
      options.min_tags < 1 || options.min_tags > grid.rows * grid.cols ||
      options.max_hamming < 0 || options.max_hamming > 2 ||
      !std::isfinite(options.min_decision_margin) || options.min_decision_margin < 0.0 ||
      !std::isfinite(options.min_border_distance) || options.min_border_distance < 3.0 ||
      !std::isfinite(options.max_subpixel_displacement_squared) || options.max_subpixel_displacement_squared <= 0.0) {
    throw std::invalid_argument("Invalid AprilGrid detector options (tag_border must be 1 or 2, decimation in [1,8])");
  }
  const double extent = grid.tag_size * (1.0 + grid.tag_spacing) * std::max(grid.rows, grid.cols);
  if (!std::isfinite(extent)) throw std::invalid_argument("AprilGrid metric extent overflows");
}
} // namespace

struct AprilGridDetector::Impl {
  GridConfig grid;
  DetectorOptions options;
  // Destruction is reverse declaration order: the detector releases its decode
  // tables before the associated family is freed.
  Family family{nullptr, tag36h11_destroy};
  Detector detector{nullptr, apriltag_detector_destroy};

  Impl(GridConfig input_grid, DetectorOptions input_options)
      : grid(input_grid), options(input_options) {
    validate(grid, options);
    family.reset(tag36h11_create());
    if (!family) throw std::bad_alloc();

    // AprilTag3's tag36h11 bit ordering differs from the historical row-major
    // words, but represents the same printed tags. Kalibr's PDF generator adds
    // TWO black cells around the 6x6 payload, while upstream AprilTag3 uses one.
    // Adapt only geometric sampling; the codebook and decoding are unchanged.
    const unsigned extra_border = static_cast<unsigned>(options.tag_border - 1);
    for (unsigned i = 0; i < family->nbits; ++i) {
      family->bit_x[i] += extra_border;
      family->bit_y[i] += extra_border;
    }
    family->width_at_border += 2 * static_cast<int>(extra_border);
    family->total_width += 2 * static_cast<int>(extra_border);

    detector.reset(apriltag_detector_create());
    if (!detector) throw std::bad_alloc();
    detector->nthreads = 1;
    detector->quad_decimate = static_cast<float>(options.quad_decimate);
    // Nonzero sigma can modify the caller's image when decimation is one.
    detector->quad_sigma = 0.0F;
    detector->refine_edges = true;
    detector->debug = false;
    apriltag_detector_add_family_bits(detector.get(), family.get(), options.max_hamming);
  }
};

AprilGridDetector::AprilGridDetector(GridConfig grid, DetectorOptions options)
    : impl_(std::make_unique<Impl>(grid, options)) {}
AprilGridDetector::~AprilGridDetector() = default;
AprilGridDetector::AprilGridDetector(AprilGridDetector&&) noexcept = default;
AprilGridDetector& AprilGridDetector::operator=(AprilGridDetector&&) noexcept = default;

Observation AprilGridDetector::detect(const cv::Mat& gray, std::int64_t timestamp_ns) {
  if (!impl_) throw std::logic_error("Cannot use a moved-from AprilGrid detector");
  if (gray.empty() || gray.dims != 2 || gray.type() != CV_8UC1) {
    throw std::invalid_argument("AprilGrid detection requires a nonempty CV_8UC1 image");
  }
  // AprilTag's C implementation performs some offset arithmetic using int.
  // Validate the extent as well as stride to avoid overflow on malformed input.
  constexpr auto max_offset = static_cast<std::size_t>(std::numeric_limits<int>::max());
  if (gray.step[0] > max_offset ||
      static_cast<std::size_t>(gray.rows - 1) * gray.step[0] + gray.cols > max_offset) {
    throw std::invalid_argument("Image extent exceeds AprilTag3's signed 32-bit buffer addressing");
  }
  Observation observation{timestamp_ns, {}};
  if (gray.rows < 9 || gray.cols < 9) return observation;
  const auto& options = impl_->options;
  const auto& grid = impl_->grid;
  image_u8_t image{gray.cols, gray.rows, static_cast<int>(gray.step[0]), gray.data};
  Detections detections(apriltag_detector_detect(impl_->detector.get(), &image),
                        apriltag_detections_destroy);
  if (!detections) throw std::runtime_error("AprilTag3 detection allocation failed");

  std::vector<apriltag_detection_t*> valid;
  valid.reserve(static_cast<std::size_t>(zarray_size(detections.get())));
  for (int i = 0; i < zarray_size(detections.get()); ++i) {
    apriltag_detection_t* detection = nullptr;
    zarray_get(detections.get(), i, &detection);
    if (detection->id < 0 || detection->id >= grid.rows * grid.cols ||
        detection->hamming > options.max_hamming ||
        !std::isfinite(detection->decision_margin) ||
        detection->decision_margin < options.min_decision_margin) continue;
    bool inside = true;
    for (const auto& point : detection->p) {
      const double x = point[0] - 0.5;
      const double y = point[1] - 0.5;
      inside = inside && std::isfinite(x) && std::isfinite(y) &&
               x >= options.min_border_distance && y >= options.min_border_distance &&
               x < gray.cols - options.min_border_distance &&
               y < gray.rows - options.min_border_distance;
    }
    if (inside) valid.push_back(detection);
  }
  std::sort(valid.begin(), valid.end(), [](const auto* a, const auto* b) { return a->id < b->id; });
  for (std::size_t i = 1; i < valid.size(); ++i) {
    if (valid[i - 1]->id == valid[i]->id) {
      throw std::runtime_error("Duplicate AprilGrid tag ID " + std::to_string(valid[i]->id) +
                               " at timestamp " + std::to_string(timestamp_ns) +
                               "; multiple boards or stray tags are visible");
    }
  }
  if (valid.size() < static_cast<std::size_t>(options.min_tags)) return observation;

  std::vector<cv::Point2f> pixels;
  pixels.reserve(4 * valid.size());
  for (const auto* detection : valid) {
    for (const auto& point : detection->p) {
      // AprilTag uses centers at (0.5,0.5); OpenCV uses centers at (0,0).
      pixels.emplace_back(static_cast<float>(point[0] - 0.5), static_cast<float>(point[1] - 0.5));
    }
  }
  const auto raw_pixels = pixels;
  cv::cornerSubPix(gray, pixels, cv::Size(2, 2), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.01));

  observation.corners.reserve(pixels.size());
  const double pitch = grid.tag_size * (1.0 + grid.tag_spacing);
  constexpr int corner_x[4]{0, 1, 1, 0};
  constexpr int corner_y[4]{0, 0, 1, 1};
  for (std::size_t i = 0; i < valid.size(); ++i) {
    const int id = valid[i]->id;
    for (int corner = 0; corner < 4; ++corner) {
      const std::size_t index = 4 * i + corner;
      const cv::Point2f displacement = pixels[index] - raw_pixels[index];
      if (!std::isfinite(pixels[index].x) || !std::isfinite(pixels[index].y) ||
          displacement.dot(displacement) > options.max_subpixel_displacement_squared) continue;
      observation.corners.push_back(Corner{
          id, corner,
          Eigen::Vector2d(pixels[index].x, pixels[index].y),
          Eigen::Vector3d((id % grid.cols) * pitch + corner_x[corner] * grid.tag_size,
                          (id / grid.cols) * pitch + corner_y[corner] * grid.tag_size, 0.0)});
    }
  }
  return observation;
}

} // namespace kalibr2
