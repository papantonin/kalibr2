#pragma once

#include "kalibr2/types.hpp"
#include <opencv2/core/mat.hpp>
#include <memory>

namespace kalibr2 {

struct DetectorOptions {
  // Only quad search is decimated; decoding and corner refinement use the
  // original image. One preserves all available image information.
  double quad_decimate{1.0};
  int tag_border{2}; // Kalibr PDF targets: 2; standard AprilTag3 targets: 1.
  int min_tags{4};
  int max_hamming{0};
  double min_decision_margin{0.0};
  double min_border_distance{4.0};
  double max_subpixel_displacement_squared{1.5};
};

// A detector is deliberately single-threaded and not reentrant. The streaming
// pipeline should keep one instance per TBB worker, avoiding nested thread pools.
// No input image is retained or copied; only the detected corners survive detect.
class AprilGridDetector {
public:
  explicit AprilGridDetector(GridConfig grid, DetectorOptions options = {});
  ~AprilGridDetector();
  AprilGridDetector(AprilGridDetector&&) noexcept;
  AprilGridDetector& operator=(AprilGridDetector&&) noexcept;
  AprilGridDetector(const AprilGridDetector&) = delete;
  AprilGridDetector& operator=(const AprilGridDetector&) = delete;

  // Accepts CV_8UC1, including an ROI with padded rows. Empty observations mean
  // insufficient target visibility; malformed input and duplicate IDs throw.
  // Corners are sorted by tag ID, then local corner ID 0..3 (BL, BR, TR, TL
  // on the canonical Kalibr target). Pixel coordinates follow OpenCV convention.
  Observation detect(const cv::Mat& gray, std::int64_t timestamp_ns);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace kalibr2
