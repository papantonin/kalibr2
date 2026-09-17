#pragma once
#include "kalibr2/types.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>

namespace kalibr2 {
struct ImageFrame {
  std::int64_t timestamp_ns{};
  std::shared_ptr<const void> owner; // Keeps a zero-copy ROS mono8 view alive.
  cv::Mat gray;
  std::size_t source_bytes{};
};
class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual bool next(ImageFrame&) = 0;
  virtual std::vector<ImuSample> take_imu() = 0;
};
// Kalibr directory format: cam0/<timestamp_ns>.png and imu0.csv.
std::unique_ptr<FrameSource> directory_source(const std::string& path,
                                             const CameraConfig& camera);
} // namespace kalibr2
