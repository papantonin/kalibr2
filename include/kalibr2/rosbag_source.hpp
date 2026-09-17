#pragma once

#include "kalibr2/source.hpp"
#include <memory>
#include <string>

namespace kalibr2 {
// Single-pass ROS 2 bag reader. next() also gathers intervening IMU samples.
// Drain next() before take_imu(); images are released after each caller use.
class RosbagSource final : public FrameSource {
 public:
  RosbagSource(const std::string& uri, const CameraConfig& camera, const ImuConfig& imu);
  ~RosbagSource() override;
  bool next(ImageFrame& frame) override;
  std::vector<ImuSample> take_imu() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace kalibr2
