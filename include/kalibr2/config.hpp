#pragma once
#include "kalibr2/types.hpp"
#include <string>
namespace kalibr2 {
CameraConfig load_camera(const std::string& path);
ImuConfig load_imu(const std::string& path);
GridConfig load_grid(const std::string& path);
void save_result(const std::string& directory, const CameraConfig& camera,
                 const CalibrationResult& result);
} // namespace kalibr2
