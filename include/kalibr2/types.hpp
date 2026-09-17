#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace kalibr2 {
struct CameraConfig {
  std::string topic;
  std::string model{"pinhole"};
  std::string distortion_model{"radtan"};
  std::array<double, 4> intrinsics{}; // fx, fy, cx, cy
  std::array<double, 4> distortion{};
  int width{}, height{};
};
struct ImuConfig {
  std::string topic;
  double update_rate{};
  double accelerometer_noise_density{};
  double accelerometer_random_walk{};
  double gyroscope_noise_density{};
  double gyroscope_random_walk{};
};
struct GridConfig {
  int rows{}, cols{};
  double tag_size{}, tag_spacing{}; // spacing is a fraction of tag_size
};
struct Corner {
  int tag_id{}, corner_id{};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
};
struct Observation {
  std::int64_t timestamp_ns{};
  std::vector<Corner> corners;
};
struct ImuSample {
  std::int64_t timestamp_ns{};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};
struct SolverOptions {
  int threads{1};
  int max_iterations{100};
  double knot_spacing{0.05};
  double max_time_offset{0.05};
  double pixel_sigma{1.0};
};
struct CalibrationResult {
  Eigen::Matrix4d T_cam_imu{Eigen::Matrix4d::Identity()};
  double timeshift_cam_imu{}; // t_imu = t_cam + timeshift_cam_imu
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{Eigen::Vector3d::Zero()};
  double initial_cost{}, final_cost{}, reprojection_rmse{};
  int iterations{};
  bool converged{};
  std::string report;
};
} // namespace kalibr2
