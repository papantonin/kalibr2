#pragma once

#include "kalibr2/types.hpp"

namespace kalibr2 {

// Offline, global-shutter, single-camera calibration with fixed intrinsics.
// The target defines the world frame. The trajectory is a cubic cumulative
// SO(3) spline plus a cubic Euclidean position spline; biases are constant.
// T_cam_imu transforms IMU coordinates into camera coordinates, and
// t_imu = t_camera + timeshift_cam_imu. Invalid/insufficient inputs throw.
// A numerically unsuccessful or boundary-limited fit has converged == false.
CalibrationResult calibrate(const std::vector<Observation>& observations,
                            const std::vector<ImuSample>& imu,
                            const CameraConfig& camera,
                            const ImuConfig& imu_config,
                            const SolverOptions& options = {});

}  // namespace kalibr2
