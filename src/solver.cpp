#include "kalibr2/solver.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace kalibr2 {
namespace {
constexpr double kGravity = 9.80665;
using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Knot = std::array<double, 4>;

double seconds(std::int64_t timestamp, std::int64_t origin) {
  return static_cast<double>((static_cast<long double>(timestamp) -
                              static_cast<long double>(origin)) * 1e-9L);
}

template <class T> double scalar(const T& value) { return static_cast<double>(value); }
template <class T, int N> double scalar(const ceres::Jet<T, N>& value) {
  return scalar(value.a);
}
template <class T> Eigen::Quaternion<T> quaternion(const T* p) {
  return Eigen::Quaternion<T>(p[0], p[1], p[2], p[3]);
}
Knot coefficients(const Quat& q) { return {q.w(), q.x(), q.y(), q.z()}; }

template <class T> Eigen::Matrix<T, 3, 1> logarithm(const Eigen::Quaternion<T>& q) {
  const T input[4] = {q.w(), q.x(), q.y(), q.z()};
  Eigen::Matrix<T, 3, 1> output;
  ceres::QuaternionToAngleAxis(input, output.data());
  return output;
}
template <class T> Eigen::Quaternion<T> exponential(const Eigen::Matrix<T, 3, 1>& w) {
  T output[4];
  ceres::AngleAxisToQuaternion(w.data(), output);
  return quaternion(output);
}

template <class T> struct State {
  Eigen::Quaternion<T> rotation;
  Eigen::Matrix<T, 3, 1> position, acceleration, omega;
};

// Four local control poses, with translation blocks following all rotation
// blocks. Analytic time derivatives and AutoDiff parameter derivatives.
// The angular-velocity recurrence is that of Sommer et al., CVPR 2020,
// https://arxiv.org/abs/1911.08860; no finite differencing is used in the fit.
template <class T>
State<T> evaluate(const T* const* blocks, int count, int offset, T u, double h) {
  using Vector = Eigen::Matrix<T, 3, 1>;
  const T u2 = u * u, u3 = u2 * u, one = T(1) - u;
  const T b[4] = {one * one * one / T(6),
                  (T(3) * u3 - T(6) * u2 + T(4)) / T(6),
                  (-T(3) * u3 + T(3) * u2 + T(3) * u + T(1)) / T(6),
                  u3 / T(6)};
  const T bdd[4] = {one / T(h * h), (T(3) * u - T(2)) / T(h * h),
                    (T(1) - T(3) * u) / T(h * h), u / T(h * h)};
  const T beta[3] = {T(1) - b[0], b[2] + b[3], b[3]};
  const T betad[3] = {one * one / T(2 * h),
                      (-u2 + u + T(0.5)) / T(h), u2 / T(2 * h)};
  State<T> state;
  state.position.setZero();
  state.acceleration.setZero();
  state.omega.setZero();
  state.rotation = quaternion(blocks[offset]);
  for (int j = 0; j < 4; ++j) {
    const Eigen::Map<const Vector> p(blocks[count + offset + j]);
    state.position += b[j] * p;
    state.acceleration += bdd[j] * p;
    if (j > 0) {
      const Vector delta = logarithm(quaternion(blocks[offset + j - 1]).conjugate() *
                                     quaternion(blocks[offset + j]));
      const Vector scaled = beta[j - 1] * delta;
      const auto increment = exponential(scaled);
      state.omega = increment.conjugate() * state.omega + betad[j - 1] * delta;
      state.rotation = state.rotation * increment;
    }
  }
  return state;
}

template <class T>
bool project(const Eigen::Matrix<T, 3, 1>& point, const CameraConfig& camera,
             Eigen::Matrix<T, 2, 1>* pixel) {
  // Behind-camera candidates must not silently produce a valid reprojection.
  if (scalar(point.z()) <= 1e-5) return false;
  const T x = point.x() / point.z(), y = point.y() / point.z();
  const T r2 = x * x + y * y;
  T xd, yd;
  if (camera.distortion_model == "radtan") {
    const T radial = T(1) + T(camera.distortion[0]) * r2 +
                     T(camera.distortion[1]) * r2 * r2;
    xd = x * radial + T(2 * camera.distortion[2]) * x * y +
         T(camera.distortion[3]) * (r2 + T(2) * x * x);
    yd = y * radial + T(camera.distortion[2]) * (r2 + T(2) * y * y) +
         T(2 * camera.distortion[3]) * x * y;
  } else {
    T scale = T(1);
    if (scalar(r2) > 1e-12) {
      using std::atan;
      using std::sqrt;
      const T r = sqrt(r2), theta = atan(r), theta2 = theta * theta;
      scale = theta * (T(1) + theta2 * (T(camera.distortion[0]) + theta2 *
              (T(camera.distortion[1]) + theta2 * (T(camera.distortion[2]) +
               theta2 * T(camera.distortion[3]))))) / r;
    } else {
      // The series also preserves the correct derivative on the optical axis.
      scale = T(1) + (T(camera.distortion[0]) - T(1.0 / 3.0)) * r2;
    }
    xd = scale * x;
    yd = scale * y;
  }
  (*pixel)[0] = T(camera.intrinsics[0]) * xd + T(camera.intrinsics[2]);
  (*pixel)[1] = T(camera.intrinsics[1]) * yd + T(camera.intrinsics[3]);
  return true;
}

struct CameraFactor {
  const Observation* observation;
  const CameraConfig* camera;
  int count, first;
  double time, start, spacing, sigma;

  template <class T> bool operator()(const T* const* parameters, T* residual) const {
    const T coordinate = (T(time - start) + parameters[2 * count + 2][0]) / T(spacing);
    const int segment = static_cast<int>(std::floor(scalar(coordinate)));
    const int offset = segment - first;
    if (offset < 0 || offset + 3 >= count) return false;
    const auto state = evaluate(parameters, count, offset, coordinate - T(segment), spacing);
    const auto R_ci = quaternion(parameters[2 * count]);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_ci(parameters[2 * count + 1]);
    for (std::size_t i = 0; i < observation->corners.size(); ++i) {
      const auto& corner = observation->corners[i];
      const Eigen::Matrix<T, 3, 1> point =
          R_ci * (state.rotation.conjugate() *
                  (corner.point.template cast<T>() - state.position)) + p_ci;
      Eigen::Matrix<T, 2, 1> pixel;
      if (!project(point, *camera, &pixel)) return false;
      residual[2 * i] = (pixel.x() - T(corner.pixel.x())) / T(sigma);
      residual[2 * i + 1] = (pixel.y() - T(corner.pixel.y())) / T(sigma);
    }
    return true;
  }
};

struct ImuFactor {
  ImuSample sample;
  double u, spacing, gyro_weight, accel_weight;
  template <class T> bool operator()(const T* const* parameters, T* residual) const {
    const auto state = evaluate(parameters, 4, 0, T(u), spacing);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gravity(parameters[8]);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gyro_bias(parameters[9]);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> accel_bias(parameters[10]);
    const Eigen::Matrix<T, 3, 1> gyro = state.omega + gyro_bias - sample.gyro.cast<T>();
    const Eigen::Matrix<T, 3, 1> accel = state.rotation.conjugate() *
        (state.acceleration - gravity) + accel_bias - sample.accel.cast<T>();
    for (int axis = 0; axis < 3; ++axis) {
      residual[axis] = T(gyro_weight) * gyro[axis];
      residual[3 + axis] = T(accel_weight) * accel[axis];
    }
    return true;
  }
};

struct Pose { double time; Quat rotation; Vec3 position; const Observation* observation; };

std::vector<Pose> cameraPoses(const std::vector<Observation>& observations,
                              const CameraConfig& camera, std::int64_t origin,
                              double lower, double upper) {
  const cv::Matx33d K(camera.intrinsics[0], 0, camera.intrinsics[2],
                      0, camera.intrinsics[1], camera.intrinsics[3], 0, 0, 1);
  const cv::Vec4d D(camera.distortion[0], camera.distortion[1],
                    camera.distortion[2], camera.distortion[3]);
  std::vector<Pose> poses;
  for (const auto& observation : observations) {
    const double t = seconds(observation.timestamp_ns, origin);
    if (observation.corners.size() < 6 || t < lower || t > upper) continue;
    std::vector<cv::Point3d> points;
    std::vector<cv::Point2d> pixels, normalized;
    Vec3 mean = Vec3::Zero();
    for (const auto& corner : observation.corners) {
      points.emplace_back(corner.point.x(), corner.point.y(), corner.point.z());
      pixels.emplace_back(corner.pixel.x(), corner.pixel.y());
      mean += corner.point;
    }
    mean /= static_cast<double>(points.size());
    Eigen::Matrix3d spread = Eigen::Matrix3d::Zero();
    for (const auto& corner : observation.corners) {
      const Vec3 delta = corner.point - mean;
      spread.noalias() += delta * delta.transpose();
    }
    if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(spread).eigenvalues()[1] < 1e-9)
      continue;
    if (camera.distortion_model == "equidistant")
      cv::fisheye::undistortPoints(pixels, normalized, K, D);
    else
      cv::undistortPoints(pixels, normalized, K, D);
    cv::Vec3d rotation_vector, translation;
    if (!cv::solvePnP(points, normalized, cv::Matx33d::eye(), cv::noArray(),
                     rotation_vector, translation, false, cv::SOLVEPNP_ITERATIVE)) continue;
    cv::Matx33d rotation;
    cv::Rodrigues(rotation_vector, rotation);
    Eigen::Matrix3d R_cw;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R_cw(r, c) = rotation(r, c);
    const Vec3 p_cw(translation[0], translation[1], translation[2]);
    if (!R_cw.allFinite() || !p_cw.allFinite()) continue;
    bool positive = true;
    for (const auto& corner : observation.corners)
      positive = positive && (R_cw * corner.point + p_cw).z() > 1e-5;
    if (!positive) continue;
    poses.push_back({t, Quat(R_cw.transpose()).normalized(), -R_cw.transpose() * p_cw,
                     &observation});
  }
  return poses;
}

Pose interpolatePose(const std::vector<Pose>& poses, double t) {
  auto next = std::upper_bound(poses.begin(), poses.end(), t,
                             [](double value, const Pose& pose) { return value < pose.time; });
  next = std::max(poses.begin() + 1, std::min(next, poses.end() - 1));
  const auto& a = *(next - 1);
  const auto& b = *next;
  const double alpha = (t - a.time) / (b.time - a.time);
  return {t, a.rotation.slerp(alpha, b.rotation).normalized(),
          (1 - alpha) * a.position + alpha * b.position, nullptr};
}

Vec3 interpolateGyro(const std::vector<ImuSample>& imu, std::int64_t origin, double t) {
  auto next = std::upper_bound(imu.begin(), imu.end(), t,
      [origin](double value, const ImuSample& sample) {
        return value < seconds(sample.timestamp_ns, origin);
      });
  next = std::max(imu.begin() + 1, std::min(next, imu.end() - 1));
  const double a = seconds((next - 1)->timestamp_ns, origin);
  const double b = seconds(next->timestamp_ns, origin);
  const double alpha = (t - a) / (b - a);
  return (1 - alpha) * (next - 1)->gyro + alpha * next->gyro;
}

struct RotationSeed { Quat rotation; Vec3 bias; double shift; };
RotationSeed initializeRotation(const std::vector<Pose>& poses,
                                const std::vector<ImuSample>& imu,
                                std::int64_t origin, double bound) {
  std::vector<Vec3> visual;
  std::vector<double> times;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double dt = poses[i].time - poses[i - 1].time;
    visual.push_back(logarithm(poses[i - 1].rotation.conjugate() * poses[i].rotation) / dt);
    times.push_back(0.5 * (poses[i].time + poses[i - 1].time));
  }
  Vec3 vmean = Vec3::Zero();
  for (const auto& v : visual) vmean += v;
  vmean /= static_cast<double>(visual.size());
  double best = std::numeric_limits<double>::infinity();
  RotationSeed result{Quat::Identity(), Vec3::Zero(), 0};
  const int steps = bound > 0 ? 100 : 0;
  for (int k = 0; k <= steps; ++k) {
    const double shift = steps ? bound * (2.0 * k / steps - 1) : 0;
    std::vector<Vec3> measured;
    Vec3 mmean = Vec3::Zero();
    for (double t : times) {
      measured.push_back(interpolateGyro(imu, origin, t + shift));
      mmean += measured.back();
    }
    mmean /= static_cast<double>(measured.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < visual.size(); ++i)
      covariance.noalias() += (visual[i] - vmean) * (measured[i] - mmean).transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    const Eigen::Matrix3d R_ci = svd.matrixU() * correction * svd.matrixV().transpose();
    const Vec3 bias = mmean - R_ci.transpose() * vmean;
    double error = 0;
    for (std::size_t i = 0; i < visual.size(); ++i)
      error += (visual[i] - R_ci * (measured[i] - bias)).squaredNorm();
    if (error < best) {
      best = error;
      result = {Quat(R_ci).normalized(), bias, shift};
    }
  }
  return result;
}

void validate(const std::vector<Observation>& observations, const std::vector<ImuSample>& imu,
              const CameraConfig& camera, const ImuConfig& config, const SolverOptions& options) {
  auto positive = [](double x) { return std::isfinite(x) && x > 0; };
  if (camera.model != "pinhole" ||
      (camera.distortion_model != "radtan" && camera.distortion_model != "equidistant"))
    throw std::invalid_argument("Solver supports only pinhole-radtan or pinhole-equidistant cameras");
  if (!positive(camera.intrinsics[0]) || !positive(camera.intrinsics[1]) ||
      camera.width <= 0 || camera.height <= 0)
    throw std::invalid_argument("Invalid focal length or image dimensions");
  for (double value : camera.intrinsics)
    if (!std::isfinite(value)) throw std::invalid_argument("Non-finite camera intrinsics");
  for (double value : camera.distortion)
    if (!std::isfinite(value)) throw std::invalid_argument("Non-finite camera distortion");
  if (!positive(config.update_rate) || !positive(config.gyroscope_noise_density) ||
      !positive(config.accelerometer_noise_density))
    throw std::invalid_argument("Positive finite IMU rate and noise densities are required");
  if (!positive(options.knot_spacing) || !positive(options.pixel_sigma) ||
      !std::isfinite(options.max_time_offset) || options.max_time_offset < 0 ||
      options.threads < 1 || options.max_iterations < 1)
    throw std::invalid_argument("Invalid solver options");
  if (observations.size() < 12 || imu.size() < 100)
    throw std::invalid_argument("At least 12 target observations and 100 IMU samples are required");
  for (std::size_t i = 0; i < observations.size(); ++i) {
    if (i && observations[i].timestamp_ns <= observations[i - 1].timestamp_ns)
      throw std::invalid_argument("Camera timestamps must be strictly increasing");
    for (const auto& corner : observations[i].corners)
      if (!corner.pixel.allFinite() || !corner.point.allFinite())
        throw std::invalid_argument("Non-finite corner coordinates");
  }
  for (std::size_t i = 0; i < imu.size(); ++i) {
    if (i && imu[i].timestamp_ns <= imu[i - 1].timestamp_ns)
      throw std::invalid_argument("IMU timestamps must be strictly increasing");
    if (!imu[i].gyro.allFinite() || !imu[i].accel.allFinite())
      throw std::invalid_argument("Non-finite IMU measurement");
  }
}
}  // namespace

CalibrationResult calibrate(const std::vector<Observation>& observations,
                            const std::vector<ImuSample>& imu,
                            const CameraConfig& camera, const ImuConfig& imu_config,
                            const SolverOptions& options) {
  validate(observations, imu, camera, imu_config, options);
  const std::int64_t origin = observations.front().timestamp_ns;
  const double h = options.knot_spacing, bound = options.max_time_offset;
  const double imu_start = seconds(imu.front().timestamp_ns, origin);
  const double imu_end = seconds(imu.back().timestamp_ns, origin);
  auto poses = cameraPoses(observations, camera, origin, imu_start + bound, imu_end - bound);
  if (poses.size() < 12 || poses.back().time - poses.front().time < std::max(1.0, 10 * h))
    throw std::invalid_argument("Insufficient valid target poses within the IMU overlap and time-shift margin");
  // Reject unobservable stationary/single-axis sequences before optimization.
  Vec3 gyro_mean = Vec3::Zero();
  std::size_t imu_count = 0;
  for (const auto& sample : imu) {
    const double t = seconds(sample.timestamp_ns, origin);
    if (t >= poses.front().time - bound && t <= poses.back().time + bound) {
      gyro_mean += sample.gyro;
      ++imu_count;
    }
  }
  if (imu_count < 100) throw std::invalid_argument("Too few overlapping IMU samples");
  gyro_mean /= static_cast<double>(imu_count);
  Eigen::Matrix3d gyro_covariance = Eigen::Matrix3d::Zero();
  for (const auto& sample : imu) {
    const double t = seconds(sample.timestamp_ns, origin);
    if (t >= poses.front().time - bound && t <= poses.back().time + bound) {
      const Vec3 delta = sample.gyro - gyro_mean;
      gyro_covariance.noalias() += delta * delta.transpose() / static_cast<double>(imu_count);
    }
  }
  if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(gyro_covariance).eigenvalues()[1] < 1e-4)
    throw std::invalid_argument("Insufficient rotational excitation: rotate about at least two independent axes");

  const auto seed = initializeRotation(poses, imu, origin, bound);
  Knot q_ci = coefficients(seed.rotation);
  Vec3 p_ci = Vec3::Zero(), gyro_bias = seed.bias, accel_bias = Vec3::Zero();
  Vec3 gravity(0, 0, -kGravity);
  double shift = seed.shift;
  const double start = poses.front().time - bound - 2 * h;
  const double end = poses.back().time + bound + 2 * h;
  const double required_knots = std::ceil((end - start) / h) + 4;
  if (!std::isfinite(required_knots) || required_knots > 1000000)
    throw std::invalid_argument("Requested spline has too many knots; increase knot spacing or shorten the sequence");
  const int knot_count = static_cast<int>(required_knots);
  std::vector<Knot> rotations(knot_count);
  std::vector<Vec3> positions(knot_count);
  for (int k = 0; k < knot_count; ++k) {
    const auto pose = interpolatePose(poses, start + (k - 1) * h - shift);
    rotations[k] = coefficients((pose.rotation * seed.rotation).normalized());
    positions[k] = pose.position;
  }
  // Estimate gravity and the accelerometer bias jointly from the visual seed.
  Eigen::MatrixXd A(3 * imu_count, 6);
  Eigen::VectorXd y(3 * imu_count);
  int row = 0;
  for (const auto& sample : imu) {
    const double t = seconds(sample.timestamp_ns, origin);
    if (t < poses.front().time - bound || t > poses.back().time + bound) continue;
    const double coordinate = (t - start) / h;
    const int segment = static_cast<int>(std::floor(coordinate));
    std::array<const double*, 8> blocks;
    for (int j = 0; j < 4; ++j) {
      blocks[j] = rotations[segment + j].data();
      blocks[4 + j] = positions[segment + j].data();
    }
    const auto state = evaluate(blocks.data(), 4, 0, coordinate - segment, h);
    const Eigen::Matrix3d R_iw = state.rotation.conjugate().toRotationMatrix();
    A.block<3, 3>(row, 0) = -R_iw;
    A.block<3, 3>(row, 3) = Eigen::Matrix3d::Identity();
    y.segment<3>(row) = sample.accel - R_iw * state.acceleration;
    row += 3;
  }
  const Eigen::VectorXd seed_accel = A.colPivHouseholderQr().solve(y);
  if (seed_accel.allFinite() && seed_accel.head<3>().norm() > 1) {
    gravity = kGravity * seed_accel.head<3>().normalized();
    const Eigen::VectorXd remaining = y - A.leftCols(3) * gravity;
    accel_bias.setZero();
    for (int r = 0; r < row; r += 3) accel_bias += remaining.segment<3>(r);
    accel_bias /= static_cast<double>(imu_count);
  }

  ceres::Problem problem;
  for (int k = 0; k < knot_count; ++k) {
    problem.AddParameterBlock(rotations[k].data(), 4, new ceres::QuaternionManifold);
    problem.AddParameterBlock(positions[k].data(), 3);
  }
  problem.AddParameterBlock(q_ci.data(), 4, new ceres::QuaternionManifold);
  problem.AddParameterBlock(p_ci.data(), 3);
  problem.AddParameterBlock(gravity.data(), 3, new ceres::SphereManifold<3>);
  problem.AddParameterBlock(gyro_bias.data(), 3);
  problem.AddParameterBlock(accel_bias.data(), 3);
  problem.AddParameterBlock(&shift, 1);
  if (bound > 0) {
    problem.SetParameterLowerBound(&shift, 0, -bound);
    problem.SetParameterUpperBound(&shift, 0, bound);
  } else {
    problem.SetParameterBlockConstant(&shift);
  }
  std::vector<std::pair<CameraFactor, std::vector<double*>>> camera_factors;
  std::vector<bool> used(knot_count, false);
  for (const auto& pose : poses) {
    // A fixed *superset* of support blocks is necessary: an optimized offset
    // may cross knot boundaries. Reserving the neighboring segment also handles
    // floating-point values exactly on a bound without an invalid block access.
    const int first = static_cast<int>(std::floor((pose.time - bound - start) / h)) - 1;
    const int last = static_cast<int>(std::floor((pose.time + bound - start) / h)) + 4;
    const int count = last - first + 1;
    CameraFactor factor{pose.observation, &camera, count, first, pose.time, start, h,
                        options.pixel_sigma};
    auto* cost = new ceres::DynamicAutoDiffCostFunction<CameraFactor, 4>(new CameraFactor(factor));
    std::vector<double*> blocks;
    for (int k = first; k <= last; ++k) {
      cost->AddParameterBlock(4);
      blocks.push_back(rotations[k].data());
      used[k] = true;
    }
    for (int k = first; k <= last; ++k) {
      cost->AddParameterBlock(3);
      blocks.push_back(positions[k].data());
    }
    cost->AddParameterBlock(4);
    cost->AddParameterBlock(3);
    cost->AddParameterBlock(1);
    blocks.push_back(q_ci.data());
    blocks.push_back(p_ci.data());
    blocks.push_back(&shift);
    cost->SetNumResiduals(static_cast<int>(2 * pose.observation->corners.size()));
    // Frame-level robust loss; the threshold scales with the number of pixels.
    auto* loss = new ceres::HuberLoss(3 * std::sqrt(2.0 * pose.observation->corners.size()));
    problem.AddResidualBlock(cost, loss, blocks);
    camera_factors.emplace_back(factor, std::move(blocks));
  }
  const double sqrt_dt = std::sqrt(1.0 / imu_config.update_rate);
  const double gyro_weight = sqrt_dt / imu_config.gyroscope_noise_density;
  const double accel_weight = sqrt_dt / imu_config.accelerometer_noise_density;
  for (const auto& sample : imu) {
    const double t = seconds(sample.timestamp_ns, origin);
    if (t < poses.front().time - bound || t > poses.back().time + bound) continue;
    const double coordinate = (t - start) / h;
    const int segment = static_cast<int>(std::floor(coordinate));
    auto* cost = new ceres::DynamicAutoDiffCostFunction<ImuFactor, 4>(
        new ImuFactor{sample, coordinate - segment, h, gyro_weight, accel_weight});
    std::vector<double*> blocks;
    for (int k = segment; k < segment + 4; ++k) {
      cost->AddParameterBlock(4);
      blocks.push_back(rotations[k].data());
      used[k] = true;
    }
    for (int k = segment; k < segment + 4; ++k) {
      cost->AddParameterBlock(3);
      blocks.push_back(positions[k].data());
    }
    for (double* parameter : {gravity.data(), gyro_bias.data(), accel_bias.data()}) {
      cost->AddParameterBlock(3);
      blocks.push_back(parameter);
    }
    cost->SetNumResiduals(6);
    problem.AddResidualBlock(cost, nullptr, blocks);
  }
  for (int k = 0; k < knot_count; ++k) {
    if (!used[k]) {
      problem.SetParameterBlockConstant(rotations[k].data());
      problem.SetParameterBlockConstant(positions[k].data());
    }
  }
  ceres::Solver::Options solve_options;
  solve_options.num_threads = options.threads;
  solve_options.max_num_iterations = options.max_iterations;
  solve_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solve_options.function_tolerance = 1e-8;
  solve_options.parameter_tolerance = 1e-9;
  solve_options.gradient_tolerance = 1e-10;
  solve_options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(solve_options, &problem, &summary);

  CalibrationResult result;
  result.T_cam_imu.topLeftCorner<3, 3>() = quaternion(q_ci.data()).toRotationMatrix();
  result.T_cam_imu.topRightCorner<3, 1>() = p_ci;
  result.timeshift_cam_imu = shift;
  result.gyro_bias = gyro_bias;
  result.accel_bias = accel_bias;
  result.gravity = gravity;
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  result.iterations = static_cast<int>(summary.iterations.size());
  double squared_pixels = 0;
  std::size_t corner_count = 0;
  bool valid_projection = true;
  int frame_index = 0;
  for (const auto& entry : camera_factors) {
    const auto& factor = entry.first;
    std::vector<double> residual(2 * factor.observation->corners.size());
    std::vector<const double*> blocks(entry.second.begin(), entry.second.end());
    if (!factor(blocks.data(), residual.data())) { valid_projection = false; break; }
    for (std::size_t i = 0; i < factor.observation->corners.size(); ++i) {
      const double rx = residual[2 * i] * options.pixel_sigma;
      const double ry = residual[2 * i + 1] * options.pixel_sigma;
      squared_pixels += rx * rx + ry * ry;
      const auto& corner = factor.observation->corners[i];
      result.reprojection_residuals.push_back({factor.observation->timestamp_ns, frame_index,
          corner.tag_id, corner.corner_id, corner.pixel, Eigen::Vector2d(rx, ry)});
    }
    corner_count += factor.observation->corners.size();
    ++frame_index;
  }
  result.reprojection_rmse = valid_projection && corner_count ?
      std::sqrt(squared_pixels / static_cast<double>(corner_count)) :
      std::numeric_limits<double>::infinity();
  const bool boundary = bound > 0 && std::abs(shift) >= bound * 0.995;
  result.converged = summary.termination_type == ceres::CONVERGENCE &&
      summary.IsSolutionUsable() && result.T_cam_imu.allFinite() && gravity.allFinite() &&
      gyro_bias.allFinite() && accel_bias.allFinite() &&
      result.reprojection_rmse < 5 * options.pixel_sigma && !boundary;
  std::ostringstream report;
  report << summary.BriefReport() << "\nCubic cumulative SO(3) + R3 spline; "
         << knot_count << " knots, " << poses.size() << " camera frames, "
         << imu_count << " IMU samples. Fixed intrinsics; constant IMU biases; "
         << "noise whitened using the configured IMU sampling period.\n"
         << "Convergence is a numerical fit check, not an observability or accuracy certificate.";
  if (boundary) report << "\nTime offset reached its search bound; enlarge max_time_offset and retry.";
  if (result.reprojection_rmse >= 5 * options.pixel_sigma)
    report << "\nReprojection error exceeds the acceptance threshold.";
  result.report = report.str();
  return result;
}
}  // namespace kalibr2
