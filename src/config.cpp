#include "kalibr2/config.hpp"
#include "kalibr2/report.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace kalibr2 {
namespace {
double positive(const YAML::Node& n, const char* key) {
  const double v = n[key].as<double>();
  if (!std::isfinite(v) || v <= 0) throw std::runtime_error(std::string(key) + " must be finite and positive");
  return v;
}
template<std::size_t N>
std::array<double, N> array(const YAML::Node& n, const char* key) {
  const auto a = n[key];
  if (!a.IsSequence() || a.size() != N) throw std::runtime_error(std::string(key) + " has an invalid size");
  std::array<double, N> result{};
  for (std::size_t i=0; i<N; ++i) {
    result[i] = a[i].as<double>();
    if (!std::isfinite(result[i])) throw std::runtime_error(std::string(key) + " must be finite");
  }
  return result;
}
}
CameraConfig load_camera(const std::string& path) {
  const auto root = YAML::LoadFile(path);
  if (root["cam1"]) std::clog << "Single-camera prototype: selecting cam0 from the camera chain.\n";
  const auto n = root["cam0"];
  if (!n) throw std::runtime_error("Camera YAML must contain cam0");
  CameraConfig c;
  c.topic = n["rostopic"].as<std::string>();
  c.model = n["camera_model"].as<std::string>();
  c.distortion_model = n["distortion_model"].as<std::string>();
  if (c.model != "pinhole" || (c.distortion_model != "radtan" && c.distortion_model != "equidistant"))
    throw std::runtime_error("Supported camera models: pinhole-radtan, pinhole-equidistant");
  c.intrinsics = array<4>(n, "intrinsics");
  c.distortion = array<4>(n, "distortion_coeffs");
  const auto resolution = n["resolution"];
  if (!resolution.IsSequence() || resolution.size()!=2) throw std::runtime_error("resolution must contain width,height");
  c.width = resolution[0].as<int>(); c.height = resolution[1].as<int>();
  if (c.width<=0 || c.height<=0 || c.width>32768 || c.height>32768 || c.intrinsics[0]<=0 || c.intrinsics[1]<=0 || c.topic.empty())
    throw std::runtime_error("Invalid camera resolution, focal length or topic");
  return c;
}
ImuConfig load_imu(const std::string& path) {
  const auto n = YAML::LoadFile(path);
  ImuConfig c;
  c.topic = n["rostopic"].as<std::string>();
  if(c.topic.empty()) throw std::runtime_error("IMU topic must not be empty");
  c.update_rate = positive(n,"update_rate");
  c.accelerometer_noise_density = positive(n,"accelerometer_noise_density");
  c.accelerometer_random_walk = positive(n,"accelerometer_random_walk");
  c.gyroscope_noise_density = positive(n,"gyroscope_noise_density");
  c.gyroscope_random_walk = positive(n,"gyroscope_random_walk");
  return c;
}
GridConfig load_grid(const std::string& path) {
  const auto n = YAML::LoadFile(path);
  if(n["target_type"].as<std::string>() != "aprilgrid") throw std::runtime_error("Only AprilGrid targets are supported");
  GridConfig c;
  c.rows=n["tagRows"].as<int>(); c.cols=n["tagCols"].as<int>();
  c.tag_size=positive(n,"tagSize"); c.tag_spacing=positive(n,"tagSpacing");
  if(c.rows<=0 || c.cols<=0 || c.rows>587 || c.cols>587 || c.rows*c.cols>587)
    throw std::runtime_error("AprilGrid must use 1..587 tag36h11 IDs");
  return c;
}
void save_result(const std::string& directory, const CameraConfig& c, const CalibrationResult& r) {
  if(!r.converged) throw std::runtime_error("Refusing to export an unconverged calibration");
  YAML::Node n, cam;
  cam["camera_model"]=c.model; cam["distortion_model"]=c.distortion_model;
  cam["rostopic"]=c.topic;
  for(auto v:c.intrinsics) cam["intrinsics"].push_back(v);
  for(auto v:c.distortion) cam["distortion_coeffs"].push_back(v);
  cam["resolution"].push_back(c.width); cam["resolution"].push_back(c.height);
  for(int i=0;i<4;++i) {
    YAML::Node row; for(int j=0;j<4;++j) row.push_back(r.T_cam_imu(i,j));
    cam["T_cam_imu"].push_back(row);
  }
  cam["timeshift_cam_imu"]=r.timeshift_cam_imu;
  n["cam0"]=cam;
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << n;
  std::ofstream out(std::filesystem::path(directory)/"camchain-imucam.yaml");
  out << emitter.c_str() << '\n';
  if(!out) throw std::runtime_error("Unable to write calibration YAML");
  std::ofstream report(std::filesystem::path(directory)/"solver.txt");
  report << r.report << "\nReprojection RMSE [px]: " << r.reprojection_rmse
         << "\nGyro bias: " << r.gyro_bias.transpose()
         << "\nAccelerometer bias: " << r.accel_bias.transpose()
         << "\nGravity: " << r.gravity.transpose() << '\n';
  if(!report) throw std::runtime_error("Unable to write solver report");
  save_diagnostics_report(directory, c, r);
}
} // namespace kalibr2
