#include "kalibr2/solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
constexpr std::int64_t kEpoch = 1700000000000000000LL;
constexpr double kOffset = 0.0373;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);  // Active in Release builds.
}

struct Motion { Mat3 rotation; Vec3 position, omega, acceleration; };

// Independent analytic motion, deliberately NOT generated using the fitted
// spline. Multiple frequencies/axes excite time, extrinsics, gravity and biases.
Motion motion(double t) {
  const double x = 0.24 * std::sin(1.3 * t) + 0.07 * std::sin(2.9 * t);
  const double y = 0.27 * std::cos(1.7 * t) + 0.05 * std::sin(2.6 * t);
  const double z = 0.21 * std::sin(1.9 * t) + 0.04 * std::cos(3.1 * t);
  const double xd = 0.24 * 1.3 * std::cos(1.3 * t) + 0.07 * 2.9 * std::cos(2.9 * t);
  const double yd = -0.27 * 1.7 * std::sin(1.7 * t) + 0.05 * 2.6 * std::cos(2.6 * t);
  const double zd = 0.21 * 1.9 * std::cos(1.9 * t) - 0.04 * 3.1 * std::sin(3.1 * t);
  const Mat3 Rx = Eigen::AngleAxisd(x, Vec3::UnitX()).toRotationMatrix();
  const Mat3 Ry = Eigen::AngleAxisd(y, Vec3::UnitY()).toRotationMatrix();
  const Mat3 Rz = Eigen::AngleAxisd(z, Vec3::UnitZ()).toRotationMatrix();
  Motion state;
  state.rotation = Rx * Ry * Rz;
  state.omega = Rz.transpose() * Ry.transpose() * (xd * Vec3::UnitX()) +
                Rz.transpose() * (yd * Vec3::UnitY()) + zd * Vec3::UnitZ();
  state.position = Vec3(0.35 + 0.17 * std::sin(1.1 * t) + 0.03 * std::sin(3.2 * t),
                       0.25 + 0.13 * std::cos(1.4 * t) + 0.025 * std::sin(2.5 * t),
                      -1.8 + 0.11 * std::sin(1.7 * t) + 0.04 * std::cos(2.2 * t));
  state.acceleration = Vec3(-0.17 * 1.1 * 1.1 * std::sin(1.1 * t) - 0.03 * 3.2 * 3.2 * std::sin(3.2 * t),
                           -0.13 * 1.4 * 1.4 * std::cos(1.4 * t) - 0.025 * 2.5 * 2.5 * std::sin(2.5 * t),
                           -0.11 * 1.7 * 1.7 * std::sin(1.7 * t) - 0.04 * 2.2 * 2.2 * std::cos(2.2 * t));
  return state;
}

struct Dataset {
  kalibr2::CameraConfig camera;
  kalibr2::ImuConfig imu_config;
  std::vector<kalibr2::Observation> observations;
  std::vector<kalibr2::ImuSample> imu;
  Mat3 R_ci = Eigen::AngleAxisd(0.23, Vec3(0.3, -0.6, 0.5).normalized()).toRotationMatrix();
  Vec3 p_ci{0.043, -0.018, 0.027};
  Vec3 gyro_bias{0.009, -0.007, 0.004};
  Vec3 accel_bias{0.08, -0.05, 0.04};
  Vec3 gravity = 9.80665 * Vec3(0.3, -0.4, -9.79).normalized();
};

Dataset generate(const std::string& distortion) {
  Dataset data;
  data.camera.model = "pinhole";
  data.camera.distortion_model = distortion;
  data.camera.width = 1280;
  data.camera.height = 960;
  data.camera.intrinsics = {710, 705, 640, 480};
  data.camera.distortion = distortion == "radtan" ?
      std::array<double, 4>{-0.035, 0.008, 0.0008, -0.0005} :
      std::array<double, 4>{0.02, -0.004, 0.0007, -0.0001};
  data.imu_config.update_rate = 200;
  data.imu_config.gyroscope_noise_density = 0.0002;
  data.imu_config.accelerometer_noise_density = 0.005;
  data.imu_config.gyroscope_random_walk = 0.00001;
  data.imu_config.accelerometer_random_walk = 0.0001;
  for (int sample = 0; sample <= 1600; ++sample) {
    const double t = sample / 200.0;
    const auto state = motion(t);
    // Tiny deterministic measurement noise avoids relying on exact arithmetic.
    const Vec3 ng(0.00005 * std::sin(23 * t), 0.00005 * std::cos(29 * t),
                  0.00005 * std::sin(31 * t));
    const Vec3 na(0.001 * std::sin(37 * t), 0.001 * std::cos(41 * t),
                  0.001 * std::sin(43 * t));
    data.imu.push_back({kEpoch + static_cast<std::int64_t>(std::llround(t * 1e9)),
                       state.omega + data.gyro_bias + ng,
                       state.rotation.transpose() * (state.acceleration - data.gravity) +
                       data.accel_bias + na});
  }
  const cv::Matx33d K(710, 0, 640, 0, 705, 480, 0, 0, 1);
  const cv::Vec4d D(data.camera.distortion[0], data.camera.distortion[1],
                    data.camera.distortion[2], data.camera.distortion[3]);
  for (int frame = 3; frame < 197; ++frame) {
    const double t_camera = frame / 25.0;
    const auto state = motion(t_camera + kOffset);
    kalibr2::Observation observation;
    observation.timestamp_ns = kEpoch + static_cast<std::int64_t>(std::llround(t_camera * 1e9));
    std::vector<cv::Point3d> camera_points;
    for (int row = 0; row < 5; ++row) {
      for (int col = 0; col < 6; ++col) {
        kalibr2::Corner corner;
        corner.tag_id = row * 6 + col;
        corner.corner_id = 0;
        corner.point = Vec3(0.12 * col, 0.12 * row, 0);
        const Vec3 point = data.R_ci * state.rotation.transpose() *
                           (corner.point - state.position) + data.p_ci;
        camera_points.emplace_back(point.x(), point.y(), point.z());
        observation.corners.push_back(corner);
      }
    }
    std::vector<cv::Point2d> pixels;
    if (distortion == "radtan")
      cv::projectPoints(camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D, pixels);
    else
      cv::fisheye::projectPoints(camera_points, pixels, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D);
    for (std::size_t corner = 0; corner < pixels.size(); ++corner) {
      observation.corners[corner].pixel = Eigen::Vector2d(
          pixels[corner].x + 0.005 * std::sin(frame * 2.7 + corner),
          pixels[corner].y + 0.005 * std::cos(frame * 1.9 + corner));
    }
    data.observations.push_back(std::move(observation));
  }
  return data;
}

void recovery(const Dataset& data) {
  kalibr2::SolverOptions options;
  options.threads = 2;
  options.max_iterations = 80;
  options.knot_spacing = 0.075;
  options.max_time_offset = 0.08;
  options.pixel_sigma = 0.25;
  const auto fit = kalibr2::calibrate(data.observations, data.imu, data.camera,
                                     data.imu_config, options);
  const double rotation_error = Eigen::AngleAxisd(
      fit.T_cam_imu.topLeftCorner<3, 3>() * data.R_ci.transpose()).angle();
  const double translation_error = (fit.T_cam_imu.topRightCorner<3, 1>() - data.p_ci).norm();
  const double time_error = std::abs(fit.timeshift_cam_imu - kOffset);
  std::cout << data.camera.distortion_model << ": rotation=" << rotation_error
            << " rad, translation=" << translation_error << " m, time=" << time_error
            << " s, reprojection=" << fit.reprojection_rmse << " px\n"
            << fit.report << '\n';
  require(fit.converged, "Synthetic calibration did not converge");
  require(rotation_error < 0.005, "Incorrect camera/IMU rotation or transform convention");
  require(translation_error < 0.006, "Incorrect camera/IMU translation");
  require(time_error < 0.0008, "Incorrect camera-to-IMU time offset or sign");
  require((fit.gyro_bias - data.gyro_bias).norm() < 0.002, "Gyroscope bias was not recovered");
  require((fit.accel_bias - data.accel_bias).norm() < 0.045, "Accelerometer bias was not recovered");
  require((fit.gravity - data.gravity).norm() < 0.045, "Gravity direction was not recovered");
  require(fit.reprojection_rmse < 0.12, "Unexpectedly high synthetic reprojection error");
  require(fit.final_cost < fit.initial_cost, "Optimization did not lower the cost");
}

void expectInvalid(const std::function<void()>& action, const std::string& message) {
  try { action(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error(message);
}

// Render actual PNG images for the CLI test. Image generation uses an independent
// plane homography and analytic motion, not the solver's projection or spline.
void writeFixture(const std::filesystem::path& root, int scale) {
  require(scale>=1 && scale<=3,"Fixture scale must be 1..3");
  cv::setNumThreads(1);
  const auto data=generate("radtan");
  std::filesystem::create_directories(root/"cam0");
  constexpr std::uint64_t codes[]{0xd5d628584ULL,0xd97f18b49ULL,0xdd280910eULL,
                                 0xe479e9c98ULL,0xebcbca822ULL,0xf31dab3acULL};
  cv::Mat board(520,720,CV_8UC1,cv::Scalar(255));
  for(int id=0;id<6;++id) {
    const int x=80+(id%3)*200,y=80+(1-id/3)*200;
    board(cv::Rect(x,y,160,160)).setTo(0);
    for(int r=0;r<6;++r) for(int c=0;c<6;++c)
      if((codes[id]>>(35-r*6-c))&1ULL) board(cv::Rect(x+(c+2)*16,y+(r+2)*16,16,16)).setTo(255);
    for(int dx:{-40,160}) for(int dy:{-40,160}) board(cv::Rect(x+dx,y+dy,40,40)).setTo(0);
  }
  const std::vector<cv::Point2f> source{{79.5F,439.5F},{639.5F,439.5F},{639.5F,79.5F},{79.5F,79.5F}};
  const std::vector<Vec3> points{{0,0,0},{0.56,0,0},{0.56,0.36,0},{0,0.36,0}};
  const cv::Matx33d K(710*scale,0,640*scale,0,705*scale,480*scale,0,0,1);
  for(const auto& o:data.observations) {
    const double t=static_cast<double>(o.timestamp_ns-kEpoch)*1e-9;
    auto state=motion(t+kOffset);
    // Face the PRINTED side of the board. Left-transforming the complete motion
    // and gravity leaves body-frame IMU measurements and camera/IMU extrinsics
    // unchanged; viewing the opposite side would mirror all tag payloads.
    const Mat3 world_rotation=Eigen::AngleAxisd(std::acos(-1.0),Vec3::UnitX()).toRotationMatrix();
    state.rotation=world_rotation*state.rotation;
    state.position=world_rotation*state.position+Vec3(0,0.5,0);
    std::vector<cv::Point2f> projected;
    for(const auto& p:points) {
      const Vec3 pc=data.R_ci*state.rotation.transpose()*(p-state.position)+data.p_ci;
      projected.emplace_back(static_cast<float>(K(0,0)*pc.x()/pc.z()+K(0,2)),
                             static_cast<float>(K(1,1)*pc.y()/pc.z()+K(1,2)));
    }
    cv::Mat frame;
    cv::warpPerspective(board,frame,cv::getPerspectiveTransform(source,projected),
                        cv::Size(1280*scale,960*scale),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255));
    require(cv::imwrite((root/"cam0"/(std::to_string(o.timestamp_ns)+".png")).string(),frame),"Unable to write fixture image");
  }
  std::ofstream csv(root/"imu0.csv"); csv<<std::setprecision(17);
  for(const auto& s:data.imu) csv<<s.timestamp_ns<<','<<s.gyro.x()<<','<<s.gyro.y()<<','<<s.gyro.z()
    <<','<<s.accel.x()<<','<<s.accel.y()<<','<<s.accel.z()<<'\n';
  YAML::Node cam,imu,grid;
  cam["cam0"]["camera_model"]="pinhole";cam["cam0"]["distortion_model"]="radtan";
  cam["cam0"]["intrinsics"]=std::vector<double>{710.*scale,705.*scale,640.*scale,480.*scale};
  cam["cam0"]["distortion_coeffs"]=std::vector<double>{0,0,0,0};
  cam["cam0"]["resolution"]=std::vector<int>{1280*scale,960*scale};cam["cam0"]["rostopic"]="/cam0/image_raw";
  imu["rostopic"]="/imu0";imu["update_rate"]=data.imu_config.update_rate;
  imu["gyroscope_noise_density"]=data.imu_config.gyroscope_noise_density;
  imu["accelerometer_noise_density"]=data.imu_config.accelerometer_noise_density;
  imu["gyroscope_random_walk"]=data.imu_config.gyroscope_random_walk;
  imu["accelerometer_random_walk"]=data.imu_config.accelerometer_random_walk;
  grid["target_type"]="aprilgrid";grid["tagRows"]=2;grid["tagCols"]=3;grid["tagSize"]=0.16;grid["tagSpacing"]=0.25;
  std::ofstream(root/"camera.yaml")<<cam;std::ofstream(root/"imu.yaml")<<imu;std::ofstream(root/"target.yaml")<<grid;
  require(static_cast<bool>(csv),"Unable to write IMU fixture");
}

void verifyExport(const std::string& path) {
  const auto n=YAML::LoadFile(path)["cam0"];
  const auto reference=generate("radtan");
  Mat3 R;Vec3 p;
  for(int i=0;i<3;++i) {
    for(int j=0;j<3;++j) R(i,j)=n["T_cam_imu"][i][j].as<double>();
    p[i]=n["T_cam_imu"][i][3].as<double>();
  }
  const double re=Eigen::AngleAxisd(R*reference.R_ci.transpose()).angle();
  const double pe=(p-reference.p_ci).norm();
  const double te=std::abs(n["timeshift_cam_imu"].as<double>()-kOffset);
  std::cout<<"PNG→CLI→YAML errors: rotation="<<re<<" rad, translation="<<pe<<" m, shift="<<te<<" s\n";
  require(re<0.005 && pe<0.015 && te<0.002,"End-to-end calibration differs from analytic ground truth");
}
}  // namespace

int main(int argc,char** argv) {
  try {
    if((argc==3 || argc==4) && std::string(argv[1])=="--write-fixture") {
      writeFixture(argv[2],argc==4 ? std::stoi(argv[3]) : 1);return 0;
    }
    if(argc==3 && std::string(argv[1])=="--verify-export") { verifyExport(argv[2]);return 0; }
    const auto radtan = generate("radtan");
    recovery(radtan);
    recovery(generate("equidistant"));
    auto stationary = radtan.imu;
    for (auto& sample : stationary) sample.gyro = Vec3(0.01, -0.02, 0.03);
    expectInvalid([&] {
      kalibr2::calibrate(radtan.observations, stationary, radtan.camera, radtan.imu_config);
    }, "Stationary IMU sequence should be rejected");
    auto camera = radtan.camera;
    camera.model = "omni";
    expectInvalid([&] {
      kalibr2::calibrate(radtan.observations, radtan.imu, camera, radtan.imu_config);
    }, "Unsupported camera model should be rejected");
    auto unordered = radtan.imu;
    unordered[1].timestamp_ns = unordered[0].timestamp_ns;
    expectInvalid([&] {
      kalibr2::calibrate(radtan.observations, unordered, radtan.camera, radtan.imu_config);
    }, "Duplicate IMU timestamp should be rejected");
    expectInvalid([&] {
      kalibr2::calibrate({}, radtan.imu, radtan.camera, radtan.imu_config);
    }, "Empty camera sequence should be rejected");
    std::cout << "Solver tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Solver test failed: " << error.what() << '\n';
    return 1;
  }
}
