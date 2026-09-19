#include "kalibr2/config.hpp"
#include "kalibr2/pipeline.hpp"
#include "kalibr2/solver.hpp"
#ifdef KALIBR2_WITH_ROS2
#include "kalibr2/rosbag_source.hpp"
#endif
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <thread>
#include <sys/resource.h>

namespace {
void help() {
  std::cout << R"(Kalibr2 0.1 — experimental camera–IMU calibration

  kalibr2 extract   --bag BAG2 | --dataset DIRECTORY  [options]
  kalibr2 calibrate --bag BAG2 | --dataset DIRECTORY | --cache FILE [options]

Required: --camera camchain.yaml --imu imu.yaml --target aprilgrid.yaml --output NEW_DIRECTORY
Options:
  --threads N              CPU workers (default: hardware concurrency, capped at 8)
  --image-memory-mib N     Optional pixel-buffer budget limiting images in flight
  --detector NAME          kalibr (default) or apriltag3
  --tag-border 1|2         2: original Kalibr boards; 1: standard AprilTag3 boards
  --decimate X             AprilTag3 quad-search decimation (default: 1; kalibr requires 1)
  --knot-spacing SEC       Cubic spline knot spacing (default: 0.05)
  --max-time-offset SEC    Absolute camera-to-IMU offset bound (default: 0.05)
  --max-iterations N       Ceres iteration limit (default: 100)

Inputs: one global-shutter camera (pinhole-radtan or pinhole-equidistant), one IMU,
fixed camera intrinsics, stationary AprilGrid. Directory: cam0/<nanoseconds>.png
or .jpg, imu0.csv: timestamp_ns,wx,wy,wz,ax,ay,az (SI units).
Convert ROS1 bags using rosbags-convert before --bag (see README).
Output: cache/observations.cache and cache/extraction.json; calibrate also writes
Kalibr-compatible camchain-imucam.yaml, solver.txt and a timestamped PDF report. Existing directories are refused.
Prototype differences from Kalibr: cubic Lie splines, constant IMU biases.
)";
}
int integer(const std::string& text) {
  int value{}; const auto [p,e]=std::from_chars(text.data(),text.data()+text.size(),value);
  if(e!=std::errc{} || p!=text.data()+text.size() || value<=0) throw std::runtime_error("Expected a positive integer: "+text);
  return value;
}
double number(const std::string& text) {
  std::size_t end{}; const double value=std::stod(text,&end);
  if(end!=text.size() || !std::isfinite(value) || value<=0) throw std::runtime_error("Expected a finite positive number: "+text);
  return value;
}
}
int main(int argc,char** argv) {
  try {
    if(argc<2 || std::string(argv[1])=="--help") { help(); return 0; }
    const std::string command=argv[1];
    if(command!="extract" && command!="calibrate") throw std::runtime_error("Unknown command: "+command);
    std::map<std::string,std::string> args;
    const std::set<std::string> valid={"--bag","--dataset","--cache","--camera","--imu","--target","--output",
      "--detector","--threads","--image-memory-mib","--tag-border","--decimate","--knot-spacing","--max-time-offset","--max-iterations"};
    for(int i=2;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") { help(); return 0; }
      if(!valid.contains(key) || i+1==argc || !args.emplace(key,argv[++i]).second)
        throw std::runtime_error("Unknown, duplicate or incomplete option: "+key);
    }
    for(const auto* key:{"--camera","--imu","--target","--output"})
      if(!args.contains(key)) throw std::runtime_error(std::string("Missing ")+key);
    if(args.contains("--bag")+args.contains("--dataset")+args.contains("--cache")!=1)
      throw std::runtime_error("Select exactly one input: --bag, --dataset or --cache");
    if(command=="extract" && args.contains("--cache")) throw std::runtime_error("extract requires image data");
    if(args.contains("--cache") && (args.contains("--detector") || args.contains("--tag-border") || args.contains("--decimate")))
      throw std::runtime_error("Detector settings cannot change cached detections; run extract again");
    const auto camera=kalibr2::load_camera(args.at("--camera"));
    const auto imu=kalibr2::load_imu(args.at("--imu"));
    const auto grid=kalibr2::load_grid(args.at("--target"));
    kalibr2::PipelineOptions pipeline;
    pipeline.threads=std::min(8U,std::max(1U,std::thread::hardware_concurrency()));
    if(args.contains("--threads")) pipeline.threads=integer(args.at("--threads"));
    if(args.contains("--image-memory-mib")) pipeline.image_memory_bytes=static_cast<std::size_t>(integer(args.at("--image-memory-mib")))*1048576;
    kalibr2::DetectorOptions detector;
    if(args.contains("--detector")) detector.backend=kalibr2::parse_detector_backend(args.at("--detector"));
    if(args.contains("--tag-border")) detector.tag_border=integer(args.at("--tag-border"));
    if(args.contains("--decimate")) detector.quad_decimate=number(args.at("--decimate"));
    if(detector.backend==kalibr2::DetectorBackend::Kalibr && detector.quad_decimate!=1.0)
      throw std::runtime_error("--detector kalibr requires --decimate 1");
    kalibr2::SolverOptions solver; solver.threads=pipeline.threads;
    if(args.contains("--knot-spacing")) solver.knot_spacing=number(args.at("--knot-spacing"));
    if(args.contains("--max-time-offset")) solver.max_time_offset=number(args.at("--max-time-offset"));
    if(args.contains("--max-iterations")) solver.max_iterations=integer(args.at("--max-iterations"));
    const std::filesystem::path output(args.at("--output"));
    if(std::filesystem::exists(output)) throw std::runtime_error("Output path already exists: "+output.string());
    // Avoid nested OpenCV thread pools competing with frame-level parallelism.
    cv::setNumThreads(1);
    kalibr2::ExtractedData data;
    if(args.contains("--cache")) data=kalibr2::read_cache(args.at("--cache"),camera,grid);
    else {
      std::unique_ptr<kalibr2::FrameSource> source;
      if(args.contains("--dataset")) source=kalibr2::directory_source(args.at("--dataset"),camera);
      else {
#ifdef KALIBR2_WITH_ROS2
        source=std::make_unique<kalibr2::RosbagSource>(args.at("--bag"),camera,imu);
#else
        throw std::runtime_error("Built without ROS2; rebuild with KALIBR2_WITH_ROS2=ON");
#endif
      }
      std::cerr << "Extracting AprilGrid using " << kalibr2::detector_name(detector.backend) << " with " << pipeline.threads << " workers";
      if(pipeline.image_memory_bytes>0) std::cerr << ", " << pipeline.image_memory_bytes/1048576 << " MiB pixel-buffer budget";
      else std::cerr << ", no pixel-buffer budget";
      std::cerr << "...\n";
      data=kalibr2::extract(*source,camera,grid,detector,pipeline);
    }
    if(!std::filesystem::create_directories(output)) throw std::runtime_error("Unable to create output directory");
    const auto cache_dir = output / "cache";
    if(!std::filesystem::create_directories(cache_dir)) throw std::runtime_error("Unable to create cache directory");
    kalibr2::write_cache((cache_dir/"observations.cache").string(),data,camera,grid);
    std::ofstream stats(cache_dir/"extraction.json");
    rusage usage{}; getrusage(RUSAGE_SELF,&usage);
    stats << "{\n  \"frames\": " << data.frames << ",\n  \"detected_frames\": " << data.detected_frames
          << ",\n  \"imu_samples\": " << data.imu.size() << ",\n  \"max_in_flight\": " << data.max_in_flight
          << ",\n  \"elapsed_seconds\": " << data.elapsed_seconds << ",\n  \"tag_border\": " << data.tag_border
          << ",\n  \"detector\": \"" << kalibr2::detector_name(data.detector_backend) << "\""
          << ",\n  \"decimate\": " << data.decimate
          << ",\n  \"peak_rss_kib_at_extraction\": " << usage.ru_maxrss
          << ",\n  \"from_cache\": " << (data.from_cache ? "true" : "false") << "\n}\n";
    if(!stats) throw std::runtime_error("Unable to write extraction statistics");
    for(const auto* key:{"--camera","--imu","--target"})
      std::filesystem::copy_file(args.at(key),cache_dir/(std::string(key+2)+"-input.yaml"));
    std::cerr << "Detected target in " << data.detected_frames << '/' << data.frames << " frames; "
              << data.imu.size() << " IMU samples; " << data.elapsed_seconds << " s.\n";
    if(data.observations.empty()) throw std::runtime_error("No valid AprilGrid: check target dimensions, tag-border, focus and visibility");
    if(command=="calibrate") {
      std::cerr << "Optimizing trajectory, extrinsics, time offset, gravity and constant biases...\n";
      const auto fit=kalibr2::calibrate(data.observations,data.imu,camera,imu,solver);
      if(!fit.converged) {
        std::ofstream failure(output/"solver-failed.txt"); failure << fit.report;
        throw std::runtime_error("Solver did not converge; observation cache retained, calibration not exported");
      }
      kalibr2::save_result(output.string(),camera,fit);
      std::cout << "Reprojection RMSE: " << fit.reprojection_rmse << " px; camera→IMU shift: "
                << fit.timeshift_cam_imu << " s\n";
    }
    std::cout << "Results: " << output << '\n';
    return 0;
  } catch(const std::exception& e) { std::cerr << "kalibr2: " << e.what() << '\n'; return 1; }
}
