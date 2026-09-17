#include "kalibr2/pipeline.hpp"
#include <oneapi/tbb/enumerable_thread_specific.h>
#include <oneapi/tbb/parallel_pipeline.h>
#include <oneapi/tbb/task_arena.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace kalibr2 {
namespace {
std::string signature(const CameraConfig& c, const GridConfig& g) {
  std::ostringstream out; out << std::setprecision(17);
  out << "KALIBR2_CACHE_V1 " << c.width << ' ' << c.height << ' ' << c.model << ' ' << c.distortion_model;
  for(double v:c.intrinsics) out << ' ' << v;
  for(double v:c.distortion) out << ' ' << v;
  out << ' ' << g.rows << ' ' << g.cols << ' ' << g.tag_size << ' ' << g.tag_spacing;
  return out.str();
}
}
ExtractedData extract(FrameSource& source, const CameraConfig& camera,
                      const GridConfig& grid, const DetectorOptions& detector,
                      const PipelineOptions& options) {
  if(options.threads<1 || camera.width<=0 || camera.height<=0)
    throw std::runtime_error("Invalid pipeline dimensions or thread count");
  // Conservative allowance for a serialized color frame, decoded color/gray
  // buffers and codec input. Admission takes place BEFORE the next frame read.
  const auto reservation=static_cast<std::size_t>(camera.width)*camera.height*16+2*1024*1024;
  if(options.image_memory_bytes<reservation)
    throw std::runtime_error("Image budget too small for one frame; need at least " +
                             std::to_string((reservation+1048575)/1048576) + " MiB");
  const auto tokens=std::min<std::size_t>(options.threads, options.image_memory_bytes/reservation);
  ExtractedData result;
  result.tag_border=detector.tag_border;
  result.decimate=detector.quad_decimate;
  result.max_in_flight=tokens;
  const auto begin=std::chrono::steady_clock::now();
  oneapi::tbb::task_arena arena(static_cast<int>(tokens));
  oneapi::tbb::enumerable_thread_specific<std::unique_ptr<AprilGridDetector>> workers(
      [&] { return std::make_unique<AprilGridDetector>(grid,detector); });
  std::int64_t last=-1;
  arena.execute([&] {
    oneapi::tbb::parallel_pipeline(tokens,
      oneapi::tbb::make_filter<void,std::shared_ptr<ImageFrame>>(
        oneapi::tbb::filter_mode::serial_in_order,[&](oneapi::tbb::flow_control& flow) {
          auto frame=std::make_shared<ImageFrame>();
          if(!source.next(*frame)) { flow.stop(); return std::shared_ptr<ImageFrame>{}; }
          if(frame->timestamp_ns<=last) throw std::runtime_error("Camera timestamps must be strictly increasing");
          last=frame->timestamp_ns;
          if(frame->gray.type()!=CV_8UC1 || frame->gray.cols!=camera.width || frame->gray.rows!=camera.height)
            throw std::runtime_error("Source returned an invalid grayscale image");
          return frame;
        }) &
      oneapi::tbb::make_filter<std::shared_ptr<ImageFrame>,Observation>(
        oneapi::tbb::filter_mode::parallel,[&](const std::shared_ptr<ImageFrame>& frame) {
          return workers.local()->detect(frame->gray,frame->timestamp_ns);
        }) &
      oneapi::tbb::make_filter<Observation,void>(
        oneapi::tbb::filter_mode::serial_in_order,[&](Observation observation) {
          ++result.frames;
          if(result.frames%100==0) std::clog << "Processed " << result.frames << " frames...\n";
          if(!observation.corners.empty()) {
            ++result.detected_frames;
            result.observations.push_back(std::move(observation));
          }
        }));
  });
  result.imu=source.take_imu();
  result.elapsed_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  if(result.frames==0) throw std::runtime_error("No camera frames found");
  if(result.imu.empty()) throw std::runtime_error("No IMU samples found");
  return result;
}
void write_cache(const std::string& path, const ExtractedData& d,
                 const CameraConfig& camera, const GridConfig& grid) {
  std::ofstream out(path);
  out << std::setprecision(17) << signature(camera,grid) << '\n';
  out << d.tag_border << ' ' << d.decimate << '\n';
  out << d.frames << ' ' << d.observations.size() << ' ' << d.imu.size() << '\n';
  for(const auto& o:d.observations) {
    out << o.timestamp_ns << ' ' << o.corners.size() << '\n';
    for(const auto& c:o.corners)
      out << c.tag_id << ' ' << c.corner_id << ' ' << c.pixel.x() << ' ' << c.pixel.y() << '\n';
  }
  for(const auto& i:d.imu) out << i.timestamp_ns << ' ' << i.gyro.transpose() << ' ' << i.accel.transpose() << '\n';
  if(!out) throw std::runtime_error("Unable to write observation cache");
}
ExtractedData read_cache(const std::string& path, const CameraConfig& camera,
                         const GridConfig& grid) {
  std::ifstream in(path); std::string line; std::getline(in,line);
  if(line!=signature(camera,grid)) throw std::runtime_error("Cache version, camera or AprilGrid differs from configuration");
  ExtractedData d; std::size_t no{},ni{};
  if(!(in>>d.tag_border>>d.decimate) || (d.tag_border!=1 && d.tag_border!=2) ||
     !std::isfinite(d.decimate) || d.decimate<1 || d.decimate>8)
    throw std::runtime_error("Invalid cached detector settings");
  d.from_cache=true;
  if(!(in>>d.frames>>no>>ni) || no>d.frames || ni==0) throw std::runtime_error("Invalid cache counts");
  auto stamp=[](std::int64_t t,std::int64_t previous) {
    if(t<0 || t<=previous) throw std::runtime_error("Invalid cache timestamps");
  };
  std::int64_t previous=-1;
  for(std::size_t i=0;i<no;++i) {
    Observation o; std::size_t nc{};
    if(!(in>>o.timestamp_ns>>nc) || nc==0 || nc>static_cast<std::size_t>(4*grid.rows*grid.cols))
      throw std::runtime_error("Invalid cache observation");
    stamp(o.timestamp_ns,previous); previous=o.timestamp_ns;
    std::set<std::pair<int,int>> seen;
    for(std::size_t j=0;j<nc;++j) {
      Corner c;
      if(!(in>>c.tag_id>>c.corner_id>>c.pixel.x()>>c.pixel.y()) || c.tag_id<0 ||
          c.tag_id>=grid.rows*grid.cols || c.corner_id<0 || c.corner_id>3 || !c.pixel.allFinite() ||
          c.pixel.x()<0 || c.pixel.x()>=camera.width || c.pixel.y()<0 || c.pixel.y()>=camera.height ||
          !seen.emplace(c.tag_id,c.corner_id).second) throw std::runtime_error("Invalid cached corner");
      const int dx=c.corner_id==1 || c.corner_id==2, dy=c.corner_id>=2;
      c.point=Eigen::Vector3d((c.tag_id%grid.cols*(1+grid.tag_spacing)+dx)*grid.tag_size,
                             (c.tag_id/grid.cols*(1+grid.tag_spacing)+dy)*grid.tag_size,0);
      o.corners.push_back(c);
    }
    d.observations.push_back(std::move(o));
  }
  previous=-1;
  for(std::size_t i=0;i<ni;++i) {
    ImuSample s;
    if(!(in>>s.timestamp_ns>>s.gyro.x()>>s.gyro.y()>>s.gyro.z()>>s.accel.x()>>s.accel.y()>>s.accel.z()) ||
       !s.gyro.allFinite() || !s.accel.allFinite()) throw std::runtime_error("Invalid cached IMU sample");
    stamp(s.timestamp_ns,previous); previous=s.timestamp_ns;
    d.imu.push_back(s);
  }
  if(in>>line) throw std::runtime_error("Unexpected data after cache");
  d.detected_frames=d.observations.size();
  return d;
}
} // namespace kalibr2
