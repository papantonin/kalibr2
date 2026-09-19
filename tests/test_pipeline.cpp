#include "kalibr2/pipeline.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>

namespace {
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
template<class F> void rejects(F&& f) {
  bool rejected=false; try { f(); } catch(const std::exception&) { rejected=true; }
  require(rejected,"Expected failure");
}
class Source : public kalibr2::FrameSource {
 public:
  int reads{};
  bool next(kalibr2::ImageFrame& f) override {
    if(reads==12) return false;
    f.timestamp_ns=1700000000000000000LL+(++reads)*10000000;
    f.gray=cv::Mat(48,64,CV_8UC1,cv::Scalar(255));
    return true;
  }
  std::vector<kalibr2::ImuSample> take_imu() override {
    return {{1700000000000000000LL,Eigen::Vector3d(1,2,3),Eigen::Vector3d(0,0,9.81)}};
  }
};
}
int main() {
  const auto path=std::filesystem::temp_directory_path()/
    ("kalibr2-cache-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    cv::setNumThreads(1);
    kalibr2::CameraConfig c; c.width=64;c.height=48;c.intrinsics={100,100,32,24};
    kalibr2::GridConfig g{2,2,0.1,0.3};
    kalibr2::PipelineOptions opts; opts.threads=4;
    Source s;
    const auto extracted=kalibr2::extract(s,c,g,{},opts);
    require(extracted.frames==12 && extracted.detected_frames==0,"Frames lost or false detections");
    require(extracted.max_in_flight==4,"Default admission should follow the thread count");
    Source limited;opts.image_memory_bytes=3*1048576;
    const auto memory_limited=kalibr2::extract(limited,c,g,{},opts);
    require(memory_limited.max_in_flight==1,"Configured memory admission did not restrict concurrency");
    require(extracted.imu.size()==1,"IMU lost");
    Source unread;opts.image_memory_bytes=1;
    rejects([&]{kalibr2::extract(unread,c,g,{},opts);});
    require(unread.reads==0,"Admission must reject BEFORE image allocation");
    kalibr2::ExtractedData d=extracted;
    kalibr2::Observation o; o.timestamp_ns=1700000000000000123LL;
    o.corners.push_back({3,2,Eigen::Vector2d(20.25,30.5),Eigen::Vector3d(0.23,0.23,0)});
    d.observations.push_back(o);
    kalibr2::write_cache(path.string(),d,c,g);
    const auto roundtrip=kalibr2::read_cache(path.string(),c,g);
    require(roundtrip.observations.at(0).timestamp_ns==o.timestamp_ns,"Epoch precision lost");
    require((roundtrip.observations[0].corners[0].point-o.corners[0].point).norm()<1e-14,"Corner geometry changed");
    require((roundtrip.imu[0].gyro-d.imu[0].gyro).norm()==0,"IMU cache changed");
    require(roundtrip.detector_backend==kalibr2::DetectorBackend::Kalibr,"Default backend lost");
    d.detector_backend=kalibr2::DetectorBackend::AprilTag3;
    kalibr2::write_cache(path.string(),d,c,g);
    require(kalibr2::read_cache(path.string(),c,g).detector_backend==kalibr2::DetectorBackend::AprilTag3,
            "AprilTag 3 backend lost in cache");
    // Old V1 caches contain only AprilTag3 observations, without a backend token.
    std::ifstream cache(path); std::ostringstream contents; contents << cache.rdbuf(); cache.close();
    auto old=contents.str();
    old.replace(old.find("KALIBR2_CACHE_V2"),std::string("KALIBR2_CACHE_V2").size(),"KALIBR2_CACHE_V1");
    old.erase(old.find("apriltag3 "),10);
    { std::ofstream legacy(path); legacy << old; }
    const auto v1=kalibr2::read_cache(path.string(),c,g);
    require(v1.detector_backend==kalibr2::DetectorBackend::AprilTag3 && v1.observations.size()==1,
            "V1 cache compatibility broken");
    auto corrupt_backend=contents.str();
    corrupt_backend.replace(corrupt_backend.find("apriltag3 "),9,"unknown");
    { std::ofstream invalid(path); invalid << corrupt_backend; }
    rejects([&]{kalibr2::read_cache(path.string(),c,g);});
    kalibr2::write_cache(path.string(),d,c,g);
    auto other=c; other.intrinsics[0]+=1;
    rejects([&]{kalibr2::read_cache(path.string(),other,g);});
    std::ofstream corrupt(path,std::ios::app); corrupt << "garbage";corrupt.close();
    rejects([&]{kalibr2::read_cache(path.string(),c,g);});
    std::filesystem::remove(path);
    std::cout << "Pipeline admission, ordering and cache validation passed\n";
    return 0;
  } catch(const std::exception& e) {
    std::filesystem::remove(path);
    std::cerr << e.what() << '\n'; return 1;
  }
}
