#include "kalibr2/source.hpp"
#include "kalibr2/image_validation.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace kalibr2 {
namespace {
std::int64_t timestamp(const std::string& s) {
  std::int64_t value{};
  const auto [p, e] = std::from_chars(s.data(),s.data()+s.size(),value);
  if(e!=std::errc{} || p!=s.data()+s.size() || value<0) throw std::runtime_error("Invalid nanosecond timestamp: "+s);
  return value;
}
class DirectorySource final : public FrameSource {
 public:
  DirectorySource(const std::string& path, CameraConfig camera): camera_(std::move(camera)) {
    const std::filesystem::path root(path);
    for(const auto& file:std::filesystem::directory_iterator(root/"cam0")) {
      if(!file.is_regular_file()) continue;
      const auto ext=file.path().extension().string();
      if(ext!=".png" && ext!=".jpg" && ext!=".jpeg") continue;
      files_.emplace_back(timestamp(file.path().stem().string()),file.path());
    }
    std::sort(files_.begin(),files_.end());
    if(files_.empty()) throw std::runtime_error("No PNG/JPEG frames in cam0/");
    for(std::size_t i=1;i<files_.size();++i)
      if(files_[i-1].first==files_[i].first) throw std::runtime_error("Duplicate camera timestamp");
    std::ifstream input(root/"imu0.csv");
    if(!input) throw std::runtime_error("Missing imu0.csv");
    std::string line;
    while(std::getline(input,line)) {
      if(line.empty() || line[0]=='#') continue;
      std::replace(line.begin(),line.end(),',',' ');
      std::istringstream row(line); std::string stamp; ImuSample sample;
      if(!(row>>stamp>>sample.gyro.x()>>sample.gyro.y()>>sample.gyro.z()
              >>sample.accel.x()>>sample.accel.y()>>sample.accel.z()))
        throw std::runtime_error("Expected imu0.csv columns: timestamp_ns,wx,wy,wz,ax,ay,az");
      std::string extra; if(row>>extra) throw std::runtime_error("Unexpected extra IMU CSV column");
      sample.timestamp_ns=timestamp(stamp);
      if(!sample.gyro.allFinite() || !sample.accel.allFinite()) throw std::runtime_error("Non-finite IMU sample");
      if(!imu_.empty() && sample.timestamp_ns<=imu_.back().timestamp_ns) throw std::runtime_error("IMU timestamps must be strictly increasing");
      imu_.push_back(sample);
    }
    if(imu_.empty()) throw std::runtime_error("Empty IMU stream");
  }
  bool next(ImageFrame& frame) override {
    if(index_==files_.size()) return false;
    const auto& [stamp,path]=files_[index_++];
    const auto pixels=static_cast<std::size_t>(camera_.width)*camera_.height;
    frame.source_bytes=std::filesystem::file_size(path);
    if(frame.source_bytes>pixels*8+1024*1024) throw std::runtime_error("Image file exceeds expected resolution budget: "+path.string());
    std::vector<unsigned char> encoded(frame.source_bytes);
    std::ifstream image(path,std::ios::binary);
    if(!image.read(reinterpret_cast<char*>(encoded.data()),static_cast<std::streamsize>(encoded.size())))
      throw std::runtime_error("Unable to read image: "+path.string());
    validate_encoded_dimensions(encoded.data(),encoded.size(),camera_.width,camera_.height);
    frame.gray=cv::imdecode(encoded,cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);
    if(frame.gray.empty() || frame.gray.cols!=camera_.width || frame.gray.rows!=camera_.height)
      throw std::runtime_error("Image dimensions disagree with camera YAML: "+path.string());
    frame.timestamp_ns=stamp;
    return true;
  }
  std::vector<ImuSample> take_imu() override { return std::move(imu_); }
 private:
  CameraConfig camera_;
  std::vector<std::pair<std::int64_t,std::filesystem::path>> files_;
  std::vector<ImuSample> imu_;
  std::size_t index_{};
};
}
std::unique_ptr<FrameSource> directory_source(const std::string& path,const CameraConfig& camera) {
  return std::make_unique<DirectorySource>(path,camera);
}
} // namespace kalibr2
