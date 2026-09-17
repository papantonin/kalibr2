#include "kalibr2/rosbag_source.hpp"
#include "kalibr2/image_validation.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

namespace kalibr2 {
namespace {
constexpr std::size_t max_image_bytes = 512ULL * 1024 * 1024;
constexpr std::size_t max_imu_bytes = 64ULL * 1024;

std::int64_t checked_timestamp(const builtin_interfaces::msg::Time& stamp,
                               std::int64_t& previous, const std::string& topic) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
      (stamp.sec == 0 && stamp.nanosec == 0))
    throw std::runtime_error("Invalid or missing header timestamp on " + topic);
  const auto value = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  if (value <= previous)
    throw std::runtime_error("Header timestamps must strictly increase on " + topic);
  previous = value;
  return value;
}

template <typename Message>
Message deserialize(rosbag2_storage::SerializedBagMessage& bag_message) {
  // Transfer the buffer without copying it, leaving an INITIALIZED empty buffer
  // for rosbag2's deleter. A raw move resets its allocator and makes that deleter
  // fail rcutils_uint8_array_fini(), even though rclcpp frees the image payload.
  rclcpp::SerializedMessage serialized;
  std::swap(serialized.get_rcl_serialized_message(), *bag_message.serialized_data);
  Message message;
  rclcpp::Serialization<Message> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

cv::Mat raw_gray(const sensor_msgs::msg::Image& image, const CameraConfig& camera) {
  if (image.width != static_cast<std::uint32_t>(camera.width) ||
      image.height != static_cast<std::uint32_t>(camera.height))
    throw std::runtime_error("ROS image dimensions differ from camera configuration");
  int channels = 1;
  int depth = CV_8U;
  int conversion = -1;
  if (image.encoding == "mono8") {}
  else if (image.encoding == "mono16") depth = CV_16U;
  else if (image.encoding == "rgb8") { channels = 3; conversion = cv::COLOR_RGB2GRAY; }
  else if (image.encoding == "bgr8") { channels = 3; conversion = cv::COLOR_BGR2GRAY; }
  else if (image.encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2GRAY; }
  else if (image.encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2GRAY; }
  else throw std::runtime_error("Unsupported ROS image encoding: " + image.encoding);
  if (image.is_bigendian > 1)
    throw std::runtime_error("Invalid ROS image endian flag");
  const std::size_t bytes_per_pixel = static_cast<std::size_t>(channels) *
                                    (depth == CV_16U ? 2 : 1);
  const std::size_t min_step = static_cast<std::size_t>(camera.width) * bytes_per_pixel;
  const std::size_t expected_bytes = static_cast<std::size_t>(image.step) * image.height;
  if (image.step < min_step || image.step > min_step + 65536 ||
      image.data.size() != expected_bytes || expected_bytes > max_image_bytes)
    throw std::runtime_error("Invalid ROS image stride or payload length");
  // cv::Mat's external-memory constructor is not const-correct; all operations read this view.
  const cv::Mat view(camera.height, camera.width, CV_MAKETYPE(depth, channels),
                     const_cast<unsigned char*>(image.data.data()), image.step);
  if (image.encoding == "mono8") return view;
  cv::Mat gray;
  if (depth == CV_16U) {
    cv::Mat native = view.clone();
    const bool host_big_endian = std::endian::native == std::endian::big;
    if (static_cast<bool>(image.is_bigendian) != host_big_endian) {
      for (int row = 0; row < native.rows; ++row) {
        auto* values = native.ptr<std::uint16_t>(row);
        for (int col = 0; col < native.cols; ++col)
          values[col] = static_cast<std::uint16_t>((values[col] >> 8) | (values[col] << 8));
      }
    }
    native.convertTo(gray, CV_8U, 1.0 / 256.0);
  } else {
    cv::cvtColor(view, gray, conversion);
  }
  return gray;
}

cv::Mat compressed_gray(const sensor_msgs::msg::CompressedImage& image,
                         const CameraConfig& camera) {
  std::string format = image.format;
  std::transform(format.begin(), format.end(), format.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (format.find("compresseddepth") != std::string::npos ||
      (!format.empty() && format.find("jpeg") == std::string::npos &&
       format.find("jpg") == std::string::npos && format.find("png") == std::string::npos))
    throw std::runtime_error("Unsupported CompressedImage format: " + image.format);
  validate_encoded_dimensions(image.data.data(), image.data.size(), camera.width, camera.height);
  cv::Mat gray = cv::imdecode(image.data, cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);
  if (gray.empty() || gray.cols != camera.width || gray.rows != camera.height ||
      gray.type() != CV_8UC1)
    throw std::runtime_error("Unable to decode ROS compressed image at configured dimensions");
  return gray;
}
} // namespace

struct RosbagSource::Impl {
  rosbag2_cpp::Reader reader;
  CameraConfig camera;
  ImuConfig imu_config;
  std::vector<ImuSample> imu;
  std::string camera_type;
  std::size_t serialized_image_limit{};
  std::size_t image_count{};
  std::int64_t previous_image_stamp{};
  std::int64_t previous_imu_stamp{};
  bool drained{};
  bool imu_taken{};

  Impl(const std::string& uri, const CameraConfig& camera_in, const ImuConfig& imu_in)
      : camera(camera_in), imu_config(imu_in) {
    if (camera.width <= 0 || camera.height <= 0 || camera.topic.empty() ||
        imu_config.topic.empty() || camera.topic == imu_config.topic)
      throw std::runtime_error("ROS bag requires dimensions and distinct camera/IMU topics");
    const auto pixels = static_cast<std::uint64_t>(camera.width) * camera.height;
    if (pixels > (max_image_bytes - 65536) / 8)
      throw std::runtime_error("Configured resolution exceeds ROS image memory safety limit");
    serialized_image_limit = static_cast<std::size_t>(pixels * 8 + 65536);
    rosbag2_storage::StorageOptions storage;
    storage.uri = uri;
    const auto extension = std::filesystem::path(uri).extension();
    if (extension == ".mcap") storage.storage_id = "mcap";
    else if (extension == ".db3") storage.storage_id = "sqlite3";
    // An empty identifier lets rosbag2 select the plugin using directory metadata.
    rosbag2_cpp::ConverterOptions converter;
    // Accept the native serialization format only, avoiding format conversion.
    converter.input_serialization_format = rmw_get_serialization_format();
    converter.output_serialization_format = rmw_get_serialization_format();
    reader.open(storage, converter);
    bool found_imu = false;
    for (const auto& topic : reader.get_all_topics_and_types()) {
      if((topic.name==camera.topic || topic.name==imu_config.topic) &&
         topic.serialization_format!=converter.input_serialization_format)
        throw std::runtime_error("Unsupported bag serialization format on " + topic.name);
      if (topic.name == camera.topic) {
        if (!camera_type.empty() && camera_type != topic.type)
          throw std::runtime_error("Multiple message types recorded on camera topic " + camera.topic);
        camera_type = topic.type;
        if (camera_type != "sensor_msgs/msg/Image" &&
            camera_type != "sensor_msgs/msg/CompressedImage")
          throw std::runtime_error("Unsupported camera topic type: " + camera_type);
      }
      if (topic.name == imu_config.topic) {
        found_imu = true;
        if (topic.type != "sensor_msgs/msg/Imu")
          throw std::runtime_error("Unsupported IMU topic type: " + topic.type);
      }
    }
    if (camera_type.empty()) throw std::runtime_error("Camera topic missing from bag: " + camera.topic);
    if (!found_imu) throw std::runtime_error("IMU topic missing from bag: " + imu_config.topic);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {camera.topic, imu_config.topic};
    reader.set_filter(filter);
  }

  bool next(ImageFrame& frame) {
    if (drained) return false;
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      if (!bag_message || !bag_message->serialized_data)
        throw std::runtime_error("Bag contains an empty serialized message");
      const std::size_t bytes = bag_message->serialized_data->buffer_length;
      const bool is_imu = bag_message->topic_name == imu_config.topic;
      if (!is_imu && bag_message->topic_name != camera.topic) continue;
      if (bytes == 0 || bytes > (is_imu ? max_imu_bytes : serialized_image_limit))
        throw std::runtime_error("Serialized message exceeds configured size bound on " +
                                 bag_message->topic_name);
      if (is_imu) {
        const auto message = deserialize<sensor_msgs::msg::Imu>(*bag_message);
        ImuSample sample;
        sample.timestamp_ns = checked_timestamp(message.header.stamp, previous_imu_stamp,
                                                 imu_config.topic);
        sample.gyro = {message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z};
        sample.accel = {message.linear_acceleration.x, message.linear_acceleration.y,
                        message.linear_acceleration.z};
        if (!sample.gyro.allFinite() || !sample.accel.allFinite())
          throw std::runtime_error("Non-finite IMU measurement on " + imu_config.topic);
        if (message.angular_velocity_covariance[0] == -1 ||
            message.linear_acceleration_covariance[0] == -1)
          throw std::runtime_error("IMU message declares angular velocity or acceleration unavailable");
        imu.push_back(sample);
        continue;
      }
      ImageFrame decoded;
      decoded.source_bytes = bytes;
      if (camera_type == "sensor_msgs/msg/Image") {
        const auto message = std::make_shared<sensor_msgs::msg::Image>(
            deserialize<sensor_msgs::msg::Image>(*bag_message));
        decoded.timestamp_ns = checked_timestamp(message->header.stamp, previous_image_stamp, camera.topic);
        decoded.gray = raw_gray(*message, camera);
        if(message->encoding=="mono8") decoded.owner=message;
      } else {
        const auto message = deserialize<sensor_msgs::msg::CompressedImage>(*bag_message);
        decoded.timestamp_ns = checked_timestamp(message.header.stamp, previous_image_stamp, camera.topic);
        decoded.gray = compressed_gray(message, camera);
      }
      ++image_count;
      frame = std::move(decoded);
      return true;
    }
    drained = true;
    if (image_count == 0) throw std::runtime_error("Camera topic has no image messages");
    if (imu.empty()) throw std::runtime_error("IMU topic has no measurements");
    return false;
  }
};

RosbagSource::RosbagSource(const std::string& uri, const CameraConfig& camera, const ImuConfig& imu)
    : impl_(std::make_unique<Impl>(uri, camera, imu)) {}
RosbagSource::~RosbagSource() = default;
bool RosbagSource::next(ImageFrame& frame) { return impl_->next(frame); }
std::vector<ImuSample> RosbagSource::take_imu() {
  if (!impl_->drained)
    throw std::logic_error("Drain all ROS bag images before requesting IMU measurements");
  if (impl_->imu_taken) throw std::logic_error("IMU measurements already transferred");
  impl_->imu_taken = true;
  return std::move(impl_->imu);
}
} // namespace kalibr2
