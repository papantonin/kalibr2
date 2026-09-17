#include "kalibr2/image_validation.hpp"
#include "kalibr2/rosbag_source.hpp"

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
template <typename Function>
void expect_error(Function&& function, const std::string& context) {
  bool threw = false;
  try { function(); } catch (const std::exception&) { threw = true; }
  require(threw, "Expected rejection: " + context);
}
struct TemporaryDirectory {
  std::filesystem::path path;
  TemporaryDirectory() {
    char pattern[] = "/tmp/kalibr2-rosbag-test-XXXXXX";
    const char* created = mkdtemp(pattern);
    if (!created) throw std::runtime_error("Cannot create rosbag test directory");
    path = created;
  }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

template <typename Message>
void write(rosbag2_cpp::Writer& writer, const Message& message, const std::string& topic,
           const std::string& type, std::int64_t bag_stamp) {
  auto serialized = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<Message> serialization;
  serialization.serialize_message(&message, serialized.get());
  writer.write(serialized, topic, type, rclcpp::Time(bag_stamp));
}

template <typename Message>
void make_bag(const std::filesystem::path& path, const Message& image,
              const std::string& image_type, const std::string& storage = "sqlite3",
              bool include_imu = true, bool repeat_image = false) {
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions options;
  options.uri = path.string();
  options.storage_id = storage;
  writer.open(options);
  if (include_imu) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp.sec = 10;
    imu.header.stamp.nanosec = 123;
    imu.angular_velocity.x = 0.25;
    imu.linear_acceleration.z = 9.81;
    write(writer, imu, "/imu", "sensor_msgs/msg/Imu", 1000000);
    // Camera header stamps intentionally differ from bag receive timestamps.
    write(writer, image, "/camera", image_type, 2000000);
    if (repeat_image) write(writer, image, "/camera", image_type, 2500000);
    imu.header.stamp.nanosec = 456;
    write(writer, imu, "/imu", "sensor_msgs/msg/Imu", 3000000);
  } else {
    write(writer, image, "/camera", image_type, 2000000);
  }
  writer.close();
}

kalibr2::CameraConfig camera() {
  kalibr2::CameraConfig config;
  config.topic = "/camera";
  config.width = 4;
  config.height = 3;
  return config;
}
kalibr2::ImuConfig imu_config() {
  kalibr2::ImuConfig config;
  config.topic = "/imu";
  return config;
}

sensor_msgs::msg::Image raw_image(const std::string& encoding) {
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 10;
  image.header.stamp.nanosec = 234;
  image.width = 4;
  image.height = 3;
  image.encoding = encoding;
  const bool mono16 = encoding == "mono16";
  const int channels = encoding == "rgb8" || encoding == "bgr8" ? 3 :
                       encoding == "rgba8" || encoding == "bgra8" ? 4 : 1;
  const int bytes = mono16 ? 2 : channels;
  image.step = image.width * bytes + 4; // Verify row padding is respected.
  image.data.resize(image.step * image.height, 255);
  image.is_bigendian = mono16 ? 1 : 0;
  for (unsigned row = 0; row < image.height; ++row) {
    for (unsigned col = 0; col < image.width; ++col) {
      auto* pixel = image.data.data() + row * image.step + col * bytes;
      if (mono16) { pixel[0] = 100; pixel[1] = 0; }
      else if (channels == 1) pixel[0] = 100;
      else {
        pixel[0] = encoding[0] == 'r' ? 255 : 0;
        pixel[1] = 0;
        pixel[2] = encoding[0] == 'b' ? 255 : 0;
        if (channels == 4) pixel[3] = 255;
      }
    }
  }
  return image;
}

void verify_bag(const std::filesystem::path& path, unsigned char intensity) {
  kalibr2::RosbagSource source(path.string(), camera(), imu_config());
  expect_error([&] { source.take_imu(); }, "take_imu before complete traversal");
  kalibr2::ImageFrame frame;
  require(source.next(frame), "Missing image");
  require(frame.timestamp_ns == 10000000234LL, "Used bag receive time instead of header time");
  require(frame.source_bytes > 0, "Missing source size");
  require(frame.gray.type() == CV_8UC1 && frame.gray.cols == 4 && frame.gray.rows == 3,
          "Wrong decoded image shape");
  const cv::Mat held_image = frame.gray;
  require(!source.next(frame), "Unexpected second image");
  require(!source.next(frame), "Drained source should stay drained");
  require(cv::countNonZero(held_image != intensity) == 0, "Pixel conversion or image lifetime failed");
  auto imu = source.take_imu();
  require(imu.size() == 2, "Trailing IMU sample was lost");
  require(imu[0].timestamp_ns == 10000000123LL && imu[1].timestamp_ns == 10000000456LL,
          "Incorrect IMU header timestamps");
  require(std::abs(imu[0].gyro.x() - 0.25) < 1e-12 &&
          std::abs(imu[0].accel.z() - 9.81) < 1e-12, "Incorrect IMU values");
  expect_error([&] { source.take_imu(); }, "take_imu twice");
}
} // namespace

int main() {
  try {
    TemporaryDirectory temp;
    for (const std::string encoding : {"mono8", "mono16", "rgb8", "bgr8", "rgba8", "bgra8"}) {
      const auto path = temp.path / encoding;
      make_bag(path, raw_image(encoding), "sensor_msgs/msg/Image");
      verify_bag(path, encoding.starts_with("mono") ? 100 : 76);
    }
    // Both storage plugins and direct file URIs must work, not just directories.
    const auto mcap = temp.path / "mcap_bag";
    make_bag(mcap, raw_image("mono8"), "sensor_msgs/msg/Image", "mcap");
    verify_bag(mcap, 100);
    for (const auto& entry : std::filesystem::directory_iterator(mcap))
      if (entry.path().extension() == ".mcap") verify_bag(entry.path(), 100);
    for (const auto& entry : std::filesystem::directory_iterator(temp.path / "mono8"))
      if (entry.path().extension() == ".db3") verify_bag(entry.path(), 100);

    for (const std::string extension : {".png", ".jpg"}) {
      sensor_msgs::msg::CompressedImage image;
      image.header.stamp.sec = 10;
      image.header.stamp.nanosec = 234;
      image.format = extension == ".png" ? "mono8; png compressed" : "jpeg";
      cv::imencode(extension, cv::Mat(3, 4, CV_8UC1, cv::Scalar(100)), image.data);
      const auto path = temp.path / ("compressed" + extension);
      make_bag(path, image, "sensor_msgs/msg/CompressedImage");
      verify_bag(path, 100);
      expect_error([&] { kalibr2::validate_encoded_dimensions(image.data.data(), image.data.size(), 5, 3); },
                   "Encoded resolution mismatch");
      expect_error([&] { kalibr2::validate_encoded_dimensions(image.data.data(), 8, 4, 3); },
                   "Truncated image header");
    }

    auto bad_image = raw_image("mono8");
    bad_image.step = 1;
    make_bag(temp.path / "bad_stride", bad_image, "sensor_msgs/msg/Image");
    expect_error([&] {
      kalibr2::RosbagSource source((temp.path / "bad_stride").string(), camera(), imu_config());
      kalibr2::ImageFrame frame;
      source.next(frame);
    }, "Invalid image stride");
    bad_image = raw_image("mono8");
    bad_image.header.stamp.sec = 0;
    bad_image.header.stamp.nanosec = 0;
    make_bag(temp.path / "zero_timestamp", bad_image, "sensor_msgs/msg/Image");
    expect_error([&] {
      kalibr2::RosbagSource source((temp.path / "zero_timestamp").string(), camera(), imu_config());
      kalibr2::ImageFrame frame;
      source.next(frame);
    }, "Empty header timestamp");
    make_bag(temp.path / "duplicate_timestamp", raw_image("mono8"), "sensor_msgs/msg/Image", "sqlite3", true, true);
    expect_error([&] {
      kalibr2::RosbagSource source((temp.path / "duplicate_timestamp").string(), camera(), imu_config());
      kalibr2::ImageFrame frame;
      while (source.next(frame)) {}
    }, "Duplicate camera timestamp");
    make_bag(temp.path / "missing_imu", raw_image("mono8"), "sensor_msgs/msg/Image", "sqlite3", false);
    expect_error([&] {
      kalibr2::RosbagSource source((temp.path / "missing_imu").string(), camera(), imu_config());
    }, "Missing IMU topic");
    expect_error([&] {
      auto missing_camera = camera();
      missing_camera.topic = "/missing_camera";
      kalibr2::RosbagSource source((temp.path / "mono8").string(), missing_camera, imu_config());
    }, "Missing camera topic");
    make_bag(temp.path / "wrong_type", sensor_msgs::msg::Imu{}, "sensor_msgs/msg/Imu");
    expect_error([&] {
      kalibr2::RosbagSource source((temp.path / "wrong_type").string(), camera(), imu_config());
    }, "Unsupported camera type");
    std::cout << "ROS bag adapter tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ROS bag adapter test failed: " << error.what() << '\n';
    return 1;
  }
}
