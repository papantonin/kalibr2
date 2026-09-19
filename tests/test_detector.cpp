#include "kalibr2/detector.hpp"

#include <opencv2/imgproc.hpp>
#include <oneapi/tbb/enumerable_thread_specific.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_arena.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition, const std::string& description) {
  if (!condition) throw std::runtime_error(description);
}

// Reference codewords from Kalibr's kalibr_create_target_pdf, not AprilTag3's
// reordered codebook. Render the published PDF convention directly: MSB at
// top-left, white=1, IDs increasing to the right and from bottom to top.
// https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_create_target_pdf
constexpr std::array<std::uint64_t, 6> historical_codes{
    0xd5d628584ULL, 0xd97f18b49ULL, 0xdd280910eULL,
    0xe479e9c98ULL, 0xebcbca822ULL, 0xf31dab3acULL};
constexpr int margin = 80;
constexpr int side = 160;
constexpr int gap = 40;
constexpr int rows = 2;
constexpr int cols = 3;
const kalibr2::GridConfig grid{rows, cols, 0.08, 0.25};

cv::Mat make_board(int border, bool duplicate = false) {
  cv::Mat board(2 * margin + rows * side + (rows - 1) * gap,
                2 * margin + cols * side + (cols - 1) * gap,
                CV_8UC1, cv::Scalar(255));
  const int cell = side / (6 + 2 * border);
  require(cell * (6 + 2 * border) == side, "Test tag must contain integral pixel cells");
  for (int id = 0; id < rows * cols; ++id) {
    const int x = margin + (id % cols) * (side + gap);
    const int y = margin + (rows - 1 - id / cols) * (side + gap);
    board(cv::Rect(x, y, side, side)).setTo(0);
    const auto code = historical_codes[duplicate && id == 1 ? 0 : id];
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        if ((code >> (35 - r * 6 - c)) & 1ULL) {
          board(cv::Rect(x + (border + c) * cell, y + (border + r) * cell, cell, cell)).setTo(255);
        }
      }
    }
    // Kalibr symmCorners=true adds black diagonal squares in the white gaps.
    for (int dx : {-gap, side}) {
      for (int dy : {-gap, side}) board(cv::Rect(x + dx, y + dy, gap, gap)).setTo(0);
    }
  }
  return board;
}

Eigen::Vector2d rotated_pixel(Eigen::Vector2d point, int rotation, int width, int height) {
  switch (rotation) {
    case 0: return point;
    case 1: return {height - 1 - point.y(), point.x()};
    case 2: return {width - 1 - point.x(), height - 1 - point.y()};
    case 3: return {point.y(), width - 1 - point.x()};
  }
  throw std::runtime_error("Invalid test rotation");
}

void check_board(const kalibr2::Observation& observation, int rotation = 0, bool require_all = true) {
  require(observation.timestamp_ns == 1234567890123456789LL, "Timestamp must remain exact");
  require(require_all ? observation.corners.size()==24 : observation.corners.size()>=16,
          "Insufficient Kalibr corners; detected " + std::to_string(observation.corners.size()));
  constexpr int offset_x[4]{0, 1, 1, 0};
  constexpr int offset_y[4]{1, 1, 0, 0};
  const int width = 2 * margin + cols * side + (cols - 1) * gap;
  const int height = 2 * margin + rows * side + (rows - 1) * gap;
  int previous=-1;
  for (const auto& corner:observation.corners) {
      const int id=corner.tag_id, c=corner.corner_id;
      require(id>=0 && id<6 && c>=0 && c<4 && 4*id+c>previous, "ID/corner ordering mismatch");
      previous=4*id+c;
      Eigen::Vector2d expected_pixel{
          margin + (id % cols) * (side + gap) + offset_x[c] * side - 0.5,
          margin + (rows - 1 - id / cols) * (side + gap) + offset_y[c] * side - 0.5};
      expected_pixel = rotated_pixel(expected_pixel, rotation, width, height);
      require((corner.pixel - expected_pixel).norm() < 0.2,
              "Kalibr image corner mismatch for tag " + std::to_string(id) +
              " corner " + std::to_string(c) + ": " + std::to_string(corner.pixel.x()) +
              "," + std::to_string(corner.pixel.y()));
      const Eigen::Vector3d expected_point{
          (id % cols) * 0.1 + offset_x[c] * 0.08,
          (id / cols) * 0.1 + (1 - offset_y[c]) * 0.08, 0.0};
      require((corner.point - expected_point).norm() < 1e-12, "Kalibr metric target geometry mismatch");
  }
}

template <class Exception, class Action>
void expect_throw(Action&& action, const std::string& description) {
  try { action(); }
  catch (const Exception&) { return; }
  throw std::runtime_error(description);
}
} // namespace

int main() {
  try {
    cv::setNumThreads(1);
    for (const auto backend : {kalibr2::DetectorBackend::AprilTag3, kalibr2::DetectorBackend::Kalibr}) {
      kalibr2::DetectorOptions defaults;
      defaults.backend = backend;
      for (const int border : {1, 2}) {
        const cv::Mat original = make_board(border);
        for (const double decimation : {1.0, 2.0}) {
          if (backend == kalibr2::DetectorBackend::Kalibr && decimation != 1.0) continue;
          std::cout << "Testing " << kalibr2::detector_name(backend) << " border=" << border << " decimation=" << decimation << std::endl;
          kalibr2::DetectorOptions options = defaults;
          options.tag_border = border;
          options.quad_decimate = decimation;
          kalibr2::AprilGridDetector detector(grid, options);
          for (int rotation = 0; rotation < 4; ++rotation) {
            cv::Mat oriented;
            if (rotation == 0) oriented = original;
            if (rotation == 1) cv::rotate(original, oriented, cv::ROTATE_90_CLOCKWISE);
            if (rotation == 2) cv::rotate(original, oriented, cv::ROTATE_180);
            if (rotation == 3) cv::rotate(original, oriented, cv::ROTATE_90_COUNTERCLOCKWISE);
            // Exercise zero-copy views with padding: an ROI's stride exceeds width.
            cv::Mat padded(oriented.rows + 10, oriented.cols + 32, CV_8UC1, cv::Scalar(137));
            cv::Mat roi = padded(cv::Rect(13, 5, oriented.cols, oriented.rows));
            oriented.copyTo(roi);
            require(!roi.isContinuous(), "Test image must have padded rows");
            const cv::Mat before = padded.clone();
            // Decimation can lose a tag: validate all returned coordinates and
            // require four visible tags; full-resolution detection must find all.
            check_board(detector.detect(roi, 1234567890123456789LL), rotation, decimation==1.0);
            require(cv::norm(padded, before, cv::NORM_INF) == 0, "Detector must not modify the input buffer");
          }
        }
      }

      const cv::Mat board = make_board(2);
      kalibr2::AprilGridDetector detector(grid, defaults);
      expect_throw<std::runtime_error>([&] { detector.detect(make_board(2, true), 1); },
                                      "Duplicate board IDs must be rejected");
      expect_throw<std::invalid_argument>([&] { detector.detect(cv::Mat{}, 1); },
                                         "Empty images must be rejected");
      expect_throw<std::invalid_argument>([&] { detector.detect(cv::Mat(100, 100, CV_8UC3), 1); },
                                         "Color input must be explicitly converted by the pipeline");
      expect_throw<std::invalid_argument>([] {
        kalibr2::AprilGridDetector invalid(kalibr2::GridConfig{100000, 100000, 0.08, 0.25});
      }, "Out-of-range tag counts must be rejected without multiplication overflow");
      expect_throw<std::invalid_argument>([] {
        kalibr2::DetectorOptions options;
        options.tag_border = 3;
        kalibr2::AprilGridDetector invalid(grid, options);
      }, "Unsupported borders must be rejected");
      require(detector.detect(cv::Mat(100, 100, CV_8UC1, cv::Scalar(255)), 42).corners.empty(),
              "Blank frames must yield empty observations");
      kalibr2::AprilGridDetector smaller_grid(kalibr2::GridConfig{2, 2, 0.08, 0.25}, defaults);
      const auto limited = smaller_grid.detect(board, 42);
      require(limited.corners.size() == 16, "Tags outside configured ID range must be excluded");
      for (const auto& corner : limited.corners) require(corner.tag_id < 4, "Out-of-range ID survived filtering");

      // Same immutable image, independent detector state per TBB worker. The
      // caller's bounded streaming scheduler controls how many images can live.
      oneapi::tbb::enumerable_thread_specific<std::unique_ptr<kalibr2::AprilGridDetector>> workers(
          [&] { return std::make_unique<kalibr2::AprilGridDetector>(grid, defaults); });
      oneapi::tbb::task_arena arena(4);
      arena.execute([&] {
        oneapi::tbb::parallel_for(0, 16, [&](int) {
          check_board(workers.local()->detect(board, 1234567890123456789LL));
        });
      });
      if (backend == kalibr2::DetectorBackend::Kalibr) {
        // Oblique, small tags with corner-touching separators caused real-data losses.
        cv::Mat oblique;
        const auto M = cv::getRotationMatrix2D(cv::Point2f(board.cols / 2.0F, board.rows / 2.0F), 35, 0.2);
        cv::warpAffine(board, oblique, M, board.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
        require(detector.detect(oblique, 1).corners.size() >= 20,
                "Historical detector must recover oblique tags with separators");
        auto invalid = defaults; invalid.quad_decimate = 2;
        expect_throw<std::invalid_argument>([&] { kalibr2::AprilGridDetector bad(grid, invalid); },
                                           "Historical decimation must not be silently ignored");
      }
    }
    expect_throw<std::invalid_argument>([] { kalibr2::parse_detector_backend("unknown"); },
                                       "Unknown backend must fail");
    std::cout << "AprilGrid: Kalibr border 2 and standard border 1, rotations, full-resolution refinement, "
                 "strides, immutable input, invalid/duplicate IDs and parallel workers passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "AprilGrid test failure: " << error.what() << '\n';
    return 1;
  }
}
