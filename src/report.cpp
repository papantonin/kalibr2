#include "kalibr2/report.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace kalibr2 {
namespace {
struct Summary {
  std::size_t count{};
  double mean{}, median{}, p95{}, max{}, rmse{};
};
struct FrameSummary {
  int index{};
  std::int64_t timestamp_ns{};
  double time_s{};
  std::size_t count{};
  double mean{}, max{};
};
std::string number(double value, int precision = 6) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}
std::string json_number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}
std::string utc_time_minute() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &time);
#else
  if (const std::tm* value = std::gmtime(&time)) utc = *value;
#endif
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%d %H:%M UTC");
  return out.str();
}
std::string pdf_escape(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  for (char c : text) {
    if (c == '(' || c == ')' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}
std::vector<double> residual_norms(const CalibrationResult& result) {
  std::vector<double> norms;
  norms.reserve(result.reprojection_residuals.size());
  for (const auto& r : result.reprojection_residuals) norms.push_back(r.residual_pixel.norm());
  return norms;
}
Summary summarize(std::vector<double> values) {
  Summary s;
  s.count = values.size();
  if (values.empty()) return s;
  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  double squared = 0;
  for (double value : values) squared += value * value;
  std::sort(values.begin(), values.end());
  const auto pick = [&](double q) {
    const std::size_t index = std::min<std::size_t>(values.size() - 1,
        static_cast<std::size_t>(std::floor(q * static_cast<double>(values.size() - 1))));
    return values[index];
  };
  s.mean = sum / static_cast<double>(values.size());
  s.median = pick(0.5);
  s.p95 = pick(0.95);
  s.max = values.back();
  s.rmse = std::sqrt(squared / static_cast<double>(values.size()));
  return s;
}
std::vector<FrameSummary> frame_summaries(const CalibrationResult& result) {
  struct Acc { std::int64_t timestamp{}; std::size_t count{}; double sum{}, max{}; };
  std::map<int, Acc> grouped;
  for (const auto& r : result.reprojection_residuals) {
    auto& acc = grouped[r.frame_index];
    acc.timestamp = r.timestamp_ns;
    const double norm = r.residual_pixel.norm();
    ++acc.count;
    acc.sum += norm;
    acc.max = std::max(acc.max, norm);
  }
  std::vector<FrameSummary> frames;
  if (grouped.empty()) return frames;
  const std::int64_t first = grouped.begin()->second.timestamp;
  for (const auto& [index, acc] : grouped) {
    frames.push_back({index, acc.timestamp,
        static_cast<double>((static_cast<long double>(acc.timestamp) -
                             static_cast<long double>(first)) * 1e-9L),
        acc.count, acc.count ? acc.sum / static_cast<double>(acc.count) : 0.0, acc.max});
  }
  return frames;
}
void write_csv(const std::filesystem::path& path, const CalibrationResult& result) {
  std::ofstream out(path);
  out << "timestamp_ns,frame_index,tag_id,corner_id,observed_x,observed_y,residual_x_px,residual_y_px,residual_norm_px\n";
  out << std::setprecision(17);
  for (const auto& r : result.reprojection_residuals) {
    out << r.timestamp_ns << ',' << r.frame_index << ',' << r.tag_id << ',' << r.corner_id << ','
        << r.observed_pixel.x() << ',' << r.observed_pixel.y() << ','
        << r.residual_pixel.x() << ',' << r.residual_pixel.y() << ','
        << r.residual_pixel.norm() << '\n';
  }
  if (!out) throw std::runtime_error("Unable to write residuals CSV");
}
void write_summary_json(const std::filesystem::path& path, const CalibrationResult& result,
                        const Summary& summary, const std::vector<FrameSummary>& frames,
                        const std::string& generated_at) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"corner_residuals\": " << summary.count << ",\n";
  out << "  \"camera_frames\": " << frames.size() << ",\n";
  out << "  \"generated_at_utc\": \"" << generated_at << "\",\n";
  out << "  \"rmse_px\": " << json_number(result.reprojection_rmse) << ",\n";
  out << "  \"mean_px\": " << json_number(summary.mean) << ",\n";
  out << "  \"median_px\": " << json_number(summary.median) << ",\n";
  out << "  \"p95_px\": " << json_number(summary.p95) << ",\n";
  out << "  \"max_px\": " << json_number(summary.max) << ",\n";
  out << "  \"timeshift_cam_imu_s\": " << json_number(result.timeshift_cam_imu) << ",\n";
  out << "  \"initial_cost\": " << json_number(result.initial_cost) << ",\n";
  out << "  \"final_cost\": " << json_number(result.final_cost) << "\n";
  out << "}\n";
  if (!out) throw std::runtime_error("Unable to write residual summary JSON");
}
class Pdf {
 public:
  explicit Pdf(std::filesystem::path path) : path_(std::move(path)) {}
  void page() { pages_.emplace_back(); }
  void text(double x, double y, double size, const std::string& value) {
    current() << "BT /F1 " << size << " Tf " << x << ' ' << y << " Td (" << pdf_escape(value) << ") Tj ET\n";
  }
  void line(double x1, double y1, double x2, double y2) {
    current() << "0 G 0.8 w " << x1 << ' ' << y1 << " m " << x2 << ' ' << y2 << " l S\n";
  }
  void rect(double x, double y, double w, double h, double gray) {
    current() << gray << " g " << x << ' ' << y << ' ' << w << ' ' << h << " re f\n0 G 0.3 w "
              << x << ' ' << y << ' ' << w << ' ' << h << " re S\n";
  }
  void color_rect(double x, double y, double w, double h, double intensity) {
    intensity = std::clamp(intensity, 0.0, 1.0);
    const double r = 1.0;
    const double g = 1.0 - 0.75 * intensity;
    const double b = 1.0 - 0.75 * intensity;
    current() << r << ' ' << g << ' ' << b << " rg " << x << ' ' << y << ' ' << w << ' ' << h << " re f\n"
              << "0.75 G 0.2 w " << x << ' ' << y << ' ' << w << ' ' << h << " re S\n";
  }
  void polyline(const std::vector<std::pair<double, double>>& pts) {
    if (pts.size() < 2) return;
    current() << "0 0.2 0.75 RG 1.1 w " << pts[0].first << ' ' << pts[0].second << " m ";
    for (std::size_t i = 1; i < pts.size(); ++i) current() << pts[i].first << ' ' << pts[i].second << " l ";
    current() << "S\n0 G\n";
  }
  void write() {
    if (pages_.empty()) page();
    std::vector<std::string> objects;
    objects.push_back("<< /Type /Catalog /Pages 2 0 R >>\n");
    std::ostringstream kids;
    kids << "<< /Type /Pages /Kids [";
    const int first_page = 4;
    for (std::size_t i = 0; i < pages_.size(); ++i) kids << first_page + static_cast<int>(2 * i) << " 0 R ";
    kids << "] /Count " << pages_.size() << " >>\n";
    objects.push_back(kids.str());
    objects.push_back("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>\n");
    for (std::size_t i = 0; i < pages_.size(); ++i) {
      const int content_id = first_page + static_cast<int>(2 * i) + 1;
      std::ostringstream page_obj;
      page_obj << "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /Font << /F1 3 0 R >> >> /Contents "
               << content_id << " 0 R >>\n";
      objects.push_back(page_obj.str());
      const std::string stream = pages_[i].str();
      std::ostringstream content;
      content << "<< /Length " << stream.size() << " >>\nstream\n" << stream << "endstream\n";
      objects.push_back(content.str());
    }
    std::ofstream out(path_, std::ios::binary);
    out << "%PDF-1.4\n";
    std::vector<std::streamoff> offsets{0};
    for (std::size_t i = 0; i < objects.size(); ++i) {
      offsets.push_back(out.tellp());
      out << i + 1 << " 0 obj\n" << objects[i] << "endobj\n";
    }
    const auto xref = out.tellp();
    out << "xref\n0 " << offsets.size() << "\n0000000000 65535 f \n";
    for (std::size_t i = 1; i < offsets.size(); ++i) {
      out << std::setw(10) << std::setfill('0') << offsets[i] << " 00000 n \n";
    }
    out << std::setfill(' ') << "trailer\n<< /Size " << offsets.size() << " /Root 1 0 R >>\nstartxref\n"
        << xref << "\n%%EOF\n";
    if (!out) throw std::runtime_error("Unable to write calibration PDF report");
  }
 private:
  std::ostringstream& current() {
    if (pages_.empty()) page();
    return pages_.back();
  }
  std::filesystem::path path_;
  std::vector<std::ostringstream> pages_;
};
void draw_report(const std::filesystem::path& path, const CameraConfig& camera,
                 const CalibrationResult& result, const Summary& summary,
                 const std::vector<FrameSummary>& frames,
                 const std::string& generated_at) {
  Pdf pdf(path);
  pdf.page();
  pdf.text(50, 800, 20, "Kalibr2 calibration report");
  pdf.text(50, 779, 10, "Generated: " + generated_at);
  pdf.text(50, 760, 10, "Camera topic: " + camera.topic);
  pdf.text(50, 744, 10, "Resolution: " + std::to_string(camera.width) + " x " + std::to_string(camera.height));
  pdf.text(50, 728, 10, "Model: " + camera.model + " / " + camera.distortion_model);

  pdf.text(50, 695, 13, "Extrinsics T_cam_imu");
  for (int i = 0; i < 4; ++i) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(8);
    row << "[ ";
    for (int j = 0; j < 4; ++j) row << std::setw(12) << result.T_cam_imu(i, j) << ' ';
    row << ']';
    pdf.text(65, 675 - 16 * i, 9, row.str());
  }

  pdf.text(50, 585, 13, "Timing");
  pdf.text(65, 565, 10, "timeshift_cam_imu: " + number(result.timeshift_cam_imu, 9) + " s");
  pdf.text(65, 549, 10, "timeshift_cam_imu: " + number(result.timeshift_cam_imu * 1000.0, 6) + " ms");

  pdf.text(50, 515, 13, "Reprojection error [px]");
  pdf.text(65, 495, 10, "RMSE: " + number(result.reprojection_rmse, 6));
  pdf.text(65, 479, 10, "Mean: " + number(summary.mean, 6));
  pdf.text(65, 463, 10, "Median: " + number(summary.median, 6));
  pdf.text(65, 447, 10, "P95: " + number(summary.p95, 6));
  pdf.text(65, 431, 10, "Max: " + number(summary.max, 6));
  pdf.text(65, 415, 10, "Camera frames / corner residuals: " + std::to_string(frames.size()) +
           " / " + std::to_string(summary.count));

  pdf.text(50, 378, 13, "Optimization");
  pdf.text(65, 358, 10, "Ceres iterations: " + std::to_string(result.iterations));
  pdf.text(65, 342, 10, "Initial cost: " + number(result.initial_cost, 3));
  pdf.text(65, 326, 10, "Final cost: " + number(result.final_cost, 3));

  pdf.text(50, 289, 13, "Biases and gravity");
  pdf.text(65, 269, 10, "Gyro bias: " + number(result.gyro_bias.x(), 8) + ", " +
           number(result.gyro_bias.y(), 8) + ", " + number(result.gyro_bias.z(), 8));
  pdf.text(65, 253, 10, "Accel bias: " + number(result.accel_bias.x(), 8) + ", " +
           number(result.accel_bias.y(), 8) + ", " + number(result.accel_bias.z(), 8));
  pdf.text(65, 237, 10, "Gravity: " + number(result.gravity.x(), 8) + ", " +
           number(result.gravity.y(), 8) + ", " + number(result.gravity.z(), 8));

  pdf.text(50, 195, 13, "Detailed files");
  pdf.text(65, 175, 10, "residuals.csv: one row per AprilGrid corner residual");
  pdf.text(65, 159, 10, "residual_summary.json: numeric summary and generated_at_utc");
  pdf.write();
}
} // namespace
void save_diagnostics_report(const std::string& directory, const CameraConfig& camera,
                             const CalibrationResult& result) {
  const auto root = std::filesystem::path(directory);
  const auto norms = residual_norms(result);
  const auto summary = summarize(norms);
  const auto frames = frame_summaries(result);
  const auto generated_at = utc_time_minute();
  write_csv(root / "residuals.csv", result);
  write_summary_json(root / "residual_summary.json", result, summary, frames, generated_at);
  draw_report(root / "calibration-report.pdf", camera, result, summary, frames, generated_at);
}
} // namespace kalibr2
