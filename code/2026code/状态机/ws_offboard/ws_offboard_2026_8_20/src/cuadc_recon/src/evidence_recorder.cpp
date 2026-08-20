#include <cuadc_recon/evidence_recorder.hpp>

#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace cuadc_recon
{
namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

}  // namespace

ReconEvidenceRecorder::ReconEvidenceRecorder(
  rclcpp::Node & node, EvidenceRecorderOptions options)
: node_(node), options_(std::move(options))
{
  if (options_.directory.empty()) {
    throw std::invalid_argument("recon evidence directory must not be empty");
  }
  if (options_.image_topic.empty()) {
    throw std::invalid_argument("recon image topic must not be empty");
  }
  if (!std::isfinite(options_.minimum_interval_s) || options_.minimum_interval_s <= 0.0) {
    throw std::invalid_argument("recon capture interval must be finite and positive");
  }
  if (options_.jpeg_quality < 1 || options_.jpeg_quality > 100) {
    throw std::invalid_argument("recon JPEG quality must be in [1, 100]");
  }

  try {
    const auto session_stamp = std::chrono::system_clock::now().time_since_epoch().count();
    session_directory_ = std::filesystem::absolute(options_.directory) /
      ("recon_" + std::to_string(session_stamp));
    std::filesystem::create_directories(session_directory_);
    manifest_path_ = session_directory_ / "manifest.jsonl";
    manifest_.open(manifest_path_, std::ios::out | std::ios::app);
    if (!manifest_.is_open()) {
      set_fault("cannot_open_recon_manifest");
    }
  } catch (const std::exception & error) {
    set_fault(std::string("cannot_initialize_recon_recorder:") + error.what());
  }

  mission_sub_ = node_.create_subscription<cuadc_interfaces::msg::MissionStatus>(
    "/cuadc/mission/status", rclcpp::QoS(10).reliable(),
    std::bind(&ReconEvidenceRecorder::mission_callback, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_.get_logger(), "Recon evidence recorder: topic=%s directory=%s",
    options_.image_topic.c_str(), session_directory_.c_str());
}

void ReconEvidenceRecorder::mission_callback(
  const cuadc_interfaces::msg::MissionStatus::SharedPtr message)
{
  const bool next_active = message->active &&
    message->state == cuadc_interfaces::msg::MissionStatus::HAZARD_WAYPOINT_RECON;
  if (next_active && !recon_active_) {
    // Allow the first frame after entering reconnaissance to be captured.
    last_capture_ = Clock::time_point{};
    if (healthy_) {
      detail_ = "hazard_recon_recording";
      image_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
        options_.image_topic, rclcpp::SensorDataQoS(),
        std::bind(&ReconEvidenceRecorder::image_callback, this, std::placeholders::_1));
      RCLCPP_INFO(node_.get_logger(), "Recon evidence capture enabled");
    } else {
      RCLCPP_ERROR(
        node_.get_logger(), "Recon evidence capture unavailable: %s", detail_.c_str());
    }
  } else if (!next_active && recon_active_) {
    image_sub_.reset();
    detail_ = healthy_ ? "hazard_recon_capture_complete" : detail_;
    RCLCPP_INFO(
      node_.get_logger(), "Recon evidence capture stopped after %llu JPEG files",
      static_cast<unsigned long long>(capture_count_));
  }
  mission_id_ = message->mission_id;
  recon_active_ = next_active;
}

void ReconEvidenceRecorder::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr message)
{
  if (!recon_active_ || !healthy_) {
    return;
  }
  const auto now = Clock::now();
  if (last_capture_ != Clock::time_point{} &&
    std::chrono::duration<double>(now - last_capture_).count() < options_.minimum_interval_s)
  {
    return;
  }

  try {
    const auto converted = cv_bridge::toCvShare(
      message, sensor_msgs::image_encodings::BGR8);
    if (converted->image.empty()) {
      throw std::runtime_error("received empty RGB image");
    }
    const std::uint64_t sequence = capture_count_ + 1U;
    std::ostringstream filename;
    filename << "recon_" << std::setw(6) << std::setfill('0') << sequence << "_" <<
      message->header.stamp.sec << "_" << message->header.stamp.nanosec << ".jpg";
    const auto final_path = session_directory_ / filename.str();
    const auto temporary_path = session_directory_ /
      (final_path.stem().string() + ".tmp.jpg");
    const std::vector<int> jpeg_options{
      cv::IMWRITE_JPEG_QUALITY, options_.jpeg_quality};
    if (!cv::imwrite(temporary_path.string(), converted->image, jpeg_options)) {
      throw std::runtime_error("cv::imwrite returned false");
    }
    std::error_code rename_error;
    std::filesystem::rename(temporary_path, final_path, rename_error);
    if (rename_error) {
      std::error_code cleanup_error;
      std::filesystem::remove(temporary_path, cleanup_error);
      throw std::runtime_error("cannot commit JPEG: " + rename_error.message());
    }

    const double steady_s = std::chrono::duration<double>(now - started_).count();
    manifest_ << "{\"sequence\":" << sequence <<
      ",\"steady_s\":" << std::fixed << std::setprecision(3) << steady_s <<
      ",\"ros_stamp_sec\":" << message->header.stamp.sec <<
      ",\"ros_stamp_nanosec\":" << message->header.stamp.nanosec <<
      ",\"mission_id\":\"" << json_escape(mission_id_) << "\"" <<
      ",\"frame_id\":\"" << json_escape(message->header.frame_id) << "\"" <<
      ",\"width\":" << message->width <<
      ",\"height\":" << message->height <<
      ",\"image\":\"" << json_escape(filename.str()) << "\"}\n";
    manifest_.flush();
    if (!manifest_) {
      throw std::runtime_error("cannot append reconnaissance manifest");
    }
    capture_count_ = sequence;
    last_capture_ = now;
    detail_ = "hazard_recon_recording";
  } catch (const std::exception & error) {
    set_fault(std::string("recon_capture_failed:") + error.what());
  }
}

void ReconEvidenceRecorder::set_fault(const std::string & detail)
{
  healthy_ = false;
  image_sub_.reset();
  detail_ = detail;
  RCLCPP_ERROR(node_.get_logger(), "%s", detail_.c_str());
}

}  // namespace cuadc_recon
