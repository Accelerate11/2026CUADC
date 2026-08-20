#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <cuadc_interfaces/msg/mission_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace cuadc_recon
{

struct EvidenceRecorderOptions
{
  std::filesystem::path directory;
  std::string image_topic{"/camera/camera/color/image_raw"};
  double minimum_interval_s{1.0};
  int jpeg_quality{90};
};

// Records reconnaissance evidence in the health/recorder process. It has no
// mission command, target, service, or action interface.
class ReconEvidenceRecorder
{
public:
  ReconEvidenceRecorder(rclcpp::Node & node, EvidenceRecorderOptions options);
  ReconEvidenceRecorder(const ReconEvidenceRecorder &) = delete;
  ReconEvidenceRecorder & operator=(const ReconEvidenceRecorder &) = delete;

  bool healthy() const noexcept {return healthy_;}
  std::uint64_t capture_count() const noexcept {return capture_count_;}
  const std::string & detail() const noexcept {return detail_;}
  const std::filesystem::path & session_directory() const noexcept
  {
    return session_directory_;
  }

private:
  using Clock = std::chrono::steady_clock;

  void mission_callback(const cuadc_interfaces::msg::MissionStatus::SharedPtr message);
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message);
  void set_fault(const std::string & detail);

  rclcpp::Node & node_;
  EvidenceRecorderOptions options_;
  bool recon_active_{false};
  bool healthy_{true};
  std::uint64_t capture_count_{0U};
  std::string mission_id_;
  std::string detail_{"ready"};
  std::filesystem::path session_directory_;
  std::filesystem::path manifest_path_;
  std::ofstream manifest_;
  Clock::time_point started_{Clock::now()};
  Clock::time_point last_capture_{};
  rclcpp::Subscription<cuadc_interfaces::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace cuadc_recon
