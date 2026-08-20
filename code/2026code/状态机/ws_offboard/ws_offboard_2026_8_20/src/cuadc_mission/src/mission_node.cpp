/*
 * CUADC 2026 complete autonomous mission.
 *
 * Pilot confirmation:
 *   WAIT_FCU -> WAIT_GUIDED -> LOCK_FRAME -> PRESTREAM -> WAIT_ARM
 *
 * Autonomous sequence:
 *   TAKEOFF -> SEARCH -> ALIGN -> RELEASE -> ALIGN -> RELEASE
 *   -> RECON_CLIMB -> RECON_SURVEY -> RETURN_CLIMB
 *   -> RETURN_HOME -> LAND -> DISARM -> DONE
 *
 * Hazard recognition is deliberately not part of this node. RECON_SURVEY only
 * traverses public field waypoints and has no hazard-image subscription.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <future>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cuadc_interfaces/action/release_payload.hpp>
#include <cuadc_interfaces/msg/mission_status.hpp>
#include <cuadc_interfaces/msg/payload_status.hpp>
#include <cuadc_interfaces/msg/safety_status.hpp>
#include <cuadc_interfaces/srv/payload_command.hpp>
#include <cuadc_recon/route_planner.hpp>
#include <cuadc_vehicle/vehicle_adapter.hpp>

using namespace std::chrono_literals;

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kVersion = "cuadc-flight-mission-2026-08-20-v13";
constexpr const char * kRealRuntimeGate =
  "CUADC_FLIGHT_LAUNCH_PREFLIGHT_PASSED";

double normalize_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double normalize_degrees(double value)
{
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

double distance_xy(const Point3 & lhs, const Point3 & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double distance_xyz(const Point3 & lhs, const Point3 & rhs)
{
  return std::hypot(distance_xy(lhs, rhs), lhs.z - rhs.z);
}

struct HeadingSample
{
  SteadyTimePoint stamp;
  double yaw = 0.0;
};

struct NavigationSample
{
  rclcpp::Time stamp;
  Point3 position;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct Segment
{
  Point3 start;
  Point3 end;
  SteadyTimePoint start_time;
  double duration_s = 1.0;
};

struct BucketTrack
{
  std::size_t id = 0U;
  Point3 local;
  Point3 body;
  double diameter = 0.0;
  double confidence = 0.0;
  double position_deviation = 0.0;
  double diameter_deviation = 0.0;
  std::size_t confirmations = 0U;
  std::deque<double> diameter_samples;
  rclcpp::Time stamp;
  SteadyTimePoint arrival;
  bool frozen_memory = false;
};

enum class State
{
  WAIT_FCU,
  WAIT_GUIDED,
  LOCK_FRAME,
  PRESTREAM,
  WAIT_ARM,
  TAKEOFF,
  SEARCH,
  ALIGN,
  RELEASE,
  RECON_CLIMB,
  RECON_SURVEY,
  RETURN_CLIMB,
  RETURN_HOME,
  LAND,
  DISARM,
  DONE,
  PILOT_OVERRIDE,
  ABORT
};

Point3 vector3_at(
  const std::vector<double> & values, std::size_t index, const Point3 & fallback)
{
  const std::size_t offset = index * 3U;
  if (offset + 2U >= values.size()) {
    return fallback;
  }
  return Point3{values[offset], values[offset + 1U], values[offset + 2U]};
}

double median(std::deque<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

double median_absolute_deviation(
  const std::deque<double> & values, double center)
{
  std::deque<double> deviations;
  for (const double value : values) {
    deviations.push_back(std::abs(value - center));
  }
  return median(std::move(deviations));
}

}  // namespace

class MissionNode final : public rclcpp::Node
{
public:
  using ReleasePayload = cuadc_interfaces::action::ReleasePayload;
  using ReleaseGoalHandle = rclcpp_action::ClientGoalHandle<ReleasePayload>;
  using PayloadCommand = cuadc_interfaces::srv::PayloadCommand;
  using PayloadStatus = cuadc_interfaces::msg::PayloadStatus;
  using SafetyStatus = cuadc_interfaces::msg::SafetyStatus;
  using MissionStatus = cuadc_interfaces::msg::MissionStatus;

  MissionNode()
  : Node("mission_node")
  {
    declare_parameters();
    load_parameters();

    vehicle_ = std::make_unique<cuadc_vehicle::VehicleAdapter>(
      *this,
      cuadc_vehicle::VehicleCallbacks{
        std::bind(
          &MissionNode::state_callback, this,
          std::placeholders::_1),
        std::bind(
          &MissionNode::odom_callback, this,
          std::placeholders::_1),
        std::bind(
          &MissionNode::extended_state_callback, this,
          std::placeholders::_1),
        std::bind(
          &MissionNode::compass_callback, this,
          std::placeholders::_1)});
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MissionNode::bucket_callback, this, std::placeholders::_1));

    payload_command_client_ =
      create_client<PayloadCommand>("/cuadc/payload/command");
    payload_release_client_ = rclcpp_action::create_client<ReleasePayload>(
      this, "/cuadc/payload/release");
    const auto durable_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    payload_status_sub_ = create_subscription<PayloadStatus>(
      "/cuadc/payload/status", durable_qos,
      std::bind(
        &MissionNode::payload_status_callback, this,
        std::placeholders::_1));
    payload_safety_sub_ = create_subscription<SafetyStatus>(
      "/cuadc/payload/safety_status", durable_qos,
      std::bind(
        &MissionNode::payload_safety_status_callback, this,
        std::placeholders::_1));
    safety_status_sub_ = create_subscription<SafetyStatus>(
      "/cuadc/safety/status", durable_qos,
      std::bind(
        &MissionNode::safety_status_callback, this,
        std::placeholders::_1));
    mission_status_pub_ = create_publisher<MissionStatus>(
      "/cuadc/mission/status", durable_qos);

    // ROS time is reserved for capture stamps and navigation interpolation.
    // Every elapsed-duration, watchdog, dwell, actuator deadline, and
    // trajectory phase uses steady time and survives FCU clock correction.
    const SteadyTimePoint current_arrival = SteadyClock::now();
    last_odom_arrival_ = current_arrival;
    last_compass_arrival_ = current_arrival;
    last_extended_state_arrival_ = current_arrival;
    last_safety_status_arrival_ = current_arrival;
    last_vision_message_arrival_ = current_arrival;
    last_aligned_vision_arrival_ = current_arrival;
    search_vision_acquire_start_arrival_ = current_arrival;
    state_enter_time_ = current_arrival;
    mission_start_time_ = current_arrival;
    last_request_time_ = current_arrival - std::chrono::seconds(2);
    last_status_time_ = current_arrival;
    last_mission_status_time_ = current_arrival - std::chrono::seconds(1);
    takeoff_request_time_ = current_arrival;
    land_request_time_ = current_arrival;
    arm_request_time_ = current_arrival;
    payload_initialization_sent_time_ = current_arrival;
    release_action_sent_time_ = current_arrival;
    segment_.start_time = current_arrival;
    effective_search_lane_count_ = resolved_search_lane_count();
    timer_ = create_wall_timer(50ms, std::bind(&MissionNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CUADC mission ready [%s]: flight_enable=%s payload_backend=action "
      "arm=%s heights(takeoff/search/align/release)=%.2f/%.2f/%.2f/%.2f m "
      "search_lanes=%d(%s) recon_lanes=%d expected_recon_waypoints=%d",
      kVersion, flight_enable_ ? "true" : "false",
      auto_arm_on_guided_ ? "automatic" : "manual",
      takeoff_alt_m_, search_alt_m_, coarse_alt_m_, fine_alt_m_,
      effective_search_lane_count_, search_lane_count_ == 0 ? "auto" : "explicit",
      recon_lane_count_, recon_lane_count_ * 2);
  }

private:
  void declare_parameters()
  {
    declare_parameter<bool>("flight_enable", false);
    declare_parameter<std::string>("config_bundle_sha256", "");
    declare_parameter<int64_t>("mission_epoch", 0);
    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");
    declare_parameter<bool>("lock_mission_yaw_to_initial_heading", true);
    declare_parameter<bool>("yaw_to_target", false);
    declare_parameter<bool>("auto_arm_on_guided", false);
    declare_parameter<double>("aircraft_max_horizontal_speed_m_s", 4.5);
    declare_parameter<double>("aircraft_max_drop_speed_m_s", 0.8);
    declare_parameter<double>("aircraft_max_mission_altitude_m", 6.0);
    declare_parameter<double>("aircraft_max_release_tilt_deg", 5.0);

    declare_parameter<double>("vision_heartbeat_timeout_s", 1.5);
    declare_parameter<double>("vision_max_pipeline_delay_s", 1.5);
    declare_parameter<double>("vision_future_tolerance_s", 0.05);
    declare_parameter<double>("vision_transform_tolerance_s", 0.10);
    declare_parameter<double>("nav_interpolation_max_gap_s", 0.20);
    declare_parameter<double>("odom_history_s", 5.0);
    declare_parameter<int>("vision_min_messages_before_takeoff", 3);
    declare_parameter<int>("search_vision_acquire_min_frames", 3);
    declare_parameter<double>("search_vision_acquire_timeout_s", 5.0);

    declare_parameter<double>("takeoff_alt_m", 4.0);
    declare_parameter<bool>("enforce_final_drop_heights", true);
    declare_parameter<double>("search_alt_m", 2.0);
    declare_parameter<double>("coarse_alt_m", 1.8);
    declare_parameter<double>("fine_alt_m", 1.8);
    declare_parameter<double>("transit_speed_m_s", 4.0);
    declare_parameter<double>("search_speed_m_s", 2.0);
    declare_parameter<double>("recon_speed_m_s", 3.0);
    declare_parameter<double>("drop_approach_speed_m_s", 0.7);
    declare_parameter<double>("release_positioning_speed_m_s", 0.25);
    declare_parameter<double>("min_segment_time_s", 1.0);
    declare_parameter<double>("waypoint_accept_radius_m", 0.40);

    declare_parameter<double>("drop_area_near_edge_field_x_m", 30.0);
    declare_parameter<double>("drop_area_length_field_x_m", 5.0);
    declare_parameter<double>("drop_area_width_field_y_m", 8.0);
    declare_parameter<int>("search_lane_count", 0);
    declare_parameter<double>("cross_track_camera_fov_deg", 42.0);
    declare_parameter<double>("search_lane_overlap_ratio", 0.30);
    declare_parameter<double>("search_edge_margin_m", 0.35);
    declare_parameter<double>("search_cross_margin_m", 0.55);
    declare_parameter<int>("max_search_passes", 3);

    declare_parameter<double>("recon_area_center_field_x_m", 55.0);
    declare_parameter<double>("recon_area_length_field_x_m", 5.0);
    declare_parameter<double>("recon_area_width_field_y_m", 8.0);
    declare_parameter<double>("recon_hover_alt_m", 3.5);
    declare_parameter<int>("recon_lane_count", 3);
    declare_parameter<double>("recon_edge_margin_m", 0.35);
    declare_parameter<double>("recon_cross_margin_m", 0.55);
    declare_parameter<double>("recon_waypoint_hold_s", 1.0);

    declare_parameter<int>("bucket_required_count", 3);
    declare_parameter<int>("bucket_min_confirmations", 5);
    declare_parameter<double>("track_gate_m", 0.45);
    declare_parameter<double>("diameter_track_gate_m", 0.08);
    declare_parameter<double>("bucket_track_max_gap_s", 0.60);
    declare_parameter<double>("bucket_selection_max_age_s", 45.0);
    declare_parameter<double>("bucket_distinct_min_separation_m", 0.20);
    declare_parameter<double>("bucket_distinct_min_diameter_m", 0.025);
    declare_parameter<double>("bucket_target_ranking_stable_s", 0.80);
    declare_parameter<int>("bucket_diameter_filter_window", 9);
    declare_parameter<double>("known_bucket_memory_s", 120.0);
    declare_parameter<double>("released_bucket_exclusion_m", 0.25);
    declare_parameter<double>("bucket_position_filter_alpha", 0.25);
    declare_parameter<double>("bucket_body_filter_alpha", 1.0);
    declare_parameter<double>("bucket_diameter_filter_alpha", 0.20);
    declare_parameter<double>("bucket_confidence_filter_alpha", 0.30);
    declare_parameter<double>("bucket_max_position_deviation_m", 0.25);
    declare_parameter<double>("bucket_max_diameter_deviation_m", 0.035);
    declare_parameter<double>("bucket_min_track_confidence", 0.25);

    declare_parameter<double>("coarse_error_m", 0.15);
    declare_parameter<double>("fine_error_m", 0.08);
    declare_parameter<double>("coarse_stable_s", 0.8);
    declare_parameter<double>("fine_stable_s", 0.8);
    declare_parameter<double>("alignment_timeout_s", 12.0);
    declare_parameter<double>("detection_timeout_s", 3.0);
    declare_parameter<double>("target_guidance_max_age_s", 0.5);
    declare_parameter<double>("recover_hold_s", 0.5);

    declare_parameter<double>("direct_release_xy_tolerance_m", 0.10);
    declare_parameter<double>("direct_release_height_tolerance_m", 0.10);
    declare_parameter<double>("direct_release_stable_s", 0.80);
    declare_parameter<double>("release_pose_refresh_reset_m", 0.03);
    declare_parameter<double>("direct_release_timeout_s", 6.0);
    declare_parameter<double>("direct_release_max_horizontal_speed_m_s", 0.08);
    declare_parameter<double>("direct_release_max_vertical_speed_m_s", 0.05);
    declare_parameter<double>("direct_release_max_tilt_deg", 5.0);
    declare_parameter<double>("direct_release_max_angular_rate_deg_s", 5.0);
    declare_parameter<double>("release_max_yaw_error_deg", 5.0);

    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("return_climb_timeout_s", 30.0);
    declare_parameter<double>("return_home_timeout_s", 120.0);
    declare_parameter<double>("land_timeout_s", 120.0);
    declare_parameter<double>("disarm_timeout_s", 20.0);
    declare_parameter<double>("odom_timeout_s", 1.5);
    declare_parameter<double>("compass_timeout_s", 1.0);
    declare_parameter<double>("extended_state_timeout_s", 2.5);
    declare_parameter<double>("safety_status_timeout_s", 2.0);
    declare_parameter<double>("landing_confirm_stable_s", 1.5);
    declare_parameter<double>("landing_max_relative_altitude_m", 0.30);
    declare_parameter<double>("landing_max_horizontal_speed_m_s", 0.20);
    declare_parameter<double>("landing_max_vertical_speed_m_s", 0.15);
    declare_parameter<double>("heading_lock_stability_s", 2.0);
    declare_parameter<double>("heading_lock_max_variation_deg", 2.0);
    declare_parameter<double>("position_lock_stability_s", 2.0);
    declare_parameter<double>("stationary_speed_max_m_s", 0.15);
    declare_parameter<double>("service_ack_timeout_s", 3.0);

    declare_parameter<int>("payload_count", 2);
    declare_parameter<std::vector<double>>(
      "payload_release_offsets_body_m",
      std::vector<double>{0.026, -0.065, -0.32, -0.026, 0.065, -0.32});
    declare_parameter<double>("payload_initialization_timeout_s", 8.0);
    declare_parameter<double>("payload_action_timeout_s", 12.0);
  }

  void load_parameters()
  {
    flight_enable_ = get_parameter("flight_enable").as_bool();
    config_bundle_sha256_ =
      get_parameter("config_bundle_sha256").as_string();
    mission_epoch_ = static_cast<std::uint64_t>(
      std::max<int64_t>(0, get_parameter("mission_epoch").as_int()));
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();
    lock_initial_heading_ =
      get_parameter("lock_mission_yaw_to_initial_heading").as_bool();
    yaw_to_target_ = get_parameter("yaw_to_target").as_bool();
    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();
    aircraft_max_horizontal_speed_m_s_ =
      get_parameter("aircraft_max_horizontal_speed_m_s").as_double();
    aircraft_max_drop_speed_m_s_ =
      get_parameter("aircraft_max_drop_speed_m_s").as_double();
    aircraft_max_mission_altitude_m_ =
      get_parameter("aircraft_max_mission_altitude_m").as_double();
    aircraft_max_release_tilt_rad_ =
      get_parameter("aircraft_max_release_tilt_deg").as_double() * kPi / 180.0;

    vision_heartbeat_timeout_s_ =
      std::max(0.3, get_parameter("vision_heartbeat_timeout_s").as_double());
    vision_max_pipeline_delay_s_ =
      std::max(0.05, get_parameter("vision_max_pipeline_delay_s").as_double());
    vision_future_tolerance_s_ =
      std::max(0.0, get_parameter("vision_future_tolerance_s").as_double());
    vision_transform_tolerance_s_ =
      std::max(0.01, get_parameter("vision_transform_tolerance_s").as_double());
    nav_interpolation_max_gap_s_ =
      std::max(0.02, get_parameter("nav_interpolation_max_gap_s").as_double());
    odom_history_s_ =
      std::max(1.0, get_parameter("odom_history_s").as_double());
    vision_min_messages_before_takeoff_ = std::max(
      1, static_cast<int>(
        get_parameter("vision_min_messages_before_takeoff").as_int()));
    search_vision_acquire_min_frames_ = std::max(
      3, static_cast<int>(
        get_parameter("search_vision_acquire_min_frames").as_int()));
    search_vision_acquire_timeout_s_ = std::max(
      1.0, get_parameter("search_vision_acquire_timeout_s").as_double());

    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    if (get_parameter("enforce_final_drop_heights").as_bool()) {
      search_alt_m_ = 2.0;
      coarse_alt_m_ = 1.8;
      fine_alt_m_ = 1.8;
    } else {
      search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
      coarse_alt_m_ = std::max(1.0, get_parameter("coarse_alt_m").as_double());
      fine_alt_m_ = std::max(1.0, get_parameter("fine_alt_m").as_double());
    }
    transit_speed_m_s_ =
      std::max(0.3, get_parameter("transit_speed_m_s").as_double());
    search_speed_m_s_ =
      std::max(0.3, get_parameter("search_speed_m_s").as_double());
    recon_speed_m_s_ =
      std::max(0.3, get_parameter("recon_speed_m_s").as_double());
    drop_approach_speed_m_s_ =
      std::max(0.1, get_parameter("drop_approach_speed_m_s").as_double());
    release_positioning_speed_m_s_ = std::max(
      0.05, get_parameter("release_positioning_speed_m_s").as_double());
    min_segment_s_ =
      std::max(0.3, get_parameter("min_segment_time_s").as_double());
    accept_radius_m_ =
      std::max(0.1, get_parameter("waypoint_accept_radius_m").as_double());

    drop_near_x_m_ =
      get_parameter("drop_area_near_edge_field_x_m").as_double();
    drop_length_x_m_ =
      std::max(1.0, get_parameter("drop_area_length_field_x_m").as_double());
    drop_width_y_m_ =
      std::max(1.0, get_parameter("drop_area_width_field_y_m").as_double());
    search_lane_count_ = std::max(
      0, static_cast<int>(get_parameter("search_lane_count").as_int()));
    cross_track_camera_fov_rad_ = std::clamp(
      get_parameter("cross_track_camera_fov_deg").as_double(), 5.0, 170.0) *
      kPi / 180.0;
    search_lane_overlap_ratio_ = std::clamp(
      get_parameter("search_lane_overlap_ratio").as_double(), 0.0, 0.90);
    search_edge_margin_m_ =
      std::max(0.0, get_parameter("search_edge_margin_m").as_double());
    search_cross_margin_m_ =
      std::max(0.0, get_parameter("search_cross_margin_m").as_double());
    max_search_passes_ = std::max(
      1, static_cast<int>(get_parameter("max_search_passes").as_int()));

    recon_center_x_m_ =
      get_parameter("recon_area_center_field_x_m").as_double();
    recon_length_x_m_ =
      std::max(1.0, get_parameter("recon_area_length_field_x_m").as_double());
    recon_width_y_m_ =
      std::max(1.0, get_parameter("recon_area_width_field_y_m").as_double());
    recon_alt_m_ =
      std::max(1.0, get_parameter("recon_hover_alt_m").as_double());
    recon_lane_count_ = std::max(
      1, static_cast<int>(get_parameter("recon_lane_count").as_int()));
    recon_edge_margin_m_ =
      std::max(0.0, get_parameter("recon_edge_margin_m").as_double());
    recon_cross_margin_m_ =
      std::max(0.0, get_parameter("recon_cross_margin_m").as_double());
    recon_waypoint_hold_s_ =
      std::max(0.0, get_parameter("recon_waypoint_hold_s").as_double());

    required_bucket_count_ = std::max(
      3, static_cast<int>(get_parameter("bucket_required_count").as_int()));
    min_confirmations_ = std::max(
      1, static_cast<int>(get_parameter("bucket_min_confirmations").as_int()));
    track_gate_m_ =
      std::max(0.1, get_parameter("track_gate_m").as_double());
    diameter_track_gate_m_ =
      std::max(0.01, get_parameter("diameter_track_gate_m").as_double());
    track_max_gap_s_ =
      std::max(0.1, get_parameter("bucket_track_max_gap_s").as_double());
    selection_max_age_s_ =
      std::max(0.1, get_parameter("bucket_selection_max_age_s").as_double());
    distinct_min_separation_m_ = std::max(
      0.05, get_parameter("bucket_distinct_min_separation_m").as_double());
    distinct_min_diameter_m_ = std::max(
      0.005, get_parameter("bucket_distinct_min_diameter_m").as_double());
    ranking_stable_s_ = std::max(
      0.1, get_parameter("bucket_target_ranking_stable_s").as_double());
    diameter_filter_window_ = std::max(
      3, static_cast<int>(
        get_parameter("bucket_diameter_filter_window").as_int()));
    if (diameter_filter_window_ % 2 == 0) {
      ++diameter_filter_window_;
    }
    known_memory_s_ =
      std::max(2.0, get_parameter("known_bucket_memory_s").as_double());
    released_exclusion_m_ =
      std::max(0.05, get_parameter("released_bucket_exclusion_m").as_double());
    position_filter_alpha_ = std::clamp(
      get_parameter("bucket_position_filter_alpha").as_double(), 0.01, 1.0);
    body_filter_alpha_ = std::clamp(
      get_parameter("bucket_body_filter_alpha").as_double(), 0.01, 1.0);
    diameter_filter_alpha_ = std::clamp(
      get_parameter("bucket_diameter_filter_alpha").as_double(), 0.01, 1.0);
    confidence_filter_alpha_ = std::clamp(
      get_parameter("bucket_confidence_filter_alpha").as_double(), 0.01, 1.0);
    max_position_deviation_m_ = std::max(
      0.01, get_parameter("bucket_max_position_deviation_m").as_double());
    max_diameter_deviation_m_ = std::max(
      0.001, get_parameter("bucket_max_diameter_deviation_m").as_double());
    min_track_confidence_ = std::clamp(
      get_parameter("bucket_min_track_confidence").as_double(), 0.0, 1.0);

    coarse_error_m_ =
      std::max(0.05, get_parameter("coarse_error_m").as_double());
    fine_error_m_ =
      std::max(0.03, get_parameter("fine_error_m").as_double());
    coarse_stable_s_ =
      std::max(0.1, get_parameter("coarse_stable_s").as_double());
    fine_stable_s_ =
      std::max(0.1, get_parameter("fine_stable_s").as_double());
    alignment_timeout_s_ =
      std::max(1.0, get_parameter("alignment_timeout_s").as_double());
    detection_timeout_s_ =
      std::max(0.2, get_parameter("detection_timeout_s").as_double());
    target_guidance_max_age_s_ = std::clamp(
      get_parameter("target_guidance_max_age_s").as_double(),
      0.05, detection_timeout_s_);
    payload_transition_hold_s_ =
      std::max(0.0, get_parameter("recover_hold_s").as_double());

    direct_release_xy_tolerance_m_ = std::max(
      0.03, get_parameter("direct_release_xy_tolerance_m").as_double());
    direct_release_height_tolerance_m_ = std::max(
      0.03, get_parameter("direct_release_height_tolerance_m").as_double());
    direct_release_stable_s_ =
      std::max(0.2, get_parameter("direct_release_stable_s").as_double());
    release_pose_refresh_reset_m_ = std::clamp(
      get_parameter("release_pose_refresh_reset_m").as_double(), 0.005, 0.15);
    direct_release_timeout_s_ =
      std::max(2.0, get_parameter("direct_release_timeout_s").as_double());
    direct_release_max_horizontal_speed_m_s_ = std::max(
      0.03,
      get_parameter("direct_release_max_horizontal_speed_m_s").as_double());
    direct_release_max_vertical_speed_m_s_ = std::max(
      0.02, get_parameter("direct_release_max_vertical_speed_m_s").as_double());
    direct_release_max_tilt_rad_ =
      std::max(1.0, get_parameter("direct_release_max_tilt_deg").as_double()) *
      kPi / 180.0;
    direct_release_max_angular_rate_rad_s_ = std::max(
      1.0,
      get_parameter("direct_release_max_angular_rate_deg_s").as_double()) *
      kPi / 180.0;
    release_max_yaw_error_rad_ =
      std::max(0.5, get_parameter("release_max_yaw_error_deg").as_double()) *
      kPi / 180.0;

    prestream_s_ =
      std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ =
      std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ =
      std::max(30.0, get_parameter("mission_timeout_s").as_double());
    return_climb_timeout_s_ = std::clamp(
      get_parameter("return_climb_timeout_s").as_double(), 5.0, 300.0);
    return_home_timeout_s_ = std::clamp(
      get_parameter("return_home_timeout_s").as_double(), 10.0, 600.0);
    land_timeout_s_ = std::clamp(
      get_parameter("land_timeout_s").as_double(), 10.0, 600.0);
    disarm_timeout_s_ = std::clamp(
      get_parameter("disarm_timeout_s").as_double(), 5.0, 120.0);
    odom_timeout_s_ =
      std::max(0.2, get_parameter("odom_timeout_s").as_double());
    compass_timeout_s_ =
      std::max(0.2, get_parameter("compass_timeout_s").as_double());
    extended_state_timeout_s_ =
      std::max(0.2, get_parameter("extended_state_timeout_s").as_double());
    safety_status_timeout_s_ =
      std::max(0.5, get_parameter("safety_status_timeout_s").as_double());
    landing_confirm_stable_s_ =
      std::max(0.2, get_parameter("landing_confirm_stable_s").as_double());
    landing_max_relative_altitude_m_ = std::max(
      0.10, get_parameter("landing_max_relative_altitude_m").as_double());
    landing_max_horizontal_speed_m_s_ = std::max(
      0.05, get_parameter("landing_max_horizontal_speed_m_s").as_double());
    landing_max_vertical_speed_m_s_ = std::max(
      0.05, get_parameter("landing_max_vertical_speed_m_s").as_double());
    heading_stability_s_ =
      std::max(0.5, get_parameter("heading_lock_stability_s").as_double());
    heading_max_variation_rad_ =
      std::max(
      0.1, get_parameter("heading_lock_max_variation_deg").as_double()) *
      kPi / 180.0;
    position_stability_s_ =
      std::max(0.5, get_parameter("position_lock_stability_s").as_double());
    stationary_speed_m_s_ =
      std::max(0.02, get_parameter("stationary_speed_max_m_s").as_double());
    service_ack_timeout_s_ =
      std::max(0.5, get_parameter("service_ack_timeout_s").as_double());

    payload_count_ =
      std::max(1, static_cast<int>(get_parameter("payload_count").as_int()));
    release_offsets_ =
      get_parameter("payload_release_offsets_body_m").as_double_array();
    payload_initialization_timeout_s_ = std::clamp(
      get_parameter("payload_initialization_timeout_s").as_double(),
      2.0, 30.0);
    payload_action_timeout_s_ = std::clamp(
      get_parameter("payload_action_timeout_s").as_double(), 3.0, 30.0);

    const bool release_offsets_valid =
      release_offsets_.size() >= static_cast<std::size_t>(payload_count_ * 3) &&
      std::all_of(
      release_offsets_.begin(), release_offsets_.end(),
      [](double value) {return std::isfinite(value);});
    const bool field_geometry_valid =
      drop_length_x_m_ > 2.0 * search_edge_margin_m_ &&
      drop_width_y_m_ > 2.0 * search_cross_margin_m_ &&
      recon_length_x_m_ > 2.0 * recon_edge_margin_m_ &&
      recon_width_y_m_ > 2.0 * recon_cross_margin_m_;
    const bool aircraft_limits_valid =
      std::isfinite(aircraft_max_horizontal_speed_m_s_) &&
      std::isfinite(aircraft_max_drop_speed_m_s_) &&
      std::isfinite(aircraft_max_mission_altitude_m_) &&
      std::isfinite(aircraft_max_release_tilt_rad_) &&
      aircraft_max_horizontal_speed_m_s_ >= 0.3 &&
      aircraft_max_drop_speed_m_s_ >= 0.1 &&
      aircraft_max_mission_altitude_m_ >= 1.0 &&
      aircraft_max_release_tilt_rad_ > 0.0 &&
      transit_speed_m_s_ <= aircraft_max_horizontal_speed_m_s_ &&
      search_speed_m_s_ <= aircraft_max_horizontal_speed_m_s_ &&
      recon_speed_m_s_ <= aircraft_max_horizontal_speed_m_s_ &&
      drop_approach_speed_m_s_ <= aircraft_max_drop_speed_m_s_ &&
      release_positioning_speed_m_s_ <= aircraft_max_drop_speed_m_s_ &&
      std::max({takeoff_alt_m_, search_alt_m_, coarse_alt_m_, fine_alt_m_, recon_alt_m_}) <=
      aircraft_max_mission_altitude_m_ &&
      direct_release_max_tilt_rad_ <= aircraft_max_release_tilt_rad_;
    const char * runtime_gate = std::getenv("CUADC_REAL_RUNTIME_GATE");
    const char * runtime_bundle = std::getenv("CUADC_CONFIG_BUNDLE_SHA256");
    const bool bundle_format_valid = std::regex_match(
      config_bundle_sha256_, std::regex("^[0-9a-f]{64}$"));
    const bool real_runtime_gate_valid =
      runtime_gate != nullptr && std::string(runtime_gate) == kRealRuntimeGate &&
      runtime_bundle != nullptr &&
      std::string(runtime_bundle) == config_bundle_sha256_ &&
      bundle_format_valid;

    config_valid_ =
      flight_enable_ && lock_initial_heading_ && !yaw_to_target_ &&
      payload_count_ == 2 && release_offsets_valid &&
      field_geometry_valid && aircraft_limits_valid &&
      recon_lane_count_ == 3 && mission_epoch_ > 0U &&
      real_runtime_gate_valid;
    if (flight_enable_ && !real_runtime_gate_valid) {
      RCLCPP_ERROR(
        get_logger(),
        "Real-flight runtime gate or signed config bundle SHA is invalid; "
        "start only through cuadc_bringup/flight.launch.py");
    }
    if (!config_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Safety configuration invalid: flight_enable must be true, yaw must be "
        "fixed, payload_count must be two, recon must be exactly three lanes, "
        "mission_epoch must be nonzero, and real flight must pass the signed "
        "cuadc_bringup preflight gate");
    }
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr message)
  {
    const bool was_guided = guided_active_;
    const bool was_armed = fcu_state_.armed;
    fcu_state_ = *message;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (guided_active_ && !was_guided) {
      RCLCPP_WARN(
        get_logger(),
        "GUIDED detected; keep the aircraft stationary while frame gates lock");
    }
    if (!guided_active_ && was_guided && guided_required_state(state_)) {
      publish_setpoint_ = false;
      vehicle_->revoke_for_pilot();
      if (state_ == State::RELEASE &&
        (release_command_pending_ || release_open_ ||
        stow_command_pending_))
      {
        release_abort_requested_ = true;
        request_payload_action_cancel(
          "pilot left GUIDED during payload operation");
      }
      mark_failure("Pilot left GUIDED during autonomous flight");
      enter(State::PILOT_OVERRIDE);
      return;
    }

    // LAND is an expected FCU mode after the node requests autonomous
    // landing. Any other armed mode in LAND/DISARM is a pilot or failsafe
    // handoff; stop issuing LAND/DISARM service requests immediately.
    const bool landing_or_disarm =
      state_ == State::LAND || state_ == State::DISARM;
    const bool automatic_landing_mode =
      fcu_state_.mode == "GUIDED" || fcu_state_.mode == "LAND";
    if (fcu_state_.armed && landing_or_disarm &&
      (!fcu_state_.connected || !automatic_landing_mode))
    {
      publish_setpoint_ = false;
      vehicle_->revoke_for_pilot();
      mark_failure(
        "Pilot/failsafe handoff observed during LAND/DISARM; node control stopped");
      enter(State::PILOT_OVERRIDE);
      return;
    }

    if (fcu_state_.armed && !was_armed && state_ == State::WAIT_ARM) {
      RCLCPP_INFO(get_logger(), "FCU armed; starting autonomous takeoff");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const rclcpp::Time receipt_time = now();
    const SteadyTimePoint receipt_arrival = SteadyClock::now();
    const Point3 sample_position{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    double sample_roll = current_roll_;
    double sample_pitch = current_pitch_;
    double sample_yaw = current_vehicle_yaw_;
    const auto & quaternion = message->pose.pose.orientation;
    const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    if (norm > 1.0e-6) {
      const double qx = quaternion.x / norm;
      const double qy = quaternion.y / norm;
      const double qz = quaternion.z / norm;
      const double qw = quaternion.w / norm;
      sample_roll = std::atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy));
      sample_pitch = std::asin(std::clamp(
        2.0 * (qw * qy - qz * qx), -1.0, 1.0));
      sample_yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
    }

    position_ = sample_position;
    current_roll_ = sample_roll;
    current_pitch_ = sample_pitch;
    current_vehicle_yaw_ = sample_yaw;
    horizontal_speed_m_s_ = std::hypot(
      message->twist.twist.linear.x, message->twist.twist.linear.y);
    vertical_speed_m_s_ = message->twist.twist.linear.z;
    angular_rate_rad_s_ = std::sqrt(
      message->twist.twist.angular.x * message->twist.twist.angular.x +
      message->twist.twist.angular.y * message->twist.twist.angular.y +
      message->twist.twist.angular.z * message->twist.twist.angular.z);
    have_odom_ = true;
    last_odom_arrival_ = receipt_arrival;

    if (!navigation_history_.empty()) {
      const double delta_s =
        (receipt_time - navigation_history_.back().stamp).seconds();
      if (delta_s < -0.5) {
        navigation_history_.clear();
      } else if (delta_s <= 0.0) {
        return;
      }
    }
    navigation_history_.push_back(
      NavigationSample{
        receipt_time, sample_position, sample_roll, sample_pitch, sample_yaw});
    while (!navigation_history_.empty() &&
      (receipt_time - navigation_history_.front().stamp).seconds() >
      odom_history_s_)
    {
      navigation_history_.pop_front();
    }

    if (!frame_locked_) {
      if (horizontal_speed_m_s_ <= stationary_speed_m_s_) {
        if (!position_stable_since_.has_value()) {
          position_stable_since_ = receipt_arrival;
        }
      } else {
        position_stable_since_.reset();
      }
    }
  }

  void extended_state_callback(
    const mavros_msgs::msg::ExtendedState::SharedPtr message)
  {
    extended_state_ = *message;
    have_extended_state_ = true;
    last_extended_state_arrival_ = SteadyClock::now();
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(message->data);
    current_heading_enu_ =
      normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    const SteadyTimePoint compass_arrival = SteadyClock::now();
    last_compass_arrival_ = compass_arrival;
    have_compass_ = true;
    heading_samples_.push_back(
      HeadingSample{compass_arrival, current_heading_enu_});
    while (!heading_samples_.empty() &&
      std::chrono::duration<double>(
        compass_arrival - heading_samples_.front().stamp).count() >
      heading_stability_s_ + 0.5)
    {
      heading_samples_.pop_front();
    }
  }

  std::optional<NavigationSample> navigation_sample_at(
    const rclcpp::Time & stamp) const
  {
    if (navigation_history_.size() < 2U) {
      return std::nullopt;
    }
    const auto upper = std::lower_bound(
      navigation_history_.begin(), navigation_history_.end(), stamp,
      [](const NavigationSample & sample, const rclcpp::Time & value) {
        return sample.stamp < value;
      });
    if (upper == navigation_history_.begin()) {
      const double distance_s = (upper->stamp - stamp).seconds();
      return distance_s >= 0.0 &&
             distance_s <= vision_transform_tolerance_s_ ?
             std::optional<NavigationSample>(*upper) : std::nullopt;
    }
    if (upper == navigation_history_.end()) {
      const NavigationSample & latest = navigation_history_.back();
      const double distance_s = (stamp - latest.stamp).seconds();
      return distance_s >= 0.0 &&
             distance_s <= vision_transform_tolerance_s_ ?
             std::optional<NavigationSample>(latest) : std::nullopt;
    }

    const NavigationSample & after = *upper;
    const NavigationSample & before = *(upper - 1);
    const double gap_s = (after.stamp - before.stamp).seconds();
    if (gap_s <= 0.0 || gap_s > nav_interpolation_max_gap_s_) {
      return std::nullopt;
    }
    const double ratio = std::clamp(
      (stamp - before.stamp).seconds() / gap_s, 0.0, 1.0);
    NavigationSample result;
    result.stamp = stamp;
    result.position = Point3{
      before.position.x + ratio * (after.position.x - before.position.x),
      before.position.y + ratio * (after.position.y - before.position.y),
      before.position.z + ratio * (after.position.z - before.position.z)};
    result.roll = normalize_angle(
      before.roll + ratio * normalize_angle(after.roll - before.roll));
    result.pitch = normalize_angle(
      before.pitch + ratio * normalize_angle(after.pitch - before.pitch));
    result.yaw = normalize_angle(
      before.yaw + ratio * normalize_angle(after.yaw - before.yaw));
    return result;
  }

  bool release_command_committed() const
  {
    return release_actuation_committed_;
  }

  bool vision_guidance_active() const
  {
    return state_ == State::SEARCH || state_ == State::ALIGN ||
      (state_ == State::RELEASE && !release_command_committed());
  }

  bool active_target_id_matches_plan() const
  {
    return active_bucket_.has_value() && target_plan_locked_ &&
      payload_index_ < selected_target_ids_.size() &&
      active_bucket_->id == selected_target_ids_[payload_index_] &&
      !released_target(active_bucket_->id);
  }

  bool refresh_active_target_from_current_track()
  {
    if (!active_target_id_matches_plan()) {
      return false;
    }
    const std::size_t selected_id = active_bucket_->id;
    const auto current = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {
        return track.id == selected_id;
      });
    if (current == known_buckets_.end()) {
      return false;
    }
    const bool frozen_memory = active_bucket_->frozen_memory;
    active_bucket_ = *current;
    active_bucket_->frozen_memory = frozen_memory;
    return true;
  }

  double active_target_observation_age_s() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return steady_age_s(active_bucket_->arrival);
  }

  bool active_target_observation_fresh() const
  {
    const double age_s = active_target_observation_age_s();
    return vision_heartbeat_fresh() && aligned_vision_fresh() &&
      std::isfinite(age_s) && age_s >= 0.0 &&
      age_s <= target_guidance_max_age_s_;
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    const rclcpp::Time receipt_time = now();
    const SteadyTimePoint receipt_arrival = SteadyClock::now();
    have_vision_message_ = true;
    last_vision_message_arrival_ = receipt_arrival;
    // Once release is committed, visual callbacks must never redirect
    // LAND/PILOT_OVERRIDE during ACK/stow cleanup. After both payloads, the
    // hazard-area survey is waypoint-only and has no bucket-vision dependency.
    if (release_command_committed() ||
      payload_index_ >= static_cast<std::size_t>(payload_count_))
    {
      return;
    }
    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting vision frame with zero capture timestamp");
      return;
    }

    const rclcpp::Time observation_time(
      message->header.stamp, get_clock()->get_clock_type());
    const double delay_s = (receipt_time - observation_time).seconds();
    if (!std::isfinite(delay_s) ||
      delay_s < -vision_future_tolerance_s_ ||
      delay_s > vision_max_pipeline_delay_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting vision timestamp delay=%.3fs", delay_s);
      return;
    }
    if (last_vision_capture_stamp_.has_value()) {
      const double capture_delta_s =
        (observation_time - *last_vision_capture_stamp_).seconds();
      if (capture_delta_s <= 0.0) {
        if (capture_delta_s < -0.5) {
          const bool safe_search_rebase = mission_started_ &&
            state_ == State::SEARCH && payload_index_ == 0U &&
            released_target_ids_.empty() && released_positions_.empty();
          navigation_history_.clear();
          last_vision_capture_stamp_.reset();
          known_buckets_.clear();
          clear_target_plan();
          have_aligned_vision_ = false;
          vision_message_count_ = 0U;
          if (safe_search_rebase) {
            // Frames accepted before this capture-epoch discontinuity cannot
            // satisfy the post-gap three-frame acquisition contract. Preserve
            // an already-running steady deadline/hold, but reset its baseline.
            search_vision_entry_aligned_sequence_ = aligned_vision_sequence_;
            if (search_vision_acquire_pending_) {
              search_vision_reacquire_reason_ =
                "capture_clock_rollback_before_first_release";
            }
          }
          if (!mission_started_) {
            RCLCPP_WARN(
              get_logger(),
              "VISION_CAPTURE_CLOCK_REBASE preflight delta=%.3fs",
              capture_delta_s);
          } else if (safe_search_rebase) {
            RCLCPP_WARN(
              get_logger(),
              "VISION_CAPTURE_CLOCK_REBASE search_safe=true "
              "delta=%.3fs index=%zu",
              capture_delta_s, search_index_);
            begin_search_vision_reacquire(
              "capture_clock_rollback_before_first_release");
          } else {
            fail_and_return(
              "Vision capture clock moved backwards outside safe SEARCH rebase");
          }
        }
        return;
      }
    }
    const auto navigation = navigation_sample_at(observation_time);
    if (!navigation.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No capture-time odometry for vision frame");
      return;
    }
    last_vision_capture_stamp_ = observation_time;
    if (have_aligned_vision_ &&
      std::chrono::duration<double>(
        receipt_arrival - last_aligned_vision_arrival_).count() >
      vision_heartbeat_timeout_s_)
    {
      vision_message_count_ = 0U;
    }
    have_aligned_vision_ = true;
    last_aligned_vision_arrival_ = receipt_arrival;
    ++vision_message_count_;
    ++aligned_vision_sequence_;

    if (!frame_locked_ || !vision_guidance_active() ||
      message->poses.empty())
    {
      return;
    }

    std::vector<BucketTrack> detections;
    detections.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      const Point3 body{pose.position.x, pose.position.y, pose.position.z};
      const double diameter = pose.orientation.x;
      double confidence = pose.orientation.y;
      if (!std::isfinite(confidence) || confidence <= 0.0) {
        confidence = 1.0;
      }
      confidence = std::clamp(confidence, 0.0, 1.0);
      if (!std::isfinite(body.x) || !std::isfinite(body.y) ||
        !std::isfinite(body.z) || !std::isfinite(diameter) ||
        diameter < 0.08 || diameter > 0.35 ||
        confidence < min_track_confidence_)
      {
        continue;
      }
      const Point3 local = body_to_local(body, *navigation);
      if (!inside_drop_area(local, 0.8)) {
        continue;
      }
      BucketTrack detection;
      detection.local = local;
      detection.body = body;
      detection.diameter = diameter;
      detection.confidence = confidence;
      detection.confirmations = 1U;
      detection.diameter_samples.push_back(diameter);
      detection.stamp = observation_time;
      detection.arrival = receipt_arrival;
      detections.push_back(std::move(detection));
    }
    merge_detections(detections);

    if (active_bucket_.has_value() &&
      (state_ == State::ALIGN ||
      (state_ == State::RELEASE && !release_command_committed())))
    {
      (void)refresh_active_target_from_current_track();
    }
  }

  Point3 body_to_local(
    const Point3 & body, const NavigationSample & navigation) const
  {
    const double cr = std::cos(navigation.roll);
    const double sr = std::sin(navigation.roll);
    const double cp = std::cos(navigation.pitch);
    const double sp = std::sin(navigation.pitch);
    const double cy = std::cos(navigation.yaw);
    const double sy = std::sin(navigation.yaw);
    const double x_roll = body.x;
    const double y_roll = cr * body.y - sr * body.z;
    const double z_roll = sr * body.y + cr * body.z;
    const double x_pitch = cp * x_roll + sp * z_roll;
    const double y_pitch = y_roll;
    const double z_pitch = -sp * x_roll + cp * z_roll;
    return Point3{
      navigation.position.x + cy * x_pitch - sy * y_pitch,
      navigation.position.y + sy * x_pitch + cy * y_pitch,
      navigation.position.z + z_pitch};
  }

  Point3 local_to_body_current(const Point3 & local) const
  {
    const double dx = local.x - position_.x;
    const double dy = local.y - position_.y;
    const double dz = local.z - position_.z;
    const double cy = std::cos(current_vehicle_yaw_);
    const double sy = std::sin(current_vehicle_yaw_);
    const double cp = std::cos(current_pitch_);
    const double sp = std::sin(current_pitch_);
    const double cr = std::cos(current_roll_);
    const double sr = std::sin(current_roll_);
    const double x_yaw = cy * dx + sy * dy;
    const double y_yaw = -sy * dx + cy * dy;
    const double z_yaw = dz;
    const double x_pitch = cp * x_yaw - sp * z_yaw;
    const double y_pitch = y_yaw;
    const double z_pitch = sp * x_yaw + cp * z_yaw;
    return Point3{
      x_pitch,
      cr * y_pitch + sr * z_pitch,
      -sr * y_pitch + cr * z_pitch};
  }

  Point3 field_to_local(
    double field_x, double field_y, double relative_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double cosine = std::cos(mission_yaw_);
    const double sine = std::sin(mission_yaw_);
    return Point3{
      origin.x + cosine * field_x - sine * field_y,
      origin.y + sine * field_x + cosine * field_y,
      origin.z + relative_z};
  }

  Point3 local_to_field(const Point3 & local) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double dx = local.x - origin.x;
    const double dy = local.y - origin.y;
    const double cosine = std::cos(mission_yaw_);
    const double sine = std::sin(mission_yaw_);
    return Point3{
      cosine * dx + sine * dy,
      -sine * dx + cosine * dy,
      local.z - origin.z};
  }

  bool inside_drop_area(const Point3 & local, double extra) const
  {
    const Point3 field = local_to_field(local);
    return field.x >= drop_near_x_m_ - extra &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ + extra &&
      std::abs(field.y) <= drop_width_y_m_ * 0.5 + extra;
  }

  bool inside_release_area(const Point3 & local) const
  {
    constexpr double margin_m = 0.10;
    const Point3 field = local_to_field(local);
    return field.x >= drop_near_x_m_ + margin_m &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ - margin_m &&
      std::abs(field.y) <= drop_width_y_m_ * 0.5 - margin_m;
  }

  bool released_target(std::size_t id) const
  {
    return std::find(
      released_target_ids_.begin(), released_target_ids_.end(), id) !=
      released_target_ids_.end();
  }

  bool released_position(const Point3 & local) const
  {
    return std::any_of(
      released_positions_.begin(), released_positions_.end(),
      [this, &local](const Point3 & released) {
        return distance_xy(released, local) < released_exclusion_m_;
      });
  }

  void smooth_track(BucketTrack & track, const BucketTrack & detection)
  {
    const double gap_s = (detection.stamp - track.stamp).seconds();
    if (gap_s < 0.0 || gap_s > track_max_gap_s_) {
      const std::size_t id = track.id;
      track = detection;
      track.id = id;
      return;
    }
    const double residual = distance_xy(track.local, detection.local);
    track.position_deviation =
      (1.0 - position_filter_alpha_) * track.position_deviation +
      position_filter_alpha_ * residual;
    track.local.x =
      (1.0 - position_filter_alpha_) * track.local.x +
      position_filter_alpha_ * detection.local.x;
    track.local.y =
      (1.0 - position_filter_alpha_) * track.local.y +
      position_filter_alpha_ * detection.local.y;
    track.local.z =
      (1.0 - position_filter_alpha_) * track.local.z +
      position_filter_alpha_ * detection.local.z;
    track.body.x =
      (1.0 - body_filter_alpha_) * track.body.x +
      body_filter_alpha_ * detection.body.x;
    track.body.y =
      (1.0 - body_filter_alpha_) * track.body.y +
      body_filter_alpha_ * detection.body.y;
    track.body.z =
      (1.0 - body_filter_alpha_) * track.body.z +
      body_filter_alpha_ * detection.body.z;
    track.diameter_samples.push_back(detection.diameter);
    while (track.diameter_samples.size() >
      static_cast<std::size_t>(diameter_filter_window_))
    {
      track.diameter_samples.pop_front();
    }
    const double robust_diameter = median(track.diameter_samples);
    track.diameter =
      (1.0 - diameter_filter_alpha_) * track.diameter +
      diameter_filter_alpha_ * robust_diameter;
    track.diameter_deviation =
      median_absolute_deviation(track.diameter_samples, robust_diameter);
    track.confidence =
      (1.0 - confidence_filter_alpha_) * track.confidence +
      confidence_filter_alpha_ * detection.confidence;
    ++track.confirmations;
    track.stamp = detection.stamp;
    track.arrival = detection.arrival;
  }

  void merge_detections(const std::vector<BucketTrack> & detections)
  {
    known_buckets_.erase(
      std::remove_if(
        known_buckets_.begin(), known_buckets_.end(),
        [this](const BucketTrack & track) {
          const bool selected = std::find(
            selected_target_ids_.begin(), selected_target_ids_.end(), track.id) !=
            selected_target_ids_.end();
          return steady_age_s(track.arrival) > known_memory_s_ &&
                 !released_target(track.id) && !selected;
        }),
      known_buckets_.end());

    struct Association
    {
      std::size_t track = 0U;
      std::size_t detection = 0U;
      double cost = 0.0;
    };
    std::vector<Association> associations;
    for (std::size_t track_index = 0U;
      track_index < known_buckets_.size(); ++track_index)
    {
      for (std::size_t detection_index = 0U;
        detection_index < detections.size(); ++detection_index)
      {
        const double position_delta = distance_xy(
          known_buckets_[track_index].local, detections[detection_index].local);
        const double diameter_delta = std::abs(
          known_buckets_[track_index].diameter -
          detections[detection_index].diameter);
        if (position_delta <= track_gate_m_ &&
          diameter_delta <= diameter_track_gate_m_)
        {
          associations.push_back(
            Association{
              track_index, detection_index,
              position_delta / track_gate_m_ +
              diameter_delta / diameter_track_gate_m_});
        }
      }
    }
    std::sort(
      associations.begin(), associations.end(),
      [](const Association & lhs, const Association & rhs) {
        return lhs.cost < rhs.cost;
      });
    std::vector<bool> track_used(known_buckets_.size(), false);
    std::vector<bool> detection_used(detections.size(), false);
    for (const Association & association : associations) {
      if (track_used[association.track] ||
        detection_used[association.detection])
      {
        continue;
      }
      smooth_track(
        known_buckets_[association.track], detections[association.detection]);
      track_used[association.track] = true;
      detection_used[association.detection] = true;
    }
    for (std::size_t index = 0U; index < detections.size(); ++index) {
      if (detection_used[index]) {
        continue;
      }
      BucketTrack track = detections[index];
      track.id = next_bucket_id_++;
      known_buckets_.push_back(std::move(track));
    }
  }

  bool track_ready(const BucketTrack & track) const
  {
    const double age_s = steady_age_s(track.arrival);
    return track.confirmations >= static_cast<std::size_t>(min_confirmations_) &&
      age_s >= 0.0 && age_s <= selection_max_age_s_ &&
      track.confidence >= min_track_confidence_ &&
      track.position_deviation <= max_position_deviation_m_ &&
      track.diameter_deviation <= max_diameter_deviation_m_ &&
      !released_target(track.id) && !released_position(track.local);
  }

  bool try_lock_target_plan()
  {
    if (target_plan_locked_) {
      return true;
    }
    std::vector<const BucketTrack *> reliable;
    for (const BucketTrack & track : known_buckets_) {
      if (track_ready(track)) {
        reliable.push_back(&track);
      }
    }
    std::sort(
      reliable.begin(), reliable.end(),
      [](const BucketTrack * lhs, const BucketTrack * rhs) {
        if (lhs->confirmations != rhs->confirmations) {
          return lhs->confirmations > rhs->confirmations;
        }
        if (std::abs(lhs->confidence - rhs->confidence) > 1.0e-6) {
          return lhs->confidence > rhs->confidence;
        }
        return lhs->position_deviation + lhs->diameter_deviation <
               rhs->position_deviation + rhs->diameter_deviation;
      });

    std::vector<const BucketTrack *> distinct;
    for (const BucketTrack * candidate : reliable) {
      const bool independent = std::all_of(
        distinct.begin(), distinct.end(),
        [this, candidate](const BucketTrack * accepted) {
          return distance_xy(candidate->local, accepted->local) >=
                 distinct_min_separation_m_ &&
                 std::abs(candidate->diameter - accepted->diameter) >=
                 distinct_min_diameter_m_;
        });
      if (independent) {
        distinct.push_back(candidate);
      }
      if (distinct.size() >= static_cast<std::size_t>(required_bucket_count_)) {
        break;
      }
    }
    if (distinct.size() < static_cast<std::size_t>(required_bucket_count_)) {
      ranking_candidate_ids_.clear();
      ranking_stable_since_.reset();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "SEARCH waiting for three independent buckets: ready=%zu known=%zu",
        distinct.size(), known_buckets_.size());
      return false;
    }
    std::sort(
      distinct.begin(), distinct.end(),
      [](const BucketTrack * lhs, const BucketTrack * rhs) {
        if (std::abs(lhs->diameter - rhs->diameter) > 1.0e-6) {
          return lhs->diameter < rhs->diameter;
        }
        return lhs->id < rhs->id;
      });
    std::vector<std::size_t> proposed_ids;
    proposed_ids.reserve(distinct.size());
    for (const BucketTrack * track : distinct) {
      proposed_ids.push_back(track->id);
    }
    if (proposed_ids != ranking_candidate_ids_) {
      ranking_candidate_ids_ = proposed_ids;
      ranking_stable_since_ = SteadyClock::now();
      RCLCPP_INFO(
        get_logger(),
        "Bucket ranking candidate ids=%zu/%zu/%zu diameters=%.3f/%.3f/%.3f",
        distinct[0]->id, distinct[1]->id, distinct[2]->id,
        distinct[0]->diameter, distinct[1]->diameter, distinct[2]->diameter);
      return false;
    }
    if (!ranking_stable_since_.has_value() ||
      steady_age_s(*ranking_stable_since_) < ranking_stable_s_)
    {
      return false;
    }

    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();
    for (int index = 0; index < payload_count_; ++index) {
      const std::size_t selected_index = static_cast<std::size_t>(index);
      selected_target_ids_.push_back(distinct[selected_index]->id);
      selected_target_positions_.push_back(distinct[selected_index]->local);
      selected_target_diameters_.push_back(distinct[selected_index]->diameter);
      selected_target_confidences_.push_back(distinct[selected_index]->confidence);
    }
    target_plan_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "DROP_TARGETS_LOCKED smallest_id=%zu second_id=%zu diameters=%.3f/%.3f",
      selected_target_ids_[0], selected_target_ids_[1],
      selected_target_diameters_[0], selected_target_diameters_[1]);
    return true;
  }

  std::optional<BucketTrack> current_target_for_payload()
  {
    if (!try_lock_target_plan() ||
      payload_index_ >= selected_target_ids_.size())
    {
      return std::nullopt;
    }
    const std::size_t selected_id = selected_target_ids_[payload_index_];
    const auto current = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {
        return track.id == selected_id;
      });
    if (current == known_buckets_.end()) {
      return std::nullopt;
    }
    const double age_s = steady_age_s(current->arrival);
    // Leaving SEARCH starts active visual guidance, so use the same strict
    // freshness boundary as ALIGN/RELEASE instead of the track-memory timeout.
    if (!vision_heartbeat_fresh() || !aligned_vision_fresh() ||
      age_s < 0.0 || age_s > target_guidance_max_age_s_)
    {
      return std::nullopt;
    }
    return *current;
  }

  std::optional<BucketTrack> last_known_target_for_payload(
    std::size_t payload) const
  {
    if (!target_plan_locked_ || payload >= selected_target_ids_.size() ||
      payload >= selected_target_positions_.size() ||
      payload >= selected_target_diameters_.size() ||
      payload >= selected_target_confidences_.size())
    {
      return std::nullopt;
    }
    const std::size_t selected_id = selected_target_ids_[payload];
    const auto current = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {
        return track.id == selected_id;
      });
    if (current == known_buckets_.end()) {
      return std::nullopt;
    }
    BucketTrack target = *current;
    target.frozen_memory = true;
    return target;
  }

  bool target_identity_valid(const BucketTrack & target) const
  {
    if (!target_plan_locked_ ||
      payload_index_ >= selected_target_ids_.size() ||
      payload_index_ >= selected_target_positions_.size() ||
      payload_index_ >= selected_target_diameters_.size() ||
      target.id != selected_target_ids_[payload_index_] ||
      released_target(target.id))
    {
      return false;
    }
    return distance_xy(
      target.local, selected_target_positions_[payload_index_]) <=
      max_position_deviation_m_ &&
      std::abs(
      target.diameter - selected_target_diameters_[payload_index_]) <=
      max_diameter_deviation_m_ &&
      target.confidence >= min_track_confidence_ &&
      target.position_deviation <= max_position_deviation_m_ &&
      target.diameter_deviation <= max_diameter_deviation_m_;
  }

  void clear_target_plan()
  {
    active_bucket_.reset();
    target_plan_locked_ = false;
    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();
    ranking_candidate_ids_.clear();
    ranking_stable_since_.reset();
  }

  static double steady_age_s(const SteadyTimePoint & stamp)
  {
    return std::chrono::duration<double>(SteadyClock::now() - stamp).count();
  }

  double odom_age_s() const
  {
    return have_odom_ ? steady_age_s(last_odom_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  double compass_age_s() const
  {
    return have_compass_ ? steady_age_s(last_compass_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  double extended_state_age_s() const
  {
    return have_extended_state_ ? steady_age_s(last_extended_state_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  double vision_message_age_s() const
  {
    return have_vision_message_ ? steady_age_s(last_vision_message_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  double aligned_vision_age_s() const
  {
    return have_aligned_vision_ ? steady_age_s(last_aligned_vision_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  bool safety_status_fresh() const
  {
    return safety_seen_ && safety_ready_ &&
      steady_age_s(last_safety_status_arrival_) <= safety_status_timeout_s_;
  }

  bool odom_fresh() const
  {
    return odom_age_s() <= odom_timeout_s_;
  }

  bool compass_fresh() const
  {
    return compass_age_s() <= compass_timeout_s_;
  }

  bool extended_state_fresh() const
  {
    return extended_state_age_s() <= extended_state_timeout_s_;
  }

  bool on_ground_reported() const
  {
    return extended_state_fresh() &&
      extended_state_.landed_state ==
      mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  bool vision_heartbeat_fresh() const
  {
    return vision_message_age_s() <= vision_heartbeat_timeout_s_;
  }

  bool aligned_vision_fresh() const
  {
    return aligned_vision_age_s() <= vision_heartbeat_timeout_s_;
  }

  bool vision_ready_before_takeoff() const
  {
    return vision_message_count_ >=
      static_cast<std::size_t>(vision_min_messages_before_takeoff_) &&
      vision_heartbeat_fresh() && aligned_vision_fresh();
  }

  bool search_vision_acquisition_pending() const
  {
    return state_ == State::SEARCH && search_vision_acquire_pending_;
  }

  std::size_t search_new_aligned_frame_count() const
  {
    if (aligned_vision_sequence_ < search_vision_entry_aligned_sequence_) {
      return 0U;
    }
    return aligned_vision_sequence_ - search_vision_entry_aligned_sequence_;
  }

  void begin_search_vision_reacquire(const std::string & reason)
  {
    if (state_ != State::SEARCH || search_vision_acquire_pending_) {
      return;
    }
    search_vision_hold_position_ = position_;
    target_ = search_vision_hold_position_;
    search_vision_entry_aligned_sequence_ = aligned_vision_sequence_;
    search_vision_acquire_start_arrival_ = SteadyClock::now();
    search_vision_acquire_pending_ = true;
    search_vision_reacquire_reason_ = reason;
    // An outage must not count toward the ranking dwell that can transition
    // SEARCH to ALIGN. Existing track identity may be refreshed after recovery.
    ranking_candidate_ids_.clear();
    ranking_stable_since_.reset();
    RCLCPP_WARN(
      get_logger(),
      "SEARCH_VISION_REACQUIRE_BEGIN reason=%s index=%zu "
      "raw_age=%.3f aligned_age=%.3f odom_age=%.3f hold=(%.2f,%.2f,%.2f)",
      reason.c_str(), search_index_, vision_message_age_s(),
      aligned_vision_age_s(), odom_age_s(),
      search_vision_hold_position_.x, search_vision_hold_position_.y,
      search_vision_hold_position_.z);
  }

  double mean_heading() const
  {
    double sine_sum = 0.0;
    double cosine_sum = 0.0;
    for (const HeadingSample & sample : heading_samples_) {
      sine_sum += std::sin(sample.yaw);
      cosine_sum += std::cos(sample.yaw);
    }
    return std::atan2(sine_sum, cosine_sum);
  }

  bool heading_stable() const
  {
    if (heading_samples_.size() < 5U ||
      std::chrono::duration<double>(
        heading_samples_.back().stamp -
        heading_samples_.front().stamp).count() < heading_stability_s_)
    {
      return false;
    }
    const double mean = mean_heading();
    return std::all_of(
      heading_samples_.begin(), heading_samples_.end(),
      [this, mean](const HeadingSample & sample) {
        return std::abs(normalize_angle(sample.yaw - mean)) <=
               heading_max_variation_rad_;
      });
  }

  bool frame_lock_ready() const
  {
    return config_valid_ && fcu_state_.connected && guided_active_ &&
      !fcu_state_.armed && odom_fresh() && compass_fresh() &&
      heading_stable() && vision_ready_before_takeoff() &&
      safety_status_fresh() &&
      position_stable_since_.has_value() &&
      steady_age_s(*position_stable_since_) >= position_stability_s_ &&
      horizontal_speed_m_s_ <= stationary_speed_m_s_ &&
      servos_initialized_ && payload_status_seen_ &&
      !payload_fault_latched_ && payload_runtime_gate_open_;
  }

  int resolved_search_lane_count() const
  {
    if (search_lane_count_ > 0) {
      return search_lane_count_;
    }
    const double half_width =
      std::max(0.2, drop_width_y_m_ * 0.5 - search_cross_margin_m_);
    const double span = 2.0 * half_width;
    const double footprint =
      2.0 * search_alt_m_ * std::tan(cross_track_camera_fov_rad_ * 0.5);
    const double spacing =
      std::max(0.10, footprint * (1.0 - search_lane_overlap_ratio_));
    if (span < 1.0e-6) {
      return 1;
    }
    return std::max(
      1, static_cast<int>(std::ceil(span / spacing - 1.0e-9)) + 1);
  }

  void lock_frame()
  {
    home_ = position_;
    // MAVROS local position is ENU, so its quaternion yaw is the only yaw
    // that can rotate field coordinates without an unmodelled magnetic
    // declination. Compass heading remains the correct CommandTOL yaw input.
    mission_yaw_ = normalize_angle(current_vehicle_yaw_);
    locked_compass_deg_ = current_compass_deg_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_search_route();
    build_recon_route();
    RCLCPP_INFO(
      get_logger(),
      "FRAME_LOCKED home=(%.2f,%.2f,%.2f) compass=%.2fdeg "
      "odom_yaw_enu=%.2fdeg magnetic_declination_not_applied=true",
      home_->x, home_->y, home_->z, locked_compass_deg_,
      mission_yaw_ * 180.0 / kPi);
  }

  void build_search_route()
  {
    search_route_.clear();
    effective_search_lane_count_ = resolved_search_lane_count();
    const double first_x = drop_near_x_m_ + search_edge_margin_m_;
    const double last_x =
      drop_near_x_m_ + drop_length_x_m_ - search_edge_margin_m_;
    const double half_y =
      std::max(0.2, drop_width_y_m_ * 0.5 - search_cross_margin_m_);
    for (int lane = 0; lane < effective_search_lane_count_; ++lane) {
      const double fraction = effective_search_lane_count_ == 1 ?
        0.5 :
        static_cast<double>(lane) /
        static_cast<double>(effective_search_lane_count_ - 1);
      const double field_y = -half_y + 2.0 * half_y * fraction;
      const bool forward = lane % 2 == 0;
      search_route_.push_back(
        field_to_local(forward ? first_x : last_x, field_y, search_alt_m_));
      search_route_.push_back(
        field_to_local(forward ? last_x : first_x, field_y, search_alt_m_));
    }
    RCLCPP_INFO(
      get_logger(),
      "SEARCH_ROUTE lanes=%d waypoints=%zu mode=%s altitude=%.2f "
      "cross_fov=%.1fdeg overlap=%.0f%%",
      effective_search_lane_count_, search_route_.size(),
      search_lane_count_ == 0 ? "auto" : "explicit", search_alt_m_,
      cross_track_camera_fov_rad_ * 180.0 / kPi,
      search_lane_overlap_ratio_ * 100.0);
  }

  void build_recon_route()
  {
    recon_route_.clear();
    const cuadc_recon::RouteSpec spec{
      recon_center_x_m_,
      0.0,
      recon_length_x_m_ - 2.0 * recon_edge_margin_m_,
      recon_width_y_m_ - 2.0 * recon_cross_margin_m_,
      recon_alt_m_,
      recon_lane_count_};
    const auto field_route = cuadc_recon::generate_waypoint_route(spec);
    for (const auto & point : field_route) {
      recon_route_.push_back(field_to_local(point.x, point.y, point.z));
    }
    RCLCPP_INFO(
      get_logger(),
      "RECON_ROUTE lanes=%d waypoints=%zu center_x=%.2f size=%.2fx%.2f "
      "altitude=%.2f waypoint_only=true hazard_visual=false",
      recon_lane_count_, recon_route_.size(), recon_center_x_m_,
      recon_length_x_m_, recon_width_y_m_, recon_alt_m_);
  }

  void tick()
  {
    check_service_results();
    check_payload_initialization();
    initialize_payload_if_ready();
    monitor_visual_health();
    monitor_safety_health();
    publish_mission_status();

    if (publish_setpoint_ && frame_locked_) {
      publish_setpoint();
    }
    if (mission_started_ && mission_timeout_applies(state_) &&
      steady_age_s(mission_start_time_) > mission_timeout_s_)
    {
      if (state_ == State::RELEASE &&
        release_actuation_committed_ && !stow_complete_)
      {
        require_stow_before_safe_return(
          "Mission timeout during committed servo cleanup");
        fail_and_land(
          "Mission timeout during servo cleanup; landing without horizontal return");
      } else {
        fail_and_return("Mission timeout");
      }
    }
    if (steady_age_s(last_status_time_) >= 4.0) {
      last_status_time_ = SteadyClock::now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s mode=%s armed=%s pos=(%.1f,%.1f,%.1f) payload=%zu/%d "
        "tracks=%zu recon=%zu/%zu failure=%s",
        state_name(state_).c_str(), fcu_state_.mode.c_str(),
        fcu_state_.armed ? "true" : "false",
        position_.x, position_.y, position_.z,
        std::min(
          payload_index_ + 1U, static_cast<std::size_t>(payload_count_)),
        payload_count_, known_buckets_.size(), recon_index_, recon_route_.size(),
        mission_failed_ ? "yes" : "no");
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_ = false;
        if (!config_valid_) {
          mark_failure("Preflight configuration interlock rejected mission");
          enter(State::ABORT);
        } else if (fcu_state_.connected) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        publish_setpoint_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail_and_land("Aircraft armed before GUIDED/frame lock");
        } else if (guided_active_) {
          heading_samples_.clear();
          position_stable_since_.reset();
          enter(State::LOCK_FRAME);
        }
        break;

      case State::LOCK_FRAME:
        publish_setpoint_ = false;
        if (fcu_state_.armed) {
          fail_and_land("Aircraft armed before frame lock completed");
        } else if (frame_lock_ready()) {
          if (!vehicle_->grant_mission_authority()) {
            mark_failure("Vehicle command authority grant was rejected");
            enter(State::ABORT);
          } else {
            lock_frame();
            publish_setpoint_ = true;
            enter(State::PRESTREAM);
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "LOCK_FRAME odom=%s compass=%s heading=%s stationary=%s "
            "vision=%s payload=%s safety=%s",
            odom_fresh() ? "yes" : "no", compass_fresh() ? "yes" : "no",
            heading_stable() ? "yes" : "no",
            position_stable_since_.has_value() ? "yes" : "no",
            vision_ready_before_takeoff() ? "yes" : "no",
            servos_initialized_ ? "yes" : "no",
            safety_status_fresh() ? "yes" : "no");
        }
        break;

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!odom_fresh() || !compass_fresh() ||
          !vision_ready_before_takeoff())
        {
          fail_and_land("Prestream safety gate lost");
        } else if (steady_age_s(state_enter_time_) >= prestream_s_) {
          RCLCPP_WARN(
            get_logger(),
            auto_arm_on_guided_ ?
            "Prestream complete; automatic arm is enabled" :
            "Prestream complete; waiting for pilot arm");
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = home_.value_or(position_);
        if (!vision_ready_before_takeoff()) {
          fail_and_land("Vision gate lost before arm");
        } else if (fcu_state_.armed) {
          if (!servos_initialized_) {
            fail_and_land("Armed before payload stow initialization ACK");
          } else {
            mission_started_ = true;
            mission_start_time_ = SteadyClock::now();
            publish_setpoint_ = false;
            enter(State::TAKEOFF);
          }
        } else if (auto_arm_on_guided_) {
          request_arm(true);
        }
        break;

      case State::TAKEOFF:
        publish_setpoint_ = false;
        if (!flight_gate_ok()) {
          break;
        }
        if (!takeoff_sent_) {
          request_takeoff();
        }
        if (relative_altitude() >= takeoff_alt_m_ * 0.90) {
          publish_setpoint_ = true;
          start_search();
        } else if (
          steady_age_s(state_enter_time_) > takeoff_timeout_s_)
        {
          fail_and_land("Takeoff timeout");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;

      case State::ALIGN:
        if (flight_gate_ok()) {
          update_alignment();
        }
        break;

      case State::RELEASE:
        if (flight_gate_ok()) {
          update_release();
        }
        break;

      case State::RECON_CLIMB:
        if (flight_gate_ok()) {
          update_recon_climb();
        }
        break;

      case State::RECON_SURVEY:
        if (flight_gate_ok()) {
          update_recon_survey();
        }
        break;

      case State::RETURN_CLIMB:
        if (flight_gate_ok()) {
          update_return_climb();
        }
        break;

      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          update_return_home();
        }
        break;

      case State::LAND:
        publish_setpoint_ = false;
        if (update_landing_confirmation()) {
          enter(fcu_state_.armed ? State::DISARM : State::DONE);
        } else if (
          steady_age_s(state_enter_time_) > land_timeout_s_)
        {
          if (!stop_control_for_observed_handoff(
              "LAND watchdog timeout"))
          {
            request_land();
          }
        } else if (fcu_state_.armed) {
          request_land();
        }
        break;

      case State::DISARM:
        publish_setpoint_ = false;
        if (!landing_confirmation_ready()) {
          reset_landing_confirmation();
          enter(State::LAND);
        } else if (!fcu_state_.armed) {
          vehicle_->reset_disarmed();
          enter(State::DONE);
        } else if (
          steady_age_s(state_enter_time_) > disarm_timeout_s_)
        {
          if (!stop_control_for_observed_handoff(
              "DISARM watchdog timeout"))
          {
            request_arm(false);
          }
        } else {
          request_arm(false);
        }
        break;

      case State::DONE:
        publish_setpoint_ = false;
        report_terminal_result();
        if (steady_age_s(state_enter_time_) >= 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_ = false;
        if (!release_command_pending_ && !stow_command_pending_ &&
          !release_abort_requested_)
        {
          if (steady_age_s(state_enter_time_) >= 1.0) {
            RCLCPP_WARN(
              get_logger(), "PILOT_OVERRIDE node exiting; pilot owns flight");
            rclcpp::shutdown();
          }
        } else if (
          steady_age_s(state_enter_time_) >
          payload_action_timeout_s_ + 5.0)
        {
          RCLCPP_ERROR(
            get_logger(),
            "PILOT_OVERRIDE actuator cleanup timed out; physical state uncertain");
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          RCLCPP_ERROR(
            get_logger(), "CUADC_MISSION_ABORT reason=%s",
            terminal_reason_.c_str());
          rclcpp::shutdown();
        }
        break;
    }
  }

  bool guided_required_state(State state) const
  {
    return state == State::LOCK_FRAME || state == State::PRESTREAM ||
      state == State::WAIT_ARM || state == State::TAKEOFF ||
      state == State::SEARCH || state == State::ALIGN ||
      state == State::RELEASE || state == State::RECON_CLIMB ||
      state == State::RECON_SURVEY || state == State::RETURN_CLIMB ||
      state == State::RETURN_HOME;
  }

  bool mission_timeout_applies(State state) const
  {
    return state != State::RETURN_CLIMB && state != State::RETURN_HOME &&
      state != State::LAND && state != State::DISARM &&
      state != State::DONE && state != State::PILOT_OVERRIDE &&
      state != State::ABORT;
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail_and_land("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      vehicle_->revoke_for_pilot();
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    const bool takeoff_odom_grace =
      state_ == State::TAKEOFF &&
      steady_age_s(state_enter_time_) <= 5.0;
    if (!odom_fresh() && !takeoff_odom_grace) {
      fail_and_land("Odometry stale during mission");
      return false;
    }
    if (state_ == State::SEARCH &&
      !search_vision_acquisition_pending() &&
      (!vision_heartbeat_fresh() || !aligned_vision_fresh()))
    {
      begin_search_vision_reacquire(
        "flight_gate_raw_or_aligned_stale");
    }
    return true;
  }

  void monitor_visual_health()
  {
    if (mission_started_ && state_ == State::SEARCH &&
      !search_vision_acquisition_pending() &&
      (!vision_heartbeat_fresh() || !aligned_vision_fresh()))
    {
      begin_search_vision_reacquire(
        "health_monitor_raw_or_aligned_stale");
    }
  }

  void start_search()
  {
    active_bucket_.reset();
    alignment_stable_since_.reset();
    ranking_stable_since_.reset();
    search_index_ = 0U;
    ++search_pass_;
    if (search_route_.empty()) {
      fail_and_return("Search route is empty");
      return;
    }
    search_vision_acquire_pending_ = false;
    enter(State::SEARCH);
    begin_search_vision_reacquire("search_entry");
    RCLCPP_INFO(
      get_logger(),
      "SEARCH_START pass=%d/%d lanes=%d waypoints=%zu altitude=%.2f "
      "vision_acquire_frames=%d timeout=%.1fs hold=(%.2f,%.2f,%.2f)",
      search_pass_, max_search_passes_, effective_search_lane_count_,
      search_route_.size(), search_alt_m_, search_vision_acquire_min_frames_,
      search_vision_acquire_timeout_s_, search_vision_hold_position_.x,
      search_vision_hold_position_.y, search_vision_hold_position_.z);
  }

  void update_search()
  {
    if (search_vision_acquisition_pending()) {
      target_ = search_vision_hold_position_;
      const std::size_t new_aligned_frames = search_new_aligned_frame_count();
      if (new_aligned_frames >=
        static_cast<std::size_t>(search_vision_acquire_min_frames_) &&
        odom_fresh() && vision_heartbeat_fresh() &&
        aligned_vision_fresh())
      {
        if (search_index_ >= search_route_.size()) {
          fail_and_return("Search reacquisition route index invalid");
          return;
        }
        const std::string reason = search_vision_reacquire_reason_;
        search_vision_acquire_pending_ = false;
        const double segment_speed =
          search_pass_ == 1 && search_index_ == 0U ?
          transit_speed_m_s_ : search_speed_m_s_;
        start_segment(
          position_, search_route_[search_index_], segment_speed);
        RCLCPP_INFO(
          get_logger(),
          "SEARCH_VISION_REACQUIRE_ACQUIRED reason=%s index=%zu "
          "new_aligned_frames=%zu raw_age=%.3f aligned_age=%.3f "
          "odom_age=%.3f",
          reason.c_str(), search_index_, new_aligned_frames,
          vision_message_age_s(), aligned_vision_age_s(), odom_age_s());
        search_vision_reacquire_reason_.clear();
        return;
      }
      if (steady_age_s(search_vision_acquire_start_arrival_) >=
        search_vision_acquire_timeout_s_)
      {
        fail_and_return("Search vision reacquisition timeout: " +
          search_vision_reacquire_reason_);
        return;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SEARCH_VISION_REACQUIRE_WAIT reason=%s index=%zu hold=true "
        "new_aligned_frames=%zu/%d raw=%s aligned=%s odom=%s",
        search_vision_reacquire_reason_.c_str(), search_index_,
        new_aligned_frames, search_vision_acquire_min_frames_,
        vision_heartbeat_fresh() ? "yes" : "no",
        aligned_vision_fresh() ? "yes" : "no",
        odom_fresh() ? "yes" : "no");
      return;
    }

    const auto candidate = current_target_for_payload();
    if (candidate.has_value()) {
      active_bucket_ = candidate;
      alignment_stable_since_.reset();
      enter(State::ALIGN);
      RCLCPP_INFO(
        get_logger(),
        "ALIGN_TARGET payload=%zu id=%zu diameter=%.3f confidence=%.2f "
        "body=(%.2f,%.2f,%.2f)",
        payload_index_ + 1U, candidate->id, candidate->diameter,
        candidate->confidence, candidate->body.x, candidate->body.y,
        candidate->body.z);
      return;
    }
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      return;
    }
    ++search_index_;
    if (search_index_ < search_route_.size()) {
      start_segment(
        position_, search_route_[search_index_], search_speed_m_s_);
    } else if (search_pass_ < max_search_passes_) {
      start_search();
    } else {
      fail_and_return("No three-bucket plan after maximum search passes");
    }
  }

  Point3 desired_release_pose(double relative_altitude_m) const
  {
    if (!active_bucket_.has_value() || !home_.has_value()) {
      return position_;
    }
    const Point3 release_offset =
      vector3_at(release_offsets_, payload_index_, Point3{});
    const double cosine = std::cos(mission_yaw_);
    const double sine = std::sin(mission_yaw_);
    return Point3{
      active_bucket_->local.x -
      (cosine * release_offset.x - sine * release_offset.y),
      active_bucket_->local.y -
      (sine * release_offset.x + cosine * release_offset.y),
      home_->z + relative_altitude_m};
  }

  double release_alignment_error() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const Point3 release_offset =
      vector3_at(release_offsets_, payload_index_, Point3{});
    const Point3 target_body =
      local_to_body_current(active_bucket_->local);
    const double body_error = std::hypot(
      target_body.x - release_offset.x,
      target_body.y - release_offset.y);
    const Point3 desired = desired_release_pose(relative_altitude());
    return std::max(body_error, distance_xy(position_, desired));
  }

  void update_alignment()
  {
    if (!active_bucket_.has_value() || !home_.has_value()) {
      fail_and_return("ALIGN entered without target/home");
      return;
    }
    if (!active_target_id_matches_plan()) {
      fail_and_return("ALIGN target ID no longer matches the frozen plan");
      return;
    }
    const double state_age_s = steady_age_s(state_enter_time_);
    if (state_age_s > alignment_timeout_s_) {
      alignment_stable_since_.reset();
      fail_and_return("ALIGN reacquisition/positioning timeout; payload preserved");
      return;
    }

    const bool refreshed = refresh_active_target_from_current_track();
    target_ = desired_release_pose(coarse_alt_m_);
    if (!refreshed || !active_target_observation_fresh()) {
      alignment_stable_since_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ALIGN_REACQUIRE id=%zu refreshed=%s heartbeat=%s capture_aligned=%s "
        "observation_age=%.3f guidance_max_age=%.3f; "
        "using last-known position only",
        active_bucket_->id, refreshed ? "yes" : "no",
        vision_heartbeat_fresh() ? "yes" : "no",
        aligned_vision_fresh() ? "yes" : "no",
        active_target_observation_age_s(), target_guidance_max_age_s_);
      return;
    }
    if (!target_identity_valid(*active_bucket_)) {
      alignment_stable_since_.reset();
      fail_and_return("Frozen target identity/quality changed during ALIGN");
      return;
    }
    if (payload_index_ > 0U &&
      state_age_s < payload_transition_hold_s_)
    {
      alignment_stable_since_.reset();
      target_ = position_;
      return;
    }

    target_ = desired_release_pose(coarse_alt_m_);
    const bool aligned =
      distance_xy(position_, target_) <= coarse_error_m_ &&
      release_alignment_error() <= coarse_error_m_ &&
      std::abs(position_.z - target_.z) <= 0.15;
    if (!aligned) {
      alignment_stable_since_.reset();
      return;
    }
    if (!alignment_stable_since_.has_value()) {
      alignment_stable_since_ = SteadyClock::now();
      return;
    }
    if (steady_age_s(*alignment_stable_since_) < coarse_stable_s_) {
      return;
    }

    if (!refresh_active_target_from_current_track() ||
      !active_target_observation_fresh() ||
      !target_identity_valid(*active_bucket_))
    {
      alignment_stable_since_.reset();
      return;
    }
    frozen_release_pose_ = desired_release_pose(fine_alt_m_);
    active_bucket_->frozen_memory = true;
    reset_release_cycle();
    enter(State::RELEASE);
    RCLCPP_INFO(
      get_logger(),
      "RELEASE_POSE_FROZEN payload=%zu id=%zu pose=(%.3f,%.3f,%.3f) "
      "observation_age=%.3f",
      payload_index_ + 1U, active_bucket_->id,
      frozen_release_pose_.x, frozen_release_pose_.y, frozen_release_pose_.z,
      active_target_observation_age_s());
  }

  void require_stow_before_safe_return(const std::string & reason)
  {
    release_abort_requested_ = true;
    release_cleanup_return_requested_ = true;
    if (release_cleanup_failure_reason_.empty()) {
      release_cleanup_failure_reason_ = reason;
    }
    mark_failure(reason);
    RCLCPP_ERROR(
      get_logger(),
      "RELEASE_CLEANUP_REQUIRED reason=%s committed=%s confirmed=%s",
      reason.c_str(), release_actuation_committed_ ? "true" : "false",
      release_actuation_confirmed_ ? "true" : "false");
  }

  void reset_release_gate_stability()
  {
    release_gate_stable_since_.reset();
    release_gate_pose_anchor_.reset();
  }

  void reset_release_cycle()
  {
    release_command_pending_ = false;
    release_open_ = false;
    stow_command_pending_ = false;
    stow_complete_ = false;
    release_abort_requested_ = false;
    release_actuation_committed_ = false;
    release_actuation_confirmed_ = false;
    release_action_succeeded_ = false;
    release_action_failed_ = false;
    release_goal_handle_.reset();
    release_cleanup_return_requested_ = false;
    release_cleanup_failure_reason_.clear();
    reset_release_gate_stability();
  }

  void update_release()
  {
    target_ = frozen_release_pose_;

    if (release_cleanup_return_requested_) {
      if (stow_complete_ &&
        !release_command_pending_ && !stow_command_pending_)
      {
        release_abort_requested_ = false;
        const std::string reason = release_cleanup_failure_reason_.empty() ?
          "Payload state uncertain; stow confirmed before safe return" :
          release_cleanup_failure_reason_;
        release_cleanup_return_requested_ = false;
        fail_and_return(reason);
      }
      return;
    }

    if (release_action_failed_) {
      release_action_failed_ = false;
      if (release_actuation_committed_ && !stow_complete_) {
        require_stow_before_safe_return(
          "Payload action failed after release commit without stow ACK");
        fail_and_land(
          "Payload state uncertain after committed release; landing in place");
      } else {
        fail_and_return("Payload action rejected or failed before release commit");
      }
      return;
    }

    if (!active_bucket_.has_value()) {
      if (release_command_committed()) {
        require_stow_before_safe_return(
          "Committed release lost target memory; cleanup retained");
        return;
      }
      fail_and_return("RELEASE entered without frozen target");
      return;
    }

    if (release_command_pending_ && !release_command_committed()) {
      if (steady_age_s(release_action_sent_time_) > payload_action_timeout_s_) {
        request_payload_action_cancel("release action did not commit before deadline");
        fail_and_return("Payload release action commit timeout; payload preserved");
      }
      return;
    }
    if (release_command_pending_ && release_actuation_committed_ &&
      steady_age_s(release_action_sent_time_) > payload_action_timeout_s_)
    {
      request_payload_action_cancel(
        "committed payload action exceeded completion deadline");
      require_stow_before_safe_return(
        "Committed payload action timed out before confirmed restow");
      fail_and_land(
        "Committed payload action timeout; landing while payload node restows");
      return;
    }

    if (!release_command_committed()) {
      const bool refreshed = refresh_active_target_from_current_track();
      const bool observation_ok =
        refreshed && active_target_observation_fresh();
      if (!observation_ok) {
        reset_release_gate_stability();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 750,
          "RELEASE_VISION_GATE id=%zu refreshed=%s heartbeat=%s "
          "capture_aligned=%s observation_age=%.3f guidance_max_age=%.3f",
          active_bucket_->id, refreshed ? "yes" : "no",
          vision_heartbeat_fresh() ? "yes" : "no",
          aligned_vision_fresh() ? "yes" : "no",
          active_target_observation_age_s(), target_guidance_max_age_s_);
        if (steady_age_s(state_enter_time_) >
          direct_release_timeout_s_)
        {
          fail_and_return(
            "Release vision reacquisition timeout; payload preserved");
        }
        return;
      }

      const bool identity_ok = target_identity_valid(*active_bucket_);
      if (!identity_ok) {
        reset_release_gate_stability();
        fail_and_return(
          "Release target identity/quality gate failed; payload preserved");
        return;
      }

      const Point3 refreshed_release_pose =
        desired_release_pose(fine_alt_m_);
      frozen_release_pose_ = refreshed_release_pose;
      target_ = frozen_release_pose_;
      if (release_gate_stable_since_.has_value() &&
        release_gate_pose_anchor_.has_value() &&
        distance_xy(
          refreshed_release_pose, *release_gate_pose_anchor_) >
        release_pose_refresh_reset_m_)
      {
        reset_release_gate_stability();
        RCLCPP_INFO(
          get_logger(),
          "RELEASE_POSE_REFRESH reset_stability=true shift_exceeded=%.3f",
          release_pose_refresh_reset_m_);
      }

      const double xy_error = distance_xy(position_, frozen_release_pose_);
      const double height_error =
        std::abs(position_.z - frozen_release_pose_.z);
      const double yaw_error =
        std::abs(normalize_angle(current_vehicle_yaw_ - mission_yaw_));
      const double compensated_error = release_alignment_error();
      const bool area_ok =
        inside_release_area(active_bucket_->local) &&
        inside_release_area(frozen_release_pose_);
      const bool motion_ok =
        horizontal_speed_m_s_ <= direct_release_max_horizontal_speed_m_s_ &&
        std::abs(vertical_speed_m_s_) <=
        direct_release_max_vertical_speed_m_s_;
      const bool attitude_ok =
        std::abs(current_roll_) <= direct_release_max_tilt_rad_ &&
        std::abs(current_pitch_) <= direct_release_max_tilt_rad_ &&
        angular_rate_rad_s_ <= direct_release_max_angular_rate_rad_s_;
      const bool gate_ok =
        area_ok &&
        xy_error <= direct_release_xy_tolerance_m_ &&
        height_error <= direct_release_height_tolerance_m_ &&
        compensated_error <=
        std::max(fine_error_m_, direct_release_xy_tolerance_m_) &&
        motion_ok && attitude_ok &&
        yaw_error <= release_max_yaw_error_rad_;

      if (!gate_ok) {
        reset_release_gate_stability();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "RELEASE_GATE identity=yes area=%s motion=%s attitude=%s "
          "xy=%.3f z=%.3f align=%.3f vxy=%.3f vz=%.3f tilt=%.2f "
          "omega=%.2fdeg_s yaw_error=%.2fdeg observation_age=%.3f",
          area_ok ? "yes" : "no", motion_ok ? "yes" : "no",
          attitude_ok ? "yes" : "no", xy_error, height_error,
          compensated_error, horizontal_speed_m_s_,
          std::abs(vertical_speed_m_s_),
          std::max(std::abs(current_roll_), std::abs(current_pitch_)) *
          180.0 / kPi, angular_rate_rad_s_ * 180.0 / kPi,
          yaw_error * 180.0 / kPi, active_target_observation_age_s());
      } else if (!release_gate_stable_since_.has_value()) {
        release_gate_stable_since_ = SteadyClock::now();
        release_gate_pose_anchor_ = refreshed_release_pose;
      } else if (
        steady_age_s(*release_gate_stable_since_) >=
        std::max(direct_release_stable_s_, fine_stable_s_))
      {
        const bool command_observation_ok =
          refresh_active_target_from_current_track() &&
          active_target_observation_fresh() &&
          target_identity_valid(*active_bucket_);
        const Point3 final_release_pose =
          desired_release_pose(fine_alt_m_);
        const bool command_pose_stable =
          release_gate_pose_anchor_.has_value() &&
          distance_xy(final_release_pose, *release_gate_pose_anchor_) <=
          release_pose_refresh_reset_m_;
        frozen_release_pose_ = final_release_pose;
        target_ = frozen_release_pose_;
        if (!command_observation_ok || !command_pose_stable) {
          reset_release_gate_stability();
          RCLCPP_WARN(
            get_logger(),
            "Release command blocked: final live target/pose refresh failed");
          return;
        }
        if (!send_payload_release_action()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Waiting for /cuadc/payload/release action server");
          if (steady_age_s(state_enter_time_) > direct_release_timeout_s_) {
            fail_and_return(
              "Payload action server unavailable; payload preserved");
          }
          return;
        }
      }
      if (!release_command_committed() &&
        steady_age_s(state_enter_time_) > direct_release_timeout_s_)
      {
        fail_and_return("Release gates/backend timeout; payload preserved");
      }
      return;
    }

    if (release_action_succeeded_ && release_actuation_confirmed_ &&
      stow_complete_ && !release_command_pending_)
    {
      finish_payload_release();
    }
  }

  void finish_payload_release()
  {
    if (!active_bucket_.has_value()) {
      fail_and_return("Payload completion lost target identity");
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "PAYLOAD_RELEASE_COMPLETE payload=%zu id=%zu diameter=%.3f "
      "backend=cuadc_payload_action",
      payload_index_ + 1U, active_bucket_->id, active_bucket_->diameter);
    released_positions_.push_back(active_bucket_->local);
    if (!released_target(active_bucket_->id)) {
      released_target_ids_.push_back(active_bucket_->id);
    }
    ++payload_index_;
    active_bucket_.reset();
    reset_release_cycle();
    alignment_stable_since_.reset();

    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      start_recon_climb();
      return;
    }
    const auto last_known = last_known_target_for_payload(payload_index_);
    if (!last_known.has_value() || released_target(last_known->id)) {
      fail_and_return("Frozen second-smallest target ID unavailable");
      return;
    }
    active_bucket_ = *last_known;
    target_ = position_;
    enter(State::ALIGN);
    RCLCPP_INFO(
      get_logger(),
      "SECOND_TARGET_ID_FROZEN id=%zu diameter=%.3f local=(%.2f,%.2f) "
      "last_observation_age=%.3f",
      active_bucket_->id, active_bucket_->diameter,
      active_bucket_->local.x, active_bucket_->local.y,
      active_target_observation_age_s());
  }

  void start_recon_climb()
  {
    if (!home_.has_value() || recon_route_.empty()) {
      fail_and_return("Recon route unavailable after successful drops");
      return;
    }
    recon_index_ = 0U;
    // Structural guarantee: once both releases are ACKed and stowed, mission
    // guidance permanently disconnects from bucket perception. Hazard-area
    // reconnaissance is generated solely from the signed field route.
    bucket_sub_.reset();
    known_buckets_.clear();
    active_bucket_.reset();
    have_vision_message_ = false;
    have_aligned_vision_ = false;
    recon_hold_since_.reset();
    const double safe_altitude_m = std::max(takeoff_alt_m_, recon_alt_m_);
    start_segment(
      position_,
      Point3{position_.x, position_.y, home_->z + safe_altitude_m},
      transit_speed_m_s_);
    enter(State::RECON_CLIMB);
    RCLCPP_INFO(
      get_logger(),
      "RECON_CLIMB target_altitude=%.2f waypoints=%zu hazard_visual=false",
      safe_altitude_m, recon_route_.size());
  }

  void update_recon_climb()
  {
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      return;
    }
    start_segment(position_, recon_route_.front(), transit_speed_m_s_);
    enter(State::RECON_SURVEY);
  }

  void update_recon_survey()
  {
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      recon_hold_since_.reset();
      return;
    }
    target_ = segment_.end;
    if (!recon_hold_since_.has_value()) {
      recon_hold_since_ = SteadyClock::now();
      return;
    }
    if (steady_age_s(*recon_hold_since_) < recon_waypoint_hold_s_) {
      return;
    }
    const std::size_t reached = recon_index_ + 1U;
    RCLCPP_INFO(
      get_logger(), "RECON_WAYPOINT reached=%zu total=%zu",
      reached, recon_route_.size());
    recon_hold_since_.reset();
    ++recon_index_;
    if (recon_index_ >= recon_route_.size()) {
      recon_completed_ = true;
      RCLCPP_INFO(
        get_logger(), "RECON_COMPLETE reached=%zu total=%zu",
        recon_index_, recon_route_.size());
      start_return_home();
      return;
    }
    start_segment(position_, recon_route_[recon_index_], recon_speed_m_s_);
  }

  void start_return_home()
  {
    if (!home_.has_value()) {
      fail_and_land("Cannot return without locked home");
      return;
    }
    publish_setpoint_ = true;
    start_segment(
      position_,
      Point3{position_.x, position_.y, home_->z + takeoff_alt_m_},
      transit_speed_m_s_);
    enter(State::RETURN_CLIMB);
    RCLCPP_INFO(
      get_logger(), "RETURN_CLIMB target_altitude=%.2f", takeoff_alt_m_);
  }

  void update_return_climb()
  {
    target_ = sample_segment();
    if (segment_complete(accept_radius_m_)) {
      start_segment(
        position_,
        Point3{home_->x, home_->y, home_->z + takeoff_alt_m_},
        transit_speed_m_s_);
      enter(State::RETURN_HOME);
      return;
    }
    if (steady_age_s(state_enter_time_) >
      return_climb_timeout_s_)
    {
      mark_failure("RETURN_CLIMB watchdog timeout");
      RCLCPP_ERROR(
        get_logger(),
        "RETURN_CLIMB_TIMEOUT limit=%.1fs transitioning_to=LAND",
        return_climb_timeout_s_);
      publish_setpoint_ = false;
      enter(State::LAND);
    }
  }

  void update_return_home()
  {
    target_ = sample_segment();
    if (segment_complete(std::max(accept_radius_m_, 0.5))) {
      publish_setpoint_ = false;
      enter(State::LAND);
      return;
    }
    if (steady_age_s(state_enter_time_) >
      return_home_timeout_s_)
    {
      mark_failure("RETURN_HOME watchdog timeout");
      RCLCPP_ERROR(
        get_logger(),
        "RETURN_HOME_TIMEOUT limit=%.1fs transitioning_to=LAND",
        return_home_timeout_s_);
      publish_setpoint_ = false;
      enter(State::LAND);
    }
  }

  void start_segment(
    const Point3 & start, const Point3 & end, double speed_m_s)
  {
    segment_ = Segment{
      start, end, SteadyClock::now(),
      std::max(
        min_segment_s_,
        distance_xyz(start, end) / std::max(0.1, speed_m_s))};
  }

  Point3 sample_segment() const
  {
    const double ratio = std::clamp(
      steady_age_s(segment_.start_time) / segment_.duration_s,
      0.0, 1.0);
    const double smooth = ratio * ratio * (3.0 - 2.0 * ratio);
    return Point3{
      segment_.start.x + smooth * (segment_.end.x - segment_.start.x),
      segment_.start.y + smooth * (segment_.end.y - segment_.start.y),
      segment_.start.z + smooth * (segment_.end.z - segment_.start.z)};
  }

  bool segment_complete(double radius) const
  {
    return steady_age_s(segment_.start_time) >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  void initialize_payload_if_ready()
  {
    if (!config_valid_ || !flight_enable_ || servos_initialized_ ||
      initialization_failed_ || payload_initialization_pending_ ||
      !fcu_state_.connected || fcu_state_.armed ||
      !payload_command_ready_ || !payload_runtime_gate_open_ ||
      !payload_command_client_->service_is_ready() ||
      payload_initialization_index_ >= static_cast<std::size_t>(payload_count_))
    {
      return;
    }

    auto request = std::make_shared<PayloadCommand::Request>();
    request->payload_index =
      static_cast<std::uint8_t>(payload_initialization_index_ + 1U);
    request->command = PayloadCommand::Request::STOW;
    request->request_id = "startup-stow-" +
      std::to_string(payload_initialization_index_ + 1U) + "-" +
      config_bundle_sha256_.substr(0U, 12U);
    payload_initialization_request_id_ = request->request_id;
    payload_initialization_future_ =
      payload_command_client_->async_send_request(request).future.share();
    payload_initialization_pending_ = true;
    payload_initialization_service_accepted_ = false;
    initialization_started_ = true;
    payload_initialization_sent_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(), "PAYLOAD_INITIAL_STOW_REQUEST payload=%zu request_id=%s",
      payload_initialization_index_ + 1U,
      payload_initialization_request_id_.c_str());
  }

  void check_payload_initialization()
  {
    if (!payload_initialization_pending_) {
      return;
    }
    if (payload_initialization_future_.valid() &&
      payload_initialization_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = payload_initialization_future_.get();
      payload_initialization_future_ = {};
      if (!response->accepted ||
        response->request_id != payload_initialization_request_id_)
      {
        initialization_failed_ = true;
        config_valid_ = false;
        payload_initialization_pending_ = false;
        RCLCPP_ERROR(
          get_logger(), "Payload initial stow rejected: %s",
          response->message.c_str());
        return;
      }
      payload_initialization_service_accepted_ = true;
    }
    if (steady_age_s(payload_initialization_sent_time_) >
      payload_initialization_timeout_s_)
    {
      initialization_failed_ = true;
      config_valid_ = false;
      payload_initialization_pending_ = false;
      payload_initialization_future_ = {};
      RCLCPP_ERROR(
        get_logger(),
        "Payload initial stow timed out; restart after inspecting the mechanism");
    }
  }

  void payload_status_callback(const PayloadStatus::SharedPtr message)
  {
    payload_status_seen_ = true;
    last_payload_status_ = *message;
    if (message->state == PayloadStatus::FAULT) {
      payload_fault_latched_ = true;
    }
    if (!payload_initialization_pending_ ||
      !payload_initialization_service_accepted_ ||
      message->request_id != payload_initialization_request_id_ ||
      message->payload_index != payload_initialization_index_ + 1U)
    {
      return;
    }
    if (message->state == PayloadStatus::FAULT) {
      initialization_failed_ = true;
      config_valid_ = false;
      payload_initialization_pending_ = false;
      RCLCPP_ERROR(get_logger(), "Payload entered FAULT during initial stow");
      return;
    }
    if (message->state != PayloadStatus::STOWED || message->busy) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "PAYLOAD_INITIAL_STOW_CONFIRMED payload=%zu",
      payload_initialization_index_ + 1U);
    ++payload_initialization_index_;
    payload_initialization_pending_ = false;
    payload_initialization_service_accepted_ = false;
    initialization_started_ = false;
    if (payload_initialization_index_ >=
      static_cast<std::size_t>(payload_count_))
    {
      servos_initialized_ = true;
      RCLCPP_INFO(
        get_logger(), "PAYLOAD_INITIALIZED both channels stowed with ACKs");
    }
  }

  void payload_safety_status_callback(const SafetyStatus::SharedPtr message)
  {
    payload_command_ready_ = message->mavros_command_ready;
    payload_runtime_gate_open_ = message->runtime_gate_open;
    payload_fault_latched_ = payload_fault_latched_ || message->fault_latched;
  }

  bool send_payload_release_action()
  {
    if (release_command_pending_ || release_goal_handle_ ||
      !payload_release_client_->action_server_is_ready() ||
      payload_index_ >= static_cast<std::size_t>(payload_count_))
    {
      return false;
    }
    ReleasePayload::Goal goal;
    goal.mission_epoch = mission_epoch_;
    goal.command_id = "release-" + std::to_string(payload_index_ + 1U);
    goal.payload_index = static_cast<std::uint8_t>(payload_index_ + 1U);

    rclcpp_action::Client<ReleasePayload>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const ReleaseGoalHandle::SharedPtr & handle) {
        if (!handle) {
          release_command_pending_ = false;
          release_action_failed_ = true;
          RCLCPP_ERROR(get_logger(), "Payload release action goal rejected");
          return;
        }
        release_goal_handle_ = handle;
        if (release_abort_requested_) {
          request_payload_action_cancel("abort was requested while goal was pending");
        }
      };
    options.feedback_callback =
      [this](
        ReleaseGoalHandle::SharedPtr,
        const std::shared_ptr<const ReleasePayload::Feedback> feedback) {
        release_actuation_committed_ =
          release_actuation_committed_ || feedback->committed;
        release_actuation_confirmed_ =
          release_actuation_confirmed_ || feedback->release_ack;
        stow_complete_ = stow_complete_ || feedback->stow_ack;
        release_open_ = release_actuation_confirmed_ && !stow_complete_;
        stow_command_pending_ =
          feedback->stage == ReleasePayload::Feedback::RESTOWING;
      };
    options.result_callback =
      [this](const ReleaseGoalHandle::WrappedResult & wrapped) {
        release_command_pending_ = false;
        stow_command_pending_ = false;
        if (wrapped.result) {
          release_actuation_committed_ =
            release_actuation_committed_ || wrapped.result->committed;
          release_actuation_confirmed_ =
            release_actuation_confirmed_ || wrapped.result->release_ack;
          stow_complete_ = stow_complete_ || wrapped.result->stow_ack;
          release_open_ = release_actuation_confirmed_ && !stow_complete_;
        }
        const bool succeeded =
          wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success &&
          wrapped.result->committed && wrapped.result->release_ack &&
          wrapped.result->stow_ack;
        release_action_succeeded_ = succeeded;
        release_action_failed_ = !succeeded;
        if (state_ == State::PILOT_OVERRIDE && stow_complete_) {
          release_abort_requested_ = false;
        }
        RCLCPP_INFO(
          get_logger(),
          "PAYLOAD_ACTION_RESULT success=%s committed=%s release_ack=%s "
          "stow_ack=%s detail=%s",
          succeeded ? "true" : "false",
          release_actuation_committed_ ? "true" : "false",
          release_actuation_confirmed_ ? "true" : "false",
          stow_complete_ ? "true" : "false",
          wrapped.result ? wrapped.result->detail.c_str() : "missing_result");
      };
    release_command_pending_ = true;
    release_action_sent_time_ = SteadyClock::now();
    payload_release_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "PAYLOAD_ACTION_SENT epoch=%llu command_id=%s payload=%u",
      static_cast<unsigned long long>(goal.mission_epoch),
      goal.command_id.c_str(), static_cast<unsigned int>(goal.payload_index));
    return true;
  }

  void request_payload_action_cancel(const std::string & reason)
  {
    release_abort_requested_ = true;
    if (!release_goal_handle_) {
      return;
    }
    payload_release_client_->async_cancel_goal(release_goal_handle_);
    RCLCPP_WARN(
      get_logger(), "PAYLOAD_ACTION_CANCEL_REQUESTED reason=%s",
      reason.c_str());
  }

  void safety_status_callback(const SafetyStatus::SharedPtr message)
  {
    safety_seen_ = true;
    last_safety_status_arrival_ = SteadyClock::now();
    safety_ready_ =
      !message->fault_latched && message->level != SafetyStatus::FAULT &&
      message->level != SafetyStatus::BLOCKED && message->runtime_gate_open;
    if (safety_ready_ || safety_abort_handled_ || !mission_started_ ||
      state_ == State::DONE || state_ == State::ABORT ||
      state_ == State::PILOT_OVERRIDE || state_ == State::LAND ||
      state_ == State::DISARM)
    {
      return;
    }
    safety_abort_handled_ = true;
    const std::string reason = "External safety monitor fault: " + message->detail;
    if (state_ == State::RELEASE && release_command_pending_) {
      request_payload_action_cancel(reason);
    }
    if (state_ == State::RELEASE && release_actuation_committed_ &&
      !stow_complete_)
    {
      require_stow_before_safe_return(reason);
      fail_and_land(reason + "; landing while payload restow resolves");
    } else if (state_ == State::RETURN_CLIMB || state_ == State::RETURN_HOME) {
      fail_and_land(reason);
    } else {
      fail_and_return(reason);
    }
  }

  void monitor_safety_health()
  {
    if (!mission_started_ || safety_abort_handled_ || safety_status_fresh() ||
      state_ == State::DONE || state_ == State::ABORT ||
      state_ == State::PILOT_OVERRIDE || state_ == State::LAND ||
      state_ == State::DISARM)
    {
      return;
    }
    safety_abort_handled_ = true;
    const std::string reason = "Safety monitor status heartbeat is stale";
    if (state_ == State::RELEASE && release_command_pending_) {
      request_payload_action_cancel(reason);
    }
    if ((state_ == State::RELEASE && release_actuation_committed_ &&
      !stow_complete_) || state_ == State::RETURN_CLIMB ||
      state_ == State::RETURN_HOME)
    {
      fail_and_land(reason);
    } else {
      fail_and_return(reason);
    }
  }

  std::uint8_t mission_status_state() const
  {
    switch (state_) {
      case State::TAKEOFF: return MissionStatus::TAKEOFF;
      case State::SEARCH:
      case State::ALIGN: return MissionStatus::BASKET_SEARCH;
      case State::RELEASE: return MissionStatus::PAYLOAD_DROP;
      case State::RECON_CLIMB:
      case State::RECON_SURVEY: return MissionStatus::HAZARD_WAYPOINT_RECON;
      case State::RETURN_CLIMB:
      case State::RETURN_HOME: return MissionStatus::RETURNING;
      case State::LAND:
      case State::DISARM: return MissionStatus::LANDING;
      case State::DONE:
        return mission_failed_ ? MissionStatus::ABORTED : MissionStatus::COMPLETE;
      case State::ABORT:
      case State::PILOT_OVERRIDE: return MissionStatus::ABORTED;
      default: return MissionStatus::IDLE;
    }
  }

  void publish_mission_status()
  {
    if (steady_age_s(last_mission_status_time_) < 0.25) {
      return;
    }
    last_mission_status_time_ = SteadyClock::now();
    MissionStatus status;
    status.header.stamp = now();
    status.mission_id = config_bundle_sha256_.substr(0U, 12U) + "-" +
      std::to_string(mission_epoch_);
    status.state = mission_status_state();
    status.active = mission_started_ && state_ != State::DONE &&
      state_ != State::ABORT && state_ != State::PILOT_OVERRIDE;
    status.safe_to_continue = safety_status_fresh() && !mission_failed_;
    status.detail = terminal_reason_.empty() ?
      state_name(state_) : state_name(state_) + ":" + terminal_reason_;
    mission_status_pub_->publish(status);
  }

  Point3 rate_limited_drop_target(
    const Point3 & desired, double speed_m_s)
  {
    const SteadyTimePoint current = SteadyClock::now();
    if (!drop_setpoint_initialized_) {
      last_drop_setpoint_ = position_;
      last_drop_setpoint_time_ = current;
      drop_setpoint_initialized_ = true;
    }
    const double dt_s = std::clamp(
      std::chrono::duration<double>(
        current - last_drop_setpoint_time_).count(),
      0.0, 0.20);
    const double dx = desired.x - last_drop_setpoint_.x;
    const double dy = desired.y - last_drop_setpoint_.y;
    const double dz = desired.z - last_drop_setpoint_.z;
    const double remaining_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double max_step_m = std::max(0.0, speed_m_s * dt_s);
    Point3 commanded = desired;
    if (remaining_m > max_step_m && remaining_m > 1.0e-9) {
      const double ratio = max_step_m / remaining_m;
      commanded = Point3{
        last_drop_setpoint_.x + ratio * dx,
        last_drop_setpoint_.y + ratio * dy,
        last_drop_setpoint_.z + ratio * dz};
    }
    last_drop_setpoint_ = commanded;
    last_drop_setpoint_time_ = current;
    return commanded;
  }

  void publish_setpoint()
  {
    Point3 published_target = target_;
    if (state_ == State::ALIGN) {
      published_target =
        rate_limited_drop_target(target_, drop_approach_speed_m_s_);
    } else if (state_ == State::RELEASE) {
      published_target =
        rate_limited_drop_target(target_, release_positioning_speed_m_s_);
    } else {
      drop_setpoint_initialized_ = false;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.pose.position.x = published_target.x;
    message.pose.position.y = published_target.y;
    message.pose.position.z = published_target.z;
    message.pose.orientation.z = yaw_qz_;
    message.pose.orientation.w = yaw_qw_;
    if (!vehicle_->publish_position_setpoint(message)) {
      publish_setpoint_ = false;
      fail_and_land("Vehicle adapter rejected a mission setpoint");
    }
  }

  bool request_allowed() const
  {
    return steady_age_s(last_request_time_) >= 1.0;
  }

  void mark_request()
  {
    last_request_time_ = SteadyClock::now();
  }

  void request_takeoff()
  {
    if (!vehicle_->may_takeoff() ||
      !vehicle_->takeoff_client()->service_is_ready() || takeoff_future_.valid() ||
      !request_allowed())
    {
      return;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(takeoff_alt_m_);
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    takeoff_request_time_ = SteadyClock::now();
    takeoff_future_ =
      vehicle_->takeoff_client()->async_send_request(request).future.share();
    takeoff_sent_ = true;
    RCLCPP_INFO(
      get_logger(), "TAKEOFF_REQUEST altitude=%.2f yaw=%.2f",
      takeoff_alt_m_, locked_compass_deg_);
  }

  void request_land()
  {
    if (!vehicle_->may_land() || !fcu_state_.armed ||
      !vehicle_->land_client()->service_is_ready() ||
      land_future_.valid() || !request_allowed())
    {
      return;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    land_request_time_ = SteadyClock::now();
    land_future_ = vehicle_->land_client()->async_send_request(request).future.share();
  }

  void request_arm(bool arm)
  {
    if (!vehicle_->arm_client()->service_is_ready() || arm_future_.valid() ||
      !request_allowed())
    {
      return;
    }
    if (arm) {
      if (!vehicle_->may_arm() || !auto_arm_on_guided_ || state_ != State::WAIT_ARM ||
        !guided_active_ || !vision_ready_before_takeoff() ||
        !servos_initialized_ || !safety_ready_)
      {
        return;
      }
    } else if (!vehicle_->may_land() || !landing_confirmation_ready()) {
      return;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    pending_arm_value_ = arm;
    mark_request();
    arm_request_time_ = SteadyClock::now();
    arm_future_ = vehicle_->arm_client()->async_send_request(request).future.share();
  }

  void check_service_results()
  {
    if (takeoff_future_.valid()) {
      const bool ready =
        takeoff_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out =
        steady_age_s(takeoff_request_time_) > service_ack_timeout_s_;
      if (ready) {
        const auto response = takeoff_future_.get();
        RCLCPP_INFO(
          get_logger(), "TAKEOFF_ACK success=%s result=%u",
          response->success ? "true" : "false",
          static_cast<unsigned int>(response->result));
        if (!response->success) {
          takeoff_sent_ = false;
        }
        takeoff_future_ = {};
      } else if (timed_out) {
        RCLCPP_ERROR(get_logger(), "TAKEOFF service ACK timeout; retrying");
        takeoff_future_ = {};
        takeoff_sent_ = false;
      }
    }
    if (land_future_.valid()) {
      const bool ready =
        land_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out =
        steady_age_s(land_request_time_) > service_ack_timeout_s_;
      if (ready) {
        const auto response = land_future_.get();
        RCLCPP_INFO(
          get_logger(), "LAND_ACK success=%s result=%u",
          response->success ? "true" : "false",
          static_cast<unsigned int>(response->result));
        land_future_ = {};
      } else if (timed_out) {
        RCLCPP_ERROR(get_logger(), "LAND service ACK timeout; retrying");
        land_future_ = {};
      }
    }
    if (arm_future_.valid()) {
      const bool ready =
        arm_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out =
        steady_age_s(arm_request_time_) > service_ack_timeout_s_;
      if (ready) {
        const auto response = arm_future_.get();
        RCLCPP_INFO(
          get_logger(), "%s_ACK success=%s result=%u",
          pending_arm_value_ ? "ARM" : "DISARM",
          response->success ? "true" : "false",
          static_cast<unsigned int>(response->result));
        arm_future_ = {};
      } else if (timed_out) {
        RCLCPP_ERROR(
          get_logger(), "%s service ACK timeout; retrying",
          pending_arm_value_ ? "ARM" : "DISARM");
        arm_future_ = {};
      }
    }
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  bool landing_candidate() const
  {
    const bool velocity_valid =
      std::isfinite(horizontal_speed_m_s_) &&
      std::isfinite(vertical_speed_m_s_);
    const bool velocity_low =
      velocity_valid &&
      horizontal_speed_m_s_ <= landing_max_horizontal_speed_m_s_ &&
      std::abs(vertical_speed_m_s_) <= landing_max_vertical_speed_m_s_;
    if (extended_state_fresh()) {
      if (on_ground_reported()) {
        return !odom_fresh() || velocity_low;
      }
      // A fresh, explicit airborne/takeoff/landing report is authoritative.
      // Never let a low or reset odometry altitude override it into DISARM.
      if (extended_state_.landed_state !=
        mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED)
      {
        return false;
      }
    }
    const double altitude_m = relative_altitude();
    const double fallback_altitude_m =
      std::min(landing_max_relative_altitude_m_, 0.10);
    return odom_fresh() && std::isfinite(altitude_m) && velocity_low &&
      std::abs(altitude_m) <= fallback_altitude_m;
  }

  void reset_landing_confirmation()
  {
    landing_stable_since_.reset();
  }

  bool update_landing_confirmation()
  {
    if (!landing_candidate()) {
      landing_stable_since_.reset();
      return false;
    }
    if (!landing_stable_since_.has_value()) {
      landing_stable_since_ = SteadyClock::now();
      return false;
    }
    return steady_age_s(*landing_stable_since_) >=
      landing_confirm_stable_s_;
  }

  bool landing_confirmation_ready() const
  {
    return landing_stable_since_.has_value() && landing_candidate() &&
      steady_age_s(*landing_stable_since_) >=
      landing_confirm_stable_s_;
  }

  bool stop_control_for_observed_handoff(const std::string & reason)
  {
    mark_failure(reason);
    publish_setpoint_ = false;
    const bool safe_handoff_observed =
      !fcu_state_.armed || !guided_active_ || fcu_state_.mode == "LAND";
    if (!safe_handoff_observed) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SAFE_HANDOFF_DEFERRED reason=%s mode=%s armed=true; "
        "continuing FCU safety requests",
        reason.c_str(), fcu_state_.mode.c_str());
      return false;
    }
    if (!terminal_reported_) {
      terminal_reported_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "CUADC_MISSION_SAFE_ABORT reason=%s armed=%s mode=%s "
        "control_stopped=true safe_handoff_observed=true",
        reason.c_str(), fcu_state_.armed ? "true" : "false",
        fcu_state_.mode.c_str());
    }
    rclcpp::shutdown();
    return true;
  }

  void mark_failure(const std::string & reason)
  {
    if (!mission_failed_) {
      mission_failed_ = true;
      terminal_reason_ = reason;
    }
    if (state_ == State::RELEASE &&
      (release_actuation_committed_ || release_command_pending_ ||
      release_open_ || stow_command_pending_))
    {
      release_abort_requested_ = true;
      if (release_command_pending_) {
        request_payload_action_cancel(reason);
      }
    }
  }

  void fail_and_return(const std::string & reason)
  {
    mark_failure(reason);
    RCLCPP_ERROR(get_logger(), "MISSION_FAILURE: %s", reason.c_str());
    if (fcu_state_.armed && home_.has_value() && guided_active_ &&
      odom_fresh())
    {
      start_return_home();
    } else {
      enter(fcu_state_.armed ? State::LAND : State::ABORT);
    }
  }

  void fail_and_land(const std::string & reason)
  {
    mark_failure(reason);
    RCLCPP_ERROR(get_logger(), "MISSION_FAILURE: %s", reason.c_str());
    publish_setpoint_ = false;
    vehicle_->enter_fault_landing();
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  void report_terminal_result()
  {
    if (terminal_reported_) {
      return;
    }
    terminal_reported_ = true;
    const bool success =
      !mission_failed_ && !fcu_state_.armed &&
      payload_index_ >= static_cast<std::size_t>(payload_count_) &&
      recon_completed_ && landing_confirmation_ready();
    if (success) {
      std::cout <<
        "任务完成：双投放、危险物区域6端点遍历、返航、降落、上锁" <<
        std::endl;
      RCLCPP_INFO(
        get_logger(),
        "CUADC_MISSION_SUCCESS payloads=2 recon_waypoints=%zu "
        "landed=true disarmed=true",
        recon_route_.size());
    } else {
      const std::string reason = terminal_reason_.empty() ?
        "terminal success conditions not met" : terminal_reason_;
      RCLCPP_ERROR(
        get_logger(), "CUADC_MISSION_SAFE_ABORT reason=%s",
        reason.c_str());
    }
  }

  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "STATE %s -> %s",
      state_name(state_).c_str(), state_name(next).c_str());
    if (next == State::LAND) {
      reset_landing_confirmation();
    }
    state_ = next;
    state_enter_time_ = SteadyClock::now();
  }

  static std::string state_name(State state)
  {
    switch (state) {
      case State::WAIT_FCU: return "WAIT_FCU";
      case State::WAIT_GUIDED: return "WAIT_GUIDED";
      case State::LOCK_FRAME: return "LOCK_FRAME";
      case State::PRESTREAM: return "PRESTREAM";
      case State::WAIT_ARM: return "WAIT_ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::SEARCH: return "SEARCH";
      case State::ALIGN: return "ALIGN";
      case State::RELEASE: return "RELEASE";
      case State::RECON_CLIMB: return "RECON_CLIMB";
      case State::RECON_SURVEY: return "RECON_SURVEY";
      case State::RETURN_CLIMB: return "RETURN_CLIMB";
      case State::RETURN_HOME: return "RETURN_HOME";
      case State::LAND: return "LAND";
      case State::DISARM: return "DISARM";
      case State::DONE: return "DONE";
      case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
      case State::ABORT: return "ABORT";
    }
    return "UNKNOWN";
  }

  bool config_valid_ = false;
  bool flight_enable_ = false;
  bool lock_initial_heading_ = true;
  bool yaw_to_target_ = false;
  bool auto_arm_on_guided_ = false;
  bool pending_arm_value_ = false;
  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool have_extended_state_ = false;
  bool have_vision_message_ = false;
  bool have_aligned_vision_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_ = false;
  bool search_vision_acquire_pending_ = false;
  bool mission_started_ = false;
  bool takeoff_sent_ = false;
  bool target_plan_locked_ = false;
  bool mission_failed_ = false;
  bool recon_completed_ = false;
  bool terminal_reported_ = false;
  bool servos_initialized_ = false;
  bool initialization_started_ = false;
  bool initialization_failed_ = false;
  bool payload_initialization_pending_ = false;
  bool payload_initialization_service_accepted_ = false;
  bool payload_status_seen_ = false;
  bool payload_fault_latched_ = false;
  bool payload_command_ready_ = false;
  bool payload_runtime_gate_open_ = false;
  bool safety_seen_ = false;
  bool safety_ready_ = false;
  bool safety_abort_handled_ = false;
  bool release_command_pending_ = false;
  bool release_open_ = false;
  bool stow_command_pending_ = false;
  bool stow_complete_ = false;
  bool release_abort_requested_ = false;
  bool release_actuation_committed_ = false;
  bool release_actuation_confirmed_ = false;
  bool release_action_succeeded_ = false;
  bool release_action_failed_ = false;
  bool release_cleanup_return_requested_ = false;

  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  Point3 position_;
  Point3 target_;
  Point3 search_vision_hold_position_;
  Point3 frozen_release_pose_;
  std::optional<Point3> home_;
  Segment segment_;
  std::optional<BucketTrack> active_bucket_;
  std::optional<SteadyTimePoint> position_stable_since_;
  std::optional<SteadyTimePoint> ranking_stable_since_;
  std::optional<SteadyTimePoint> alignment_stable_since_;
  std::optional<SteadyTimePoint> release_gate_stable_since_;
  std::optional<Point3> release_gate_pose_anchor_;
  std::optional<SteadyTimePoint> recon_hold_since_;
  std::optional<SteadyTimePoint> landing_stable_since_;
  std::optional<rclcpp::Time> last_vision_capture_stamp_;
  std::deque<HeadingSample> heading_samples_;
  std::deque<NavigationSample> navigation_history_;
  std::vector<BucketTrack> known_buckets_;
  std::vector<Point3> released_positions_;
  std::vector<std::size_t> released_target_ids_;
  std::vector<std::size_t> selected_target_ids_;
  std::vector<Point3> selected_target_positions_;
  std::vector<double> selected_target_diameters_;
  std::vector<double> selected_target_confidences_;
  std::vector<std::size_t> ranking_candidate_ids_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;

  std::string bucket_topic_;
  std::string config_bundle_sha256_;
  std::string payload_initialization_request_id_;
  std::string terminal_reason_;
  std::string release_cleanup_failure_reason_;
  std::string search_vision_reacquire_reason_;
  double takeoff_alt_m_ = 4.0;
  double aircraft_max_horizontal_speed_m_s_ = 4.5;
  double aircraft_max_drop_speed_m_s_ = 0.8;
  double aircraft_max_mission_altitude_m_ = 6.0;
  double aircraft_max_release_tilt_rad_ = 5.0 * kPi / 180.0;
  double search_alt_m_ = 2.0;
  double coarse_alt_m_ = 1.8;
  double fine_alt_m_ = 1.8;
  double transit_speed_m_s_ = 4.0;
  double search_speed_m_s_ = 2.0;
  double recon_speed_m_s_ = 3.0;
  double drop_approach_speed_m_s_ = 0.7;
  double release_positioning_speed_m_s_ = 0.25;
  double min_segment_s_ = 1.0;
  double accept_radius_m_ = 0.4;
  double drop_near_x_m_ = 30.0;
  double drop_length_x_m_ = 5.0;
  double drop_width_y_m_ = 8.0;
  double cross_track_camera_fov_rad_ = 42.0 * kPi / 180.0;
  double search_lane_overlap_ratio_ = 0.30;
  double search_edge_margin_m_ = 0.35;
  double search_cross_margin_m_ = 0.55;
  double recon_center_x_m_ = 55.0;
  double recon_length_x_m_ = 5.0;
  double recon_width_y_m_ = 8.0;
  double recon_alt_m_ = 3.5;
  double recon_edge_margin_m_ = 0.35;
  double recon_cross_margin_m_ = 0.55;
  double recon_waypoint_hold_s_ = 1.0;
  double track_gate_m_ = 0.45;
  double diameter_track_gate_m_ = 0.08;
  double track_max_gap_s_ = 0.60;
  double selection_max_age_s_ = 45.0;
  double distinct_min_separation_m_ = 0.20;
  double distinct_min_diameter_m_ = 0.025;
  double ranking_stable_s_ = 0.80;
  double known_memory_s_ = 120.0;
  double released_exclusion_m_ = 0.25;
  double position_filter_alpha_ = 0.25;
  double body_filter_alpha_ = 1.0;
  double diameter_filter_alpha_ = 0.20;
  double confidence_filter_alpha_ = 0.30;
  double max_position_deviation_m_ = 0.25;
  double max_diameter_deviation_m_ = 0.035;
  double min_track_confidence_ = 0.25;
  double coarse_error_m_ = 0.15;
  double fine_error_m_ = 0.08;
  double coarse_stable_s_ = 0.8;
  double fine_stable_s_ = 0.8;
  double alignment_timeout_s_ = 12.0;
  double detection_timeout_s_ = 3.0;
  double target_guidance_max_age_s_ = 0.5;
  double payload_transition_hold_s_ = 0.5;
  double direct_release_xy_tolerance_m_ = 0.10;
  double direct_release_height_tolerance_m_ = 0.10;
  double direct_release_stable_s_ = 0.80;
  double release_pose_refresh_reset_m_ = 0.03;
  double direct_release_timeout_s_ = 6.0;
  double direct_release_max_horizontal_speed_m_s_ = 0.08;
  double direct_release_max_vertical_speed_m_s_ = 0.05;
  double direct_release_max_tilt_rad_ = 5.0 * kPi / 180.0;
  double direct_release_max_angular_rate_rad_s_ = 5.0 * kPi / 180.0;
  double release_max_yaw_error_rad_ = 5.0 * kPi / 180.0;
  double vision_heartbeat_timeout_s_ = 1.5;
  double vision_max_pipeline_delay_s_ = 1.5;
  double vision_future_tolerance_s_ = 0.05;
  double vision_transform_tolerance_s_ = 0.10;
  double nav_interpolation_max_gap_s_ = 0.20;
  double odom_history_s_ = 5.0;
  double search_vision_acquire_timeout_s_ = 5.0;
  double prestream_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double return_climb_timeout_s_ = 30.0;
  double return_home_timeout_s_ = 120.0;
  double land_timeout_s_ = 120.0;
  double disarm_timeout_s_ = 20.0;
  double odom_timeout_s_ = 1.5;
  double compass_timeout_s_ = 1.0;
  double extended_state_timeout_s_ = 2.5;
  double safety_status_timeout_s_ = 2.0;
  double landing_confirm_stable_s_ = 1.5;
  double landing_max_relative_altitude_m_ = 0.30;
  double landing_max_horizontal_speed_m_s_ = 0.20;
  double landing_max_vertical_speed_m_s_ = 0.15;
  double heading_stability_s_ = 2.0;
  double heading_max_variation_rad_ = 2.0 * kPi / 180.0;
  double position_stability_s_ = 2.0;
  double stationary_speed_m_s_ = 0.15;
  double service_ack_timeout_s_ = 3.0;
  double payload_initialization_timeout_s_ = 8.0;
  double payload_action_timeout_s_ = 12.0;
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;
  double angular_rate_rad_s_ = 0.0;
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double current_vehicle_yaw_ = 0.0;
  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;

  int search_lane_count_ = 0;
  int effective_search_lane_count_ = 8;
  int max_search_passes_ = 3;
  int recon_lane_count_ = 3;
  int required_bucket_count_ = 3;
  int min_confirmations_ = 5;
  int diameter_filter_window_ = 9;
  int vision_min_messages_before_takeoff_ = 3;
  int search_vision_acquire_min_frames_ = 3;
  int payload_count_ = 2;
  int search_pass_ = 0;
  std::size_t search_index_ = 0U;
  std::size_t payload_index_ = 0U;
  std::size_t recon_index_ = 0U;
  std::size_t next_bucket_id_ = 1U;
  std::size_t vision_message_count_ = 0U;
  std::size_t aligned_vision_sequence_ = 0U;
  std::size_t search_vision_entry_aligned_sequence_ = 0U;
  std::size_t payload_initialization_index_ = 0U;
  std::uint64_t mission_epoch_ = 0U;
  bool drop_setpoint_initialized_ = false;
  Point3 last_drop_setpoint_;
  SteadyTimePoint last_drop_setpoint_time_;
  std::vector<double> release_offsets_;

  SteadyTimePoint last_odom_arrival_;
  SteadyTimePoint last_compass_arrival_;
  SteadyTimePoint last_extended_state_arrival_;
  SteadyTimePoint last_safety_status_arrival_;
  SteadyTimePoint last_vision_message_arrival_;
  SteadyTimePoint last_aligned_vision_arrival_;
  SteadyTimePoint search_vision_acquire_start_arrival_;
  SteadyTimePoint state_enter_time_;
  SteadyTimePoint mission_start_time_;
  SteadyTimePoint last_request_time_;
  SteadyTimePoint last_status_time_;
  SteadyTimePoint last_mission_status_time_;
  SteadyTimePoint takeoff_request_time_;
  SteadyTimePoint land_request_time_;
  SteadyTimePoint arm_request_time_;
  SteadyTimePoint payload_initialization_sent_time_;
  SteadyTimePoint release_action_sent_time_;

  std::unique_ptr<cuadc_vehicle::VehicleAdapter> vehicle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Subscription<PayloadStatus>::SharedPtr payload_status_sub_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr payload_safety_sub_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr safety_status_sub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Client<PayloadCommand>::SharedPtr payload_command_client_;
  rclcpp_action::Client<ReleasePayload>::SharedPtr payload_release_client_;
  ReleaseGoalHandle::SharedPtr release_goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
  rclcpp::Client<PayloadCommand>::SharedFuture payload_initialization_future_;
  PayloadStatus last_payload_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
