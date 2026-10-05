/*
 * CUADC 2026 完整自主任务。
 *
 * 飞手确认阶段：
 *   WAIT_FCU -> WAIT_GUIDED -> LOCK_FRAME -> PRESTREAM -> WAIT_ARM
 *
 * 自主执行阶段：
 *   TAKEOFF -> SEARCH -> ALIGN -> RELEASE -> ALIGN -> RELEASE
 *   -> RECON_CLIMB -> RECON_SURVEY -> RETURN_CLIMB
 *   -> RETURN_HOME -> LAND -> DISARM -> DONE
 *
 * 本节点不执行危险物识别。RECON_SURVEY 只遍历场地航点，
 * 不订阅危险物图像或识别结果。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kVersion = "cuadc-full-mission-2026-09-17-v16-geographic-heading";

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

struct PendingVisionDetection
{
  std::size_t index = 0U;
  Point3 body;
  Point3 rim_body;
  double diameter = 0.0;
  double confidence = 0.0;
};

struct PendingVisionFrame
{
  rclcpp::Time stamp;
  SteadyTimePoint arrival;
  std::vector<PendingVisionDetection> detections;
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
  Point3 rim_body;
  double diameter = 0.0;
  double confidence = 0.0;
  double position_deviation = 0.0;
  double diameter_deviation = 0.0;
  std::size_t confirmations = 0U;
  std::size_t consecutive_frame_count = 1U;
  std::size_t last_frame_sequence = 0U;
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

enum class ServoPurpose
{
  INITIALIZE,
  RELEASE,
  STOW
};

struct PendingServoCommand
{
  ServoPurpose purpose = ServoPurpose::INITIALIZE;
  std::size_t payload = 0U;
  SteadyTimePoint sent;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future;
};

template<typename T>
T value_at(const std::vector<T> & values, std::size_t index, const T & fallback)
{
  if (index < values.size()) {
    return values[index];
  }
  return values.empty() ? fallback : values.back();
}

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

}  // 匿名命名空间结束

class CuadcFullMissionNode final : public rclcpp::Node
{
public:
  CuadcFullMissionNode()
  : Node("cuadc_full_mission_node")
  {
    declare_parameters();
    load_parameters();
    open_projection_log();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&CuadcFullMissionNode::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&CuadcFullMissionNode::odom_callback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&CuadcFullMissionNode::pose_callback, this, std::placeholders::_1));
    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", rclcpp::SensorDataQoS(),
      std::bind(
        &CuadcFullMissionNode::extended_state_callback, this,
        std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&CuadcFullMissionNode::compass_callback, this, std::placeholders::_1));
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CuadcFullMissionNode::bucket_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    mission_state_pub_ = create_publisher<std_msgs::msg::String>(
      "/cuadc/mission_state", rclcpp::QoS(20).reliable());
    release_snapshot_pub_ = create_publisher<std_msgs::msg::String>(
      "/cuadc/release_snapshot", rclcpp::QoS(10).reliable());
    takeoff_client_ =
      create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ =
      create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ =
      create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ =
      create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    virtual_release_client_ =
      create_client<std_srvs::srv::Trigger>("/drop_controller/release");

    // ROS 时间仅用于采集时间戳与导航插值。
    // 所有经过时长、看门狗、停留时间、执行器期限和轨迹阶段
    // 均使用单调时钟，避免飞控校时改变这些计时过程。
    const SteadyTimePoint current_arrival = SteadyClock::now();
    last_odom_arrival_ = current_arrival;
    last_pose_arrival_ = current_arrival;
    nav_rate_window_start_ = current_arrival;
    last_compass_arrival_ = current_arrival;
    last_extended_state_arrival_ = current_arrival;
    last_vision_message_arrival_ = current_arrival;
    last_aligned_vision_arrival_ = current_arrival;
    search_vision_acquire_start_arrival_ = current_arrival;
    state_enter_time_ = current_arrival;
    mission_start_time_ = current_arrival;
    release_open_time_ = current_arrival;
    last_request_time_ = current_arrival - std::chrono::seconds(2);
    last_servo_attempt_time_ =
      current_arrival - std::chrono::seconds(2);
    last_status_time_ = current_arrival;
    takeoff_request_time_ = current_arrival;
    land_request_time_ = current_arrival;
    arm_request_time_ = current_arrival;
    virtual_release_sent_time_ = current_arrival;
    segment_.start_time = current_arrival;
    effective_search_lane_count_ = resolved_search_lane_count();
    timer_ = create_wall_timer(50ms, std::bind(&CuadcFullMissionNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CUADC mission ready [%s]: flight_enable=%s release_mode=%s "
      "arm=%s mission=%s heights(takeoff/search/align/release)=%.2f/%.2f/%.2f/%.2f m "
      "payload_targets=%d search_lanes=%d(%s) recon_lanes=%d expected_recon_waypoints=%d",
      kVersion, flight_enable_ ? "true" : "false", release_mode_.c_str(),
      auto_arm_on_guided_ ? "automatic" : "manual",
      drop_only_mode_ ? "drop_only" : "full",
      takeoff_alt_m_, search_alt_m_, coarse_alt_m_, fine_alt_m_, payload_count_,
      effective_search_lane_count_, search_lane_count_ == 0 ? "auto" : "explicit",
      recon_lane_count_, recon_lane_count_ * 2);
    RCLCPP_INFO(
      get_logger(),
      "COARSE_RELEASE_TRIAL_MODE=%s; fine vision refresh retained; "
      "release prechecks removed",
      coarse_release_trial_mode_ ? "enabled" : "disabled");
  }

private:
  void declare_parameters()
  {
    declare_parameter<bool>("flight_enable", false);
    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");
    declare_parameter<std::string>("bucket_projection_log_path", "");
    declare_parameter<bool>("lock_mission_yaw_to_initial_heading", true);
    declare_parameter<double>("geographic_heading_deg", -1.0);
    declare_parameter<bool>("yaw_to_target", false);
    declare_parameter<bool>("auto_arm_on_guided", false);
    declare_parameter<bool>("drop_only_mode", false);
    declare_parameter<bool>("coarse_release_trial_mode", false);
    declare_parameter<bool>("fixed_release_test_mode", false);
    declare_parameter<bool>("fixed_release_after_search", false);
    declare_parameter<std::vector<double>>(
      "fixed_release_points_field_xy",
      std::vector<double>{32.5, -2.6, 32.5, 2.6});

    declare_parameter<double>("vision_heartbeat_timeout_s", 1.5);
    declare_parameter<double>("vision_max_pipeline_delay_s", 1.5);
    declare_parameter<double>("vision_future_tolerance_s", 0.05);
    declare_parameter<double>("vision_transform_tolerance_s", 0.20);
    declare_parameter<double>("nav_interpolation_max_gap_s", 0.50);
    declare_parameter<double>("odom_history_s", 5.0);
    declare_parameter<double>("vision_pending_buffer_s", 1.00);
    declare_parameter<int>("vision_min_messages_before_takeoff", 3);
    declare_parameter<int>("search_vision_acquire_min_frames", 3);
    declare_parameter<double>("search_vision_acquire_timeout_s", 5.0);

    declare_parameter<double>("takeoff_alt_m", 3.0);
    declare_parameter<bool>("enforce_final_drop_heights", true);
    declare_parameter<double>("search_alt_m", 3.0);
    declare_parameter<double>("coarse_alt_m", 2.0);
    declare_parameter<double>("fine_alt_m", 0.5);
    declare_parameter<double>("transit_speed_m_s", 10.5);
    declare_parameter<double>("drop_target_transit_speed_m_s", 6.0);
    declare_parameter<double>("recon_transit_speed_m_s", 9.5);
    declare_parameter<double>("return_speed_m_s", 11.0);
    declare_parameter<double>("return_alt_m", 2.5);
    declare_parameter<double>("search_speed_m_s", 1.75);
    declare_parameter<double>("search_forward_step_m", 1.5);
    declare_parameter<double>("recon_speed_m_s", 1.5);
    declare_parameter<double>("drop_approach_speed_m_s", 5.0);
    declare_parameter<double>("descent_vertical_speed_m_s", 0.2);
    declare_parameter<double>("alignment_horizontal_speed_m_s", 0.5);
    declare_parameter<double>("release_positioning_speed_m_s", 0.2);
    declare_parameter<double>("release_stabilization_s", 2.0);
    declare_parameter<double>("release_visual_xy_tolerance_m", 0.03);
    declare_parameter<double>("middle_calibration_error_m", 0.12);
    declare_parameter<double>("middle_calibration_stable_s", 0.50);
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
    declare_parameter<bool>("use_post_search_cluster_fit", false);
    declare_parameter<double>("cluster_position_scale_m", 0.60);
    declare_parameter<double>("cluster_diameter_scale_m", 0.06);
    declare_parameter<bool>("motion_compensation_enabled", false);
    declare_parameter<std::vector<double>>(
      "motion_compensation_matrix",
      std::vector<double>{0.85335529, 0.80596326, -0.97181809, 0.97507692});
    declare_parameter<std::vector<double>>(
      "motion_compensation_reference_field_xy",
      std::vector<double>{31.75, 0.0});
    declare_parameter<std::vector<double>>(
      "bucket_diameter_centers_m",
      std::vector<double>{0.145, 0.195, 0.235});

    declare_parameter<double>("recon_area_center_field_x_m", 57.45);
    declare_parameter<double>("recon_area_length_field_x_m", 2.4);
    declare_parameter<double>("recon_area_width_field_y_m", 8.0);
    declare_parameter<double>("recon_hover_alt_m", 2.0);
    declare_parameter<int>("recon_lane_count", 2);
    declare_parameter<double>("recon_edge_margin_m", 0.35);
    declare_parameter<double>("recon_cross_margin_m", 0.55);
    declare_parameter<double>("recon_waypoint_hold_s", 0.0);

    declare_parameter<int>("bucket_required_count", 2);
    declare_parameter<bool>("fallback_release_on_incomplete_bucket_set", false);
    declare_parameter<int>("bucket_min_confirmations", 3);
    declare_parameter<double>("track_gate_m", 0.45);
    declare_parameter<double>("consecutive_track_position_gate_m", 0.50);
    declare_parameter<int>("consecutive_track_frames", 3);
    declare_parameter<double>("diameter_track_gate_m", 0.08);
    declare_parameter<double>("bucket_track_max_gap_s", 0.60);
    declare_parameter<double>("bucket_selection_max_age_s", 10.0);
    declare_parameter<double>("bucket_distinct_min_separation_m", 0.14);
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
    declare_parameter<double>("bucket_max_diameter_deviation_m", 0.050);
    declare_parameter<double>("bucket_min_track_confidence", 0.25);

    declare_parameter<double>("coarse_error_m", 0.15);
    declare_parameter<double>("fine_error_m", 0.08);
    declare_parameter<double>("coarse_stable_s", 0.8);
    declare_parameter<double>("alignment_timeout_s", 12.0);
    declare_parameter<double>("detection_timeout_s", 3.0);
    declare_parameter<double>("target_guidance_max_age_s", 0.5);
    declare_parameter<double>("recover_hold_s", 0.5);

    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("drop_phase_timeout_s", 150.0);
    declare_parameter<double>("return_climb_timeout_s", 30.0);
    declare_parameter<double>("return_home_timeout_s", 120.0);
    declare_parameter<double>("land_timeout_s", 120.0);
    declare_parameter<double>("disarm_timeout_s", 20.0);
    declare_parameter<double>("odom_timeout_s", 1.5);
    declare_parameter<double>("compass_timeout_s", 1.0);
    declare_parameter<double>("extended_state_timeout_s", 2.5);
    declare_parameter<double>("landing_confirm_stable_s", 1.5);
    declare_parameter<double>("landing_max_relative_altitude_m", 0.30);
    declare_parameter<double>("landing_max_horizontal_speed_m_s", 0.20);
    declare_parameter<double>("landing_max_vertical_speed_m_s", 0.15);
    declare_parameter<double>("heading_lock_stability_s", 2.0);
    declare_parameter<double>("heading_lock_max_variation_deg", 2.0);
    declare_parameter<double>("position_lock_stability_s", 2.0);
    declare_parameter<double>("stationary_speed_max_m_s", 0.15);
    declare_parameter<double>("service_ack_timeout_s", 3.0);

    declare_parameter<std::string>("release_mode", "servo");
    declare_parameter<int>("payload_count", 2);
    declare_parameter<std::vector<double>>(
      "payload_release_offsets_body_m",
      std::vector<double>{0.029, -0.070, -0.320, -0.031, 0.055, -0.320});
    declare_parameter<std::vector<int64_t>>(
      "servo_channels", std::vector<int64_t>{7, 8});
    declare_parameter<std::vector<int64_t>>(
      "servo_stowed_pwm", std::vector<int64_t>{1100, 1100});
    declare_parameter<std::vector<int64_t>>(
      "servo_release_pwm", std::vector<int64_t>{1900, 1900});
    declare_parameter<std::vector<double>>(
      "servo_release_duration_s", std::vector<double>{0.7, 0.7});
    declare_parameter<bool>("servo_initialize_stowed", true);
    declare_parameter<bool>("servo_return_to_stowed", true);
    declare_parameter<double>("servo_ack_timeout_s", 3.0);
    declare_parameter<double>("virtual_release_ack_timeout_s", 3.0);
  }

  void load_parameters()
  {
    flight_enable_ = get_parameter("flight_enable").as_bool();
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();
    bucket_projection_log_path_ =
      get_parameter("bucket_projection_log_path").as_string();
    lock_initial_heading_ =
      get_parameter("lock_mission_yaw_to_initial_heading").as_bool();
    configured_geographic_heading_deg_ =
      get_parameter("geographic_heading_deg").as_double();
    yaw_to_target_ = get_parameter("yaw_to_target").as_bool();
    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();
    drop_only_mode_ = get_parameter("drop_only_mode").as_bool();
    coarse_release_trial_mode_ =
      get_parameter("coarse_release_trial_mode").as_bool();
    fixed_release_test_mode_ =
      get_parameter("fixed_release_test_mode").as_bool();
    fixed_release_after_search_ =
      get_parameter("fixed_release_after_search").as_bool();
    fixed_release_points_field_xy_ =
      get_parameter("fixed_release_points_field_xy").as_double_array();

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
    vision_pending_buffer_s_ =
      get_parameter("vision_pending_buffer_s").as_double();
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
      search_alt_m_ = 3.0;
      coarse_alt_m_ = 2.0;
      fine_alt_m_ = 0.5;
    } else {
      search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
      coarse_alt_m_ = std::max(1.0, get_parameter("coarse_alt_m").as_double());
      fine_alt_m_ = std::max(0.5, get_parameter("fine_alt_m").as_double());
    }
    transit_speed_m_s_ =
      std::max(0.3, get_parameter("transit_speed_m_s").as_double());
    drop_target_transit_speed_m_s_ = std::max(
      0.3, get_parameter("drop_target_transit_speed_m_s").as_double());
    recon_transit_speed_m_s_ =
      std::max(0.3, get_parameter("recon_transit_speed_m_s").as_double());
    return_speed_m_s_ =
      std::max(0.3, get_parameter("return_speed_m_s").as_double());
    return_alt_m_ =
      std::max(1.0, get_parameter("return_alt_m").as_double());
    search_speed_m_s_ =
      std::max(0.3, get_parameter("search_speed_m_s").as_double());
    search_forward_step_m_ =
      std::max(0.5, get_parameter("search_forward_step_m").as_double());
    recon_speed_m_s_ =
      std::max(0.3, get_parameter("recon_speed_m_s").as_double());
    drop_approach_speed_m_s_ =
      std::max(0.1, get_parameter("drop_approach_speed_m_s").as_double());
    descent_vertical_speed_m_s_ = std::clamp(
      get_parameter("descent_vertical_speed_m_s").as_double(), 0.05, 1.0);
    alignment_horizontal_speed_m_s_ = std::clamp(
      get_parameter("alignment_horizontal_speed_m_s").as_double(), 0.05, 2.0);
    release_positioning_speed_m_s_ = std::max(
      0.05, get_parameter("release_positioning_speed_m_s").as_double());
    release_stabilization_s_ = std::max(
      1.5, get_parameter("release_stabilization_s").as_double());
    release_visual_xy_tolerance_m_ = std::max(
      0.005, get_parameter("release_visual_xy_tolerance_m").as_double());
    middle_calibration_error_m_ = std::max(
      0.05, get_parameter("middle_calibration_error_m").as_double());
    middle_calibration_stable_s_ = std::max(
      0.10, get_parameter("middle_calibration_stable_s").as_double());
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
      0, static_cast<int>(get_parameter("max_search_passes").as_int()));
    use_post_search_cluster_fit_ =
      get_parameter("use_post_search_cluster_fit").as_bool();
    cluster_position_scale_m_ =
      std::max(0.05, get_parameter("cluster_position_scale_m").as_double());
    cluster_diameter_scale_m_ =
      std::max(0.005, get_parameter("cluster_diameter_scale_m").as_double());
    motion_compensation_enabled_ =
      get_parameter("motion_compensation_enabled").as_bool();
    motion_compensation_matrix_ =
      get_parameter("motion_compensation_matrix").as_double_array();
    motion_compensation_reference_field_xy_ =
      get_parameter("motion_compensation_reference_field_xy").as_double_array();
    bucket_diameter_centers_m_ =
      get_parameter("bucket_diameter_centers_m").as_double_array();

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
      2, static_cast<int>(get_parameter("bucket_required_count").as_int()));
    fallback_release_on_incomplete_bucket_set_ =
      get_parameter("fallback_release_on_incomplete_bucket_set").as_bool();
    min_confirmations_ = std::max(
      1, static_cast<int>(get_parameter("bucket_min_confirmations").as_int()));
    track_gate_m_ =
      std::max(0.1, get_parameter("track_gate_m").as_double());
    diameter_track_gate_m_ =
      std::max(0.01, get_parameter("diameter_track_gate_m").as_double());
    consecutive_track_position_gate_m_ = std::max(
      0.05, get_parameter("consecutive_track_position_gate_m").as_double());
    consecutive_track_frames_ = std::max(
      2, static_cast<int>(get_parameter("consecutive_track_frames").as_int()));
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
    alignment_timeout_s_ =
      std::max(1.0, get_parameter("alignment_timeout_s").as_double());
    detection_timeout_s_ =
      std::max(0.2, get_parameter("detection_timeout_s").as_double());
    target_guidance_max_age_s_ = std::clamp(
      get_parameter("target_guidance_max_age_s").as_double(),
      0.05, detection_timeout_s_);
    payload_transition_hold_s_ =
      std::max(0.0, get_parameter("recover_hold_s").as_double());

    prestream_s_ =
      std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ =
      std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ =
      std::max(0.0, get_parameter("mission_timeout_s").as_double());
    drop_phase_timeout_s_ =
      std::max(30.0, get_parameter("drop_phase_timeout_s").as_double());
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

    release_mode_ = get_parameter("release_mode").as_string();
    payload_count_ =
      std::max(1, static_cast<int>(get_parameter("payload_count").as_int()));
    release_offsets_frd_ =
      get_parameter("payload_release_offsets_body_m").as_double_array();
    release_offsets_ = release_offsets_frd_;
    // 外部参数使用飞控机体 FRD 坐标；在 MAVROS ENU 控制计算内部，
    // 继续沿用既有的 FLU 坐标约定。
    for (std::size_t i = 0; i + 2U < release_offsets_.size(); i += 3U) {
      release_offsets_[i + 1U] = -release_offsets_[i + 1U];
      release_offsets_[i + 2U] = -release_offsets_[i + 2U];
    }
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    release_duration_s_ =
      get_parameter("servo_release_duration_s").as_double_array();
    initialize_stowed_ =
      get_parameter("servo_initialize_stowed").as_bool();
    return_to_stowed_ =
      get_parameter("servo_return_to_stowed").as_bool();
    servo_ack_timeout_s_ =
      std::max(0.5, get_parameter("servo_ack_timeout_s").as_double());
    virtual_release_ack_timeout_s_ =
      std::max(0.5, get_parameter("virtual_release_ack_timeout_s").as_double());

    const bool mode_valid =
      release_mode_ == "servo" || release_mode_ == "virtual";
    const bool release_offsets_valid =
      release_offsets_.size() >= static_cast<std::size_t>(payload_count_ * 3) &&
      std::all_of(
      release_offsets_.begin(), release_offsets_.end(),
      [](double value) {return std::isfinite(value);});
    const bool servo_valid =
      release_mode_ != "servo" ||
      (initialize_stowed_ && return_to_stowed_ &&
      servo_channels_.size() >= static_cast<std::size_t>(payload_count_) &&
      stowed_pwm_.size() >= static_cast<std::size_t>(payload_count_) &&
      release_pwm_.size() >= static_cast<std::size_t>(payload_count_) &&
      release_duration_s_.size() >= static_cast<std::size_t>(payload_count_) &&
      servo_channels_[0] == 7 && servo_channels_[1] == 8 &&
      stowed_pwm_[0] == 1100 && stowed_pwm_[1] == 1100 &&
      release_pwm_[0] == 1900 && release_pwm_[1] == 1900 &&
      std::abs(release_duration_s_[0] - 0.7) <= 1.0e-3 &&
      std::abs(release_duration_s_[1] - 0.7) <= 1.0e-3);

    bool fixed_release_valid = !fixed_release_test_mode_;
    if (fixed_release_test_mode_ && coarse_release_trial_mode_ &&
      !drop_only_mode_ && fixed_release_points_field_xy_.size() == 4U &&
      std::all_of(
        fixed_release_points_field_xy_.begin(),
        fixed_release_points_field_xy_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      fixed_release_valid = true;
      for (std::size_t index = 0U; index < 2U; ++index) {
        const double x = fixed_release_points_field_xy_[index * 2U];
        const double y = fixed_release_points_field_xy_[index * 2U + 1U];
        fixed_release_valid = fixed_release_valid &&
          x >= drop_near_x_m_ && x <= drop_near_x_m_ + drop_length_x_m_ &&
          std::abs(y) <= drop_width_y_m_ * 0.5;
      }
    }
    const bool geographic_heading_valid =
      std::isfinite(configured_geographic_heading_deg_) &&
      (configured_geographic_heading_deg_ >= 0.0 &&
      configured_geographic_heading_deg_ < 360.0);
    config_valid_ =
      flight_enable_ && !yaw_to_target_ &&
      mode_valid && payload_count_ == 2 && release_offsets_valid && servo_valid &&
      fixed_release_valid && geographic_heading_valid;
    if (!config_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Safety configuration invalid: flight_enable must be true, yaw must be "
        "fixed, geographic_heading_deg must be in [0,360), "
        "release_mode must be servo|virtual, payload_count must be two, "
        "servo mode requires CH7/CH8 1100->1900 for 0.7s with ACKed stow, "
        "and fixed-release test requires two finite XY points plus coarse mode");
    }
  }

  bool uses_servo() const
  {
    return release_mode_ == "servo";
  }

  bool uses_virtual() const
  {
    return release_mode_ == "virtual";
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr message)
  {
    const bool was_guided = guided_active_;
    const bool was_armed = fcu_state_.armed;
    fcu_state_ = *message;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (guided_active_ && !was_guided) {
      RCLCPP_INFO(
        get_logger(),
        "GUIDED detected; keep the aircraft stationary while frame gates lock");
    }
    if (!guided_active_ && was_guided && guided_required_state(state_)) {
      publish_setpoint_ = false;
      if (state_ == State::RELEASE &&
        (release_command_pending_ || release_open_ ||
        stow_command_pending_ || virtual_release_pending_))
      {
        release_abort_requested_ = true;
      }
      if (uses_servo() && release_open_ && !stow_command_pending_ &&
        !stow_complete_)
      {
        stow_command_pending_ =
          send_servo(payload_index_, false, ServoPurpose::STOW);
      }
      mark_failure("Pilot left GUIDED during autonomous flight");
      enter(State::PILOT_OVERRIDE);
      return;
    }

    // 节点请求自主降落后，飞控进入 LAND 是预期行为。
    // 在 LAND/DISARM 阶段，其他已解锁模式表示飞手或失效保护接管，
    // 此时立即停止继续发送 LAND/DISARM 服务请求。
    const bool landing_or_disarm =
      state_ == State::LAND || state_ == State::DISARM;
    const bool automatic_landing_mode =
      fcu_state_.mode == "GUIDED" || fcu_state_.mode == "LAND";
    if (fcu_state_.armed && landing_or_disarm &&
      (!fcu_state_.connected || !automatic_landing_mode))
    {
      publish_setpoint_ = false;
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
    if (have_odom_) {
      odom_max_gap_s_ = std::max(
        odom_max_gap_s_,
        std::chrono::duration<double>(
          receipt_arrival - last_odom_arrival_).count());
    }
    ++odom_message_count_;
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

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    const SteadyTimePoint receipt_arrival = SteadyClock::now();
    if (have_pose_) {
      pose_max_gap_s_ = std::max(
        pose_max_gap_s_,
        std::chrono::duration<double>(
          receipt_arrival - last_pose_arrival_).count());
    }
    ++pose_message_count_;
    latest_local_pose_ = message->pose;
    have_pose_ = true;
    last_pose_arrival_ = receipt_arrival;
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
      return upper->stamp == stamp ?
             std::optional<NavigationSample>(*upper) : std::nullopt;
    }
    if (upper == navigation_history_.end()) {
      return std::nullopt;
    }

    const NavigationSample & after = *upper;
    const NavigationSample & before = *(upper - 1);
    if (after.stamp == stamp) {
      return after;
    }
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
    if (!target_plan_locked_ || payload_index_ >= selected_target_ids_.size() ||
      released_target(selected_target_ids_[payload_index_]))
    {
      return false;
    }
    const std::size_t selected_id = selected_target_ids_[payload_index_];
    // 相机继续提供诊断信息，但固定位置测试不会读取或替换
    // 检测轨迹给出的坐标。
    if (fixed_release_test_mode_ || fixed_release_direct_active_) {
      return true;
    }
    // 优先使用当前帧。重新检测到的桶可能获得新的局部轨迹编号，
    // 此时以最近候选目标的关联结果完成目标交接。
    if (reacquire_nearest_active_target(selected_id)) {
      return true;
    }
    if (cluster_fit_plan_active_) {
      // 粗对准和中间对准使用稳定跟踪目标，
      // 避免在单帧中切换到距离最近的原始检测框。
      const auto current = std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [selected_id](const BucketTrack & track) { return track.id == selected_id; });
      if (current != known_buckets_.end()) {
        active_bucket_ = *current;
        return true;
      }
      // 前一个载荷投放期间可能发生视觉中断；新观测到达前，
      // 保留最近一次有效的候选目标坐标。
      return active_bucket_.has_value() && !released_target(active_bucket_->id);
    }
    const auto current = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {
        return track.id == selected_id;
      });
    if (current == known_buckets_.end()) {
      return active_bucket_.has_value() && !released_target(active_bucket_->id);
    }
    const bool frozen_memory = active_bucket_->frozen_memory;
    active_bucket_ = *current;
    active_bucket_->frozen_memory = frozen_memory;
    return true;
  }

  bool reacquire_nearest_active_target(std::size_t selected_id)
  {
    // 视觉中断后，重新检测到的目标可能获得新的局部轨迹编号。
    // 选择最接近粗定位候选的有效检测，
    // 同时保留任务槽位编号，避免两载荷投放方案交换目标。
    std::vector<const BucketTrack *> valid;
    for (const BucketTrack & detection : latest_frame_detections_) {
      if (std::isfinite(detection.local.x) && std::isfinite(detection.local.y) &&
        std::isfinite(detection.local.z) &&
        detection.diameter >= 0.08 && detection.diameter <= 0.35 &&
        detection.confidence >= min_track_confidence_ &&
        inside_drop_area(detection.local, 0.8))
      {
        valid.push_back(&detection);
      }
    }
    if (valid.empty()) {
      return false;
    }
    Point3 reference = active_bucket_.has_value() ? active_bucket_->local : Point3{};
    if (payload_index_ < selected_target_positions_.size()) {
      reference = selected_target_positions_[payload_index_];
    }
    const auto nearest = std::min_element(
      valid.begin(), valid.end(), [&reference](const BucketTrack * lhs, const BucketTrack * rhs) {
        const auto d2 = [&reference](const BucketTrack * track) {
          const double dx = track->local.x - reference.x;
          const double dy = track->local.y - reference.y;
          return dx * dx + dy * dy;
        };
        return d2(lhs) < d2(rhs);
      });
    BucketTrack recovered = **nearest;
    recovered.id = selected_id;
    recovered.confirmations = std::max<std::size_t>(
      recovered.confirmations, min_confirmations_);
    recovered.frozen_memory = false;
    active_bucket_ = recovered;
    const auto known = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {return track.id == selected_id;});
    if (known != known_buckets_.end()) {
      *known = recovered;
    }
    if (payload_index_ < selected_target_positions_.size()) {
      selected_target_positions_[payload_index_] = recovered.local;
    }
    if (payload_index_ < selected_target_diameters_.size()) {
      selected_target_diameters_[payload_index_] = recovered.diameter;
    }
    if (payload_index_ < selected_target_confidences_.size()) {
      selected_target_confidences_[payload_index_] = recovered.confidence;
    }
    RCLCPP_WARN(
      get_logger(),
      "TARGET_REACQUIRED_NEAREST payload=%zu preserved_id=%zu local=(%.3f,%.3f,%.3f) "
      "diameter=%.3f",
      payload_index_ + 1U, selected_id, recovered.local.x, recovered.local.y,
      recovered.local.z, recovered.diameter);
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
    // ALIGN/RELEASE 阶段允许视觉中断持续存在；
    // 获取新目标前，以最近一次有效目标坐标作为制导参考。
    // 观测数据的年龄仅用于诊断。
    return active_bucket_.has_value() && !released_target(active_bucket_->id);
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    const rclcpp::Time receipt_time = now();
    const SteadyTimePoint receipt_arrival = SteadyClock::now();
    const bool heartbeat_contiguous =
      have_vision_message_ &&
      steady_age_s(last_vision_message_arrival_) <= vision_heartbeat_timeout_s_;
    have_vision_message_ = true;
    last_vision_message_arrival_ = receipt_arrival;
    if (!heartbeat_contiguous) {
      vision_heartbeat_count_ = 0U;
    }
    ++vision_heartbeat_count_;
    // 释放流程提交后，在确认响应和回位清理期间，视觉回调
    // 不能改变 LAND/PILOT_OVERRIDE 的状态走向。两次投放后，
    // 危险物区域侦察只执行航点，不再依赖桶视觉。
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
    if (last_queued_vision_stamp_.has_value()) {
      const double capture_delta_s =
        (observation_time - *last_queued_vision_stamp_).seconds();
      if (capture_delta_s <= 0.0) {
        if (capture_delta_s < -0.5) {
          const bool safe_search_rebase = mission_started_ &&
            state_ == State::SEARCH && payload_index_ == 0U &&
            released_target_ids_.empty() && released_positions_.empty();
          navigation_history_.clear();
          pending_vision_frames_.clear();
          last_queued_vision_stamp_.reset();
          last_vision_capture_stamp_.reset();
          known_buckets_.clear();
          search_observations_.clear();
          clear_target_plan();
          have_aligned_vision_ = false;
          vision_message_count_ = 0U;
          if (safe_search_rebase) {
            // 本次采集时间段跳变前接收的帧，不能用于满足跳变后的
            // 连续三帧获取条件。保留已运行的单调时钟期限和悬停计时，
            // 但重置帧序列基线。
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

    PendingVisionFrame frame;
    frame.stamp = observation_time;
    frame.arrival = receipt_arrival;
    frame.detections.reserve(message->poses.size());
    for (std::size_t detection_index = 0U;
      detection_index < message->poses.size(); ++detection_index)
    {
      const auto & pose = message->poses[detection_index];
      double confidence = pose.orientation.y;
      if (!std::isfinite(confidence) || confidence <= 0.0) {
        confidence = 1.0;
      }
      frame.detections.push_back(
        PendingVisionDetection{
          detection_index,
          Point3{pose.position.x, pose.position.y, pose.position.z},
          Point3{pose.orientation.z, pose.orientation.w, pose.position.z},
          pose.orientation.x,
          std::clamp(confidence, 0.0, 1.0)});
    }
    last_queued_vision_stamp_ = observation_time;
    pending_vision_frames_.push_back(std::move(frame));
    process_pending_vision_frames();
  }

  void process_pending_vision_frames()
  {
    if (release_command_committed() ||
      payload_index_ >= static_cast<std::size_t>(payload_count_))
    {
      pending_vision_frames_.clear();
      return;
    }

    while (!pending_vision_frames_.empty()) {
      const auto navigation =
        navigation_sample_at(pending_vision_frames_.front().stamp);
      const bool wait_expired =
        steady_age_s(pending_vision_frames_.front().arrival) >=
        vision_pending_buffer_s_;
      if (!navigation.has_value() && !wait_expired) {
        return;
      }

      PendingVisionFrame frame = std::move(pending_vision_frames_.front());
      pending_vision_frames_.pop_front();
      process_pending_vision_frame(frame, navigation);
    }
  }

  void process_pending_vision_frame(
    const PendingVisionFrame & frame,
    const std::optional<NavigationSample> & navigation)
  {
    if (!navigation.has_value()) {
      for (const PendingVisionDetection & pending : frame.detections) {
        write_projection_log(
          frame.stamp, pending.index, pending.body,
          nullptr, nullptr, "no_capture_time_odometry");
      }
      if (!frame.detections.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No capture-time odometry for vision frame");
      }
      return;
    }

    const SteadyTimePoint processed_arrival = SteadyClock::now();
    last_vision_capture_stamp_ = frame.stamp;
    if (have_aligned_vision_ &&
      std::chrono::duration<double>(
        processed_arrival - last_aligned_vision_arrival_).count() >
      vision_heartbeat_timeout_s_)
    {
      vision_message_count_ = 0U;
    }
    have_aligned_vision_ = true;
    last_aligned_vision_arrival_ = processed_arrival;
    ++vision_message_count_;
    ++aligned_vision_sequence_;

    if (!frame_locked_ || !vision_guidance_active())
    {
      return;
    }

    std::vector<BucketTrack> detections;
    detections.reserve(frame.detections.size());
    latest_frame_detections_.clear();
    for (const PendingVisionDetection & pending : frame.detections) {
      // 视觉输出为飞控机体 FRD 坐标；先在此边界转换，再进入
      // 既有 FLU 到 ENU 的任务投影：FLU = (FRD.x, -FRD.y, -FRD.z)。
      const Point3 body{pending.body.x, -pending.body.y, -pending.body.z};
      const Point3 rim_body{
        pending.rim_body.x, -pending.rim_body.y, -pending.rim_body.z};
      const double diameter = pending.diameter;
      const double confidence = pending.confidence;
      if (!std::isfinite(body.x) || !std::isfinite(body.y) ||
        !std::isfinite(body.z) || !std::isfinite(rim_body.x) ||
        !std::isfinite(rim_body.y))
      {
        write_projection_log(
          frame.stamp, pending.index, pending.body, nullptr, nullptr,
          "invalid_metadata");
        continue;
      }
      const Point3 local = body_to_local(body, *navigation);
      // 当前侦察逻辑直接使用采集时刻的姿态，
      // 不再应用旧版针对特定航线和高度的修正。
      const Point3 corrected_local = local;
      BucketTrack current_frame_detection;
      current_frame_detection.local = corrected_local;
      current_frame_detection.body = body;
      current_frame_detection.rim_body = rim_body;
      current_frame_detection.diameter = std::isfinite(diameter) ? diameter : 0.0;
      current_frame_detection.confidence = confidence;
      current_frame_detection.stamp = frame.stamp;
      current_frame_detection.arrival = frame.arrival;
      latest_frame_detections_.push_back(current_frame_detection);
      if (!std::isfinite(diameter) ||
        diameter < 0.08 || diameter > 0.35 ||
        confidence < min_track_confidence_)
      {
        write_projection_log(
          frame.stamp, pending.index, pending.body, nullptr, nullptr,
          "invalid_metadata");
        continue;
      }
      if (!inside_drop_area(local, 0.8)) {
        write_projection_log(
          frame.stamp, pending.index, pending.body, &*navigation, &local,
          "outside_drop_area");
        continue;
      }
      write_projection_log(
        frame.stamp, pending.index, pending.body, &*navigation, &local,
        "projected");
      BucketTrack detection;
      detection.local = corrected_local;
      detection.body = body;
      detection.rim_body = rim_body;
      detection.diameter = diameter;
      detection.confidence = confidence;
      detection.confirmations = 1U;
      detection.consecutive_frame_count = 1U;
      detection.last_frame_sequence = aligned_vision_sequence_;
      detection.diameter_samples.push_back(diameter);
      detection.stamp = frame.stamp;
      detection.arrival = frame.arrival;
      if (state_ == State::SEARCH && use_post_search_cluster_fit_) {
        search_observations_.push_back(detection);
      }
      detections.push_back(std::move(detection));
    }
    merge_detections(detections);

    if (!coarse_release_trial_mode_ && active_bucket_.has_value() &&
      (state_ == State::ALIGN ||
      (state_ == State::RELEASE && !release_command_committed())))
    {
      (void)refresh_active_target_from_current_track();
    }
  }

  void open_projection_log()
  {
    if (bucket_projection_log_path_.empty()) {
      return;
    }
    projection_log_.open(bucket_projection_log_path_, std::ios::out);
    if (!projection_log_.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "Unable to open bucket projection log: %s",
        bucket_projection_log_path_.c_str());
      return;
    }
    projection_log_ <<
      "capture_time_ns,detection_index,body_frd_x_m,body_frd_y_m,body_frd_z_m,"
      "vehicle_x_m,vehicle_y_m,vehicle_z_m,roll_rad,pitch_rad,yaw_rad,"
      "local_x_m,local_y_m,local_z_m,field_x_m,field_y_m,field_z_m,status\n";
    projection_log_.flush();
    RCLCPP_INFO(
      get_logger(), "Bucket projection log enabled: %s",
      bucket_projection_log_path_.c_str());
  }

  void write_projection_log(
    const rclcpp::Time & stamp, std::size_t detection_index,
    const Point3 & body, const NavigationSample * navigation,
    const Point3 * local, const char * status)
  {
    if (!projection_log_.is_open()) {
      return;
    }
    projection_log_ << stamp.nanoseconds() << ',' << detection_index << ',' <<
      body.x << ',' << body.y << ',' << body.z << ',';
    if (navigation == nullptr || local == nullptr) {
      projection_log_ <<
        "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan," <<
        status << '\n';
      projection_log_.flush();
      return;
    }
    const Point3 field = local_to_field(*local);
    projection_log_ << navigation->position.x << ',' << navigation->position.y << ',' <<
      navigation->position.z << ',' << navigation->roll << ',' <<
      navigation->pitch << ',' << navigation->yaw << ',' << local->x << ',' <<
      local->y << ',' << local->z << ',' << field.x << ',' << field.y << ',' <<
      field.z << ',' << status << '\n';
    projection_log_.flush();
  }

  Point3 body_to_local(
    const Point3 & body, const NavigationSample & navigation) const
  {
    const double cr = std::cos(navigation.roll);
    const double sr = std::sin(navigation.roll);
    const double cp = std::cos(navigation.pitch);
    const double sp = std::sin(navigation.pitch);
    const double cy = std::cos(mission_yaw_);
    const double sy = std::sin(mission_yaw_);
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
    const double cy = std::cos(mission_yaw_);
    const double sy = std::sin(mission_yaw_);
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
    // 锁定的任务坐标系以起点为原点，方向为机头前、左、上。
    // MAVROS 设定点仍使用原生 ENU 坐标，
    // 因此每个场地点发布前都经过此处的统一旋转。
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
    // 这是 field_to_local() 的精确逆变换；先将 MAVROS ENU 里程计
    // 旋转到锁定航向的任务坐标系，再应用任务几何约束。
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

  Point3 motion_compensated_local(
    const Point3 & local, const NavigationSample & navigation) const
  {
    if (!motion_compensation_enabled_) {
      return local;
    }
    const Point3 raw_field = local_to_field(local);
    const Point3 vehicle_field = local_to_field(navigation.position);
    const double dx = vehicle_field.x - motion_compensation_reference_field_xy_[0];
    const double dy = vehicle_field.y - motion_compensation_reference_field_xy_[1];
    const double corrected_x = raw_field.x -
      motion_compensation_matrix_[0] * dx - motion_compensation_matrix_[1] * dy;
    const double corrected_y = raw_field.y -
      motion_compensation_matrix_[2] * dx - motion_compensation_matrix_[3] * dy;
    return field_to_local(corrected_x, corrected_y, raw_field.z);
  }

  bool inside_drop_area(const Point3 & local, double extra) const
  {
    const Point3 field = local_to_field(local);
    return field.x >= drop_near_x_m_ - extra &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ + extra &&
      std::abs(field.y) <= drop_width_y_m_ * 0.5 + extra;
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
    const bool consecutive_frame =
      detection.last_frame_sequence == track.last_frame_sequence + 1U &&
      residual <= consecutive_track_position_gate_m_;
    track.consecutive_frame_count = consecutive_frame ?
      track.consecutive_frame_count + 1U : 1U;
    track.last_frame_sequence = detection.last_frame_sequence;
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
    track.rim_body.x =
      (1.0 - body_filter_alpha_) * track.rim_body.x +
      body_filter_alpha_ * detection.rim_body.x;
    track.rim_body.y =
      (1.0 - body_filter_alpha_) * track.rim_body.y +
      body_filter_alpha_ * detection.rim_body.y;
    track.rim_body.z = detection.rim_body.z;
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
      !released_target(track.id);
  }

  bool try_lock_target_plan(bool allow_two = false)
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
                 distinct_min_separation_m_;
        });
      if (independent) {
        distinct.push_back(candidate);
      }
    }
    const std::size_t required = allow_two ? 2U : 3U;
    if (distinct.size() < required) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "SEARCH waiting for stable independent buckets: ready=%zu required=%d known=%zu",
        distinct.size(), static_cast<int>(required), known_buckets_.size());
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
    if (distinct.size() > 2U) {
      distinct.resize(2U);
    }
    // 冻结双目标方案时，距离飞机最近的候选目标
    // 始终分配给第一个载荷。
    std::sort(
      distinct.begin(), distinct.end(),
      [this](const BucketTrack * lhs, const BucketTrack * rhs) {
        return distance_xy(lhs->local, position_) <
               distance_xy(rhs->local, position_);
      });
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

  bool try_lock_post_search_cluster_plan()
  {
    if (target_plan_locked_) {
      return true;
    }
    if (search_observations_.size() <
      static_cast<std::size_t>(required_bucket_count_))
    {
      RCLCPP_WARN(
        get_logger(),
        "POST_SEARCH_CLUSTER unavailable: observations=%zu required_clusters=%d",
        search_observations_.size(), required_bucket_count_);
      return false;
    }

    struct Cluster
    {
      Point3 local;
      double diameter = 0.0;
      double confidence = 0.0;
      double position_rms = 0.0;
      double diameter_rms = 0.0;
      double score = 0.0;
      std::size_t count = 0U;
      SteadyTimePoint latest_arrival;
      rclcpp::Time latest_stamp;
    };
    constexpr std::size_t kClusterCount = 3U;
    std::vector<Cluster> clusters(kClusterCount);
    std::vector<std::size_t> seeds;
    seeds.reserve(kClusterCount);
    std::vector<bool> seed_used(search_observations_.size(), false);
    for (std::size_t center_index = 0U; center_index < kClusterCount; ++center_index) {
      const double diameter_center = bucket_diameter_centers_m_[center_index];
      double nearest_error = std::numeric_limits<double>::infinity();
      std::size_t nearest_index = 0U;
      for (std::size_t index = 0U; index < search_observations_.size(); ++index) {
        if (seed_used[index]) {
          continue;
        }
        const double error = std::abs(
          search_observations_[index].diameter - diameter_center);
        if (error < nearest_error) {
          nearest_error = error;
          nearest_index = index;
        }
      }
      seed_used[nearest_index] = true;
      seeds.push_back(nearest_index);
    }
    for (std::size_t index = 0U; index < kClusterCount; ++index) {
      const BucketTrack & seed = search_observations_[seeds[index]];
      clusters[index].local = seed.local;
      clusters[index].diameter = seed.diameter;
    }

    std::vector<std::size_t> assignment(search_observations_.size(), 0U);
    for (int iteration = 0; iteration < 12; ++iteration) {
      std::vector<double> weights(kClusterCount, 0.0);
      std::vector<Point3> local_sums(kClusterCount);
      std::vector<double> diameter_sums(kClusterCount, 0.0);
      for (std::size_t sample_index = 0U;
        sample_index < search_observations_.size(); ++sample_index)
      {
        const BucketTrack & sample = search_observations_[sample_index];
        double best_distance = std::numeric_limits<double>::infinity();
        std::size_t best_cluster = 0U;
        for (std::size_t cluster_index = 0U; cluster_index < kClusterCount; ++cluster_index) {
          const Cluster & center = clusters[cluster_index];
          const double dx = (sample.local.x - center.local.x) / cluster_position_scale_m_;
          const double dy = (sample.local.y - center.local.y) / cluster_position_scale_m_;
          const double diameter_delta =
            (sample.diameter - center.diameter) / cluster_diameter_scale_m_;
          const double distance = dx * dx + dy * dy + diameter_delta * diameter_delta;
          if (distance < best_distance) {
            best_distance = distance;
            best_cluster = cluster_index;
          }
        }
        assignment[sample_index] = best_cluster;
        const double weight = std::max(0.10, sample.confidence);
        weights[best_cluster] += weight;
        local_sums[best_cluster].x += weight * sample.local.x;
        local_sums[best_cluster].y += weight * sample.local.y;
        local_sums[best_cluster].z += weight * sample.local.z;
        diameter_sums[best_cluster] += weight * sample.diameter;
      }
      for (std::size_t cluster_index = 0U; cluster_index < kClusterCount; ++cluster_index) {
        if (weights[cluster_index] > 0.0) {
          clusters[cluster_index].local = Point3{
            local_sums[cluster_index].x / weights[cluster_index],
            local_sums[cluster_index].y / weights[cluster_index],
            local_sums[cluster_index].z / weights[cluster_index]};
          clusters[cluster_index].diameter = diameter_sums[cluster_index] / weights[cluster_index];
        }
      }
    }

    for (std::size_t sample_index = 0U;
      sample_index < search_observations_.size(); ++sample_index)
    {
      const BucketTrack & sample = search_observations_[sample_index];
      Cluster & cluster = clusters[assignment[sample_index]];
      const double position_error = distance_xy(sample.local, cluster.local);
      const double diameter_error = sample.diameter - cluster.diameter;
      cluster.position_rms += position_error * position_error;
      cluster.diameter_rms += diameter_error * diameter_error;
      cluster.confidence += sample.confidence;
      ++cluster.count;
      if (cluster.count == 1U || sample.arrival > cluster.latest_arrival) {
        cluster.latest_arrival = sample.arrival;
        cluster.latest_stamp = sample.stamp;
      }
    }
    for (Cluster & cluster : clusters) {
      if (cluster.count == 0U) {
        continue;
      }
      const double sample_count = static_cast<double>(cluster.count);
      cluster.position_rms = std::sqrt(cluster.position_rms / sample_count);
      cluster.diameter_rms = std::sqrt(cluster.diameter_rms / sample_count);
      cluster.confidence /= static_cast<double>(cluster.count);
      cluster.score = static_cast<double>(cluster.count) * cluster.confidence /
        (1.0 + cluster.position_rms / cluster_position_scale_m_ +
        cluster.diameter_rms / cluster_diameter_scale_m_);
    }
    std::sort(clusters.begin(), clusters.end(), [](const Cluster & lhs, const Cluster & rhs) {
      return lhs.score > rhs.score;
    });
    std::vector<Cluster> distinct_clusters;
    distinct_clusters.reserve(kClusterCount);
    for (const Cluster & candidate : clusters) {
      if (candidate.count == 0U) {
        continue;
      }
      const bool independent = std::all_of(
        distinct_clusters.begin(), distinct_clusters.end(),
        [this, &candidate](const Cluster & accepted) {
          return distance_xy(candidate.local, accepted.local) >=
            distinct_min_separation_m_;
        });
      if (independent) {
        distinct_clusters.push_back(candidate);
      }
      if (distinct_clusters.size() == kClusterCount) {
        break;
      }
    }
    if (distinct_clusters.size() < static_cast<std::size_t>(required_bucket_count_))
    {
      RCLCPP_WARN(
        get_logger(),
        "POST_SEARCH_CLUSTER produced fewer than three independent targets "
        "at separation %.2f m",
        distinct_min_separation_m_);
      return false;
    }

    // 位置选择仅依据空间关系；三个目标都可信时，选择直径最大
    // 和中等的两个桶；只有两个可信目标时，两者都使用。
    std::sort(
      distinct_clusters.begin(), distinct_clusters.end(),
      [](const Cluster & lhs, const Cluster & rhs) {
        return lhs.diameter > rhs.diameter;
      });
    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();
    clustered_target_tracks_.clear();
    for (std::size_t index = 0U; index < distinct_clusters.size(); ++index) {
      const Cluster & cluster = distinct_clusters[index];
      RCLCPP_INFO(
        get_logger(),
        "POST_SEARCH_CLUSTER rank=%zu samples=%zu score=%.3f local=(%.2f,%.2f,%.2f) diameter=%.3f conf=%.2f rms=(%.3f,%.3f)",
        index + 1U, cluster.count, cluster.score, cluster.local.x, cluster.local.y,
        cluster.local.z, cluster.diameter, cluster.confidence,
        cluster.position_rms, cluster.diameter_rms);
      if (index >= static_cast<std::size_t>(payload_count_)) {
        continue;
      }
      BucketTrack target;
      target.id = next_bucket_id_++;
      target.local = cluster.local;
      target.diameter = cluster.diameter;
      target.confidence = cluster.confidence;
      target.position_deviation = cluster.position_rms;
      target.diameter_deviation = cluster.diameter_rms;
      target.confirmations = cluster.count;
      target.stamp = cluster.latest_stamp;
      target.arrival = cluster.latest_arrival;
      target.frozen_memory = true;
      clustered_target_tracks_.push_back(target);
      selected_target_ids_.push_back(target.id);
      selected_target_positions_.push_back(target.local);
      selected_target_diameters_.push_back(target.diameter);
      selected_target_confidences_.push_back(target.confidence);
    }
    if (clustered_target_tracks_.size() != static_cast<std::size_t>(payload_count_)) {
      return false;
    }
    cluster_fit_plan_active_ = true;
    target_plan_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "POST_SEARCH_TARGETS_LOCKED payload_targets=%d ids=%zu/%zu scores select two most credible clusters",
      payload_count_,
      selected_target_ids_[0], selected_target_ids_[1]);
    return true;
  }

  bool try_lock_fallback_target_plan()
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

    // 后备方案只放宽三个桶的尺寸排序规则，
    // 所选目标仍须通过正常的多帧轨迹质量检查。
    std::vector<const BucketTrack *> distinct;
    for (const BucketTrack * candidate : reliable) {
      const bool independent = std::all_of(
        distinct.begin(), distinct.end(),
        [this, candidate](const BucketTrack * accepted) {
          return distance_xy(candidate->local, accepted->local) >=
                 distinct_min_separation_m_;
        });
      if (independent) {
        distinct.push_back(candidate);
      }
      if (distinct.size() >= static_cast<std::size_t>(payload_count_)) {
        break;
      }
    }
    if (distinct.size() < static_cast<std::size_t>(payload_count_)) {
      RCLCPP_WARN(
        get_logger(),
        "FALLBACK_DROP unavailable: reliable_independent=%zu required_payloads=%d known=%zu",
        distinct.size(), payload_count_, known_buckets_.size());
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
    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();
    for (int index = 0; index < payload_count_; ++index) {
      const BucketTrack * target = distinct[static_cast<std::size_t>(index)];
      selected_target_ids_.push_back(target->id);
      selected_target_positions_.push_back(target->local);
      selected_target_diameters_.push_back(target->diameter);
      selected_target_confidences_.push_back(target->confidence);
    }
    target_plan_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "FALLBACK_DROP_TARGETS_LOCKED ids=%zu/%zu diameters=%.3f/%.3f; "
      "three-bucket ranking unavailable, proceeding with probable targets",
      selected_target_ids_[0], selected_target_ids_[1],
      selected_target_diameters_[0], selected_target_diameters_[1]);
    return true;
  }

  std::optional<BucketTrack> current_target_for_payload()
  {
    if (cluster_fit_plan_active_) {
      if (payload_index_ < clustered_target_tracks_.size()) {
        return clustered_target_tracks_[payload_index_];
      }
      return std::nullopt;
    }
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
    if (age_s < 0.0 || age_s > known_memory_s_)
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
    if (cluster_fit_plan_active_) {
      const auto cluster_target = std::find_if(
        clustered_target_tracks_.begin(), clustered_target_tracks_.end(),
        [selected_id](const BucketTrack & track) { return track.id == selected_id; });
      return cluster_target == clustered_target_tracks_.end() ?
        std::nullopt : std::optional<BucketTrack>(*cluster_target);
    }
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
    if (cluster_fit_plan_active_) {
      return true;
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
    coarse_xy_filter_initialized_ = false;
    fixed_release_direct_active_ = false;
    target_plan_locked_ = false;
    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();
    clustered_target_tracks_.clear();
    cluster_fit_plan_active_ = false;
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

  double pose_age_s() const
  {
    return have_pose_ ? steady_age_s(last_pose_arrival_) :
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
    // 空 PoseArray 是每次采集主动发送的心跳。起飞点附近不应
    // 要求检测到桶，因此采集与里程计的时间对齐检查及目标检测要求
    // 仅在进入 SEARCH 后强制执行。
    return vision_heartbeat_count_ >=
      static_cast<std::size_t>(vision_min_messages_before_takeoff_) &&
      vision_heartbeat_fresh();
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
    // 视觉中断时间不能计入从 SEARCH 转入 ALIGN 所需的排序稳定时间。
    // 视觉恢复后可以刷新已有轨迹的目标关联。
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
      !fcu_state_.armed && odom_fresh() && vision_ready_before_takeoff() &&
      position_stable_since_.has_value() &&
      steady_age_s(*position_stable_since_) >= position_stability_s_ &&
      horizontal_speed_m_s_ <= stationary_speed_m_s_ &&
      (!uses_servo() || servos_initialized_);
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
    const double odom_yaw_enu = normalize_angle(current_vehicle_yaw_);
    const double compass_yaw_enu = mean_heading();
    // 地理航向以正北为零，顺时针增加；ROS ENU 偏航角
    // 以正东为零，逆时针增加，因此 yaw_enu = 90° - heading。
    mission_yaw_ = normalize_angle(
      (90.0 - configured_geographic_heading_deg_) * kPi / 180.0);
    const double measured_compass_deg = normalize_degrees(
      90.0 - compass_yaw_enu * 180.0 / kPi);
    locked_compass_deg_ = normalize_degrees(
      90.0 - mission_yaw_ * 180.0 / kPi);
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_search_route();
    build_recon_route();
    RCLCPP_INFO(
      get_logger(),
      "FRAME_LOCKED home=(%.2f,%.2f,%.2f) source=%s "
      "commanded_heading=%.2fdeg measured_compass=%.2fdeg "
      "compass_yaw_enu=%.2fdeg odom_yaw_enu=%.2fdeg mission_yaw_enu=%.2fdeg",
      home_->x, home_->y, home_->z,
      "route_yaml", locked_compass_deg_,
      measured_compass_deg,
      compass_yaw_enu * 180.0 / kPi, odom_yaw_enu * 180.0 / kPi,
      mission_yaw_ * 180.0 / kPi);
  }

  void build_search_route()
  {
    search_route_.clear();
    effective_search_lane_count_ = 2;
    const double first_x =
      drop_near_x_m_ + search_edge_margin_m_ + 0.5;
    const double field_far_x = drop_near_x_m_ + drop_length_x_m_;
    const double half_y =
      std::max(0.2, drop_width_y_m_ * 0.5 - search_cross_margin_m_);
    const double second_x =
      std::min(field_far_x, first_x + search_forward_step_m_);
    // FLU 任务坐标系：+X 为锁定的机头前方，+Y 为机体左方。
    search_route_.push_back(field_to_local(first_x, half_y, search_alt_m_));
    search_route_.push_back(field_to_local(first_x, -half_y, search_alt_m_));
    search_route_.push_back(field_to_local(second_x, -half_y, search_alt_m_));
    search_route_.push_back(field_to_local(second_x, half_y, search_alt_m_));
    RCLCPP_INFO(
      get_logger(),
      "SEARCH_ROUTE pattern=left_to_right_forward_right_to_left "
      "lanes=%d waypoints=%zu altitude=%.2f forward_step=%.2f",
      effective_search_lane_count_, search_route_.size(),
      search_alt_m_, second_x - first_x);
  }

  void build_recon_route()
  {
    recon_route_.clear();
    const double near_x =
      recon_center_x_m_ - recon_length_x_m_ * 0.5 + recon_edge_margin_m_;
    const double far_x =
      recon_center_x_m_ + recon_length_x_m_ * 0.5 - recon_edge_margin_m_;
    const double half_y =
      std::max(0.2, recon_width_y_m_ * 0.5 - recon_cross_margin_m_);
    recon_entry_ = field_to_local(near_x, 0.0, recon_alt_m_);
    for (int lane = 0; lane < recon_lane_count_; ++lane) {
      const double fraction = recon_lane_count_ == 1 ?
        0.5 :
        static_cast<double>(lane) /
        static_cast<double>(recon_lane_count_ - 1);
      const double field_x = near_x + (far_x - near_x) * fraction;
      const bool forward = lane % 2 == 0;
      recon_route_.push_back(
        field_to_local(
          field_x, forward ? -half_y : half_y, recon_alt_m_));
      recon_route_.push_back(
        field_to_local(
          field_x, forward ? half_y : -half_y, recon_alt_m_));
    }
    RCLCPP_INFO(
      get_logger(),
      "RECON_ROUTE lanes=%d waypoints=%zu center_x=%.2f size=%.2fx%.2f "
      "altitude=%.2f entry_field=(%.2f,0.00) waypoint_only=true "
      "hazard_visual=false",
      recon_lane_count_, recon_route_.size(), recon_center_x_m_,
      recon_length_x_m_, recon_width_y_m_, recon_alt_m_, near_x);
  }

  std::vector<Point3> rounded_route(
    const std::vector<Point3> & input, double radius) const
  {
    if (input.size() < 3U) {
      return input;
    }
    std::vector<Point3> result;
    result.reserve(input.size() * 5U);
    result.push_back(input.front());
    for (std::size_t i = 1U; i + 1U < input.size(); ++i) {
      const Point3 & prev = input[i - 1U];
      const Point3 & corner = input[i];
      const Point3 & next = input[i + 1U];
      const double in_len = distance_xy(prev, corner);
      const double out_len = distance_xy(corner, next);
      const double cut = std::min({radius, in_len * 0.35, out_len * 0.35});
      if (cut < 0.05) {
        result.push_back(corner);
        continue;
      }
      const double in_x = (corner.x - prev.x) / in_len;
      const double in_y = (corner.y - prev.y) / in_len;
      const double out_x = (next.x - corner.x) / out_len;
      const double out_y = (next.y - corner.y) / out_len;
      const Point3 entry{corner.x - in_x * cut, corner.y - in_y * cut, corner.z};
      const Point3 exit{corner.x + out_x * cut, corner.y + out_y * cut, corner.z};
      result.push_back(entry);
      for (int s = 1; s <= 3; ++s) {
        const double t = static_cast<double>(s) / 4.0;
        const double a = (1.0 - t) * (1.0 - t);
        const double b = 2.0 * (1.0 - t) * t;
        const double c = t * t;
        result.push_back(Point3{
          a * entry.x + b * corner.x + c * exit.x,
          a * entry.y + b * corner.y + c * exit.y,
          corner.z});
      }
      result.push_back(exit);
    }
    result.push_back(input.back());
    return result;
  }

  void tick()
  {
    check_service_results();
    check_servo_results();
    handle_abort_stow();
    initialize_servos_if_ready();
    process_pending_vision_frames();
    monitor_visual_health();

    if (publish_setpoint_ && frame_locked_) {
      publish_setpoint();
    }
    if (mission_timeout_s_ > 0.0 && mission_started_ && mission_timeout_applies(state_) &&
      steady_age_s(mission_start_time_) > mission_timeout_s_)
    {
      if (state_ == State::RELEASE && uses_servo() &&
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
    if (drop_phase_start_time_.has_value() && !drop_phase_fixed_release_active_ &&
      payload_index_ < static_cast<std::size_t>(payload_count_) &&
      (state_ == State::SEARCH || state_ == State::ALIGN) &&
      steady_age_s(*drop_phase_start_time_) >= drop_phase_timeout_s_)
    {
      start_drop_phase_timeout_release();
      return;
    }
    if (steady_age_s(last_status_time_) >= 4.0) {
      const SteadyTimePoint status_time = SteadyClock::now();
      const double nav_window_s = std::max(
        1.0e-6,
        std::chrono::duration<double>(
          status_time - nav_rate_window_start_).count());
      last_status_time_ = status_time;
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
      RCLCPP_INFO(
        get_logger(),
        "NAV_STREAM window=%.2fs odom_hz=%.2f odom_gap_max=%.3fs "
        "odom_age=%.3fs odom_pos=(%.3f,%.3f,%.3f) "
        "odom_rpy=(%.3f,%.3f,%.3f) pose_hz=%.2f pose_gap_max=%.3fs "
        "pose_age=%.3fs pose_pos=(%.3f,%.3f,%.3f) "
        "pose_q=(%.4f,%.4f,%.4f,%.4f)",
        nav_window_s,
        static_cast<double>(odom_message_count_) / nav_window_s,
        odom_max_gap_s_, odom_age_s(), position_.x, position_.y, position_.z,
        current_roll_, current_pitch_, current_vehicle_yaw_,
        static_cast<double>(pose_message_count_) / nav_window_s,
        pose_max_gap_s_, pose_age_s(), latest_local_pose_.position.x,
        latest_local_pose_.position.y, latest_local_pose_.position.z,
        latest_local_pose_.orientation.x, latest_local_pose_.orientation.y,
        latest_local_pose_.orientation.z, latest_local_pose_.orientation.w);
      odom_message_count_ = 0U;
      pose_message_count_ = 0U;
      odom_max_gap_s_ = 0.0;
      pose_max_gap_s_ = 0.0;
      nav_rate_window_start_ = status_time;
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
          lock_frame();
          publish_setpoint_ = true;
          enter(State::PRESTREAM);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "LOCK_FRAME odom=%s compass_advisory=%s heading_advisory=%s stationary=%s "
            "vision=%s servo=%s",
            odom_fresh() ? "yes" : "no", compass_fresh() ? "yes" : "no",
            heading_stable() ? "yes" : "no",
            position_stable_since_.has_value() ? "yes" : "no",
            vision_ready_before_takeoff() ? "yes" : "no",
            (!uses_servo() || servos_initialized_) ? "yes" : "no");
        }
        break;

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!odom_fresh() || !vision_ready_before_takeoff())
        {
          fail_and_land("Prestream odometry or vision gate lost");
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
          if (uses_servo() && !servos_initialized_) {
            fail_and_land("Armed before servo initialization ACK");
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
          if (fixed_release_test_mode_) {
            start_fixed_release_test();
          } else {
            start_search();
          }
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
          !virtual_release_pending_ && !release_abort_requested_)
        {
          if (steady_age_s(state_enter_time_) >= 1.0) {
            RCLCPP_WARN(
              get_logger(), "PILOT_OVERRIDE node exiting; pilot owns flight");
            rclcpp::shutdown();
          }
        } else if (
          steady_age_s(state_enter_time_) >
          2.0 * std::max(servo_ack_timeout_s_, virtual_release_ack_timeout_s_) +
          2.0)
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
            get_logger(), "CUADC_FULL_MISSION_ABORT reason=%s",
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
    return true;
  }

  void monitor_visual_health()
  {
    // 视觉中断时，SEARCH 继续沿原航线运行；
    // 恢复由正常帧处理完成，不增加悬停或重新获取目标的门槛。
  }

  void start_search()
  {
    if (!drop_phase_start_time_.has_value()) {
      drop_phase_start_time_ = SteadyClock::now();
    }
    active_bucket_.reset();
    alignment_stable_since_.reset();
    ranking_stable_since_.reset();
    if (search_pass_ == 0 && use_post_search_cluster_fit_) {
      search_observations_.clear();
      clustered_target_tracks_.clear();
      cluster_fit_plan_active_ = false;
    }
    search_index_ = 0U;
    ++search_pass_;
    if (search_route_.empty()) {
      fail_and_return("Search route is empty");
      return;
    }
    search_vision_acquire_pending_ = false;
    enter(State::SEARCH);
    const double segment_speed =
      search_pass_ == 1 && search_index_ == 0U ?
      transit_speed_m_s_ : search_speed_m_s_;
    start_segment(position_, search_route_[search_index_], segment_speed);
    RCLCPP_INFO(
      get_logger(),
      "SEARCH_START pass=%d/%d lanes=%d waypoints=%zu altitude=%.2f "
      "route=fixed_order vision_recovery=route_continues",
      search_pass_, max_search_passes_, effective_search_lane_count_,
      search_route_.size(), search_alt_m_);
  }

  void continue_release_from_coarse_coordinate(const std::string & phase)
  {
    middle_calibration_active_ = false;
    middle_calibration_stable_since_.reset();
    coarse_coordinate_fallback_active_ = true;
    release_uses_coarse_fallback_ = true;
    target_ = Point3{position_.x, position_.y,
      home_.has_value() ? home_->z + fine_alt_m_ : position_.z};
    coarse_fallback_release_pose_ = target_;
    RCLCPP_WARN(
      get_logger(),
      "DROP_VISION_LOST phase=%s; continuing release from last target coordinate "
      "pose=(%.3f,%.3f,%.3f)",
      phase.c_str(), target_.x, target_.y, target_.z);
  }

  void start_fixed_release_test(
    bool direct_release = false, std::size_t start_payload = 0U)
  {
    clear_target_plan();
    clustered_target_tracks_.clear();
    selected_target_ids_.clear();
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    selected_target_confidences_.clear();

    const SteadyTimePoint arrival = SteadyClock::now();
    for (std::size_t index = 0U; index < 2U; ++index) {
      BucketTrack target;
      target.id = 9001U + index;
      target.local = field_to_local(
        fixed_release_points_field_xy_[index * 2U],
        fixed_release_points_field_xy_[index * 2U + 1U],
        0.0);
      target.diameter = 0.20;
      target.confidence = 1.0;
      target.confirmations = 1U;
      target.stamp = now();
      target.arrival = arrival;
      target.frozen_memory = true;
      clustered_target_tracks_.push_back(target);
      selected_target_ids_.push_back(target.id);
      selected_target_positions_.push_back(target.local);
      selected_target_diameters_.push_back(target.diameter);
      selected_target_confidences_.push_back(target.confidence);
    }
    cluster_fit_plan_active_ = true;
    target_plan_locked_ = true;
    fixed_release_direct_active_ = direct_release;
    payload_index_ = std::min(
      start_payload, clustered_target_tracks_.size() - 1U);
    active_bucket_ = clustered_target_tracks_[payload_index_];
    start_fixed_target_transit();
    RCLCPP_WARN(
      get_logger(),
      "FIXED_RELEASE_TEST_START vision_coordinates_ignored=true "
      "points_field=(%.2f,%.2f)/(%.2f,%.2f) recon_after_release=%s",
      fixed_release_points_field_xy_[0], fixed_release_points_field_xy_[1],
      fixed_release_points_field_xy_[2], fixed_release_points_field_xy_[3],
      drop_only_mode_ ? "false" : "true");
  }

  void start_drop_phase_timeout_release()
  {
    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      return;
    }
    drop_phase_fixed_release_active_ = true;
    RCLCPP_WARN(
      get_logger(),
      "DROP_PHASE_TIMEOUT elapsed=%.1fs; fixed release from payload=%zu",
      steady_age_s(*drop_phase_start_time_), payload_index_ + 1U);
    start_fixed_release_test(true, payload_index_);
  }

  void start_fixed_target_transit()
  {
    if (!active_bucket_.has_value()) {
      fail_and_return("Fixed-release transit has no active target");
      return;
    }
    const Point3 destination = desired_release_pose(
      fixed_release_direct_active_ ? search_alt_m_ : coarse_alt_m_);
    start_segment(position_, destination, drop_target_transit_speed_m_s_);
    fixed_release_transit_active_ = true;
    target_ = position_;
    enter(State::ALIGN);
    const Point3 field = local_to_field(active_bucket_->local);
    RCLCPP_WARN(
      get_logger(),
      "FIXED_RELEASE_TRANSIT payload=%zu field=(%.2f,%.2f) "
      "aircraft_target=(%.2f,%.2f,%.2f)",
      payload_index_ + 1U, field.x, field.y,
      destination.x, destination.y, destination.z);
  }

  bool update_fixed_target_transit()
  {
    if (!fixed_release_transit_active_) {
      return false;
    }
    target_ = sample_segment();
    if (segment_complete(accept_radius_m_)) {
      fixed_release_transit_active_ = false;
      state_enter_time_ = SteadyClock::now();
      if (fixed_release_direct_active_) {
        frozen_release_pose_ = target_;
        enter(State::RELEASE);
      }
      RCLCPP_WARN(
        get_logger(),
        "FIXED_RELEASE_TRANSIT_COMPLETE payload=%zu; beginning release descent",
        payload_index_ + 1U);
    }
    return true;
  }

  void update_search()
  {
    if (search_vision_acquisition_pending()) {
      target_ = search_vision_hold_position_;
      const std::size_t new_aligned_frames = search_new_aligned_frame_count();
      if (vision_heartbeat_fresh())
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
        const std::string reason = search_vision_reacquire_reason_;
        search_vision_acquire_pending_ = false;
        start_segment(
          position_, search_route_[search_index_], search_speed_m_s_);
        RCLCPP_WARN(
          get_logger(),
          "SEARCH_VISION_REACQUIRE_TIMEOUT reason=%s; continuing route index=%zu",
          reason.c_str(), search_index_);
        search_vision_reacquire_reason_.clear();
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

    const auto candidate = use_post_search_cluster_fit_ ?
      std::optional<BucketTrack>{} : current_target_for_payload();
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
    } else if (use_post_search_cluster_fit_) {
      if (try_lock_post_search_cluster_plan()) {
        const auto clustered = current_target_for_payload();
        if (!clustered.has_value()) {
          fail_and_return("Clustered targets unavailable after search");
          return;
        }
        active_bucket_ = clustered;
        alignment_stable_since_.reset();
        enter(State::ALIGN);
        RCLCPP_INFO(
          get_logger(),
          "POST_SEARCH_CLUSTER_ALIGN payload=%zu id=%zu diameter=%.3f confidence=%.2f",
          payload_index_ + 1U, clustered->id, clustered->diameter,
          clustered->confidence);
      } else if (max_search_passes_ == 0 || search_pass_ < max_search_passes_) {
        RCLCPP_WARN(
          get_logger(),
          "POST_SEARCH_CLUSTER_FIRST_PASS_UNUSABLE observations=%zu; starting reverse scan pass %d/%d",
          search_observations_.size(), search_pass_ + 1, max_search_passes_);
        start_search();
      } else if (fixed_release_after_search_) {
        start_fixed_release_test(true);
      } else {
        fail_and_return("No usable clustered targets after maximum search passes");
      }
    } else if (try_lock_target_plan(true)) {
      const auto selected = current_target_for_payload();
      if (!selected.has_value()) {
        fail_and_return("Selected two-bucket plan unavailable");
        return;
      }
      active_bucket_ = selected;
      alignment_stable_since_.reset();
      enter(State::ALIGN);
      RCLCPP_INFO(
        get_logger(),
        "TWO_TARGETS_AT_PASS_END_ALIGN payload=%zu id=%zu diameter=%.3f",
        payload_index_ + 1U, selected->id, selected->diameter);
    } else if (max_search_passes_ == 0 || search_pass_ < max_search_passes_) {
      start_search();
    } else if (fixed_release_after_search_) {
      start_fixed_release_test(true);
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

  Point3 visual_release_target(double relative_altitude_m) const
  {
    if (!active_bucket_.has_value() || !home_.has_value()) {
      return Point3{position_.x, position_.y, home_.has_value() ?
        home_->z + relative_altitude_m : position_.z};
    }
    const std::size_t start = payload_index_ * 3U;
    if (start + 1U >= release_offsets_frd_.size()) {
      return Point3{position_.x, position_.y, home_->z + relative_altitude_m};
    }
    // active_bucket_->body 使用内部 FLU 坐标。修正量为桶位置减去
    // 十字准星位置；桶位于投放点投影的右方或前方时，
    // 控制飞机向相同方向修正。
    const double error_x = active_bucket_->rim_body.x - release_offsets_frd_[start];
    const double error_y = active_bucket_->rim_body.y + release_offsets_frd_[start + 1U];
    const double cr = std::cos(current_roll_);
    const double sr = std::sin(current_roll_);
    const double cp = std::cos(current_pitch_);
    const double sp = std::sin(current_pitch_);
    const double x_pitch = cp * error_x + sp * sr * error_y;
    const double y_pitch = cr * error_y;
    return Point3{
      position_.x + std::cos(mission_yaw_) * x_pitch - std::sin(mission_yaw_) * y_pitch,
      position_.y + std::sin(mission_yaw_) * x_pitch + std::cos(mission_yaw_) * y_pitch,
      home_->z + relative_altitude_m};
  }

  double visual_release_alignment_error() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const std::size_t start = payload_index_ * 3U;
    if (start + 1U >= release_offsets_frd_.size()) {
      return std::numeric_limits<double>::infinity();
    }
    const double error_x = active_bucket_->rim_body.x - release_offsets_frd_[start];
    const double error_y = active_bucket_->rim_body.y + release_offsets_frd_[start + 1U];
    return std::hypot(error_x, error_y);
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
    if (!active_bucket_.has_value() || released_target(active_bucket_->id)) {
      fail_and_return("ALIGN entered without an active unreleased target");
      return;
    }
    if ((fixed_release_test_mode_ || fixed_release_direct_active_) &&
      update_fixed_target_transit()) {
      return;
    }
    // 下降过程中持续根据实时视觉目标修正位置。
    // 发布的设定点仍受速度限制，视觉反馈不能绕过
    // 下降速度约束。
    update_coarse_release_alignment();
  }

  void update_coarse_release_alignment()
  {
    // 先到达机械标定的粗对准姿态，随后在下降过程中持续
    // 根据实时视觉目标修正 XY 位置；设定点发布器
    // 同时限制水平和垂直变化速度。
    const Point3 coarse_pose = desired_release_pose(coarse_alt_m_);
    if (!coarse_release_descent_active_) {
      target_ = coarse_pose;
      const bool coarse_reached =
        distance_xy(position_, coarse_pose) <= coarse_error_m_ &&
        std::abs(position_.z - coarse_pose.z) <= 0.15;
      if (!coarse_reached) {
        return;
      }
      coarse_release_descent_active_ = true;
      middle_calibration_active_ = false;
      middle_calibration_stable_since_.reset();
      last_visual_guidance_target_ = coarse_pose;
      coarse_fallback_release_pose_ = Point3{
        position_.x, position_.y, home_->z + fine_alt_m_};
      drop_setpoint_initialized_ = false;
      RCLCPP_WARN(
        get_logger(),
        "FINE_MECHANICAL_DESCENT_BEGIN payload=%zu id=%zu coarse=(%.3f,%.3f,%.3f) "
        "release_altitude=%.3f middle_horizontal_translation=disabled",
        payload_index_ + 1U, active_bucket_->id, coarse_pose.x, coarse_pose.y,
        coarse_pose.z, home_->z + fine_alt_m_);
      return;
    }

    if (coarse_coordinate_fallback_active_) {
      target_ = coarse_fallback_release_pose_;
      const bool release_altitude_reached =
        distance_xy(position_, target_) <= coarse_error_m_ &&
        std::abs(position_.z - target_.z) <= 0.15;
      if (!release_altitude_reached) {
        return;
      }
      frozen_release_pose_ = target_;
      active_bucket_->frozen_memory = true;
      reset_release_cycle();
      enter(State::RELEASE);
      RCLCPP_WARN(
        get_logger(),
        "COARSE_FALLBACK_RELEASE_READY payload=%zu id=%zu "
        "pose=(%.3f,%.3f,%.3f); next RELEASE tick sends servo",
        payload_index_ + 1U, active_bucket_->id, frozen_release_pose_.x,
        frozen_release_pose_.y, frozen_release_pose_.z);
      return;
    }

    const bool refreshed = refresh_active_target_from_current_track();
    const bool visual_valid = refreshed && active_target_observation_fresh() &&
      target_identity_valid(*active_bucket_);
    if (visual_valid) {
      target_ = visual_release_target(fine_alt_m_);
      last_visual_guidance_target_ = target_;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINE_DESCENT_VISION_UPDATE payload=%zu id=%zu target=(%.3f,%.3f,%.3f)",
        payload_index_ + 1U, active_bucket_->id,
        target_.x, target_.y, target_.z);
    } else {
      target_ = last_visual_guidance_target_;
      target_.z = home_->z + fine_alt_m_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINE_DESCENT_VISION_STALE payload=%zu; retaining mechanical XY",
        payload_index_ + 1U);
    }
    const bool release_altitude_reached =
      distance_xy(position_, target_) <= coarse_error_m_ &&
      std::abs(position_.z - target_.z) <= 0.15;
    if (!release_altitude_reached) {
      return;
    }

    frozen_release_pose_ = target_;
    active_bucket_->frozen_memory = true;
    reset_release_cycle();
    enter(State::RELEASE);
    RCLCPP_WARN(
      get_logger(),
      "FINE_DESCENT_READY payload=%zu id=%zu pose=(%.3f,%.3f,%.3f); "
      "next RELEASE tick sends servo",
      payload_index_ + 1U, active_bucket_->id, frozen_release_pose_.x,
      frozen_release_pose_.y, frozen_release_pose_.z);
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

  void reset_release_cycle()
  {
    release_command_pending_ = false;
    release_open_ = false;
    stow_command_pending_ = false;
    stow_complete_ = false;
    release_abort_requested_ = false;
    virtual_release_pending_ = false;
    release_actuation_committed_ = false;
    release_actuation_confirmed_ = false;
    release_cleanup_return_requested_ = false;
    release_cleanup_failure_reason_.clear();
    virtual_release_future_ = {};
    release_stabilization_since_.reset();
  }

  void update_release()
  {
    target_ = frozen_release_pose_;

    if (release_cleanup_return_requested_) {
      if (uses_servo() && stow_complete_ &&
        !release_command_pending_ && !stow_command_pending_)
      {
        release_abort_requested_ = false;
        const std::string reason = release_cleanup_failure_reason_.empty() ?
          "Servo state uncertain; stow confirmed before safe return" :
          release_cleanup_failure_reason_;
        release_cleanup_return_requested_ = false;
        fail_and_return(reason);
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

    if (!release_command_committed()) {
      const bool refreshed = refresh_active_target_from_current_track();
      if (refreshed && !fixed_release_direct_active_ &&
        !release_uses_coarse_fallback_)
      {
        frozen_release_pose_ = visual_release_target(fine_alt_m_);
        target_ = frozen_release_pose_;
      }

      // 在释放高度保持固定的稳定时间窗口。
      // 此期间持续更新 XY 并限制变化速度，因此仍能修正风扰
      // 和少量残余对准误差。计时器只依据经过时间，
      // 到期后即发送释放指令，
      // 不再要求 XY 误差继续减小。
      // ALIGN 可在正常的 0.15 m 到达容差内交接；
      // 飞机实际到达释放平面后，才开始固定 1.5 s 窗口。
      // 在此之前，使用同一固定高度设定点继续完成下降。
      if (!release_stabilization_since_.has_value() &&
        std::abs(position_.z - frozen_release_pose_.z) > 0.05)
      {
        return;
      }
      if (!release_stabilization_since_.has_value()) {
        release_stabilization_since_ = SteadyClock::now();
        RCLCPP_INFO(
          get_logger(), "RELEASE_STABILIZATION_BEGIN payload=%zu duration=%.2fs",
          payload_index_ + 1U, release_stabilization_s_);
      }
      const double visual_error = refreshed ?
        visual_release_alignment_error() :
        std::numeric_limits<double>::infinity();
      if (visual_error <= release_visual_xy_tolerance_m_) {
        RCLCPP_INFO(
          get_logger(),
          "VISUAL_RELEASE_TOLERANCE payload=%zu error=%.3fm threshold=%.3fm",
          payload_index_ + 1U, visual_error, release_visual_xy_tolerance_m_);
      } else if (steady_age_s(*release_stabilization_since_) < release_stabilization_s_) {
        return;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "VISUAL_RELEASE_TIMEOUT payload=%zu error=%.3fm threshold=%.3fm; forcing release",
          payload_index_ + 1U, visual_error, release_visual_xy_tolerance_m_);
      }

      if (uses_servo()) {
        release_command_pending_ =
          send_servo(payload_index_, true, ServoPurpose::RELEASE);
        if (!release_command_pending_) {
          fail_and_return("Unable to send servo release command");
          return;
        }
        release_actuation_committed_ = true;
        std_msgs::msg::String snapshot;
        snapshot.data = std::to_string(payload_index_ + 1U);
        release_snapshot_pub_->publish(snapshot);
        RCLCPP_INFO(
          get_logger(),
          "DIRECT_RELEASE_COMMAND payload=%zu id=%zu vision_refreshed=%s "
          "pose=(%.3f,%.3f,%.3f)",
          payload_index_ + 1U, active_bucket_->id, refreshed ? "yes" : "no",
          frozen_release_pose_.x, frozen_release_pose_.y, frozen_release_pose_.z);
      } else {
        virtual_release_pending_ = request_virtual_release();
        if (virtual_release_pending_) {
          release_actuation_committed_ = true;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Waiting for /drop_controller/release");
        }
      }
      return;
    }


    if (uses_servo()) {
      if (!release_actuation_confirmed_) {
        return;
      }
      if (!stow_complete_ && release_open_ && !stow_command_pending_) {
        const double duration_s =
          value_at<double>(release_duration_s_, payload_index_, 0.7);
        if (steady_age_s(release_open_time_) >= duration_s) {
          stow_command_pending_ =
            send_servo(payload_index_, false, ServoPurpose::STOW);
          if (!stow_command_pending_) {
            require_stow_before_safe_return(
              "Unable to command servo return-to-stowed");
            return;
          }
        }
      }
      if (release_actuation_confirmed_ && stow_complete_) {
        finish_payload_release();
      }
      return;
    }

    if (release_actuation_confirmed_) {
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
      "PAYLOAD_RELEASE_COMPLETE payload=%zu id=%zu diameter=%.3f backend=%s",
      payload_index_ + 1U, active_bucket_->id, active_bucket_->diameter,
      release_mode_.c_str());
    released_positions_.push_back(active_bucket_->local);
    if (!released_target(active_bucket_->id)) {
      released_target_ids_.push_back(active_bucket_->id);
    }
    ++payload_index_;
    active_bucket_.reset();
    coarse_xy_filter_initialized_ = false;
    reset_release_cycle();
    alignment_stable_since_.reset();

    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      if (drop_only_mode_) {
        RCLCPP_INFO(get_logger(), "DROP_ONLY_COMPLETE; returning to takeoff point");
        start_return_home();
      } else {
        start_recon_climb();
      }
      return;
    }
    const auto last_known = last_known_target_for_payload(payload_index_);
    if (!last_known.has_value() || released_target(last_known->id)) {
      fail_and_return("Frozen second-smallest target ID unavailable");
      return;
    }
    active_bucket_ = *last_known;
      if (fixed_release_test_mode_ || fixed_release_direct_active_) {
      start_fixed_target_transit();
    } else {
      target_ = position_;
      enter(State::ALIGN);
    }
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
    recon_hold_since_.reset();
    recon_entry_transit_active_ = false;
    const double safe_altitude_m = std::max(takeoff_alt_m_, recon_alt_m_);
    start_segment(
      position_,
      Point3{position_.x, position_.y, home_->z + safe_altitude_m},
      recon_transit_speed_m_s_);
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
    // 侦察航线在起飞前冻结；投放完成后直接前往
    // 第一个侦察航点，不绕行到依赖桶位置的入口。
    start_segment(position_, recon_route_.front(), recon_transit_speed_m_s_);
    recon_entry_transit_active_ = false;
    enter(State::RECON_SURVEY);
    const Point3 first_field = local_to_field(recon_route_.front());
    RCLCPP_INFO(
      get_logger(),
      "RECON_FIRST_WAYPOINT_TRANSIT field=(%.2f,%.2f,%.2f) speed=%.2f; "
      "route_frozen_before_takeoff",
      first_field.x, first_field.y, first_field.z, recon_transit_speed_m_s_);
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
      Point3{position_.x, position_.y, home_->z + return_alt_m_},
      return_speed_m_s_);
    enter(State::RETURN_CLIMB);
    RCLCPP_INFO(
      get_logger(), "RETURN_CLIMB target_altitude=%.2f speed=%.2f",
      return_alt_m_, return_speed_m_s_);
  }

  void update_return_climb()
  {
    target_ = sample_segment();
    if (segment_complete(accept_radius_m_)) {
      start_segment(
        position_,
        Point3{home_->x, home_->y, home_->z + return_alt_m_},
        return_speed_m_s_);
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
    const double smooth = ratio;
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

  void initialize_servos_if_ready()
  {
    if (uses_virtual()) {
      servos_initialized_ = true;
      return;
    }
    if (!config_valid_ || !flight_enable_ || servos_initialized_ ||
      initialization_started_ || !fcu_state_.connected ||
      fcu_state_.armed || !command_client_->service_is_ready() ||
      steady_age_s(last_servo_attempt_time_) < 1.0)
    {
      return;
    }
    initialization_started_ = true;
    initialization_failed_ = false;
    initialization_pending_count_ = 0U;
    for (std::size_t index = 0U;
      index < static_cast<std::size_t>(payload_count_); ++index)
    {
      if (send_servo(index, false, ServoPurpose::INITIALIZE)) {
        ++initialization_pending_count_;
      } else {
        initialization_failed_ = true;
      }
    }
    if (initialization_pending_count_ == 0U) {
      initialization_started_ = false;
      RCLCPP_ERROR(get_logger(), "Servo initialization could not be sent");
    }
  }

  bool send_servo(
    std::size_t payload, bool release, ServoPurpose purpose)
  {
    if (!uses_servo() || !command_client_->service_is_ready() ||
      payload >= servo_channels_.size() ||
      payload >= stowed_pwm_.size() || payload >= release_pwm_.size())
    {
      return false;
    }
    const int64_t channel = servo_channels_[payload];
    const int64_t pwm = release ? release_pwm_[payload] : stowed_pwm_[payload];
    if (channel <= 0 || pwm <= 0) {
      return false;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = static_cast<std::uint16_t>(183U);
    request->confirmation = static_cast<std::uint8_t>(0U);
    request->param1 = static_cast<float>(channel);
    request->param2 = static_cast<float>(pwm);
    PendingServoCommand pending;
    pending.purpose = purpose;
    pending.payload = payload;
    pending.sent = SteadyClock::now();
    pending.future = command_client_->async_send_request(request).future.share();
    pending_servo_commands_.push_back(std::move(pending));
    last_servo_attempt_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(),
      "SERVO_COMMAND payload=%zu channel=%lld pwm=%lld purpose=%d",
      payload + 1U, static_cast<long long>(channel),
      static_cast<long long>(pwm), static_cast<int>(purpose));
    return true;
  }

  void check_servo_results()
  {
    for (auto iterator = pending_servo_commands_.begin();
      iterator != pending_servo_commands_.end();)
    {
      const bool ready =
        iterator->future.wait_for(0s) == std::future_status::ready;
      const bool timed_out =
        steady_age_s(iterator->sent) > servo_ack_timeout_s_;
      if (!ready && !timed_out) {
        ++iterator;
        continue;
      }
      const bool accepted = ready && iterator->future.get()->success;
      const ServoPurpose purpose = iterator->purpose;
      const std::size_t payload = iterator->payload;
      RCLCPP_INFO(
        get_logger(),
        "SERVO_ACK payload=%zu purpose=%d accepted=%s timeout=%s",
        payload + 1U, static_cast<int>(purpose),
        accepted ? "true" : "false", timed_out ? "true" : "false");

      if (purpose == ServoPurpose::INITIALIZE) {
        if (initialization_pending_count_ > 0U) {
          --initialization_pending_count_;
        }
        initialization_failed_ = initialization_failed_ || !accepted;
      } else if (purpose == ServoPurpose::RELEASE) {
        release_command_pending_ = false;
        if (accepted) {
          release_actuation_confirmed_ = true;
          release_open_ = true;
          release_open_time_ = SteadyClock::now();
          if (state_ != State::RELEASE || release_abort_requested_ ||
            release_cleanup_return_requested_)
          {
            require_stow_before_safe_return(
              "Late servo release ACK; stow required before handoff");
          }
        } else {
          require_stow_before_safe_return(
            "Servo release rejected or ACK timed out; physical state uncertain");
        }
      } else {
        stow_command_pending_ = false;
        if (accepted) {
          stow_complete_ = true;
          release_open_ = false;
          RCLCPP_INFO(
            get_logger(), "SERVO_STOW_CONFIRMED payload=%zu", payload + 1U);
        } else {
          require_stow_before_safe_return(
            "Servo stow rejected or ACK timed out; retrying before return");
        }
      }
      iterator = pending_servo_commands_.erase(iterator);
    }

    if (initialization_started_ && initialization_pending_count_ == 0U) {
      initialization_started_ = false;
      if (initialization_failed_) {
        RCLCPP_ERROR(
          get_logger(), "Servo initialization failed; retrying while disarmed");
      } else {
        servos_initialized_ = true;
        RCLCPP_INFO(
          get_logger(), "SERVO_INITIALIZED CH7=1100 CH8=1100");
      }
    }
  }

  void handle_abort_stow()
  {
    if (!release_abort_requested_) {
      return;
    }
    if (uses_virtual()) {
      if (!virtual_release_pending_) {
        release_abort_requested_ = false;
      }
      return;
    }
    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      release_abort_requested_ = false;
      release_open_ = false;
      return;
    }
    if (release_command_pending_ || stow_command_pending_) {
      return;
    }
    if (stow_complete_) {
      release_abort_requested_ = false;
      release_open_ = false;
      return;
    }
    if (steady_age_s(last_servo_attempt_time_) < 1.0) {
      return;
    }
    stow_command_pending_ =
      send_servo(payload_index_, false, ServoPurpose::STOW);
  }

  bool request_virtual_release()
  {
    if (!uses_virtual() || !virtual_release_client_->service_is_ready() ||
      virtual_release_future_.valid())
    {
      return false;
    }
    virtual_release_sent_time_ = SteadyClock::now();
    virtual_release_future_ =
      virtual_release_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
    RCLCPP_INFO(
      get_logger(), "VIRTUAL_RELEASE_REQUEST payload=%zu", payload_index_ + 1U);
    return true;
  }

  Point3 rate_limited_drop_target(
    const Point3 & desired, double horizontal_speed_m_s,
    double vertical_speed_m_s)
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
    const double max_horizontal_step_m =
      std::max(0.0, horizontal_speed_m_s * dt_s);
    Point3 commanded = desired;
    const double horizontal_distance_m = std::hypot(dx, dy);
    if (horizontal_distance_m > max_horizontal_step_m &&
      horizontal_distance_m > 1.0e-9)
    {
      const double ratio = max_horizontal_step_m / horizontal_distance_m;
      commanded = Point3{
        last_drop_setpoint_.x + ratio * dx,
        last_drop_setpoint_.y + ratio * dy, desired.z};
    }
    const double max_vertical_step_m = std::max(0.0, vertical_speed_m_s * dt_s);
    const double dz = desired.z - last_drop_setpoint_.z;
    if (std::abs(dz) > max_vertical_step_m) {
      commanded.z = last_drop_setpoint_.z +
        std::copysign(max_vertical_step_m, dz);
    }
    last_drop_setpoint_ = commanded;
    last_drop_setpoint_time_ = current;
    return commanded;
  }

  void publish_setpoint()
  {
    Point3 published_target = target_;
    if (state_ == State::ALIGN && !fixed_release_transit_active_) {
      // 下降时，通过受速度限制的航点队列接收目标更新，
      // 使飞机在航点之间逐步降低高度。
      published_target = rate_limited_drop_target(
        target_, alignment_horizontal_speed_m_s_, descent_vertical_speed_m_s_);
    } else if (state_ == State::RELEASE) {
      // 最终视觉修正必须使用与下降阶段相同的受限设定点流。
      // 不能直接发布刚观测到的十字准星目标，
      // 否则会绕过配置的投放定位速度限制，
      // 并可能在视觉估计改变时造成飞机过冲。
      published_target = rate_limited_drop_target(
        target_, release_positioning_speed_m_s_, descent_vertical_speed_m_s_);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "RELEASE_LIVE_SETPOINT payload=%zu target=(%.3f,%.3f,%.3f)",
        payload_index_ + 1U, published_target.x, published_target.y,
        published_target.z);
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
    setpoint_pub_->publish(message);
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
    if (!takeoff_client_->service_is_ready() || takeoff_future_.valid() ||
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
      takeoff_client_->async_send_request(request).future.share();
    takeoff_sent_ = true;
    RCLCPP_INFO(
      get_logger(), "TAKEOFF_REQUEST altitude=%.2f yaw=%.2f",
      takeoff_alt_m_, locked_compass_deg_);
  }

  void request_land()
  {
    if (!fcu_state_.armed || !land_client_->service_is_ready() ||
      land_future_.valid() || !request_allowed())
    {
      return;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    land_request_time_ = SteadyClock::now();
    land_future_ = land_client_->async_send_request(request).future.share();
  }

  void request_arm(bool arm)
  {
    if (!arm_client_->service_is_ready() || arm_future_.valid() ||
      !request_allowed())
    {
      return;
    }
    if (arm) {
      if (!auto_arm_on_guided_ || state_ != State::WAIT_ARM ||
        !guided_active_ || !vision_ready_before_takeoff() ||
        (uses_servo() && !servos_initialized_))
      {
        return;
      }
    } else if (!landing_confirmation_ready()) {
      return;
    }
    auto request =
      std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    pending_arm_value_ = arm;
    mark_request();
    arm_request_time_ = SteadyClock::now();
    arm_future_ = arm_client_->async_send_request(request).future.share();
  }

  void check_service_results()
  {
    check_virtual_release_result();

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

  void check_virtual_release_result()
  {
    if (!virtual_release_future_.valid()) {
      return;
    }
    const bool ready =
      virtual_release_future_.wait_for(0s) == std::future_status::ready;
    const bool timed_out =
      steady_age_s(virtual_release_sent_time_) >
      virtual_release_ack_timeout_s_;
    if (!ready && !timed_out) {
      return;
    }
    if (ready) {
      const auto response = virtual_release_future_.get();
      virtual_release_future_ = {};
      virtual_release_pending_ = false;
      RCLCPP_INFO(
        get_logger(), "VIRTUAL_RELEASE_ACK success=%s message=%s",
        response->success ? "true" : "false", response->message.c_str());
      if (response->success && state_ == State::RELEASE &&
        !release_abort_requested_)
      {
        release_actuation_confirmed_ = true;
        release_open_ = true;
        stow_complete_ = true;
        release_open_time_ = SteadyClock::now();
      } else if (response->success) {
        release_actuation_confirmed_ = true;
        mark_failure("Late virtual release ACK after state change");
      } else if (state_ == State::PILOT_OVERRIDE) {
        release_abort_requested_ = false;
        mark_failure(
          "Virtual drop judge rejected release after pilot override");
        RCLCPP_ERROR(
          get_logger(),
          "PILOT_OVERRIDE virtual release rejected; remaining pilot-owned");
      } else {
        fail_and_return("Virtual drop judge rejected release");
      }
    } else {
      virtual_release_future_ = {};
      virtual_release_pending_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "Virtual release ACK timeout; late physical/sim result is uncertain");
      if (state_ == State::PILOT_OVERRIDE) {
        release_abort_requested_ = false;
        mark_failure("Virtual release ACK timeout after pilot override");
        RCLCPP_ERROR(
          get_logger(),
          "PILOT_OVERRIDE virtual release timed out; remaining pilot-owned");
      } else if (state_ == State::RELEASE) {
        fail_and_return("Virtual release ACK timeout");
      } else {
        release_abort_requested_ = false;
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
      // 新鲜且明确的空中、起飞或降落状态报告具有优先权。
      // 不能因里程计高度较低或被重置，就覆盖该报告而进入 DISARM。
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
        "CUADC_FULL_MISSION_SAFE_ABORT reason=%s armed=%s mode=%s "
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
      release_open_ || stow_command_pending_ ||
      virtual_release_pending_))
    {
      release_abort_requested_ = true;
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
      (drop_only_mode_ || recon_completed_) && landing_confirmation_ready();
    if (success) {
      if (drop_only_mode_) {
        std::cout << "任务完成：双投放、返航、降落、上锁" << std::endl;
        RCLCPP_INFO(
          get_logger(),
          "CUADC_DROP_ONLY_SUCCESS payloads=2 landed=true disarmed=true");
      } else {
        std::cout <<
          "任务完成：双投放、危险物区域6端点遍历、返航、降落、上锁" <<
          std::endl;
        RCLCPP_INFO(
          get_logger(),
          "CUADC_FULL_MISSION_SUCCESS payloads=2 recon_waypoints=%zu "
          "landed=true disarmed=true",
          recon_route_.size());
      }
    } else {
      const std::string reason = terminal_reason_.empty() ?
        "terminal success conditions not met" : terminal_reason_;
      RCLCPP_ERROR(
        get_logger(), "CUADC_FULL_MISSION_SAFE_ABORT reason=%s",
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
    if (next == State::ALIGN) {
      coarse_release_descent_active_ = false;
      coarse_coordinate_fallback_active_ = false;
      release_uses_coarse_fallback_ = false;
      middle_calibration_active_ = false;
      middle_calibration_stable_since_.reset();
      drop_setpoint_initialized_ = false;
    }
    state_ = next;
    state_enter_time_ = SteadyClock::now();
    if (mission_state_pub_) {
      std_msgs::msg::String message;
      message.data = state_name(state_);
      mission_state_pub_->publish(message);
    }
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
  double configured_geographic_heading_deg_ = -1.0;
  bool yaw_to_target_ = false;
  bool auto_arm_on_guided_ = false;
  bool drop_only_mode_ = false;
  bool coarse_release_trial_mode_ = false;
  bool fixed_release_test_mode_ = false;
  bool fixed_release_after_search_ = false;
  bool fixed_release_direct_active_ = false;
  bool drop_phase_fixed_release_active_ = false;
  bool coarse_xy_filter_initialized_ = false;
  bool fallback_release_on_incomplete_bucket_set_ = false;
  bool pending_arm_value_ = false;
  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_pose_ = false;
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
  bool initialize_stowed_ = true;
  bool return_to_stowed_ = true;
  bool servos_initialized_ = false;
  bool initialization_started_ = false;
  bool initialization_failed_ = false;
  bool release_command_pending_ = false;
  bool release_open_ = false;
  bool stow_command_pending_ = false;
  bool stow_complete_ = false;
  bool release_abort_requested_ = false;
  bool virtual_release_pending_ = false;
  bool release_actuation_committed_ = false;
  bool release_actuation_confirmed_ = false;
  bool release_cleanup_return_requested_ = false;

  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  geometry_msgs::msg::Pose latest_local_pose_;
  Point3 position_;
  Point3 target_;
  Point3 search_vision_hold_position_;
  Point3 frozen_release_pose_;
  Point3 coarse_fallback_release_pose_;
  Point3 recon_entry_;
  std::optional<Point3> home_;
  Segment segment_;
  std::optional<BucketTrack> active_bucket_;
  Point3 coarse_xy_filter_;
  std::optional<SteadyTimePoint> position_stable_since_;
  std::optional<SteadyTimePoint> ranking_stable_since_;
  std::optional<SteadyTimePoint> alignment_stable_since_;
  std::optional<SteadyTimePoint> recon_hold_since_;
  std::optional<SteadyTimePoint> landing_stable_since_;
  std::optional<rclcpp::Time> last_queued_vision_stamp_;
  std::optional<rclcpp::Time> last_vision_capture_stamp_;
  std::deque<HeadingSample> heading_samples_;
  std::deque<NavigationSample> navigation_history_;
  std::deque<PendingVisionFrame> pending_vision_frames_;
  std::vector<BucketTrack> known_buckets_;
  std::vector<BucketTrack> latest_frame_detections_;
  std::vector<BucketTrack> search_observations_;
  std::vector<BucketTrack> clustered_target_tracks_;
  std::vector<Point3> released_positions_;
  std::vector<std::size_t> released_target_ids_;
  std::vector<std::size_t> selected_target_ids_;
  std::vector<Point3> selected_target_positions_;
  std::vector<double> selected_target_diameters_;
  std::vector<double> selected_target_confidences_;
  std::vector<std::size_t> ranking_candidate_ids_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;
  std::vector<PendingServoCommand> pending_servo_commands_;
  std::ofstream projection_log_;

  std::string bucket_topic_;
  std::string bucket_projection_log_path_;
  std::string release_mode_ = "servo";
  std::string terminal_reason_;
  std::string release_cleanup_failure_reason_;
  std::string search_vision_reacquire_reason_;
  double takeoff_alt_m_ = 3.0;
  double search_alt_m_ = 3.0;
  double coarse_alt_m_ = 2.0;
  double fine_alt_m_ = 0.5;
  double transit_speed_m_s_ = 8.5;
  double drop_target_transit_speed_m_s_ = 6.0;
  double recon_transit_speed_m_s_ = 9.5;
  double return_speed_m_s_ = 9.0;
  double return_alt_m_ = 2.5;
  double search_speed_m_s_ = 1.75;
  double search_forward_step_m_ = 2.5;
  double recon_speed_m_s_ = 1.5;
  double drop_approach_speed_m_s_ = 5.0;
  double descent_vertical_speed_m_s_ = 0.2;
  double alignment_horizontal_speed_m_s_ = 0.5;
  double release_positioning_speed_m_s_ = 0.2;
  double release_stabilization_s_ = 2.0;
  double release_visual_xy_tolerance_m_ = 0.03;
  double middle_calibration_error_m_ = 0.12;
  double middle_calibration_stable_s_ = 0.50;
  double min_segment_s_ = 1.0;
  double accept_radius_m_ = 0.4;
  double drop_near_x_m_ = 30.0;
  double drop_length_x_m_ = 5.0;
  double drop_width_y_m_ = 8.0;
  double cross_track_camera_fov_rad_ = 42.0 * kPi / 180.0;
  double search_lane_overlap_ratio_ = 0.30;
  double search_edge_margin_m_ = 0.35;
  double search_cross_margin_m_ = 0.55;
  double cluster_position_scale_m_ = 0.60;
  double cluster_diameter_scale_m_ = 0.06;
  bool motion_compensation_enabled_ = true;
  std::vector<double> motion_compensation_matrix_ =
    {0.85335529, 0.80596326, -0.97181809, 0.97507692};
  std::vector<double> motion_compensation_reference_field_xy_ = {31.75, 0.0};
  std::vector<double> bucket_diameter_centers_m_ = {0.145, 0.195, 0.235};
  double recon_center_x_m_ = 57.45;
  double recon_length_x_m_ = 2.4;
  double recon_width_y_m_ = 8.0;
  double recon_alt_m_ = 2.0;
  double recon_edge_margin_m_ = 0.35;
  double recon_cross_margin_m_ = 0.55;
  double recon_waypoint_hold_s_ = 1.0;
  double track_gate_m_ = 0.45;
  double diameter_track_gate_m_ = 0.08;
  double consecutive_track_position_gate_m_ = 0.50;
  double track_max_gap_s_ = 0.60;
  double selection_max_age_s_ = 10.0;
  double distinct_min_separation_m_ = 0.14;
  double distinct_min_diameter_m_ = 0.025;
  double ranking_stable_s_ = 0.80;
  double known_memory_s_ = 120.0;
  double released_exclusion_m_ = 0.25;
  double position_filter_alpha_ = 0.25;
  double body_filter_alpha_ = 1.0;
  double diameter_filter_alpha_ = 0.20;
  double confidence_filter_alpha_ = 0.30;
  double max_position_deviation_m_ = 0.25;
  double max_diameter_deviation_m_ = 0.050;
  double min_track_confidence_ = 0.25;
  double coarse_error_m_ = 0.15;
  double fine_error_m_ = 0.08;
  double coarse_stable_s_ = 0.8;
  double alignment_timeout_s_ = 12.0;
  double detection_timeout_s_ = 3.0;
  double target_guidance_max_age_s_ = 0.5;
  double payload_transition_hold_s_ = 0.5;
  double vision_heartbeat_timeout_s_ = 1.5;
  double vision_max_pipeline_delay_s_ = 1.5;
  double vision_future_tolerance_s_ = 0.05;
  double vision_transform_tolerance_s_ = 0.20;
  double nav_interpolation_max_gap_s_ = 0.50;
  double odom_history_s_ = 5.0;
  double vision_pending_buffer_s_ = 1.00;
  double search_vision_acquire_timeout_s_ = 5.0;
  double prestream_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double drop_phase_timeout_s_ = 150.0;
  double return_climb_timeout_s_ = 30.0;
  double return_home_timeout_s_ = 120.0;
  double land_timeout_s_ = 120.0;
  double disarm_timeout_s_ = 20.0;
  double odom_timeout_s_ = 1.5;
  double compass_timeout_s_ = 1.0;
  double extended_state_timeout_s_ = 2.5;
  double landing_confirm_stable_s_ = 1.5;
  double landing_max_relative_altitude_m_ = 0.30;
  double landing_max_horizontal_speed_m_s_ = 0.20;
  double landing_max_vertical_speed_m_s_ = 0.15;
  double heading_stability_s_ = 2.0;
  double heading_max_variation_rad_ = 2.0 * kPi / 180.0;
  double position_stability_s_ = 2.0;
  double stationary_speed_m_s_ = 0.15;
  double service_ack_timeout_s_ = 3.0;
  double servo_ack_timeout_s_ = 3.0;
  double virtual_release_ack_timeout_s_ = 3.0;
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
  double odom_max_gap_s_ = 0.0;
  double pose_max_gap_s_ = 0.0;

  int search_lane_count_ = 0;
  int effective_search_lane_count_ = 8;
  int max_search_passes_ = 3;
  int recon_lane_count_ = 2;
  int required_bucket_count_ = 2;
  int min_confirmations_ = 3;
  int consecutive_track_frames_ = 3;
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
  std::size_t vision_heartbeat_count_ = 0U;
  std::size_t odom_message_count_ = 0U;
  std::size_t pose_message_count_ = 0U;
  std::size_t aligned_vision_sequence_ = 0U;
  std::size_t search_vision_entry_aligned_sequence_ = 0U;
  std::size_t initialization_pending_count_ = 0U;
  bool drop_setpoint_initialized_ = false;
  bool use_post_search_cluster_fit_ = false;
  bool cluster_fit_plan_active_ = false;
  bool coarse_release_descent_active_ = false;
  bool coarse_coordinate_fallback_active_ = false;
  bool release_uses_coarse_fallback_ = false;
  bool middle_calibration_active_ = false;
  bool fixed_release_transit_active_ = false;
  bool recon_entry_transit_active_ = false;
  Point3 last_drop_setpoint_;
  Point3 last_visual_guidance_target_;
  SteadyTimePoint last_drop_setpoint_time_;
  std::vector<double> release_offsets_;
  std::vector<double> release_offsets_frd_;
  std::vector<double> fixed_release_points_field_xy_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> stowed_pwm_;
  std::vector<int64_t> release_pwm_;
  std::vector<double> release_duration_s_;

  SteadyTimePoint last_odom_arrival_;
  SteadyTimePoint last_pose_arrival_;
  SteadyTimePoint nav_rate_window_start_;
  SteadyTimePoint last_compass_arrival_;
  SteadyTimePoint last_extended_state_arrival_;
  SteadyTimePoint last_vision_message_arrival_;
  SteadyTimePoint last_aligned_vision_arrival_;
  SteadyTimePoint search_vision_acquire_start_arrival_;
  SteadyTimePoint state_enter_time_;
  std::optional<SteadyTimePoint> middle_calibration_stable_since_;
  std::optional<SteadyTimePoint> drop_phase_start_time_;
  SteadyTimePoint mission_start_time_;
  SteadyTimePoint release_open_time_;
  std::optional<SteadyTimePoint> release_stabilization_since_;
  SteadyTimePoint last_request_time_;
  SteadyTimePoint last_servo_attempt_time_;
  SteadyTimePoint last_status_time_;
  SteadyTimePoint takeoff_request_time_;
  SteadyTimePoint land_request_time_;
  SteadyTimePoint arm_request_time_;
  SteadyTimePoint virtual_release_sent_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr
    extended_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr release_snapshot_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr virtual_release_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture virtual_release_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuadcFullMissionNode>());
  rclcpp::shutdown();
  return 0;
}
