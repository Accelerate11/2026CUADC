/*
 * 问题说明：
 * 1. 程序锁定的航向始终为 0°，并非飞机真实起飞航向。
 * 2. 所有航点因此沿 ENU 地图固定方向生成，导致飞机偏航后朝错误方向飞行。
 * 3. TAKEOFF 请求未显式设置 yaw，默认 0° 可能再次触发偏航。
 * 4. 多个 MAVROS 位置话题同时更新当前位置，产生位置跳变。
 *
 * 应使用 compass_hdg 锁定真实航向，起飞时显式设置 yaw，
 * 并只保留一个主要本地位置源。
 */
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

enum class MissionState {
  WAITING_FCU,
  WAITING_NAV_STABLE,
  PRESTREAM_HOLD,
  SETTING_GUIDED,
  ARMING,
  TAKEOFF,
  SCANNING,
  RETURN_HOME,
  LANDING,
  DONE
};

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ReconTarget {
  std::string id;
  std::string marker;
  Point3 point;
};

struct DropTarget {
  std::string id;
  Point3 point;
  double radius = 0.0;
  int score = 0;
};

struct Waypoint {
  Point3 point;
  double hold_s = 0.0;
  std::string label;
  bool release_payload = false;
  std::size_t payload_index = 0;
  std::string drop_target_id;
};

struct Segment {
  Point3 start;
  Point3 end;
  double duration_s = 1.0;
  rclcpp::Time start_time;
};

struct HeadingSample {
  rclcpp::Time stamp;
  double yaw = 0.0;
};

class CuadcFullMissionNode : public rclcpp::Node {
public:
  CuadcFullMissionNode() : Node("cuadc_full_mission_node") {
    declare_parameter<double>("takeoff_alt", 4.0);
    declare_parameter<double>("cruise_alt", 5.0);
    declare_parameter<double>("drop_hover_alt", 3.0);
    declare_parameter<double>("recon_hover_alt", 3.5);
    declare_parameter<double>("drop_settle_s", 2.0);
    declare_parameter<double>("post_drop_hold_s", 0.8);
    declare_parameter<double>("mission_yaw_rad", 0.0);
    declare_parameter<double>("field_yaw_rad", 0.0);
    declare_parameter<bool>("lock_mission_yaw_to_initial_heading", true);
    declare_parameter<bool>("yaw_to_target", false);
    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("stationary_speed_max_m_s", 0.15);
    declare_parameter<double>("heading_lock_stability_s", 2.0);
    declare_parameter<double>("heading_lock_max_variation_deg", 2.0);
    declare_parameter<double>("prearm_position_tolerance_m", 0.25);
    declare_parameter<double>("prearm_heading_tolerance_deg", 5.0);
    declare_parameter<double>("max_yaw_rate_deg_s", 45.0);
    declare_parameter<double>("waypoint_accept_radius", 0.35);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 420.0);
    declare_parameter<double>("home_lock_stability_s", 2.0);
    declare_parameter<double>("home_lock_max_xy_jump_m", 0.35);
    declare_parameter<double>("home_lock_max_z_jump_m", 0.35);
    declare_parameter<double>("max_abs_local_position_m", 1000.0);
    declare_parameter<double>("service_request_interval_s", 1.0);
    declare_parameter<bool>("require_fcu_home_before_arm", true);
    declare_parameter<double>("trajectory_speed", 2.0);
    declare_parameter<double>("min_segment_time_s", 2.0);
    declare_parameter<double>("bucket_hold_s", 4.0);
    declare_parameter<bool>("use_generated_scene", true);
    declare_parameter<std::string>("generated_scene_path", "");
    declare_parameter<std::string>("scene_coordinate_mode", "gazebo_world_to_mavros_local");
    declare_parameter<std::vector<std::string>>("drop_targets", std::vector<std::string>{"drop_1", "drop_2"});
    declare_parameter<std::vector<double>>("payload_release_offsets_body_m", std::vector<double>{0.0, 0.0, -0.16, 0.0, 0.0, -0.16});
    declare_parameter<std::vector<double>>("payload_drop_bias_body_m", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("camera_offset_body_m", std::vector<double>{0.0, 0.0, -0.08});
    declare_parameter<std::vector<double>>("camera_optical_to_body_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0});
    declare_parameter<std::vector<double>>("camera_target_point_camera_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("camera_target_point_body_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<bool>("use_camera_target_point_body", false);
    declare_parameter<std::string>("release_mode", "virtual");
    declare_parameter<std::vector<int64_t>>("servo_channels", std::vector<int64_t>{7, 8});
    declare_parameter<std::vector<int64_t>>("servo_release_pwm", std::vector<int64_t>{1250, 1250});
    declare_parameter<std::vector<int64_t>>("servo_stowed_pwm", std::vector<int64_t>{1000, 1000});
    declare_parameter<std::vector<double>>("servo_release_duration_s", std::vector<double>{0.7, 0.7});
    declare_parameter<bool>("servo_return_to_stowed", true);
    declare_parameter<bool>("servo_initialize_stowed", true);
    declare_parameter<std::vector<double>>(
      "scan_waypoints_xy",
      std::vector<double>{52.5, -4.0, 52.5, 4.0, 57.5, 4.0, 57.5, -4.0, 55.0, 0.0});

    takeoff_alt_ = get_parameter("takeoff_alt").as_double();
    cruise_alt_ = get_parameter("cruise_alt").as_double();
    drop_hover_alt_ = get_parameter("drop_hover_alt").as_double();
    recon_hover_alt_ = get_parameter("recon_hover_alt").as_double();
    drop_settle_s_ = get_parameter("drop_settle_s").as_double();
    post_drop_hold_s_ = get_parameter("post_drop_hold_s").as_double();
    mission_yaw_rad_ = get_parameter("mission_yaw_rad").as_double();
    field_yaw_rad_ = get_parameter("field_yaw_rad").as_double();
    lock_mission_yaw_to_initial_heading_ =
      get_parameter("lock_mission_yaw_to_initial_heading").as_bool();
    current_yaw_ = mission_yaw_rad_;
    desired_yaw_ = mission_yaw_rad_;
    yaw_to_target_ = get_parameter("yaw_to_target").as_bool();
    prestream_hold_s_ = std::max(0.5, get_parameter("prestream_hold_s").as_double());
    stationary_speed_max_m_s_ =
      std::max(0.01, get_parameter("stationary_speed_max_m_s").as_double());
    heading_lock_stability_s_ =
      std::max(0.5, get_parameter("heading_lock_stability_s").as_double());
    heading_lock_max_variation_rad_ =
      std::max(0.1, get_parameter("heading_lock_max_variation_deg").as_double()) * M_PI / 180.0;
    prearm_position_tolerance_m_ =
      std::max(0.05, get_parameter("prearm_position_tolerance_m").as_double());
    prearm_heading_tolerance_rad_ =
      std::max(0.1, get_parameter("prearm_heading_tolerance_deg").as_double()) * M_PI / 180.0;
    max_yaw_rate_rad_s_ =
      std::max(1.0, get_parameter("max_yaw_rate_deg_s").as_double()) * M_PI / 180.0;
    accept_radius_ = get_parameter("waypoint_accept_radius").as_double();
    takeoff_timeout_s_ = get_parameter("takeoff_timeout_s").as_double();
    mission_timeout_s_ = get_parameter("mission_timeout_s").as_double();
    home_lock_stability_s_ = std::max(0.0, get_parameter("home_lock_stability_s").as_double());
    home_lock_max_xy_jump_m_ =
      std::max(0.01, get_parameter("home_lock_max_xy_jump_m").as_double());
    home_lock_max_z_jump_m_ =
      std::max(0.01, get_parameter("home_lock_max_z_jump_m").as_double());
    max_abs_local_position_m_ =
      std::max(1.0, get_parameter("max_abs_local_position_m").as_double());
    service_request_interval_s_ =
      std::max(0.2, get_parameter("service_request_interval_s").as_double());
    require_fcu_home_before_arm_ = get_parameter("require_fcu_home_before_arm").as_bool();
    trajectory_speed_ = std::max(0.1, get_parameter("trajectory_speed").as_double());
    min_segment_time_s_ = std::max(0.5, get_parameter("min_segment_time_s").as_double());
    bucket_hold_s_ = std::max(0.0, get_parameter("bucket_hold_s").as_double());
    use_generated_scene_ = get_parameter("use_generated_scene").as_bool();
    generated_scene_path_ = get_parameter("generated_scene_path").as_string();
    scene_coordinate_mode_ = get_parameter("scene_coordinate_mode").as_string();
    requested_drop_targets_ = get_parameter("drop_targets").as_string_array();
    payload_release_offsets_body_ = get_parameter("payload_release_offsets_body_m").as_double_array();
    payload_drop_bias_body_ = get_parameter("payload_drop_bias_body_m").as_double_array();
    camera_offset_body_ = get_parameter("camera_offset_body_m").as_double_array();
    camera_optical_to_body_rotation_ = matrix3_param(
      "camera_optical_to_body_rotation",
      std::array<double, 9>{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0});
    camera_target_point_camera_ = get_parameter("camera_target_point_camera_m").as_double_array();
    camera_target_point_body_ = get_parameter("camera_target_point_body_m").as_double_array();
    use_camera_target_point_body_ = get_parameter("use_camera_target_point_body").as_bool();
    release_mode_ = get_parameter("release_mode").as_string();
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    servo_release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    servo_stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    servo_release_duration_s_ = get_parameter("servo_release_duration_s").as_double_array();
    servo_return_to_stowed_ = get_parameter("servo_return_to_stowed").as_bool();
    servo_initialize_stowed_ = get_parameter("servo_initialize_stowed").as_bool();

    if (use_generated_scene_) {
      load_generated_scene();
    }

    fallback_scan_waypoints_ =
      parse_waypoints(get_parameter("scan_waypoints_xy").as_double_array());

    auto best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", best_effort,
      std::bind(&CuadcFullMissionNode::state_cb, this, std::placeholders::_1));
    home_position_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(
      "/mavros/home_position/home", best_effort,
      std::bind(&CuadcFullMissionNode::home_position_cb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", best_effort,
      std::bind(&CuadcFullMissionNode::odom_cb, this, std::placeholders::_1));
    global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/global_position/local", best_effort,
      std::bind(&CuadcFullMissionNode::global_odom_cb, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", best_effort,
      std::bind(&CuadcFullMissionNode::pose_cb, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    release_cli_ = create_client<std_srvs::srv::Trigger>("/drop_controller/release");
    command_long_cli_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    state_enter_time_ = now();
    mission_start_time_ = now();
    last_state_log_ = now();
    last_yaw_update_time_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&CuadcFullMissionNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "CUADC full mission C++ node started; drop_targets=%zu, recon_targets=%zu, coordinate_mode=%s, field_yaw=%.3f rad, lock_initial_yaw=%s",
      drop_targets_.size(), recon_targets_.size(), scene_coordinate_mode_.c_str(),
      field_yaw_rad_, lock_mission_yaw_to_initial_heading_ ? "true" : "false");
    log_mount_calibration();
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
  }

  void home_position_cb(const mavros_msgs::msg::HomePosition::SharedPtr msg) {
    (void)msg;
    if (!have_fcu_home_) {
      RCLCPP_INFO(get_logger(), "FCU home position received from /mavros/home_position/home");
    }
    have_fcu_home_ = true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    have_local_velocity_ = true;
    update_local_pose(
      Point3{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z},
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w,
      "odom", true);
  }

  void global_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    update_local_pose(
      Point3{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z},
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w,
      "global_position/local", false);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    update_local_pose(
      Point3{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w,
      "pose", false);
  }

  void update_local_pose(
    const Point3 & observed, double qx, double qy, double qz, double qw, const char * source,
    bool primary_heading_source)
  {
    if (!valid_local_pose(observed)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid MAVROS local position from %s: (%.3g, %.3g, %.3g)",
        source, observed.x, observed.y, observed.z);
      return;
    }

    if (have_local_position_) {
      const double xy_jump = distance_xy(current_position_, observed);
      const double z_jump = std::abs(current_position_.z - observed.z);
      if (xy_jump > home_lock_max_xy_jump_m_ || z_jump > home_lock_max_z_jump_m_) {
        position_stable_since_ = now();
        RCLCPP_WARN(
          get_logger(),
          "Local %s jumped before home lock: dxy=%.2f m, dz=%.2f m. Waiting %.1f s for RTK/local origin stability.",
          source, xy_jump, z_jump, home_lock_stability_s_);
      }
    } else {
      position_stable_since_ = now();
      RCLCPP_INFO(get_logger(), "First MAVROS local position received from MAVROS %s", source);
    }

    current_position_ = observed;
    have_local_position_ = true;

    const double q_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    if (q_norm_sq > 1e-12 && (primary_heading_source || !have_odom_heading_)) {
      current_heading_rad_ = yaw_from_quaternion(qx, qy, qz, qw);
      have_local_heading_ = true;
      if (primary_heading_source) {
        have_odom_heading_ = true;
      }
      record_heading_sample(current_heading_rad_);
    }
  }

  void record_heading_sample(double yaw) {
    const auto stamp = now();
    heading_samples_.push_back(HeadingSample{stamp, normalize_angle(yaw)});
    const double keep_s = heading_lock_stability_s_ + 0.5;
    while (!heading_samples_.empty() &&
      (stamp - heading_samples_.front().stamp).seconds() > keep_s)
    {
      heading_samples_.pop_front();
    }
  }

  bool heading_stable_for_lock() const {
    if (!have_local_heading_ || heading_samples_.size() < 2U) {
      return false;
    }
    const double span_s =
      (heading_samples_.back().stamp - heading_samples_.front().stamp).seconds();
    if (span_s < heading_lock_stability_s_) {
      return false;
    }
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & sample : heading_samples_) {
      sin_sum += std::sin(sample.yaw);
      cos_sum += std::cos(sample.yaw);
    }
    const double mean = std::atan2(sin_sum, cos_sum);
    double max_variation = 0.0;
    for (const auto & sample : heading_samples_) {
      max_variation =
        std::max(max_variation, std::abs(normalize_angle(sample.yaw - mean)));
    }
    return max_variation <= heading_lock_max_variation_rad_;
  }

  bool navigation_preflight_ready() const {
    const bool stationary =
      !have_local_velocity_ || current_horizontal_speed_m_s_ <= stationary_speed_max_m_s_;
    return current_state_.connected && !current_state_.armed &&
      have_local_position_ && have_local_heading_ &&
      (!require_fcu_home_before_arm_ || have_fcu_home_) &&
      local_position_stable() && heading_stable_for_lock() && stationary;
  }

  bool locked_navigation_preflight_ready() const {
    if (!mission_frame_locked_ || !home_.has_value() || !navigation_preflight_ready()) {
      return false;
    }
    return distance_xyz(current_position_, *home_) <= prearm_position_tolerance_m_ &&
      std::abs(normalize_angle(current_heading_rad_ - mission_heading_rad_)) <=
      prearm_heading_tolerance_rad_;
  }

  void try_lock_takeoff_heading_frame() {
    if (mission_frame_locked_ || mission_state_ != MissionState::WAITING_NAV_STABLE ||
      !navigation_preflight_ready())
    {
      return;
    }

    home_ = current_position_;
    mission_heading_rad_ =
      lock_mission_yaw_to_initial_heading_ ? current_heading_rad_ : mission_yaw_rad_;
    mission_yaw_rad_ = mission_heading_rad_;
    if (lock_mission_yaw_to_initial_heading_) {
      field_yaw_rad_ = mission_heading_rad_;
    }
    current_yaw_ = mission_heading_rad_;
    desired_yaw_ = mission_heading_rad_;
    target_ = *home_;
    mission_frame_locked_ = true;
    last_yaw_update_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "Stable takeoff frame locked: origin=(%.3f, %.3f, %.3f), fixed_yaw=%.2f deg, "
      "field_yaw=%.2f deg; position, stationary and heading-window checks passed",
      home_->x, home_->y, home_->z,
      mission_heading_rad_ * 180.0 / M_PI, field_yaw_rad_ * 180.0 / M_PI);
  }

  void reset_takeoff_frame() {
    mission_frame_locked_ = false;
    home_.reset();
    publish_setpoints_ = false;
    target_ = current_position_;
    current_yaw_ = have_local_heading_ ? current_heading_rad_ : mission_yaw_rad_;
    desired_yaw_ = current_yaw_;
    heading_samples_.clear();
    position_stable_since_ = now();
  }

  bool local_position_stable() const {
    if (!have_local_position_) {
      return false;
    }
    if (home_lock_stability_s_ <= 0.0) {
      return true;
    }
    return seconds_since(position_stable_since_) >= home_lock_stability_s_;
  }

  bool valid_local_pose(const Point3 & p) const {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::abs(p.x) <= max_abs_local_position_m_ &&
      std::abs(p.y) <= max_abs_local_position_m_ &&
      std::abs(p.z) <= max_abs_local_position_m_;
  }

  double relative_altitude() const {
    if (!home_.has_value()) {
      return 0.0;
    }
    return current_position_.z - home_->z;
  }

  double mission_altitude(double relative_altitude_m) const {
    return home_.value_or(Point3{}).z + relative_altitude_m;
  }

  Point3 scene_point_to_local(const Point3 & point, double relative_altitude_m) const {
    Point3 mapped = point;
    if (targets_are_home_relative_ && home_.has_value()) {
      mapped.x += home_->x;
      mapped.y += home_->y;
    }
    mapped.z = mission_altitude(relative_altitude_m);
    return mapped;
  }

  void control_loop() {
    check_service_results();
    try_lock_takeoff_heading_frame();
    update_commanded_yaw();

    if (servo_initialize_stowed_ && !servos_initialized_ && current_state_.connected &&
        (release_mode_ == "servo" || release_mode_ == "both") &&
        command_long_cli_->service_is_ready()) {
      for (std::size_t payload_index = 0; payload_index < servo_channels_.size(); ++payload_index) {
        send_servo_for_payload(payload_index, false);
      }
      servos_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Payload servos initialized to stowed PWM");
    }

    if (publish_setpoints_) {
      publish_setpoint(target_);
    }

    if (
      mission_started_ && seconds_since(mission_start_time_) > mission_timeout_s_ &&
      mission_state_ != MissionState::RETURN_HOME &&
      mission_state_ != MissionState::LANDING &&
      mission_state_ != MissionState::DONE)
    {
      RCLCPP_WARN(get_logger(), "Mission timeout, returning home");
      start_return_home();
    }

    if (seconds_since(last_state_log_) > 5.0) {
      last_state_log_ = now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s, pos=(%.1f, %.1f, %.1f), target=(%.1f, %.1f, %.1f), "
        "yaw_vehicle=%.2fdeg yaw_command=%.2fdeg yaw_locked=%.2fdeg",
        state_name(mission_state_).c_str(), current_position_.x, current_position_.y,
        current_position_.z, target_.x, target_.y, target_.z,
        current_heading_rad_ * 180.0 / M_PI,
        current_yaw_ * 180.0 / M_PI,
        mission_heading_rad_ * 180.0 / M_PI);
    }

    switch (mission_state_) {
      case MissionState::WAITING_FCU:
        publish_setpoints_ = false;
        if (current_state_.connected) {
          enter_state(MissionState::WAITING_NAV_STABLE);
        }
        break;

      case MissionState::WAITING_NAV_STABLE:
        publish_setpoints_ = false;
        if (!current_state_.connected) {
          reset_takeoff_frame();
          enter_state(MissionState::WAITING_FCU);
        } else if (mission_frame_locked_) {
          target_ = home_.value_or(current_position_);
          current_yaw_ = mission_heading_rad_;
          desired_yaw_ = mission_heading_rad_;
          publish_setpoints_ = true;
          enter_state(MissionState::PRESTREAM_HOLD);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting to lock takeoff frame: local=%s heading=%s position_stable=%s "
            "heading_stable=%s stationary=%s fcu_home=%s",
            have_local_position_ ? "yes" : "no",
            have_local_heading_ ? "yes" : "no",
            local_position_stable() ? "yes" : "no",
            heading_stable_for_lock() ? "yes" : "no",
            (!have_local_velocity_ || current_horizontal_speed_m_s_ <= stationary_speed_max_m_s_) ?
            "yes" : "no",
            have_fcu_home_ ? "yes" : "no");
        }
        break;

      case MissionState::PRESTREAM_HOLD:
        if (!locked_navigation_preflight_ready()) {
          RCLCPP_WARN(
            get_logger(),
            "Navigation stability lost before GUIDED; clearing frame and remaining disarmed");
          reset_takeoff_frame();
          enter_state(MissionState::WAITING_NAV_STABLE);
          break;
        }
        target_ = home_.value_or(current_position_);
        desired_yaw_ = mission_heading_rad_;
        if (seconds_since(state_enter_time_) >= prestream_hold_s_) {
          enter_state(MissionState::SETTING_GUIDED);
        }
        break;

      case MissionState::SETTING_GUIDED:
        if (!locked_navigation_preflight_ready()) {
          RCLCPP_WARN(
            get_logger(),
            "Navigation stability lost while setting GUIDED; clearing frame and remaining disarmed");
          reset_takeoff_frame();
          enter_state(MissionState::WAITING_NAV_STABLE);
        } else if (current_state_.mode == "GUIDED") {
          enter_state(MissionState::ARMING);
        } else if (!pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;

      case MissionState::ARMING:
        if (current_state_.armed) {
          mission_start_time_ = now();
          mission_started_ = true;
          enter_state(MissionState::TAKEOFF);
        } else if (!locked_navigation_preflight_ready()) {
          RCLCPP_WARN(
            get_logger(),
            "Navigation stability lost before arming; clearing frame and remaining disarmed");
          reset_takeoff_frame();
          enter_state(MissionState::WAITING_NAV_STABLE);
        } else if (!pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;

      case MissionState::TAKEOFF:
        publish_setpoints_ = false;
        if (!have_local_position_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for /mavros/local_position/odom before takeoff");
          break;
        }
        if (!mission_frame_locked_ || !home_.has_value()) {
          RCLCPP_ERROR(get_logger(), "Takeoff frame was lost after arming; landing immediately");
          enter_state(MissionState::LANDING);
          break;
        }
        desired_yaw_ = mission_heading_rad_;
        if (!takeoff_sent_ && call_takeoff(takeoff_alt_)) {
          takeoff_sent_ = true;
          state_enter_time_ = now();
        }
        if (relative_altitude() >= takeoff_alt_ * 0.9) {
          publish_setpoints_ = true;
          start_scan();
        } else if (seconds_since(state_enter_time_) > takeoff_timeout_s_) {
          RCLCPP_WARN(
            get_logger(), "Takeoff timeout at relative altitude %.2f m, starting scan with current position",
            relative_altitude());
          publish_setpoints_ = true;
          start_scan();
        }
        break;

      case MissionState::SCANNING:
        update_scan_trajectory();
        break;

      case MissionState::RETURN_HOME:
        update_return_trajectory();
        break;

      case MissionState::LANDING:
        publish_setpoints_ = false;
        if (!land_sent_) {
          call_land();
          land_sent_ = true;
        }
        if (!home_.has_value() || relative_altitude() < 0.2) {
          enter_state(MissionState::DONE);
        }
        break;

      case MissionState::DONE:
        RCLCPP_INFO(get_logger(), "Mission complete");
        rclcpp::shutdown();
        break;
    }
  }

  void load_generated_scene() {
    if (generated_scene_path_.empty()) {
      RCLCPP_WARN(get_logger(), "use_generated_scene=true but generated_scene_path is empty");
      return;
    }

    try {
      const YAML::Node scene = YAML::LoadFile(generated_scene_path_);
      const YAML::Node field = scene["field"];
      if (field) {
        log_field_dimensions(field);
      }

      const YAML::Node vehicle = scene["vehicle"];
      if (vehicle && vehicle["pose"] && vehicle["pose"].IsSequence() && vehicle["pose"].size() >= 2) {
        vehicle_spawn_world_.x = vehicle["pose"][0].as<double>();
        vehicle_spawn_world_.y = vehicle["pose"][1].as<double>();
      }

      const YAML::Node drop_targets = scene["drop_targets"];
      if (drop_targets && drop_targets.IsMap()) {
        for (const auto & item : drop_targets) {
          const std::string id = item.first.as<std::string>();
          const YAML::Node node = item.second;
          Point3 p;
          p.x = node["x"].as<double>();
          p.y = node["y"].as<double>();
          p.z = drop_hover_alt_;
          p = transform_scene_point(p);
          DropTarget target;
          target.id = id;
          target.point = p;
          target.radius = node["radius"] ? node["radius"].as<double>() : 0.0;
          target.score = node["score"] ? node["score"].as<int>() : 0;
          drop_targets_.push_back(target);
        }
      } else {
        RCLCPP_WARN(get_logger(), "No drop_targets map found in %s", generated_scene_path_.c_str());
      }

      const YAML::Node targets = scene["recon_targets"];
      if (!targets || !targets.IsMap()) {
        RCLCPP_WARN(get_logger(), "No recon_targets map found in %s", generated_scene_path_.c_str());
        return;
      }

      for (const auto & item : targets) {
        const std::string id = item.first.as<std::string>();
        const YAML::Node node = item.second;
        Point3 p;
        p.x = node["x"].as<double>();
        p.y = node["y"].as<double>();
        p.z = recon_hover_alt_;
        p = transform_scene_point(p);

        recon_targets_.push_back(ReconTarget{
          id,
          node["marker"] ? node["marker"].as<std::string>() : std::string{"unknown"},
          p});
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to load generated scene %s: %s",
        generated_scene_path_.c_str(), ex.what());
    }
  }


  void log_field_dimensions(const YAML::Node & field) const {
    auto log_center_size = [this](const char * name, const YAML::Node & center, const YAML::Node & size) {
      if (center && size && center.IsSequence() && size.IsSequence() && center.size() >= 2 && size.size() >= 2) {
        RCLCPP_INFO(
          get_logger(),
          "Field %s center=(%.2f, %.2f), size=(%.2f x %.2f)",
          name, center[0].as<double>(), center[1].as<double>(), size[0].as<double>(), size[1].as<double>());
      }
    };
    if (field["takeoff_line_x"]) {
      RCLCPP_INFO(get_logger(), "Field takeoff_line_x=%.2f", field["takeoff_line_x"].as<double>());
    }
    log_center_size("drop_area", field["drop_area_center"], field["drop_area_size"]);
    log_center_size("recon_area", field["recon_area_center"], field["recon_area_size"]);
  }

  Point3 transform_scene_point(const Point3 & p) const {
    const auto rotate_field_to_local = [this](const Point3 & point) {
      const double c = std::cos(field_yaw_rad_);
      const double s = std::sin(field_yaw_rad_);
      return Point3{
        c * point.x - s * point.y,
        s * point.x + c * point.y,
        point.z};
    };

    if (scene_coordinate_mode_ == "field_local_home_relative") {
      targets_are_home_relative_ = true;
      return rotate_field_to_local(p);
    }

    if (scene_coordinate_mode_ == "mavros_local") {
      return p;
    }

    if (scene_coordinate_mode_ == "gazebo_enu") {
      return p;
    }

    if (scene_coordinate_mode_ == "gazebo_world_to_mavros_local") {
      targets_are_home_relative_ = true;
      return rotate_field_to_local(Point3{p.x - vehicle_spawn_world_.x, p.y - vehicle_spawn_world_.y, p.z});
    }

    if (scene_coordinate_mode_ == "legacy_lateral_forward") {
      // The original desktop code stored pairs as (lateral, forward), producing
      // points like (-4, 52.5). Gazebo/MAVROS local setpoints here use x=forward,
      // y=lateral, so swap them before publishing.
      targets_are_home_relative_ = true;
      return rotate_field_to_local(Point3{p.y, p.x, p.z});
    }

    RCLCPP_WARN_ONCE(
      get_logger(), "Unknown scene_coordinate_mode=%s; using gazebo_enu",
      scene_coordinate_mode_.c_str());
    return p;
  }

  std::array<double, 9> matrix3_param(
    const std::string & name, const std::array<double, 9> & fallback) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 9) {
      RCLCPP_WARN(
        get_logger(), "%s must contain 9 row-major values; using fallback", name.c_str());
      return fallback;
    }
    std::array<double, 9> matrix{};
    std::copy(values.begin(), values.end(), matrix.begin());
    return matrix;
  }

  static Point3 vector3_from_values(
    const std::vector<double> & values, std::size_t offset, const Point3 & fallback)
  {
    if (values.size() < offset + 3) {
      return fallback;
    }
    return Point3{values[offset + 0], values[offset + 1], values[offset + 2]};
  }

  Point3 camera_mount_offset_body() const {
    return vector3_from_values(camera_offset_body_, 0, Point3{0.0, 0.0, -0.08});
  }

  Point3 body_from_camera_vector(const Point3 & camera_point) const {
    return Point3{
      camera_optical_to_body_rotation_[0] * camera_point.x +
        camera_optical_to_body_rotation_[1] * camera_point.y +
        camera_optical_to_body_rotation_[2] * camera_point.z,
      camera_optical_to_body_rotation_[3] * camera_point.x +
        camera_optical_to_body_rotation_[4] * camera_point.y +
        camera_optical_to_body_rotation_[5] * camera_point.z,
      camera_optical_to_body_rotation_[6] * camera_point.x +
        camera_optical_to_body_rotation_[7] * camera_point.y +
        camera_optical_to_body_rotation_[8] * camera_point.z};
  }

  Point3 body_vector_to_local(const Point3 & body_vector) const {
    const double c = std::cos(mission_yaw_rad_);
    const double s = std::sin(mission_yaw_rad_);
    return Point3{
      c * body_vector.x - s * body_vector.y,
      s * body_vector.x + c * body_vector.y,
      body_vector.z};
  }

  Point3 payload_release_vector_body(std::size_t payload_index) const {
    const std::size_t offset = payload_index * 3;
    const Point3 release = vector3_from_values(
      payload_release_offsets_body_, offset, Point3{0.0, 0.0, -0.16});
    const Point3 bias = vector3_from_values(payload_drop_bias_body_, offset, Point3{0.0, 0.0, 0.0});
    return Point3{release.x + bias.x, release.y + bias.y, release.z + bias.z};
  }

  Point3 camera_view_target_body() const {
    if (use_camera_target_point_body_) {
      return vector3_from_values(camera_target_point_body_, 0, Point3{});
    }

    const Point3 camera_point = vector3_from_values(camera_target_point_camera_, 0, Point3{});
    const Point3 mount = camera_mount_offset_body();
    const Point3 camera_to_target_body = body_from_camera_vector(camera_point);
    return Point3{
      mount.x + camera_to_target_body.x,
      mount.y + camera_to_target_body.y,
      mount.z + camera_to_target_body.z};
  }

  void log_mount_calibration() const {
    const Point3 mount = camera_mount_offset_body();
    const Point3 camera_z_body{
      camera_optical_to_body_rotation_[2],
      camera_optical_to_body_rotation_[5],
      camera_optical_to_body_rotation_[8]};
    const Point3 view_body = camera_view_target_body();
    RCLCPP_INFO(
      get_logger(),
      "Camera mount body FLU offset=(%.3f, %.3f, %.3f), optical +Z in body=(%.3f, %.3f, %.3f), view target body=(%.3f, %.3f, %.3f)",
      mount.x, mount.y, mount.z,
      camera_z_body.x, camera_z_body.y, camera_z_body.z,
      view_body.x, view_body.y, view_body.z);
  }

  Point3 compensated_release_point(const Point3 & bucket_point, std::size_t payload_index) const {
    const Point3 release_body = payload_release_vector_body(payload_index);
    const Point3 release_local = body_vector_to_local(release_body);
    return Point3{bucket_point.x - release_local.x, bucket_point.y - release_local.y, bucket_point.z};
  }

  Point3 compensated_camera_view_point(const Point3 & observed_point) const {
    const Point3 view_body = camera_view_target_body();
    const Point3 view_local = body_vector_to_local(view_body);
    return Point3{observed_point.x - view_local.x, observed_point.y - view_local.y, observed_point.z};
  }

  std::vector<DropTarget> select_drop_targets() const {
    std::vector<DropTarget> selected;
    if (!requested_drop_targets_.empty()) {
      for (const auto & id : requested_drop_targets_) {
        auto it = std::find_if(
          drop_targets_.begin(), drop_targets_.end(),
          [&id](const DropTarget & target) { return target.id == id; });
        if (it != drop_targets_.end()) {
          selected.push_back(*it);
        } else {
          RCLCPP_WARN(get_logger(), "Requested drop target %s was not found", id.c_str());
        }
      }
      return selected;
    }

    selected = drop_targets_;
    std::sort(selected.begin(), selected.end(), [](const DropTarget & a, const DropTarget & b) {
      return a.score > b.score;
    });
    if (selected.size() > 2) {
      selected.resize(2);
    }
    return selected;
  }

  std::vector<Waypoint> make_scan_route(const Point3 & start) const {
    std::vector<Waypoint> route;
    Point3 cursor = start;

    std::vector<DropTarget> selected_drops = select_drop_targets();
    for (std::size_t i = 0; i < selected_drops.size(); ++i) {
      DropTarget target = selected_drops[i];
      target.point = scene_point_to_local(target.point, drop_hover_alt_);
      const Point3 release_point = compensated_release_point(target.point, i);
      const std::string label = target.id + " score=" + std::to_string(target.score);
      const Point3 cruise_point{release_point.x, release_point.y, mission_altitude(cruise_alt_)};
      route.push_back(Waypoint{cruise_point, 0.0, "drop cruise " + label, false, 0, ""});
      route.push_back(Waypoint{release_point, drop_settle_s_, "drop settle " + label, false, 0, ""});
      route.push_back(Waypoint{release_point, post_drop_hold_s_, "drop release " + label, true, i, target.id});
      route.push_back(Waypoint{cruise_point, 0.0, "drop climb " + label, false, 0, ""});
      cursor = cruise_point;
    }

    if (!recon_targets_.empty()) {
      std::vector<ReconTarget> remaining = recon_targets_;
      for (auto & target : remaining) {
        target.point = scene_point_to_local(target.point, recon_hover_alt_);
      }

      while (!remaining.empty()) {
        auto best_it = std::min_element(
          remaining.begin(), remaining.end(),
          [&cursor](const ReconTarget & a, const ReconTarget & b) {
            return distance_xy(cursor, a.point) < distance_xy(cursor, b.point);
          });

        const Point3 view_point = compensated_camera_view_point(best_it->point);
        route.push_back(Waypoint{
          view_point,
          bucket_hold_s_,
          "recon " + best_it->id + "(" + best_it->marker + ")",
          false,
          0,
          ""});
        cursor = view_point;
        remaining.erase(best_it);
      }
      return route;
    }

    if (!route.empty()) {
      return route;
    }

    for (std::size_t i = 0; i < fallback_scan_waypoints_.size(); ++i) {
      route.push_back(Waypoint{
        scene_point_to_local(fallback_scan_waypoints_[i], recon_hover_alt_),
        bucket_hold_s_,
        "fallback_" + std::to_string(i + 1),
        false,
        0,
        ""});
    }
    return route;
  }

  void start_scan() {
    scan_index_ = 0;
    holding_at_waypoint_ = false;
    scan_route_ = make_scan_route(current_position_at_altitude());

    if (scan_route_.empty()) {
      RCLCPP_WARN(get_logger(), "Scan route is empty, returning home");
      start_return_home();
      return;
    }

    for (std::size_t i = 0; i < scan_route_.size(); ++i) {
      const auto & wp = scan_route_[i];
      RCLCPP_INFO(
        get_logger(),
        "Scan wp %zu/%zu %s -> ENU/map (x=%.2f, y=%.2f, z=%.2f), hold %.1fs",
        i + 1, scan_route_.size(), wp.label.c_str(),
        wp.point.x, wp.point.y, wp.point.z, wp.hold_s);
    }

    start_segment(current_position_at_altitude(), scan_route_.front().point);
    enter_state(MissionState::SCANNING);
  }

  void update_scan_trajectory() {
    if (scan_index_ >= scan_route_.size()) {
      start_return_home();
      return;
    }

    if (holding_at_waypoint_) {
      const auto & wp = scan_route_[scan_index_];
      target_ = wp.point;
      if (wp.release_payload) {
        update_release_during_hold(wp);
      }
      if (seconds_since(hold_start_time_) < wp.hold_s) {
        return;
      }
      if (wp.release_payload && servo_return_to_stowed_ && !servo_return_sent_ &&
          (release_mode_ == "servo" || release_mode_ == "both")) {
        send_servo_for_payload(wp.payload_index, false);
        servo_return_sent_ = true;
      }

      holding_at_waypoint_ = false;
      ++scan_index_;
      if (scan_index_ >= scan_route_.size()) {
        RCLCPP_INFO(get_logger(), "Scan trajectory finished, returning home");
        start_return_home();
        return;
      }
      start_segment(active_segment_.end, scan_route_[scan_index_].point);
      return;
    }

    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (!segment_finished() && distance_xy(current_position_, active_segment_.end) > accept_radius_) {
      return;
    }

    target_ = active_segment_.end;
    update_yaw_to_target(target_);
    if (scan_route_[scan_index_].hold_s > 0.0) {
      begin_hold_at_current_waypoint();
      return;
    }

    ++scan_index_;
    if (scan_index_ >= scan_route_.size()) {
      RCLCPP_INFO(get_logger(), "Scan trajectory finished, returning home");
      start_return_home();
      return;
    }
    start_segment(active_segment_.end, scan_route_[scan_index_].point);
  }


  void begin_hold_at_current_waypoint() {
    holding_at_waypoint_ = true;
    hold_start_time_ = now();
    release_started_ = false;
    servo_return_sent_ = false;
    const auto & wp = scan_route_[scan_index_];
    RCLCPP_INFO(
      get_logger(), "Holding over %s for %.1fs%s",
      wp.label.c_str(), wp.hold_s, wp.release_payload ? " with release" : "");
  }

  void update_release_during_hold(const Waypoint & wp) {
    if (!release_started_) {
      release_started_ = true;
      release_start_time_ = now();
      RCLCPP_INFO(
        get_logger(), "Release payload %zu for %s at vehicle center (%.2f, %.2f, %.2f)",
        wp.payload_index + 1, wp.drop_target_id.c_str(), wp.point.x, wp.point.y, wp.point.z);
      if (release_mode_ == "virtual" || release_mode_ == "both") {
        call_virtual_release();
      }
      if (release_mode_ == "servo" || release_mode_ == "both") {
        send_servo_for_payload(wp.payload_index, true);
      }
      if (release_mode_ != "virtual" && release_mode_ != "servo" && release_mode_ != "both") {
        RCLCPP_WARN(get_logger(), "Unknown release_mode=%s; no release command sent", release_mode_.c_str());
      }
      return;
    }

    const double release_duration = value_for_index(servo_release_duration_s_, wp.payload_index, 0.7);
    if (servo_return_to_stowed_ && !servo_return_sent_ &&
        (release_mode_ == "servo" || release_mode_ == "both") &&
        seconds_since(release_start_time_) >= release_duration) {
      send_servo_for_payload(wp.payload_index, false);
      servo_return_sent_ = true;
    }
  }

  void call_virtual_release() {
    if (!release_cli_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Virtual release service is not ready");
      return;
    }
    pending_release_future_ = release_cli_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
  }

  void send_servo_for_payload(std::size_t payload_index, bool release) {
    const int channel = static_cast<int>(value_for_index(servo_channels_, payload_index, int64_t{-1}));
    const int pwm = static_cast<int>(
      value_for_index(release ? servo_release_pwm_ : servo_stowed_pwm_, payload_index, int64_t{-1}));
    if (channel <= 0 || pwm <= 0) {
      RCLCPP_WARN(get_logger(), "Servo config missing for payload %zu", payload_index + 1);
      return;
    }
    if (!command_long_cli_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "MAVROS command_long service is not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast = false;
    req->command = 183;  // MAV_CMD_DO_SET_SERVO
    req->confirmation = 0;
    req->param1 = static_cast<float>(channel);
    req->param2 = static_cast<float>(pwm);
    command_long_cli_->async_send_request(req);
    RCLCPP_INFO(
      get_logger(), "Sent DO_SET_SERVO payload=%zu channel=%d pwm=%d", payload_index + 1, channel, pwm);
  }

  template<typename T>
  static T value_for_index(const std::vector<T> & values, std::size_t index, T fallback) {
    if (index < values.size()) {
      return values[index];
    }
    if (!values.empty()) {
      return values.back();
    }
    return fallback;
  }

  void start_return_home() {
    const auto home = home_.value_or(current_position_);
    const Point3 return_point{home.x, home.y, mission_altitude(cruise_alt_)};
    holding_at_waypoint_ = false;
    start_segment(current_position_, return_point);
    enter_state(MissionState::RETURN_HOME);
  }

  void update_return_trajectory() {
    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (segment_finished() || distance_xy(current_position_, active_segment_.end) <= accept_radius_) {
      enter_state(MissionState::LANDING);
    }
  }

  void start_segment(const Point3 & start, const Point3 & end) {
    const double dist = distance_xyz(start, end);
    const double duration = std::max(min_segment_time_s_, dist / trajectory_speed_);
    active_segment_ = Segment{start, end, duration, now()};
    target_ = start;
    update_yaw_to_target(end);
    RCLCPP_INFO(
      get_logger(),
      "New min-snap segment ENU/map: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f), %.1f s",
      start.x, start.y, start.z, end.x, end.y, end.z, duration);
  }

  Point3 sample_segment() const {
    const double elapsed =
      std::clamp(seconds_since(active_segment_.start_time), 0.0, active_segment_.duration_s);
    const double tau = elapsed / active_segment_.duration_s;
    const double s = min_snap_blend(tau);
    return interpolate(active_segment_.start, active_segment_.end, s);
  }

  bool segment_finished() const {
    return seconds_since(active_segment_.start_time) >= active_segment_.duration_s;
  }

  Point3 current_position_at_altitude() const {
    return Point3{current_position_.x, current_position_.y, mission_altitude(takeoff_alt_)};
  }

  std::vector<Point3> parse_waypoints(const std::vector<double> & xy_values) const {
    std::vector<Point3> waypoints;
    if (xy_values.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(), "scan_waypoints_xy has odd length, ignoring the last value");
    }
    for (std::size_t i = 0; i + 1 < xy_values.size(); i += 2) {
      Point3 p{xy_values[i], xy_values[i + 1], recon_hover_alt_};
      waypoints.push_back(transform_scene_point(p));
    }
    if (waypoints.empty()) {
      waypoints.push_back(Point3{0.0, 0.0, recon_hover_alt_});
    }
    return waypoints;
  }

  void publish_setpoint(const Point3 & point) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = point.x;
    msg.pose.position.y = point.y;
    msg.pose.position.z = point.z;
    msg.pose.orientation.z = std::sin(current_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(current_yaw_ * 0.5);
    setpoint_pub_->publish(msg);
  }

  static double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void update_commanded_yaw() {
    const double dt = std::clamp(seconds_since(last_yaw_update_time_), 0.0, 0.2);
    last_yaw_update_time_ = now();
    const double error = normalize_angle(desired_yaw_ - current_yaw_);
    current_yaw_ = normalize_angle(
      current_yaw_ +
      std::clamp(error, -max_yaw_rate_rad_s_ * dt, max_yaw_rate_rad_s_ * dt));
  }

  void update_yaw_to_target(const Point3 & point) {
    if (lock_mission_yaw_to_initial_heading_ || !yaw_to_target_) {
      desired_yaw_ = mission_heading_rad_;
      return;
    }
    const double dx = point.x - current_position_.x;
    const double dy = point.y - current_position_.y;
    if (std::hypot(dx, dy) > 0.2) {
      desired_yaw_ = std::atan2(dy, dx);
    }
  }

  void call_set_mode(const std::string & mode) {
    if (!set_mode_cli_->service_is_ready() || !service_request_allowed()) {
      return;
    }
    mark_service_request();
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    pending_set_mode_future_ = set_mode_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Requested mode: %s", mode.c_str());
  }

  void call_arm(bool arm) {
    if (!arming_cli_->service_is_ready() || !service_request_allowed()) {
      return;
    }
    mark_service_request();
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    pending_arm_future_ = arming_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Requested arming: %s", arm ? "true" : "false");
  }

  bool call_takeoff(double altitude) {
    if (!takeoff_cli_->service_is_ready() || !service_request_allowed()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Takeoff service not ready or request throttled");
      return false;
    }
    mark_service_request();
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(altitude);
    pending_takeoff_future_ = takeoff_cli_->async_send_request(req).future.share();
    RCLCPP_INFO(get_logger(), "Takeoff command sent to %.1f m", altitude);
    return true;
  }

  void call_land() {
    if (!land_cli_->service_is_ready() || !service_request_allowed()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Land service not ready or request throttled");
      return;
    }
    mark_service_request();
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_cli_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "Land command sent");
  }

  bool service_request_allowed() const {
    return !have_last_service_request_time_ ||
      seconds_since(last_service_request_time_) >= service_request_interval_s_;
  }

  void mark_service_request() {
    last_service_request_time_ = now();
    have_last_service_request_time_ = true;
  }

  void check_service_results() {
    if (
      pending_set_mode_future_.valid() &&
      pending_set_mode_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_set_mode_future_.get();
      if (!result->mode_sent) {
        RCLCPP_WARN(get_logger(), "Set mode failed, will retry");
      }
      pending_set_mode_future_ = {};
    }

    if (
      pending_arm_future_.valid() &&
      pending_arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_arm_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Arming failed, will retry");
      }
      pending_arm_future_ = {};
    }

    if (
      pending_takeoff_future_.valid() &&
      pending_takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_takeoff_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Takeoff command rejected");
      }
      pending_takeoff_future_ = {};
    }

    if (
      pending_release_future_.valid() &&
      pending_release_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_release_future_.get();
      RCLCPP_INFO(
        get_logger(), "Virtual release result: success=%s, %s",
        result->success ? "true" : "false", result->message.c_str());
      pending_release_future_ = {};
    }
  }

  void enter_state(MissionState next_state) {
    if (mission_state_ == next_state) {
      return;
    }
    mission_state_ = next_state;
    state_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "Enter %s", state_name(next_state).c_str());
  }

  double seconds_since(const rclcpp::Time & start) const {
    return (now() - start).seconds();
  }

  static double min_snap_blend(double tau) {
    tau = std::clamp(tau, 0.0, 1.0);
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;
    return 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;
  }

  static Point3 interpolate(const Point3 & a, const Point3 & b, double s) {
    return Point3{
      a.x + (b.x - a.x) * s,
      a.y + (b.y - a.y) * s,
      a.z + (b.z - a.z) * s};
  }

  static double distance_xy(const Point3 & a, const Point3 & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double distance_xyz(const Point3 & a, const Point3 & b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static double yaw_from_quaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::string state_name(MissionState state) {
    switch (state) {
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::WAITING_NAV_STABLE: return "WAITING_NAV_STABLE";
      case MissionState::PRESTREAM_HOLD: return "PRESTREAM_HOLD";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::SCANNING: return "SCANNING";
      case MissionState::RETURN_HOME: return "RETURN_HOME";
      case MissionState::LANDING: return "LANDING";
      case MissionState::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  double takeoff_alt_ = 4.0;
  double cruise_alt_ = 5.0;
  double drop_hover_alt_ = 3.0;
  double recon_hover_alt_ = 3.5;
  double drop_settle_s_ = 2.0;
  double post_drop_hold_s_ = 0.8;
  double accept_radius_ = 0.35;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 420.0;
  double home_lock_stability_s_ = 2.0;
  double home_lock_max_xy_jump_m_ = 0.35;
  double home_lock_max_z_jump_m_ = 0.35;
  double max_abs_local_position_m_ = 1000.0;
  double service_request_interval_s_ = 1.0;
  double trajectory_speed_ = 2.0;
  double min_segment_time_s_ = 2.0;
  double bucket_hold_s_ = 4.0;
  double prestream_hold_s_ = 1.5;
  double stationary_speed_max_m_s_ = 0.15;
  double heading_lock_stability_s_ = 2.0;
  double heading_lock_max_variation_rad_ = 2.0 * M_PI / 180.0;
  double prearm_position_tolerance_m_ = 0.25;
  double prearm_heading_tolerance_rad_ = 5.0 * M_PI / 180.0;
  double max_yaw_rate_rad_s_ = M_PI / 4.0;
  double current_horizontal_speed_m_s_ = 0.0;
  double current_yaw_ = 0.0;
  double desired_yaw_ = 0.0;
  double current_heading_rad_ = 0.0;
  double mission_yaw_rad_ = 0.0;
  double mission_heading_rad_ = 0.0;
  double field_yaw_rad_ = 0.0;
  bool use_generated_scene_ = true;
  bool yaw_to_target_ = false;
  bool lock_mission_yaw_to_initial_heading_ = false;
  bool require_fcu_home_before_arm_ = true;
  bool servo_return_to_stowed_ = true;
  bool servo_initialize_stowed_ = true;
  bool use_camera_target_point_body_ = false;
  std::string generated_scene_path_;
  std::string scene_coordinate_mode_;
  std::string release_mode_ = "virtual";
  std::vector<std::string> requested_drop_targets_;
  std::vector<double> payload_release_offsets_body_;
  std::vector<double> payload_drop_bias_body_;
  std::vector<double> camera_offset_body_;
  std::array<double, 9> camera_optical_to_body_rotation_{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0};
  std::vector<double> camera_target_point_camera_;
  std::vector<double> camera_target_point_body_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> servo_release_pwm_;
  std::vector<int64_t> servo_stowed_pwm_;
  std::vector<double> servo_release_duration_s_;
  std::deque<HeadingSample> heading_samples_;
  mutable bool targets_are_home_relative_ = false;
  Point3 vehicle_spawn_world_;

  mavros_msgs::msg::State current_state_;
  Point3 current_position_;
  std::optional<Point3> home_;
  Point3 target_;
  std::vector<DropTarget> drop_targets_;
  std::vector<ReconTarget> recon_targets_;
  std::vector<Point3> fallback_scan_waypoints_;
  std::vector<Waypoint> scan_route_;
  std::size_t scan_index_ = 0;
  Segment active_segment_;

  MissionState mission_state_ = MissionState::WAITING_FCU;
  bool have_local_position_ = false;
  bool have_local_heading_ = false;
  bool have_odom_heading_ = false;
  bool have_local_velocity_ = false;
  bool have_fcu_home_ = false;
  bool mission_frame_locked_ = false;
  bool mission_started_ = false;
  bool publish_setpoints_ = false;
  bool takeoff_sent_ = false;
  bool land_sent_ = false;
  bool holding_at_waypoint_ = false;
  bool release_started_ = false;
  bool servo_return_sent_ = false;
  bool servos_initialized_ = false;
  bool have_last_service_request_time_ = false;

  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_state_log_;
  rclcpp::Time last_yaw_update_time_;
  rclcpp::Time hold_start_time_;
  rclcpp::Time release_start_time_;
  rclcpp::Time position_stable_since_;
  rclcpp::Time last_service_request_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_long_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture pending_set_mode_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture pending_arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture pending_takeoff_future_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture pending_release_future_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CuadcFullMissionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
