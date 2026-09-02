/*
 * 代码功能：全流程
 * 是否进行仿真验证：是
 * 是否进行飞行验证：否
 * 去赛前最后一版优先高价值目标投放代码，由于区赛采取保守策略，这一版代码没有实际使用
 */
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
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
  DROP_SEARCH,
  DROP_ALIGN_COARSE,
  DROP_ALIGN_FINE,
  DROP_RELEASE,
  DROP_RECOVER,
  RECON_SURVEY,
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

struct BucketTrack {
  Point3 local;
  Point3 body;
  double diameter = 0.0;
  int score_estimate = 0;
  rclcpp::Time stamp;
  std::size_t updates = 1;
};

struct Waypoint {
  Point3 point;
  double hold_s = 0.0;
  std::string label;
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

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

class CuadcFullMissionNode : public rclcpp::Node {
public:
  CuadcFullMissionNode() : Node("cuadc_full_mission_node") {
    declare_parameter<double>("takeoff_alt", 4.0);
    declare_parameter<double>("cruise_alt", 5.0);
    declare_parameter<double>("drop_hover_alt", 3.0);
    declare_parameter<double>("recon_hover_alt", 3.5);
    declare_parameter<double>("drop_search_alt", 4.2);
    declare_parameter<double>("drop_coarse_alt", 3.4);
    declare_parameter<double>("drop_fine_alt", 3.0);
    declare_parameter<int>("drop_search_lane_count", 3);
    declare_parameter<double>("drop_search_edge_margin_m", 0.35);
    declare_parameter<double>("drop_search_cross_margin_m", 0.55);
    declare_parameter<double>("coarse_body_error_m", 0.45);
    declare_parameter<double>("fine_body_error_m", 0.055);
    declare_parameter<double>("release_body_error_m", 0.055);
    declare_parameter<double>("coarse_stable_s", 0.8);
    declare_parameter<double>("fine_stable_s", 0.9);
    declare_parameter<double>("drop_lost_timeout_s", 3.0);
    declare_parameter<double>("track_gate_m", 0.85);
    declare_parameter<int>("bucket_min_confirmations", 3);
    declare_parameter<double>("align_initial_track_grace_s", 10.0);
    declare_parameter<double>("known_bucket_memory_s", 180.0);
    declare_parameter<double>("drop_target_timeout_s", 30.0);
    declare_parameter<double>("fine_align_timeout_s", 12.0);
    declare_parameter<int>("release_gate_max_retries", 4);
    declare_parameter<double>("timeout_release_body_error_m", 0.45);
    declare_parameter<bool>("allow_degraded_timeout_release", true);
    declare_parameter<double>("b_zone_final_approach_timeout_s", 10.0);
    declare_parameter<double>("align_retarget_interval_s", 0.65);
    declare_parameter<double>("align_retarget_delta_m", 0.45);
    declare_parameter<double>("align_target_smoothing", 0.30);
    declare_parameter<int>("max_drop_search_passes", 3);
    declare_parameter<bool>("use_known_bucket_after_release", true);
    declare_parameter<bool>("search_until_candidates_for_payloads", true);
    declare_parameter<bool>("first_payload_require_500_and_300", true);
    declare_parameter<double>("drop_release_hold_s", 0.6);
    declare_parameter<double>("post_drop_climb_hold_s", 0.4);
    declare_parameter<int>("payload_count", 2);
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
    declare_parameter<double>("local_pose_timeout_s", 0.75);
    declare_parameter<double>("service_request_interval_s", 1.0);
    declare_parameter<bool>("require_fcu_home_before_arm", true);
    declare_parameter<double>("trajectory_speed", 2.0);
    declare_parameter<double>("min_segment_time_s", 2.0);
    declare_parameter<double>("bucket_hold_s", 4.0);
    declare_parameter<bool>("use_generated_scene", true);
    declare_parameter<std::string>("generated_scene_path", "");
    declare_parameter<std::string>("scene_coordinate_mode", "gazebo_world_to_mavros_local");
    declare_parameter<std::string>("bucket_detection_topic", "/perception/drop_buckets_body");
    declare_parameter<std::vector<double>>("payload_release_offsets_body_m", std::vector<double>{0.0, 0.0, -0.16, 0.0, 0.0, -0.16});
    declare_parameter<std::vector<double>>("payload_drop_bias_body_m", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("camera_offset_body_m", std::vector<double>{0.0, 0.0, -0.08});
    declare_parameter<std::vector<double>>("camera_optical_to_body_rotation", std::vector<double>{-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0});
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
    drop_search_alt_ = get_parameter("drop_search_alt").as_double();
    drop_coarse_alt_ = get_parameter("drop_coarse_alt").as_double();
    drop_fine_alt_ = get_parameter("drop_fine_alt").as_double();
    drop_search_lane_count_ = std::max(1, static_cast<int>(get_parameter("drop_search_lane_count").as_int()));
    drop_search_edge_margin_m_ = std::max(0.0, get_parameter("drop_search_edge_margin_m").as_double());
    drop_search_cross_margin_m_ = std::max(0.0, get_parameter("drop_search_cross_margin_m").as_double());
    coarse_body_error_m_ = std::max(0.05, get_parameter("coarse_body_error_m").as_double());
    fine_body_error_m_ = std::max(0.03, get_parameter("fine_body_error_m").as_double());
    release_body_error_m_ = std::max(0.02, get_parameter("release_body_error_m").as_double());
    coarse_stable_s_ = std::max(0.1, get_parameter("coarse_stable_s").as_double());
    fine_stable_s_ = std::max(0.1, get_parameter("fine_stable_s").as_double());
    drop_lost_timeout_s_ = std::max(0.2, get_parameter("drop_lost_timeout_s").as_double());
    track_gate_m_ = std::max(0.1, get_parameter("track_gate_m").as_double());
    bucket_min_confirmations_ = std::max(1, static_cast<int>(get_parameter("bucket_min_confirmations").as_int()));
    align_initial_track_grace_s_ = std::max(0.5, get_parameter("align_initial_track_grace_s").as_double());
    known_bucket_memory_s_ = std::max(1.0, get_parameter("known_bucket_memory_s").as_double());
    drop_target_timeout_s_ = std::max(5.0, get_parameter("drop_target_timeout_s").as_double());
    fine_align_timeout_s_ = std::max(2.0, get_parameter("fine_align_timeout_s").as_double());
    release_gate_max_retries_ = std::max(0, static_cast<int>(get_parameter("release_gate_max_retries").as_int()));
    timeout_release_body_error_m_ = std::max(
      release_body_error_m_, get_parameter("timeout_release_body_error_m").as_double());
    allow_degraded_timeout_release_ = get_parameter("allow_degraded_timeout_release").as_bool();
    b_zone_final_approach_timeout_s_ = std::max(
      2.0, get_parameter("b_zone_final_approach_timeout_s").as_double());
    align_retarget_interval_s_ = std::max(0.05, get_parameter("align_retarget_interval_s").as_double());
    align_retarget_delta_m_ = std::max(0.05, get_parameter("align_retarget_delta_m").as_double());
    align_target_smoothing_ = std::clamp(
      get_parameter("align_target_smoothing").as_double(), 0.05, 1.0);
    max_drop_search_passes_ = std::max(1, static_cast<int>(get_parameter("max_drop_search_passes").as_int()));
    use_known_bucket_after_release_ = get_parameter("use_known_bucket_after_release").as_bool();
    search_until_candidates_for_payloads_ = get_parameter("search_until_candidates_for_payloads").as_bool();
    first_payload_require_500_and_300_ = get_parameter("first_payload_require_500_and_300").as_bool();
    drop_release_hold_s_ = std::max(0.1, get_parameter("drop_release_hold_s").as_double());
    post_drop_climb_hold_s_ = std::max(0.0, get_parameter("post_drop_climb_hold_s").as_double());
    payload_count_ = std::max(1, static_cast<int>(get_parameter("payload_count").as_int()));
    drop_settle_s_ = get_parameter("drop_settle_s").as_double();
    post_drop_hold_s_ = get_parameter("post_drop_hold_s").as_double();
    mission_yaw_rad_ = get_parameter("mission_yaw_rad").as_double();
    field_yaw_offset_rad_ = get_parameter("field_yaw_rad").as_double();
    field_yaw_rad_ = field_yaw_offset_rad_;
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
    local_pose_timeout_s_ =
      std::max(0.1, get_parameter("local_pose_timeout_s").as_double());
    service_request_interval_s_ =
      std::max(0.2, get_parameter("service_request_interval_s").as_double());
    require_fcu_home_before_arm_ = get_parameter("require_fcu_home_before_arm").as_bool();
    trajectory_speed_ = std::max(0.1, get_parameter("trajectory_speed").as_double());
    min_segment_time_s_ = std::max(0.5, get_parameter("min_segment_time_s").as_double());
    bucket_hold_s_ = std::max(0.0, get_parameter("bucket_hold_s").as_double());
    use_generated_scene_ = get_parameter("use_generated_scene").as_bool();
    generated_scene_path_ = get_parameter("generated_scene_path").as_string();
    scene_coordinate_mode_ = get_parameter("scene_coordinate_mode").as_string();
    bucket_detection_topic_ = get_parameter("bucket_detection_topic").as_string();
    payload_release_offsets_body_ = get_parameter("payload_release_offsets_body_m").as_double_array();
    payload_drop_bias_body_ = get_parameter("payload_drop_bias_body_m").as_double_array();
    camera_offset_body_ = get_parameter("camera_offset_body_m").as_double_array();
    camera_optical_to_body_rotation_ = matrix3_param(
      "camera_optical_to_body_rotation",
      std::array<double, 9>{-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0});
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
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", best_effort,
      std::bind(&CuadcFullMissionNode::pose_cb, this, std::placeholders::_1));
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_detection_topic_, best_effort,
      std::bind(&CuadcFullMissionNode::bucket_cb, this, std::placeholders::_1));

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
      "CUADC full mission C++ node started; dynamic_bucket_search=true, recon_targets=%zu, "
      "coordinate_mode=%s, field_yaw=%.3f rad, lock_initial_yaw=%s, bucket_topic=%s",
      recon_targets_.size(), scene_coordinate_mode_.c_str(), field_yaw_offset_rad_,
      lock_mission_yaw_to_initial_heading_ ? "true" : "false", bucket_detection_topic_.c_str());
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
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    update_local_pose(
      Point3{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z},
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w,
      "local_position/pose");
  }

  void bucket_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (!have_local_position_ || !have_local_heading_ || !mission_frame_locked_) {
      return;
    }

    std::vector<BucketTrack> detections;
    detections.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      const Point3 body{
        pose.position.x,
        pose.position.y,
        pose.position.z};
      if (!std::isfinite(body.x) || !std::isfinite(body.y) || !std::isfinite(body.z)) {
        continue;
      }
      const Point3 local = body_point_to_local(body);
      if (bucket_unavailable(local)) {
        continue;
      }
      const double diameter = std::max(0.0, pose.orientation.x);
      detections.push_back(BucketTrack{
        local,
        body,
        diameter,
        score_from_diameter(diameter),
        now(),
        1});
    }

    for (const auto & detection : detections) {
      merge_bucket_track(known_buckets_, detection);
      if (mission_state_ == MissionState::DROP_SEARCH) {
        merge_bucket_track(search_candidates_, detection);
      }
    }

    if (!active_bucket_.has_value() || detections.empty()) {
      return;
    }
    if (mission_state_ != MissionState::DROP_ALIGN_COARSE &&
      mission_state_ != MissionState::DROP_ALIGN_FINE &&
      mission_state_ != MissionState::DROP_RELEASE)
    {
      return;
    }

    auto nearest = std::min_element(
      detections.begin(), detections.end(),
      [this](const BucketTrack & a, const BucketTrack & b) {
        return distance_xy(a.local, active_bucket_->local) <
               distance_xy(b.local, active_bucket_->local);
      });
    if (nearest != detections.end() &&
      distance_xy(nearest->local, active_bucket_->local) <= track_gate_m_)
    {
      smooth_bucket_track(*active_bucket_, *nearest);
    }
  }

  void update_local_pose(
    const Point3 & observed, double qx, double qy, double qz, double qw, const char * source)
  {
    if (!valid_local_pose(observed)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid MAVROS local position from %s: (%.3g, %.3g, %.3g)",
        source, observed.x, observed.y, observed.z);
      return;
    }

    if (have_local_position_) {
      if (!mission_frame_locked_) {
        const double xy_jump = distance_xy(current_position_, observed);
        const double z_jump = std::abs(current_position_.z - observed.z);
        if (xy_jump > home_lock_max_xy_jump_m_ || z_jump > home_lock_max_z_jump_m_) {
          position_stable_since_ = now();
          RCLCPP_WARN(
            get_logger(),
            "Local %s jumped before home lock: dxy=%.2f m, dz=%.2f m. Waiting %.1f s for RTK/local origin stability.",
            source, xy_jump, z_jump, home_lock_stability_s_);
        }
      }
    } else {
      position_stable_since_ = now();
      RCLCPP_INFO(get_logger(), "First MAVROS local position received from MAVROS %s", source);
    }

    current_position_ = observed;
    have_local_position_ = true;
    last_local_pose_time_ = now();
    have_last_local_pose_time_ = true;

    const double q_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    if (q_norm_sq > 1e-12) {
      const double inverse_norm = 1.0 / std::sqrt(q_norm_sq);
      current_attitude_ = Quaternion{
        qx * inverse_norm, qy * inverse_norm, qz * inverse_norm, qw * inverse_norm};
      current_heading_rad_ = horizontal_heading_from_quaternion(current_attitude_);
      have_local_heading_ = true;
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

  double mean_locked_heading() const {
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & sample : heading_samples_) {
      sin_sum += std::sin(sample.yaw);
      cos_sum += std::cos(sample.yaw);
    }
    return std::atan2(sin_sum, cos_sum);
  }

  bool navigation_preflight_ready() const {
    const bool stationary =
      !have_local_velocity_ || current_horizontal_speed_m_s_ <= stationary_speed_max_m_s_;
    return current_state_.connected && !current_state_.armed &&
      have_local_position_ && have_local_heading_ && local_pose_fresh() &&
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
    mission_heading_rad_ = lock_mission_yaw_to_initial_heading_ ?
      mean_locked_heading() : mission_yaw_rad_;
    mission_yaw_rad_ = mission_heading_rad_;
    if (lock_mission_yaw_to_initial_heading_) {
      field_yaw_rad_ = normalize_angle(mission_heading_rad_ + field_yaw_offset_rad_);
    }
    locked_orientation_ = yaw_quaternion(mission_heading_rad_);
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
    takeoff_sent_ = false;
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

  bool local_pose_fresh() const {
    return have_last_local_pose_time_ &&
      seconds_since(last_local_pose_time_) <= local_pose_timeout_s_;
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
    Point3 mapped = transform_scene_point(point);
    if (scene_targets_are_home_relative() && home_.has_value()) {
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
        "yaw_vehicle=%.2fdeg yaw_command=%.2fdeg yaw_locked=%.2fdeg, "
        "bucket_track=%s payload=%d/%d known_buckets=%zu",
        state_name(mission_state_).c_str(), current_position_.x, current_position_.y,
        current_position_.z, target_.x, target_.y, target_.z,
        current_heading_rad_ * 180.0 / M_PI,
        current_yaw_ * 180.0 / M_PI,
        mission_heading_rad_ * 180.0 / M_PI,
        active_bucket_.has_value() ? "yes" : "no",
        payload_index_, payload_count_, valid_known_bucket_count());
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
        // ArduPilot Guided TakeOff owns the vertical controller and holds the
        // current EKF yaw. Position setpoints here would replace that submode.
        publish_setpoints_ = false;
        if (!have_local_position_ || !local_pose_fresh()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for fresh /mavros/local_position/pose during takeoff");
          break;
        }
        if (!mission_frame_locked_ || !home_.has_value()) {
          RCLCPP_ERROR(get_logger(), "Takeoff frame was lost after arming; landing immediately");
          enter_state(MissionState::LANDING);
          break;
        }
        if (!current_state_.armed) {
          RCLCPP_ERROR(get_logger(), "FCU disarmed during takeoff; returning to preflight lock");
          takeoff_sent_ = false;
          mission_started_ = false;
          reset_takeoff_frame();
          enter_state(MissionState::WAITING_NAV_STABLE);
          break;
        }
        target_ = Point3{home_->x, home_->y, mission_altitude(takeoff_alt_)};
        current_yaw_ = mission_heading_rad_;
        desired_yaw_ = mission_heading_rad_;
        if (!takeoff_sent_ && call_takeoff(takeoff_alt_)) {
          takeoff_sent_ = true;
          state_enter_time_ = now();
        }
        if (relative_altitude() >= takeoff_alt_ * 0.9) {
          publish_setpoints_ = true;
          start_drop_search();
        } else if (takeoff_sent_ && seconds_since(state_enter_time_) > takeoff_timeout_s_) {
          RCLCPP_ERROR(
            get_logger(), "Takeoff timeout at relative altitude %.2f m; landing instead of advancing the mission",
            relative_altitude());
          enter_state(MissionState::LANDING);
        }
        break;

      case MissionState::DROP_SEARCH:
        update_drop_search();
        break;

      case MissionState::DROP_ALIGN_COARSE:
        update_drop_alignment(false);
        break;

      case MissionState::DROP_ALIGN_FINE:
        update_drop_alignment(true);
        break;

      case MissionState::DROP_RELEASE:
        update_dynamic_release();
        break;

      case MissionState::DROP_RECOVER:
        update_drop_recover();
        break;

      case MissionState::RECON_SURVEY:
        update_recon_survey();
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
        if (field["drop_area_center"] && field["drop_area_center"].IsSequence() &&
          field["drop_area_center"].size() >= 2)
        {
          field_drop_center_.x = field["drop_area_center"][0].as<double>();
          field_drop_center_.y = field["drop_area_center"][1].as<double>();
        }
        if (field["drop_area_size"] && field["drop_area_size"].IsSequence() &&
          field["drop_area_size"].size() >= 2)
        {
          field_drop_size_x_ = field["drop_area_size"][0].as<double>();
          field_drop_size_y_ = field["drop_area_size"][1].as<double>();
        }
      }

      const YAML::Node vehicle = scene["vehicle"];
      if (vehicle && vehicle["pose"] && vehicle["pose"].IsSequence() && vehicle["pose"].size() >= 2) {
        vehicle_spawn_world_.x = vehicle["pose"][0].as<double>();
        vehicle_spawn_world_.y = vehicle["pose"][1].as<double>();
      }

      // Bucket positions are intentionally not loaded here. In both simulation and
      // competition mode the C++ mission must discover them through perception.

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

  bool scene_targets_are_home_relative() const {
    return scene_coordinate_mode_ == "field_local_home_relative" ||
      scene_coordinate_mode_ == "gazebo_world_to_mavros_local" ||
      scene_coordinate_mode_ == "legacy_lateral_forward";
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
      return rotate_field_to_local(p);
    }

    if (scene_coordinate_mode_ == "mavros_local" ||
      scene_coordinate_mode_ == "gazebo_enu")
    {
      return p;
    }

    if (scene_coordinate_mode_ == "gazebo_world_to_mavros_local") {
      return Point3{
        p.x - vehicle_spawn_world_.x,
        p.y - vehicle_spawn_world_.y,
        p.z};
    }

    if (scene_coordinate_mode_ == "legacy_lateral_forward") {
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
    const double yaw = have_local_heading_ ? current_heading_rad_ : mission_heading_rad_;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Point3{
      c * body_vector.x - s * body_vector.y,
      s * body_vector.x + c * body_vector.y,
      body_vector.z};
  }

  Point3 body_point_to_local(const Point3 & body_point) const {
    const Point3 offset = body_vector_to_local(body_point);
    return Point3{
      current_position_.x + offset.x,
      current_position_.y + offset.y,
      current_position_.z + offset.z};
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

  static int score_from_diameter(double diameter) {
    if (diameter <= 0.0) {
      return 0;
    }
    if (diameter < 0.175) {
      return 500;
    }
    if (diameter < 0.225) {
      return 300;
    }
    return 100;
  }

  bool near_any(const Point3 & point, const std::vector<Point3> & points) const {
    return std::any_of(
      points.begin(), points.end(),
      [this, &point](const Point3 & old) {
        return distance_xy(point, old) < 0.65;
      });
  }

  bool bucket_unavailable(const Point3 & point) const {
    return near_any(point, released_buckets_) || near_any(point, skipped_buckets_);
  }

  void smooth_bucket_track(BucketTrack & track, const BucketTrack & detection) {
    constexpr double alpha = 0.35;
    track.local.x = (1.0 - alpha) * track.local.x + alpha * detection.local.x;
    track.local.y = (1.0 - alpha) * track.local.y + alpha * detection.local.y;
    track.local.z = (1.0 - alpha) * track.local.z + alpha * detection.local.z;
    track.body = detection.body;
    track.diameter = detection.diameter;
    track.score_estimate = detection.score_estimate;
    track.stamp = detection.stamp;
    ++track.updates;
  }

  void merge_bucket_track(std::vector<BucketTrack> & tracks, const BucketTrack & detection) {
    if (bucket_unavailable(detection.local)) {
      return;
    }
    auto nearest = std::min_element(
      tracks.begin(), tracks.end(),
      [this, &detection](const BucketTrack & a, const BucketTrack & b) {
        return distance_xy(a.local, detection.local) < distance_xy(b.local, detection.local);
      });
    if (nearest != tracks.end() &&
      distance_xy(nearest->local, detection.local) <= track_gate_m_)
    {
      smooth_bucket_track(*nearest, detection);
      return;
    }
    tracks.push_back(detection);
  }

  bool valid_bucket_track(const BucketTrack & track, double max_age_s) const {
    return track.updates >= static_cast<std::size_t>(bucket_min_confirmations_) &&
      !bucket_unavailable(track.local) &&
      seconds_since(track.stamp) <= max_age_s;
  }

  std::vector<BucketTrack> valid_known_buckets() const {
    std::vector<BucketTrack> valid;
    for (const auto & track : known_buckets_) {
      if (valid_bucket_track(track, known_bucket_memory_s_)) {
        valid.push_back(track);
      }
    }
    return valid;
  }

  std::size_t valid_known_bucket_count() const {
    return valid_known_buckets().size();
  }

  std::optional<BucketTrack> best_known_bucket() const {
    auto valid = valid_known_buckets();
    if (valid.empty()) {
      return std::nullopt;
    }
    const auto best = std::max_element(
      valid.begin(), valid.end(),
      [this](const BucketTrack & a, const BucketTrack & b) {
        if (a.score_estimate != b.score_estimate) {
          return a.score_estimate < b.score_estimate;
        }
        if (a.updates != b.updates) {
          return a.updates < b.updates;
        }
        return distance_xy(current_position_, a.local) >
               distance_xy(current_position_, b.local);
      });
    return *best;
  }

  int remaining_payloads() const {
    return std::max(0, payload_count_ - payload_index_);
  }

  bool enough_known_buckets_for_plan() const {
    auto valid = valid_known_buckets();
    const int remaining = remaining_payloads();
    if (remaining <= 0) {
      return true;
    }
    if (valid.size() < static_cast<std::size_t>(remaining)) {
      return false;
    }
    if (first_payload_require_500_and_300_ && payload_index_ == 0 && remaining >= 2) {
      std::vector<int> scores;
      scores.reserve(valid.size());
      for (const auto & bucket : valid) {
        scores.push_back(bucket.score_estimate);
      }
      std::sort(scores.begin(), scores.end(), std::greater<int>());
      return scores.size() >= 2 && scores[0] >= 500 && scores[1] >= 300;
    }
    return true;
  }

  Point3 field_offset_point(
    const Point3 & center, double field_x, double field_y, double relative_altitude_m) const
  {
    const double c = std::cos(field_yaw_rad_);
    const double s = std::sin(field_yaw_rad_);
    return Point3{
      center.x + c * field_x - s * field_y,
      center.y + s * field_x + c * field_y,
      mission_altitude(relative_altitude_m)};
  }

  double route_cost_from(const Point3 & start, const std::vector<Waypoint> & route) const {
    if (route.empty()) {
      return 0.0;
    }
    double cost = distance_xy(start, route.front().point);
    for (std::size_t i = 1; i < route.size(); ++i) {
      cost += distance_xy(route[i - 1].point, route[i].point);
    }
    return cost;
  }

  std::vector<Waypoint> make_drop_search_route() const {
    const Point3 center = scene_point_to_local(field_drop_center_, drop_search_alt_);
    const double half_x = std::max(
      0.2, field_drop_size_x_ * 0.5 - drop_search_edge_margin_m_);
    const double half_y = std::max(
      0.2, field_drop_size_y_ * 0.5 - drop_search_cross_margin_m_);

    std::vector<double> lanes;
    lanes.reserve(static_cast<std::size_t>(drop_search_lane_count_));
    if (drop_search_lane_count_ == 1) {
      lanes.push_back(0.0);
    } else {
      for (int i = 0; i < drop_search_lane_count_; ++i) {
        const double ratio = static_cast<double>(i) /
          static_cast<double>(drop_search_lane_count_ - 1);
        lanes.push_back(-half_y + 2.0 * half_y * ratio);
      }
    }

    const auto build = [this, &center, half_x](
      std::vector<double> lane_order, bool start_forward)
      {
        std::vector<Waypoint> route;
        bool forward = start_forward;
        for (double lane : lane_order) {
          route.push_back(Waypoint{
            field_offset_point(
              center, forward ? -half_x : half_x, lane, drop_search_alt_),
            0.0, "drop_search"});
          route.push_back(Waypoint{
            field_offset_point(
              center, forward ? half_x : -half_x, lane, drop_search_alt_),
            0.0, "drop_search"});
          forward = !forward;
        }
        return route;
      };

    std::vector<double> reversed = lanes;
    std::reverse(reversed.begin(), reversed.end());
    std::vector<std::vector<Waypoint>> candidates;
    candidates.push_back(build(lanes, true));
    candidates.push_back(build(lanes, false));
    candidates.push_back(build(reversed, true));
    candidates.push_back(build(reversed, false));
    const auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [this](const auto & a, const auto & b) {
        return route_cost_from(current_position_, a) <
               route_cost_from(current_position_, b);
      });
    return best == candidates.end() ? std::vector<Waypoint>{} : *best;
  }

  std::vector<Waypoint> make_recon_route() const {
    std::vector<Waypoint> route;
    std::vector<ReconTarget> remaining = recon_targets_;
    Point3 cursor = current_position_;
    for (auto & target : remaining) {
      target.point = scene_point_to_local(target.point, recon_hover_alt_);
    }
    while (!remaining.empty()) {
      auto nearest = std::min_element(
        remaining.begin(), remaining.end(),
        [this, &cursor](const ReconTarget & a, const ReconTarget & b) {
          return distance_xy(cursor, a.point) < distance_xy(cursor, b.point);
        });
      const Point3 view_point = compensated_camera_view_point(nearest->point);
      route.push_back(Waypoint{
        view_point,
        bucket_hold_s_,
        "recon " + nearest->id + "(" + nearest->marker + ")"});
      cursor = view_point;
      remaining.erase(nearest);
    }
    if (route.empty()) {
      for (std::size_t i = 0; i < fallback_scan_waypoints_.size(); ++i) {
        route.push_back(Waypoint{
          scene_point_to_local(fallback_scan_waypoints_[i], recon_hover_alt_),
          bucket_hold_s_,
          "recon_fallback_" + std::to_string(i + 1)});
      }
    }
    return route;
  }

  void reset_route(const std::vector<Waypoint> & route) {
    scan_route_ = route;
    scan_index_ = 0;
    holding_at_waypoint_ = false;
  }

  bool update_route_trajectory() {
    if (scan_index_ >= scan_route_.size()) {
      return true;
    }

    if (holding_at_waypoint_) {
      target_ = scan_route_[scan_index_].point;
      if (seconds_since(hold_start_time_) < scan_route_[scan_index_].hold_s) {
        return false;
      }
      holding_at_waypoint_ = false;
      ++scan_index_;
      if (scan_index_ >= scan_route_.size()) {
        return true;
      }
      start_segment(current_position_, scan_route_[scan_index_].point);
      return false;
    }

    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (!segment_finished() ||
      distance_xy(current_position_, active_segment_.end) > accept_radius_)
    {
      return false;
    }

    target_ = active_segment_.end;
    if (scan_route_[scan_index_].hold_s > 0.0) {
      holding_at_waypoint_ = true;
      hold_start_time_ = now();
      RCLCPP_INFO(
        get_logger(), "Holding over %s for %.1f s",
        scan_route_[scan_index_].label.c_str(), scan_route_[scan_index_].hold_s);
      return false;
    }

    ++scan_index_;
    if (scan_index_ >= scan_route_.size()) {
      return true;
    }
    start_segment(current_position_, scan_route_[scan_index_].point);
    return false;
  }

  void start_drop_search() {
    active_bucket_.reset();
    search_candidates_.clear();
    release_started_ = false;
    servo_return_sent_ = false;
    force_release_reason_.clear();
    b_zone_fuse_start_time_.reset();
    target_lock_time_.reset();
    release_gate_retries_ = 0;
    ++search_pass_;

    reset_route(make_drop_search_route());
    if (scan_route_.empty()) {
      RCLCPP_ERROR(get_logger(), "Drop search route is empty; continuing reconnaissance");
      start_recon_survey();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Start dynamic drop search pass %d/%d, lanes=%d, waypoints=%zu, "
      "area_center_field=(%.2f, %.2f), area_size=(%.2f x %.2f); "
      "no bucket coordinates loaded",
      search_pass_, max_drop_search_passes_, drop_search_lane_count_,
      scan_route_.size(), field_drop_center_.x, field_drop_center_.y,
      field_drop_size_x_, field_drop_size_y_);
    start_segment(current_position_, scan_route_.front().point);
    enter_state(MissionState::DROP_SEARCH);
  }

  void update_drop_search() {
    const auto best = best_known_bucket();
    if (best.has_value() &&
      (!search_until_candidates_for_payloads_ || enough_known_buckets_for_plan()))
    {
      commit_bucket_candidate(
        *best,
        enough_known_buckets_for_plan() ?
        "enough confirmed unreleased buckets" : "confirmed bucket detected");
      return;
    }

    if (!update_route_trajectory()) {
      return;
    }

    const auto best_after_pass = best_known_bucket();
    if (best_after_pass.has_value()) {
      commit_bucket_candidate(*best_after_pass, "best bucket after full search pass");
    } else if (search_pass_ < max_drop_search_passes_) {
      start_drop_search();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No confirmed bucket after %d full search passes; skipping blind release",
        search_pass_);
      start_recon_survey();
    }
  }

  void commit_bucket_candidate(const BucketTrack & candidate, const std::string & reason) {
    if (bucket_unavailable(candidate.local)) {
      continue_drop_plan("candidate became unavailable");
      return;
    }
    active_bucket_ = candidate;
    active_bucket_->stamp = now();
    target_lock_time_ = now();
    release_gate_retries_ = 0;
    force_release_reason_.clear();
    b_zone_fuse_start_time_.reset();
    RCLCPP_INFO(
      get_logger(),
      "Lock discovered bucket (%s): local=(%.2f, %.2f), diameter=%.3f, "
      "score_est=%d, confirmations=%zu",
      reason.c_str(), candidate.local.x, candidate.local.y,
      candidate.diameter, candidate.score_estimate, candidate.updates);
    start_drop_alignment(false);
  }

  Point3 release_setpoint(double relative_altitude_m) const {
    if (!active_bucket_.has_value()) {
      return Point3{
        current_position_.x,
        current_position_.y,
        mission_altitude(relative_altitude_m)};
    }
    Point3 bucket{
      active_bucket_->local.x,
      active_bucket_->local.y,
      mission_altitude(relative_altitude_m)};
    return compensated_release_point(bucket, static_cast<std::size_t>(payload_index_));
  }

  double body_release_error() const {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const Point3 release_body =
      payload_release_vector_body(static_cast<std::size_t>(payload_index_));
    return std::hypot(
      active_bucket_->body.x - release_body.x,
      active_bucket_->body.y - release_body.y);
  }

  double local_release_error() const {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const Point3 release_local =
      body_vector_to_local(payload_release_vector_body(static_cast<std::size_t>(payload_index_)));
    const Point3 release_point{
      current_position_.x + release_local.x,
      current_position_.y + release_local.y,
      current_position_.z + release_local.z};
    return distance_xy(release_point, active_bucket_->local);
  }

  bool active_track_recent() const {
    return active_bucket_.has_value() &&
      seconds_since(active_bucket_->stamp) <= drop_lost_timeout_s_;
  }

  double estimated_release_error() const {
    const double local_error = local_release_error();
    return active_track_recent() ? std::min(body_release_error(), local_error) : local_error;
  }

  void start_drop_alignment(bool fine) {
    align_stable_since_.reset();
    last_align_target_.reset();
    last_align_retarget_.reset();
    if (!target_lock_time_.has_value()) {
      target_lock_time_ = now();
    }
    const Point3 desired = release_setpoint(fine ? drop_fine_alt_ : drop_coarse_alt_);
    last_align_target_ = desired;
    last_align_retarget_ = now();
    start_segment(current_position_, desired);
    enter_state(fine ? MissionState::DROP_ALIGN_FINE : MissionState::DROP_ALIGN_COARSE);
  }

  Point3 update_alignment_setpoint(const Point3 & raw_desired) {
    if (!last_align_target_.has_value()) {
      last_align_target_ = raw_desired;
      last_align_retarget_ = now();
      start_segment(current_position_, raw_desired);
      return raw_desired;
    }

    const double alpha = align_target_smoothing_;
    const Point3 smoothed{
      (1.0 - alpha) * last_align_target_->x + alpha * raw_desired.x,
      (1.0 - alpha) * last_align_target_->y + alpha * raw_desired.y,
      (1.0 - alpha) * last_align_target_->z + alpha * raw_desired.z};
    const bool retarget_due = !last_align_retarget_.has_value() ||
      seconds_since(*last_align_retarget_) >= align_retarget_interval_s_;
    const bool moved =
      distance_xy(*last_align_target_, smoothed) >= align_retarget_delta_m_ ||
      std::abs(last_align_target_->z - smoothed.z) >= 0.20;
    if ((retarget_due || segment_finished()) && moved) {
      last_align_target_ = smoothed;
      last_align_retarget_ = now();
      start_segment(current_position_, smoothed);
    }
    return *last_align_target_;
  }

  double target_elapsed_s() const {
    return target_lock_time_.has_value() ?
      seconds_since(*target_lock_time_) : seconds_since(state_enter_time_);
  }

  std::optional<std::string> alignment_fuse_reason(bool fine) const {
    if (!active_bucket_.has_value()) {
      return std::nullopt;
    }
    if (b_zone_fuse_start_time_.has_value()) {
      const double elapsed = seconds_since(*b_zone_fuse_start_time_);
      if (elapsed >= b_zone_final_approach_timeout_s_) {
        return "B-zone final approach timeout";
      }
      return std::nullopt;
    }
    if (target_elapsed_s() >= drop_target_timeout_s_) {
      return "drop target timeout";
    }
    if (fine && seconds_since(state_enter_time_) >= fine_align_timeout_s_) {
      return "fine alignment timeout";
    }
    return std::nullopt;
  }

  void handle_target_fuse(const std::string & reason, double error) {
    if (!active_bucket_.has_value()) {
      continue_drop_plan("fuse without active bucket");
      return;
    }
    if (allow_degraded_timeout_release_ && error <= timeout_release_body_error_m_) {
      force_release_reason_ = reason + "; nearest confirmed bucket within B-zone gate";
      release_started_ = false;
      servo_return_sent_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Drop fuse: %s; releasing to nearest confirmed bucket, estimated_error=%.3f m",
        reason.c_str(), error);
      enter_state(MissionState::DROP_RELEASE);
      return;
    }
    if (allow_degraded_timeout_release_ && !b_zone_fuse_start_time_.has_value()) {
      b_zone_fuse_start_time_ = now();
      align_stable_since_.reset();
      last_align_target_.reset();
      last_align_retarget_.reset();
      RCLCPP_WARN(
        get_logger(),
        "Drop fuse: %s; nearest bucket is %.3f m away, forcing final approach",
        reason.c_str(), error);
      return;
    }
    if (allow_degraded_timeout_release_) {
      force_release_reason_ = reason + "; final approach exhausted, nearest-bucket release";
      release_started_ = false;
      servo_return_sent_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Drop fuse final release to nearest confirmed bucket, estimated_error=%.3f m",
        error);
      enter_state(MissionState::DROP_RELEASE);
      return;
    }
    continue_drop_plan("drop fuse disabled after " + reason);
  }

  void update_drop_alignment(bool fine) {
    if (!active_bucket_.has_value()) {
      continue_drop_plan("alignment lost active bucket");
      return;
    }

    const Point3 desired = update_alignment_setpoint(
      release_setpoint(fine ? drop_fine_alt_ : drop_coarse_alt_));
    target_ = sample_segment();
    const double error = estimated_release_error();

    if (!active_track_recent()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Bucket image track stale; continuing toward fused RTK/local bucket coordinate");
    }
    if (b_zone_fuse_start_time_.has_value() && error <= timeout_release_body_error_m_) {
      handle_target_fuse("B-zone final approach reached release gate", error);
      return;
    }
    const auto fuse_reason = alignment_fuse_reason(fine);
    if (fuse_reason.has_value()) {
      handle_target_fuse(*fuse_reason, error);
      return;
    }

    const double threshold = fine ? fine_body_error_m_ : coarse_body_error_m_;
    const double stable_s = fine ? fine_stable_s_ : coarse_stable_s_;
    const double position_threshold = fine ? threshold : std::max(accept_radius_, threshold);
    const bool aligned =
      distance_xy(current_position_, desired) <= position_threshold &&
      error <= threshold;
    if (!aligned) {
      align_stable_since_.reset();
      return;
    }
    if (!align_stable_since_.has_value()) {
      align_stable_since_ = now();
      return;
    }
    if (seconds_since(*align_stable_since_) < stable_s) {
      return;
    }

    if (fine) {
      RCLCPP_INFO(
        get_logger(), "Fine alignment stable, estimated release error=%.3f m", error);
      release_started_ = false;
      servo_return_sent_ = false;
      enter_state(MissionState::DROP_RELEASE);
    } else {
      RCLCPP_INFO(
        get_logger(), "Coarse alignment stable, error=%.3f m; descending", error);
      start_drop_alignment(true);
    }
  }

  void continue_drop_plan(const std::string & reason) {
    active_bucket_.reset();
    target_lock_time_.reset();
    force_release_reason_.clear();
    b_zone_fuse_start_time_.reset();
    if (payload_index_ >= payload_count_) {
      start_recon_survey();
      return;
    }
    const auto next = best_known_bucket();
    if (next.has_value()) {
      commit_bucket_candidate(*next, reason);
    } else if (search_pass_ < max_drop_search_passes_) {
      start_drop_search();
    } else {
      RCLCPP_WARN(
        get_logger(), "No discovered bucket after %s; preserving payload and continuing reconnaissance",
        reason.c_str());
      start_recon_survey();
    }
  }

  void update_dynamic_release() {
    if (!active_bucket_.has_value()) {
      continue_drop_plan("release without active bucket");
      return;
    }

    if (!release_started_) {
      const double error = estimated_release_error();
      if (error > release_body_error_m_ && force_release_reason_.empty()) {
        ++release_gate_retries_;
        if (release_gate_retries_ >= release_gate_max_retries_ ||
          target_elapsed_s() >= drop_target_timeout_s_)
        {
          handle_target_fuse("release gate retry/timeout fuse", error);
          return;
        }
        RCLCPP_WARN(
          get_logger(),
          "Release gate held: error=%.3f > %.3f m, retry=%d/%d",
          error, release_body_error_m_, release_gate_retries_, release_gate_max_retries_);
        start_drop_alignment(true);
        return;
      }

      release_started_ = true;
      release_start_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Release payload %d/%d over dynamically discovered bucket at local=(%.2f, %.2f), "
        "error=%.3f m%s",
        payload_index_ + 1, payload_count_, active_bucket_->local.x, active_bucket_->local.y,
        error, force_release_reason_.empty() ? "" : " [FUSE]");
      if (release_mode_ == "virtual" || release_mode_ == "both") {
        call_virtual_release();
      }
      if (release_mode_ == "servo" || release_mode_ == "both") {
        send_servo_for_payload(static_cast<std::size_t>(payload_index_), true);
      }
      return;
    }

    const double release_duration = value_for_index(
      servo_release_duration_s_, static_cast<std::size_t>(payload_index_), 0.7);
    if (servo_return_to_stowed_ && !servo_return_sent_ &&
      (release_mode_ == "servo" || release_mode_ == "both") &&
      seconds_since(release_start_time_) >= release_duration)
    {
      send_servo_for_payload(static_cast<std::size_t>(payload_index_), false);
      servo_return_sent_ = true;
    }
    if (seconds_since(release_start_time_) < drop_release_hold_s_) {
      return;
    }

    released_buckets_.push_back(active_bucket_->local);
    ++payload_index_;
    active_bucket_.reset();
    target_lock_time_.reset();
    force_release_reason_.clear();
    b_zone_fuse_start_time_.reset();
    recover_hold_started_ = false;
    start_segment(
      current_position_,
      Point3{current_position_.x, current_position_.y, mission_altitude(cruise_alt_)});
    enter_state(MissionState::DROP_RECOVER);
  }

  void update_drop_recover() {
    target_ = sample_segment();
    if (!segment_finished() ||
      distance_xy(current_position_, active_segment_.end) > accept_radius_)
    {
      return;
    }
    if (!recover_hold_started_) {
      recover_hold_started_ = true;
      recover_hold_start_time_ = now();
      return;
    }
    if (seconds_since(recover_hold_start_time_) < post_drop_climb_hold_s_) {
      return;
    }

    if (payload_index_ >= payload_count_) {
      start_recon_survey();
      return;
    }
    const auto next = use_known_bucket_after_release_ ? best_known_bucket() : std::nullopt;
    if (next.has_value()) {
      commit_bucket_candidate(*next, "known unreleased bucket after previous drop");
    } else if (search_pass_ < max_drop_search_passes_) {
      start_drop_search();
    } else {
      RCLCPP_WARN(get_logger(), "No second bucket discovered; continuing reconnaissance");
      start_recon_survey();
    }
  }

  void start_recon_survey() {
    reset_route(make_recon_route());
    if (scan_route_.empty()) {
      start_return_home();
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Start reconnaissance survey after dynamic drop phase, waypoints=%zu",
      scan_route_.size());
    start_segment(current_position_, scan_route_.front().point);
    enter_state(MissionState::RECON_SURVEY);
  }

  void update_recon_survey() {
    if (update_route_trajectory()) {
      RCLCPP_INFO(get_logger(), "Reconnaissance route finished, returning home");
      start_return_home();
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
    if (segment_finished() &&
      distance_xy(current_position_, active_segment_.end) <= accept_radius_)
    {
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
      waypoints.push_back(p);
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
    const Quaternion command_orientation = lock_mission_yaw_to_initial_heading_ ?
      locked_orientation_ : yaw_quaternion(current_yaw_);
    msg.pose.orientation.x = command_orientation.x;
    msg.pose.orientation.y = command_orientation.y;
    msg.pose.orientation.z = command_orientation.z;
    msg.pose.orientation.w = command_orientation.w;
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
        takeoff_sent_ = false;
        RCLCPP_WARN(
          get_logger(), "Takeoff command rejected by FCU (MAV_RESULT=%u); will retry while armed",
          static_cast<unsigned int>(result->result));
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

  static double horizontal_heading_from_quaternion(const Quaternion & q) {
    const double forward_east = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double forward_north = 2.0 * (q.x * q.y + q.w * q.z);
    return std::atan2(forward_north, forward_east);
  }

  static Quaternion yaw_quaternion(double yaw) {
    return Quaternion{0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
  }

  static std::string state_name(MissionState state) {
    switch (state) {
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::WAITING_NAV_STABLE: return "WAITING_NAV_STABLE";
      case MissionState::PRESTREAM_HOLD: return "PRESTREAM_HOLD";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::DROP_SEARCH: return "DROP_SEARCH";
      case MissionState::DROP_ALIGN_COARSE: return "DROP_ALIGN_COARSE";
      case MissionState::DROP_ALIGN_FINE: return "DROP_ALIGN_FINE";
      case MissionState::DROP_RELEASE: return "DROP_RELEASE";
      case MissionState::DROP_RECOVER: return "DROP_RECOVER";
      case MissionState::RECON_SURVEY: return "RECON_SURVEY";
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
  double drop_search_alt_ = 4.2;
  double drop_coarse_alt_ = 3.4;
  double drop_fine_alt_ = 3.0;
  double drop_search_edge_margin_m_ = 0.35;
  double drop_search_cross_margin_m_ = 0.55;
  double coarse_body_error_m_ = 0.45;
  double fine_body_error_m_ = 0.055;
  double release_body_error_m_ = 0.055;
  double coarse_stable_s_ = 0.8;
  double fine_stable_s_ = 0.9;
  double drop_lost_timeout_s_ = 3.0;
  double track_gate_m_ = 0.85;
  double align_initial_track_grace_s_ = 10.0;
  double known_bucket_memory_s_ = 180.0;
  double drop_target_timeout_s_ = 30.0;
  double fine_align_timeout_s_ = 12.0;
  double timeout_release_body_error_m_ = 0.45;
  double b_zone_final_approach_timeout_s_ = 10.0;
  double align_retarget_interval_s_ = 0.65;
  double align_retarget_delta_m_ = 0.45;
  double align_target_smoothing_ = 0.30;
  double drop_release_hold_s_ = 0.6;
  double post_drop_climb_hold_s_ = 0.4;
  double field_drop_size_x_ = 5.0;
  double field_drop_size_y_ = 8.0;
  int drop_search_lane_count_ = 3;
  int bucket_min_confirmations_ = 3;
  int release_gate_max_retries_ = 4;
  int max_drop_search_passes_ = 3;
  int payload_count_ = 2;
  int payload_index_ = 0;
  int search_pass_ = 0;
  int release_gate_retries_ = 0;
  double drop_settle_s_ = 2.0;
  double post_drop_hold_s_ = 0.8;
  double accept_radius_ = 0.35;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 420.0;
  double home_lock_stability_s_ = 2.0;
  double home_lock_max_xy_jump_m_ = 0.35;
  double home_lock_max_z_jump_m_ = 0.35;
  double max_abs_local_position_m_ = 1000.0;
  double local_pose_timeout_s_ = 0.75;
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
  double field_yaw_offset_rad_ = 0.0;
  bool use_generated_scene_ = true;
  bool yaw_to_target_ = false;
  bool lock_mission_yaw_to_initial_heading_ = false;
  bool require_fcu_home_before_arm_ = true;
  bool servo_return_to_stowed_ = true;
  bool servo_initialize_stowed_ = true;
  bool use_camera_target_point_body_ = false;
  bool allow_degraded_timeout_release_ = true;
  bool use_known_bucket_after_release_ = true;
  bool search_until_candidates_for_payloads_ = true;
  bool first_payload_require_500_and_300_ = true;
  bool recover_hold_started_ = false;
  std::string generated_scene_path_;
  std::string scene_coordinate_mode_;
  std::string bucket_detection_topic_ = "/perception/drop_buckets_body";
  std::string release_mode_ = "virtual";
  std::string force_release_reason_;
  std::vector<double> payload_release_offsets_body_;
  std::vector<double> payload_drop_bias_body_;
  std::vector<double> camera_offset_body_;
  std::array<double, 9> camera_optical_to_body_rotation_{-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0};
  std::vector<double> camera_target_point_camera_;
  std::vector<double> camera_target_point_body_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> servo_release_pwm_;
  std::vector<int64_t> servo_stowed_pwm_;
  std::vector<double> servo_release_duration_s_;
  std::deque<HeadingSample> heading_samples_;
  Point3 vehicle_spawn_world_;
  Point3 field_drop_center_{30.0, 0.0, 0.0};
  Quaternion current_attitude_;
  Quaternion locked_orientation_;

  mavros_msgs::msg::State current_state_;
  Point3 current_position_;
  std::optional<Point3> home_;
  Point3 target_;
  std::optional<BucketTrack> active_bucket_;
  std::optional<Point3> last_align_target_;
  std::vector<BucketTrack> known_buckets_;
  std::vector<BucketTrack> search_candidates_;
  std::vector<Point3> released_buckets_;
  std::vector<Point3> skipped_buckets_;
  std::vector<ReconTarget> recon_targets_;
  std::vector<Point3> fallback_scan_waypoints_;
  std::vector<Waypoint> scan_route_;
  std::size_t scan_index_ = 0;
  Segment active_segment_;

  MissionState mission_state_ = MissionState::WAITING_FCU;
  bool have_local_position_ = false;
  bool have_local_heading_ = false;
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
  bool have_last_local_pose_time_ = false;

  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_state_log_;
  rclcpp::Time last_yaw_update_time_;
  rclcpp::Time hold_start_time_;
  rclcpp::Time release_start_time_;
  rclcpp::Time recover_hold_start_time_;
  std::optional<rclcpp::Time> align_stable_since_;
  std::optional<rclcpp::Time> last_align_retarget_;
  std::optional<rclcpp::Time> target_lock_time_;
  std::optional<rclcpp::Time> b_zone_fuse_start_time_;
  rclcpp::Time position_stable_since_;
  rclcpp::Time last_service_request_time_;
  rclcpp::Time last_local_pose_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
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
