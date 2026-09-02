/*
 * CUADC 2026 自主任务节点 - 公开/社区版
 *
 * 目的
 * -------
 * 本文件是内部比赛任务控制器的刻意简化公开版本。它保留了整体的
 * ROS 2 / MAVROS 架构、状态机组织、坐标系接口和安全联锁机制，
 * 同时刻意省略或替换了仍具有比赛价值的细节。
 *
 * 本公开版刻意未包含的内容
 * -------------------------------------------------
 * 1. 比赛目标追踪、去重、稳定性评分及排名。
 * 2. 标定后的相机/机体/投放口外参及载荷弹道学参数。
 * 3. 比赛场地几何结构、优化的搜索航线及边界恢复逻辑。
 * 4. 正式版相机采集 / 飞控(FCU)里程计的硬时间同步逻辑。
 * 5. 比赛投放阈值、重试、补偿及评分逻辑。
 * 6. 正式版侦察视点及特定阶段的速度调优。
 * 7. 任何基于评分规则得出的场地校准或目标选择策略。
 *
 * 因此，公开代码使用了保守的演示参数和简单的参考算法。其旨在用于
 * 学习和重新实现，而非直接用于比赛或在未经测试的飞行器上直接飞行。
 *
 * 在进行任何真实飞行前：请在仿真环境中进行测试，拆下螺旋桨进行执行器
 * 检查，验证舵机方向/PWM，验证坐标系惯例，并针对实际飞行器调整
 * 每一个安全阈值。
 *
 * V1 公开版变体说明
 * -----------------------
 * 本版本展示了保守/经典的任务架构：
 *   WAIT_FCU (等待飞控) -> WAIT_NAV_STABLE (等待导航稳定) -> PRESTREAM (预流) -> WAIT_GUIDED (等待指令模式) -> WAIT_ARM (等待解锁)
 *   -> TAKEOFF (起飞) -> SEARCH (搜索) -> ALIGN (对齐) -> RELEASE (投放) -> SEARCH/RECON (搜索/侦察)
 *   -> RETURN_HOME (返航) -> LAND (降落) -> DISARM (上锁) -> DONE (完成)
 *
 * 真实的 V1 分支包含多阶段目标对齐和投放口补偿。公开版保留了状态边界和扩展点，但使用了目标正上方对齐的演示对齐模型。
 * 由于cuadc_full_mission_node_3代码在反复实测中依旧保留较强的任务执行能力，后续的代码改动均为针对node_3的微调尝试。
 */


#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Detection
{
  std::size_t id = 0U;
  Point3 body;
  Point3 local;
  double size_hint = 0.0;
  double confidence = 0.0;
  rclcpp::Time stamp;
};

struct Segment
{
  Point3 start;
  Point3 end;
  rclcpp::Time start_time;
  double duration_s = 1.0;
};

double normalize_angle(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}

double normalize_degrees(double x)
{
  x = std::fmod(x, 360.0);
  return x < 0.0 ? x + 360.0 : x;
}

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance_xyz(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace


namespace
{
constexpr const char * kMissionVersion = "cuadc-community-v1-2026-09";

enum class State
{
  WAIT_FCU,
  WAIT_NAV_STABLE,
  PRESTREAM,
  WAIT_GUIDED,
  WAIT_ARM,
  TAKEOFF,
  SEARCH,
  ALIGN,
  RELEASE,
  RECON_TRANSIT,
  RECON_SCAN,
  RETURN_HOME,
  LAND,
  DISARM,
  DONE,
  PILOT_OVERRIDE,
  ABORT
};
}  // namespace

class CuadcPublicMissionV1 final : public rclcpp::Node
{
public:
  CuadcPublicMissionV1()
  : Node("cuadc_public_mission_v1")
  {
    declare_parameters();
    load_parameters();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&CuadcPublicMissionV1::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionV1::odom_callback, this, std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionV1::compass_callback, this, std::placeholders::_1));
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionV1::bucket_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    mission_state_pub_ = create_publisher<std_msgs::msg::String>(
      "/cuadc/mission_state", 10);
    recon_capture_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/cuadc/recon/capture_request", 10);
    recon_ack_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/cuadc/recon/capture_done", 10,
      std::bind(&CuadcPublicMissionV1::recon_ack_callback, this, std::placeholders::_1));

    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    state_enter_time_ = now();
    mission_start_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_odom_time_ = now();
    last_vision_time_ = now();
    segment_.start_time = now();

    timer_ = create_wall_timer(50ms, std::bind(&CuadcPublicMissionV1::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CUADC V1 public mission node ready [%s]. Competition-sensitive logic omitted.",
      kMissionVersion);
  }

private:
  // ---------------------------------------------------------------------------
  // Parameters: generic demo defaults, NOT competition tuning.
  // ---------------------------------------------------------------------------
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");

    declare_parameter<double>("takeoff_alt_m", 3.0);
    declare_parameter<double>("search_alt_m", 2.5);
    declare_parameter<double>("align_alt_m", 2.0);
    declare_parameter<double>("release_alt_m", 1.5);
    declare_parameter<double>("return_alt_m", 3.0);

    declare_parameter<double>("transit_speed_m_s", 1.5);
    declare_parameter<double>("search_speed_m_s", 0.8);
    declare_parameter<double>("return_speed_m_s", 1.5);

    declare_parameter<double>("waypoint_accept_radius_m", 0.45);
    declare_parameter<double>("align_accept_radius_m", 0.30);
    declare_parameter<double>("align_stable_s", 0.8);
    declare_parameter<double>("release_stable_s", 0.8);
    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("odom_timeout_s", 1.0);
    declare_parameter<double>("vision_timeout_s", 1.5);

    declare_parameter<bool>("auto_arm_on_guided", false);
    declare_parameter<bool>("enable_release_output", false);

    // Generic example actuator mapping only. Users MUST replace after bench testing.
    declare_parameter<std::vector<int64_t>>(
      "servo_channels", std::vector<int64_t>{9, 10});
    declare_parameter<std::vector<int64_t>>(
      "servo_stowed_pwm", std::vector<int64_t>{1100, 1100});
    declare_parameter<std::vector<int64_t>>(
      "servo_release_pwm", std::vector<int64_t>{1900, 1900});
    declare_parameter<double>("servo_release_hold_s", 0.5);

    // Demo mission rectangle; unrelated to competition field geometry.
    declare_parameter<double>("demo_search_x_min_m", 6.0);
    declare_parameter<double>("demo_search_x_max_m", 10.0);
    declare_parameter<double>("demo_search_half_width_m", 2.0);
    declare_parameter<int>("demo_search_lanes", 3);

    // Demo recon area; unrelated to competition recon waypoints.
    declare_parameter<double>("demo_recon_x_m", 14.0);
    declare_parameter<double>("demo_recon_half_width_m", 1.5);
  }

  void load_parameters()
  {
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();

    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
    align_alt_m_ = std::max(1.0, get_parameter("align_alt_m").as_double());
    release_alt_m_ = std::max(0.8, get_parameter("release_alt_m").as_double());
    return_alt_m_ = std::max(1.0, get_parameter("return_alt_m").as_double());

    transit_speed_m_s_ = std::max(0.2, get_parameter("transit_speed_m_s").as_double());
    search_speed_m_s_ = std::max(0.2, get_parameter("search_speed_m_s").as_double());
    return_speed_m_s_ = std::max(0.2, get_parameter("return_speed_m_s").as_double());

    waypoint_accept_radius_m_ = std::max(
      0.1, get_parameter("waypoint_accept_radius_m").as_double());
    align_accept_radius_m_ = std::max(
      0.05, get_parameter("align_accept_radius_m").as_double());
    align_stable_s_ = std::max(0.1, get_parameter("align_stable_s").as_double());
    release_stable_s_ = std::max(0.1, get_parameter("release_stable_s").as_double());
    prestream_hold_s_ = std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ = std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ = std::max(30.0, get_parameter("mission_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    vision_timeout_s_ = std::max(0.2, get_parameter("vision_timeout_s").as_double());

    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();
    enable_release_output_ = get_parameter("enable_release_output").as_bool();
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    servo_stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    servo_release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    servo_release_hold_s_ = std::max(
      0.1, get_parameter("servo_release_hold_s").as_double());

    demo_search_x_min_m_ = get_parameter("demo_search_x_min_m").as_double();
    demo_search_x_max_m_ = get_parameter("demo_search_x_max_m").as_double();
    demo_search_half_width_m_ = std::max(
      0.5, get_parameter("demo_search_half_width_m").as_double());
    demo_search_lanes_ = std::max(
      1, static_cast<int>(get_parameter("demo_search_lanes").as_int()));
    demo_recon_x_m_ = get_parameter("demo_recon_x_m").as_double();
    demo_recon_half_width_m_ = std::max(
      0.5, get_parameter("demo_recon_half_width_m").as_double());
  }

  // ---------------------------------------------------------------------------
  // ROS callbacks
  // ---------------------------------------------------------------------------
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (was_guided && !guided_active_ && autonomous_state(state_)) {
      RCLCPP_WARN(get_logger(), "Pilot left GUIDED; stopping autonomous control");
      publish_setpoint_enabled_ = false;
      enter(State::PILOT_OVERRIDE);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_ = Point3{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    current_heading_enu_ = normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    have_compass_ = true;
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    last_vision_time_ = now();
    have_vision_ = true;
    detections_.clear();
    detections_.reserve(msg->poses.size());

    std::size_t id = 1U;
    for (const auto & pose : msg->poses) {
      const Point3 body{pose.position.x, pose.position.y, pose.position.z};
      if (!std::isfinite(body.x) || !std::isfinite(body.y) || !std::isfinite(body.z)) {
        continue;
      }
      Detection d;
      d.id = id++;
      d.body = body;
      d.local = body_to_local_demo(body);
      d.size_hint = std::isfinite(pose.orientation.x) ? pose.orientation.x : 0.0;
      d.confidence = std::isfinite(pose.orientation.y) ? pose.orientation.y : 1.0;
      // Public edition intentionally does not expose production capture-time
      // synchronization; use reception time for the teaching example.
      d.stamp = now();
      detections_.push_back(d);
    }
  }

  void recon_ack_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    const std::uint32_t id = msg->data;
    if (std::find(recon_acks_.begin(), recon_acks_.end(), id) == recon_acks_.end()) {
      recon_acks_.push_back(id);
    }
  }

  // ---------------------------------------------------------------------------
  // Public/demo coordinate handling
  // ---------------------------------------------------------------------------
  Point3 body_to_local_demo(const Point3 & body) const
  {
    // Production code uses capture-time pose interpolation and calibrated
    // transforms.  Public version intentionally demonstrates only yaw rotation.
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{
      position_.x + c * body.x - s * body.y,
      position_.y + s * body.x + c * body.y,
      position_.z + body.z};
  }

  Point3 field_to_local(double x, double y, double rel_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{
      origin.x + c * x - s * y,
      origin.y + s * x + c * y,
      origin.z + rel_z};
  }

  void lock_frame()
  {
    home_ = position_;
    mission_yaw_ = current_heading_enu_;
    locked_compass_deg_ = current_compass_deg_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_demo_search_route();
    build_demo_recon_route();
  }

  void build_demo_search_route()
  {
    // Production route planning intentionally omitted.  This is a generic
    // lawn-mower rectangle unrelated to the competition field.
    search_route_.clear();
    const double x0 = std::min(demo_search_x_min_m_, demo_search_x_max_m_);
    const double x1 = std::max(demo_search_x_min_m_, demo_search_x_max_m_);
    for (int lane = 0; lane < demo_search_lanes_; ++lane) {
      const double t = demo_search_lanes_ == 1 ?
        0.5 : static_cast<double>(lane) / static_cast<double>(demo_search_lanes_ - 1);
      const double y = -demo_search_half_width_m_ + 2.0 * demo_search_half_width_m_ * t;
      const bool forward = lane % 2 == 0;
      search_route_.push_back(field_to_local(forward ? x0 : x1, y, search_alt_m_));
      search_route_.push_back(field_to_local(forward ? x1 : x0, y, search_alt_m_));
    }
  }

  void build_demo_recon_route()
  {
    recon_route_.clear();
    recon_route_.push_back(field_to_local(
      demo_recon_x_m_, -demo_recon_half_width_m_, search_alt_m_));
    recon_route_.push_back(field_to_local(
      demo_recon_x_m_ + 2.0, -demo_recon_half_width_m_, search_alt_m_));
    recon_route_.push_back(field_to_local(
      demo_recon_x_m_ + 2.0, demo_recon_half_width_m_, search_alt_m_));
    recon_route_.push_back(field_to_local(
      demo_recon_x_m_, demo_recon_half_width_m_, search_alt_m_));
  }

  // ---------------------------------------------------------------------------
  // Target-selection demo hook
  // ---------------------------------------------------------------------------
  std::optional<Detection> choose_demo_target() const
  {
    // Competition tracking/ranking is intentionally omitted.  This public
    // example only chooses the closest currently visible, unused detection.
    if (!vision_fresh()) {
      return std::nullopt;
    }

    const Detection * best = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & d : detections_) {
      bool used = false;
      for (const auto & p : released_target_points_) {
        if (distance_xy(d.local, p) < 0.6) {
          used = true;
          break;
        }
      }
      if (used) {
        continue;
      }
      const double distance = distance_xy(position_, d.local);
      if (distance < best_distance) {
        best_distance = distance;
        best = &d;
      }
    }
    return best ? std::optional<Detection>(*best) : std::nullopt;
  }

  Point3 desired_demo_align_pose(const Detection & d) const
  {
    // Production version aligns the calibrated physical release port, not the
    // vehicle center.  Public edition intentionally uses center-over-target.
    return Point3{d.local.x, d.local.y, home_->z + align_alt_m_};
  }

  Point3 desired_demo_release_pose(const Detection & d) const
  {
    return Point3{d.local.x, d.local.y, home_->z + release_alt_m_};
  }

  // ---------------------------------------------------------------------------
  // Main state machine
  // ---------------------------------------------------------------------------
  void tick()
  {
    check_service_results();
    if (publish_setpoint_enabled_ && frame_locked_) {
      publish_setpoint();
    }

    if (mission_started_ && state_ != State::RETURN_HOME && state_ != State::LAND &&
      state_ != State::DISARM && state_ != State::DONE &&
      state_ != State::PILOT_OVERRIDE && state_ != State::ABORT &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      fail_and_return("Mission timeout");
      return;
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail("Aircraft armed before mission frame lock");
        } else if (navigation_ready()) {
          lock_frame();
          publish_setpoint_enabled_ = true;
          enter(State::PRESTREAM);
        }
        break;

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!navigation_ready()) {
          fail("Navigation lost during prestream");
        } else if ((now() - state_enter_time_).seconds() >= prestream_hold_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = home_.value_or(position_);
        if (guided_active_) {
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = home_.value_or(position_);
        if (!guided_active_) {
          enter(State::WAIT_GUIDED);
        } else if (fcu_state_.armed) {
          mission_started_ = true;
          mission_start_time_ = now();
          publish_setpoint_enabled_ = false;
          enter(State::TAKEOFF);
        } else if (auto_arm_on_guided_) {
          request_arm(true);
        }
        break;

      case State::TAKEOFF:
        publish_setpoint_enabled_ = false;
        if (!flight_gate_ok()) {
          break;
        }
        if (!takeoff_sent_) {
          request_takeoff();
        }
        if (relative_altitude() >= takeoff_alt_m_ * 0.90) {
          publish_setpoint_enabled_ = true;
          start_search();
        } else if ((now() - state_enter_time_).seconds() > takeoff_timeout_s_) {
          fail_and_return("Takeoff timeout");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;
      case State::ALIGN:
        if (flight_gate_ok()) {
          update_align();
        }
        break;
      case State::RELEASE:
        if (flight_gate_ok()) {
          update_release();
        }
        break;
      case State::RECON_TRANSIT:
        if (flight_gate_ok()) {
          update_recon_transit();
        }
        break;
      case State::RECON_SCAN:
        if (flight_gate_ok()) {
          update_recon_scan();
        }
        break;
      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          update_return();
        }
        break;

      case State::LAND:
        publish_setpoint_enabled_ = false;
        if (landing_candidate()) {
          if (!landing_stable_since_.has_value()) {
            landing_stable_since_ = now();
          } else if ((now() - *landing_stable_since_).seconds() > 2.0) {
            enter(fcu_state_.armed ? State::DISARM : State::DONE);
          }
        } else {
          landing_stable_since_.reset();
          if (fcu_state_.armed) {
            request_land();
          }
        }
        break;

      case State::DISARM:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.armed) {
          enter(State::DONE);
        } else if (landing_candidate()) {
          request_arm(false);
        } else {
          enter(State::LAND);
        }
        break;

      case State::DONE:
        publish_setpoint_enabled_ = false;
        if (!done_logged_) {
          done_logged_ = true;
          RCLCPP_INFO(
            get_logger(), "V1 public mission ended: payloads=%zu recon_acks=%zu failed=%s",
            payload_index_, recon_acks_.size(), mission_failed_ ? "true" : "false");
        }
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_enabled_ = false;
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          rclcpp::shutdown();
        }
        break;
    }
  }

  // ---------------------------------------------------------------------------
  // Search / align / release
  // ---------------------------------------------------------------------------
  void start_search()
  {
    current_target_.reset();
    align_stable_since_.reset();
    release_stable_since_.reset();
    if (search_route_.empty()) {
      fail_and_return("Demo search route is empty");
      return;
    }
    if (search_index_ >= search_route_.size()) {
      search_index_ = 0U;
    }
    start_segment(position_, search_route_[search_index_], search_speed_m_s_);
    enter(State::SEARCH);
  }

  void update_search()
  {
    const auto candidate = choose_demo_target();
    if (candidate.has_value()) {
      current_target_ = candidate;
      target_ = desired_demo_align_pose(*current_target_);
      align_stable_since_.reset();
      enter(State::ALIGN);
      return;
    }

    target_ = sample_segment();
    if (segment_complete(waypoint_accept_radius_m_)) {
      ++search_index_;
      if (search_index_ >= search_route_.size()) {
        fail_and_return("No target found during public demo search");
        return;
      }
      start_segment(position_, search_route_[search_index_], search_speed_m_s_);
    }
  }

  void update_align()
  {
    if (!current_target_.has_value()) {
      start_search();
      return;
    }

    // Simple demo refresh: only accept a nearby visible detection.
    const auto refreshed = choose_demo_target();
    if (refreshed.has_value() &&
      distance_xy(refreshed->local, current_target_->local) < 0.8)
    {
      current_target_ = refreshed;
    }

    target_ = desired_demo_align_pose(*current_target_);
    const bool aligned =
      distance_xy(position_, target_) <= align_accept_radius_m_ &&
      std::abs(position_.z - target_.z) <= 0.20;

    if (!aligned) {
      align_stable_since_.reset();
      return;
    }
    if (!align_stable_since_.has_value()) {
      align_stable_since_ = now();
      return;
    }
    if ((now() - *align_stable_since_).seconds() < align_stable_s_) {
      return;
    }

    // V1 public keeps the historical two-stage interface, but not the real
    // precision descent / release-port model.
    frozen_release_pose_ = desired_demo_release_pose(*current_target_);
    target_ = frozen_release_pose_;
    release_stable_since_.reset();
    enter(State::RELEASE);
  }

  void update_release()
  {
    if (!current_target_.has_value()) {
      fail_and_return("Release entered without demo target");
      return;
    }

    target_ = frozen_release_pose_;
    const bool at_release_pose =
      distance_xy(position_, frozen_release_pose_) <= align_accept_radius_m_ &&
      std::abs(position_.z - frozen_release_pose_.z) <= 0.15 &&
      horizontal_speed_m_s_ <= 0.25 && std::abs(vertical_speed_m_s_) <= 0.20;

    if (!at_release_pose) {
      release_stable_since_.reset();
      return;
    }
    if (!release_stable_since_.has_value()) {
      release_stable_since_ = now();
      return;
    }
    if ((now() - *release_stable_since_).seconds() < release_stable_s_) {
      return;
    }

    if (!release_command_sent_) {
      if (enable_release_output_) {
        if (!send_demo_servo(payload_index_, true)) {
          fail_and_return("Public demo release output could not be sent");
          return;
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "RELEASE OUTPUT DISABLED: simulating payload %zu completion", payload_index_ + 1U);
      }
      release_command_sent_ = true;
      release_command_time_ = now();
      return;
    }

    if ((now() - release_command_time_).seconds() < servo_release_hold_s_) {
      return;
    }

    if (enable_release_output_) {
      (void)send_demo_servo(payload_index_, false);
    }

    released_target_points_.push_back(current_target_->local);
    ++payload_index_;
    current_target_.reset();
    release_command_sent_ = false;
    release_stable_since_.reset();

    if (payload_index_ >= 2U) {
      start_recon();
    } else {
      start_search();
    }
  }

  // ---------------------------------------------------------------------------
  // Recon / return / landing
  // ---------------------------------------------------------------------------
  void start_recon()
  {
    if (recon_route_.empty()) {
      start_return();
      return;
    }
    recon_index_ = 0U;
    start_segment(position_, recon_route_.front(), transit_speed_m_s_);
    enter(State::RECON_TRANSIT);
  }

  void update_recon_transit()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }
    request_demo_photo(recon_index_);
    enter(State::RECON_SCAN);
  }

  void update_recon_scan()
  {
    if (recon_ack_received(recon_index_)) {
      ++recon_index_;
      if (recon_index_ >= recon_route_.size()) {
        start_return();
        return;
      }
      start_segment(position_, recon_route_[recon_index_], transit_speed_m_s_);
      enter(State::RECON_TRANSIT);
      return;
    }

    // Public edition does not block forever if an external demo camera is absent.
    if ((now() - state_enter_time_).seconds() > 1.0) {
      ++recon_index_;
      if (recon_index_ >= recon_route_.size()) {
        start_return();
      } else {
        start_segment(position_, recon_route_[recon_index_], transit_speed_m_s_);
        enter(State::RECON_TRANSIT);
      }
    }
  }

  void start_return()
  {
    if (!home_.has_value()) {
      fail("Return requested without home");
      return;
    }
    start_segment(
      position_, Point3{home_->x, home_->y, home_->z + return_alt_m_}, return_speed_m_s_);
    enter(State::RETURN_HOME);
  }

  void update_return()
  {
    target_ = sample_segment();
    if (segment_complete(std::max(waypoint_accept_radius_m_, 0.5))) {
      publish_setpoint_enabled_ = false;
      enter(State::LAND);
    }
  }

  bool landing_candidate() const
  {
    return odom_fresh() && std::abs(relative_altitude()) <= 0.15 &&
      horizontal_speed_m_s_ <= 0.20 && std::abs(vertical_speed_m_s_) <= 0.15;
  }

  // ---------------------------------------------------------------------------
  // Generic motion / MAVROS helpers
  // ---------------------------------------------------------------------------
  void start_segment(const Point3 & start, const Point3 & end, double speed)
  {
    const double duration = std::max(0.5, distance_xyz(start, end) / std::max(0.1, speed));
    segment_ = Segment{start, end, now(), duration};
  }

  Point3 sample_segment() const
  {
    const double t = std::clamp(
      (now() - segment_.start_time).seconds() / segment_.duration_s, 0.0, 1.0);
    const double smooth = t * t * (3.0 - 2.0 * t);
    return Point3{
      segment_.start.x + smooth * (segment_.end.x - segment_.start.x),
      segment_.start.y + smooth * (segment_.end.y - segment_.start.y),
      segment_.start.z + smooth * (segment_.end.z - segment_.start.z)};
  }

  bool segment_complete(double radius) const
  {
    return (now() - segment_.start_time).seconds() >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  bool navigation_ready() const
  {
    return fcu_state_.connected && have_odom_ && have_compass_ && odom_fresh();
  }

  bool odom_fresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  bool vision_fresh() const
  {
    return have_vision_ && (now() - last_vision_time_).seconds() <= vision_timeout_s_;
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    if (!odom_fresh()) {
      fail_and_return("Odometry stale during mission");
      return false;
    }
    return true;
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  bool autonomous_state(State s) const
  {
    return s == State::WAIT_ARM || s == State::TAKEOFF || s == State::SEARCH ||
      s == State::ALIGN || s == State::RELEASE || s == State::RECON_TRANSIT ||
      s == State::RECON_SCAN || s == State::RETURN_HOME;
  }

  void publish_setpoint()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_.x;
    msg.pose.position.y = target_.y;
    msg.pose.position.z = target_.z;
    msg.pose.orientation.z = yaw_qz_;
    msg.pose.orientation.w = yaw_qw_;
    setpoint_pub_->publish(msg);
  }

  bool request_allowed() const
  {
    return (now() - last_request_time_).seconds() >= 1.0;
  }

  void request_takeoff()
  {
    if (!request_allowed() || takeoff_future_.valid() || !takeoff_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(takeoff_alt_m_);
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    takeoff_future_ = takeoff_client_->async_send_request(req).future.share();
    takeoff_sent_ = true;
  }

  void request_land()
  {
    if (!request_allowed() || land_future_.valid() || !land_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    land_future_ = land_client_->async_send_request(req).future.share();
  }

  void request_arm(bool arm)
  {
    if (!request_allowed() || arm_future_.valid() || !arm_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    last_request_time_ = now();
    arm_future_ = arm_client_->async_send_request(req).future.share();
  }

  bool send_demo_servo(std::size_t payload, bool release)
  {
    if (!enable_release_output_ || payload >= servo_channels_.size() ||
      payload >= servo_release_pwm_.size() || payload >= servo_stowed_pwm_.size() ||
      !command_client_->service_is_ready())
    {
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast = false;
    req->command = static_cast<std::uint16_t>(183U);
    req->param1 = static_cast<float>(servo_channels_[payload]);
    req->param2 = static_cast<float>(
      release ? servo_release_pwm_[payload] : servo_stowed_pwm_[payload]);
    command_client_->async_send_request(req);
    return true;
  }

  void check_service_results()
  {
    if (takeoff_future_.valid() && takeoff_future_.wait_for(0s) == std::future_status::ready) {
      const auto res = takeoff_future_.get();
      if (!res->success) {
        takeoff_sent_ = false;
      }
      takeoff_future_ = {};
    }
    if (land_future_.valid() && land_future_.wait_for(0s) == std::future_status::ready) {
      (void)land_future_.get();
      land_future_ = {};
    }
    if (arm_future_.valid() && arm_future_.wait_for(0s) == std::future_status::ready) {
      (void)arm_future_.get();
      arm_future_ = {};
    }
  }

  void request_demo_photo(std::size_t index)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.point.x = static_cast<double>(index);
    if (index < recon_route_.size()) {
      msg.point.y = recon_route_[index].x;
      msg.point.z = recon_route_[index].y;
    }
    recon_capture_pub_->publish(msg);
  }

  bool recon_ack_received(std::size_t index) const
  {
    return std::find(
      recon_acks_.begin(), recon_acks_.end(), static_cast<std::uint32_t>(index)) != recon_acks_.end();
  }

  void fail(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "PUBLIC_MISSION_ABORT: %s", reason.c_str());
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  void fail_and_return(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "PUBLIC_MISSION_FAILURE: %s", reason.c_str());
    if (fcu_state_.armed && home_.has_value() && guided_active_ && odom_fresh()) {
      start_return();
    } else {
      enter(fcu_state_.armed ? State::LAND : State::ABORT);
    }
  }

  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_enter_time_ = now();
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    mission_state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "STATE -> %s", msg.data.c_str());
  }

  static std::string state_name(State s)
  {
    switch (s) {
      case State::WAIT_FCU: return "WAIT_FCU";
      case State::WAIT_NAV_STABLE: return "WAIT_NAV_STABLE";
      case State::PRESTREAM: return "PRESTREAM";
      case State::WAIT_GUIDED: return "WAIT_GUIDED";
      case State::WAIT_ARM: return "WAIT_ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::SEARCH: return "SEARCH";
      case State::ALIGN: return "ALIGN";
      case State::RELEASE: return "RELEASE";
      case State::RECON_TRANSIT: return "RECON_TRANSIT";
      case State::RECON_SCAN: return "RECON_SCAN";
      case State::RETURN_HOME: return "RETURN_HOME";
      case State::LAND: return "LAND";
      case State::DISARM: return "DISARM";
      case State::DONE: return "DONE";
      case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
      case State::ABORT: return "ABORT";
    }
    return "UNKNOWN";
  }

  // ROS I/O
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr recon_ack_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr recon_capture_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;

  // Mission state
  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  Point3 position_;
  Point3 target_;
  Point3 frozen_release_pose_;
  std::optional<Point3> home_;
  std::optional<Detection> current_target_;
  Segment segment_;
  std::vector<Detection> detections_;
  std::vector<Point3> released_target_points_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;
  std::vector<std::uint32_t> recon_acks_;
  std::optional<rclcpp::Time> align_stable_since_;
  std::optional<rclcpp::Time> release_stable_since_;
  std::optional<rclcpp::Time> landing_stable_since_;

  std::string bucket_topic_;
  std::string terminal_reason_;
  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool have_vision_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_enabled_ = false;
  bool mission_started_ = false;
  bool mission_failed_ = false;
  bool done_logged_ = false;
  bool takeoff_sent_ = false;
  bool auto_arm_on_guided_ = false;
  bool enable_release_output_ = false;
  bool release_command_sent_ = false;

  std::size_t search_index_ = 0U;
  std::size_t payload_index_ = 0U;
  std::size_t recon_index_ = 0U;

  double takeoff_alt_m_ = 3.0;
  double search_alt_m_ = 2.5;
  double align_alt_m_ = 2.0;
  double release_alt_m_ = 1.5;
  double return_alt_m_ = 3.0;
  double transit_speed_m_s_ = 1.5;
  double search_speed_m_s_ = 0.8;
  double return_speed_m_s_ = 1.5;
  double waypoint_accept_radius_m_ = 0.45;
  double align_accept_radius_m_ = 0.30;
  double align_stable_s_ = 0.8;
  double release_stable_s_ = 0.8;
  double prestream_hold_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double odom_timeout_s_ = 1.0;
  double vision_timeout_s_ = 1.5;
  double servo_release_hold_s_ = 0.5;
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;
  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;
  double demo_search_x_min_m_ = 6.0;
  double demo_search_x_max_m_ = 10.0;
  double demo_search_half_width_m_ = 2.0;
  int demo_search_lanes_ = 3;
  double demo_recon_x_m_ = 14.0;
  double demo_recon_half_width_m_ = 1.5;

  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> servo_stowed_pwm_;
  std::vector<int64_t> servo_release_pwm_;

  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_vision_time_;
  rclcpp::Time release_command_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuadcPublicMissionV1>());
  rclcpp::shutdown();
  return 0;
}
