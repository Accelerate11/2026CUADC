/*
 * 视觉任务状态机公开参考版
 *
 * 说明：
 * 1. 本文件根据项目完整状态机整理为适合公开仓库展示的参考实现。
 * 2. 公开版保留 ROS2/MAVROS 接口、任务阶段、状态机、安全接管和模块边界。
 * 3. 公开版主动移除了比赛版本中的关键实现细节，包括但不限于：
 *    - 视觉帧与导航历史的精确时间对齐策略；
 *    - 多目标轨迹融合、重绑定、置信度融合与目标排序策略；
 *    - 搜索航线和侦察航线的真实生成方法及现场参数；
 *    - 投放口刚体几何补偿、姿态补偿、延迟预测和精对准闭环；
 *    - 比赛实测门限、速度曲线、机构外参和舵机参数。
 * 4. 被隐藏的部分只保留“输入、输出、功能目标和调用位置”，便于读者理解总体思路。
 * 5. 本文件默认关闭自动飞行和真实载荷动作，不能直接作为比赛或真实飞行程序使用。
 *
 * 许可证：请以公开仓库根目录中的 LICENSE 文件为准。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <functional>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kPublicVersion = "public-reference-2026";

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Segment
{
  Point3 start;
  Point3 end;
  SteadyTimePoint start_time;
  double duration_s = 1.0;
};

struct PublicTrack
{
  std::size_t id = 0U;
  Point3 local;
  std::size_t confirmations = 0U;
  SteadyTimePoint last_seen;
};

enum class State
{
  WAIT_FCU,
  WAIT_NAV_STABLE,
  PRESTREAM,
  WAIT_GUIDED,
  WAIT_ARM,
  TAKEOFF,
  SEARCH,
  ALIGN_COARSE,
  ALIGN_FINE,
  RELEASE,
  RECON_TRANSIT,
  RECON_DESCEND,
  RECON_SCAN,
  RECON_RETURN,
  RETURN_HOME,
  LAND,
  DISARM,
  DONE,
  PILOT_OVERRIDE,
  ABORT
};

enum class ServoPurpose
{
  RELEASE,
  STOW
};

struct PendingServoCommand
{
  ServoPurpose purpose = ServoPurpose::RELEASE;
  std::size_t payload = 0U;
  SteadyTimePoint sent;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future;
};

double normalize_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double normalize_degrees(double value)
{
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance_xyz(const Point3 & a, const Point3 & b)
{
  return std::hypot(distance_xy(a, b), a.z - b.z);
}

}  // 命名空间结束

class VisualMissionPublicNode final : public rclcpp::Node
{
public:
  VisualMissionPublicNode()
  : Node("visual_mission_public_node")
  {
    declare_parameters();
    load_parameters();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&VisualMissionPublicNode::state_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&VisualMissionPublicNode::odom_callback, this, std::placeholders::_1));

    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&VisualMissionPublicNode::compass_callback, this, std::placeholders::_1));

    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualMissionPublicNode::bucket_callback, this, std::placeholders::_1));

    recon_ack_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/cuadc/recon/capture_done", 10,
      std::bind(&VisualMissionPublicNode::recon_ack_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

    recon_photo_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/cuadc/recon/photo_mode", 10);

    recon_capture_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/cuadc/recon/capture_request", 10);

    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    const auto now_steady = SteadyClock::now();
    state_enter_time_ = now_steady;
    mission_start_time_ = now_steady;
    last_odom_arrival_ = now_steady;
    last_compass_arrival_ = now_steady;
    last_request_time_ = now_steady - 2s;
    last_status_time_ = now_steady;

    timer_ = create_wall_timer(50ms, std::bind(&VisualMissionPublicNode::tick, this));

    RCLCPP_WARN(
      get_logger(),
      "公开参考版已启动 [%s]。默认关闭自动飞行和真实载荷动作，不能直接用于比赛飞行。",
      kPublicVersion);
  }

private:
  void declare_parameters()
  {
    declare_parameter<bool>("flight_enable", false);
    declare_parameter<bool>("auto_arm_on_guided", false);
    declare_parameter<bool>("payload_actuation_enable", false);
    declare_parameter<bool>("simulate_release_when_actuation_disabled", true);

    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");

    /*
     * 公开版不提供比赛实测高度和速度。
     * 这些参数默认使用无效值，必须由使用者在自己的平台上重新设计、仿真和验证。
     */
    declare_parameter<double>("takeoff_alt_m", -1.0);
    declare_parameter<double>("search_alt_m", -1.0);
    declare_parameter<double>("coarse_alt_m", -1.0);
    declare_parameter<double>("fine_alt_m", -1.0);
    declare_parameter<double>("recon_alt_m", -1.0);
    declare_parameter<double>("return_alt_m", -1.0);

    declare_parameter<double>("transit_speed_m_s", -1.0);
    declare_parameter<double>("search_speed_m_s", -1.0);
    declare_parameter<double>("align_speed_m_s", -1.0);
    declare_parameter<double>("fine_speed_m_s", -1.0);
    declare_parameter<double>("return_speed_m_s", -1.0);

    /*
     * 航线采用外部参数直接输入，公开版不包含比赛场地航线生成算法。
     * 数组格式为 x1,y1,x2,y2,...，坐标位于任务场地平面坐标系。
     */
    declare_parameter<std::vector<double>>("search_route_field_xy", std::vector<double>{});
    declare_parameter<std::vector<double>>("recon_route_field_xy", std::vector<double>{});
    declare_parameter<double>("field_lateral_offset_m", 0.0);

    /*
     * 下列门限只用于公开示例的简单逻辑，不代表原项目的实测参数。
     */
    declare_parameter<int>("public_min_confirmations", 3);
    declare_parameter<double>("public_track_gate_m", 0.8);
    declare_parameter<double>("public_track_timeout_s", 1.0);
    declare_parameter<double>("public_coarse_radius_m", 0.50);
    declare_parameter<double>("public_fine_radius_m", 0.25);
    declare_parameter<double>("public_height_tolerance_m", 0.20);
    declare_parameter<double>("public_stable_hold_s", 0.50);
    declare_parameter<double>("waypoint_accept_radius_m", 0.50);

    declare_parameter<double>("odom_timeout_s", 1.5);
    declare_parameter<double>("compass_timeout_s", 1.5);
    declare_parameter<double>("prestream_hold_s", 1.0);
    declare_parameter<double>("takeoff_timeout_s", 45.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("service_ack_timeout_s", 3.0);

    declare_parameter<double>("landing_altitude_m", 0.30);
    declare_parameter<double>("landing_horizontal_speed_m_s", 0.25);
    declare_parameter<double>("landing_vertical_speed_m_s", 0.20);
    declare_parameter<double>("landing_stable_s", 1.0);

    /*
     * 真实机构通道、PWM 和机构外参属于平台配置，不在公开代码中给出。
     * 如需接入自己的机构，请在 YAML 中显式填写。
     */
    declare_parameter<int>("payload_count", 2);
    declare_parameter<std::vector<int64_t>>("servo_channels", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("servo_stowed_pwm", std::vector<int64_t>{});
    declare_parameter<std::vector<int64_t>>("servo_release_pwm", std::vector<int64_t>{});
    declare_parameter<double>("servo_release_hold_s", 0.5);
    declare_parameter<double>("servo_ack_timeout_s", 3.0);
  }

  void load_parameters()
  {
    flight_enable_ = get_parameter("flight_enable").as_bool();
    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();
    payload_actuation_enable_ = get_parameter("payload_actuation_enable").as_bool();
    simulate_release_when_actuation_disabled_ =
      get_parameter("simulate_release_when_actuation_disabled").as_bool();

    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();

    takeoff_alt_m_ = get_parameter("takeoff_alt_m").as_double();
    search_alt_m_ = get_parameter("search_alt_m").as_double();
    coarse_alt_m_ = get_parameter("coarse_alt_m").as_double();
    fine_alt_m_ = get_parameter("fine_alt_m").as_double();
    recon_alt_m_ = get_parameter("recon_alt_m").as_double();
    return_alt_m_ = get_parameter("return_alt_m").as_double();

    transit_speed_m_s_ = get_parameter("transit_speed_m_s").as_double();
    search_speed_m_s_ = get_parameter("search_speed_m_s").as_double();
    align_speed_m_s_ = get_parameter("align_speed_m_s").as_double();
    fine_speed_m_s_ = get_parameter("fine_speed_m_s").as_double();
    return_speed_m_s_ = get_parameter("return_speed_m_s").as_double();

    search_route_field_xy_ = get_parameter("search_route_field_xy").as_double_array();
    recon_route_field_xy_ = get_parameter("recon_route_field_xy").as_double_array();
    field_lateral_offset_m_ = get_parameter("field_lateral_offset_m").as_double();

    public_min_confirmations_ = std::max(
      1, static_cast<int>(get_parameter("public_min_confirmations").as_int()));
    public_track_gate_m_ = std::max(0.10, get_parameter("public_track_gate_m").as_double());
    public_track_timeout_s_ = std::max(
      0.10, get_parameter("public_track_timeout_s").as_double());
    public_coarse_radius_m_ = std::max(
      0.05, get_parameter("public_coarse_radius_m").as_double());
    public_fine_radius_m_ = std::max(
      0.03, get_parameter("public_fine_radius_m").as_double());
    public_height_tolerance_m_ = std::max(
      0.03, get_parameter("public_height_tolerance_m").as_double());
    public_stable_hold_s_ = std::max(
      0.10, get_parameter("public_stable_hold_s").as_double());
    waypoint_accept_radius_m_ = std::max(
      0.10, get_parameter("waypoint_accept_radius_m").as_double());

    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    compass_timeout_s_ = std::max(0.2, get_parameter("compass_timeout_s").as_double());
    prestream_hold_s_ = std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ = std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ = std::max(30.0, get_parameter("mission_timeout_s").as_double());
    service_ack_timeout_s_ = std::max(
      0.5, get_parameter("service_ack_timeout_s").as_double());

    landing_altitude_m_ = std::max(
      0.05, get_parameter("landing_altitude_m").as_double());
    landing_horizontal_speed_m_s_ = std::max(
      0.05, get_parameter("landing_horizontal_speed_m_s").as_double());
    landing_vertical_speed_m_s_ = std::max(
      0.05, get_parameter("landing_vertical_speed_m_s").as_double());
    landing_stable_s_ = std::max(
      0.2, get_parameter("landing_stable_s").as_double());

    payload_count_ = std::max(
      1, static_cast<int>(get_parameter("payload_count").as_int()));
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    servo_stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    servo_release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    servo_release_hold_s_ = std::max(
      0.1, get_parameter("servo_release_hold_s").as_double());
    servo_ack_timeout_s_ = std::max(
      0.5, get_parameter("servo_ack_timeout_s").as_double());

    config_valid_ = validate_public_configuration();
  }

  bool validate_public_configuration() const
  {
    if (!flight_enable_) {
      return false;
    }

    const bool flight_values_ready =
      takeoff_alt_m_ > 0.0 && search_alt_m_ > 0.0 && coarse_alt_m_ > 0.0 &&
      fine_alt_m_ > 0.0 && recon_alt_m_ > 0.0 && return_alt_m_ > 0.0 &&
      transit_speed_m_s_ > 0.0 && search_speed_m_s_ > 0.0 &&
      align_speed_m_s_ > 0.0 && fine_speed_m_s_ > 0.0 && return_speed_m_s_ > 0.0;

    const bool route_values_ready =
      search_route_field_xy_.size() >= 4U && search_route_field_xy_.size() % 2U == 0U &&
      recon_route_field_xy_.size() >= 2U && recon_route_field_xy_.size() % 2U == 0U;

    if (!flight_values_ready || !route_values_ready) {
      return false;
    }

    if (!payload_actuation_enable_) {
      return true;
    }

    return servo_channels_.size() >= static_cast<std::size_t>(payload_count_) &&
      servo_stowed_pwm_.size() >= static_cast<std::size_t>(payload_count_) &&
      servo_release_pwm_.size() >= static_cast<std::size_t>(payload_count_);
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (was_guided && !guided_active_ && is_automatic_state(state_)) {
      publish_setpoint_enabled_ = false;
      RCLCPP_WARN(get_logger(), "检测到飞手离开 GUIDED，自动任务立即停止并交还控制权。");
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
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;

    const auto & q = msg->pose.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1.0e-6) {
      const double qx = q.x / norm;
      const double qy = q.y / norm;
      const double qz = q.z / norm;
      const double qw = q.w / norm;
      current_roll_ = std::atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy));
      current_pitch_ = std::asin(std::clamp(
        2.0 * (qw * qy - qz * qx), -1.0, 1.0));
    }

    have_odom_ = true;
    last_odom_arrival_ = SteadyClock::now();
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }

    current_compass_deg_ = normalize_degrees(msg->data);
    current_yaw_enu_ = normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    have_compass_ = true;
    last_compass_arrival_ = SteadyClock::now();
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!frame_locked_ || !accepting_visual_targets()) {
      return;
    }

    /*
     * 核心算法隐藏说明：
     * 完整版本会按视觉采集时刻查找导航历史，并结合当时的位置、姿态和任务坐标系
     * 完成严格的时空对齐。公开版不提供这套时间同步和姿态补偿方法。
     * 这里仅用“当前航向 + 当前机体中心”的简化变换展示接口关系。
     */
    for (const auto & pose : msg->poses) {
      const Point3 body{
        pose.position.x,
        pose.position.y,
        pose.position.z};

      if (!std::isfinite(body.x) || !std::isfinite(body.y) || !std::isfinite(body.z)) {
        continue;
      }

      const Point3 local = public_body_to_local(body);
      merge_public_track(local);
    }

    remove_stale_public_tracks();
  }

  void recon_ack_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    const std::size_t id = static_cast<std::size_t>(msg->data);
    if (id == 0U) {
      return;
    }
    if (std::find(recon_saved_ids_.begin(), recon_saved_ids_.end(), id) == recon_saved_ids_.end()) {
      recon_saved_ids_.push_back(id);
    }
  }

  Point3 public_body_to_local(const Point3 & body) const
  {
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);

    return Point3{
      position_.x + c * body.x - s * body.y,
      position_.y + s * body.x + c * body.y,
      position_.z + body.z};
  }

  void merge_public_track(const Point3 & detection)
  {
    /*
     * 核心算法隐藏说明：
     * 正式版本使用更严格的多维门控、历史一致性、稳定性统计和目标重绑定。
     * 公开版只保留最基础的平面最近邻示例，不能反映比赛版本的跟踪性能。
     */
    auto best = public_tracks_.end();
    double best_distance = std::numeric_limits<double>::infinity();

    for (auto it = public_tracks_.begin(); it != public_tracks_.end(); ++it) {
      const double d = distance_xy(it->local, detection);
      if (d < best_distance && d <= public_track_gate_m_) {
        best = it;
        best_distance = d;
      }
    }

    if (best == public_tracks_.end()) {
      PublicTrack track;
      track.id = next_track_id_++;
      track.local = detection;
      track.confirmations = 1U;
      track.last_seen = SteadyClock::now();
      public_tracks_.push_back(track);
      return;
    }

    constexpr double kPublicExampleAlpha = 0.35;
    best->local.x = (1.0 - kPublicExampleAlpha) * best->local.x + kPublicExampleAlpha * detection.x;
    best->local.y = (1.0 - kPublicExampleAlpha) * best->local.y + kPublicExampleAlpha * detection.y;
    best->local.z = (1.0 - kPublicExampleAlpha) * best->local.z + kPublicExampleAlpha * detection.z;
    ++best->confirmations;
    best->last_seen = SteadyClock::now();
  }

  void remove_stale_public_tracks()
  {
    public_tracks_.erase(
      std::remove_if(
        public_tracks_.begin(), public_tracks_.end(),
        [this](const PublicTrack & track) {
          return steady_age_s(track.last_seen) > public_track_timeout_s_;
        }),
      public_tracks_.end());
  }

  bool try_lock_public_target_plan()
  {
    if (target_plan_locked_) {
      return true;
    }

    std::vector<PublicTrack> ready;
    for (const auto & track : public_tracks_) {
      if (track.confirmations >= static_cast<std::size_t>(public_min_confirmations_)) {
        ready.push_back(track);
      }
    }

    if (ready.size() < static_cast<std::size_t>(payload_count_)) {
      return false;
    }

    /*
     * 核心算法隐藏说明：
     * 比赛版本会依据任务规则和目标质量对候选目标进行稳定排序，并保证多个目标之间
     * 的空间独立性。公开版故意不公开该评分函数，仅按轨迹编号选择前若干目标。
     */
    std::sort(
      ready.begin(), ready.end(),
      [](const PublicTrack & a, const PublicTrack & b) {
        return a.id < b.id;
      });

    selected_target_ids_.clear();
    selected_target_positions_.clear();

    for (int i = 0; i < payload_count_; ++i) {
      selected_target_ids_.push_back(ready[static_cast<std::size_t>(i)].id);
      selected_target_positions_.push_back(ready[static_cast<std::size_t>(i)].local);
    }

    target_plan_locked_ = true;
    return true;
  }

  std::optional<Point3> current_payload_target() const
  {
    if (!target_plan_locked_ ||
      payload_index_ >= selected_target_positions_.size())
    {
      return std::nullopt;
    }
    return selected_target_positions_[payload_index_];
  }

  std::optional<Point3> latest_track_position(std::size_t id) const
  {
    for (const auto & track : public_tracks_) {
      if (track.id == id && steady_age_s(track.last_seen) <= public_track_timeout_s_) {
        return track.local;
      }
    }
    return std::nullopt;
  }

  Point3 field_to_local(double field_x, double field_y, double relative_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    const double corrected_y = field_y + field_lateral_offset_m_;

    return Point3{
      origin.x + c * field_x - s * corrected_y,
      origin.y + s * field_x + c * corrected_y,
      origin.z + relative_z};
  }

  std::vector<Point3> build_route_from_parameter(
    const std::vector<double> & flat_xy,
    double relative_z) const
  {
    std::vector<Point3> route;
    for (std::size_t i = 0U; i + 1U < flat_xy.size(); i += 2U) {
      route.push_back(field_to_local(flat_xy[i], flat_xy[i + 1U], relative_z));
    }
    return route;
  }

  bool odom_fresh() const
  {
    return have_odom_ && steady_age_s(last_odom_arrival_) <= odom_timeout_s_;
  }

  bool compass_fresh() const
  {
    return have_compass_ && steady_age_s(last_compass_arrival_) <= compass_timeout_s_;
  }

  bool navigation_ready_to_lock() const
  {
    /*
     * 核心算法隐藏说明：
     * 正式版本对航向稳定性、静止状态、视觉健康度和导航频率有额外门禁。
     * 公开版只展示最基本的新鲜度检查。
     */
    return odom_fresh() && compass_fresh() && !fcu_state_.armed;
  }

  void lock_frame()
  {
    home_ = position_;
    mission_yaw_ = current_yaw_enu_;
    locked_compass_deg_ = current_compass_deg_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);

    search_route_local_ = build_route_from_parameter(search_route_field_xy_, search_alt_m_);
    recon_route_local_ = build_route_from_parameter(recon_route_field_xy_, recon_alt_m_);

    target_ = *home_;
    frame_locked_ = true;

    RCLCPP_INFO(
      get_logger(),
      "任务坐标系已锁定。公开版航线由参数直接读取，不包含真实航线生成算法。");
  }

  bool accepting_visual_targets() const
  {
    return state_ == State::SEARCH ||
      state_ == State::ALIGN_COARSE ||
      state_ == State::ALIGN_FINE;
  }

  bool is_automatic_state(State state) const
  {
    return state == State::WAIT_ARM ||
      state == State::TAKEOFF ||
      state == State::SEARCH ||
      state == State::ALIGN_COARSE ||
      state == State::ALIGN_FINE ||
      state == State::RELEASE ||
      state == State::RECON_TRANSIT ||
      state == State::RECON_DESCEND ||
      state == State::RECON_SCAN ||
      state == State::RECON_RETURN ||
      state == State::RETURN_HOME ||
      state == State::LAND ||
      state == State::DISARM;
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      abort_or_land("飞行过程中飞控连接或解锁状态异常");
      return false;
    }

    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }

    if (!odom_fresh()) {
      abort_or_land("飞行过程中里程计数据超时");
      return false;
    }

    return true;
  }

  void tick()
  {
    check_service_results();
    check_servo_results();

    if (publish_setpoint_enabled_ && frame_locked_) {
      publish_setpoint();
    }

    if (mission_started_ &&
      state_ != State::LAND &&
      state_ != State::DISARM &&
      state_ != State::DONE &&
      state_ != State::PILOT_OVERRIDE &&
      state_ != State::ABORT &&
      steady_age_s(mission_start_time_) > mission_timeout_s_)
    {
      RCLCPP_ERROR(get_logger(), "任务总超时，进入返航流程。");
      start_return_home();
    }

    if (steady_age_s(last_status_time_) >= 4.0) {
      last_status_time_ = SteadyClock::now();
      RCLCPP_INFO(
        get_logger(),
        "状态=%s 模式=%s 解锁=%s 位置=(%.1f, %.1f, %.1f) 载荷序号=%zu",
        state_name(state_).c_str(),
        fcu_state_.mode.c_str(),
        fcu_state_.armed ? "是" : "否",
        position_.x, position_.y, position_.z,
        payload_index_ + 1U);
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_enabled_ = false;
        if (!flight_enable_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "公开版 flight_enable=false，节点只展示结构，不会进入自动飞行。");
          break;
        }
        if (!config_valid_) {
          RCLCPP_ERROR(get_logger(), "公开版配置不完整，拒绝进入自动飞行。");
          enter(State::ABORT);
        } else if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (navigation_ready_to_lock()) {
          lock_frame();
          publish_setpoint_enabled_ = true;
          enter(State::PRESTREAM);
        }
        break;

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!odom_fresh() || !compass_fresh()) {
          abort_or_land("进入任务前导航数据失效");
        } else if (steady_age_s(state_enter_time_) >= prestream_hold_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = home_.value_or(position_);
        if (!fcu_state_.connected) {
          enter(State::ABORT);
        } else if (guided_active_) {
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = home_.value_or(position_);
        if (!guided_active_) {
          enter(State::WAIT_GUIDED);
        } else if (fcu_state_.armed) {
          mission_started_ = true;
          mission_start_time_ = SteadyClock::now();
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
        } else if (steady_age_s(state_enter_time_) > takeoff_timeout_s_) {
          abort_or_land("起飞超时");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;

      case State::ALIGN_COARSE:
        if (flight_gate_ok()) {
          update_alignment_coarse();
        }
        break;

      case State::ALIGN_FINE:
        if (flight_gate_ok()) {
          update_alignment_fine();
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

      case State::RECON_DESCEND:
        if (flight_gate_ok()) {
          update_recon_descend();
        }
        break;

      case State::RECON_SCAN:
        if (flight_gate_ok()) {
          update_recon_scan();
        }
        break;

      case State::RECON_RETURN:
        if (flight_gate_ok()) {
          start_return_home();
        }
        break;

      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          update_return_home();
        }
        break;

      case State::LAND:
        publish_setpoint_enabled_ = false;
        if (update_landing_confirmation()) {
          if (fcu_state_.armed) {
            enter(State::DISARM);
          } else {
            enter(State::DONE);
          }
        } else if (fcu_state_.armed) {
          request_land();
        }
        break;

      case State::DISARM:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.armed) {
          enter(State::DONE);
        } else if (landing_confirmation_ready()) {
          request_arm(false);
        }
        break;

      case State::DONE:
        publish_setpoint_enabled_ = false;
        if (steady_age_s(state_enter_time_) > 1.0) {
          RCLCPP_INFO(get_logger(), "公开版任务流程结束。");
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_enabled_ = false;
        if (steady_age_s(state_enter_time_) > 1.0) {
          RCLCPP_WARN(get_logger(), "飞手已接管，公开版节点退出。");
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          RCLCPP_ERROR(get_logger(), "任务在起飞前终止。");
          rclcpp::shutdown();
        }
        break;
    }
  }

  void start_search()
  {
    if (search_route_local_.empty()) {
      abort_or_land("公开版未提供搜索航线参数");
      return;
    }

    search_index_ = 0U;
    alignment_stable_since_.reset();
    start_segment(position_, search_route_local_.front(), transit_speed_m_s_);
    enter(State::SEARCH);
  }

  void update_search()
  {
    /*
     * 核心算法隐藏说明：
     * 正式版本会将“目标数量、轨迹稳定性、空间独立性、任务优先级、搜索剩余航程”
     * 联合起来决定何时结束搜索。公开版只在满足最小目标数后结束搜索。
     */
    if (try_lock_public_target_plan()) {
      payload_index_ = 0U;
      alignment_stable_since_.reset();
      enter(State::ALIGN_COARSE);
      return;
    }

    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }

    ++search_index_;
    if (search_index_ >= search_route_local_.size()) {
      if (!try_lock_public_target_plan()) {
        abort_or_land("公开示例未找到足够稳定目标");
        return;
      }
      enter(State::ALIGN_COARSE);
      return;
    }

    start_segment(position_, search_route_local_[search_index_], search_speed_m_s_);
  }

  Point3 public_desired_pose_for_target(const Point3 & target_local, double relative_altitude) const
  {
    /*
     * 核心算法隐藏说明：
     * 正式版本根据机体姿态、任务航向、载荷机构安装位置和动作延迟计算真实投放点，
     * 再反求飞机中心应到达的位置。公开版故意省略该几何闭环，仅让机体中心靠近目标。
     */
    const Point3 origin = home_.value_or(Point3{});
    return Point3{target_local.x, target_local.y, origin.z + relative_altitude};
  }

  void update_alignment_coarse()
  {
    const auto frozen = current_payload_target();
    if (!frozen.has_value()) {
      abort_or_land("粗对准阶段缺少冻结目标");
      return;
    }

    Point3 target_local = *frozen;
    if (payload_index_ < selected_target_ids_.size()) {
      const auto live = latest_track_position(selected_target_ids_[payload_index_]);
      if (live.has_value()) {
        target_local = *live;
        selected_target_positions_[payload_index_] = *live;
      }
    }

    const Point3 desired = public_desired_pose_for_target(target_local, coarse_alt_m_);
    slew_target_toward(desired, align_speed_m_s_);

    const bool aligned =
      distance_xy(position_, desired) <= public_coarse_radius_m_ &&
      std::abs(position_.z - desired.z) <= public_height_tolerance_m_;

    if (!update_stable_hold(aligned)) {
      return;
    }

    alignment_stable_since_.reset();
    enter(State::ALIGN_FINE);
  }

  void update_alignment_fine()
  {
    const auto frozen = current_payload_target();
    if (!frozen.has_value()) {
      abort_or_land("细对准阶段缺少冻结目标");
      return;
    }

    Point3 target_local = *frozen;
    if (payload_index_ < selected_target_ids_.size()) {
      const auto live = latest_track_position(selected_target_ids_[payload_index_]);
      if (live.has_value()) {
        target_local = *live;
        selected_target_positions_[payload_index_] = *live;
      }
    }

    /*
     * 核心算法隐藏说明：
     * 完整版本的细对准包含视觉新鲜度判定、目标身份约束、短时丢失策略、姿态补偿和
     * 实际投放点误差闭环。公开版仅展示“降低高度后进行第二阶段对准”的状态机思路。
     */
    const Point3 desired = public_desired_pose_for_target(target_local, fine_alt_m_);
    slew_target_toward(desired, fine_speed_m_s_);

    const bool aligned =
      distance_xy(position_, desired) <= public_fine_radius_m_ &&
      std::abs(position_.z - desired.z) <= public_height_tolerance_m_;

    if (!update_stable_hold(aligned)) {
      return;
    }

    release_target_local_ = target_local;
    alignment_stable_since_.reset();
    release_stable_since_.reset();
    release_command_sent_ = false;
    release_ack_received_ = false;
    stow_command_pending_ = false;
    release_completed_ = false;
    enter(State::RELEASE);
  }

  void update_release()
  {
    if (!release_target_local_.has_value()) {
      abort_or_land("投放阶段缺少目标位置");
      return;
    }

    const Point3 desired = public_desired_pose_for_target(*release_target_local_, fine_alt_m_);
    slew_target_toward(desired, fine_speed_m_s_);

    /*
     * 公开版只保留通用安全门概念，不公开比赛版本的真实误差、速度、姿态、角速度和
     * 连续稳定门限组合。这里的参数仅用于演示如何组织门禁逻辑。
     */
    const bool position_ok = distance_xy(position_, desired) <= public_fine_radius_m_;
    const bool height_ok = std::abs(position_.z - desired.z) <= public_height_tolerance_m_;
    const bool motion_ok = horizontal_speed_m_s_ < 0.30 && std::abs(vertical_speed_m_s_) < 0.20;
    const bool attitude_ok = std::abs(current_roll_) < 15.0 * kPi / 180.0 &&
      std::abs(current_pitch_) < 15.0 * kPi / 180.0;

    const bool release_gate = position_ok && height_ok && motion_ok && attitude_ok;

    if (!release_gate) {
      release_stable_since_.reset();
      return;
    }

    if (!release_stable_since_.has_value()) {
      release_stable_since_ = SteadyClock::now();
      return;
    }

    if (steady_age_s(*release_stable_since_) < public_stable_hold_s_) {
      return;
    }

    if (!payload_actuation_enable_) {
      if (simulate_release_when_actuation_disabled_ && !release_completed_) {
        RCLCPP_WARN(
          get_logger(),
          "真实载荷动作已关闭，公开版仅模拟完成第 %zu 个载荷流程。",
          payload_index_ + 1U);
        release_completed_ = true;
        finish_payload_release();
      }
      return;
    }

    if (!release_command_sent_) {
      release_command_sent_ = send_servo(payload_index_, true, ServoPurpose::RELEASE);
    }

    if (release_completed_) {
      finish_payload_release();
    }
  }

  void finish_payload_release()
  {
    if (!release_completed_) {
      return;
    }

    release_target_local_.reset();
    release_stable_since_.reset();
    release_completed_ = false;
    release_command_sent_ = false;
    release_ack_received_ = false;
    stow_command_pending_ = false;

    ++payload_index_;
    if (payload_index_ < static_cast<std::size_t>(payload_count_)) {
      alignment_stable_since_.reset();
      enter(State::ALIGN_COARSE);
      return;
    }

    start_recon_phase();
  }

  void start_recon_phase()
  {
    if (recon_route_local_.empty()) {
      start_return_home();
      return;
    }

    std_msgs::msg::Bool photo_mode;
    photo_mode.data = true;
    recon_photo_mode_pub_->publish(photo_mode);

    recon_index_ = 0U;
    recon_saved_ids_.clear();

    Point3 transit_target = recon_route_local_.front();
    transit_target.z = home_->z + return_alt_m_;
    start_segment(position_, transit_target, transit_speed_m_s_);
    enter(State::RECON_TRANSIT);
  }

  void update_recon_transit()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }

    Point3 descend_target = recon_route_local_.front();
    descend_target.z = home_->z + recon_alt_m_;
    start_segment(position_, descend_target, align_speed_m_s_);
    enter(State::RECON_DESCEND);
  }

  void update_recon_descend()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }

    recon_index_ = 0U;
    start_segment(position_, recon_route_local_.front(), search_speed_m_s_);
    enter(State::RECON_SCAN);
  }

  void update_recon_scan()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }

    request_recon_photo(recon_index_);
    ++recon_index_;

    if (recon_index_ >= recon_route_local_.size()) {
      std_msgs::msg::Bool photo_mode;
      photo_mode.data = false;
      recon_photo_mode_pub_->publish(photo_mode);
      enter(State::RECON_RETURN);
      return;
    }

    start_segment(position_, recon_route_local_[recon_index_], search_speed_m_s_);
  }

  void request_recon_photo(std::size_t index)
  {
    if (index >= recon_route_local_.size()) {
      return;
    }

    geometry_msgs::msg::PointStamped message;
    message.header.stamp = now();
    message.header.frame_id = "recon_wp_" + std::to_string(index + 1U);
    message.point.x = recon_route_local_[index].x;
    message.point.y = recon_route_local_[index].y;
    message.point.z = recon_route_local_[index].z;
    recon_capture_pub_->publish(message);
  }

  void start_return_home()
  {
    if (!home_.has_value()) {
      enter(State::ABORT);
      return;
    }

    Point3 return_target = *home_;
    return_target.z = home_->z + return_alt_m_;
    start_segment(position_, return_target, return_speed_m_s_);
    enter(State::RETURN_HOME);
  }

  void update_return_home()
  {
    target_ = sample_segment();
    if (!segment_complete(std::max(waypoint_accept_radius_m_, 0.50))) {
      return;
    }

    publish_setpoint_enabled_ = false;
    enter(State::LAND);
  }

  bool update_stable_hold(bool condition)
  {
    if (!condition) {
      alignment_stable_since_.reset();
      return false;
    }

    if (!alignment_stable_since_.has_value()) {
      alignment_stable_since_ = SteadyClock::now();
      return false;
    }

    return steady_age_s(*alignment_stable_since_) >= public_stable_hold_s_;
  }

  void start_segment(const Point3 & start, const Point3 & end, double speed_m_s)
  {
    segment_.start = start;
    segment_.end = end;
    segment_.start_time = SteadyClock::now();

    const double distance = distance_xyz(start, end);
    const double safe_speed = std::max(0.10, speed_m_s);
    segment_.duration_s = std::max(0.5, distance / safe_speed);
    target_ = start;
  }

  Point3 sample_segment() const
  {
    const double elapsed = steady_age_s(segment_.start_time);
    const double t = std::clamp(elapsed / segment_.duration_s, 0.0, 1.0);

    return Point3{
      segment_.start.x + t * (segment_.end.x - segment_.start.x),
      segment_.start.y + t * (segment_.end.y - segment_.start.y),
      segment_.start.z + t * (segment_.end.z - segment_.start.z)};
  }

  bool segment_complete(double radius) const
  {
    return distance_xyz(position_, segment_.end) <= radius &&
      steady_age_s(segment_.start_time) >= segment_.duration_s * 0.8;
  }

  void slew_target_toward(const Point3 & desired, double speed_m_s)
  {
    const double dt = 0.05;
    const double max_step = std::max(0.01, speed_m_s * dt);

    const double dx = desired.x - target_.x;
    const double dy = desired.y - target_.y;
    const double dz = desired.z - target_.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance <= max_step || distance < 1.0e-6) {
      target_ = desired;
      return;
    }

    const double scale = max_step / distance;
    target_.x += dx * scale;
    target_.y += dy * scale;
    target_.z += dz * scale;
  }

  bool send_servo(std::size_t payload, bool release, ServoPurpose purpose)
  {
    if (!payload_actuation_enable_ || !command_client_->service_is_ready()) {
      return false;
    }

    if (payload >= servo_channels_.size() ||
      payload >= servo_stowed_pwm_.size() ||
      payload >= servo_release_pwm_.size())
    {
      return false;
    }

    const int64_t channel = servo_channels_[payload];
    const int64_t pwm = release ? servo_release_pwm_[payload] : servo_stowed_pwm_[payload];
    if (channel <= 0 || pwm <= 0) {
      return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = 183;
    request->confirmation = 0;
    request->param1 = static_cast<float>(channel);
    request->param2 = static_cast<float>(pwm);

    PendingServoCommand pending;
    pending.purpose = purpose;
    pending.payload = payload;
    pending.sent = SteadyClock::now();
    pending.future = command_client_->async_send_request(request).future.share();
    pending_servo_commands_.push_back(std::move(pending));
    return true;
  }

  void check_servo_results()
  {
    for (auto it = pending_servo_commands_.begin(); it != pending_servo_commands_.end();) {
      const bool ready = it->future.wait_for(0s) == std::future_status::ready;
      const bool timed_out = steady_age_s(it->sent) > servo_ack_timeout_s_;

      if (!ready && !timed_out) {
        ++it;
        continue;
      }

      const bool accepted = ready && it->future.get()->success;

      if (it->purpose == ServoPurpose::RELEASE) {
        if (!accepted) {
          abort_or_land("载荷动作命令未获得确认");
        } else {
          release_hold_start_ = SteadyClock::now();
          release_ack_received_ = true;
        }
      } else {
        stow_command_pending_ = false;
        if (!accepted) {
          abort_or_land("载荷机构回收命令未获得确认");
        } else {
          release_completed_ = true;
        }
      }

      it = pending_servo_commands_.erase(it);
    }

    if (release_ack_received_ && !stow_command_pending_ &&
      steady_age_s(release_hold_start_) >= servo_release_hold_s_)
    {
      stow_command_pending_ = send_servo(payload_index_, false, ServoPurpose::STOW);
      if (!stow_command_pending_) {
        abort_or_land("无法发送载荷机构回收命令");
      }
      release_ack_received_ = false;
    }
  }

  void publish_setpoint()
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.pose.position.x = target_.x;
    message.pose.position.y = target_.y;
    message.pose.position.z = target_.z;
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
    if (!takeoff_client_->service_is_ready() || takeoff_future_.valid() || !request_allowed()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(takeoff_alt_m_);
    request->yaw = static_cast<float>(locked_compass_deg_);

    mark_request();
    takeoff_request_time_ = SteadyClock::now();
    takeoff_future_ = takeoff_client_->async_send_request(request).future.share();
    takeoff_sent_ = true;
  }

  void request_land()
  {
    if (!land_client_->service_is_ready() || land_future_.valid() || !request_allowed()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->yaw = static_cast<float>(locked_compass_deg_);

    mark_request();
    land_request_time_ = SteadyClock::now();
    land_future_ = land_client_->async_send_request(request).future.share();
  }

  void request_arm(bool arm)
  {
    if (!arm_client_->service_is_ready() || arm_future_.valid() || !request_allowed()) {
      return;
    }

    if (arm && !auto_arm_on_guided_) {
      return;
    }

    if (!arm && !landing_confirmation_ready()) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;

    pending_arm_value_ = arm;
    mark_request();
    arm_request_time_ = SteadyClock::now();
    arm_future_ = arm_client_->async_send_request(request).future.share();
  }

  void check_service_results()
  {
    if (takeoff_future_.valid()) {
      const bool ready = takeoff_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out = steady_age_s(takeoff_request_time_) > service_ack_timeout_s_;

      if (ready) {
        const auto response = takeoff_future_.get();
        if (!response->success) {
          takeoff_sent_ = false;
        }
        takeoff_future_ = {};
      } else if (timed_out) {
        takeoff_future_ = {};
        takeoff_sent_ = false;
      }
    }

    if (land_future_.valid()) {
      const bool ready = land_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out = steady_age_s(land_request_time_) > service_ack_timeout_s_;

      if (ready || timed_out) {
        if (ready) {
          static_cast<void>(land_future_.get());
        }
        land_future_ = {};
      }
    }

    if (arm_future_.valid()) {
      const bool ready = arm_future_.wait_for(0s) == std::future_status::ready;
      const bool timed_out = steady_age_s(arm_request_time_) > service_ack_timeout_s_;

      if (ready || timed_out) {
        if (ready) {
          static_cast<void>(arm_future_.get());
        }
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
    return home_.has_value() && odom_fresh() &&
      relative_altitude() <= landing_altitude_m_ &&
      horizontal_speed_m_s_ <= landing_horizontal_speed_m_s_ &&
      std::abs(vertical_speed_m_s_) <= landing_vertical_speed_m_s_;
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

    return steady_age_s(*landing_stable_since_) >= landing_stable_s_;
  }

  bool landing_confirmation_ready() const
  {
    return landing_stable_since_.has_value() &&
      landing_candidate() &&
      steady_age_s(*landing_stable_since_) >= landing_stable_s_;
  }

  void abort_or_land(const std::string & reason)
  {
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());

    if (fcu_state_.armed) {
      enter(State::LAND);
    } else {
      enter(State::ABORT);
    }
  }

  void enter(State next)
  {
    if (state_ == next) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "状态切换：%s -> %s",
      state_name(state_).c_str(),
      state_name(next).c_str());

    state_ = next;
    state_enter_time_ = SteadyClock::now();

    if (next != State::ALIGN_COARSE && next != State::ALIGN_FINE) {
      alignment_stable_since_.reset();
    }
  }

  static std::string state_name(State state)
  {
    switch (state) {
      case State::WAIT_FCU: return "WAIT_FCU";
      case State::WAIT_NAV_STABLE: return "WAIT_NAV_STABLE";
      case State::PRESTREAM: return "PRESTREAM";
      case State::WAIT_GUIDED: return "WAIT_GUIDED";
      case State::WAIT_ARM: return "WAIT_ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::SEARCH: return "SEARCH";
      case State::ALIGN_COARSE: return "ALIGN_COARSE";
      case State::ALIGN_FINE: return "ALIGN_FINE";
      case State::RELEASE: return "RELEASE";
      case State::RECON_TRANSIT: return "RECON_TRANSIT";
      case State::RECON_DESCEND: return "RECON_DESCEND";
      case State::RECON_SCAN: return "RECON_SCAN";
      case State::RECON_RETURN: return "RECON_RETURN";
      case State::RETURN_HOME: return "RETURN_HOME";
      case State::LAND: return "LAND";
      case State::DISARM: return "DISARM";
      case State::DONE: return "DONE";
      case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
      case State::ABORT: return "ABORT";
    }
    return "UNKNOWN";
  }

  static double steady_age_s(const SteadyTimePoint & time_point)
  {
    return std::chrono::duration<double>(SteadyClock::now() - time_point).count();
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr recon_ack_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recon_photo_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr recon_capture_pub_;

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State fcu_state_;
  State state_ = State::WAIT_FCU;

  Point3 position_;
  Point3 target_;
  Segment segment_;
  std::optional<Point3> home_;
  std::optional<Point3> release_target_local_;

  bool have_odom_ = false;
  bool have_compass_ = false;
  bool guided_active_ = false;
  bool frame_locked_ = false;
  bool config_valid_ = false;
  bool mission_started_ = false;
  bool publish_setpoint_enabled_ = false;
  bool takeoff_sent_ = false;

  double current_compass_deg_ = 0.0;
  double current_yaw_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;

  SteadyTimePoint state_enter_time_;
  SteadyTimePoint mission_start_time_;
  SteadyTimePoint last_odom_arrival_;
  SteadyTimePoint last_compass_arrival_;
  SteadyTimePoint last_request_time_;
  SteadyTimePoint last_status_time_;
  SteadyTimePoint takeoff_request_time_;
  SteadyTimePoint land_request_time_;
  SteadyTimePoint arm_request_time_;
  SteadyTimePoint release_hold_start_;

  std::optional<SteadyTimePoint> alignment_stable_since_;
  std::optional<SteadyTimePoint> release_stable_since_;
  std::optional<SteadyTimePoint> landing_stable_since_;

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
  std::vector<PendingServoCommand> pending_servo_commands_;

  bool pending_arm_value_ = false;
  bool payload_actuation_enable_ = false;
  bool simulate_release_when_actuation_disabled_ = true;
  bool release_command_sent_ = false;
  bool release_ack_received_ = false;
  bool release_completed_ = false;
  bool stow_command_pending_ = false;

  bool flight_enable_ = false;
  bool auto_arm_on_guided_ = false;
  std::string bucket_topic_;

  double takeoff_alt_m_ = -1.0;
  double search_alt_m_ = -1.0;
  double coarse_alt_m_ = -1.0;
  double fine_alt_m_ = -1.0;
  double recon_alt_m_ = -1.0;
  double return_alt_m_ = -1.0;

  double transit_speed_m_s_ = -1.0;
  double search_speed_m_s_ = -1.0;
  double align_speed_m_s_ = -1.0;
  double fine_speed_m_s_ = -1.0;
  double return_speed_m_s_ = -1.0;

  std::vector<double> search_route_field_xy_;
  std::vector<double> recon_route_field_xy_;
  double field_lateral_offset_m_ = 0.0;

  int public_min_confirmations_ = 3;
  double public_track_gate_m_ = 0.8;
  double public_track_timeout_s_ = 1.0;
  double public_coarse_radius_m_ = 0.50;
  double public_fine_radius_m_ = 0.25;
  double public_height_tolerance_m_ = 0.20;
  double public_stable_hold_s_ = 0.50;
  double waypoint_accept_radius_m_ = 0.50;

  double odom_timeout_s_ = 1.5;
  double compass_timeout_s_ = 1.5;
  double prestream_hold_s_ = 1.0;
  double takeoff_timeout_s_ = 45.0;
  double mission_timeout_s_ = 240.0;
  double service_ack_timeout_s_ = 3.0;

  double landing_altitude_m_ = 0.30;
  double landing_horizontal_speed_m_s_ = 0.25;
  double landing_vertical_speed_m_s_ = 0.20;
  double landing_stable_s_ = 1.0;

  int payload_count_ = 2;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> servo_stowed_pwm_;
  std::vector<int64_t> servo_release_pwm_;
  double servo_release_hold_s_ = 0.5;
  double servo_ack_timeout_s_ = 3.0;

  std::vector<PublicTrack> public_tracks_;
  std::size_t next_track_id_ = 1U;
  bool target_plan_locked_ = false;
  std::vector<std::size_t> selected_target_ids_;
  std::vector<Point3> selected_target_positions_;

  std::vector<Point3> search_route_local_;
  std::vector<Point3> recon_route_local_;
  std::size_t search_index_ = 0U;
  std::size_t recon_index_ = 0U;
  std::size_t payload_index_ = 0U;
  std::vector<std::size_t> recon_saved_ids_;

  std::string terminal_reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualMissionPublicNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
