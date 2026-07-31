/*
 * 航向锁定与视觉双投放逻辑
 *
 * 一、锁定机头方向
 * 1. 使用 /mavros/global_position/compass_hdg 获取真实航向，不再使用
 *    可能固定为 0 度的 odom 四元数 yaw 作为主要航向。
 * 2. compass_hdg 为正北 0 度、顺时针增加，转换为 ROS ENU：
 *      yaw_enu = radians(90.0 - compass_hdg_deg)
 * 3. 飞机静止且航向稳定后，同时锁定 mission_home、mission_yaw 和
 *    field_yaw；场地 +X 始终对应启动时机头正前方。
 * 4. PRESTREAM、GUIDED、ARMING、TAKEOFF 及全任务持续发布同一固定 yaw；
 *    CommandTOL 起飞/降落显式填写锁定航向，yaw_to_target 必须为 false。
 * 5. /mavros/local_position/odom 是唯一主要位置源；只从其姿态提取 roll、
 *    pitch 补偿下视投影，yaw 仍仅取 compass_hdg。
 *
 * 二、三个桶与相对高价值目标
 * 1. 单帧检测在 local ENU 中做一对一关联，每个 track 每帧最多确认一次。
 * 2. local 位置使用可标定 EMA，直径使用独立 track 的滑窗中位数和 MAD；
 *    机体系坐标默认使用最新帧，避免飞机移动时跨位置拖尾。
 * 3. 必须取得三个近期、稳定、空间独立的桶，且相对直径排序稳定后，
 *    冻结最小桶和次小桶的持久 ID；第一瓶、第二瓶分别只跟踪这两个 ID。
 * 4. PoseArray.header.stamp 必须是相机取帧时刻；使用 odom 历史插值到
 *    该时刻后再转 local ENU，拒绝零戳、过旧/未来戳和乱序帧。
 * 5. 视觉心跳或时间对齐失效时立即禁止投放；导航健康则先返航再降落，
 *    否则直接受控降落。视觉进程退出不得同时关闭任务节点和 MAVROS。
 * 6. 对齐总超时只进入强制投放候选：仍须保持冻结目标 ID/位置/直径，
 *    目标新鲜且误差不超过 0.20 m，并在投放区内、1.2 m 高度、低速度、
 *    小姿态/角速度条件连续稳定 0.8 s；8 s 内不满足则保瓶返航。
 *
 * 三、实际投放设置
 *    CUAV V5 Nano：投放物 1 -> 输出通道 7，投放物 2 -> 输出通道 8。
 *    1100 us = 挂载位置，1900 us = 投放位置。
 *    使用 MAV_CMD_DO_SET_SERVO（command=183）直接控制：
 *      启动：CH7=1100、CH8=1100
 *      第一次：CH7=1900，保持 0.7 s，再回 CH7=1100
 *      第二次：CH8=1900，保持 0.7 s，再回 CH8=1100
 *    投放口机体 FLU 外参：
 *      投放口 1 = [ 0.026, -0.065, -0.32 ]
 *      投放口 2 = [-0.026,  0.065, -0.32 ]
 *    FLU：+X 机头前方，+Y 机体左侧，+Z 机体上方。
 *
 * 四、落地与上锁确认
 * 1. 降落命令 ACK 不是落地完成；必须由 /mavros/extended_state 连续收到
 *    多条独立 ON_GROUND，并同时满足相对高度绝对值、水平/垂直速度门。
 * 2. 上述条件的首末消息跨越 1.5 s 后才请求 disarm；最终仍以
 *    /mavros/state.armed=false 为任务结束依据。
 *
 * 五、比赛状态流
 *    锁定航向/起点 -> 飞手切 GUIDED -> 安全门禁通过后状态机自动解锁
 *    -> 自动定点起飞
 *    -> 规则投放区搜索三个桶 -> 投最小桶 -> 投次小桶 -> 自动返航
 *    -> 自动降落 -> 多消息落地确认 -> 显式上锁。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
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

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

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

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance_xyz(const Point3 & a, const Point3 & b)
{
  return std::hypot(distance_xy(a, b), a.z - b.z);
}

struct HeadingSample
{
  rclcpp::Time stamp;
  double yaw_enu = 0.0;
};

struct NavigationSample
{
  rclcpp::Time stamp;
  Point3 position;
  double roll = 0.0;
  double pitch = 0.0;
};

struct Segment
{
  Point3 start;
  Point3 end;
  rclcpp::Time start_time;
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
  RECOVER,
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
  RETURN_STOWED
};

struct PendingServoCommand
{
  ServoPurpose purpose = ServoPurpose::INITIALIZE;
  std::size_t payload = 0U;
  rclcpp::Time sent;
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

}  // namespace

class VisualDropMissionNode final : public rclcpp::Node
{
public:
  VisualDropMissionNode()
  : Node("visual_drop_mission_node")
  {
    declare_parameters();
    load_parameters();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&VisualDropMissionNode::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&VisualDropMissionNode::odom_callback, this, std::placeholders::_1));
    extended_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", rclcpp::SensorDataQoS(),
      std::bind(
        &VisualDropMissionNode::extended_state_callback,
        this, std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&VisualDropMissionNode::compass_callback, this, std::placeholders::_1));
    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisualDropMissionNode::bucket_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>(
      "/mavros/cmd/command");

    state_enter_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_servo_attempt_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_status_time_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&VisualDropMissionNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Visual drop node ready: manual GUIDED + %s arm, autonomous two-bottle drop",
      auto_arm_on_guided_ ? "automatic" : "manual");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");
    declare_parameter<bool>("lock_mission_yaw_to_initial_heading", true);
    declare_parameter<bool>("yaw_to_target", false);
    declare_parameter<double>("vision_heartbeat_timeout_s", 1.5);
    declare_parameter<double>("vision_max_pipeline_delay_s", 1.5);
    declare_parameter<double>("vision_future_tolerance_s", 0.05);
    declare_parameter<double>("vision_transform_tolerance_s", 0.10);
    declare_parameter<double>("nav_interpolation_max_gap_s", 0.20);
    declare_parameter<double>("odom_history_s", 5.0);
    declare_parameter<int>("vision_min_messages_before_takeoff", 3);

    declare_parameter<double>("takeoff_alt_m", 4.0);
    declare_parameter<double>("search_alt_m", 4.2);
    declare_parameter<double>("coarse_alt_m", 3.4);
    declare_parameter<double>("fine_alt_m", 1.2);
    declare_parameter<double>("transit_speed_m_s", 2.8);
    declare_parameter<double>("min_segment_time_s", 1.0);
    declare_parameter<double>("waypoint_accept_radius_m", 0.40);

    declare_parameter<double>("drop_area_near_edge_field_x_m", 30.0);
    declare_parameter<double>("drop_area_length_field_x_m", 5.0);
    declare_parameter<double>("drop_area_width_field_y_m", 8.0);
    declare_parameter<int>("search_lane_count", 3);
    declare_parameter<double>("search_edge_margin_m", 0.35);
    declare_parameter<double>("search_cross_margin_m", 0.55);
    declare_parameter<int>("max_search_passes", 3);

    declare_parameter<int>("bucket_required_count", 3);
    declare_parameter<int>("bucket_min_confirmations", 5);
    declare_parameter<double>("track_gate_m", 0.45);
    declare_parameter<double>("diameter_track_gate_m", 0.08);
    declare_parameter<double>("bucket_track_max_gap_s", 0.60);
    declare_parameter<double>("bucket_selection_max_age_s", 10.0);
    declare_parameter<double>("bucket_distinct_min_separation_m", 0.20);
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
    declare_parameter<double>("coarse_error_m", 0.45);
    declare_parameter<double>("fine_error_m", 0.08);
    declare_parameter<double>("coarse_stable_s", 0.6);
    declare_parameter<double>("fine_stable_s", 0.8);
    declare_parameter<double>("alignment_timeout_s", 12.0);
    declare_parameter<double>("detection_timeout_s", 3.0);
    declare_parameter<double>("recover_hold_s", 0.5);
    declare_parameter<double>("forced_release_height_tolerance_m", 0.12);
    declare_parameter<double>("forced_release_xy_tolerance_m", 0.25);
    declare_parameter<bool>("forced_release_enabled", true);
    declare_parameter<double>("forced_release_max_target_age_s", 1.0);
    declare_parameter<double>("forced_release_max_target_error_m", 0.20);
    declare_parameter<double>("forced_release_min_confidence", 0.35);
    declare_parameter<double>("forced_release_area_margin_m", 0.10);
    declare_parameter<double>("forced_release_stable_s", 0.80);
    declare_parameter<double>("forced_release_descent_timeout_s", 8.0);
    declare_parameter<double>("release_max_horizontal_speed_m_s", 0.15);
    declare_parameter<double>("release_max_vertical_speed_m_s", 0.10);
    declare_parameter<double>("release_max_tilt_deg", 8.0);
    declare_parameter<double>("release_max_angular_rate_deg_s", 10.0);

    // 保留飞手切入 GUIDED 作为启动确认；进入 WAIT_ARM 后是否由状态机解锁。
    declare_parameter<bool>("auto_arm_on_guided", true);
    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("odom_timeout_s", 0.75);
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

    declare_parameter<std::string>("release_mode", "servo");
    declare_parameter<int>("payload_count", 2);
    declare_parameter<std::vector<double>>(
      "payload_release_offsets_body_m",
      std::vector<double>{0.026, -0.065, -0.32, -0.026, 0.065, -0.32});
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
  }

  void load_parameters()
  {
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();
    lock_initial_heading_ =
      get_parameter("lock_mission_yaw_to_initial_heading").as_bool();
    yaw_to_target_ = get_parameter("yaw_to_target").as_bool();
    vision_heartbeat_timeout_s_ = std::max(
      0.3, get_parameter("vision_heartbeat_timeout_s").as_double());
    vision_max_pipeline_delay_s_ = std::max(
      0.05, get_parameter("vision_max_pipeline_delay_s").as_double());
    vision_future_tolerance_s_ = std::max(
      0.0, get_parameter("vision_future_tolerance_s").as_double());
    vision_transform_tolerance_s_ = std::max(
      0.01, get_parameter("vision_transform_tolerance_s").as_double());
    nav_interpolation_max_gap_s_ = std::max(
      0.02, get_parameter("nav_interpolation_max_gap_s").as_double());
    odom_history_s_ = std::max(
      1.0, get_parameter("odom_history_s").as_double());
    vision_min_messages_before_takeoff_ = std::max(
      1, static_cast<int>(
        get_parameter("vision_min_messages_before_takeoff").as_int()));
    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    search_alt_m_ = std::max(takeoff_alt_m_, get_parameter("search_alt_m").as_double());
    coarse_alt_m_ = std::max(1.0, get_parameter("coarse_alt_m").as_double());
    fine_alt_m_ = std::max(1.0, get_parameter("fine_alt_m").as_double());
    speed_m_s_ = std::max(0.3, get_parameter("transit_speed_m_s").as_double());
    min_segment_s_ =
      std::max(0.3, get_parameter("min_segment_time_s").as_double());
    accept_radius_m_ =
      std::max(0.1, get_parameter("waypoint_accept_radius_m").as_double());

    drop_near_x_m_ = get_parameter("drop_area_near_edge_field_x_m").as_double();
    drop_length_x_m_ =
      std::max(1.0, get_parameter("drop_area_length_field_x_m").as_double());
    drop_width_y_m_ =
      std::max(1.0, get_parameter("drop_area_width_field_y_m").as_double());
    search_lane_count_ =
      std::max(1, static_cast<int>(get_parameter("search_lane_count").as_int()));
    search_edge_margin_m_ =
      std::max(0.0, get_parameter("search_edge_margin_m").as_double());
    search_cross_margin_m_ =
      std::max(0.0, get_parameter("search_cross_margin_m").as_double());
    max_search_passes_ =
      std::max(1, static_cast<int>(get_parameter("max_search_passes").as_int()));

    required_bucket_count_ = std::max(
      3, static_cast<int>(get_parameter("bucket_required_count").as_int()));
    min_confirmations_ = std::max(
      1, static_cast<int>(get_parameter("bucket_min_confirmations").as_int()));
    track_gate_m_ = std::max(0.1, get_parameter("track_gate_m").as_double());
    diameter_track_gate_m_ =
      std::max(0.01, get_parameter("diameter_track_gate_m").as_double());
    track_max_gap_s_ =
      std::max(0.1, get_parameter("bucket_track_max_gap_s").as_double());
    selection_max_age_s_ =
      std::max(0.1, get_parameter("bucket_selection_max_age_s").as_double());
    distinct_min_separation_m_ = std::max(
      0.05, get_parameter("bucket_distinct_min_separation_m").as_double());
    target_ranking_stable_s_ = std::max(
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
    coarse_error_m_ = std::max(0.1, get_parameter("coarse_error_m").as_double());
    fine_error_m_ = std::max(0.03, get_parameter("fine_error_m").as_double());
    coarse_stable_s_ =
      std::max(0.1, get_parameter("coarse_stable_s").as_double());
    fine_stable_s_ =
      std::max(0.1, get_parameter("fine_stable_s").as_double());
    alignment_timeout_s_ =
      std::max(1.0, get_parameter("alignment_timeout_s").as_double());
    detection_timeout_s_ =
      std::max(0.2, get_parameter("detection_timeout_s").as_double());
    recover_hold_s_ = std::max(0.0, get_parameter("recover_hold_s").as_double());
    forced_release_height_tolerance_m_ = std::max(
      0.05, get_parameter("forced_release_height_tolerance_m").as_double());
    forced_release_xy_tolerance_m_ = std::max(
      0.05, get_parameter("forced_release_xy_tolerance_m").as_double());
    forced_release_enabled_ = get_parameter("forced_release_enabled").as_bool();
    forced_release_max_target_age_s_ = std::max(
      0.1, get_parameter("forced_release_max_target_age_s").as_double());
    forced_release_max_target_error_m_ = std::max(
      fine_error_m_, get_parameter("forced_release_max_target_error_m").as_double());
    forced_release_min_confidence_ = std::clamp(
      get_parameter("forced_release_min_confidence").as_double(), 0.0, 1.0);
    forced_release_area_margin_m_ = std::max(
      0.0, get_parameter("forced_release_area_margin_m").as_double());
    forced_release_stable_s_ = std::max(
      0.1, get_parameter("forced_release_stable_s").as_double());
    forced_release_descent_timeout_s_ = std::max(
      1.0, get_parameter("forced_release_descent_timeout_s").as_double());
    release_max_horizontal_speed_m_s_ = std::max(
      0.05, get_parameter("release_max_horizontal_speed_m_s").as_double());
    release_max_vertical_speed_m_s_ = std::max(
      0.05, get_parameter("release_max_vertical_speed_m_s").as_double());
    release_max_tilt_rad_ = std::max(
      1.0, get_parameter("release_max_tilt_deg").as_double()) * kPi / 180.0;
    release_max_angular_rate_rad_s_ = std::max(
      1.0, get_parameter("release_max_angular_rate_deg_s").as_double()) *
      kPi / 180.0;

    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();
    prestream_s_ = std::max(1.0, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ =
      std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ =
      std::max(30.0, get_parameter("mission_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    compass_timeout_s_ =
      std::max(0.2, get_parameter("compass_timeout_s").as_double());
    extended_state_timeout_s_ = std::max(
      0.2, get_parameter("extended_state_timeout_s").as_double());
    landing_confirm_stable_s_ = std::max(
      0.2, get_parameter("landing_confirm_stable_s").as_double());
    landing_max_relative_altitude_m_ = std::max(
      0.10, get_parameter("landing_max_relative_altitude_m").as_double());
    landing_max_horizontal_speed_m_s_ = std::max(
      0.05, get_parameter("landing_max_horizontal_speed_m_s").as_double());
    landing_max_vertical_speed_m_s_ = std::max(
      0.05, get_parameter("landing_max_vertical_speed_m_s").as_double());
    heading_stability_s_ =
      std::max(0.5, get_parameter("heading_lock_stability_s").as_double());
    heading_max_variation_rad_ =
      std::max(0.1, get_parameter("heading_lock_max_variation_deg").as_double()) *
      kPi / 180.0;
    position_stability_s_ =
      std::max(0.5, get_parameter("position_lock_stability_s").as_double());
    stationary_speed_m_s_ =
      std::max(0.02, get_parameter("stationary_speed_max_m_s").as_double());

    release_mode_ = get_parameter("release_mode").as_string();
    payload_count_ = std::max(
      1, static_cast<int>(get_parameter("payload_count").as_int()));
    release_offsets_ =
      get_parameter("payload_release_offsets_body_m").as_double_array();
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    release_duration_s_ =
      get_parameter("servo_release_duration_s").as_double_array();
    initialize_stowed_ = get_parameter("servo_initialize_stowed").as_bool();
    return_to_stowed_ = get_parameter("servo_return_to_stowed").as_bool();
    servo_ack_timeout_s_ =
      std::max(0.5, get_parameter("servo_ack_timeout_s").as_double());

    config_valid_ =
      lock_initial_heading_ && !yaw_to_target_ && release_mode_ == "servo" &&
      payload_count_ == 2 && initialize_stowed_ && return_to_stowed_ &&
      servo_channels_.size() >= static_cast<std::size_t>(payload_count_) &&
      stowed_pwm_.size() >= static_cast<std::size_t>(payload_count_) &&
      release_pwm_.size() >= static_cast<std::size_t>(payload_count_) &&
      release_duration_s_.size() >= static_cast<std::size_t>(payload_count_) &&
      release_offsets_.size() >= static_cast<std::size_t>(payload_count_ * 3) &&
      servo_channels_[0] == 7 && servo_channels_[1] == 8 &&
      stowed_pwm_[0] == 1100 && stowed_pwm_[1] == 1100 &&
      release_pwm_[0] == 1900 && release_pwm_[1] == 1900 &&
      std::abs(release_duration_s_[0] - 0.7) <= 1.0e-3 &&
      std::abs(release_duration_s_[1] - 0.7) <= 1.0e-3;
    if (!config_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid safety configuration: require fixed initial heading, "
        "servo mode, two payloads, CH7/CH8=1100->1900 for 0.7s, and safe stow");
    }
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    const bool was_armed = fcu_state_.armed;
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (guided_active_ && !was_guided) {
      guided_announced_ = true;
      if (auto_arm_on_guided_) {
        std::cout <<
          "GUIDED已切入；安全门禁通过后状态机将自动解锁，请立即保持遥控器不动" <<
          std::endl;
      } else {
        std::cout << "GUIDED,开始任务；确认场地安全后手动解锁" << std::endl;
      }
    }
    if (!guided_active_ && was_guided && is_automatic_flight_state(state_)) {
      publish_setpoint_ = false;
      if (state_ == State::RELEASE &&
        (release_command_pending_ || release_started_ || return_command_pending_))
      {
        release_abort_requested_ = true;
      }
      if (state_ == State::RELEASE && release_started_ && !return_sent_ &&
        !return_command_pending_)
      {
        return_command_pending_ =
          send_servo(payload_index_, false, ServoPurpose::RETURN_STOWED);
      }
      std::cout << "已离开GUIDED,飞手紧急接管,自动节点退出" << std::endl;
      enter(State::PILOT_OVERRIDE);
      return;
    }
    if (fcu_state_.armed && !was_armed && state_ == State::WAIT_ARM) {
      std::cout << "检测到飞控已解锁,自动定点起飞" << std::endl;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time receipt_time = now();
    const Point3 sample_position{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    double sample_roll = current_roll_;
    double sample_pitch = current_pitch_;
    const auto & q = msg->pose.pose.orientation;
    const double norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1.0e-6) {
      const double qx = q.x / norm;
      const double qy = q.y / norm;
      const double qz = q.z / norm;
      const double qw = q.w / norm;
      sample_roll = std::atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy));
      sample_pitch = std::asin(std::clamp(
        2.0 * (qw * qy - qz * qx), -1.0, 1.0));
    }

    position_ = sample_position;
    current_roll_ = sample_roll;
    current_pitch_ = sample_pitch;
    horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;
    angular_rate_rad_s_ = std::sqrt(
      msg->twist.twist.angular.x * msg->twist.twist.angular.x +
      msg->twist.twist.angular.y * msg->twist.twist.angular.y +
      msg->twist.twist.angular.z * msg->twist.twist.angular.z);
    have_odom_ = true;
    last_odom_time_ = receipt_time;

    // CUADC_LATEST_20260731: camera capture stamps and odom history use
    // the NUC ROS clock. This avoids FCU header-clock skew breaking visual
    // frame / odometry interpolation.
    const rclcpp::Time sample_stamp = receipt_time;

    bool append_history = true;
    if (!navigation_history_.empty()) {
      const double delta_s =
        (sample_stamp - navigation_history_.back().stamp).seconds();
      if (delta_s < -0.5) {
        RCLCPP_WARN(
          get_logger(), "ROS time moved backwards; clearing odom history");
        navigation_history_.clear();
      } else if (delta_s <= 0.0) {
        append_history = false;
      }
    }
    if (append_history) {
      navigation_history_.push_back(NavigationSample{
        sample_stamp, sample_position, sample_roll, sample_pitch});
      while (!navigation_history_.empty() &&
        (sample_stamp - navigation_history_.front().stamp).seconds() >
        odom_history_s_)
      {
        navigation_history_.pop_front();
      }
    }

    if (!frame_locked_) {
      if (horizontal_speed_m_s_ <= stationary_speed_m_s_) {
        if (!position_stable_since_.has_value()) {
          position_stable_since_ = receipt_time;
        }
      } else {
        position_stable_since_.reset();
      }
    }
  }

  void extended_state_callback(
    const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    extended_state_ = *msg;
    have_extended_state_ = true;
    last_extended_state_time_ = now();
    ++extended_state_sequence_;
    // ExtendedState is retained for diagnostics only. The field unit did not
    // publish a reliable ON_GROUND state, so landing confirmation below uses
    // local odometry altitude and velocity instead.
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    current_heading_enu_ =
      normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    last_compass_time_ = now();
    have_compass_ = true;
    heading_samples_.push_back(HeadingSample{last_compass_time_, current_heading_enu_});
    while (!heading_samples_.empty() &&
      (last_compass_time_ - heading_samples_.front().stamp).seconds() >
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
    NavigationSample interpolated;
    interpolated.stamp = stamp;
    interpolated.position = Point3{
      before.position.x + ratio * (after.position.x - before.position.x),
      before.position.y + ratio * (after.position.y - before.position.y),
      before.position.z + ratio * (after.position.z - before.position.z)};
    interpolated.roll = normalize_angle(
      before.roll + ratio * normalize_angle(after.roll - before.roll));
    interpolated.pitch = normalize_angle(
      before.pitch + ratio * normalize_angle(after.pitch - before.pitch));
    return interpolated;
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    const rclcpp::Time receipt_time = now();
    have_vision_message_ = true;
    last_vision_message_time_ = receipt_time;

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Vision frame has zero capture timestamp; rejecting frame");
      return;
    }
    const rclcpp::Time observation_time(
      msg->header.stamp, get_clock()->get_clock_type());
    const double pipeline_delay_s = (receipt_time - observation_time).seconds();
    if (!std::isfinite(pipeline_delay_s) ||
      pipeline_delay_s < -vision_future_tolerance_s_ ||
      pipeline_delay_s > vision_max_pipeline_delay_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Vision capture timestamp rejected: pipeline_delay=%.3fs allowed=[-%.3f,%.3f]",
        pipeline_delay_s, vision_future_tolerance_s_,
        vision_max_pipeline_delay_s_);
      return;
    }

    const auto nav = navigation_sample_at(observation_time);
    if (!nav.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No bracketing odom for vision capture stamp; rejecting frame");
      return;
    }

    // Commit the monotonic baseline only after the timestamp and odom
    // interpolation have both passed. Invalid future/stale frames cannot poison it.
    if (last_vision_capture_stamp_.has_value()) {
      const double capture_delta_s =
        (observation_time - *last_vision_capture_stamp_).seconds();
      if (capture_delta_s <= 0.0) {
        if (capture_delta_s < -0.5) {
          have_time_aligned_vision_ = false;
          vision_message_count_ = 0U;
          known_buckets_.clear();
          active_bucket_.reset();
          target_selection_locked_ = false;
          selected_target_ids_.clear();
          selected_target_positions_.clear();
          selected_target_diameters_.clear();
          ranking_candidate_ids_.clear();
          target_ranking_stable_since_.reset();
          last_vision_capture_stamp_ = observation_time;
          if (mission_started_) {
            begin_visual_failsafe("Vision capture clock moved backwards");
          }
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Out-of-order vision capture rejected: delta=%.3fs", capture_delta_s);
        return;
      }
    }
    last_vision_capture_stamp_ = observation_time;
    if (have_time_aligned_vision_) {
      const double valid_frame_gap_s =
        (receipt_time - last_time_aligned_vision_time_).seconds();
      if (valid_frame_gap_s < 0.0 ||
        valid_frame_gap_s > vision_heartbeat_timeout_s_)
      {
        vision_message_count_ = 0U;
      }
    }
    have_time_aligned_vision_ = true;
    last_time_aligned_vision_time_ = receipt_time;
    ++vision_message_count_;

    if (!frame_locked_ || !accepting_visual_targets() || msg->poses.empty()) {
      return;
    }

    std::vector<BucketTrack> detections;
    detections.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      const Point3 body{
        pose.position.x, pose.position.y, pose.position.z};
      const double diameter = std::max(0.0, pose.orientation.x);
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
      const Point3 local = body_to_local(body, *nav);
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
      detections.push_back(std::move(detection));
    }

    merge_frame_detections(detections);

    if (active_bucket_.has_value()) {
      const auto current = std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [this](const BucketTrack & track) {
          return track.id == active_bucket_->id;
        });
      if (current != known_buckets_.end()) {
        active_bucket_ = *current;
      }
    }
  }

  bool accepting_visual_targets() const
  {
    return state_ == State::SEARCH || state_ == State::ALIGN_COARSE ||
      state_ == State::ALIGN_FINE || state_ == State::RELEASE;
  }

  Point3 body_to_local(
    const Point3 & body, const NavigationSample & nav) const
  {
    // Position/roll/pitch are interpolated at camera capture time. Yaw remains
    // the mission-locked compass heading, never the odom quaternion yaw.
    const double cr = std::cos(nav.roll);
    const double sr = std::sin(nav.roll);
    const double cp = std::cos(nav.pitch);
    const double sp = std::sin(nav.pitch);
    const double cy = std::cos(mission_yaw_);
    const double sy = std::sin(mission_yaw_);

    const double x_roll = body.x;
    const double y_roll = cr * body.y - sr * body.z;
    const double z_roll = sr * body.y + cr * body.z;
    const double x_pitch = cp * x_roll + sp * z_roll;
    const double y_pitch = y_roll;
    const double z_pitch = -sp * x_roll + cp * z_roll;
    return Point3{
      nav.position.x + cy * x_pitch - sy * y_pitch,
      nav.position.y + sy * x_pitch + cy * y_pitch,
      nav.position.z + z_pitch};
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

  Point3 field_to_local(double field_x, double field_y, double relative_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{
      origin.x + c * field_x - s * field_y,
      origin.y + s * field_x + c * field_y,
      origin.z + relative_z};
  }

  Point3 local_to_field(const Point3 & local) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double dx = local.x - origin.x;
    const double dy = local.y - origin.y;
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{c * dx + s * dy, -s * dx + c * dy, local.z - origin.z};
  }

  bool inside_drop_area(const Point3 & local, double extra) const
  {
    const Point3 field = local_to_field(local);
    return field.x >= drop_near_x_m_ - extra &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ + extra &&
      std::abs(field.y) <= drop_width_y_m_ * 0.5 + extra;
  }

  bool bucket_already_used(const Point3 & local) const
  {
    return std::any_of(
      released_buckets_.begin(), released_buckets_.end(),
      [this, &local](const Point3 & used) {
        return distance_xy(used, local) < released_exclusion_m_;
      });
  }

  bool target_id_already_used(std::size_t id) const
  {
    return std::find(
      released_target_ids_.begin(), released_target_ids_.end(), id) !=
      released_target_ids_.end();
  }

  static double median(std::deque<double> values)
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

  static double median_absolute_deviation(
    const std::deque<double> & values, double center)
  {
    std::deque<double> deviations;
    for (const double value : values) {
      deviations.push_back(std::abs(value - center));
    }
    return median(std::move(deviations));
  }

  void smooth_track(BucketTrack & track, const BucketTrack & detection)
  {
    const double gap_s = (detection.stamp - track.stamp).seconds();
    if (gap_s > track_max_gap_s_) {
      track.local = detection.local;
      track.body = detection.body;
      track.diameter = detection.diameter;
      track.confidence = detection.confidence;
      track.position_deviation = 0.0;
      track.diameter_deviation = 0.0;
      track.confirmations = 1U;
      track.diameter_samples.clear();
      track.diameter_samples.push_back(detection.diameter);
      track.stamp = detection.stamp;
      return;
    }

    const double position_residual = distance_xy(track.local, detection.local);
    track.position_deviation =
      (1.0 - position_filter_alpha_) * track.position_deviation +
      position_filter_alpha_ * position_residual;
    track.local.x =
      (1.0 - position_filter_alpha_) * track.local.x +
      position_filter_alpha_ * detection.local.x;
    track.local.y =
      (1.0 - position_filter_alpha_) * track.local.y +
      position_filter_alpha_ * detection.local.y;
    track.local.z =
      (1.0 - position_filter_alpha_) * track.local.z +
      position_filter_alpha_ * detection.local.z;

    // Body coordinates change as the aircraft moves; default alpha=1 keeps the latest view.
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
    track.diameter_deviation = median_absolute_deviation(
      track.diameter_samples, robust_diameter);
    track.confidence =
      (1.0 - confidence_filter_alpha_) * track.confidence +
      confidence_filter_alpha_ * detection.confidence;
    ++track.confirmations;
    track.stamp = detection.stamp;
  }

  void merge_frame_detections(const std::vector<BucketTrack> & detections)
  {
    known_buckets_.erase(
      std::remove_if(
        known_buckets_.begin(), known_buckets_.end(),
        [this](const BucketTrack & track) {
          return (now() - track.stamp).seconds() > known_memory_s_ &&
                 !target_id_already_used(track.id) &&
                 std::find(
                   selected_target_ids_.begin(), selected_target_ids_.end(),
                   track.id) == selected_target_ids_.end();
        }),
      known_buckets_.end());

    struct Association
    {
      std::size_t track = 0U;
      std::size_t detection = 0U;
      double cost = 0.0;
    };
    std::vector<Association> associations;
    for (std::size_t ti = 0U; ti < known_buckets_.size(); ++ti) {
      for (std::size_t di = 0U; di < detections.size(); ++di) {
        const double position_delta =
          distance_xy(known_buckets_[ti].local, detections[di].local);
        const double diameter_delta = std::abs(
          known_buckets_[ti].diameter - detections[di].diameter);
        if (position_delta <= track_gate_m_ &&
          diameter_delta <= diameter_track_gate_m_)
        {
          associations.push_back(Association{
            ti, di,
            position_delta / track_gate_m_ +
            diameter_delta / diameter_track_gate_m_});
        }
      }
    }
    std::sort(
      associations.begin(), associations.end(),
      [](const Association & a, const Association & b) {
        return a.cost < b.cost;
      });

    std::vector<bool> track_used(known_buckets_.size(), false);
    std::vector<bool> detection_used(detections.size(), false);
    for (const auto & association : associations) {
      if (track_used[association.track] || detection_used[association.detection]) {
        continue;
      }
      smooth_track(
        known_buckets_[association.track], detections[association.detection]);
      track_used[association.track] = true;
      detection_used[association.detection] = true;
    }

    for (std::size_t di = 0U; di < detections.size(); ++di) {
      if (detection_used[di]) {
        continue;
      }
      BucketTrack track = detections[di];
      track.id = next_bucket_id_++;
      known_buckets_.push_back(std::move(track));
    }
  }

  bool track_ready_for_selection(const BucketTrack & track) const
  {
    return track.confirmations >= static_cast<std::size_t>(min_confirmations_) &&
      (now() - track.stamp).seconds() <= selection_max_age_s_ &&
      track.confidence >= min_track_confidence_ &&
      track.position_deviation <= max_position_deviation_m_ &&
      track.diameter_deviation <= max_diameter_deviation_m_ &&
      !target_id_already_used(track.id);
  }

  bool try_lock_target_plan()
  {
    if (target_selection_locked_) {
      return true;
    }

    std::vector<const BucketTrack *> reliable;
    for (const auto & track : known_buckets_) {
      if (track_ready_for_selection(track)) {
        reliable.push_back(&track);
      }
    }
    std::sort(
      reliable.begin(), reliable.end(),
      [](const BucketTrack * a, const BucketTrack * b) {
        if (a->confirmations != b->confirmations) {
          return a->confirmations > b->confirmations;
        }
        if (std::abs(a->confidence - b->confidence) > 1.0e-6) {
          return a->confidence > b->confidence;
        }
        return (a->position_deviation + a->diameter_deviation) <
               (b->position_deviation + b->diameter_deviation);
      });

    std::vector<const BucketTrack *> distinct;
    for (const BucketTrack * candidate : reliable) {
      const bool separated = std::all_of(
        distinct.begin(), distinct.end(),
        [this, candidate](const BucketTrack * accepted) {
          return distance_xy(candidate->local, accepted->local) >=
                 distinct_min_separation_m_;
        });
      if (separated) {
        distinct.push_back(candidate);
      }
      if (distinct.size() >= static_cast<std::size_t>(required_bucket_count_)) {
        break;
      }
    }

    if (distinct.size() < static_cast<std::size_t>(required_bucket_count_)) {
      ranking_candidate_ids_.clear();
      target_ranking_stable_since_.reset();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "Waiting for three stable distinct buckets: ready=%zu required=%d known=%zu",
        distinct.size(), required_bucket_count_, known_buckets_.size());
      return false;
    }

    std::sort(
      distinct.begin(), distinct.end(),
      [](const BucketTrack * a, const BucketTrack * b) {
        if (std::abs(a->diameter - b->diameter) > 1.0e-6) {
          return a->diameter < b->diameter;
        }
        return a->id < b->id;
      });
    std::vector<std::size_t> proposed_ids;
    for (const BucketTrack * track : distinct) {
      proposed_ids.push_back(track->id);
    }

    if (proposed_ids != ranking_candidate_ids_) {
      ranking_candidate_ids_ = proposed_ids;
      target_ranking_stable_since_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Three-bucket ranking candidate: #%zu=%.3fm #%zu=%.3fm #%zu=%.3fm; "
        "holding %.1fs before freeze",
        distinct[0]->id, distinct[0]->diameter,
        distinct[1]->id, distinct[1]->diameter,
        distinct[2]->id, distinct[2]->diameter,
        target_ranking_stable_s_);
      return false;
    }
    if (!target_ranking_stable_since_.has_value() ||
      (now() - *target_ranking_stable_since_).seconds() < target_ranking_stable_s_)
    {
      return false;
    }

    selected_target_ids_.assign(
      ranking_candidate_ids_.begin(),
      ranking_candidate_ids_.begin() + payload_count_);
    selected_target_positions_.clear();
    selected_target_diameters_.clear();
    for (int index = 0; index < payload_count_; ++index) {
      selected_target_positions_.push_back(distinct[index]->local);
      selected_target_diameters_.push_back(distinct[index]->diameter);
    }
    target_selection_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "HIGH_VALUE_TARGETS_LOCKED: three stable buckets; "
      "payload1=#%zu diameter=%.3fm payload2=#%zu diameter=%.3fm",
      selected_target_ids_[0], distinct[0]->diameter,
      selected_target_ids_[1], distinct[1]->diameter);
    return true;
  }

  std::optional<BucketTrack> best_bucket()
  {
    if (!try_lock_target_plan() ||
      payload_index_ >= selected_target_ids_.size())
    {
      return std::nullopt;
    }
    const std::size_t selected_id = selected_target_ids_[payload_index_];
    if (target_id_already_used(selected_id)) {
      return std::nullopt;
    }
    const auto selected = std::find_if(
      known_buckets_.begin(), known_buckets_.end(),
      [selected_id](const BucketTrack & track) {
        return track.id == selected_id;
      });
    if (selected == known_buckets_.end() ||
      (now() - selected->stamp).seconds() > known_memory_s_)
    {
      return std::nullopt;
    }
    return *selected;
  }

  bool odom_fresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  bool compass_fresh() const
  {
    return have_compass_ &&
      (now() - last_compass_time_).seconds() <= compass_timeout_s_;
  }

  bool extended_state_fresh() const
  {
    if (!have_extended_state_) {
      return false;
    }
    const double age_s = (now() - last_extended_state_time_).seconds();
    return age_s >= 0.0 && age_s <= extended_state_timeout_s_;
  }

  bool on_ground_reported() const
  {
    return extended_state_fresh() &&
      extended_state_.landed_state ==
      mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  bool vision_heartbeat_fresh() const
  {
    if (!have_vision_message_) {
      return false;
    }
    const double age_s = (now() - last_vision_message_time_).seconds();
    return age_s >= 0.0 && age_s <= vision_heartbeat_timeout_s_;
  }

  bool vision_time_alignment_fresh() const
  {
    if (!have_time_aligned_vision_) {
      return false;
    }
    const double age_s = (now() - last_time_aligned_vision_time_).seconds();
    return age_s >= 0.0 && age_s <= vision_heartbeat_timeout_s_;
  }

  bool vision_ready_for_takeoff() const
  {
    return vision_message_count_ >=
      static_cast<std::size_t>(vision_min_messages_before_takeoff_) &&
      vision_heartbeat_fresh() && vision_time_alignment_fresh();
  }

  bool state_requires_vision(State state) const
  {
    // TAKEOFF is executed by ArduPilot CommandTOL. Vision becomes mandatory
    // from SEARCH onward, where bucket coordinates are actually used.
    return state == State::SEARCH ||
      state == State::ALIGN_COARSE || state == State::ALIGN_FINE ||
      state == State::RELEASE || state == State::RECOVER;
  }

  bool landing_candidate() const
  {
    const double relative_altitude_m = relative_altitude();

    // Do not depend on MAVROS ExtendedState. Use conservative odometry-only
    // thresholds so a low hover is not mistaken for touchdown.
    return odom_fresh() &&
      std::isfinite(relative_altitude_m) &&
      std::isfinite(horizontal_speed_m_s_) &&
      std::isfinite(vertical_speed_m_s_) &&
      std::abs(relative_altitude_m) <=
        std::min(landing_max_relative_altitude_m_, 0.12) &&
      horizontal_speed_m_s_ <=
        std::min(landing_max_horizontal_speed_m_s_, 0.10) &&
      std::abs(vertical_speed_m_s_) <=
        std::min(landing_max_vertical_speed_m_s_, 0.08);
  }

  void reset_landing_confirmation()
  {
    landing_stable_since_.reset();
    landing_first_on_ground_message_time_.reset();
    landing_last_on_ground_message_time_.reset();
    landing_last_extended_state_sequence_ = 0U;
    landing_ground_message_count_ = 0U;
  }

  bool update_landing_confirmation()
  {
    if (!landing_candidate()) {
      reset_landing_confirmation();
      return false;
    }
    if (!landing_stable_since_.has_value()) {
      landing_stable_since_ = now();
      return false;
    }
    return (now() - *landing_stable_since_).seconds() >=
      std::max(2.0, landing_confirm_stable_s_);
  }

  bool landing_confirmation_ready() const
  {
    return landing_stable_since_.has_value() &&
      landing_candidate() &&
      (now() - *landing_stable_since_).seconds() >=
      std::max(2.0, landing_confirm_stable_s_);
  }

  void mark_mission_failure(const std::string & reason)
  {
    if (!mission_failed_) {
      mission_failed_ = true;
      terminal_reason_ = reason;
    }
    if (state_ == State::RELEASE &&
      (release_command_pending_ || release_started_ || return_command_pending_))
    {
      release_abort_requested_ = true;
    }
  }

  void begin_visual_failsafe(const std::string & reason)
  {
    mark_mission_failure(reason);
    if (state_ == State::RETURN_HOME || state_ == State::LAND ||
      state_ == State::DISARM || state_ == State::DONE ||
      state_ == State::ABORT || state_ == State::PILOT_OVERRIDE)
    {
      return;
    }
    RCLCPP_ERROR(
      get_logger(), "VISUAL_FAILSAFE: %s; release inhibited", reason.c_str());
    forced_release_ = false;
    forced_release_stable_since_.reset();
    if (!fcu_state_.armed) {
      enter(State::ABORT);
      return;
    }
    const bool can_return = state_ != State::TAKEOFF && home_.has_value() &&
      guided_active_ && odom_fresh() &&
      relative_altitude() > landing_max_relative_altitude_m_;
    if (can_return) {
      RCLCPP_WARN(get_logger(), "Visual fault: returning home before landing");
      start_return_home();
    } else {
      RCLCPP_WARN(get_logger(), "Visual fault: requesting direct controlled landing");
      publish_setpoint_ = false;
      enter(State::LAND);
    }
  }

  void monitor_visual_health()
  {
    if (!mission_started_ || !state_requires_vision(state_)) {
      return;
    }
    if (!vision_heartbeat_fresh()) {
      begin_visual_failsafe("Vision PoseArray heartbeat timed out");
    } else if (!vision_time_alignment_fresh()) {
      begin_visual_failsafe("Vision frames cannot be time-aligned with odometry");
    }
  }

  double mean_heading() const
  {
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & sample : heading_samples_) {
      sin_sum += std::sin(sample.yaw_enu);
      cos_sum += std::cos(sample.yaw_enu);
    }
    return std::atan2(sin_sum, cos_sum);
  }

  bool heading_stable() const
  {
    if (heading_samples_.size() < 5U ||
      (heading_samples_.back().stamp - heading_samples_.front().stamp).seconds() <
      heading_stability_s_)
    {
      return false;
    }
    const double mean = mean_heading();
    return std::all_of(
      heading_samples_.begin(), heading_samples_.end(),
      [this, mean](const HeadingSample & sample) {
        return std::abs(normalize_angle(sample.yaw_enu - mean)) <=
               heading_max_variation_rad_;
      });
  }

  bool navigation_ready_to_lock() const
  {
    // Pre-takeoff ground gate is intentionally bypassed on this field build.
    // The aircraft must still be disarmed, stationary, position-stable, and
    // have valid odometry, heading, vision timing, and servo initialization.
    return config_valid_ && fcu_state_.connected && !fcu_state_.armed &&
      odom_fresh() && compass_fresh() && heading_stable() &&
      vision_ready_for_takeoff() &&
      position_stable_since_.has_value() &&
      (now() - *position_stable_since_).seconds() >= position_stability_s_ &&
      horizontal_speed_m_s_ <= stationary_speed_m_s_;
  }

  void lock_frame()
  {
    home_ = position_;
    mission_yaw_ = mean_heading();
    locked_compass_deg_ =
      normalize_degrees(90.0 - mission_yaw_ * 180.0 / kPi);
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_search_route();
    RCLCPP_INFO(
      get_logger(),
      "Competition frame locked: compass=%.2f yaw_enu=%.2f home=(%.2f,%.2f,%.2f), "
      "drop_field_x=[%.2f,%.2f], y=[%.2f,%.2f]",
      locked_compass_deg_, mission_yaw_ * 180.0 / kPi,
      home_->x, home_->y, home_->z,
      drop_near_x_m_, drop_near_x_m_ + drop_length_x_m_,
      -drop_width_y_m_ * 0.5, drop_width_y_m_ * 0.5);
  }

  void build_search_route()
  {
    search_route_.clear();
    const double first_x = drop_near_x_m_ + search_edge_margin_m_;
    const double last_x =
      drop_near_x_m_ + drop_length_x_m_ - search_edge_margin_m_;
    const double half_y =
      std::max(0.2, drop_width_y_m_ * 0.5 - search_cross_margin_m_);
    for (int lane = 0; lane < search_lane_count_; ++lane) {
      const double fraction = search_lane_count_ == 1 ?
        0.5 : static_cast<double>(lane) /
        static_cast<double>(search_lane_count_ - 1);
      const double y = -half_y + 2.0 * half_y * fraction;
      const bool forward = lane % 2 == 0;
      search_route_.push_back(
        field_to_local(forward ? first_x : last_x, y, search_alt_m_));
      search_route_.push_back(
        field_to_local(forward ? last_x : first_x, y, search_alt_m_));
    }
  }

  void tick()
  {
    monitor_visual_health();
    check_service_results();
    check_servo_results();
    handle_release_abort_stow();
    initialize_servos_if_ready();

    if (publish_setpoint_ && frame_locked_) {
      publish_setpoint();
    }

    if (mission_started_ && state_ != State::RETURN_HOME &&
      state_ != State::LAND && state_ != State::DISARM &&
      state_ != State::DONE && state_ != State::PILOT_OVERRIDE &&
      state_ != State::ABORT && state_ != State::RELEASE &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      RCLCPP_ERROR(get_logger(), "Mission timeout; returning home");
      mark_mission_failure("Mission timeout");
      start_return_home();
    }

    if ((now() - last_status_time_).seconds() >= 4.0) {
      last_status_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s mode=%s armed=%s pos=(%.1f,%.1f,%.1f) payload=%zu/2 "
        "known=%zu target=%s forced=%s",
        state_name(state_).c_str(), fcu_state_.mode.c_str(),
        fcu_state_.armed ? "true" : "false",
        position_.x, position_.y, position_.z,
        payload_index_ + 1U, known_buckets_.size(),
        active_bucket_.has_value() ? "yes" : "no",
        forced_release_ ? "yes" : "no");
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_ = false;
        if (!config_valid_) {
          mark_mission_failure("Invalid safety configuration");
          enter(State::ABORT);
        } else if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          abort_or_land("Aircraft armed before navigation/vision safety lock");
        } else if (navigation_ready_to_lock()) {
          lock_frame();
          publish_setpoint_ = true;
          enter(State::PRESTREAM);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting lock: odom=%s compass=%s heading=%s stationary=%s "
            "ground=ignored vision=%s disarmed=%s",
            odom_fresh() ? "yes" : "no", compass_fresh() ? "yes" : "no",
            heading_stable() ? "yes" : "no",
            horizontal_speed_m_s_ <= stationary_speed_m_s_ ? "yes" : "no",
            vision_ready_for_takeoff() ? "yes" : "no",
            fcu_state_.armed ? "no" : "yes");
        }
        break;

      case State::PRESTREAM:
        target_ = *home_;
        if (!odom_fresh() || !compass_fresh()) {
          abort_or_land("Navigation stale before GUIDED");
        } else if (!vision_ready_for_takeoff()) {
          abort_or_land("Vision safety gate lost during prestream");
        } else if ((now() - state_enter_time_).seconds() >= prestream_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = *home_;
        if (!fcu_state_.connected) {
          mark_mission_failure("FCU disconnected before GUIDED");
          enter(State::ABORT);
        } else if (fcu_state_.armed && !vision_ready_for_takeoff())
        {
          abort_or_land("Aircraft armed while visual safety gate was not ready");
        } else if (guided_active_) {
          if (!vision_ready_for_takeoff()) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "GUIDED detected but takeoff inhibited: vision=%s; ground gate ignored",
              vision_ready_for_takeoff() ? "ready" : "not-ready");
          } else {
            if (!guided_announced_) {
              guided_announced_ = true;
              if (auto_arm_on_guided_) {
                std::cout <<
                  "GUIDED已切入；安全门禁通过后状态机将自动解锁，请立即保持遥控器不动" <<
                  std::endl;
              } else {
                std::cout << "GUIDED,开始任务；确认场地安全后手动解锁" << std::endl;
              }
            }
            enter(State::WAIT_ARM);
          }
        }
        break;

      case State::WAIT_ARM:
        target_ = *home_;
        if (!guided_active_) {
          guided_announced_ = false;
          enter(State::WAIT_GUIDED);
        } else if (!vision_ready_for_takeoff()) {
          if (fcu_state_.armed) {
            abort_or_land("Takeoff inhibited: visual gate lost before arm");
          } else {
            RCLCPP_WARN(
              get_logger(), "Safety gate lost while waiting for arm; returning to wait");
            enter(State::WAIT_GUIDED);
          }
        } else if (fcu_state_.armed) {
          if (initialize_stowed_ && !servos_initialized_) {
            abort_or_land("Armed before servo initialization ACK");
          } else {
            mission_started_ = true;
            mission_start_time_ = now();
            publish_setpoint_ = false;
            enter(State::TAKEOFF);
          }
        } else if (initialize_stowed_ && !servos_initialized_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "GUIDED active; waiting for CH7/CH8 stowed ACK before arm");
        } else if (auto_arm_on_guided_) {
          request_arm();
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "GUIDED active; automatic arm requested, waiting for FCU armed=true");
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "GUIDED active; waiting for pilot to arm");
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
          (now() - state_enter_time_).seconds() > takeoff_timeout_s_)
        {
          abort_or_land("Takeoff timeout");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;

      case State::ALIGN_COARSE:
        if (flight_gate_ok()) {
          update_alignment(false);
        }
        break;

      case State::ALIGN_FINE:
        if (flight_gate_ok()) {
          update_alignment(true);
        }
        break;

      case State::RELEASE:
        if (flight_gate_ok()) {
          update_release();
        }
        break;

      case State::RECOVER:
        if (flight_gate_ok()) {
          update_recover();
        }
        break;

      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          update_return();
        }
        break;

      case State::LAND:
        publish_setpoint_ = false;
        if (update_landing_confirmation()) {
          if (fcu_state_.armed) {
            enter(State::DISARM);
          } else {
            enter(State::DONE);
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Waiting odom-only landing confirmation: "
            "rel_alt=%.2f hspeed=%.2f vz=%.2f",
            relative_altitude(), horizontal_speed_m_s_, vertical_speed_m_s_);
          if (fcu_state_.armed) {
            request_land();
          }
        }
        break;

      case State::DISARM:
        publish_setpoint_ = false;
        if (!landing_confirmation_ready()) {
          RCLCPP_ERROR(
            get_logger(), "Landing confirmation lost; refusing disarm and returning to LAND");
          reset_landing_confirmation();
          enter(State::LAND);
        } else if (!fcu_state_.armed) {
          enter(State::DONE);
        } else {
          request_disarm();
        }
        break;

      case State::DONE:
        if (!success_logged_) {
          success_logged_ = true;
          const bool complete_success =
            !mission_failed_ && !fcu_state_.armed && landing_confirmation_ready() &&
            payload_index_ >= static_cast<std::size_t>(payload_count_);
          if (complete_success) {
            std::cout << "任务完成：两瓶视觉投放、返航、降落、上锁" << std::endl;
            RCLCPP_INFO(
              get_logger(),
              "VISUAL_DROP_MISSION_SUCCESS: two payloads processed, landed and disarmed");
          } else {
            const std::string reason = terminal_reason_.empty() ?
              "mission ended without all success conditions" : terminal_reason_;
            std::cout << "任务安全终止：" << reason << "；已确认落地并上锁" << std::endl;
            RCLCPP_ERROR(
              get_logger(), "VISUAL_DROP_MISSION_SAFE_ABORT: %s", reason.c_str());
          }
        }
        if ((now() - state_enter_time_).seconds() >= 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_ = false;
        if ((now() - state_enter_time_).seconds() >= 1.0 &&
          !release_abort_requested_ && !release_command_pending_ &&
          !return_command_pending_)
        {
          RCLCPP_WARN(
            get_logger(), "PILOT_OVERRIDE: exiting without flight-mode commands");
          rclcpp::shutdown();
        } else if ((now() - state_enter_time_).seconds() >=
          2.0 * servo_ack_timeout_s_ + 2.0)
        {
          RCLCPP_ERROR(
            get_logger(), "PILOT_OVERRIDE: servo safe-stow confirmation timed out");
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          const std::string reason = terminal_reason_.empty() ?
            "preflight abort" : terminal_reason_;
          RCLCPP_ERROR(
            get_logger(), "VISUAL_DROP_MISSION_ABORTED: %s", reason.c_str());
          rclcpp::shutdown();
        }
        break;
    }
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      abort_or_land("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    // mission_yaw_ is locked before GUIDED, so a live compass stream is not
    // required after launch. During the MAVLink traffic spike immediately
    // after arming / CommandTOL, allow odometry up to 5 s to recover.
    const bool takeoff_odom_grace =
      state_ == State::TAKEOFF &&
      (now() - state_enter_time_).seconds() <= 5.0;

    if (!odom_fresh()) {
      if (takeoff_odom_grace) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TAKEOFF: odom temporarily stale; waiting within 5s grace period");
      } else {
        abort_or_land("Odometry stale during mission");
        return false;
      }
    }
    if (state_requires_vision(state_)) {
      if (!vision_heartbeat_fresh()) {
        begin_visual_failsafe("Vision PoseArray heartbeat timed out");
        return false;
      }
      if (!vision_time_alignment_fresh()) {
        begin_visual_failsafe("Vision frames cannot be time-aligned with odometry");
        return false;
      }
    }
    return true;
  }

  void start_search()
  {
    active_bucket_.reset();
    forced_release_ = false;
    release_abort_requested_ = false;
    return_sent_ = false;
    forced_release_stable_since_.reset();
    alignment_stable_since_.reset();
    alignment_start_time_.reset();
    search_index_ = 0U;
    ++search_pass_;
    if (search_route_.empty()) {
      RCLCPP_ERROR(get_logger(), "Search route empty; returning home");
      mark_mission_failure("Search route empty");
      start_return_home();
      return;
    }
    start_segment(position_, search_route_.front());
    enter(State::SEARCH);
    RCLCPP_INFO(
      get_logger(), "Start drop-area search pass %d/%d for payload %zu",
      search_pass_, max_search_passes_, payload_index_ + 1U);
  }

  void update_search()
  {
    const auto candidate = best_bucket();
    if (candidate.has_value()) {
      active_bucket_ = candidate;
      alignment_start_time_ = now();
      alignment_stable_since_.reset();
      forced_release_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Frozen target locked for payload %zu: id=%zu diameter=%.3f "
        "local=(%.2f,%.2f), confirms=%zu",
        payload_index_ + 1U, candidate->id, candidate->diameter,
        candidate->local.x, candidate->local.y, candidate->confirmations);
      enter(State::ALIGN_COARSE);
      return;
    }

    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      return;
    }
    ++search_index_;
    if (search_index_ < search_route_.size()) {
      start_segment(position_, search_route_[search_index_]);
      return;
    }
    if (search_pass_ < max_search_passes_) {
      start_search();
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "No confirmed bucket after %d passes; preserving remaining payload and returning",
        max_search_passes_);
      mark_mission_failure("No confirmed bucket after maximum search passes");
      start_return_home();
    }
  }

  Point3 desired_release_pose(double relative_altitude_m) const
  {
    if (!active_bucket_.has_value()) {
      return Point3{position_.x, position_.y, home_->z + relative_altitude_m};
    }
    const Point3 release_body =
      vector3_at(release_offsets_, payload_index_, Point3{});
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{
      active_bucket_->local.x - (c * release_body.x - s * release_body.y),
      active_bucket_->local.y - (s * release_body.x + c * release_body.y),
      home_->z + relative_altitude_m};
  }

  bool active_detection_recent() const
  {
    return active_bucket_.has_value() &&
      (now() - active_bucket_->stamp).seconds() <= detection_timeout_s_;
  }

  double release_error() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const Point3 release_body =
      vector3_at(release_offsets_, payload_index_, Point3{});
    const Point3 target_body = local_to_body_current(active_bucket_->local);
    const double body_error = std::hypot(
      target_body.x - release_body.x,
      target_body.y - release_body.y);
    const Point3 desired = desired_release_pose(relative_altitude());
    const double local_error = distance_xy(position_, desired);
    return std::max(body_error, local_error);
  }

  bool active_target_identity_valid() const
  {
    if (!active_bucket_.has_value() || !target_selection_locked_ ||
      payload_index_ >= selected_target_ids_.size() ||
      payload_index_ >= selected_target_positions_.size() ||
      payload_index_ >= selected_target_diameters_.size() ||
      active_bucket_->id != selected_target_ids_[payload_index_] ||
      target_id_already_used(active_bucket_->id))
    {
      return false;
    }
    return distance_xy(
      active_bucket_->local, selected_target_positions_[payload_index_]) <=
      max_position_deviation_m_ &&
      std::abs(
      active_bucket_->diameter - selected_target_diameters_[payload_index_]) <=
      max_diameter_deviation_m_;
  }

  bool inside_release_area(const Point3 & local) const
  {
    const Point3 field = local_to_field(local);
    const double half_width = drop_width_y_m_ * 0.5;
    return field.x >= drop_near_x_m_ + forced_release_area_margin_m_ &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ -
      forced_release_area_margin_m_ &&
      std::abs(field.y) <= half_width - forced_release_area_margin_m_;
  }

  std::optional<std::string> forced_release_target_violation() const
  {
    if (!forced_release_enabled_) {
      return std::string("forced release disabled");
    }
    if (!vision_heartbeat_fresh() || !vision_time_alignment_fresh()) {
      return std::string("visual heartbeat/time alignment unhealthy");
    }
    if (!active_target_identity_valid()) {
      return std::string("target ID no longer matches frozen payload plan");
    }
    const double target_age_s = (now() - active_bucket_->stamp).seconds();
    if (target_age_s < 0.0 || target_age_s > forced_release_max_target_age_s_) {
      return std::string("target observation is stale");
    }
    if (active_bucket_->confidence < forced_release_min_confidence_ ||
      active_bucket_->position_deviation > max_position_deviation_m_ ||
      active_bucket_->diameter_deviation > max_diameter_deviation_m_)
    {
      return std::string("target track quality below forced-release threshold");
    }
    if (!inside_release_area(active_bucket_->local) ||
      !inside_release_area(desired_release_pose(fine_alt_m_)))
    {
      return std::string("target/release point outside inset drop area");
    }
    if (release_error() > forced_release_max_target_error_m_) {
      return std::string("target error exceeds forced-release limit");
    }
    return std::nullopt;
  }

  std::optional<std::string> release_gate_violation(bool forced) const
  {
    if (!vision_heartbeat_fresh() || !vision_time_alignment_fresh()) {
      return std::string("visual health gate not ready");
    }
    if (!active_target_identity_valid()) {
      return std::string("frozen target identity invalid");
    }
    const double max_age_s = forced ?
      forced_release_max_target_age_s_ : detection_timeout_s_;
    const double target_age_s = (now() - active_bucket_->stamp).seconds();
    if (target_age_s < 0.0 || target_age_s > max_age_s) {
      return std::string("target age gate not ready");
    }
    const double min_confidence = forced ?
      forced_release_min_confidence_ : min_track_confidence_;
    if (active_bucket_->confidence < min_confidence ||
      active_bucket_->position_deviation > max_position_deviation_m_ ||
      active_bucket_->diameter_deviation > max_diameter_deviation_m_)
    {
      return std::string("target quality gate not ready");
    }
    if (!inside_release_area(active_bucket_->local)) {
      return std::string("target outside inset drop area");
    }
    const Point3 desired = forced ?
      forced_release_pose_ : desired_release_pose(fine_alt_m_);
    if (!inside_release_area(desired)) {
      return std::string("release point outside inset drop area");
    }
    const double max_error = forced ?
      forced_release_max_target_error_m_ : fine_error_m_;
    const double xy_tolerance = forced ?
      forced_release_xy_tolerance_m_ : fine_error_m_;
    if (release_error() > max_error) {
      return std::string("target alignment error too large");
    }
    if (distance_xy(position_, desired) > xy_tolerance) {
      return std::string("aircraft XY has not reached release pose");
    }
    if (std::abs(position_.z - desired.z) >
      forced_release_height_tolerance_m_)
    {
      return std::string("release height gate not ready");
    }
    if (horizontal_speed_m_s_ > release_max_horizontal_speed_m_s_ ||
      std::abs(vertical_speed_m_s_) > release_max_vertical_speed_m_s_)
    {
      return std::string("aircraft velocity gate not ready");
    }
    if (std::abs(current_roll_) > release_max_tilt_rad_ ||
      std::abs(current_pitch_) > release_max_tilt_rad_ ||
      angular_rate_rad_s_ > release_max_angular_rate_rad_s_)
    {
      return std::string("aircraft attitude/rate gate not ready");
    }
    return std::nullopt;
  }

  void reject_forced_release(const std::string & reason)
  {
    const std::string full_reason = "Forced release rejected: " + reason;
    mark_mission_failure(full_reason);
    RCLCPP_ERROR(get_logger(), "%s; preserving payload", full_reason.c_str());
    forced_release_ = false;
    forced_release_stable_since_.reset();
    start_return_home();
  }

  void update_alignment(bool fine)
  {
    if (!active_bucket_.has_value() || !alignment_start_time_.has_value()) {
      start_search();
      return;
    }
    const double elapsed = (now() - *alignment_start_time_).seconds();
    if (elapsed >= alignment_timeout_s_) {
      const auto violation = forced_release_target_violation();
      if (violation.has_value()) {
        reject_forced_release(*violation);
        return;
      }
      forced_release_ = true;
      forced_release_pose_ = desired_release_pose(fine_alt_m_);
      target_ = forced_release_pose_;
      alignment_stable_since_.reset();
      RCLCPP_WARN(
        get_logger(),
        "ALIGNMENT_TIMEOUT: payload=%zu target_id=%zu elapsed=%.1f error=%.3f; "
        "descend over compensated target pose to %.2fm, then re-check all gates",
        payload_index_ + 1U, active_bucket_->id, elapsed,
        release_error(), fine_alt_m_);
      enter(State::RELEASE);
      return;
    }

    const double altitude = fine ? fine_alt_m_ : coarse_alt_m_;
    const double threshold = fine ? fine_error_m_ : coarse_error_m_;
    const double stable_s = fine ? fine_stable_s_ : coarse_stable_s_;
    target_ = desired_release_pose(altitude);
    const double position_error = distance_xy(position_, target_);
    const double error = release_error();
    const bool aligned =
      position_error <= threshold &&
      error <= threshold &&
      std::abs(position_.z - target_.z) <= std::max(0.15, threshold) &&
      (!fine || active_detection_recent());
    if (!aligned) {
      alignment_stable_since_.reset();
      return;
    }
    if (!alignment_stable_since_.has_value()) {
      alignment_stable_since_ = now();
      return;
    }
    if ((now() - *alignment_stable_since_).seconds() < stable_s) {
      return;
    }
    alignment_stable_since_.reset();
    if (fine) {
      forced_release_ = false;
      RCLCPP_INFO(
        get_logger(), "Fine visual alignment stable: error=%.3f", error);
      enter(State::RELEASE);
    } else {
      RCLCPP_INFO(
        get_logger(), "Coarse visual alignment stable: error=%.3f", error);
      enter(State::ALIGN_FINE);
    }
  }

  void update_release()
  {
    if (!active_bucket_.has_value()) {
      abort_or_land("Release entered without a target");
      return;
    }
    target_ = forced_release_ ?
      forced_release_pose_ : desired_release_pose(fine_alt_m_);

    if (!release_started_ && !release_command_pending_) {
      const auto violation = release_gate_violation(forced_release_);
      if (violation.has_value()) {
        forced_release_stable_since_.reset();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Release inhibited (%s): forced=%s error=%.3f xy=%.3f z=%.3f "
          "hspeed=%.3f vz=%.3f",
          violation->c_str(), forced_release_ ? "yes" : "no",
          release_error(), distance_xy(position_, target_),
          std::abs(position_.z - target_.z), horizontal_speed_m_s_,
          vertical_speed_m_s_);
        if ((now() - state_enter_time_).seconds() >=
          forced_release_descent_timeout_s_)
        {
          if (forced_release_) {
            reject_forced_release(
              "release gates did not remain valid before descent timeout");
          } else {
            mark_mission_failure("Normal release safety gates timed out");
            RCLCPP_ERROR(
              get_logger(), "Normal release gates timed out; preserving payload");
            start_return_home();
          }
        } else if (!forced_release_ &&
          (!active_detection_recent() || release_error() > fine_error_m_))
        {
          alignment_stable_since_.reset();
          enter(State::ALIGN_FINE);
        }
        return;
      }

      if (!forced_release_stable_since_.has_value()) {
        forced_release_stable_since_ = now();
        RCLCPP_INFO(
          get_logger(),
          "All release gates valid for payload %zu; holding %.2fs",
          payload_index_ + 1U, forced_release_stable_s_);
        return;
      }
      if ((now() - *forced_release_stable_since_).seconds() <
        forced_release_stable_s_)
      {
        return;
      }

      target_ = position_;
      release_command_pending_ =
        send_servo(payload_index_, true, ServoPurpose::RELEASE);
      if (!release_command_pending_) {
        abort_or_land("Unable to send release command");
      }
      return;
    }

    const double duration =
      value_at<double>(release_duration_s_, payload_index_, 0.7);
    if (release_started_ && return_to_stowed_ && !return_sent_ &&
      !return_command_pending_ &&
      (now() - release_start_time_).seconds() >= duration)
    {
      return_command_pending_ =
        send_servo(payload_index_, false, ServoPurpose::RETURN_STOWED);
      if (!return_command_pending_) {
        abort_or_land("Unable to send stowed-return command after release");
      }
      return;
    }

    if (release_started_ && (!return_to_stowed_ || return_sent_)) {
      finish_payload_release();
    }
  }

  void finish_payload_release()
  {
    RCLCPP_INFO(
      get_logger(), "Payload %zu complete on target id=%zu diameter=%.3f%s",
      payload_index_ + 1U, active_bucket_->id, active_bucket_->diameter,
      forced_release_ ? " [TIMEOUT_FORCED_AT_1P2M]" : "");
    released_buckets_.push_back(active_bucket_->local);
    if (!target_id_already_used(active_bucket_->id)) {
      released_target_ids_.push_back(active_bucket_->id);
    }
    ++payload_index_;
    active_bucket_.reset();
    release_started_ = false;
    release_command_pending_ = false;
    return_sent_ = false;
    return_command_pending_ = false;
    forced_release_ = false;
    release_abort_requested_ = false;
    forced_release_stable_since_.reset();
    alignment_start_time_.reset();
    alignment_stable_since_.reset();

    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      start_return_home();
      return;
    }
    recover_stable_since_.reset();
    start_segment(
      position_,
      Point3{position_.x, position_.y, home_->z + search_alt_m_});
    enter(State::RECOVER);
  }

  void update_recover()
  {
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      recover_stable_since_.reset();
      return;
    }
    if (!recover_stable_since_.has_value()) {
      recover_stable_since_ = now();
      return;
    }
    if ((now() - *recover_stable_since_).seconds() < recover_hold_s_) {
      return;
    }
    const auto candidate = best_bucket();
    if (candidate.has_value()) {
      active_bucket_ = candidate;
      alignment_start_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Reacquired frozen second-smallest target for payload %zu: id=%zu diameter=%.3f",
        payload_index_ + 1U, candidate->id, candidate->diameter);
      enter(State::ALIGN_COARSE);
    } else {
      search_pass_ = 0;
      start_search();
    }
  }

  void start_return_home()
  {
    if (!home_.has_value()) {
      abort_or_land("Cannot return without home");
      return;
    }
    start_segment(
      position_,
      Point3{home_->x, home_->y, home_->z + search_alt_m_});
    enter(State::RETURN_HOME);
  }

  void update_return()
  {
    target_ = sample_segment();
    if (segment_complete(std::max(accept_radius_m_, 0.5))) {
      publish_setpoint_ = false;
      enter(State::LAND);
    }
  }

  void start_segment(const Point3 & start, const Point3 & end)
  {
    segment_ = Segment{
      start, end, now(),
      std::max(min_segment_s_, distance_xyz(start, end) / speed_m_s_)};
  }

  Point3 sample_segment() const
  {
    const double ratio = std::clamp(
      (now() - segment_.start_time).seconds() / segment_.duration_s, 0.0, 1.0);
    const double smooth = ratio * ratio * (3.0 - 2.0 * ratio);
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

  void initialize_servos_if_ready()
  {
    if (!initialize_stowed_ || servos_initialized_ || initialization_started_ ||
      !fcu_state_.connected || fcu_state_.armed ||
      !command_client_->service_is_ready() ||
      (now() - last_servo_attempt_time_).seconds() < 1.0)
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
    }
  }

  bool send_servo(
    std::size_t payload, bool release, ServoPurpose purpose)
  {
    if (!command_client_->service_is_ready() ||
      payload >= servo_channels_.size() ||
      payload >= stowed_pwm_.size() ||
      payload >= release_pwm_.size())
    {
      return false;
    }
    const int64_t channel = servo_channels_[payload];
    const int64_t pwm = release ? release_pwm_[payload] : stowed_pwm_[payload];
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
    pending.sent = now();
    pending.future = command_client_->async_send_request(request).future.share();
    pending_servo_commands_.push_back(std::move(pending));
    last_servo_attempt_time_ = now();
    RCLCPP_INFO(
      get_logger(), "MAV_CMD_DO_SET_SERVO payload=%zu channel=%ld pwm=%ld",
      payload + 1U, static_cast<long>(channel), static_cast<long>(pwm));
    return true;
  }

  void check_servo_results()
  {
    for (auto it = pending_servo_commands_.begin();
      it != pending_servo_commands_.end();)
    {
      const bool ready =
        it->future.wait_for(0s) == std::future_status::ready;
      const bool timed_out =
        (now() - it->sent).seconds() > servo_ack_timeout_s_;
      if (!ready && !timed_out) {
        ++it;
        continue;
      }
      const bool accepted = ready && it->future.get()->success;
      RCLCPP_INFO(
        get_logger(), "DO_SET_SERVO ACK payload=%zu purpose=%d accepted=%s",
        it->payload + 1U, static_cast<int>(it->purpose),
        accepted ? "true" : "false");

      if (it->purpose == ServoPurpose::INITIALIZE) {
        if (initialization_pending_count_ > 0U) {
          --initialization_pending_count_;
        }
        initialization_failed_ = initialization_failed_ || !accepted;
      } else if (it->purpose == ServoPurpose::RELEASE) {
        release_command_pending_ = false;
        if (accepted && !release_abort_requested_ && state_ == State::RELEASE) {
          release_started_ = true;
          release_start_time_ = now();
          std::cout << "飞控确认投瓶" << it->payload + 1U << "：通道" <<
            servo_channels_[it->payload] << "=" <<
            release_pwm_[it->payload] << "us" << std::endl;
        } else if (accepted) {
          release_started_ = false;
          release_abort_requested_ = true;
          RCLCPP_ERROR(
            get_logger(),
            "Late release ACK after failsafe/override; physical release uncertain, "
            "not counting payload and commanding safe stow");
        } else if (release_abort_requested_) {
          release_started_ = false;
          RCLCPP_WARN(
            get_logger(), "Aborted release command rejected/timed out; commanding stow");
        } else {
          abort_or_land("Release command rejected or ACK timeout");
        }
      } else {
        return_command_pending_ = false;
        if (accepted) {
          return_sent_ = true;
          if (release_abort_requested_) {
            release_started_ = false;
          }
          std::cout << "投放舵机收回确认：通道" <<
            servo_channels_[it->payload] << "=" <<
            stowed_pwm_[it->payload] << "us" << std::endl;
        } else if (release_abort_requested_) {
          RCLCPP_ERROR(
            get_logger(), "Failsafe stow rejected/ACK timeout; will retry while node stays active");
        } else {
          abort_or_land("Stowed-return command rejected or ACK timeout");
        }
      }
      it = pending_servo_commands_.erase(it);
    }

    if (initialization_started_ && initialization_pending_count_ == 0U) {
      initialization_started_ = false;
      if (initialization_failed_) {
        RCLCPP_ERROR(get_logger(), "Servo initialization failed; retrying");
      } else {
        servos_initialized_ = true;
        std::cout << "投放舵机就绪：通道7=1100us,通道8=1100us" << std::endl;
      }
    }
  }

  void handle_release_abort_stow()
  {
    if (!release_abort_requested_) {
      return;
    }
    if (payload_index_ >= static_cast<std::size_t>(payload_count_)) {
      release_abort_requested_ = false;
      release_started_ = false;
      return;
    }
    if (release_command_pending_ || return_command_pending_) {
      return;
    }
    if (return_sent_) {
      release_abort_requested_ = false;
      release_started_ = false;
      RCLCPP_WARN(get_logger(), "Failsafe servo stow confirmed");
      return;
    }
    if ((now() - last_servo_attempt_time_).seconds() < 1.0) {
      return;
    }
    return_command_pending_ =
      send_servo(payload_index_, false, ServoPurpose::RETURN_STOWED);
    if (return_command_pending_) {
      RCLCPP_WARN(
        get_logger(), "Failsafe: commanding payload channel back to stowed PWM");
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
    return (now() - last_request_time_).seconds() >= 1.0;
  }

  void mark_request()
  {
    last_request_time_ = now();
  }

  void request_takeoff()
  {
    if (!takeoff_client_->service_is_ready() || takeoff_future_.valid() ||
      !request_allowed())
    {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(takeoff_alt_m_);
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    takeoff_future_ = takeoff_client_->async_send_request(request).future.share();
    takeoff_sent_ = true;
    RCLCPP_INFO(
      get_logger(), "CommandTOL takeoff altitude=%.1f explicit_yaw=%.2f",
      takeoff_alt_m_, locked_compass_deg_);
  }

  void request_land()
  {
    if (!fcu_state_.armed || !land_client_->service_is_ready() ||
      land_future_.valid() || !request_allowed())
    {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    land_future_ = land_client_->async_send_request(request).future.share();
    RCLCPP_INFO(get_logger(), "Requested landing");
  }

  void request_arm()
  {
    if (!auto_arm_on_guided_ || state_ != State::WAIT_ARM || !guided_active_ ||
      fcu_state_.armed || !vision_ready_for_takeoff() ||
      (initialize_stowed_ && !servos_initialized_) ||
      !arm_client_->service_is_ready() || arm_future_.valid() || !request_allowed())
    {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    mark_request();
    pending_arm_value_ = true;
    arm_future_ = arm_client_->async_send_request(request).future.share();
    RCLCPP_INFO(
      get_logger(),
      "Requested automatic arm after GUIDED and all pre-takeoff safety gates");
  }

  void request_disarm()
  {
    if (!landing_confirmation_ready() || !fcu_state_.armed ||
      !arm_client_->service_is_ready() ||
      arm_future_.valid() || !request_allowed())
    {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;
    mark_request();
    pending_arm_value_ = false;
    arm_future_ = arm_client_->async_send_request(request).future.share();
    RCLCPP_INFO(get_logger(), "Requested explicit disarm after landing");
  }

  void check_service_results()
  {
    if (takeoff_future_.valid() &&
      takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = takeoff_future_.get();
      RCLCPP_INFO(
        get_logger(), "Takeoff ACK success=%s result=%u",
        response->success ? "true" : "false", response->result);
      if (!response->success) {
        takeoff_sent_ = false;
      }
      takeoff_future_ = {};
    }
    if (land_future_.valid() &&
      land_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = land_future_.get();
      RCLCPP_INFO(
        get_logger(), "Land ACK success=%s result=%u",
        response->success ? "true" : "false", response->result);
      land_future_ = {};
    }
    if (arm_future_.valid() &&
      arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = arm_future_.get();
      RCLCPP_INFO(
        get_logger(), "%s ACK success=%s result=%u",
        pending_arm_value_ ? "Arm" : "Disarm",
        response->success ? "true" : "false", response->result);
      if (pending_arm_value_ && !response->success && state_ == State::WAIT_ARM) {
        RCLCPP_WARN(get_logger(), "Automatic arm rejected; will retry while gates remain valid");
      }
      arm_future_ = {};
    }
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  bool is_automatic_flight_state(State state) const
  {
    return state == State::WAIT_ARM || state == State::TAKEOFF ||
      state == State::SEARCH || state == State::ALIGN_COARSE ||
      state == State::ALIGN_FINE || state == State::RELEASE ||
      state == State::RECOVER || state == State::RETURN_HOME;
  }

  void abort_or_land(const std::string & reason)
  {
    mark_mission_failure(reason);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s",
      state_name(state_).c_str(), state_name(next).c_str());
    if (next == State::LAND && state_ != State::DISARM) {
      reset_landing_confirmation();
    }
    if (next == State::RELEASE) {
      forced_release_stable_since_.reset();
    }
    state_ = next;
    state_enter_time_ = now();
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
      case State::RECOVER: return "RECOVER";
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
  bool auto_arm_on_guided_ = false;
  bool pending_arm_value_ = false;
  bool guided_active_ = false;
  bool guided_announced_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool have_extended_state_ = false;
  bool have_vision_message_ = false;
  bool have_time_aligned_vision_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_ = false;
  bool mission_started_ = false;
  bool takeoff_sent_ = false;
  bool forced_release_ = false;
  bool release_command_pending_ = false;
  bool release_started_ = false;
  bool return_command_pending_ = false;
  bool return_sent_ = false;
  bool initialization_started_ = false;
  bool initialization_failed_ = false;
  bool servos_initialized_ = false;
  bool initialize_stowed_ = true;
  bool return_to_stowed_ = true;
  bool success_logged_ = false;
  bool target_selection_locked_ = false;
  bool mission_failed_ = false;
  bool release_abort_requested_ = false;
  bool lock_initial_heading_ = true;
  bool yaw_to_target_ = false;
  bool forced_release_enabled_ = true;

  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  Point3 position_;
  Point3 target_;
  Point3 forced_release_pose_;
  std::optional<Point3> home_;
  Segment segment_;
  std::optional<rclcpp::Time> position_stable_since_;
  std::optional<rclcpp::Time> alignment_start_time_;
  std::optional<rclcpp::Time> alignment_stable_since_;
  std::optional<rclcpp::Time> recover_stable_since_;
  std::optional<rclcpp::Time> target_ranking_stable_since_;
  std::optional<rclcpp::Time> forced_release_stable_since_;
  std::optional<rclcpp::Time> landing_stable_since_;
  std::optional<rclcpp::Time> landing_first_on_ground_message_time_;
  std::optional<rclcpp::Time> landing_last_on_ground_message_time_;
  std::optional<rclcpp::Time> last_vision_capture_stamp_;
  std::optional<BucketTrack> active_bucket_;
  std::deque<HeadingSample> heading_samples_;
  std::deque<NavigationSample> navigation_history_;
  std::vector<BucketTrack> known_buckets_;
  std::vector<Point3> released_buckets_;
  std::vector<std::size_t> released_target_ids_;
  std::vector<std::size_t> selected_target_ids_;
  std::vector<Point3> selected_target_positions_;
  std::vector<double> selected_target_diameters_;
  std::vector<std::size_t> ranking_candidate_ids_;
  std::vector<Point3> search_route_;
  std::vector<PendingServoCommand> pending_servo_commands_;

  std::string bucket_topic_;
  std::string release_mode_ = "servo";
  std::string terminal_reason_;
  double takeoff_alt_m_ = 4.0;
  double search_alt_m_ = 4.2;
  double coarse_alt_m_ = 3.4;
  double fine_alt_m_ = 1.2;
  double speed_m_s_ = 2.8;
  double min_segment_s_ = 1.0;
  double accept_radius_m_ = 0.4;
  double drop_near_x_m_ = 30.0;
  double drop_length_x_m_ = 5.0;
  double drop_width_y_m_ = 8.0;
  double search_edge_margin_m_ = 0.35;
  double search_cross_margin_m_ = 0.55;
  double track_gate_m_ = 0.45;
  double diameter_track_gate_m_ = 0.08;
  double track_max_gap_s_ = 0.60;
  double selection_max_age_s_ = 10.0;
  double distinct_min_separation_m_ = 0.20;
  double target_ranking_stable_s_ = 0.80;
  double known_memory_s_ = 120.0;
  double released_exclusion_m_ = 0.25;
  double position_filter_alpha_ = 0.25;
  double body_filter_alpha_ = 1.0;
  double diameter_filter_alpha_ = 0.20;
  double confidence_filter_alpha_ = 0.30;
  double max_position_deviation_m_ = 0.25;
  double max_diameter_deviation_m_ = 0.035;
  double min_track_confidence_ = 0.25;
  double coarse_error_m_ = 0.45;
  double fine_error_m_ = 0.08;
  double coarse_stable_s_ = 0.6;
  double fine_stable_s_ = 0.8;
  double alignment_timeout_s_ = 12.0;
  double detection_timeout_s_ = 3.0;
  double recover_hold_s_ = 0.5;
  double forced_release_height_tolerance_m_ = 0.12;
  double forced_release_xy_tolerance_m_ = 0.25;
  double forced_release_max_target_age_s_ = 1.0;
  double forced_release_max_target_error_m_ = 0.20;
  double forced_release_min_confidence_ = 0.35;
  double forced_release_area_margin_m_ = 0.10;
  double forced_release_stable_s_ = 0.80;
  double forced_release_descent_timeout_s_ = 8.0;
  double release_max_horizontal_speed_m_s_ = 0.15;
  double release_max_vertical_speed_m_s_ = 0.10;
  double release_max_tilt_rad_ = 8.0 * kPi / 180.0;
  double release_max_angular_rate_rad_s_ = 10.0 * kPi / 180.0;
  double vision_heartbeat_timeout_s_ = 1.5;
  double vision_max_pipeline_delay_s_ = 1.5;
  double vision_future_tolerance_s_ = 0.05;
  double vision_transform_tolerance_s_ = 0.10;
  double nav_interpolation_max_gap_s_ = 0.20;
  double odom_history_s_ = 5.0;
  double prestream_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double odom_timeout_s_ = 0.75;
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
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;
  double angular_rate_rad_s_ = 0.0;
  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;
  double servo_ack_timeout_s_ = 3.0;

  int search_lane_count_ = 3;
  int max_search_passes_ = 3;
  int required_bucket_count_ = 3;
  int min_confirmations_ = 5;
  int diameter_filter_window_ = 9;
  int vision_min_messages_before_takeoff_ = 3;
  int payload_count_ = 2;
  int search_pass_ = 0;
  std::size_t search_index_ = 0U;
  std::size_t payload_index_ = 0U;
  std::size_t initialization_pending_count_ = 0U;
  std::size_t next_bucket_id_ = 1U;
  std::size_t vision_message_count_ = 0U;
  std::size_t extended_state_sequence_ = 0U;
  std::size_t landing_last_extended_state_sequence_ = 0U;
  std::size_t landing_ground_message_count_ = 0U;
  std::vector<double> release_offsets_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> stowed_pwm_;
  std::vector<int64_t> release_pwm_;
  std::vector<double> release_duration_s_;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_compass_time_;
  rclcpp::Time last_extended_state_time_;
  rclcpp::Time last_vision_message_time_;
  rclcpp::Time last_time_aligned_vision_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time release_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_servo_attempt_time_;
  rclcpp::Time last_status_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr
    extended_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualDropMissionNode>());
  rclcpp::shutdown();
  return 0;
}
