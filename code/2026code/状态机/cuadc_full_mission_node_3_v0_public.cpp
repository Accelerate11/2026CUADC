/*
 * CUADC 全流程任务节点
 *
 * 文件用途
 * --------
 * 本文件是比赛正式任务控制器经过主动简化后的开源版本。
 * 它保留了总体软件架构、ROS 2 / MAVROS 接口、安全状态机、坐标系设计思路
 * 以及后续扩展接口，但有意隐藏了仍具有比赛竞争价值的参数整定和若干核心算法。
 *
 * 本公开版有意不提供的内容
 * ------------------------
 * 1. 正式版目标跟踪、去重、稳定性判断与优先级排序算法。
 * 2. 相机、机体与真实投放口之间的高精度外参标定值。
 * 3. 针对比赛场地优化后的搜索航线几何参数和越界主动回收策略。
 * 4. 正式版相机曝光时刻与飞控里程计之间的硬时间同步实现。
 * 5. 正式投放门限、重试策略、运动补偿和投放判定细节。
 * 6. 比赛使用的灾情侦察航点布局与分阶段速度规划。
 * 7. 场地横向标定、动态安全余量以及与计分规则相关的目标选择策略。
 *
 * 对于上述敏感部分，本文件不会简单删除接口，而是保留清晰的输入、输出和实现思路，
 * 方便学习者基于自己的飞机、相机、场地和任务要求完成独立实现，而不会直接得到
 * 本队比赛最终版本的关键参数与算法。
 *
 * 任务流程（公开版）
 * ------------------
 *   WAIT_FCU              等待飞控连接
 *     -> WAIT_NAV_STABLE  等待导航与传感器稳定
 *     -> PRESTREAM        预发送位置设定点
 *     -> WAIT_GUIDED      等待飞手切入 GUIDED
 *     -> WAIT_ARM         等待/请求解锁
 *     -> TAKEOFF          自动起飞
 *     -> SEARCH           搜索投放目标
 *     -> ALIGN            对准目标
 *     -> RELEASE          执行投放
 *     -> SEARCH / RECON_TRANSIT
 *     -> RECON_SCAN       灾情侦察拍照
 *     -> RETURN_HOME      返航
 *     -> LAND             降落
 *     -> DISARM           上锁
 *     -> DONE             任务结束
 *
 * 使用说明
 * --------
 * - 通过 MAVROS 与 ArduPilot 飞控通信。
 * - 本地位置来源为 /mavros/local_position/odom。
 * - 投放目标检测结果使用 geometry_msgs/PoseArray 接收。
 * - 公开版默认参数仅用于展示程序结构，不代表比赛整定值，也不应直接照搬到真机。
 * - 真机测试前应先完成仿真、无桨测试、舵机方向确认和充分的地面安全检查。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kMissionVersion = "cuadc-community-edition-2026-08";

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Detection
{
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
  return std::sqrt(
    (a.x - b.x) * (a.x - b.x) +
    (a.y - b.y) * (a.y - b.y) +
    (a.z - b.z) * (a.z - b.z));
}

}  // 匿名命名空间

class CuadcPublicMissionNode final : public rclcpp::Node
{
public:
  CuadcPublicMissionNode()
  : Node("cuadc_public_mission_node")
  {
    declare_parameters();
    load_parameters();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&CuadcPublicMissionNode::state_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionNode::odom_callback, this, std::placeholders::_1));

    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionNode::compass_callback, this, std::placeholders::_1));

    bucket_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      bucket_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CuadcPublicMissionNode::bucket_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

    recon_photo_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/cuadc/recon/photo_mode", 10);
    recon_capture_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/cuadc/recon/capture_request", 10);
    recon_capture_ack_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/cuadc/recon/capture_done", 10,
      std::bind(
        &CuadcPublicMissionNode::recon_capture_ack_callback,
        this, std::placeholders::_1));

    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    state_enter_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_setpoint_time_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&CuadcPublicMissionNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CUADC public mission node ready [%s]. Competition-sensitive logic has been omitted.",
      kMissionVersion);
  }

private:
  // ---------------------------------------------------------------------------
  // 参数声明与读取
  // ---------------------------------------------------------------------------
  void declare_parameters()
  {
    declare_parameter<std::string>(
      "bucket_detection_topic", "/perception/drop_buckets_body");

    // 公开版默认值有意设置得较为通用、保守，不代表正式比赛参数。
    declare_parameter<double>("takeoff_alt_m", 3.0);
    declare_parameter<double>("search_alt_m", 2.5);
    declare_parameter<double>("align_alt_m", 2.0);
    declare_parameter<double>("return_alt_m", 3.0);

    declare_parameter<double>("transit_speed_m_s", 1.5);
    declare_parameter<double>("search_speed_m_s", 0.8);
    declare_parameter<double>("align_speed_m_s", 0.4);
    declare_parameter<double>("return_speed_m_s", 1.5);

    declare_parameter<double>("waypoint_accept_radius_m", 0.45);
    declare_parameter<double>("align_accept_radius_m", 0.30);
    declare_parameter<double>("align_stable_s", 0.8);
    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("odom_timeout_s", 1.0);
    declare_parameter<double>("vision_timeout_s", 1.5);

    declare_parameter<bool>("auto_arm_on_guided", true);
    declare_parameter<bool>("enable_release_output", false);

    // 硬件映射使用通用示例值；实际舵机通道与PWM必须由使用者自行确认。
    declare_parameter<std::vector<int64_t>>(
      "servo_channels", std::vector<int64_t>{9, 10});
    declare_parameter<std::vector<int64_t>>(
      "servo_stowed_pwm", std::vector<int64_t>{1100, 1100});
    declare_parameter<std::vector<int64_t>>(
      "servo_release_pwm", std::vector<int64_t>{1900, 1900});
    declare_parameter<double>("servo_release_hold_s", 0.5);

    // 这里只保留通用的“相机/检测结果到机体系”接口，不公开正式比赛外参标定值。
    // 公开版假定输入检测结果已经转换到机体 FLU 坐标系：+X前、+Y左、+Z上。
    declare_parameter<double>("camera_body_x_m", 0.0);
    declare_parameter<double>("camera_body_y_m", 0.0);
    declare_parameter<double>("camera_body_z_m", 0.0);

    // 下面是任务坐标系中的演示搜索矩形，并非比赛真实场地搜索几何参数。
    declare_parameter<double>("demo_search_x_min_m", 6.0);
    declare_parameter<double>("demo_search_x_max_m", 10.0);
    declare_parameter<double>("demo_search_half_width_m", 2.0);
    declare_parameter<int>("demo_search_lanes", 3);

    // 下面仅给出演示侦察点，不是比赛实际使用的灾情侦察航点。
    declare_parameter<double>("demo_recon_x_m", 14.0);
    declare_parameter<double>("demo_recon_half_width_m", 1.5);
  }

  void load_parameters()
  {
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();

    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
    align_alt_m_ = std::max(1.0, get_parameter("align_alt_m").as_double());
    return_alt_m_ = std::max(1.0, get_parameter("return_alt_m").as_double());

    transit_speed_m_s_ = std::max(0.2, get_parameter("transit_speed_m_s").as_double());
    search_speed_m_s_ = std::max(0.2, get_parameter("search_speed_m_s").as_double());
    align_speed_m_s_ = std::max(0.1, get_parameter("align_speed_m_s").as_double());
    return_speed_m_s_ = std::max(0.2, get_parameter("return_speed_m_s").as_double());

    waypoint_accept_radius_m_ = std::max(
      0.1, get_parameter("waypoint_accept_radius_m").as_double());
    align_accept_radius_m_ = std::max(
      0.05, get_parameter("align_accept_radius_m").as_double());
    align_stable_s_ = std::max(0.1, get_parameter("align_stable_s").as_double());
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

    camera_offset_body_.x = get_parameter("camera_body_x_m").as_double();
    camera_offset_body_.y = get_parameter("camera_body_y_m").as_double();
    camera_offset_body_.z = get_parameter("camera_body_z_m").as_double();

    demo_search_x_min_m_ = get_parameter("demo_search_x_min_m").as_double();
    demo_search_x_max_m_ = get_parameter("demo_search_x_max_m").as_double();
    demo_search_half_width_m_ = std::max(
      0.5, get_parameter("demo_search_half_width_m").as_double());
    demo_search_lanes_ = std::max(
      1, static_cast<int>(get_parameter("demo_search_lanes").as_int()));

    demo_recon_x_m_ = get_parameter("demo_recon_x_m").as_double();
    demo_recon_half_width_m_ = std::max(
      0.5, get_parameter("demo_recon_half_width_m").as_double());

    if (servo_channels_.size() < 2U ||
      servo_stowed_pwm_.size() < 2U ||
      servo_release_pwm_.size() < 2U)
    {
      RCLCPP_WARN(
        get_logger(),
        "Servo parameter arrays have fewer than two entries. Release output will be disabled.");
      enable_release_output_ = false;
    }
  }

  // ---------------------------------------------------------------------------
  // ROS 话题回调函数
  // ---------------------------------------------------------------------------
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (was_guided && !guided_active_ && is_automatic_state(state_)) {
      RCLCPP_WARN(get_logger(), "GUIDED lost. Switching to pilot override mode.");
      publish_setpoint_enabled_ = false;
      enter(State::PILOT_OVERRIDE);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_.x = msg->pose.pose.position.x;
    position_.y = msg->pose.pose.position.y;
    position_.z = msg->pose.pose.position.z;

    horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;

    const auto & q = msg->pose.pose.orientation;
    const double norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
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
    last_odom_time_ = now();
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    current_heading_enu_ = normalize_angle(
      (90.0 - current_compass_deg_) * kPi / 180.0);
    have_compass_ = true;
    last_compass_time_ = now();
  }

  void bucket_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    last_vision_time_ = now();
    have_vision_ = true;

    if (!frame_locked_ || msg->poses.empty()) {
      return;
    }

    // -----------------------------------------------------------------------
    // 公开版：这里有意简化“视觉观测 -> 导航坐标”的完整链路。
    // -----------------------------------------------------------------------
    // 正式实现的基本思路：
    //   1. 为每一帧图像记录真正的采集/曝光时间戳。
    //   2. 利用里程计历史，将飞机位姿插值到相机曝光时刻。
    //   3. 根据相机外参把相机坐标系中的目标转换到机体系。
    //   4. 再结合曝光时刻的飞机位姿，把机体系目标转换到本地导航坐标系。
    //   5. 对多帧目标进行跟踪、稳定性判断，并剔除重复或不可靠目标。
    //
    // 比赛正式版的具体时间同步、插值和滤波实现不在此公开。
    // 本公开代码假设 PoseArray 中的位置已经是机体 FLU 坐标，
    // 并仅使用“当前”飞机位姿做演示变换，因此不能替代严格的曝光时刻同步。
    detections_.clear();
    detections_.reserve(msg->poses.size());

    for (const auto & pose : msg->poses) {
      Detection d;
      d.body.x = pose.position.x + camera_offset_body_.x;
      d.body.y = pose.position.y + camera_offset_body_.y;
      d.body.z = pose.position.z + camera_offset_body_.z;
      d.size_hint = std::max(0.0, pose.orientation.x);
      d.confidence = std::clamp(pose.orientation.y, 0.0, 1.0);
      d.stamp = now();
      d.local = body_to_local_demo(d.body);
      detections_.push_back(d);
    }
  }

  void recon_capture_ack_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    const std::size_t id = static_cast<std::size_t>(msg->data);
    if (id == 0U) {
      return;
    }
    if (std::find(recon_photo_acks_.begin(), recon_photo_acks_.end(), id) ==
      recon_photo_acks_.end())
    {
      recon_photo_acks_.push_back(id);
    }
  }

  // ---------------------------------------------------------------------------
  // 坐标变换辅助函数
  // ---------------------------------------------------------------------------
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

  Point3 body_to_local_demo(const Point3 & body) const
  {
    // 仅用于演示的坐标变换：使用固定任务航向，不包含正式版的时间戳对齐、
    // 完整横滚/俯仰补偿以及真实投放口外参补偿模型。
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{
      position_.x + c * body.x - s * body.y,
      position_.y + s * body.x + c * body.y,
      position_.z + body.z};
  }

  // ---------------------------------------------------------------------------
  // 任务航线与任务结构构建
  // ---------------------------------------------------------------------------
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

    RCLCPP_INFO(
      get_logger(),
      "Mission frame locked: home=(%.2f, %.2f, %.2f), heading=%.1f deg",
      home_->x, home_->y, home_->z, locked_compass_deg_);
  }

  void build_demo_search_route()
  {
    // -----------------------------------------------------------------------
    // 保留思路但不公开：正式比赛搜索航线规划器。
    // -----------------------------------------------------------------------
    // 正式实现的基本思路：
    //   - 根据实际任务区尺寸生成搜索条带/蛇形航线；
    //   - 在场地边界内保留足够的横向与纵向安全余量；
    //   - 相邻航线方向交替，减少无效折返距离；
    //   - 增加主动边界保护，发现横向超调时先将飞机拉回安全区域；
    //   - 搜索是否结束应由“稳定目标数量”决定，而不是必须把整条航线飞完。
    //
    // 下面的公开实现只是一个简单、通用的“割草机式”蛇形搜索示例。
    search_route_.clear();

    const double x0 = std::min(demo_search_x_min_m_, demo_search_x_max_m_);
    const double x1 = std::max(demo_search_x_min_m_, demo_search_x_max_m_);

    for (int lane = 0; lane < demo_search_lanes_; ++lane) {
      const double t = demo_search_lanes_ == 1 ?
        0.5 : static_cast<double>(lane) / static_cast<double>(demo_search_lanes_ - 1);
      const double y = -demo_search_half_width_m_ +
        2.0 * demo_search_half_width_m_ * t;
      const bool forward = (lane % 2 == 0);
      search_route_.push_back(field_to_local(forward ? x0 : x1, y, search_alt_m_));
      search_route_.push_back(field_to_local(forward ? x1 : x0, y, search_alt_m_));
    }
  }

  void build_demo_recon_route()
  {
    // -----------------------------------------------------------------------
    // 保留思路但不公开：正式比赛灾情侦察航线。
    // -----------------------------------------------------------------------
    // 正式实现的基本思路：
    //   - 用尽可能少的相机视点覆盖整个侦察区域；
    //   - 同时考虑相机视场角、飞行高度、运动模糊和总任务时间；
    //   - 到达有效拍摄位置后立即触发拍照，避免不必要的悬停；
    //   - 高速返航前先爬升到更安全的高度。
    //
    // 公开版只使用4个演示航点。
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
  // 公开版目标选择占位实现
  // ---------------------------------------------------------------------------
  std::optional<Detection> choose_demo_target() const
  {
    // -----------------------------------------------------------------------
    // 保留思路但不公开：正式版多目标跟踪、去重与排序算法。
    // -----------------------------------------------------------------------
    // 面向比赛的完整实现至少应考虑：
    //   - 多帧时间维度上的连续确认次数；
    //   - 目标位置的空间一致性、方差或协方差；
    //   - 目标尺寸估计在多帧之间是否稳定；
    //   - 检测置信度的历史变化；
    //   - 目标ID在时间上的持续性；
    //   - 对空间上过近的重复候选进行去重；
    //   - 排除已经完成投放的目标；
    //   - 根据任务收益/计分逻辑进行目标优先级设计。
    //
    // 本演示实现仅选择“当前尚未使用的最近目标”，
    // 它只是展示接口用法，并不是比赛级目标选择策略。
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

      const double dist = distance_xy(position_, d.local);
      if (dist < best_distance) {
        best_distance = dist;
        best = &d;
      }
    }

    return best ? std::optional<Detection>(*best) : std::nullopt;
  }

  Point3 desired_demo_release_pose(const Detection & target) const
  {
    // -----------------------------------------------------------------------
    // 保留思路但不公开：真实投放口外参标定与补偿。
    // -----------------------------------------------------------------------
    // 正式实现的基本思路：
    //   vehicle_position_desired = target_local - R_yaw * release_port_offset
    //   即：让“投放口”而不是“飞机几何中心”移动到目标上方；如有需要还应继续加入
    //   横滚/俯仰姿态补偿、释放延迟以及载荷运动模型。真实外参由具体机体标定得到。
    //
    // 公开版为了便于阅读，仅让飞机中心移动到目标上方。
    return Point3{target.local.x, target.local.y, home_->z + align_alt_m_};
  }

  // ---------------------------------------------------------------------------
  // 主控制循环 / 状态机调度
  // ---------------------------------------------------------------------------
  void tick()
  {
    if (publish_setpoint_enabled_ && frame_locked_) {
      publish_setpoint();
    }

    check_service_results();

    if (mission_started_ &&
      state_ != State::RETURN_HOME && state_ != State::LAND &&
      state_ != State::DISARM && state_ != State::DONE &&
      state_ != State::PILOT_OVERRIDE && state_ != State::ABORT &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      fail_and_return("Mission timeout");
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
          break;
        }
        if (fcu_state_.armed) {
          fail("Aircraft armed before mission lock");
          break;
        }
        if (navigation_ready()) {
          lock_frame();
          publish_setpoint_enabled_ = true;
          enter(State::PRESTREAM);
        }
        break;

      case State::PRESTREAM:
        target_ = *home_;
        if (!navigation_ready()) {
          fail("Navigation lost during prestream");
        } else if ((now() - state_enter_time_).seconds() >= prestream_hold_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = *home_;
        if (guided_active_) {
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = *home_;
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
        if (!done_logged_) {
          done_logged_ = true;
          RCLCPP_INFO(
            get_logger(),
            "Mission ended. Public edition: released=%zu, recon_acks=%zu, failed=%s",
            payload_index_, recon_photo_acks_.size(), mission_failed_ ? "true" : "false");
        }
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_enabled_ = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Pilot override active. Automatic mission node will exit.");
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
  // 搜索、对准与投放流程
  // ---------------------------------------------------------------------------
  void start_search()
  {
    current_target_.reset();
    align_stable_since_.reset();

    if (search_route_.empty()) {
      fail_and_return("Search route is empty");
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
      target_ = desired_demo_release_pose(*current_target_);
      align_stable_since_.reset();
      enter(State::ALIGN);
      return;
    }

    target_ = sample_segment();
    if (segment_complete(waypoint_accept_radius_m_)) {
      ++search_index_;
      if (search_index_ >= search_route_.size()) {
        // 公开版行为：搜索航线只飞一遍；仍未发现目标时直接安全返航。
        fail_and_return("No target found during demonstration search pass");
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

    // 公开版允许新视觉观测对目标位置做有限更新，但只有在新观测仍靠近原目标时才接受；
    // 这一逻辑有意保持简单，仅用于说明“视觉更新目标”的基本接口。
    const auto refreshed = choose_demo_target();
    if (refreshed.has_value() &&
      distance_xy(refreshed->local, current_target_->local) < 0.8)
    {
      current_target_ = refreshed;
    }

    const Point3 desired = desired_demo_release_pose(*current_target_);
    slew_target_toward(desired, align_speed_m_s_);

    const bool within_position = distance_xy(position_, desired) <= align_accept_radius_m_;
    const bool within_altitude = std::abs(position_.z - desired.z) <= 0.20;
    const bool slow_enough = horizontal_speed_m_s_ <= 0.25 && std::abs(vertical_speed_m_s_) <= 0.15;

    if (!(within_position && within_altitude && slow_enough)) {
      align_stable_since_.reset();
      return;
    }

    if (!align_stable_since_.has_value()) {
      align_stable_since_ = now();
      return;
    }

    if ((now() - *align_stable_since_).seconds() >= align_stable_s_) {
      frozen_release_pose_ = desired;
      target_ = frozen_release_pose_;
      release_started_ = false;
      release_finished_ = false;
      enter(State::RELEASE);
    }
  }

  void update_release()
  {
    target_ = frozen_release_pose_;

    // -----------------------------------------------------------------------
    // 保留思路但不公开：正式版投放安全门限与释放判定。
    // -----------------------------------------------------------------------
    // 在真正驱动舵机释放载荷之前，完整实现应至少再次检查：
    //   - 当前目标身份是否仍然正确，以及观测是否足够新；
    //   - 目标点和补偿后的投放点是否仍位于合法任务区域；
    //   - 考虑投放口外参后的XY误差；
    //   - 实际高度与期望投放高度的误差；
    //   - 水平速度与垂直速度是否足够小；
    //   - 横滚角、俯仰角以及机体角速度是否满足要求；
    //   - 任意门限不满足时，是继续等待、重新对准、保瓶返航还是触发其他故障策略。
    //
    // 公开版仅保留较简单的位置和速度检查，用于展示状态机结构。
    const bool pose_ok =
      distance_xy(position_, frozen_release_pose_) <= align_accept_radius_m_ &&
      std::abs(position_.z - frozen_release_pose_.z) <= 0.20;
    const bool motion_ok =
      horizontal_speed_m_s_ <= 0.20 && std::abs(vertical_speed_m_s_) <= 0.12;
    const bool attitude_ok =
      std::abs(current_roll_) <= 10.0 * kPi / 180.0 &&
      std::abs(current_pitch_) <= 10.0 * kPi / 180.0;

    if (!release_started_) {
      if (!(pose_ok && motion_ok && attitude_ok)) {
        return;
      }

      if (enable_release_output_) {
        if (!send_servo(payload_index_, true)) {
          fail_and_return("Failed to send public-edition release command");
          return;
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "SIMULATED RELEASE payload=%zu (enable_release_output=false)",
          payload_index_ + 1U);
      }

      release_started_ = true;
      release_start_time_ = now();
      return;
    }

    if (!release_finished_ &&
      (now() - release_start_time_).seconds() >= servo_release_hold_s_)
    {
      if (enable_release_output_) {
        (void)send_servo(payload_index_, false);
      }
      release_finished_ = true;
      finish_demo_release();
    }
  }

  void finish_demo_release()
  {
    if (current_target_.has_value()) {
      released_target_points_.push_back(current_target_->local);
    }

    ++payload_index_;
    current_target_.reset();
    release_started_ = false;
    release_finished_ = false;
    align_stable_since_.reset();

    if (payload_index_ < 2U) {
      // 公开版在第一瓶之后重新进入搜索，而不使用比赛正式版的“多目标一次冻结”优化；
      // 该优化会显著影响任务时间与稳定性，因此只保留实现思路，不公开最终逻辑。
      start_search();
    } else {
      start_recon_phase();
    }
  }

  // ---------------------------------------------------------------------------
  // 灾情侦察流程
  // ---------------------------------------------------------------------------
  void start_recon_phase()
  {
    std_msgs::msg::Bool mode;
    mode.data = true;
    recon_photo_mode_pub_->publish(mode);

    recon_index_ = 0U;
    recon_photo_acks_.clear();

    if (recon_route_.empty()) {
      start_return_home();
      return;
    }

    start_segment(position_, recon_route_.front(), transit_speed_m_s_);
    enter(State::RECON_TRANSIT);
  }

  void update_recon_transit()
  {
    target_ = sample_segment();
    if (segment_complete(waypoint_accept_radius_m_)) {
      request_recon_photo(0U);
      recon_index_ = 1U;
      if (recon_index_ < recon_route_.size()) {
        start_segment(position_, recon_route_[recon_index_], search_speed_m_s_);
        enter(State::RECON_SCAN);
      } else {
        start_return_home();
      }
    }
  }

  void update_recon_scan()
  {
    if (recon_index_ >= recon_route_.size()) {
      start_return_home();
      return;
    }

    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }

    request_recon_photo(recon_index_);
    ++recon_index_;

    if (recon_index_ < recon_route_.size()) {
      start_segment(position_, recon_route_[recon_index_], search_speed_m_s_);
    } else {
      start_return_home();
    }
  }

  void request_recon_photo(std::size_t index)
  {
    if (index >= recon_route_.size()) {
      return;
    }
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "public_recon_wp_" + std::to_string(index + 1U);
    msg.point.x = recon_route_[index].x;
    msg.point.y = recon_route_[index].y;
    msg.point.z = recon_route_[index].z;
    recon_capture_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // 返航、降落与上锁
  // ---------------------------------------------------------------------------
  void start_return_home()
  {
    if (!home_.has_value()) {
      fail("No home position for return");
      return;
    }

    const Point3 high_home{
      home_->x,
      home_->y,
      home_->z + return_alt_m_};

    start_segment(position_, high_home, return_speed_m_s_);
    enter(State::RETURN_HOME);
  }

  void update_return()
  {
    target_ = sample_segment();
    if (segment_complete(std::max(0.5, waypoint_accept_radius_m_))) {
      publish_setpoint_enabled_ = false;
      enter(State::LAND);
    }
  }

  bool landing_candidate() const
  {
    return odom_fresh() &&
      std::abs(relative_altitude()) <= 0.15 &&
      horizontal_speed_m_s_ <= 0.12 &&
      std::abs(vertical_speed_m_s_) <= 0.10;
  }

  // ---------------------------------------------------------------------------
  // 轨迹生成与位置设定点辅助函数
  // ---------------------------------------------------------------------------
  void start_segment(const Point3 & start, const Point3 & end, double max_speed)
  {
    const double distance = distance_xyz(start, end);
    const double speed = std::max(0.1, max_speed);
    segment_.start = start;
    segment_.end = end;
    segment_.start_time = now();
    segment_.duration_s = std::max(0.8, distance / speed);
  }

  Point3 sample_segment() const
  {
    const double t = std::clamp(
      (now() - segment_.start_time).seconds() / segment_.duration_s,
      0.0, 1.0);

    // Smoothstep 平滑插值：公开版仅使用简单轨迹，不是比赛正式版的分阶段速度曲线。
    const double u = t * t * (3.0 - 2.0 * t);
    return Point3{
      segment_.start.x + u * (segment_.end.x - segment_.start.x),
      segment_.start.y + u * (segment_.end.y - segment_.start.y),
      segment_.start.z + u * (segment_.end.z - segment_.start.z)};
  }

  bool segment_complete(double radius) const
  {
    return (now() - segment_.start_time).seconds() >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  void slew_target_toward(const Point3 & desired, double max_speed)
  {
    const rclcpp::Time t_now = now();
    double dt = (t_now - last_setpoint_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2) {
      dt = 0.05;
    }
    last_setpoint_time_ = t_now;

    const double dx = desired.x - target_.x;
    const double dy = desired.y - target_.y;
    const double dz = desired.z - target_.z;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (d < 1.0e-6) {
      target_ = desired;
      return;
    }

    const double step = std::max(0.05, max_speed) * dt;
    if (d <= step) {
      target_ = desired;
      return;
    }

    const double scale = step / d;
    target_.x += scale * dx;
    target_.y += scale * dy;
    target_.z += scale * dz;
  }

  // ---------------------------------------------------------------------------
  // MAVROS 服务调用辅助函数
  // ---------------------------------------------------------------------------
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
    if (!takeoff_client_->service_is_ready() || takeoff_future_.valid() || !request_allowed()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(takeoff_alt_m_);
    request->yaw = static_cast<float>(locked_compass_deg_);
    mark_request();
    takeoff_future_ = takeoff_client_->async_send_request(request).future.share();
    takeoff_sent_ = true;
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
  }

  void request_arm(bool arm)
  {
    if (!arm_client_->service_is_ready() || arm_future_.valid() || !request_allowed()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    pending_arm_value_ = arm;
    mark_request();
    arm_future_ = arm_client_->async_send_request(request).future.share();
  }

  bool send_servo(std::size_t payload, bool release)
  {
    if (!command_client_->service_is_ready() ||
      payload >= servo_channels_.size() ||
      payload >= servo_stowed_pwm_.size() ||
      payload >= servo_release_pwm_.size())
    {
      return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = 183;  // MAV_CMD_DO_SET_SERVO：设置指定舵机PWM输出
    request->confirmation = 0;
    request->param1 = static_cast<float>(servo_channels_[payload]);
    request->param2 = static_cast<float>(
      release ? servo_release_pwm_[payload] : servo_stowed_pwm_[payload]);

    // 公开骨架中为了简化代码，不会阻塞状态机等待每次舵机ACK；
    // 正式实现应跟踪ACK、超时、重复发送，以及投放后舵机安全收回等行为。
    (void)command_client_->async_send_request(request);
    return true;
  }

  void check_service_results()
  {
    if (takeoff_future_.valid() &&
      takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = takeoff_future_.get();
      if (!response->success) {
        takeoff_sent_ = false;
        RCLCPP_WARN(get_logger(), "Takeoff command rejected; will retry");
      }
      takeoff_future_ = {};
    }

    if (land_future_.valid() &&
      land_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = land_future_.get();
      if (!response->success) {
        RCLCPP_WARN(get_logger(), "Land command rejected; will retry");
      }
      land_future_ = {};
    }

    if (arm_future_.valid() &&
      arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto response = arm_future_.get();
      if (!response->success) {
        RCLCPP_WARN(
          get_logger(), "%s command rejected; will retry if state still requires it",
          pending_arm_value_ ? "Arm" : "Disarm");
      }
      arm_future_ = {};
    }
  }

  // ---------------------------------------------------------------------------
  // 安全门禁与故障处理辅助函数
  // ---------------------------------------------------------------------------
  bool odom_fresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  bool compass_fresh() const
  {
    return have_compass_ && (now() - last_compass_time_).seconds() <= 1.5;
  }

  bool vision_fresh() const
  {
    return have_vision_ && (now() - last_vision_time_).seconds() <= vision_timeout_s_;
  }

  bool navigation_ready() const
  {
    return fcu_state_.connected && !fcu_state_.armed && odom_fresh() && compass_fresh();
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail("FCU disconnected or disarmed during automatic mission");
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    if (!odom_fresh()) {
      fail("Odometry stale during automatic mission");
      return false;
    }
    return true;
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  bool is_automatic_state(State state) const
  {
    return state == State::WAIT_ARM || state == State::TAKEOFF ||
      state == State::SEARCH || state == State::ALIGN ||
      state == State::RELEASE || state == State::RECON_TRANSIT ||
      state == State::RECON_SCAN || state == State::RETURN_HOME;
  }

  void fail(const std::string & reason)
  {
    if (!mission_failed_) {
      mission_failed_ = true;
      failure_reason_ = reason;
    }
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  void fail_and_return(const std::string & reason)
  {
    if (!mission_failed_) {
      mission_failed_ = true;
      failure_reason_ = reason;
    }
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    if (fcu_state_.armed && home_.has_value() && guided_active_ && odom_fresh()) {
      start_return_home();
    } else {
      enter(fcu_state_.armed ? State::LAND : State::ABORT);
    }
  }

  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s", state_name(state_).c_str(), state_name(next).c_str());
    state_ = next;
    state_enter_time_ = now();
  }

  std::string state_name(State state) const
  {
    switch (state) {
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

  // ---------------------------------------------------------------------------
  // ROS 订阅、发布、服务客户端等接口对象
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bucket_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr recon_capture_ack_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recon_photo_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr recon_capture_pub_;

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------------------------------------------------------------------
  // 状态机运行时状态变量
  // ---------------------------------------------------------------------------
  mavros_msgs::msg::State fcu_state_;
  State state_ = State::WAIT_FCU;

  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool have_vision_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_enabled_ = false;
  bool mission_started_ = false;
  bool mission_failed_ = false;
  bool done_logged_ = false;

  Point3 position_;
  Point3 target_;
  Point3 frozen_release_pose_;
  Point3 camera_offset_body_;
  std::optional<Point3> home_;
  std::optional<Detection> current_target_;

  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_compass_time_;
  rclcpp::Time last_vision_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_setpoint_time_;
  std::optional<rclcpp::Time> align_stable_since_;
  std::optional<rclcpp::Time> landing_stable_since_;

  Segment segment_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;
  std::vector<Detection> detections_;
  std::vector<Point3> released_target_points_;
  std::vector<std::size_t> recon_photo_acks_;

  std::size_t search_index_ = 0U;
  std::size_t recon_index_ = 0U;
  std::size_t payload_index_ = 0U;

  bool takeoff_sent_ = false;
  bool release_started_ = false;
  bool release_finished_ = false;
  rclcpp::Time release_start_time_;
  bool pending_arm_value_ = false;

  std::string failure_reason_;
  std::string bucket_topic_;

  // ---------------------------------------------------------------------------
  // 已加载的参数与配置缓存
  // ---------------------------------------------------------------------------
  double takeoff_alt_m_ = 3.0;
  double search_alt_m_ = 2.5;
  double align_alt_m_ = 2.0;
  double return_alt_m_ = 3.0;

  double transit_speed_m_s_ = 1.5;
  double search_speed_m_s_ = 0.8;
  double align_speed_m_s_ = 0.4;
  double return_speed_m_s_ = 1.5;

  double waypoint_accept_radius_m_ = 0.45;
  double align_accept_radius_m_ = 0.30;
  double align_stable_s_ = 0.8;
  double prestream_hold_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double odom_timeout_s_ = 1.0;
  double vision_timeout_s_ = 1.5;

  bool auto_arm_on_guided_ = true;
  bool enable_release_output_ = false;

  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> servo_stowed_pwm_;
  std::vector<int64_t> servo_release_pwm_;
  double servo_release_hold_s_ = 0.5;

  double demo_search_x_min_m_ = 6.0;
  double demo_search_x_max_m_ = 10.0;
  double demo_search_half_width_m_ = 2.0;
  int demo_search_lanes_ = 3;
  double demo_recon_x_m_ = 14.0;
  double demo_recon_half_width_m_ = 1.5;

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CuadcPublicMissionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
