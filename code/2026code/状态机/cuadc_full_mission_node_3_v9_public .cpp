/*
 * @file cuadc_full_mission_node.cpp
 * @brief CUADC 全任务状态机：准备、搜索、双次投放、航点侦察、返航与降落。
 *
 * 非同步模式的飞手确认与起飞准备：
 *   WAIT_FCU -> WAIT_GUIDED -> LOCK_FRAME -> PRESTREAM -> WAIT_ARM
 * measurement_time_sync 同步模式先准备再等待新授权：
 *   WAIT_FCU -> PRESTREAM -> WAIT_GUIDED -> WAIT_ARM
 * 同步模式在准备时建立临时坐标原点，收到就绪后的新 GUIDED 授权时再次锁定原点。
 *
 * 自主任务主流程；两次投放共用 ALIGN / RELEASE，通过 payload_index_ 区分：
 *   TAKEOFF -> SEARCH -> ALIGN -> RELEASE -> ALIGN -> RELEASE
 *   -> RECON_CLIMB -> RECON_SURVEY -> RETURN_CLIMB
 *   -> RETURN_HOME -> LAND -> DISARM -> DONE
 *
 * RECON_SURVEY 只执行预设航点覆盖，本节点没有危险物图像识别订阅。
 *
 * 坐标约定：导航与发布目标使用 MAVROS 本地 ENU（东、北、上），长度为米；
 * 任务场地坐标以锁定起点为原点，按标定航向定义前、左、上。
 * 视觉检测和内部投放口偏移使用机体 FLU（前、左、上）；外部 FRD 参数在加载时转换。
 * 时间约定：接收年龄、状态停留及服务超时使用 steady_clock；观测采集时刻、
 * 导航插值、帧身份和初始保持窗后的观测资格使用 ROS 时间，不能直接跨域相减。
 * 分段投放的 0.5 / 4 / 6 秒阶段耗时仍由单调时间计算。
 *
 * 默认使用单线程执行器；回调先更新缓存，周期 tick 再推进任务并持续发布位置设定点。
 * 服务均异步请求，ACK 表示命令应答，不等于实物脱离或飞机已到达。
 * 现场适配应核对配置、坐标外参和执行机构；本文件中的门限并非通用飞行保证。
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
#include "release_gate.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

/** @brief 将弧度角折返到 [-π, π]，用于计算跨越 ±π 时的最短角差。 */
double normalize_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

/** @brief 将有限的角度值归一到 [0, 360)，用于地理航向的显示与换算。 */
double normalize_degrees(double value)
{
  value = std::fmod(value, 360.0);
  return value < 0.0 ? value + 360.0 : value;
}

/** @brief 三维点或位移；通常以米为单位，具体参考系由所在字段或函数说明确定。 */
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/** @brief 计算同一参考系内两点的水平距离（米），忽略高度差。 */
double distance_xy(const Point3 & lhs, const Point3 & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

/** @brief 计算同一参考系内两点的三维欧氏距离（米）。 */
double distance_xyz(const Point3 & lhs, const Point3 & rhs)
{
  return std::hypot(distance_xy(lhs, rhs), lhs.z - rhs.z);
}

/** @brief 罗盘航向历史：单调接收时刻与换算后的 ENU 偏航角（弧度）。 */
struct HeadingSample
{
  SteadyTimePoint stamp;
  double yaw = 0.0;
};

/** @brief 导航历史样本：ROS 采样时刻、本地 ENU 位置及姿态角（弧度）。 */
struct NavigationSample
{
  rclcpp::Time stamp;
  Point3 position;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

/** @brief 一帧中待投影的检测：机体 FLU 桶中心/桶沿位置（米）、直径及置信度。 */
struct PendingVisionDetection
{
  std::size_t index = 0U;
  Point3 body;
  Point3 rim_body;
  double diameter = 0.0;
  double confidence = 0.0;
};

/** @brief 等待导航历史覆盖的视觉帧；采集 ROS 时间与单调接收时间分别保留。 */
struct PendingVisionFrame
{
  rclcpp::Time stamp;
  SteadyTimePoint arrival;
  std::vector<PendingVisionDetection> detections;
};

/** @brief 本地 ENU 参考航段；start_time 使用单调时钟，duration_s 为计划时长。 */
struct Segment
{
  Point3 start;
  Point3 end;
  SteadyTimePoint start_time;
  double duration_s = 1.0;
};

/**
 * @brief 跨帧关联得到的桶轨迹，同时保留场地定位证据和当前视觉引导量。
 * local 是本地 ENU 位置，body / rim_body 为采集时刻机体 FLU 位置（米）。
 * capture_navigation 记录对应采样姿态；stamp 用于观测对齐，arrival 用于接收年龄。
 * confirmations 是累计匹配数，consecutive_frame_count 才表示连续帧证据；
 * frozen_memory 只标记冻结的目标记忆，不能使过期观测重新变得新鲜。
 */
struct BucketTrack
{
  std::size_t id = 0U;
  Point3 local;
  Point3 body;
  Point3 rim_body;
  std::optional<NavigationSample> capture_navigation;
  double diameter = 0.0;
  double confidence = 0.0;
  double position_deviation = 0.0;
  double diameter_deviation = 0.0;
  std::size_t confirmations = 0U;
  std::size_t consecutive_frame_count = 1U;
  std::size_t last_frame_sequence = 0U;
  std::deque<double> diameter_samples;
  std::deque<Point3> position_samples;
  rclcpp::Time stamp;
  SteadyTimePoint arrival;
  bool frozen_memory = false;
};

/** @brief 任务状态；普通迁移由 enter() 统一记时，人工接管与失败处理另有退出分支。 */
enum class State
{
  WAIT_FCU,        ///< 等待飞控连接及起飞前数据、配置与舵机准备。
  WAIT_GUIDED,     ///< 等待飞手切入 GUIDED；同步模式在此之前已完成准备与预发送。
  LOCK_FRAME,      ///< 锁定起点与标定航向，生成任务航线。
  PRESTREAM,       ///< 起飞前持续发布位置设定点。
  WAIT_ARM,        ///< 等待解锁条件与飞控解锁状态。
  TAKEOFF,         ///< 请求起飞并等待高度达到门限。
  SEARCH,          ///< 沿搜索航线获取目标，必要时保持位置等待视觉恢复。
  ALIGN,           ///< 对当前载荷的目标进行转场、粗对准或视觉对准。
  RELEASE,         ///< 推进下降、分段门限、开舵机和回收流程。
  RECON_CLIMB,     ///< 转入侦察前的高度与入口准备。
  RECON_SURVEY,    ///< 执行侦察航点与停留，无危险物图像识别。
  RETURN_CLIMB,    ///< 返航前调整至返航高度。
  RETURN_HOME,     ///< 沿参考航段回到起点附近。
  LAND,            ///< 请求降落，并观察持续落地证据。
  DISARM,          ///< 在落地确认条件满足后请求上锁。
  DONE,            ///< 已结束；成功或失败由任务结果字段另行记录。
  PILOT_OVERRIDE,  ///< 已观察到飞手接管，停止自主设定点控制。
  ABORT           ///< 已解锁时转入 LAND，未解锁时直接结束节点；本分支不等待执行器清理。
};

/** @brief 区分舵机应答的用途，避免把初始化、开门和回收 ACK 混在一起。 */
enum class ServoPurpose
{
  INITIALIZE,
  RELEASE,
  STOW
};

/** @brief 尚未完成的异步舵机命令；payload 为从零开始的载荷索引，sent 为单调时刻。 */
struct PendingServoCommand
{
  ServoPurpose purpose = ServoPurpose::INITIALIZE;
  std::size_t payload = 0U;
  SteadyTimePoint sent;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future;
};

/**
 * @brief 按载荷索引读取参数；越界时沿用最后一项，空数组才使用显式缺省值。
 * @note 该读取规则不代替 load_parameters() 对必需数组长度的配置校验。
 */
template<typename T>
T value_at(const std::vector<T> & values, std::size_t index, const T & fallback)
{
  if (index < values.size()) {
    return values[index];
  }
  return values.empty() ? fallback : values.back();
}

/** @brief 从扁平 xyz 数组读取第 index 个三元组；不足三项时整体使用 fallback。 */
Point3 vector3_at(
  const std::vector<double> & values, std::size_t index, const Point3 & fallback)
{
  const std::size_t offset = index * 3U;
  if (offset + 2U >= values.size()) {
    return fallback;
  }
  return Point3{values[offset], values[offset + 1U], values[offset + 2U]};
}

/** @brief 在副本上排序计算中位数，不改变轨迹历史；空队列按现有约定返回零。 */
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

/** @brief 计算相对指定 center 的绝对偏差中位数，量纲与原始样本相同。 */
double median_absolute_deviation(
  const std::deque<double> & values, double center)
{
  std::deque<double> deviations;
  for (const double value : values) {
    deviations.push_back(std::abs(value - center));
  }
  return median(std::move(deviations));
}

}  // 匿名命名空间：辅助类型与函数仅在本编译单元使用。

/**
 * @brief 将遥测、视觉轨迹、异步命令和任务状态集中在一个 ROS 节点中管理。
 * 回调只缓存或处理当前输入，定时器负责周期推进；当前 main 使用单线程执行器，
 * 这些共享字段没有独立互斥保护，切换到多线程执行器时需重新审查访问关系。
 */
class CuadcFullMissionNode final : public rclcpp::Node
{
  friend struct TimeSyncTest;
public:
  /**
   * @brief 声明并读取参数，建立遥测/视觉订阅、位置发布器、服务客户端和 20 Hz 定时器。
   * 配置不合法时保留节点用于诊断，由状态推进门控阻止自主任务，不在构造期间执行起飞。
   * measurement_time_sync_ 启用时才订阅飞控时钟与 IMU 历史，用于按采集时刻还原观测。
   */
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
    if (measurement_time_sync_) {
      timesync_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/mavros/time");
      timesync_sub_ = create_subscription<mavros_msgs::msg::TimesyncStatus>(
        "/mavros/timesync_status", rclcpp::SensorDataQoS(),
        std::bind(&CuadcFullMissionNode::timesync_callback, this, std::placeholders::_1));
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::SensorDataQoS(),
        std::bind(&CuadcFullMissionNode::imu_callback, this, std::placeholders::_1));
    }
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

    // 普通状态停留、接收超时、执行机构应答和航段进度使用单调时钟，避免校时跳变。
    // ROS 时间用于采集时刻、导航插值、帧身份及保持窗后的观测资格；分段阶段耗时仍用单调时间。
    // 初始化接收时刻不表示已经收到数据，实际有效性仍由 have_* 标志区分。
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
    control_rate_window_start_ = current_arrival;
    // 50 ms 是请求的调度周期；实际抖动与运行耗时由 timed_tick() 记录。
    timer_ = create_wall_timer(50ms, std::bind(&CuadcFullMissionNode::timed_tick, this));

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
  /**
   * @brief 注册节点接受的参数名、类型和缺省值；此处不代表完成配置有效性校验。
   * 参数覆盖由 ROS 加载；load_parameters() 再换算单位、约束范围并核对关联条件。
   * 保留已退出执行流程的历史参数名以兼容已有 YAML，在对应声明处注明无运行效果。
   */
  void declare_parameters()
  {
    declare_parameter<bool>("measurement_time_sync", false);
    declare_parameter<bool>("staged_release_gate", false);
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
    // 历史二维补偿参数：对应函数从未进入任务调用链，现仅保留声明以兼容旧 YAML。
    // 这些参数不参与当前运动补偿；按采集姿态还原观测的 body_to_local() 路径仍保留。
    declare_parameter<bool>("motion_compensation_enabled", false);
    declare_parameter<std::vector<double>>(
      "motion_compensation_matrix",
      std::vector<double>{0.85335529, 0.80596326, -0.97181809, 0.97507692});
    declare_parameter<std::vector<double>>(
      "motion_compensation_reference_field_xy",
      std::vector<double>{31.75, 0.0});
    declare_parameter<std::vector<double>>(
      "bucket_diameter_centers_m",
      std::vector<double>{0.150, 0.200, 0.250});
    declare_parameter<double>("bucket_diameter_class_tolerance_m", 0.015);

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
    // 历史按位置排除参数目前无执行效果；现行目标去重仍按已投放目标 ID 处理。
    declare_parameter<double>("released_bucket_exclusion_m", 0.25);
    declare_parameter<double>("bucket_position_filter_alpha", 0.25);
    declare_parameter<int>("bucket_position_filter_window", 15);
    declare_parameter<int>("bucket_search_min_confirmations", 6);
    declare_parameter<double>("bucket_search_max_position_deviation_m", 0.20);
    declare_parameter<double>("bucket_body_filter_alpha", 1.0);
    declare_parameter<double>("bucket_diameter_filter_alpha", 0.20);
    declare_parameter<double>("bucket_confidence_filter_alpha", 0.30);
    declare_parameter<double>("bucket_max_position_deviation_m", 0.25);
    declare_parameter<double>("bucket_max_diameter_deviation_m", 0.050);
    declare_parameter<double>("bucket_min_track_confidence", 0.25);

    declare_parameter<double>("coarse_error_m", 0.15);
    declare_parameter<double>("fine_error_m", 0.08);
    declare_parameter<double>("coarse_stable_s", 0.8);
    declare_parameter<double>("alignment_timeout_s", 18.0);
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

  /**
   * @brief 读取启动参数并建立运行期配置：限幅、角度换算、投放外参转换及模式组合校验。
   * @details 这是启动时读取，不提供动态参数更新回调。米、秒与弧度在成员名及换算处标明；
   * 外部投放口偏移采用机体 FRD，内部计算转换为 FLU。config_valid_ 只汇总本函数
   * 已实现的必要条件，不表示设备匹配、场地净空、测量准确度等已经得到实测验证。
   */
  void load_parameters()
  {
    flight_enable_ = get_parameter("flight_enable").as_bool();
    bucket_topic_ = get_parameter("bucket_detection_topic").as_string();
    bucket_projection_log_path_ =
      get_parameter("bucket_projection_log_path").as_string();
    measurement_time_sync_ = get_parameter("measurement_time_sync").as_bool();
    staged_release_gate_ = get_parameter("staged_release_gate").as_bool();
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
    bucket_diameter_centers_m_ =
      get_parameter("bucket_diameter_centers_m").as_double_array();
    bucket_diameter_class_tolerance_m_ = std::max(
      0.001, get_parameter("bucket_diameter_class_tolerance_m").as_double());

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
    position_filter_alpha_ = std::clamp(
      get_parameter("bucket_position_filter_alpha").as_double(), 0.01, 1.0);
    position_filter_window_ = static_cast<std::size_t>(std::clamp<int64_t>(
      get_parameter("bucket_position_filter_window").as_int(), 5, 31));
    if (position_filter_window_ % 2U == 0U) {
      ++position_filter_window_;
    }
    search_min_confirmations_ = std::max(
      3, static_cast<int>(get_parameter("bucket_search_min_confirmations").as_int()));
    search_max_position_deviation_m_ = std::max(
      0.05, get_parameter("bucket_search_max_position_deviation_m").as_double());
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
    // 外部参数为飞控机体 FRD（前、右、下）；内部使用 FLU（前、左、上）。
    // 只反转 Y/Z 符号，X 保持不变；后续函数再结合姿态变换到本地 ENU。
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

  /** @brief 当前是否选择舵机命令投放；只查询配置，不代表舵机已初始化或动作完成。 */
  bool uses_servo() const
  {
    return release_mode_ == "servo";
  }

  /** @brief 当前是否选择虚拟投放服务；服务连接与应答由后续请求流程单独检查。 */
  bool uses_virtual() const
  {
    return release_mode_ == "virtual";
  }

  /**
   * @brief 接收飞控连接、模式与解锁状态，并处理自主控制权被收回的情况。
   * @param message MAVROS 状态回调消息；本函数按到达顺序更新缓存，不做采样时间插值。
   * @note 离开需要 GUIDED 的任务阶段时停止发布设定点，按机构状态请求收回并转入
   * PILOT_OVERRIDE；LAND/DISARM 阶段允许 GUIDED/LAND，其余已解锁模式视为控制权交接。
   * @note 可能更新失败标志、投放中止标志、舵机异步请求与任务状态；回调返回不代表收回动作完成。
   */
  void state_callback(const mavros_msgs::msg::State::SharedPtr message)
  {
    const bool was_guided = guided_active_;
    const bool was_armed = fcu_state_.armed;
    fcu_state_ = *message;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";

    if (guided_active_ && !was_guided) {
      RCLCPP_INFO(
        get_logger(),
        measurement_time_sync_ ? "GUIDED detected" :
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

    // 主动请求降落后，飞控进入 LAND 属于预期模式切换；仍处于 GUIDED 也允许继续。
    // 若此时已解锁且失联或进入其他模式，就交还控制权，避免后续继续请求降落/上锁。
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

  /**
   * @brief 判断是否允许把飞控消息头解释为已对齐的测量时间。
   * @return 关闭测量时间同步时直接为 true；开启时要求配置有效、累计合格样本超过
   * MAVROS 收敛窗口及附加裕量，并且最近合格交换的单调时钟年龄小于 1 秒。
   * @note 只读取同步门控状态，不主动同步时钟；true 不是对任意外部时间戳的有效性保证。
   */
  bool flight_clock_ready() const
  {
    return !measurement_time_sync_ ||
      (timesync_config_valid_ && timesync_good_samples_ > timesync_window_ + 10U &&
      steady_age_s(timesync_arrival_) < 1.0);
  }

  /**
   * @brief 观察 MAVROS 时间同步交换，维护采样时间可用门控与时钟纪元连续性。
   * @param message 含远端纳秒时间、观测/估计偏移及毫秒往返时间的同步状态。
   * @note 首次服务就绪时异步读取 MAVROS 同步配置；合格交换累计计数，并用单调时钟
   * 记录到达新鲜度。远端时间回退、长间断或偏移跃变可清空导航/姿态历史，避免跨纪元插值。
   * @note 本函数不修改 ROS 系统时钟，也不把状态中的估计偏移直接加到业务消息上。
   */
  void timesync_callback(const mavros_msgs::msg::TimesyncStatus::SharedPtr message)
  {
    // MAVROS 在把同步结果用于消息头之前就可能发布偏移估计，因此不能只看偏移非零。
    // 读取它实际使用的收敛窗口，并额外观察合格交换，区分收敛前的到达时间回退值与测量时间。
    if (!timesync_config_requested_ && timesync_parameters_->service_is_ready()) {
      timesync_config_requested_ = true;
      timesync_parameters_->get_parameters(
        {"convergence_window", "timesync_mode", "timesync_rate", "max_rtt_sample",
          "max_deviation_sample"},
        [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
          try {
            const auto p = future.get();
            timesync_config_valid_ = p.size() == 5U && p[0].as_int() > 0 &&
              p[0].as_int() <= 2000 && p[1].as_string() == "MAVLINK" &&
              p[2].as_double() > 0.0 && p[3].as_int() > 0 && p[4].as_int() > 0;
            if (timesync_config_valid_) {
              timesync_window_ = static_cast<std::size_t>(p[0].as_int());
              timesync_max_rtt_ms_ = std::min(10.0, static_cast<double>(p[3].as_int()));
              timesync_max_deviation_ns_ = std::min(10000000.0,
                static_cast<double>(p[4].as_int()) * 1e6);
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "FCU_TIME configuration read failed: %s", error.what());
          }
        });
    }
    const double deviation = std::abs(static_cast<double>(message->observed_offset_ns) -
      static_cast<double>(message->estimated_offset_ns));
    const bool good = timesync_config_valid_ && message->remote_timestamp_ns > 0 &&
      message->estimated_offset_ns > 0 && std::isfinite(message->round_trip_time_ms) &&
      message->round_trip_time_ms >= 0 && message->round_trip_time_ms < timesync_max_rtt_ms_ &&
      deviation < timesync_max_deviation_ns_;
    const bool epoch_change = timesync_good_samples_ > 0U &&
      (message->remote_timestamp_ns <= timesync_remote_ns_ ||
      steady_age_s(timesync_arrival_) > 1.0 ||
      std::abs(static_cast<double>(message->estimated_offset_ns) -
        static_cast<double>(timesync_offset_ns_)) > 20000000.0);
    if (!good || epoch_change) {
      // 单次往返时间不合格并不等于 MAVROS 滤波器重启；仅在纪元变化或偏移残差越界时
      // 清除本节点的累计证据，避免偶发 RTT 抖动使同步门控反复从头收敛。
      if (epoch_change || deviation >= timesync_max_deviation_ns_) {
        timesync_good_samples_ = 0U;
        navigation_history_.clear();
        attitude_history_.clear();
      }
    } else {
      ++timesync_good_samples_;
      timesync_arrival_ = SteadyClock::now();
    }
    timesync_remote_ns_ = message->remote_timestamp_ns;
    timesync_offset_ns_ = message->estimated_offset_ns;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "FCU_TIME ready=%s accepted=%zu required=%zu rtt=%.3fms residual=%.3fms",
      flight_clock_ready() ? "yes" : "no", timesync_good_samples_, timesync_window_ + 11U,
      static_cast<double>(message->round_trip_time_ms), deviation / 1e6);
  }

  /**
   * @brief 缓存可用于视觉采集时刻插值的 IMU 姿态，而不积分原始角速度。
   * @param message MAVROS IMU 消息；使用消息头的 ROS 时间及其姿态四元数约定。
   * @note 仅接收时钟门控已就绪、姿态协方差标志有效、时间在允许窗口内且严格递增、
   * 四元数模长接近 1 的样本；归一化后保存弧度制滚转/俯仰/偏航，并裁剪历史时间窗。
   * @note 被拒绝时不更新姿态历史；此入口不更新 position_ 或当前里程计姿态缓存。
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (!flight_clock_ready() || message->orientation_covariance[0] < 0) {return;}
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || age < -0.02 || age > 0.25 ||
      (!attitude_history_.empty() && stamp <= attitude_history_.back().stamp)) {return;}
    const auto & q = message->orientation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.05) {return;}
    const double x = q.x/norm, y = q.y/norm, z = q.z/norm, w = q.w/norm;
    attitude_history_.push_back(NavigationSample{stamp, Point3{},
      std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
      std::asin(std::clamp(2*(w*y-z*x), -1.0, 1.0)),
      std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))});
    while (!attitude_history_.empty() &&
      (stamp - attitude_history_.front().stamp).seconds() > odom_history_s_) {
      attitude_history_.pop_front();
    }
  }

  /**
   * @brief 更新当前运动状态，并为视觉投影积累按时间排序的导航历史。
   * @param message MAVROS 里程计；位置按本地 ENU 约定使用，姿态转换为弧度欧拉角。
   * @note 同步模式使用消息头测量时间并先拒绝零、过期、超前或乱序样本；兼容模式使用
   * 本节点接收 ROS 时间。到达间隔与静止持续时间另用单调时钟统计，避免混用两种时间域。
   * @note 更新位置、姿态、速度、计数和到达时刻；只有时钟门控就绪才追加导航历史并维护
   * 静止门控。twist 分量直接取自消息，本函数不转换其坐标系，也不全面校验所有位姿分量。
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const rclcpp::Time receipt_time = now();
    const SteadyTimePoint receipt_arrival = SteadyClock::now();
    const rclcpp::Time measurement_stamp = measurement_time_sync_ ?
      rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()) : receipt_time;
    const double measurement_age = (receipt_time - measurement_stamp).seconds();
    if (measurement_time_sync_ && (measurement_stamp.nanoseconds() <= 0 ||
      measurement_age < -0.02 || measurement_age > 0.25 ||
      (!navigation_history_.empty() && measurement_stamp <= navigation_history_.back().stamp)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "ODOM_TIME_REJECT age=%.3fs (zero/stale/future/out-of-order)", measurement_age);
      return;
    }
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

    // 当前状态缓存仍供监控使用，但未收敛的时间不能进入采集时刻插值历史。
    if (!flight_clock_ready()) {return;}

    if (!navigation_history_.empty()) {
      const double delta_s =
        (measurement_stamp - navigation_history_.back().stamp).seconds();
      if (delta_s < -0.5) {
        navigation_history_.clear();
      } else if (delta_s <= 0.0) {
        return;
      }
    }
    navigation_history_.push_back(
      NavigationSample{
        measurement_stamp, sample_position, sample_roll, sample_pitch, sample_yaw});
    while (!navigation_history_.empty() &&
      (measurement_stamp - navigation_history_.front().stamp).seconds() >
      odom_history_s_)
    {
      navigation_history_.pop_front();
    }

    if (!frame_locked_ || (measurement_time_sync_ && !mission_started_)) {
      if (horizontal_speed_m_s_ <= stationary_speed_m_s_) {
        if (!position_stable_since_.has_value()) {
          position_stable_since_ = receipt_arrival;
        }
      } else {
        position_stable_since_.reset();
      }
    }
  }

  /**
   * @brief 保存最新本地位姿消息及其接收连续性统计。
   * @param message MAVROS 本地位姿；直接缓存 pose，沿用上游坐标约定。
   * @note 消息头不参与本入口的新鲜度判定，年龄与最大间隔使用单调到达时间；
   * 本函数不向采集时刻导航历史追加样本，也不替代 odom_callback 的运动状态更新。
   */
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

  /**
   * @brief 缓存飞控扩展状态，供后续降落与着地条件判断。
   * @param message MAVROS 扩展状态消息；本入口不把接收一次状态等同于确认已经着地。
   * @note 更新状态、已接收标志和单调时钟到达时间；具体枚举值及新鲜度由使用方检查。
   */
  void extended_state_callback(
    const mavros_msgs::msg::ExtendedState::SharedPtr message)
  {
    extended_state_ = *message;
    have_extended_state_ = true;
    last_extended_state_arrival_ = SteadyClock::now();
  }

  /**
   * @brief 将罗盘航向转换为 ENU 偏航并维护锁航向所需的时间窗口。
   * @param message 以度表示的地理航向：北为 0 度、顺时针增大；非有限值直接忽略。
   * @note 输出偏航以东为零、逆时针为正，单位弧度；消息无采样时间戳，窗口按单调
   * 接收时间建立。更新当前航向、接收标志及样本队列，不在此处直接锁定任务坐标系。
   */
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

  /**
   * @brief 获取指定 ROS 测量时刻的导航样本，按模式组合位置与姿态来源。
   * @param stamp 与历史样本属于同一 ROS 时钟域的查询时刻，通常为视觉采集时间。
   * @return 兼容模式返回里程计历史结果；同步模式还要求同刻 IMU 姿态及同步门控有效，
   * 再用该姿态替换里程计姿态。任何必需历史不可用时返回 std::nullopt，不外推补造样本。
   * @note 只读取历史，不改变当前飞行状态或历史队列。
   */
  std::optional<NavigationSample> navigation_sample_at(
    const rclcpp::Time & stamp) const
  {
    auto position = interpolate_history(navigation_history_, stamp);
    if (!measurement_time_sync_ || !position) {return position;}
    const auto attitude = interpolate_history(attitude_history_, stamp);
    if (!attitude || !flight_clock_ready()) {return std::nullopt;}
    position->roll = attitude->roll;
    position->pitch = attitude->pitch;
    position->yaw = attitude->yaw;
    return position;
  }

  /**
   * @brief 在严格递增的导航历史内部查询或插值一个样本。
   * @param history 同一 ROS 时间域的样本队列；至少需两条，即使命中某条时间也不例外。
   * @param stamp 查询时刻；不允许早于首样本或晚于末样本的外推。
   * @return 精确命中时返回原样本；否则仅在相邻间隔为正且不超过配置上限时线性插值
   * 位置，并按归一化最短角差逐项插值欧拉角。历史不足或间隔不适用时返回 std::nullopt。
   * @note 欧拉角逐项插值不是四元数球面插值；本函数不修改输入历史，也不保证任意大姿态变化的模型精度。
   */
  std::optional<NavigationSample> interpolate_history(
    const std::deque<NavigationSample> & history, const rclcpp::Time & stamp) const
  {
    if (history.size() < 2U) {
      return std::nullopt;
    }
    const auto upper = std::lower_bound(
      history.begin(), history.end(), stamp,
      [](const NavigationSample & sample, const rclcpp::Time & value) {
        return sample.stamp < value;
      });
    if (upper == history.begin()) {
      return upper->stamp == stamp ?
             std::optional<NavigationSample>(*upper) : std::nullopt;
    }
    if (upper == history.end()) {
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

  /**
   * @brief 读取本轮投放是否已越过不再由视觉重定向的提交边界。
   * @return 投放执行提交标志；它用于约束后续导航，不表示异步 ACK、机构收回或物理投放已完成。
   * @note 无副作用，具体提交时机由投放状态流程设置。
   */
  bool release_command_committed() const
  {
    return release_actuation_committed_;
  }

  /**
   * @brief 判断当前任务阶段是否允许视觉结果参与目标引导。
   * @return SEARCH、ALIGN，或尚未提交投放执行的 RELEASE 阶段返回 true。
   * @note 只检查阶段与提交标志，不检查视觉心跳、采样时间或目标质量，也不改变任务状态。
   */
  bool vision_guidance_active() const
  {
    return state_ == State::SEARCH || state_ == State::ALIGN ||
      (state_ == State::RELEASE && !release_command_committed());
  }

  /**
   * @brief 按当前载荷槽位，从本帧检测或历史轨迹更新活动目标。
   * @return 计划无效或槽位已投放时为 false；true 也可能表示固定坐标模式跳过视觉，
   * 或按策略继续保留历史目标，不等于获得了一帧新鲜观测。
   * @note 优先最近邻重关联；其后才尝试同 ID 的已知轨迹。可能同时更新活动目标、已知轨迹
   * 和计划坐标/直径/置信度；普通同 ID 刷新保留原 frozen_memory 标志。
   * @pre 常规对准/释放调用路径已建立活动目标；普通同 ID 替换分支会读取其原冻结标志。
   */
  bool refresh_active_target_from_current_track()
  {
    if (!target_plan_locked_ || payload_index_ >= selected_target_ids_.size() ||
      released_target(selected_target_ids_[payload_index_]))
    {
      return false;
    }
    const std::size_t selected_id = selected_target_ids_[payload_index_];
    // 固定坐标测试仍接收相机消息用于诊断，但不允许检测结果替换事先设定的坐标。
    if (fixed_release_test_mode_ || fixed_release_direct_active_) {
      return true;
    }
    // 视觉中断后重建的轨迹可能换 ID，先按当前帧最近邻关联，保持任务槽位使用的逻辑 ID。
    if (reacquire_nearest_active_target(selected_id)) {
      return true;
    }
    if (cluster_fit_plan_active_) {
      // 只有当前帧最近邻关联失败后，聚类计划才退回同 ID 的历史轨迹；并非禁止使用原始检测。
      const auto current = std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [selected_id](const BucketTrack & track) { return track.id == selected_id; });
      if (current != known_buckets_.end()) {
        active_bucket_ = *current;
        return true;
      }
      // 没有同 ID 轨迹时仍可保留未投放目标的最后坐标；这条退路没有重新检查观测年龄。
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

  /**
   * @brief 从最近已处理帧中选取离当前计划参考点最近的合格检测，重建活动目标。
   * @param selected_id 当前载荷槽位保留的任务逻辑 ID，不直接采用检测重新分配的轨迹 ID。
   * @return 有有限坐标、合理直径、足够置信度且位于扩展投放区的候选时为 true；否则为 false。
   * @note 参考位置优先使用计划冻结位置，距离按本地 ENU 的 XY 平面计算；此处没有额外
   * 最近邻距离上限，也不检查观测年龄，不能仅凭返回值证明物理身份或采样新鲜度。
   * @note 成功时替换活动目标及可找到的同 ID 轨迹，刷新计划的坐标、直径和置信度并取消冻结标志。
   */
  bool reacquire_nearest_active_target(std::size_t selected_id)
  {
    // 新检测 ID 可能因视觉中断而重建；保留槽位 ID 是维持计划索引的约定，
    // 最近邻本身没有额外关联半径，不能作为两只物理目标绝不会混淆的保证。
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

  /**
   * @brief 计算活动目标对应视觉帧自接收以来的单调时钟年龄。
   * @return 秒数；没有活动目标时返回正无穷。该年龄不含消息到达本机之前的采集与传输延迟。
   * @note 只作读取，不刷新目标到达时刻，也不据此改变引导策略。
   */
  double active_target_observation_age_s() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return steady_age_s(active_bucket_->arrival);
  }

  /**
   * @brief 按当前“允许沿用历史坐标”的策略判断活动目标是否仍可被引用。
   * @return 目标存在且其逻辑 ID 未在已投放集合中时为 true；不应用年龄阈值。
   * @note 函数名虽含 fresh，但本实现不证明观测新鲜；观测年龄另由诊断函数保留，无副作用。
   */
  bool active_target_observation_fresh() const
  {
    // 对准/释放过程中允许继续沿用最后目标坐标；年龄仅供诊断，这里不因视觉中断而判失效。
    return active_bucket_.has_value() && !released_target(active_bucket_->id);
  }

  /**
   * @brief 接收视觉检测消息，验证采集时间连续性并排队等待同刻导航样本。
   * @param message 自定义编码的 PoseArray：position 为中心的机体 FRD 米制坐标；
   * orientation.x/y 为直径/置信度，orientation.z/w 为桶沿的 FRD x/y 分量，桶沿 z 复用中心 z。
   * 这些 orientation 字段不是姿态四元数，header.stamp 必须已映射到本节点 ROS 时间域。
   * @note 所有消息先维护单调到达心跳；已提交投放或全部载荷完成后不再排队或改目标。
   * 拒绝零、过期、超前与不递增采集时间；较大回退会清除关联历史/计划，仅在允许的首轮
   * SEARCH 重基准路径恢复采集，否则触发返航。有效帧入队后立即尝试消费导航已就绪的帧。
   */
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
    // 投放一旦提交，就不能让视觉回调在应答/收回清理期间重定向 LAND 或人工接管流程。
    // 全部载荷完成后按航点执行侦察，不再依赖桶检测；此时仍保留上面的到达心跳统计。
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
            // 回退前已对时的帧不能充当重新采集后的新帧证据；重置序列基线，
            // 但不在这里延长已经运行的单调时钟等待期限或悬停过程。
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
      // 当前兼容策略将非有限或非正置信度回退到 1.0，而不是按无效检测丢弃。
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

  /**
   * @brief 按队列顺序等待并消费能取得采集时刻导航的视觉帧。
   * @note 查询使用帧的 ROS 采集时间，等待上限使用单调到达年龄；队首导航尚不可用且
   * 未到期限时停止消费，避免越过旧帧。到期后即使导航缺失也出队，交下层记录拒绝原因。
   * @note 投放已提交或载荷完成时直接清空待处理队列；本入口可能间接更新视觉计数与目标轨迹。
   */
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

  /**
   * @brief 将一帧检测按采集时刻位姿投影到本地坐标，并融合成目标轨迹。
   * @param frame 含 ROS 采集时间、单调接收时间及机体 FRD 检测的排队帧。
   * @param navigation 同一采集时刻的导航样本；为空时只记录投影失败，不更新已对时帧心跳。
   * @note 有导航即更新已对时帧计数/序列，包括空检测帧；只有坐标系已锁且允许视觉引导
   * 才继续处理目标。先转 FRD 为 FLU，再投影为本地 ENU，独立检查直径、置信度及区域。
   * @note 更新本帧候选缓存、已知轨迹及可选搜索观测集；对准/未提交释放阶段可进一步刷新活动目标。
   */
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
    if (measurement_time_sync_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "VISION_TIME capture_ns=%ld use_age_ms=%.3f buffer_ms=%.3f",
        static_cast<long>(frame.stamp.nanoseconds()), (now() - frame.stamp).seconds() * 1000.0,
        steady_age_s(frame.arrival) * 1000.0);
    }
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
      // 视觉输入为飞控机体 FRD（前/右/下），内部投影使用 FLU（前/左/上）。
      // 只在边界翻转 y、z，避免把 FRD 向量直接代入 FLU 到 ENU 的旋转。
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
      // 当前主路径直接采用采集时刻位姿的几何投影，不叠加路线或高度相关的经验坐标修正。
      const Point3 corrected_local = local;
      BucketTrack current_frame_detection;
      current_frame_detection.local = corrected_local;
      current_frame_detection.body = body;
      current_frame_detection.rim_body = rim_body;
      current_frame_detection.capture_navigation = navigation;
      current_frame_detection.diameter = std::isfinite(diameter) ? diameter : 0.0;
      current_frame_detection.confidence = confidence;
      current_frame_detection.stamp = frame.stamp;
      current_frame_detection.arrival = frame.arrival;
      // 本帧缓存保留基础坐标有效的检测，后续最近邻使用时仍需重做直径、置信度和区域筛选。
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
      detection.capture_navigation = navigation;
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

  /**
   * @brief 按配置打开投影诊断 CSV，并写入固定列头。
   * @note 路径为空时禁用；以输出模式打开文件，会覆盖同名文件原内容。打开失败仅记录错误，
   * 不阻止任务继续。列头区分原始 FRD 检测、采集位姿、本地 ENU 与任务场地坐标。
   * @note 修改文件流并立即刷新；文件打开成功不代表后续每次写入均成功。
   */
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

  /**
   * @brief 将一次检测的投影结果或拒绝原因追加到诊断文件。
   * @param stamp ROS 时间域的视觉采集时刻，按纳秒写入。
   * @param detection_index 检测在该帧中的索引，不是持续跟踪 ID。
   * @param body 原始机体 FRD 米制检测，保持与 CSV 列头一致。
   * @param navigation 同刻导航指针；与 local 任一为空时，所有投影相关列统一写 nan。
   * @param local 已投影的本地 ENU 米制位置；有效时再换算为锁航向任务场地坐标。
   * @param status 由调用方给出的诊断状态文本，不在本函数中推断成功或失败。
   * @note 未打开文件时无操作；每条记录立即刷新流，不修改目标或任务状态。
   */
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

  /**
   * @brief 把机体 FLU 相对位置转换为本地 ENU 点坐标。
   * @param body 相对载体原点的前/左/上米制向量；原始 FRD 必须已在输入边界换轴。
   * @param navigation 对应该观测时刻的位置与弧度姿态；按 Rz(yaw)Ry(pitch)Rx(roll) 旋转后平移。
   * @return 本地 ENU 米制坐标；同步模式使用样本 yaw，兼容模式使用锁定 mission_yaw_。
   * @note 不补偿相机外参、不检查时间和分量有限性，也不修改成员；这些条件由上游保证。
   */
  Point3 body_to_local(
    const Point3 & body, const NavigationSample & navigation) const
  {
    const double cr = std::cos(navigation.roll);
    const double sr = std::sin(navigation.roll);
    const double cp = std::cos(navigation.pitch);
    const double sp = std::sin(navigation.pitch);
    const double yaw = measurement_time_sync_ ? navigation.yaw : mission_yaw_;
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
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

  /**
   * @brief 将本地 ENU 点按当前载体状态反变换为机体 FLU 相对位置。
   * @param local 与 position_ 具有相同原点和坐标轴的米制位置。
   * @return 先减当前平移，再按逆偏航、逆俯仰、逆滚转变换所得前/左/上坐标。
   * @note 同步模式使用当前里程计 yaw，兼容模式使用锁定任务 yaw；本函数不是按过去
   * 采集时刻计算的逆投影，亦不输出 FRD。只读当前状态，不验证状态新鲜度。
   */
  Point3 local_to_body_current(const Point3 & local) const
  {
    const double dx = local.x - position_.x;
    const double dy = local.y - position_.y;
    const double dz = local.z - position_.z;
    const double yaw = measurement_time_sync_ ? current_vehicle_yaw_ : mission_yaw_;
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
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

  /**
   * @brief 把锁定任务场地中的点转换为 MAVROS 本地 ENU 点。
   * @param field_x 相对 home 的任务前向米数。
   * @param field_y 相对 home 的任务左向米数。
   * @param relative_z 相对 home 的向上高度，单位米，不是绝对海拔。
   * @return 使用 mission_yaw_ 旋转 XY 并加 home 平移后的本地坐标。
   * @note home 未建立时以零点代替；实际任务几何要求调用方已通过锁坐标门控。无状态副作用。
   */
  Point3 field_to_local(
    double field_x, double field_y, double relative_z) const
  {
    // 任务场地轴按 home 处锁定的前/左/上方向建立；MAVROS 设定点仍使用本地 ENU，
    // 因而场地几何在这里统一旋转和平移，避免各航点自行重复换轴。
    const Point3 origin = home_.value_or(Point3{});
    const double cosine = std::cos(mission_yaw_);
    const double sine = std::sin(mission_yaw_);
    return Point3{
      origin.x + cosine * field_x - sine * field_y,
      origin.y + sine * field_x + cosine * field_y,
      origin.z + relative_z};
  }

  /**
   * @brief 将 MAVROS 本地 ENU 点转换为 home 原点的锁航向场地坐标。
   * @param local 本地米制位置。
   * @return 减 home 后按 mission_yaw_ 的逆旋转计算前/左/上坐标，与 field_to_local 成对使用。
   * @note home 缺失时使用零原点，不在此检查坐标框架是否已经锁定；无时间查询或成员修改。
   */
  Point3 local_to_field(const Point3 & local) const
  {
    // 先去掉 home 平移再逆旋转，与 field_to_local 使用完全相同的原点和锁定航向。
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

  /**
   * @brief 判断位置的水平投影是否位于可扩展的任务投放矩形内。
   * @param local 本地 ENU 米制点，内部先转换到任务场地坐标。
   * @param extra 米制边界裕量；正值向四边扩张，负值相应收缩。
   * @return 场地 x 与 y 同时满足闭区间条件时为 true；不检查 z、高度或观测时间。
   * @note 依赖当前 home、mission_yaw_ 和区域参数，仅作几何判断，无副作用。
   */
  bool inside_drop_area(const Point3 & local, double extra) const
  {
    const Point3 field = local_to_field(local);
    return field.x >= drop_near_x_m_ - extra &&
      field.x <= drop_near_x_m_ + drop_length_x_m_ + extra &&
      std::abs(field.y) <= drop_width_y_m_ * 0.5 + extra;
  }

  /**
   * @brief 查询任务逻辑 ID 是否已被登记为投放完成目标。
   * @param id 本节点分配并用于计划槽位的目标 ID，不是视觉框的帧内索引。
   * @return ID 存在于已投放集合时为 true；仅反映记录，不独立验证物理结果或坐标身份。
   * @note 不修改记录，不按时间自动过期。
   */
  bool released_target(std::size_t id) const
  {
    return std::find(
      released_target_ids_.begin(), released_target_ids_.end(), id) !=
      released_target_ids_.end();
  }

  /**
   * @brief 将一条已关联检测融合进已有目标，维护多帧稳健位置、尺寸与连续帧证据。
   * @param track 原地更新的已知轨迹；其逻辑 ID 在正常融合与间断重建时均保留。
   * @param detection 已完成 FRD 到 FLU/本地 ENU 换算的单帧检测，含 ROS 采集时间与单调到达时间。
   * @note 时间倒退或间隔过大时用检测重建轨迹；正常情况用各坐标窗口中位数估计位置，
   * 用三维径向距离的中位数表示位置离散度，用直径样本 MAD 表示尺寸离散度。
   * @note 更新机体系滤波量、确认数、帧连续数及时间；同步模式额外保持桶沿与采集导航来自同帧。
   */
  void smooth_track(BucketTrack & track, const BucketTrack & detection)
  {
    const double gap_s = (detection.stamp - track.stamp).seconds();
    if (gap_s < 0.0 || gap_s > track_max_gap_s_) {
      // 关联身份可延续，但跨越倒序或长间断的样本不能继续积累原窗口的稳定性证据。
      const std::size_t id = track.id;
      track = detection;
      track.id = id;
      track.position_samples.clear();
      track.position_samples.push_back(track.local);
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
    track.position_samples.push_back(detection.local);
    while (track.position_samples.size() > position_filter_window_) {
      track.position_samples.pop_front();
    }
    std::deque<double> xs;
    std::deque<double> ys;
    std::deque<double> zs;
    for (const Point3 & sample : track.position_samples) {
      xs.push_back(sample.x);
      ys.push_back(sample.y);
      zs.push_back(sample.z);
    }
    const Point3 robust_position{median(xs), median(ys), median(zs)};
    std::deque<double> radial_deviations;
    for (const Point3 & sample : track.position_samples) {
      radial_deviations.push_back(distance_xyz(sample, robust_position));
    }
    // 最终门控采用窗口的径向中位离散度；上面的指数残差更新在此被该稳健统计值覆盖。
    track.position_deviation = median(radial_deviations);
    track.local.x =
      robust_position.x;
    track.local.y =
      robust_position.y;
    track.local.z =
      robust_position.z;
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
    if (measurement_time_sync_) {
      // 桶沿用于后续姿态相关对准，必须与 capture_navigation 保持同一采集帧。
      // 这里只覆盖桶沿和采集导航；中心 body 仍保留前面的指数平滑结果。
      track.rim_body = detection.rim_body;
      track.capture_navigation = detection.capture_navigation;
    }
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

  /**
   * @brief 将一帧合格检测按位置与直径联合距离贪心关联到已知轨迹。
   * @param detections 本帧已通过上游元数据及区域筛选的检测，位置单位米且属于本地 ENU。
   * @note 先按单调到达年龄删除未选中且未登记投放的过旧轨迹；被计划引用或已投放的轨迹保留。
   * 在位置/直径门限内按归一化代价排序，一条轨迹与一条检测在本轮各最多匹配一次，
   * 不执行全局最优分配。匹配项调用 smooth_track，剩余检测分配新 ID 并追加。
   * @pre 关联门限为正，检测与已知轨迹使用相同锁定坐标定义；函数会修改轨迹库和 ID 计数器。
   */
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
      track.position_samples.push_back(track.local);
      track.id = next_bucket_id_++;
      known_buckets_.push_back(std::move(track));
    }
  }

  /**
   * @brief 检查目标是否达到通用多帧选择门限。
   * @param track 含确认次数、置信度、米制位置/直径离散度及单调接收时刻的轨迹。
   * @return 确认数、接收年龄、置信度和离散度均达标，且 ID 未登记投放时为 true。
   * @note 不检查区域、直径类别、连续帧数或采样端到端年龄；这些条件不由本函数代替。无副作用。
   */
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

  /**
   * @brief 计算搜索选靶排序使用的启发式质量分数。
   * @param track 上游已筛选的轨迹；确认数贡献在 12 次封顶，置信度加分，米制离散度扣分。
   * @return 越大越优的排序分值，不是概率、物理误差上界或可直接用于释放的有效性判据。
   * @note 只读取质量统计，不修改轨迹或覆盖后续对准所需的观测。
   */
  double search_candidate_score(const BucketTrack & track) const
  {
    // 质量分只用于搜索计划排序，不在此替换精对准使用的中心/桶沿观测，也不声明其新鲜度。
    return 0.45 * std::min<double>(static_cast<double>(track.confirmations), 12.0) +
      4.0 * track.confidence - 6.0 * track.position_deviation -
      3.0 * track.diameter_deviation;
  }

  /**
   * @brief 在通用轨迹门限之上施加搜索阶段更严格的确认数与位置稳定性要求。
   * @param track 待选目标轨迹，与 track_ready 使用相同时间和单位约定。
   * @return 通用门限及搜索专用两项门限同时满足时为 true；不改变轨迹或计划。
   */
  bool search_track_ready(const BucketTrack & track) const
  {
    return track_ready(track) &&
      track.confirmations >= static_cast<std::size_t>(search_min_confirmations_) &&
      track.position_deviation <= search_max_position_deviation_m_;
  }

  /**
   * @brief 将米制估计直径归入前三个配置中心中误差最小且在容差内的类别。
   * @param track 使用其已滤波 diameter，不读取检测原始像素尺寸。
   * @return 0/1/2 对应小/中/大类别，配置中心不足三个或没有匹配时返回 -1。
   * @note 等误差时后遍历的类别覆盖前者；此函数不检查轨迹质量、新鲜度或投放状态，无副作用。
   */
  int bucket_diameter_class(const BucketTrack & track) const
  {
    if (bucket_diameter_centers_m_.size() < 3U) {
      return -1;
    }
    int best_class = -1;
    double best_error = bucket_diameter_class_tolerance_m_;
    for (std::size_t index = 0U; index < 3U; ++index) {
      const double error = std::abs(
        track.diameter - bucket_diameter_centers_m_[index]);
      if (error <= best_error) {
        best_error = error;
        best_class = static_cast<int>(index);
      }
    }
    return best_class;
  }

  /**
   * @brief 将直径类别索引映射为固定诊断文本。
   * @param class_index 0/1/2 分别对应小/中/大，其余值均按未知类别处理。
   * @return 指向只读字符串字面量的指针；无需释放，函数不修改任何状态。
   */
  const char * bucket_diameter_class_name(int class_index) const
  {
    switch (class_index) {
      case 0: return "SMALL";
      case 1: return "MEDIUM";
      case 2: return "LARGE";
      default: return "UNKNOWN";
    }
  }

  /**
   * @brief 从搜索质量达标且空间独立的已知轨迹中冻结双载荷目标计划。
   * @param allow_two 允许按小/中直径类别提前成对锁定；快速类别未命中时，普通分支所需
   * 独立目标数由默认三个降为两个，但最终仍只选择两个目标。
   * @param fast_only 仅允许快速类别路径；未锁定时禁止退入一般直径排序分支。
   * @return 已有锁定计划或本次成功锁定时为 true；候选证据不足时为 false。
   * @note 先按质量分筛空间独立性，再按直径选较小的两个，最后按锁定瞬间到飞机的 XY
   * 距离安排先后；快速类别路径也按距离排序。写入并冻结 ID/坐标/直径/置信度数组，不直接切换任务阶段。
   * @pre 调用路径已通过配置门控，payload_count_ 为 2；已有计划直接返回，不重新验证其年龄或质量。
   */
  bool try_lock_target_plan(bool allow_two = false, bool fast_only = false)
  {
    if (target_plan_locked_) {
      return true;
    }
    std::vector<const BucketTrack *> reliable;
    for (const BucketTrack & track : known_buckets_) {
      if (search_track_ready(track)) {
        reliable.push_back(&track);
      }
    }
    std::sort(
      reliable.begin(), reliable.end(),
      [this](const BucketTrack * lhs, const BucketTrack * rhs) {
        return search_candidate_score(*lhs) > search_candidate_score(*rhs);
      });

    // 快速路径识别配置中的小/中直径类别，而非写死某一组直径数值。
    // 每类先取质量排序最高者；成对独立性不满足时，是否继续一般排序由 fast_only 决定。
    if (allow_two) {
      const BucketTrack * small = nullptr;
      const BucketTrack * medium = nullptr;
      for (const BucketTrack * candidate : reliable) {
        const int class_index = bucket_diameter_class(*candidate);
        if (class_index == 0 && small == nullptr) {
          small = candidate;
        } else if (class_index == 1 && medium == nullptr) {
          medium = candidate;
        }
      }
      if (small != nullptr && medium != nullptr &&
        distance_xy(small->local, medium->local) >= distinct_min_separation_m_)
      {
        // 此选靶路径按锁定瞬间的水平距离安排投放先后，而不是固定先投小类别。
        const BucketTrack * first = small;
        const BucketTrack * second = medium;
        if (distance_xy(medium->local, position_) <
          distance_xy(small->local, position_))
        {
          first = medium;
          second = small;
        }
        selected_target_ids_ = {first->id, second->id};
        selected_target_positions_ = {first->local, second->local};
        selected_target_diameters_ = {first->diameter, second->diameter};
        selected_target_confidences_ = {first->confidence, second->confidence};
        target_plan_locked_ = true;
        RCLCPP_INFO(
          get_logger(),
          "DROP_TARGETS_FAST_CLASS_LOCKED first_id=%zu diameter=%.3f class=%s "
          "second_id=%zu diameter=%.3f class=%s tolerance=%.3f",
          first->id, first->diameter, bucket_diameter_class_name(bucket_diameter_class(*first)),
          second->id, second->diameter, bucket_diameter_class_name(bucket_diameter_class(*second)),
          bucket_diameter_class_tolerance_m_);
        return true;
      }
    }

    // 搜索过程要求仅走快速类别路径时，类别不足就继续观察；不能退化成任意两条稳定轨迹。
    if (fast_only) {
      return false;
    }

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
    // 已经按直径留下较小两只后，再用当前距离排顺序；这一步不改变入选集合。
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
    const auto selected_track = [this](std::size_t id) -> const BucketTrack * {
      const auto it = std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [id](const BucketTrack & track) {return track.id == id;});
      return it == known_buckets_.end() ? nullptr : &*it;
    };
    const BucketTrack * first_selected = selected_track(selected_target_ids_[0]);
    const BucketTrack * second_selected = selected_track(selected_target_ids_[1]);
    RCLCPP_INFO(
      get_logger(),
      "DROP_TARGETS_LOCKED first_id=%zu second_id=%zu diameters=%.3f/%.3f classes=%s/%s",
      selected_target_ids_[0], selected_target_ids_[1],
      selected_target_diameters_[0], selected_target_diameters_[1],
      bucket_diameter_class_name(bucket_diameter_class(*std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [this](const BucketTrack & track) {
          return track.id == selected_target_ids_[0];
        }))),
      bucket_diameter_class_name(bucket_diameter_class(*std::find_if(
        known_buckets_.begin(), known_buckets_.end(),
        [this](const BucketTrack & track) {
          return track.id == selected_target_ids_[1];
        }))));
    RCLCPP_INFO(
      get_logger(),
      "SEARCH_LOCK_FILTER first_samples=%zu first_conf=%.3f first_pos_dev=%.3fm "
      "second_samples=%zu second_conf=%.3f second_pos_dev=%.3fm window=%zu",
      first_selected == nullptr ? 0U : first_selected->confirmations,
      first_selected == nullptr ? 0.0 : first_selected->confidence,
      first_selected == nullptr ? 0.0 : first_selected->position_deviation,
      second_selected == nullptr ? 0U : second_selected->confirmations,
      second_selected == nullptr ? 0.0 : second_selected->confidence,
      second_selected == nullptr ? 0.0 : second_selected->position_deviation,
      position_filter_window_);
    return true;
  }

  /**
   * @brief 对搜索期间累积的全部合格观测做固定三簇拟合，并冻结聚类投放计划。
   * @return 已有计划或本次得到足够空间独立簇并建齐载荷目标时为 true；观测/独立簇不足时为 false。
   * @note 以三个配置直径中心选种子，用 XY 与直径的归一化距离分配，按置信度加权更新中心
   * 共 12 轮；质量评分去重后再按直径降序选择目标，默认两载荷取较大两簇，不按飞机距离重排。
   * @note 为选中簇分配新逻辑 ID，记录最新单调到达时刻及其 ROS 采集时间，并标为历史冻结目标；
   * 修改聚类轨迹与计划数组，成功时设置两个锁定标志。本函数不对旧观测另加新鲜度门限。
   * @pre 前三个直径中心及正的尺度参数可用，载荷数量已通过配置校验；簇数固定为三，不随所需数量变化。
   */
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
    // 各直径中心优先挑一条尚未用作种子的观测，给后续空间/尺寸联合迭代提供初值。
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
    // 固定迭代次数，不以中心位移收敛为停止条件；尺度把米制位置与米制直径变成可比较的代价。
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

    // 分配完成后用未加权残差计算簇内 XY 与直径 RMS；它们与前面加权中心是不同统计步骤。
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

    // 先前已按质量分做空间去重；此处再按直径降序选载荷目标。
    // 因而三簇可用时选择较大两簇，两簇可用时都选，顺序并非“最近优先”或再次按分数排序。
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

  /**
   * @brief 取得当前载荷索引对应的目标副本，必要时先尝试建立普通计划。
   * @return 聚类计划直接返回槽位中的冻结轨迹；普通计划按逻辑 ID 查已知轨迹且要求单调
   * 到达年龄不超过 known_memory_s_。计划或索引无效、轨迹缺失/过旧时返回 std::nullopt。
   * @note 聚类路径不应用普通记忆年龄门限；普通路径可能经 try_lock_target_plan 修改计划。
   * 本函数不单独证明新帧、目标身份门限或投放完成条件。
   */
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

  /**
   * @brief 在允许沿用记忆坐标的流程中读取指定载荷槽位的最后已知目标。
   * @param payload 从零开始的载荷索引，要求计划及各并行元数据数组均包含该槽位。
   * @return 聚类路径返回同 ID 的聚类副本；普通路径返回已知轨迹副本并置 frozen_memory。
   * 无对应轨迹时返回 std::nullopt，不仅凭冻结坐标数组重建一个目标。
   * @note 不检查年龄、已投放标志或通用质量门限；冻结标志表示记忆来源，不表示新观测有效。
   * 只操作返回副本，不修改已知轨迹或计划。
   */
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

  /**
   * @brief 检查候选是否仍匹配当前载荷计划的逻辑身份及适用的几何质量门限。
   * @param target 本地 ENU 米制目标，含逻辑 ID、直径、置信度和离散度。
   * @return 首先要求计划/索引有效、ID 匹配且未登记投放；聚类计划到此即通过，普通计划还
   * 比较相对冻结位置的 XY 偏差、直径偏差、置信度和轨迹离散度。
   * @note 不检查时间、新帧数或类别；逻辑匹配不独立证明同一物理桶，函数无状态副作用。
   */
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

  /**
   * @brief 撤销当前选靶计划及其活动目标、聚类缓存和排序稳定性证据。
   * @note 清除锁定数组与聚类标志，并重置粗对准滤波及固定直接投放的运行标志。
   * 不改变载荷索引、已投放 ID/位置记录、已知轨迹或搜索观测集；调用方需按重置原因另行处理。
   * @note 仅修改计划相关内存，不发送飞控或舵机指令，也不自动切换任务状态。
   */
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

  /** @brief 返回单调时刻距现在的秒数；仅接收同一 steady_clock 时间域的时刻。 */
  static double steady_age_s(const SteadyTimePoint & stamp)
  {
    return std::chrono::duration<double>(SteadyClock::now() - stamp).count();
  }

  /** @brief 里程计接收年龄（秒）；从未收到时返回正无穷，使新鲜度检查自然失败。 */
  double odom_age_s() const
  {
    return have_odom_ ? steady_age_s(last_odom_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 独立位置话题接收年龄（秒）；只用于其自身诊断，不冒充里程计采样时间。 */
  double pose_age_s() const
  {
    return have_pose_ ? steady_age_s(last_pose_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 罗盘接收年龄（秒）；未接收返回正无穷，和航向数值是否为零无关。 */
  double compass_age_s() const
  {
    return have_compass_ ? steady_age_s(last_compass_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 飞控扩展状态接收年龄（秒），供落地状态的新鲜度检查使用。 */
  double extended_state_age_s() const
  {
    return have_extended_state_ ? steady_age_s(last_extended_state_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 最近视觉心跳的接收年龄（秒）；即使空检测帧也可构成心跳。 */
  double vision_message_age_s() const
  {
    return have_vision_message_ ? steady_age_s(last_vision_message_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 最近完成时间对齐的视觉帧接收年龄（秒），不等于某条桶轨迹的年龄。 */
  double aligned_vision_age_s() const
  {
    return have_aligned_vision_ ? steady_age_s(last_aligned_vision_arrival_) :
      std::numeric_limits<double>::infinity();
  }

  /** @brief 时钟门控就绪且里程计接收年龄在门限内；不单独证明其空间测量准确。 */
  bool odom_fresh() const
  {
    if (!flight_clock_ready()) {return false;}
    return odom_age_s() <= odom_timeout_s_;
  }

  /** @brief 罗盘最近一次接收是否未超过配置超时。 */
  bool compass_fresh() const
  {
    return compass_age_s() <= compass_timeout_s_;
  }

  /** @brief 飞控扩展状态最近一次接收是否未超过配置超时。 */
  bool extended_state_fresh() const
  {
    return extended_state_age_s() <= extended_state_timeout_s_;
  }

  /**
   * @brief 新鲜的飞控扩展状态是否报告 ON_GROUND；本函数不检查高度或速度。
   * 完整确认还需持续时间。landing_candidate() 在 ON_GROUND 分支中仅在里程计
   * 新鲜时附加低速要求；只有状态不明确时的回退分支才同时检查低高度。
   */
  bool on_ground_reported() const
  {
    return extended_state_fresh() &&
      extended_state_.landed_state ==
      mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  /** @brief 视觉进程近期是否仍有合格心跳输入；有心跳并不表示检测到了目标。 */
  bool vision_heartbeat_fresh() const
  {
    return vision_message_age_s() <= vision_heartbeat_timeout_s_;
  }

  /** @brief 最近对齐帧的接收年龄是否未超过视觉心跳门限。 */
  bool aligned_vision_fresh() const
  {
    return aligned_vision_age_s() <= vision_heartbeat_timeout_s_;
  }

  /**
   * @brief 检查起飞前视觉心跳数量与新鲜度；同步模式还要求时钟和帧对齐已经就绪。
   * @return 满足准备条件时为 true；不要求在起飞点看到桶，不以目标数量代替链路健康。
   */
  bool vision_ready_before_takeoff() const
  {
    if (measurement_time_sync_ && (!flight_clock_ready() || !aligned_vision_fresh() ||
      vision_message_count_ < static_cast<std::size_t>(vision_min_messages_before_takeoff_))) {
      return false;
    }
    // 空 PoseArray 是按采集帧发送的合法心跳：起飞点附近可以没有桶。
    // 同步模式在进入等待 GUIDED 前也会校验空帧的时间，不以存在检测为必要条件。
    return vision_heartbeat_count_ >=
      static_cast<std::size_t>(vision_min_messages_before_takeoff_) &&
      vision_heartbeat_fresh();
  }

  /** @brief 是否正在 SEARCH 状态内保持位置、等待视觉心跳恢复；其他状态不复用该门控。 */
  bool search_vision_acquisition_pending() const
  {
    return state_ == State::SEARCH && search_vision_acquire_pending_;
  }

  /** @brief 返回本次重获门控开始后新增的对齐帧数；序号回退时保守返回零。 */
  std::size_t search_new_aligned_frame_count() const
  {
    if (aligned_vision_sequence_ < search_vision_entry_aligned_sequence_) {
      return 0U;
    }
    return aligned_vision_sequence_ - search_vision_entry_aligned_sequence_;
  }

  /**
   * @brief SEARCH 断流时冻结当前参考位置，开始一次等待视觉心跳恢复的重获流程。
   * @param reason 写入诊断的触发原因；重复调用不会重置正在进行的等待计时。
   * @details 清除排名驻留证据以免把断流时间计入连续稳定时间，但保留可在恢复后关联的轨迹。
   * 退出等待的现行条件是心跳新鲜；新增对齐帧数保留用于诊断，并非退出所需的门限。
   */
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
    // 断流不算“排名持续稳定”；恢复后必须重新积累证据，不能直接推进到 ALIGN。
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

  /** @brief 用正余弦求航向圆均值（ENU 弧度），避免 359° 与 1° 被普通均值误算为 180°。 */
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

  /**
   * @brief 检查航向样本数量、单调时间覆盖和相对圆均值的最大角差。
   * @note 当前用于准备状态的诊断输出，pre_guided_ready() 未将它作为起飞门控。
   */
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

  /** @brief 起飞前准备满足且当前已进入 GUIDED 时，允许锁定任务坐标系。 */
  bool frame_lock_ready() const
  {
    return pre_guided_ready() && guided_active_;
  }

  /**
   * @brief 汇总配置、飞控连接/未解锁、导航、视觉、持续静止及舵机初始化条件。
   * @return 准备条件全部满足时为 true；不请求解锁，也不改变任务状态。
   */
  bool pre_guided_ready() const
  {
    return config_valid_ && fcu_state_.connected &&
      !fcu_state_.armed && odom_fresh() && vision_ready_before_takeoff() &&
      position_stable_since_.has_value() &&
      steady_age_s(*position_stable_since_) >= position_stability_s_ &&
      horizontal_speed_m_s_ <= stationary_speed_m_s_ &&
      (!uses_servo() || servos_initialized_);
  }

  /**
   * @brief 解析配置中的搜索航线数量：显式正数优先，否则按视场与重叠估计。
   * @note 构造时用于初始诊断；build_search_route() 的现行固定双线流程会覆盖有效数量。
   */
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

  /**
   * @brief 以当前本地位置锁定 home_，按配置的地理航向建立任务坐标系并生成搜索/侦察路线。
   * @details 飞机此刻的里程计偏航与罗盘均值用于诊断；任务轴实际由标定参数决定。
   * 地理航向从北顺时针，ENU 偏航从东逆时针，转换后同时缓存目标姿态四元数。
   */
  void lock_frame()
  {
    home_ = position_;
    const double odom_yaw_enu = normalize_angle(current_vehicle_yaw_);
    const double compass_yaw_enu = mean_heading();
    // 地理航向与 ENU 偏航的零点和正方向不同，因此 yaw_enu = 90° - heading。
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

  /**
   * @brief 生成两条横向搜索线及中间前移连接，共四个本地 ENU 航点。
   * @details 场地边界、边距、高度及前移距离来自参数；当前实现固定双线，
   * 不调用历史视场估算结果来生成更多航线，不能仅改 search_lane_count 就假定航迹改变。
   */
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
    // 任务轴为标定的前、左、上；所有场地航点经 field_to_local() 转换后再发布。
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

  /**
   * @brief 按侦察区域尺寸、边距和航线数建立往返航点，并记录区域入口。
   * 航点先在任务场地坐标中计算，再转为本地 ENU；相邻航线反向飞行构成覆盖路径。
   * 本函数只生成路径，不执行视觉识别，也不检查真实场地的障碍或净空。
   */
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

  /**
   * @brief 执行一次受计时包装的控制周期，并按统计窗口输出调度质量。
   * @details 无显式输入；先记录周期入口，再调用 tick()。使用 SteadyClock 统计相邻入口间隔、
   * 本次执行耗时和窗口频率；内部时间单位为 s，日志中的间隔与耗时换算为 ms。
   * 窗口到期会清零计数与最大值；本函数不代表飞控实际执行频率，也不提供实时调度保证。
   * @return 无返回值；任务推进及控制副作用由 tick() 执行。
   */
  void timed_tick()
  {
    const SteadyTimePoint started = SteadyClock::now();
    if (control_tick_count_ > 0U) {
      control_max_gap_s_ = std::max(
        control_max_gap_s_, std::chrono::duration<double>(started - last_control_tick_).count());
    }
    last_control_tick_ = started;
    tick();
    control_max_runtime_s_ = std::max(
      control_max_runtime_s_,
      std::chrono::duration<double>(SteadyClock::now() - started).count());
    ++control_tick_count_;
    const double window_s =
      std::chrono::duration<double>(SteadyClock::now() - control_rate_window_start_).count();
    if (window_s >= 4.0) {
      RCLCPP_INFO(
        get_logger(), "CONTROL_RATE hz=%.2f max_gap_ms=%.2f max_tick_ms=%.2f",
        static_cast<double>(control_tick_count_) / window_s,
        control_max_gap_s_ * 1000.0, control_max_runtime_s_ * 1000.0);
      control_rate_window_start_ = SteadyClock::now();
      control_tick_count_ = 0U;
      control_max_gap_s_ = 0.0;
      control_max_runtime_s_ = 0.0;
    }
  }

  /**
   * @brief 按固定先后顺序处理异步结果、执行器清理与任务状态推进。
   * @details 输入来自回调缓存、待处理服务和成员状态。先消费 ACK、清理与视觉帧，
   * 再按当前发布开关发送设定点，最后检查超时并运行状态分支；因此本轮新目标通常下轮发布。
   * 任务期限、状态驻留和清理期限使用 SteadyClock（s）；消息时间及节流日志使用 ROS 时钟。
   * @return 无返回值；可能修改任务状态、目标、发布开关，提交服务，或在终态关闭节点。
   */
  void tick()
  {
    // 先回收异步结果再决定本轮行为，避免仍按上一轮 pending 标志重复提交命令。
    check_service_results();
    check_servo_results();
    handle_abort_stow();
    initialize_servos_if_ready();
    process_pending_vision_frames();

    if (publish_setpoint_ && frame_locked_) {
      publish_setpoint();
    }
    if (mission_timeout_s_ > 0.0 && mission_started_ && mission_timeout_applies(state_) &&
      steady_age_s(mission_start_time_) > mission_timeout_s_)
    {
      if (state_ == State::RELEASE && uses_servo() &&
        release_actuation_committed_ && !stow_complete_)
      {
        // 命令已经提交却尚无收回确认时，总任务超时不能直接开启水平返航。
        // 保留收回清理请求，同时改走降落；这里没有据超时推断机构必定未动作。
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
      // 仅在还未提交当前释放动作的搜索/对准阶段切固定坐标路径，
      // 不把这一阶段期限套到已经在途的释放服务上。
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
        } else if (measurement_time_sync_) {
          if (fcu_state_.armed) {
            mark_failure("Aircraft armed before pre-GUIDED preparation completed");
            enter(State::ABORT);
          } else if (pre_guided_ready()) {
            // 同步路径先完成准备和预发送窗口，再宣布可切 GUIDED，
            // 避免把准备时间放在人工授权之后。收到新的 GUIDED 授权边沿时
            // 再刷新 home/坐标原点，不额外重复静止等待。
            lock_frame();
            publish_setpoint_ = true;
            enter(State::PRESTREAM);
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
              "PRE_GUIDED_PREP time=%s odom=%s vision=%s stationary=%s servo=%s; "
              "尚未就绪，请勿切换 GUIDED",
              flight_clock_ready() ? "yes" : "no", odom_fresh() ? "yes" : "no",
              vision_ready_before_takeoff() ? "yes" : "no",
              position_stable_since_.has_value() ? "yes" : "no",
              (!uses_servo() || servos_initialized_) ? "yes" : "no");
          }
        } else if (fcu_state_.connected) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        publish_setpoint_ = false;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "WAITING_FOR_GUIDED：准备已完成，请在遥控器切换 GUIDED；切换后直接解锁起飞");
        if (measurement_time_sync_) {
          if (fcu_state_.armed) {
            fail_and_land("Aircraft armed before GUIDED authorization");
          } else if (!pre_guided_ready()) {
            frame_locked_ = false;
            RCLCPP_WARN(get_logger(), "READY revoked: preflight data changed; returning to preparation");
            enter(State::WAIT_FCU);
          } else if (!guided_active_) {
            // READY 之后必须先观察到非 GUIDED，再接收一次新的切入边沿；
            // 防止节点启动前或准备期间已有的模式值被当成此次人工授权。
            guided_ready_low_seen_ = true;
          } else if (guided_ready_low_seen_) {
            lock_frame();  // 以授权瞬间的位置刷新最终原点，沿用已经通过的稳定准备条件。
            publish_setpoint_ = true;
            enter(State::WAIT_ARM);
          }
          break;
        }
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
        if (measurement_time_sync_) {
          if (fcu_state_.armed) {
            fail_and_land("Aircraft armed before pre-GUIDED preparation completed");
          } else if (!pre_guided_ready()) {
            publish_setpoint_ = false;
            frame_locked_ = false;
            enter(State::WAIT_FCU);
          } else if (steady_age_s(state_enter_time_) >= prestream_s_) {
            // 准备窗完成后暂停预发送，等待遥控器明确重新授权，不自行切飞行模式。
            publish_setpoint_ = false;
            guided_ready_low_seen_ = !guided_active_;
            RCLCPP_INFO(get_logger(),
              "READY_FOR_GUIDED：准备完成，等待切换 GUIDED；切换后直接解锁起飞%s",
              guided_active_ ? "（当前已是 GUIDED，请先切回其他模式再切入）" : "");
            enter(State::WAIT_GUIDED);
          }
          break;
        }
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
          // 起飞服务的 success 不等于已经升空；此处按导航报告的相对高度推进，
          // 仍不能把这一个高度门解释成完整的飞行稳定性证明。
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
        // 落地判断有持续窗口，服务 ACK 或低高度的单帧都不能直接触发上锁。
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
          // 请求上锁前落地证据一旦丢失就退回 LAND，不能沿用旧确认继续上锁。
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
        // 飞行控制已交给外部；只给在途执行器结果和必要清理留出有限时间，
        // 不因迟到 ACK 恢复位置设定点发布或自主任务。
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
        // 已解锁时仍交给 LAND 流程，而不是立即退出并假定飞机已经停止。
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

  /**
   * @brief 判断给定任务阶段是否要求持续保持 GUIDED 授权。
   * @param state 待判断的阶段；测量时间同步路径的 PRESTREAM 在人工授权前完成，特意豁免。
   * @return true 表示该阶段要求 GUIDED；false 表示本判据不要求，不等于该阶段已经安全。
   * @note 只读取阶段和配置，不读取时钟，不修改状态。
   */
  bool guided_required_state(State state) const
  {
    if (measurement_time_sync_ && state == State::PRESTREAM) {return false;}
    return state == State::LOCK_FRAME || state == State::PRESTREAM ||
      state == State::WAIT_ARM || state == State::TAKEOFF ||
      state == State::SEARCH || state == State::ALIGN ||
      state == State::RELEASE || state == State::RECON_CLIMB ||
      state == State::RECON_SURVEY || state == State::RETURN_CLIMB ||
      state == State::RETURN_HOME;
  }

  /**
   * @brief 判断总任务期限是否还约束给定阶段。
   * @param state 待判断的阶段；返航、降落、上锁及终态使用各自流程，不再次被总期限打断。
   * @return true 表示 tick() 应对该阶段检查总期限；本函数不计算已经经过的时间。
   * @note 纯条件判断，无时钟读取或状态副作用。
   */
  bool mission_timeout_applies(State state) const
  {
    return state != State::RETURN_CLIMB && state != State::RETURN_HOME &&
      state != State::LAND && state != State::DISARM &&
      state != State::DONE && state != State::PILOT_OVERRIDE &&
      state != State::ABORT;
  }

  /**
   * @brief 为飞行任务分支检查飞控连接、解锁、GUIDED 和里程计条件。
   * @details 输入为最新缓存的飞控/导航状态。TAKEOFF 入态后前 5 s 按单调时钟给予里程计宽限；
   * 其余阶段要求 odom_fresh()。连接/解锁异常转失败降落，GUIDED 丢失转人工接管。
   * @return true 才允许本轮继续相应飞行分支；false 可能已经切换状态，不应再推进原分支。
   */
  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail_and_land("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      // 模式切出意味着当前自主授权结束，转接管状态而不擅自请求切回 GUIDED。
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

  /**
   * @brief 开始或重新开始一轮固定次序搜索。
   * @details 读取搜索路线、遍次和聚类配置；首次进入时用 SteadyClock 记录整个投放阶段起点，
   * 以便多轮搜索共享总期限。清除当前目标与相关稳定历史，重置路线索引并增加搜索遍次。
   * 目标坐标单位 m，分段名义速度单位 m/s；空路线转失败返航，其余情况进入 SEARCH。
   * @return 无返回值；会重建当前航段，首轮首段可使用转场速度而非搜索速度。
   */
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

  /**
   * @brief 用两个配置坐标构造固定目标计划并开始当前载荷转场。
   * @param direct_release true 时在转场结束后直接进入 RELEASE；false 时继续对准流程。
   * @param start_payload 起始载荷的零基索引，按本函数构造的两个目标截断。
   * @details 清除旧计划，把场地坐标换成本地坐标（m），构造冻结目标而不把它们当作视觉检测证据。
   * 目标 stamp 使用 ROS 当前时间，arrival 使用单调时钟；会锁定计划并调用转场入口改变状态。
   * @return 无返回值；不会在此函数中发送投放命令。
   */
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

  /**
   * @brief 投放阶段超时后，为尚未处理的载荷切入固定坐标直接投放路径。
   * @pre 调用方已经建立 drop_phase_start_time_；tick() 在 SEARCH/ALIGN 超时分支调用。
   * @details 剩余载荷为零时直接返回；否则标记固定回退，按 SteadyClock 记录阶段耗时（s），
   * 保留当前载荷索引并让 start_fixed_release_test() 重建计划、开始转场。
   * @return 无返回值；这是配置流程的降级入口，不是已经完成投放的确认。
   */
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

  /**
   * @brief 为当前固定目标建立转场航段并进入 ALIGN。
   * @details 需要 active_bucket_；缺失时走失败返航。目标由投放偏置和相对高度生成（m），
   * 直接投放路径使用搜索高度，其余使用粗对准高度；名义转场速度为 m/s。
   * 航段起点时间由 start_segment() 用单调时钟记录，并置固定转场标志。
   * @return 无返回值；仅安排位置设定点路径，不保证机体达到名义速度。
   */
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

  /**
   * @brief 采样固定目标转场，并在时间及位置条件同时满足后结束转场。
   * @details 依据单调时钟采样航段，距离单位 m。完成后清固定转场标志并重置状态驻留计时；
   * 直接投放路径还会冻结目标并进入 RELEASE，普通路径留给后续对准处理。
   * @return false 表示固定转场本来未启用；true 表示本轮已处理该转场，即使本轮刚刚完成。
   * @note 调用者据返回值停止本轮普通对准，避免同周期叠加两种目标更新。
   */
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

  /**
   * @brief 为已选目标建立到粗对准位置的转场航段。
   * @details 读取当前目标和投放偏置，坐标单位 m、名义速度单位 m/s；缺目标则失败返航。
   * 通过 start_segment() 记录单调起点，标记目标间转场，暂以当前位置为目标并进入 ALIGN。
   * @return 无返回值；到达确认由 update_inter_target_transit() 完成。
   */
  void start_inter_target_transit()
  {
    if (!active_bucket_.has_value()) {
      fail_and_return("Inter-target transit has no active target");
      return;
    }
    const Point3 destination = desired_release_pose(coarse_alt_m_);
    start_segment(position_, destination, drop_target_transit_speed_m_s_);
    inter_target_transit_active_ = true;
    target_ = position_;
    enter(State::ALIGN);
    RCLCPP_INFO(
      get_logger(),
      "INTER_TARGET_TRANSIT_START payload=%zu speed=%.2f destination=(%.2f,%.2f,%.2f)",
      payload_index_ + 1U, drop_target_transit_speed_m_s_,
      destination.x, destination.y, destination.z);
  }

  /**
   * @brief 推进搜索航段、候选锁定与搜毕后的分流。
   * @details 读取当前观测、候选计划、路线和搜索遍次；正常周期采样以单调时钟定义的航段。
   * 若遗留的重获等待标志有效，先保持指定点，按心跳恢复或等待期限继续路线；
   * 否则按配置选择在线锁定或搜毕聚类，最终进入对准、下一遍搜索、固定回退或失败返航。
   * @return 无返回值；会修改目标、路线索引、候选和任务状态，距离/直径均以 m 表示。
   */
  void update_search()
  {
    if (search_vision_acquisition_pending()) {
      target_ = search_vision_hold_position_;
      const std::size_t new_aligned_frames = search_new_aligned_frame_count();
      if (vision_heartbeat_fresh())
      {
        // 现行重获退出条件是心跳新鲜；对齐帧数只记录诊断，
        // 不在此分支额外要求日志显示的最小帧数。
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
        // 重获等待有上限；超时恢复搜索航段，不让视觉断流永久占住保持点。
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

    std::optional<BucketTrack> candidate;
    if (!use_post_search_cluster_fit_) {
      // 在线选目标路径允许已稳定的标定小桶/中桶组合提前锁定，
      // 不必等整条搜索路线走完；直径中心与容差来自参数，默认示例为
      // 0.150±0.015 m 与 0.200±0.015 m，不是写死的现场桶规格。
      // 若不能提前锁定，仍保留正常的三桶比较和搜毕处理。
      (void)try_lock_target_plan(true, true);
      candidate = current_target_for_payload();
    }
    if (candidate.has_value()) {
      active_bucket_ = candidate;
      alignment_stable_since_.reset();
      start_inter_target_transit();
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
        start_inter_target_transit();
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
      start_inter_target_transit();
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

  /**
   * @brief 根据世界目标与载荷安装偏置求机体期望投放位置。
   * @param relative_altitude_m 相对锁定 home 的高度（m），不是地面测距。
   * @details 将载荷的机体系 XY 偏置按 mission_yaw_（rad）旋到本地系，再从目标坐标扣除，
   * 使安装投放点与目标对齐；缺目标或 home 时原样返回当前位置，不额外报告有效性。
   * @return 本地坐标下的期望机体位置（m）；不读取时钟或修改成员。
   */
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

  /**
   * @brief 把采集时刻的桶口观测表达成当前机体系分量。
   * @pre active_bucket_ 已存在；需要采集导航时由目标对象持有该快照。
   * @details 同步路径先用采集位姿固定观测的本地世界点，再按当前位姿转回机体系；
   * 不具备该路径条件时沿用保存的 rim_body，不在此函数中判断观测年龄。
   * @return 桶口相对分量（m）；只做几何运算，不读取时钟或修改目标。
   */
  Point3 release_rim_at_current_pose() const
  {
    if (!measurement_time_sync_ || !active_bucket_->capture_navigation) {
      return active_bucket_->rim_body;
    }
    // 保留既定投放十字定义，只把观测从采集位姿搬到当前位姿。
    // 先固定世界点再换轴，避免重复使用同一帧时把同一个世界目标
    // 错误地解释成随飞机一起平移的新目标。
    const Point3 world = body_to_local(
      active_bucket_->rim_body, *active_bucket_->capture_navigation);
    return local_to_body_current(world);
  }

  /**
   * @brief 根据当前桶口误差与投放十字偏置生成视觉修正目标。
   * @param relative_altitude_m 相对 home 的期望高度（m）。
   * @details 桶口使用内部 FLU，配置偏置使用 FRD，故左右分量通过相反符号组合；
   * 先按当前滚转/俯仰处理水平修正，再按当前或锁定航向（rad）转入本地 XY。
   * 缺目标、home 或所需偏置时保留当前 XY；函数本身不检验新鲜度，也不执行限速。
   * @return 本地位置目标（m），无状态副作用；发布前仍需经过相应设定点限速。
   */
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
    // 桶口观测按内部 FLU 表达，投放十字偏置按 FRD 配置：
    // 前向直接相减，右向与 FLU 左向异号，故 error_y 使用加法。
    // 修正方向为“桶口减投放点”，目标在投影投放点前/右方时向相应方向调整机体目标。
    const Point3 rim = release_rim_at_current_pose();
    const double error_x = rim.x - release_offsets_frd_[start];
    const double error_y = rim.y + release_offsets_frd_[start + 1U];
    const double cr = std::cos(current_roll_);
    const double sr = std::sin(current_roll_);
    const double cp = std::cos(current_pitch_);
    const double sp = std::sin(current_pitch_);
    const double x_pitch = cp * error_x + sp * sr * error_y;
    const double y_pitch = cr * error_y;
    const double yaw = measurement_time_sync_ ? current_vehicle_yaw_ : mission_yaw_;
    return Point3{
      position_.x + std::cos(yaw) * x_pitch - std::sin(yaw) * y_pitch,
      position_.y + std::sin(yaw) * x_pitch + std::cos(yaw) * y_pitch,
      home_->z + relative_altitude_m};
  }

  /**
   * @brief 计算桶口与当前载荷投放十字之间的水平观测误差。
   * @details 读取当前目标、FRD 偏置和采集导航；同步路径使用 ROS 测量时钟检查 stamp，
   * 时钟未就绪或观测已超过 0.5 s 时返回无穷大，不能视为零误差。
   * @return 水平误差模长（m）；缺目标/偏置同样返回正无穷，不修改成员。
   * @note 此处不是完整新鲜度判据；仅 staged_release_gate_ 分支另查非负年龄、帧身份与导航质量。
   * 非分阶段分支直接使用本误差或等待超时，不执行同一组附加检查。
   */
  double visual_release_alignment_error() const
  {
    if (!active_bucket_.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const std::size_t start = payload_index_ * 3U;
    if (start + 1U >= release_offsets_frd_.size()) {
      return std::numeric_limits<double>::infinity();
    }
    if (measurement_time_sync_ && (!flight_clock_ready() ||
      (now() - active_bucket_->stamp).seconds() > 0.5)) {
      return std::numeric_limits<double>::infinity();
    }
    const Point3 rim = release_rim_at_current_pose();
    const double error_x = rim.x - release_offsets_frd_[start];
    const double error_y = rim.y + release_offsets_frd_[start + 1U];
    return std::hypot(error_x, error_y);
  }

  /**
   * @brief 校验投放目标后，按转场优先级推进 ALIGN。
   * @details 当前目标和 home 缺失、或目标已记为投放时，转失败返航；
   * 先处理目标间转场，再处理固定目标转场，最后进入粗对准与下降。
   * 本函数不直接读时钟，分段和下降辅助函数使用单调计时；位置单位 m。
   * @return 无返回值；转场辅助函数已消费本周期时立即返回，避免覆盖其设定点。
   */
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
    if (inter_target_transit_active_ && update_inter_target_transit()) {
      return;
    }
    if ((fixed_release_test_mode_ || fixed_release_direct_active_) &&
      update_fixed_target_transit()) {
      return;
    }
    // 下降阶段继续用实时视觉更新期望 XY，但不直接发布视觉给出的跳变位置。
    // publish_setpoint() 会约束水平/竖直设定点变化率；这是参考轨迹限制，
    // 不是对机体实际下降速度或跟踪误差的保证。
    update_coarse_release_alignment();
  }

  /**
   * @brief 推进目标间转场，并在完成时释放普通下降流程。
   * @details 以单调时钟采样航段，位置/接收半径单位 m；
   * 完成后清转场和下降设定点初始化标志，并重置状态驻留时间。
   * @return true 表示还在转场，本周期不得继续对准；false 表示刚完成，可以同周期进入下一步。
   * @note 与 update_fixed_target_transit() 的返回约定不同，调用处必须按各自语义使用。
   */
  bool update_inter_target_transit()
  {
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      return true;
    }
    inter_target_transit_active_ = false;
    drop_setpoint_initialized_ = false;
    state_enter_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(),
      "INTER_TARGET_TRANSIT_COMPLETE payload=%zu; entering fine alignment",
      payload_index_ + 1U);
    return false;
  }

  /**
   * @brief 从机械粗位下降至投放高度，并维护视觉修正或坐标回退。
   * @pre 调用方已确认当前目标及 home 存在。
   * @details 先按水平/高度误差到达粗位（m），再向精投高度下降；视觉有效时更新 XY，
   * 失效时保留上一指导 XY。到达高度与水平门限后冻结位置、重置投放周期并进入 RELEASE。
   * 本函数不直接累计稳定时间；这里只完成下降门，后续释放门由 update_release() 单调计时。
   * @return 无返回值；修改目标、下降/回退标记和任务状态，不发送舵机服务。
   */
  void update_coarse_release_alignment()
  {
    // 先到机械偏置对应的粗位，再下降并持续修正期望 XY。
    // 本函数只写期望目标，发布端另行限制水平和竖直设定点步进，
    // 不能把这里的配置速率解释成机体已经实现的速度。
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
      // 回退路径使用已缓存的 XY，只等待位置/高度门，不伪造新的视觉观测。
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
      // 无法刷新目标或目标/身份条件不满足时，保留此前指导 XY 并继续当前下降策略。
      // visual_valid 并不检查观测年龄，不能据此声称旧观测一定进入这个回退分支；
      // 这并不表示目标重新被看见，后续 RELEASE 仍按自己的门限/超时判定。
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

  /**
   * @brief 标记投放状态不确定，需要先处理收回再尝试任务返航。
   * @param reason 诊断原因；清理原因只保留首次非空值，任务失败原因由 mark_failure() 首次锁存。
   * @details 设置中止投放与清理后返航标志，并记录失败；不在此处发送收回命令或读取时钟。
   * @return 无返回值；这是清理请求，不代表已经收到收回 ACK，更不代表机构已物理回位。
   */
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

  /**
   * @brief 清空下一次投放周期的软件握手、清理和稳定门历史。
   * @details 无显式输入；清除待应答标志、确认标志、虚拟 future 与稳定计时/判据。
   * 不会撤销已经发往外部的命令，也不会清空 pending_servo_commands_ 中的真实待处理项。
   * @return 无返回值；不读取时钟、不执行物理回位，调用者必须保证周期切换时机正确。
   */
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
    release_gate_.reset();
  }

  /**
   * @brief 推进释放前稳定门、一次命令提交、保持及收回的软件流程。
   * @details 输入为冻结位置、当前目标观测、导航速度及异步确认标志。门限耗时/保持时间用
   * SteadyClock（s），采集年龄与新帧资格用 ROS 时间（ns/s）；二者不能混作同一时钟。
   * 已提交命令后不再重新开始视觉门；先等待服务结果，舵机路径还等待收回命令被接受。
   * @return 无返回值；可能提交服务、发布快照请求、推进载荷或进入失败清理；ACK 不是物品分离证据。
   */
  void update_release()
  {
    target_ = frozen_release_pose_;

    if (release_cleanup_return_requested_) {
      // 清理路径优先于新的视觉门和释放动作。只有收回服务已被接受且没有在途命令，
      // 才允许启动失败返航；stow_complete_ 仍是协议标志，不是机械到位传感器。
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
        // 已提交的动作不能因目标记忆丢失就当成没发生，保留执行器清理责任。
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

      // 只有首次进入投放平面 ±0.05 m 后才启动稳定计时，
      // 避免 ALIGN 刚切 RELEASE、尚未完成下降时就消耗 0.5/4/6 s 门限。
      // 计时启动后 XY 仍可更新，但发布端继续限制其设定点变化率。
      if (!release_stabilization_since_.has_value() &&
        std::abs(position_.z - frozen_release_pose_.z) > 0.05)
      {
        return;
      }
      if (!release_stabilization_since_.has_value()) {
        release_stabilization_since_ = SteadyClock::now();
        release_gate_start_ros_ns_ = now().nanoseconds();
        RCLCPP_INFO(
          get_logger(), "RELEASE_STABILIZATION_BEGIN payload=%zu duration=%.2fs",
          payload_index_ + 1U, staged_release_gate_ ? 6.0 : release_stabilization_s_);
      }
      const double visual_error = refreshed ?
        visual_release_alignment_error() :
        std::numeric_limits<double>::infinity();
      if (staged_release_gate_) {
        const double elapsed = steady_age_s(*release_stabilization_since_);
        const double capture_age = (now() - active_bucket_->stamp).seconds();
        // 单调时长决定 0.5/4/6 s 阶段；ROS 采集时间只决定观测年龄与帧资格。
        // 必须确认目标来自最新非空帧，不能用反复刷新的控制周期冒充不同视觉帧。
        const bool fresh = refreshed && capture_age >= 0.0 && capture_age <= 0.5 &&
          std::isfinite(visual_error) && odom_fresh() &&
          active_bucket_->stamp == last_vision_capture_stamp_ &&
          !latest_frame_detections_.empty();
        const auto decision = release_gate_.update(elapsed, active_bucket_->stamp.nanoseconds(),
          fresh, visual_error, horizontal_speed_m_s_,
          active_bucket_->stamp.nanoseconds() >= release_gate_start_ros_ns_ + 500000000LL);
        if (decision == ReleaseGate::Decision::WAIT) {return;}
        if (decision == ReleaseGate::Decision::FORCED) {
          // 6 s 到期采用强制释放策略；此决策不代表视觉误差或水平速度已满足稳定门。
          RCLCPP_WARN(get_logger(),
            "VISUAL_RELEASE_TIMEOUT payload=%zu error=%.3fm elapsed=%.3fs; forcing release at 6s",
            payload_index_ + 1U, visual_error, elapsed);
        } else {
          RCLCPP_INFO(get_logger(),
            "VISUAL_RELEASE_TOLERANCE payload=%zu error=%.3fm gate=%s frames=%d speed=%.3fm/s elapsed=%.3fs",
            payload_index_ + 1U, visual_error,
            decision == ReleaseGate::Decision::STABLE ? "four_frames" : "relaxed_single_frame",
            release_gate_.count, horizontal_speed_m_s_, elapsed);
        }
      } else if (visual_error <= release_visual_xy_tolerance_m_) {
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
        // true 仅表示命令已入异步请求队列；先锁住“已提交”身份，
        // 后续周期只轮询结果/收回，避免因未收到 ACK 再释放一次。
        release_command_pending_ =
          send_servo(payload_index_, true, ServoPurpose::RELEASE);
        if (!release_command_pending_) {
          fail_and_return("Unable to send servo release command");
          return;
        }
        release_actuation_committed_ = true;
        std_msgs::msg::String snapshot;
        // 快照话题是拍照请求的载荷编号，不是物品已经脱离或命中目标的证据。
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
        // 在 RELEASE 服务结果出来前不启动正常保持/收回计时，异常清理由全局处理负责。
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
        // 本节点以释放/收回服务成功应答作为软件完成条件，未接入独立分离传感器。
        finish_payload_release();
      }
      return;
    }

    if (release_actuation_confirmed_) {
      finish_payload_release();
    }
  }

  /**
   * @brief 根据本节点的软件完成条件结算当前载荷并安排下一阶段。
   * @details 调用方已完成对应后端的应答/收回流程；此处登记目标 ID 和位置以避免再次选择，
   * 增加载荷索引并清空本轮历史，再进入下个固定身份目标、侦察或返航。
   * 记录坐标单位 m，观测年龄仅用于日志；本函数不读取新的传感器物理分离结果。
   * @return 无返回值；完成标志表达后端协议结果，不增加独立的实际落物确认。
   */
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
    // 下一个载荷延续已冻结目标身份，不能因直径排序短时改变而改投另一个桶。
    if (!last_known.has_value() || released_target(last_known->id)) {
      fail_and_return("Frozen second-smallest target ID unavailable");
      return;
    }
    active_bucket_ = *last_known;
    if (fixed_release_test_mode_ || fixed_release_direct_active_) {
      start_fixed_target_transit();
    } else {
      start_inter_target_transit();
    }
    RCLCPP_INFO(
      get_logger(),
      "SECOND_TARGET_ID_FROZEN id=%zu diameter=%.3f local=(%.2f,%.2f) "
      "last_observation_age=%.3f",
      active_bucket_->id, active_bucket_->diameter,
      active_bucket_->local.x, active_bucket_->local.y,
      active_target_observation_age_s());
  }

  /**
   * @brief 投放阶段结算后，在当前位置上方建立侦察前爬升段。
   * @details 需要锁定 home 和非空预制侦察路线，否则失败返航。相对高度取起飞/侦察高度较大值（m），
   * 名义速度为 m/s；清侦察索引与驻留历史，通过航段单调起点进入 RECON_CLIMB。
   * @return 无返回值；此处不根据投放目标重新生成侦察路线。
   */
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

  /**
   * @brief 完成侦察前高度转换后，直接转向预先冻结的首个侦察点。
   * @details 采样单调计时航段；仅当时间和位置门均满足后建立下一航段并进入 RECON_SURVEY。
   * 坐标/接收半径单位 m，名义转场速度单位 m/s；不插入由桶位置决定的额外入口。
   * @return 无返回值；更新 target_、航段和状态。
   */
  void update_recon_climb()
  {
    target_ = sample_segment();
    if (!segment_complete(accept_radius_m_)) {
      return;
    }
    // 侦察路线已在起飞前冻结；完成高度转换后直接去首点，
    // 不把投放桶位置再当作侦察入口来改变既定路线。
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

  /**
   * @brief 按冻结侦察点顺序完成到点与驻留，并在末点后启动返航。
   * @details 航段完成同时要求时间/位置门；不满足时清驻留历史。驻留使用 SteadyClock（s），
   * 满足后才增加索引，避免只经过目标附近便结算；坐标和接收半径单位 m。
   * @return 无返回值；会更新目标、驻留时间、侦察完成标志并可能启动返航。
   */
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

  /**
   * @brief 安排先转换至返航高度、再水平回到 home 的两段返航。
   * @details 必须有锁定 home；缺失则转失败降落。先保持当前 XY，目标 Z 为 home 高度加返航高度（m），
   * 通过 start_segment() 记录单调起点，打开设定点发布并进入 RETURN_CLIMB。
   * @return 无返回值；不重新验证导航/GUIDED，调用者及后续 flight_gate_ok() 负责相应门限。
   */
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

  /**
   * @brief 推进返航高度段，完成后建立回到 home XY 的航段。
   * @details 位置单位 m，速度单位 m/s；正常完成需航段时间与位置同时满足。
   * 以单调状态驻留时间检查独立返航爬升期限，超时锁存失败、停位置发布并转 LAND。
   * @return 无返回值；不因总任务期限反复重启返航段。
   */
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

  /**
   * @brief 推进水平返航，并在到点或返航看门狗到期后交给降落流程。
   * @details 到点半径取配置与 0.5 m 的较大值；仍需同时满足航段时间条件。
   * 返航期限使用 SteadyClock 状态驻留时间（s），超时记录失败；两种退出都停位置发布并进入 LAND。
   * @return 无返回值；进入 LAND 不代表已经触地或上锁。
   */
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

  /**
   * @brief 用起终点和名义速度建立一个带单调起点的线性设定点航段。
   * @param start 本地起点（m）。
   * @param end 本地终点（m）。
   * @param speed_m_s 计算时长用的名义速度（m/s），计算中最低按 0.1 m/s 处理。
   * @details 时长取最小航段时长与三维距离/名义速度中的较大值，单位 s；只替换 segment_。
   * @return 无返回值；不发布目标、不切状态，也不保证飞行器实际按此速度运动。
   */
  void start_segment(
    const Point3 & start, const Point3 & end, double speed_m_s)
  {
    segment_ = Segment{
      start, end, SteadyClock::now(),
      std::max(
        min_segment_s_,
        distance_xyz(start, end) / std::max(0.1, speed_m_s))};
  }

  /**
   * @brief 按单调已过时间在线性航段内插值位置设定点。
   * @details 进度限制在 [0,1]，虽然局部变量名为 smooth，当前实现仍是线性比例，
   * 没有额外加速度或 jerk 整形；读取 segment_ 的起点时刻与时长（s）。
   * @return 本地位置（m）；不修改航段，不判断机体是否已经到达终点。
   */
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

  /**
   * @brief 同时检查航段参考时间已走完和当前机体位置接近终点。
   * @param radius 当前本地三维位置到终点的允许距离（m）。
   * @return 两个条件同时满足才为 true；仅参考已到终点、或仅机体短暂接近终点都不足以完成。
   * @note 使用 SteadyClock 计算航段耗时（s），只读位置/航段，不修改状态或验证导航新鲜度。
   */
  bool segment_complete(double radius) const
  {
    return steady_age_s(segment_.start_time) >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  /**
   * @brief 初始化投放后端：虚拟模式直接软件置位，舵机模式在前置条件满足时请求收起。
   * @details 虚拟后端直接标记软件初始化完成；舵机路径要求配置有效、使能、飞控连接、服务就绪，
   * 并按单调时钟至少间隔 1 s 尝试。统计本轮待应答数量，由 check_servo_results() 汇总结果。
   * @return 无返回值；舵机初始化完成表示对应命令得到成功应答，虚拟模式不请求或等待 ACK。
   * 两者都不是独立机械位置反馈。
   */
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

  /**
   * @brief 将某载荷的收起或释放 PWM 作为 MAV_CMD_DO_SET_SERVO 异步提交。
   * @param payload 载荷的零基索引，用于选择通道及两组 PWM 配置。
   * @param release true 选择释放 PWM，false 选择收起 PWM；PWM 字段按命令约定以 μs 表示。
   * @param purpose 保存到待处理项的用途，决定后续 ACK 如何更新初始化/释放/收回状态。
   * @details 检查后端、服务、索引和正值；用 SteadyClock 记发送时刻，并入队 future 和载荷身份。
   * @return true 仅表示已提交并登记待应答项；false 表示本次未提交，不表示外部机构当前处于何位置。
   */
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

  /**
   * @brief 非阻塞轮询每个舵机命令的服务结果与 ACK 期限。
   * @details future.wait_for(0s) 只探测就绪；发送后等待时间由 SteadyClock（s）计算。
   * 按用途更新初始化计数、释放确认/开启计时或收回确认。INITIALIZE 失败只记录本轮失败，
   * 后续允许重试初始化；RELEASE/STOW 的失败与超时进入清理处理，不重发释放。
   * 处理完的队列项被移除，服务结果不能消除物理动作的不确定性。
   * @return 无返回值；accepted 来自响应 success，不是舵机到位或物品分离的传感器证据。
   */
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
      // ready 与超时可能同周期成立；现有规则在结果已就绪时仍读取其 success，
      // timeout 字段只反映等待年龄，不额外否决已返回的成功结果。
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
          // 从 ACK 到达时刻开始软件保持时间，而非从无法直接观察的实际开门时刻开始。
          if (state_ != State::RELEASE || release_abort_requested_ ||
            release_cleanup_return_requested_)
          {
            require_stow_before_safe_return(
              "Late servo release ACK; stow required before handoff");
          }
        } else {
          // 拒绝/超时都不能证明机构一定没动，故不重试释放，只请求收回清理。
          require_stow_before_safe_return(
            "Servo release rejected or ACK timed out; physical state uncertain");
        }
      } else {
        stow_command_pending_ = false;
        if (accepted) {
          // 字段名沿用“完成”，实际证据级别仍是收回命令的成功响应。
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

  /**
   * @brief 在投放中止后，等在途命令处理结束再节流重试收回。
   * @details 虚拟路径只等待其请求结束；舵机路径避免释放/收回同时在途，已有收回确认则清中止标志。
   * 没有在途项且尚未收回时，按 SteadyClock 至少间隔 1 s 提交一次收回。
   * @return 无返回值；不会恢复自主飞行授权，也不把重试发送成功当成物理回位。
   */
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
      // 先消费在途结果，避免同一载荷同时有互相矛盾的开/关请求。
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

  /**
   * @brief 为虚拟投放后端提交一次 Trigger 服务请求。
   * @details 要求虚拟后端启用、服务可用且没有有效在途 future；用 SteadyClock 保存发送时刻。
   * 此处不含载荷参数，载荷身份由本节点当前周期和后端接口约定关联。
   * @return true 表示异步请求已建立；false 表示本次没有发出请求，均不代表已发生实际/仿真投放。
   */
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

  /**
   * @brief 限制相邻投放位置设定点的水平模长步进与竖直步进。
   * @param desired 本地期望位置（m）。
   * @param horizontal_speed_m_s 水平设定点变化率配置（m/s）。
   * @param vertical_speed_m_s 竖直设定点变化率配置（m/s）。
   * @details 以 SteadyClock 算 Δt 并截到 [0,0.20] s；首次从当前位置初始化，
   * 更新上一发布位置与时间。这里只约束目标序列的变化率，不约束或保证机体真实速度/加速度。
   * @return 本轮可发布的位置设定点（m）；负速率经现有计算折为零步进。
   */
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

  /**
   * @brief 将当前目标转换为带 ROS 时间戳的本地位置/锁定航向设定点并发布。
   * @details ALIGN 非转场阶段与 RELEASE 分别使用配置的设定点变化率，其余阶段重置下降限速历史。
   * 位置使用 map 本地坐标（m），姿态使用预存航向四元数；限速 Δt 使用单调时钟，消息 stamp 使用 now()。
   * @return 无返回值；更新限速缓存并发布消息，不代表飞控已接受或机体已到达该目标。
   */
  void publish_setpoint()
  {
    Point3 published_target = target_;
    if (state_ == State::ALIGN && !fixed_release_transit_active_ &&
      !inter_target_transit_active_)
    {
      // 普通对准下降只逐步推进位置设定点，避免视觉更新把参考位置突然挪到远处。
      // 此处限制的是参考点变化，不是由发布函数保证飞机实际按该速率下降。
      published_target = rate_limited_drop_target(
        target_, alignment_horizontal_speed_m_s_, descent_vertical_speed_m_s_);
    } else if (state_ == State::RELEASE) {
      // 投放前最后修正也必须经过同一限速入口，不能把新十字目标直接跳变发布。
      // 这样保留配置的位置参考变化约束，减少视觉跳变施加的突然跟踪需求；
      // 是否超调仍取决于导航、飞控和真实动力学，不能据此保证。
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

  /**
   * @brief 判断共享飞控请求节流间隔是否已经满足。
   * @return 自上次 mark_request() 的单调时间达到 1 s 时为 true；不检查服务是否就绪。
   * @note 只读 last_request_time_，时间单位 s，无状态副作用。
   */
  bool request_allowed() const
  {
    return steady_age_s(last_request_time_) >= 1.0;
  }

  /**
   * @brief 用当前单调时刻更新共享飞控请求节流基准。
   * @details 无显式输入；由起飞、降落和解锁/上锁请求在提交前调用。
   * @return 无返回值；只更新 last_request_time_，不发请求，不代表任何 ACK。
   */
  void mark_request()
  {
    last_request_time_ = SteadyClock::now();
  }

  /**
   * @brief 在服务就绪、无在途请求且节流允许时提交起飞服务。
   * @details 高度按 takeoff_alt_m_（m）、航向按 locked_compass_deg_（°）填入请求。
   * 用单调时钟登记请求与 ACK 等待起点，并设置 takeoff_sent_；失败应答/超时由结果轮询清标志。
   * @return 无返回值；提交/ACK 都不是升空完成证据，TAKEOFF 分支另用相对高度与期限推进。
   */
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

  /**
   * @brief 在仍解锁且节流/服务条件允许时提交降落服务。
   * @details 复用锁定航向（°），使用单调时钟记录共享节流及本次 ACK 起点；
   * 有效在途 future 存在时不再重复提交。
   * @return 无返回值；本函数不确认触地，落地持续判据由独立函数处理。
   */
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

  /**
   * @brief 按当前授权和落地门限提交解锁或上锁服务。
   * @param arm true 请求解锁，false 请求上锁。
   * @details 解锁仅限已启用自动解锁的 WAIT_ARM，且 GUIDED、视觉与舵机初始化条件满足；
   * 上锁必须先通过 landing_confirmation_ready()。两类请求共享单调 1 s 节流并登记 ACK 时间。
   * @return 无返回值；只保存请求值与 future，真实 armed 状态仍从飞控状态回调观察。
   */
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

  /**
   * @brief 非阻塞消费起飞、降落、解锁/上锁及虚拟投放服务结果。
   * @details 用 future 就绪状态与 SteadyClock ACK 等待时长（s）判断处理时机。
   * 起飞被拒或超时会清 takeoff_sent_ 以允许后续重试；其他 future 完成/超时后也清理。
   * @return 无返回值；服务 success 只表示对应接口结果，飞行、落地和 armed 条件另行观察。
   */
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

  /**
   * @brief 处理虚拟投放响应、迟到应答与 ACK 超时。
   * @details 以 SteadyClock 计算发送后等待时间（s）。正常 RELEASE 的成功响应更新软件确认；
   * 迟到成功仅登记确认，不把状态拉回 RELEASE；拒绝响应除 PILOT_OVERRIDE 外仍调用
   * fail_and_return()，可能改变任务阶段。人工接管分支只记录失败，保留控制归属。
   * 超时清 future，并按当前阶段决定返航或结束中止等待。
   * @return 无返回值；后端 success 的含义依 Trigger 服务约定，不额外证明真实物品分离。
   */
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
        // 迟到的成功响应只登记结果与失败原因，不重新进入 RELEASE 或抢回控制。
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
      // 本地不再等待并不等于远端请求被取消；后端迟到结果仍可能发生。
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

  /**
   * @brief 返回当前本地 Z 相对锁定 home Z 的差值。
   * @return 高度差（m）；没有 home 时按现有约定返回 0.0，不是有效零高度测量。
   * @note 不读取时钟、不检查里程计质量，不修改状态；调用方负责前置有效性。
   */
  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  /**
   * @brief 判断当前缓存是否构成一次可进入落地持续确认的候选。
   * @details 优先使用新鲜的扩展落地状态；明确空中/起飞/降落状态禁止被低里程计高度覆盖。
   * 缺少明确状态时才用新鲜里程计、有限且足够小的速度及相对高度回退条件（m、m/s）。
   * @return true 仅代表本次候选，不代表已满足持续时间；无状态副作用，年龄由相关辅助函数检查。
   */
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
        // 飞控明确报告地面时，若导航已经过期，不因不可得速度永久卡住落地确认；
        // 若导航仍新鲜，则还要求低速度，后续仍需满足持续确认时间。
        return !odom_fresh() || velocity_low;
      }
      // 新鲜且明确的空中/起飞/降落状态优先于高度回退。
      // 防止里程计低高度或坐标重置被误当作触地，从而在仍空中时进入上锁流程。
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

  /**
   * @brief 清除已有落地稳定起点，要求后续重新累计。
   * @return 无返回值；仅重置可选单调时间点，不读取时钟、不提交上锁。
   * @note 进入 LAND 或落地条件丢失时调用，避免沿用先前驻留证据。
   */
  void reset_landing_confirmation()
  {
    landing_stable_since_.reset();
  }

  /**
   * @brief 用连续满足的落地候选维护单调稳定时间窗。
   * @details 候选不成立即清历史；首次成立仅记 SteadyClock 起点，不在同次调用立即确认。
   * @return 稳定跨度达到 landing_confirm_stable_s_（s）时为 true，否则为 false。
   * @note 会修改 landing_stable_since_；这是软件落地判据，不是本函数发出的物理制动动作。
   */
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

  /**
   * @brief 只读复核落地候选与既有持续确认窗口。
   * @return 已有起点、当前候选仍成立且单调持续时间足够时为 true。
   * @note 时间单位 s，不创建/清除窗口，不发上锁；DISARM 请求前会再次使用此判据。
   */
  bool landing_confirmation_ready() const
  {
    return landing_stable_since_.has_value() && landing_candidate() &&
      steady_age_s(*landing_stable_since_) >=
      landing_confirm_stable_s_;
  }

  /**
   * @brief 在安全请求看门狗到期时，仅在观察到交接条件后关闭节点。
   * @param reason 首次失败原因和交接日志使用的说明。
   * @details 先标记失败、停位置发布；只在未解锁、guided_active_ 为 false 或已报告 LAND
   * 中至少一项成立时退出。guided_active_ 同时包含连接条件，断联也可能使它为 false，
   * 因而此分支不等价于已经观察到模式字符串切出 GUIDED。
   * @return true 表示已经按观测条件调用 shutdown；false 表示还需继续飞控安全请求。
   * @note 这些软件退出条件不等于已经触地，也不构成独立的飞手已接管确认。
   */
  bool stop_control_for_observed_handoff(const std::string & reason)
  {
    mark_failure(reason);
    publish_setpoint_ = false;
    const bool safe_handoff_observed =
      !fcu_state_.armed || !guided_active_ || fcu_state_.mode == "LAND";
    if (!safe_handoff_observed) {
      // 不能只因本节点的看门狗到期就结束全部飞控安全请求；
      // 尚未观察到交接条件时由调用方继续请求 LAND/上锁。
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

  /**
   * @brief 锁存第一次任务失败原因，并在释放相关动作在途时请求清理。
   * @param reason 失败说明；已有失败原因不被后续级联故障覆盖。
   * @details 仅检查当前软件状态及命令标志，不读取时钟，也不立即切换飞行状态。
   * @return 无返回值；会设置 mission_failed_，必要时设置 release_abort_requested_。
   */
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

  /**
   * @brief 标记任务失败后，按剩余导航与授权条件选择返航或降落/中止。
   * @param reason 失败诊断文本。
   * @details 仅在仍解锁、有 home、仍 GUIDED 且里程计新鲜时启动两段返航；
   * 否则解锁时转 LAND，未解锁时转 ABORT。导航年龄由 odom_fresh() 判断，返航计时由航段维护。
   * @return 无返回值；不把任何故障都假定为仍具有水平返航能力，亦不恢复人工授权。
   */
  void fail_and_return(const std::string & reason)
  {
    // 本入口本身不会等待执行器收回；必须先收回的投放路径由专门清理分支约束。
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

  /**
   * @brief 锁存失败并停止位置目标发布，转入 LAND 或 ABORT。
   * @param reason 失败诊断文本。
   * @details 解锁状态下进入 LAND，未解锁则进入 ABORT；本函数不直接等待 ACK 或读取时钟。
   * @return 无返回值；只是安排下一阶段，不能据此宣布已落地、已上锁或机构已收回。
   */
  void fail_and_land(const std::string & reason)
  {
    mark_failure(reason);
    RCLCPP_ERROR(get_logger(), "MISSION_FAILURE: %s", reason.c_str());
    publish_setpoint_ = false;
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  /**
   * @brief 按本节点的全部软件终态条件只报告一次任务结果。
   * @details 无显式输入；检查失败标志、飞控已上锁、载荷计数、侦察完成及落地持续确认。
   * landing_confirmation_ready() 使用单调窗口；成功文本中的投放完成沿用后端软件结算语义。
   * @return 无返回值；设置 terminal_reported_ 并输出日志，不补充物品命中/分离的独立实测证明。
   */
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

  /**
   * @brief 统一切换任务阶段、登记单调驻留起点并发布状态名称。
   * @param next 下一阶段；与当前相同则无副作用直接返回。
   * @details 进入 LAND 清落地确认历史；进入 ALIGN 清粗下降、回退和设定点限速初始化等阶段历史。
   * 状态时间使用 SteadyClock，状态话题只含名称；本函数不自动检查迁移合法性或提交飞控服务。
   * @return 无返回值；调用方仍需配置相应目标、发布开关和在途命令清理。
   */
  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "STATE %s -> %s",
      state_name(state_).c_str(), state_name(next).c_str());
    if (next == State::LAND) {
      // 每次重新进入 LAND 都重建落地窗口，避免把先前一段候选历史挪来使用。
      reset_landing_confirmation();
    }
    if (next == State::ALIGN) {
      // 每个目标重新初始化下降/回退和限速起点，避免继承上一目标的缓存设定点。
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

  /**
   * @brief 将状态枚举映射到固定日志/话题名称。
   * @param state 要显示的任务阶段。
   * @return 对应英文状态标识；未覆盖的枚举值返回 UNKNOWN。
   * @note 纯映射，无时钟、单位或状态副作用；名称字符串保持接口兼容。
   */
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

  // 启动配置与模式选择：由 load_parameters() 缓存；布尔默认值不等于入口 YAML 的覆盖值。
  bool config_valid_ = false;  ///< 本节点已实现的配置约束是否满足，不是整机适航结论。
  bool flight_enable_ = false;
  bool lock_initial_heading_ = true;
  double configured_geographic_heading_deg_ = -1.0;
  bool yaw_to_target_ = false;
  bool auto_arm_on_guided_ = false;
  bool drop_only_mode_ = false;
  bool coarse_release_trial_mode_ = false;
  bool fixed_release_test_mode_ = false;
  bool fixed_release_after_search_ = false;
  bool fixed_release_direct_active_ = false;  ///< 当前任务已进入固定坐标投放支路。
  bool drop_phase_fixed_release_active_ = false;
  bool coarse_xy_filter_initialized_ = false;
  bool fallback_release_on_incomplete_bucket_set_ = false;
  bool pending_arm_value_ = false;
  bool guided_active_ = false;  ///< 最近飞控状态是否为 GUIDED，供准备与接管判断使用。
  bool measurement_time_sync_ = false;
  bool staged_release_gate_ = false;
  ReleaseGate release_gate_;  ///< 以新增视觉帧证据推进的分段投放门限。
  int64_t release_gate_start_ros_ns_ = 0;  ///< 门限开始的 ROS 时间（纳秒），不是单调计时。
  bool guided_ready_low_seen_ = false;
  // 时间同步：读取 MAVROS 收敛窗口后，累计可接受交换；“有offset”不等于已经同步。
  bool timesync_config_requested_ = false;
  bool timesync_config_valid_ = false;
  std::size_t timesync_good_samples_ = 0U;
  std::size_t timesync_window_ = 500U;
  double timesync_max_rtt_ms_ = 10.0;
  double timesync_max_deviation_ns_ = 10000000.0;
  uint64_t timesync_remote_ns_ = 0U;
  int64_t timesync_offset_ns_ = 0;
  SteadyTimePoint timesync_arrival_{};
  rclcpp::AsyncParametersClient::SharedPtr timesync_parameters_;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::deque<NavigationSample> attitude_history_;  ///< IMU 姿态历史，按 ROS 采样时间排序。
  // 输入存在性与运行阶段：have_* 避免将初始零值/初始化接收时刻误当成真实观测。
  bool have_odom_ = false;
  bool have_pose_ = false;
  bool have_compass_ = false;
  bool have_extended_state_ = false;
  bool have_vision_message_ = false;
  bool have_aligned_vision_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_ = false;  ///< 本节点当前是否允许发布自主位置参考。
  bool search_vision_acquire_pending_ = false;
  bool mission_started_ = false;
  bool takeoff_sent_ = false;
  bool target_plan_locked_ = false;
  bool mission_failed_ = false;
  bool recon_completed_ = false;
  bool terminal_reported_ = false;
  // 执行机构生命周期：已发送、收到应答、保持完成、回收完成分别记录，不能相互替代。
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
  bool release_actuation_committed_ = false;  ///< 已进入命令不可随视觉回调重定向的投放阶段。
  bool release_actuation_confirmed_ = false;  ///< 命令/服务层面已确认，不是瓶体脱离传感器反馈。
  bool release_cleanup_return_requested_ = false;

  // 当前位置、参考目标和冻结记忆均为本地 ENU 米制；home_ 未锁定时没有任务原点。
  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  mavros_msgs::msg::ExtendedState extended_state_;
  geometry_msgs::msg::Pose latest_local_pose_;
  Point3 position_;  ///< 最近里程计位置；使用前还需核对 odom_fresh()。
  Point3 target_;  ///< 当前请求的位置参考，不等于实测位置或机体已到达。
  Point3 search_vision_hold_position_;
  Point3 frozen_release_pose_;
  Point3 coarse_fallback_release_pose_;
  Point3 recon_entry_;
  std::optional<Point3> home_;
  Segment segment_;
  std::optional<BucketTrack> active_bucket_;  ///< 当前载荷目标；其观测年龄与轨迹身份另行判断。
  Point3 coarse_xy_filter_;
  // 各项“持续稳定”的起点使用单调时间；条件中断时必须重置对应 optional。
  std::optional<SteadyTimePoint> position_stable_since_;
  std::optional<SteadyTimePoint> ranking_stable_since_;
  std::optional<SteadyTimePoint> alignment_stable_since_;
  std::optional<SteadyTimePoint> recon_hold_since_;
  std::optional<SteadyTimePoint> landing_stable_since_;
  // 帧去重和导航插值使用采集 ROS 时间；等待队列另存到达时间来限制等待时长。
  std::optional<rclcpp::Time> last_queued_vision_stamp_;
  std::optional<rclcpp::Time> last_vision_capture_stamp_;
  std::deque<HeadingSample> heading_samples_;
  std::deque<NavigationSample> navigation_history_;
  std::deque<PendingVisionFrame> pending_vision_frames_;
  std::vector<BucketTrack> known_buckets_;  ///< 跨帧轨迹记忆；存在于此不表示仍是新鲜检测。
  std::vector<BucketTrack> latest_frame_detections_;
  std::vector<BucketTrack> search_observations_;
  std::vector<BucketTrack> clustered_target_tracks_;
  std::vector<Point3> released_positions_;
  std::vector<std::size_t> released_target_ids_;
  // 计划冻结后的载荷槽位顺序；最近邻重新关联可更新观测，但不应交换两个载荷槽位。
  std::vector<std::size_t> selected_target_ids_;
  std::vector<Point3> selected_target_positions_;
  std::vector<double> selected_target_diameters_;
  std::vector<double> selected_target_confidences_;
  std::vector<std::size_t> ranking_candidate_ids_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;
  std::vector<PendingServoCommand> pending_servo_commands_;  ///< 轮询 ACK 的异步命令集合。
  std::ofstream projection_log_;

  // 启动参数缓存：以下高度相对锁定起点，速度为参考轨迹限制；最终以实际加载值为准。
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
  double transit_speed_m_s_ = 8.5;  ///< 长距离转场参考速度，单位 m/s。
  double drop_target_transit_speed_m_s_ = 6.0;
  double recon_transit_speed_m_s_ = 9.5;
  double return_speed_m_s_ = 9.0;
  double return_alt_m_ = 2.5;
  double search_speed_m_s_ = 1.75;
  double search_forward_step_m_ = 2.5;
  double recon_speed_m_s_ = 1.5;
  double drop_approach_speed_m_s_ = 5.0;
  double descent_vertical_speed_m_s_ = 0.2;  ///< 下降参考的限速幅值，不是实际下降速度保证。
  double alignment_horizontal_speed_m_s_ = 0.5;
  double release_positioning_speed_m_s_ = 0.2;
  double release_stabilization_s_ = 2.0;
  double release_visual_xy_tolerance_m_ = 0.03;
  double middle_calibration_error_m_ = 0.12;
  double middle_calibration_stable_s_ = 0.50;
  double min_segment_s_ = 1.0;
  double accept_radius_m_ = 0.4;
  // 场地区域几何采用任务坐标（前、左、上）；发布前统一旋转平移到本地 ENU。
  double drop_near_x_m_ = 30.0;
  double drop_length_x_m_ = 5.0;
  double drop_width_y_m_ = 8.0;
  double cross_track_camera_fov_rad_ = 42.0 * kPi / 180.0;
  double search_lane_overlap_ratio_ = 0.30;
  double search_edge_margin_m_ = 0.35;
  double search_cross_margin_m_ = 0.55;
  double cluster_position_scale_m_ = 0.60;
  double cluster_diameter_scale_m_ = 0.06;
  std::vector<double> bucket_diameter_centers_m_ = {0.150, 0.200, 0.250};
  double bucket_diameter_class_tolerance_m_ = 0.015;
  double recon_center_x_m_ = 55.45;
  double recon_length_x_m_ = 2.4;
  double recon_width_y_m_ = 8.0;
  double recon_alt_m_ = 3.0;
  double recon_edge_margin_m_ = 0.35;
  double recon_cross_margin_m_ = 0.55;
  double recon_waypoint_hold_s_ = 1.0;
  // 轨迹关联、筛选和滤波门限：距离为米，时间为秒，alpha 与置信度为无量纲。
  double track_gate_m_ = 0.45;
  double diameter_track_gate_m_ = 0.08;
  double consecutive_track_position_gate_m_ = 0.50;
  double track_max_gap_s_ = 0.60;
  double selection_max_age_s_ = 10.0;
  double distinct_min_separation_m_ = 0.14;
  double distinct_min_diameter_m_ = 0.025;
  double ranking_stable_s_ = 0.80;
  double known_memory_s_ = 120.0;
  double position_filter_alpha_ = 0.25;
  std::size_t position_filter_window_ = 15U;
  int search_min_confirmations_ = 6;
  double search_max_position_deviation_m_ = 0.20;
  double body_filter_alpha_ = 1.0;
  double diameter_filter_alpha_ = 0.20;
  double confidence_filter_alpha_ = 0.30;
  double max_position_deviation_m_ = 0.25;
  double max_diameter_deviation_m_ = 0.050;
  double min_track_confidence_ = 0.25;
  double coarse_error_m_ = 0.15;
  double fine_error_m_ = 0.08;
  double coarse_stable_s_ = 0.8;
  double alignment_timeout_s_ = 18.0;
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
  // 状态预算、数据接收超时和服务应答等待均为秒；计时不依赖 ROS 校时是否跳变。
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
  // 最近导航/姿态测量：速度 m/s、角速度 rad/s、姿态与 ENU 航向 rad；罗盘保留度数。
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

  // 索引从零开始；载荷计数、航点进度与视觉帧序号各自独立，不能混作连续检测证据。
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
  // 各投放子流程的活动标志；同一 ALIGN / RELEASE 状态内仍需区分转场、下降及清理。
  bool drop_setpoint_initialized_ = false;
  bool use_post_search_cluster_fit_ = false;
  bool cluster_fit_plan_active_ = false;
  bool coarse_release_descent_active_ = false;
  bool coarse_coordinate_fallback_active_ = false;
  bool release_uses_coarse_fallback_ = false;
  bool middle_calibration_active_ = false;
  bool fixed_release_transit_active_ = false;
  bool inter_target_transit_active_ = false;
  bool recon_entry_transit_active_ = false;
  Point3 last_drop_setpoint_;
  Point3 last_visual_guidance_target_;
  SteadyTimePoint last_drop_setpoint_time_;
  std::vector<double> release_offsets_;  ///< 内部 FLU 外参，每个载荷依次存 x/y/z（米）。
  std::vector<double> release_offsets_frd_;  ///< 外部 FRD 原值，保留用于配置与诊断。
  std::vector<double> fixed_release_points_field_xy_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> stowed_pwm_;
  std::vector<int64_t> release_pwm_;
  std::vector<double> release_duration_s_;

  // 单调时间域：仅记录接收、状态、请求与参考轨迹的经过时长；不能与消息 header.stamp 相减。
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
  SteadyTimePoint control_rate_window_start_;
  SteadyTimePoint last_control_tick_;
  std::size_t control_tick_count_ = 0U;
  double control_max_gap_s_ = 0.0;
  double control_max_runtime_s_ = 0.0;

  // 持有 ROS 句柄以维持订阅、发布器、客户端与定时器的生命周期。
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
  // 仅在有效且 ready 时读取 future，周期轮询不会阻塞位置设定点发布。
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture virtual_release_future_;
};

#ifndef CUADC_TIMING_TEST
/** @brief 正常 ROS 入口；离线时间对齐测试定义 CUADC_TIMING_TEST 后复用类实现而排除此入口。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuadcFullMissionNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
