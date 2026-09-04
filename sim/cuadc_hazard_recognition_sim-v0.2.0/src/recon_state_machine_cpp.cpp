/**
 * @file recon_state_machine_cpp.cpp
 * @brief 基于 ROS 2、MAVROS 与视觉检测的无人机未知目标覆盖搜索状态机。
 *
 * 程序总体流程：
 * 1. 等待飞控连接、MAVROS 本地位置和可用定位来源；
 * 2. 切换 GUIDED 模式、解锁并起飞到指定高度；
 * 3. 仅根据公开的 recognition_area 元数据规划“割草式”覆盖航线；
 * 4. 飞行过程中接收视觉检测结果，短暂停留并融合多帧证据；
 * 5. 对空间上重复的目标观测进行合并；
 * 6. 完成覆盖后返航、降落，并将观测结果写入 JSON 文件。
 *
 * 重要设计约束：
 * - 状态机不会读取随机生成目标的真实位置，避免使用 ground truth 泄漏答案；
 * - 飞行高度始终使用 MAVROS 本地坐标系，RTK 高程只用于诊断；
 * - RTK 仅建立水平坐标变换；RTK 不可用时，可按配置回退到出生点相对坐标；
 * - 图像目标定位采用固定相机地面覆盖范围近似，默认假设地面近似平坦。
 */

// ROS 2 节点、定时器、时间、日志和通信接口。
#include <rclcpp/rclcpp.hpp>

// 项目自定义视觉检测消息，以及 MAVROS/标准 ROS 消息与服务。
#include <cuadc_hazard_recognition_sim/msg/hazard_detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// 用于读取仿真场景公开元数据。
#include <yaml-cpp/yaml.h>

// C++ 标准库：数值计算、容器、异步服务结果和文件输出。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// 允许直接使用 50ms、0s 等 chrono 时间字面量。
using namespace std::chrono_literals;
using HazardDetection = cuadc_hazard_recognition_sim::msg::HazardDetection;

/**
 * @brief 飞行任务有限状态机。
 *
 * WAITING_FCU      等待飞控、本地位置和定位源；
 * SETTING_GUIDED   请求进入 GUIDED 模式；
 * ARMING           请求解锁；
 * TAKEOFF          调用起飞服务并等待达到目标高度；
 * SCANNING         执行覆盖航线并处理视觉观测；
 * RETURN_HOME      返回起飞点上方；
 * LANDING          调用降落服务；
 * DONE             写出结果并结束节点。
 */
enum class MissionState {
  WAITING_FCU, SETTING_GUIDED, ARMING, TAKEOFF, SCANNING, RETURN_HOME, LANDING, DONE
};

/** @brief 三维点/位置。x、y、z 的具体坐标系由使用场景决定。 */
struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/** @brief 公开搜索区域描述，同时保存 world 与 RTK ENU 两套中心坐标。 */
struct SearchArea {
  // 仿真世界坐标系中的区域中心。
  Point3 center_world;
  // 以 RTK 基站为原点的 ENU 区域中心。
  Point3 center_rtk_enu;
  // 搜索区域沿自身 x、y 方向的尺寸。
  double size_x = 0.0;
  double size_y = 0.0;
  // 区域局部坐标相对 RTK ENU 的水平旋转角，单位 rad。
  double world_to_rtk_yaw = 0.0;
  // 元数据是否完整且区域尺寸合法。
  bool valid = false;
};

/** @brief 覆盖航点及其所属扫描航带编号。 */
struct CoverageWaypoint {
  Point3 point;
  std::size_t lane_index = 0;
};

/** @brief 已确认目标观测。可由多个空间邻近视角融合得到。 */
struct Observation {
  // 融合后采用的类别名称。
  std::string predicted;
  // 当前保留类别的融合置信度。
  double confidence = 0.0;
  // 被合并到该观测中的独立确认次数。
  std::size_t merged_views = 1;
  // MAVROS 本地坐标系中的目标估计位置。
  Point3 local_position;
  // RTK 基站 ENU 坐标；仅在 RTK 变换已锁定时有效。
  Point3 rtk_enu;
  // 是否包含有效的 rtk_enu。
  bool has_rtk = false;
  // 记录该观测时所处的覆盖航点序号。
  std::size_t route_index = 0;
};

/** @brief 一段按时间参数化的平滑点到点轨迹。 */
struct Segment {
  Point3 start;
  Point3 end;
  double duration_s = 1.0;
  rclcpp::Time start_time;
};

/**
 * @brief 无人机侦察覆盖搜索节点。
 *
 * 节点以 20 Hz 执行控制循环，通过位置设定点驱动无人机，
 * 并通过 MAVROS 服务完成模式切换、解锁、起飞和降落。
 */
class ReconStateMachineCpp : public rclcpp::Node {
public:
/** @brief 声明并读取参数、加载场景、建立 ROS 接口和控制定时器。 */
  ReconStateMachineCpp() : Node("recon_state_machine_cpp") {
    // ---------------- 飞行与轨迹参数 ----------------
    // 相对起飞点的巡航/扫描高度，单位 m。
    declare_parameter<double>("takeoff_alt", 1.5);
    // 判定到达航点的三维距离阈值，单位 m。
    declare_parameter<double>("waypoint_accept_radius", 0.30);
    // 起飞阶段最长等待时间，超时后仍会进入覆盖任务。
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    // 覆盖任务总超时时间，超时后强制返航。
    declare_parameter<double>("mission_timeout_s", 300.0);
    // 计算轨迹段持续时间时使用的期望平均速度，单位 m/s。
    declare_parameter<double>("trajectory_speed", 0.65);
    // 单段轨迹的最短持续时间，防止短距离指令变化过快。
    declare_parameter<double>("min_segment_time_s", 1.5);
    // ---------------- 覆盖航线参数 ----------------
    // 相邻扫描航带的间距，单位 m。
    declare_parameter<double>("coverage_lane_spacing_m", 0.70);
    // 覆盖航线相对区域 x 边界的安全内缩距离。
    declare_parameter<double>("coverage_edge_inset_x_m", 0.40);
    // 覆盖航线相对区域 y 边界的安全内缩距离。
    declare_parameter<double>("coverage_edge_inset_y_m", 0.65);
    // ---------------- 视觉确认与观测融合参数 ----------------
    // 无人机到达目标投影位置后，累计多帧证据的停留时间。
    declare_parameter<double>("detection_pause_s", 3.0);
    // 尝试移动到目标投影位置的最长时间。
    declare_parameter<double>("detection_centering_timeout_s", 5.0);
    // 高置信确认窗口要求的平均类别置信度。
    declare_parameter<double>("visual_min_confidence", 0.25);
    // 高置信确认窗口要求的最少有效帧数。
    declare_parameter<int>("visual_min_consecutive", 3);
    // 弱置信确认窗口允许的最低平均置信度。
    declare_parameter<double>("visual_weak_confidence", 0.05);
    // 弱置信目标需要更多帧才能确认。
    declare_parameter<int>("visual_weak_min_consecutive", 8);
    // 检测中心允许偏离图像中心的归一化比例；用于过滤边缘检测。
    declare_parameter<double>("visual_center_gate_fraction", 0.90);
    // 不区分类别时，判定两次观测为空间重复的半径。
    declare_parameter<double>("observation_merge_radius_m", 0.65);
    // 类别相同时允许使用更大的观测合并半径。
    declare_parameter<double>("observation_same_class_merge_radius_m", 0.85);
    // ---------------- 相机几何近似参数 ----------------
    // 输入图像宽度和高度，单位 pixel。
    declare_parameter<int>("camera_width", 848);
    // 输入图像高度，单位 pixel。
    declare_parameter<int>("camera_height", 480);
    // 当前飞行高度下，图像在地面 x/y 方向对应的近似覆盖尺寸。
    declare_parameter<double>("camera_footprint_x_m", 1.02);
    declare_parameter<double>("camera_footprint_y_m", 1.80);
    // 图像坐标投影到本地坐标时的水平旋转补偿。
    declare_parameter<double>("camera_image_yaw_deg", 0.0);
    // ---------------- 场景、定位与结果参数 ----------------
    // 仅包含公开场景元数据的 YAML 文件路径。
    declare_parameter<std::string>("generated_scene_path", "");
    // 定位模式：rtk、spawn_relative 或 auto_rtk。
    declare_parameter<std::string>("coordinate_source", "auto_rtk");
    // auto_rtk 模式等待 RTK 的时间，超时后使用出生点相对坐标。
    declare_parameter<double>("rtk_fix_timeout_s", 8.0);
    // RTK ENU 到 MAVROS 本地 ENU 的水平偏航角补偿。
    declare_parameter<double>("rtk_to_mavros_yaw_deg", 0.0);
    // 任务结束后写出的 JSON 结果文件路径。
    declare_parameter<std::string>(
      "result_path", "/tmp/cuadc_hazard_search_observations.json");

    // 使用统一辅助函数读取参数，并通过 max/clamp 对关键参数做下限或范围约束。
    const auto gp = [this](const char * name) {return get_parameter(name);};
    takeoff_alt_ = gp("takeoff_alt").as_double();
    accept_radius_ = gp("waypoint_accept_radius").as_double();
    takeoff_timeout_s_ = gp("takeoff_timeout_s").as_double();
    mission_timeout_s_ = gp("mission_timeout_s").as_double();
    trajectory_speed_ = std::max(0.1, gp("trajectory_speed").as_double());
    min_segment_time_s_ = std::max(0.5, gp("min_segment_time_s").as_double());
    lane_spacing_ = std::max(0.1, gp("coverage_lane_spacing_m").as_double());
    edge_inset_x_ = std::max(0.0, gp("coverage_edge_inset_x_m").as_double());
    edge_inset_y_ = std::max(0.0, gp("coverage_edge_inset_y_m").as_double());
    detection_pause_s_ = std::max(0.5, gp("detection_pause_s").as_double());
    detection_centering_timeout_s_ = std::max(
      1.0, gp("detection_centering_timeout_s").as_double());
    visual_min_confidence_ = std::clamp(
      gp("visual_min_confidence").as_double(), 0.0, 1.0);
    visual_min_consecutive_ = std::max(
      1, static_cast<int>(gp("visual_min_consecutive").as_int()));
    visual_weak_confidence_ = std::clamp(
      gp("visual_weak_confidence").as_double(), 0.0, visual_min_confidence_);
    visual_weak_min_consecutive_ = std::max(
      visual_min_consecutive_ + 1,
      static_cast<int>(gp("visual_weak_min_consecutive").as_int()));
    visual_center_gate_fraction_ = std::clamp(
      gp("visual_center_gate_fraction").as_double(), 0.05, 1.0);
    observation_merge_radius_ = std::max(
      0.1, gp("observation_merge_radius_m").as_double());
    observation_same_class_merge_radius_ = std::max(
      observation_merge_radius_,
      gp("observation_same_class_merge_radius_m").as_double());
    camera_width_ = std::max(1, static_cast<int>(gp("camera_width").as_int()));
    camera_height_ = std::max(1, static_cast<int>(gp("camera_height").as_int()));
    camera_footprint_x_ = std::max(0.1, gp("camera_footprint_x_m").as_double());
    camera_footprint_y_ = std::max(0.1, gp("camera_footprint_y_m").as_double());
    camera_image_yaw_ = gp("camera_image_yaw_deg").as_double() * M_PI / 180.0;
    generated_scene_path_ = gp("generated_scene_path").as_string();
    coordinate_source_ = gp("coordinate_source").as_string();
    rtk_fix_timeout_s_ = std::max(0.0, gp("rtk_fix_timeout_s").as_double());
    rtk_to_local_yaw_ = gp("rtk_to_mavros_yaw_deg").as_double() * M_PI / 180.0;
    result_path_ = gp("result_path").as_string();

    // 场景必须提供 recognition_area；目标真实位置不会被状态机读取。
    load_public_scene_metadata();
    if (!search_area_.valid) {
      throw std::runtime_error(
              "recognition_area metadata is required; bucket positions are intentionally not accepted");
    }

    // 飞控状态使用可靠、Transient Local QoS；传感器数据使用低延迟 SensorDataQoS。
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto sensor_qos = rclcpp::SensorDataQoS();
    // 订阅飞控连接、模式和解锁状态。
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", state_qos,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {current_state_ = *msg;});
    // 订阅 MAVROS 本地里程计，作为飞行控制的主位置来源。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", sensor_qos,
      std::bind(&ReconStateMachineCpp::odom_cb, this, std::placeholders::_1));
    // 订阅全球定位结果，用于建立 RTK ENU 与本地坐标的水平对齐。
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", sensor_qos,
      std::bind(&ReconStateMachineCpp::fix_cb, this, std::placeholders::_1));
    // 订阅感知节点输出的类别、置信度和图像中心位置。
    detection_sub_ = create_subscription<HazardDetection>(
      "/perception/hazard_detection", 10,
      std::bind(&ReconStateMachineCpp::detection_cb, this, std::placeholders::_1));
    // 持续发布本地位置设定点，供 MAVROS/飞控跟踪。
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());
    // MAVROS 飞行控制服务客户端。
    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    // 初始化状态计时，并以 50 ms 周期运行控制循环，即 20 Hz。
    mission_start_time_ = now();
    state_enter_time_ = now();
    last_state_log_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&ReconStateMachineCpp::control_loop, this));
    RCLCPP_INFO(
      get_logger(),
      "Unknown-target coverage state machine: area=%.2fx%.2f m coordinates=%s; "
      "random bucket positions are not loaded",
      search_area_.size_x, search_area_.size_y, coordinate_source_.c_str());
  }

private:
  // ============================================================================
  // ROS 回调与定位初始化
  // ============================================================================
  /**
   * @brief 更新 MAVROS 本地位置，并在任务开始前锁定返航原点。
   * @param msg MAVROS 本地里程计消息。
   */
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = Point3{
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    have_local_position_ = true;
    // 只在等待阶段记录一次 home，避免飞行中漂移导致返航点改变。
    if (!home_.has_value() && mission_state_ == MissionState::WAITING_FCU) {
      home_ = current_position_;
      RCLCPP_INFO(
        get_logger(), "MAVROS local origin locked at (%.3f, %.3f, %.3f)",
        home_->x, home_->y, home_->z);
    }
    try_lock_rtk();
  }

  /**
   * @brief 校验并保存最新 GNSS/RTK 定位结果。
   *
   * 无有效 fix、经纬高非有限数或经纬度越界时直接丢弃。
   */
  void fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude) ||
      std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0)
    {
      return;
    }
    latest_fix_ = *msg;
    have_valid_fix_ = true;
    try_lock_rtk();
  }

  /**
   * @brief 在 RTK、MAVROS 本地位置和基站参考均可用时锁定坐标变换。
   *
   * 锁定瞬间建立一对锚点：当前 RTK ENU 位置与当前 MAVROS 本地位置。
   * 后续只做二维旋转和平移，不使用 RTK 高程控制飞行高度。
   */
  void try_lock_rtk() {
    if (rtk_locked_ || !have_valid_fix_ || !have_local_position_ || !have_rtk_reference_) {
      return;
    }
    // 将当前经纬高转换成以 YAML 中 RTK 基站为原点的 ENU 坐标。
    rtk_rover_anchor_ = geodetic_to_rtk_enu(
      latest_fix_.latitude, latest_fix_.longitude, latest_fix_.altitude);
    mavros_local_anchor_ = current_position_;
    rtk_locked_ = true;
    using_spawn_fallback_ = false;
    RCLCPP_INFO(
      get_logger(),
      "RTK horizontal transform locked: rover EN=(%.3f, %.3f), local EN=(%.3f, %.3f), "
      "yaw=%.2f deg; RTK up diagnostic=%.3f m is not used for flight altitude",
      rtk_rover_anchor_.x, rtk_rover_anchor_.y,
      mavros_local_anchor_.x, mavros_local_anchor_.y,
      rtk_to_local_yaw_ * 180.0 / M_PI, rtk_rover_anchor_.z);
  }

  // ============================================================================
  // 视觉目标定位、候选证据累计与观测融合
  // ============================================================================

  /** @brief 清空当前视觉候选的类别、帧数和位置累计量。 */
  void reset_candidate() {
    candidate_class_scores_.clear();
    candidate_class_peaks_.clear();
    candidate_frame_count_ = 0;
    candidate_position_samples_ = 0;
    candidate_position_ = Point3{};
  }

  /**
   * @brief 根据检测框中心估算目标在 MAVROS 本地平面中的位置。
   *
   * 该模型把图像中心偏移按固定 camera_footprint 映射到地面位移，
   * 再施加 camera_image_yaw 旋转。它不使用深度，因此要求相机姿态、
   * 飞行高度和地面起伏与标定假设基本一致。
   */
  Point3 estimate_object_local(const HazardDetection & msg) const {
    // 图像 v 方向映射到机体/本地 x；负号来自图像坐标向下为正。
    const double image_dx =
      -(static_cast<double>(msg.center_v) - camera_height_ * 0.5) /
      static_cast<double>(camera_height_) * camera_footprint_x_;
    // 图像 u 方向映射到机体/本地 y；负号用于匹配当前相机安装约定。
    const double image_dy =
      -(static_cast<double>(msg.center_u) - camera_width_ * 0.5) /
      static_cast<double>(camera_width_) * camera_footprint_y_;
    // 将图像平面位移旋转到 MAVROS 本地水平坐标系。
    const double c = std::cos(camera_image_yaw_);
    const double s = std::sin(camera_image_yaw_);
    return Point3{
      current_position_.x + c * image_dx - s * image_dy,
      current_position_.y + s * image_dx + c * image_dy,
      current_position_.z};
  }

  /**
   * @brief 判断新估计位置是否靠近已记录目标。
   *
   * 任意类别使用基础合并半径；类别相同时允许使用更宽松的半径。
   */
  bool near_recorded_observation(
    const Point3 & position, const std::string & predicted) const
  {
    return std::any_of(
      observations_.begin(), observations_.end(),
      [&](const Observation & observation) {
        const double distance = distance_xy(position, observation.local_position);
        return distance <= observation_merge_radius_ ||
               (predicted == observation.predicted &&
               distance <= observation_same_class_merge_radius_);
      });
  }

  /**
   * @brief 新增目标观测，或与空间上已有观测进行增量融合。
   *
   * 位置使用按确认次数加权的均值；类别和置信度保留置信度更高的一次。
   */
  void record_observation(
    const std::string & predicted, double confidence, const Point3 & local_position)
  {
    // 先查找空间上可视为同一目标的已有记录。
    auto existing = std::find_if(
      observations_.begin(), observations_.end(),
      [&](const Observation & observation) {
        const double distance = distance_xy(local_position, observation.local_position);
        return distance <= observation_merge_radius_ ||
               (predicted == observation.predicted &&
               distance <= observation_same_class_merge_radius_);
      });
    if (existing != observations_.end()) {
      // 使用 merged_views 做在线均值更新，无需保存全部历史位置。
      const double old_views = static_cast<double>(existing->merged_views);
      const double new_views = old_views + 1.0;
      existing->local_position.x =
        (existing->local_position.x * old_views + local_position.x) / new_views;
      existing->local_position.y =
        (existing->local_position.y * old_views + local_position.y) / new_views;
      existing->local_position.z = local_position.z;
      ++existing->merged_views;
      // 只有新证据更可信时，才更新该观测的最终类别和置信度。
      if (confidence > existing->confidence) {
        existing->predicted = predicted;
        existing->confidence = confidence;
      }
      existing->route_index = scan_index_;
      if (rtk_locked_) {
        existing->rtk_enu = local_to_rtk_enu(existing->local_position);
        existing->has_rtk = true;
      }
      RCLCPP_INFO(
        get_logger(),
        "Merged repeated spatial observation: class=%s confidence=%.3f views=%zu",
        existing->predicted.c_str(), existing->confidence, existing->merged_views);
      return;
    }

    Observation observation;
    observation.predicted = predicted;
    observation.confidence = confidence;
    observation.local_position = local_position;
    observation.route_index = scan_index_;
    if (rtk_locked_) {
      observation.rtk_enu = local_to_rtk_enu(local_position);
      observation.has_rtk = true;
    }
    observations_.push_back(observation);
    RCLCPP_INFO(
      get_logger(),
      "Recorded unknown-target observation %zu: class=%s confidence=%.3f "
      "local=(%.2f,%.2f) route=%zu/%zu",
      observations_.size(), predicted.c_str(), confidence,
      local_position.x, local_position.y, scan_index_ + 1, scan_route_.size());
  }

  /**
   * @brief 处理视觉检测消息，并驱动“发现—居中—停留—融合”流程。
   *
   * 回调只在扫描状态且识别门已开启时生效。目标需要通过置信度、
   * 图像中心和空间重复性门控，之后才会中断覆盖航线进行确认。
   */
  void detection_cb(const HazardDetection::SharedPtr msg) {
    // 起飞和进入区域前不接受检测，避免把区域外物体记入结果。
    if (mission_state_ != MissionState::SCANNING || !coverage_detection_enabled_ ||
      scan_index_ >= scan_route_.size())
    {
      return;
    }
    // 空类别或连最弱证据阈值都未达到的检测直接丢弃。
    if (msg->class_name.empty() || msg->confidence < visual_weak_confidence_) {
      return;
    }
    // 将检测中心到图像中心的偏差归一化到 [0, 1] 附近。
    const double du =
      std::abs(msg->center_u - camera_width_ * 0.5) / (camera_width_ * 0.5);
    const double dv =
      std::abs(msg->center_v - camera_height_ * 0.5) / (camera_height_ * 0.5);
    // 过滤图像极边缘目标，降低固定地面投影模型的误差。
    if (du > visual_center_gate_fraction_ || dv > visual_center_gate_fraction_) {
      return;
    }

    // 将当前检测投影到 MAVROS 本地坐标。
    const Point3 estimated_position = estimate_object_local(*msg);
    // 尚未进入确认流程时，忽略已经记录过的空间重复目标。
    if (!detection_hold_active_ &&
      near_recorded_observation(estimated_position, msg->class_name))
    {
      return;
    }

    // 首次看到潜在新目标：暂停原航线，并把目标投影位置设为悬停点。
    if (!detection_hold_active_) {
      reset_candidate();
      detection_hold_active_ = true;
      detection_evidence_active_ = false;
      detection_hold_position_ = Point3{
        estimated_position.x, estimated_position.y,
        home_.value_or(Point3{}).z + takeoff_alt_};
      detection_centering_start_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Potential target seen during coverage; visually centering on projected "
        "local position (%.2f,%.2f) before evidence accumulation",
        detection_hold_position_.x, detection_hold_position_.y);
      return;
    }
    // 无人机尚未到达悬停点时，只平滑修正目标位置，不累计分类证据。
    if (!detection_evidence_active_) {
      if (distance_xy(estimated_position, detection_hold_position_) <=
        observation_same_class_merge_radius_)
      {
        // 低通更新悬停点，减少逐帧检测中心抖动造成的设定点跳变。
        detection_hold_position_.x =
          0.8 * detection_hold_position_.x + 0.2 * estimated_position.x;
        detection_hold_position_.y =
          0.8 * detection_hold_position_.y + 0.2 * estimated_position.y;
      }
      return;
    }
    // 证据累计期间若新位置偏差过大，认为不是同一目标并丢弃该帧。
    if (candidate_position_samples_ > 0 &&
      distance_xy(estimated_position, candidate_position_) > observation_merge_radius_)
    {
      return;
    }

    // 对目标位置进行在线均值融合。
    ++candidate_frame_count_;
    ++candidate_position_samples_;
    const double samples = static_cast<double>(candidate_position_samples_);
    candidate_position_.x =
      candidate_position_.x * ((samples - 1.0) / samples) +
      estimated_position.x / samples;
    candidate_position_.y =
      candidate_position_.y * ((samples - 1.0) / samples) +
      estimated_position.y / samples;
    candidate_position_.z = estimated_position.z;

    // 优先累计完整类别分布；若消息未提供分布，则退化为只累计 top-1。
    if (!msg->class_names.empty() &&
      msg->class_names.size() == msg->class_confidences.size())
    {
      for (std::size_t i = 0; i < msg->class_names.size(); ++i) {
        const double score =
          std::clamp(static_cast<double>(msg->class_confidences[i]), 0.0, 1.0);
        candidate_class_scores_[msg->class_names[i]] += score;
        candidate_class_peaks_[msg->class_names[i]] =
          std::max(candidate_class_peaks_[msg->class_names[i]], score);
      }
    } else {
      const double score = std::clamp(static_cast<double>(msg->confidence), 0.0, 1.0);
      candidate_class_scores_[msg->class_name] += score;
      candidate_class_peaks_[msg->class_name] =
        std::max(candidate_class_peaks_[msg->class_name], score);
    }
  }

  /**
   * @brief 根据累计帧数和平均置信度判断候选目标是否成立。
   * @return true 表示确认并记录目标，false 表示证据不足。
   *
   * 提供两条确认路径：少量高置信帧，或更多低置信但持续稳定的帧。
   */
  bool finalize_detection_candidate() {
    if (candidate_frame_count_ <= 0 || candidate_class_scores_.empty()) {
      return false;
    }
    // 选择累计类别得分最高的类别作为融合结果。
    const auto best = std::max_element(
      candidate_class_scores_.begin(), candidate_class_scores_.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;});
    const double average_score =
      best->second / static_cast<double>(candidate_frame_count_);
    // 高置信窗口：帧数较少，但平均分必须达到主阈值。
    const bool high_confirmation =
      candidate_frame_count_ >= visual_min_consecutive_ &&
      average_score >= visual_min_confidence_;
    // 弱置信窗口：允许较低平均分，但要求更长时间的一致观测。
    const bool weak_confirmation =
      candidate_frame_count_ >= visual_weak_min_consecutive_ &&
      average_score >= visual_weak_confidence_;
    if (!high_confirmation && !weak_confirmation) {
      RCLCPP_INFO(
        get_logger(),
        "Detection evidence rejected: best=%s average=%.3f frames=%d",
        best->first.c_str(), average_score, candidate_frame_count_);
      return false;
    }

    const double peak = candidate_class_peaks_[best->first];
    RCLCPP_INFO(
      get_logger(),
      "Coverage visual confirmation: class=%s average=%.3f peak=%.3f "
      "frames=%d mode=%s",
      best->first.c_str(), average_score, peak, candidate_frame_count_,
      high_confirmation ? "high-window" : "weak-window");
    record_observation(best->first, average_score, candidate_position_);
    return true;
  }

  // ============================================================================
  // 主任务状态机
  // ============================================================================

  /** @brief 20 Hz 主控制循环，更新服务结果、设定点、超时和任务状态。 */
  void control_loop() {
    // 异步服务不会阻塞控制线程；每周期轮询是否已返回。
    check_service_results();
    // 除起飞/降落服务阶段外，持续发送位置设定点。
    if (publish_setpoints_) publish_setpoint(target_);
    // 限频输出状态日志，避免 20 Hz 控制循环刷屏。
    if (seconds_since(last_state_log_) > 4.0) {
      last_state_log_ = now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s fcu=%s pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) "
        "localization=%s coverage=%zu/%zu detections=%zu",
        state_name(mission_state_).c_str(),
        current_state_.connected ? "connected" : "disconnected",
        current_position_.x, current_position_.y, current_position_.z,
        target_.x, target_.y, target_.z,
        rtk_locked_ ? "RTK" : (using_spawn_fallback_ ? "SPAWN_FALLBACK" : "WAITING_RTK"),
        scan_index_, scan_route_.size(), observations_.size());
    }

    // 全局安全超时：无论扫描进度如何，都转入返航流程。
    if (mission_started_ && seconds_since(mission_start_time_) > mission_timeout_s_ &&
      mission_state_ != MissionState::RETURN_HOME &&
      mission_state_ != MissionState::LANDING && mission_state_ != MissionState::DONE)
    {
      RCLCPP_WARN(get_logger(), "Coverage mission timeout; returning home");
      start_return_home();
    }

    // 状态转换由飞控反馈、位置到达判定和计时条件共同触发。
    switch (mission_state_) {
      // 等待飞控连接、本地原点和指定定位来源全部就绪。
      case MissionState::WAITING_FCU:
        update_localization_fallback();
        if (current_state_.connected && home_.has_value() && localization_ready()) {
          enter_state(MissionState::SETTING_GUIDED);
        }
        break;
      // 未进入 GUIDED 时周期性重试模式切换服务。
      case MissionState::SETTING_GUIDED:
        if (current_state_.mode == "GUIDED") {
          enter_state(MissionState::ARMING);
        } else if (!pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;
      // 未解锁时周期性重试解锁服务。
      case MissionState::ARMING:
        if (current_state_.armed) {
          enter_state(MissionState::TAKEOFF);
        } else if (!pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;
      // 起飞由 CommandTOL 服务控制，期间暂不发布位置设定点。
      case MissionState::TAKEOFF:
        publish_setpoints_ = false;
        if (!takeoff_sent_) {
          call_takeoff(takeoff_alt_);
          takeoff_sent_ = true;
          state_enter_time_ = now();
        }
        if (relative_altitude() >= takeoff_alt_ * 0.90 ||
          seconds_since(state_enter_time_) > takeoff_timeout_s_)
        {
          publish_setpoints_ = true;
          start_coverage_scan();
        }
        break;
      // 执行覆盖轨迹，必要时插入视觉居中和停留确认。
      case MissionState::SCANNING:
        update_coverage_trajectory();
        break;
      // 沿平滑轨迹返回 home 上方的巡航高度。
      case MissionState::RETURN_HOME:
        update_return_trajectory();
        break;
      // 降落服务只发送一次；高度接近 home 后进入 DONE。
      case MissionState::LANDING:
        publish_setpoints_ = false;
        if (!land_sent_) {
          call_land();
          land_sent_ = true;
        }
        if (relative_altitude() < 0.20) {
          enter_state(MissionState::DONE);
        }
        break;
      // 结果只写一次，然后关闭 ROS 2。
      case MissionState::DONE:
        if (!result_written_) {
          write_observations();
          result_written_ = true;
          RCLCPP_INFO(
            get_logger(), "Coverage mission complete; raw observations=%s",
            result_path_.c_str());
          rclcpp::shutdown();
        }
        break;
    }
  }

  // ============================================================================
  // 定位模式选择与场景元数据
  // ============================================================================

  /**
   * @brief auto_rtk 模式下，在 RTK 等待超时后启用出生点相对坐标。
   *
   * coordinate_source=rtk 时禁止回退，必须等待 RTK 锁定。
   */
  void update_localization_fallback() {
    if (rtk_locked_ || using_spawn_fallback_ || coordinate_source_ == "rtk") {
      return;
    }
    if (!current_state_.connected || !have_local_position_) {
      localization_wait_start_.reset();
      return;
    }
    if (!localization_wait_start_.has_value()) {
      localization_wait_start_ = now();
      return;
    }
    if (seconds_since(*localization_wait_start_) >= rtk_fix_timeout_s_) {
      using_spawn_fallback_ = true;
      RCLCPP_WARN(
        get_logger(),
        "No valid RTK fix %.1f s after FCU/local-pose readiness; using explicit spawn-relative fallback",
        rtk_fix_timeout_s_);
    }
  }

  /** @brief 根据 coordinate_source 判断任务是否具备可用定位。 */
  bool localization_ready() const {
    if (coordinate_source_ == "spawn_relative") {
      return true;
    }
    if (coordinate_source_ == "rtk") {
      return rtk_locked_;
    }
    return rtk_locked_ || using_spawn_fallback_;
  }

  /**
   * @brief 从 YAML 加载 RTK 基站、飞行器出生点和公开识别区域。
   *
   * 本函数有意不读取 recon_targets，确保路径规划和识别过程不知道
   * 随机目标的 ground-truth 位置。
   */
  void load_public_scene_metadata() {
    if (generated_scene_path_.empty()) {
      return;
    }
    try {
      // YAML 加载失败或字段类型异常时由 catch 统一记录错误。
      const YAML::Node scene = YAML::LoadFile(generated_scene_path_);
      scene_seed_ = scene["seed"] ? scene["seed"].as<int>() : 0;
      // RTK 基站经纬高用于建立 ECEF/ENU 参考原点。
      const auto rtk = scene["rtk_base"];
      if (rtk) {
        rtk_base_lat_ = rtk["latitude_deg"].as<double>();
        rtk_base_lon_ = rtk["longitude_deg"].as<double>();
        rtk_base_alt_ = rtk["altitude_m"].as<double>();
        have_rtk_reference_ = true;
      }
      // 出生点用于 spawn_relative 模式下把世界坐标换算到本地坐标。
      const auto vehicle = scene["vehicle"];
      if (vehicle && vehicle["pose"] && vehicle["pose"].size() >= 2) {
        vehicle_spawn_world_ = Point3{
          vehicle["pose"][0].as<double>(), vehicle["pose"][1].as<double>(), 0.0};
      }
      // recognition_area 是状态机允许使用的公开搜索边界。
      const auto area = scene["recognition_area"];
      if (area && area["center_world"] && area["size"] &&
        area["center_rtk_enu"] && area["center_world"].size() >= 2 &&
        area["size"].size() >= 2 && area["center_rtk_enu"].size() >= 2)
      {
        search_area_.center_world = Point3{
          area["center_world"][0].as<double>(), area["center_world"][1].as<double>(), 0.0};
        search_area_.center_rtk_enu = Point3{
          area["center_rtk_enu"][0].as<double>(),
          area["center_rtk_enu"][1].as<double>(), 0.0};
        search_area_.size_x = area["size"][0].as<double>();
        search_area_.size_y = area["size"][1].as<double>();
        search_area_.world_to_rtk_yaw =
          area["world_to_rtk_yaw_deg"].as<double>() * M_PI / 180.0;
        search_area_.valid = search_area_.size_x > 0.0 && search_area_.size_y > 0.0;
      }
      // Deliberately do not read scene["recon_targets"]. Those positions are
      // randomized ground truth and are reserved for post-mission evaluation.
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Cannot load public scene metadata %s: %s",
        generated_scene_path_.c_str(), ex.what());
    }
  }

  // ============================================================================
  // 坐标转换
  // ============================================================================

  /**
   * @brief 将 WGS-84 经纬高转换为以 RTK 基站为原点的局部 ENU。
   *
   * 先执行 Geodetic -> ECEF，再将 ECEF 差向量旋转到 East/North/Up。
   */
  Point3 geodetic_to_rtk_enu(double lat, double lon, double alt) const {
    // WGS-84 椭球长半轴、扁率和第一偏心率平方。
    constexpr double semi_major = 6378137.0;
    constexpr double flattening = 1.0 / 298.257223563;
    constexpr double eccentricity_sq = flattening * (2.0 - flattening);
    const auto to_ecef = [](double latitude_deg, double longitude_deg, double height) {
      const double latitude = latitude_deg * M_PI / 180.0;
      const double longitude = longitude_deg * M_PI / 180.0;
      const double sin_lat = std::sin(latitude);
      const double prime_vertical =
        semi_major / std::sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat);
      return Point3{
        (prime_vertical + height) * std::cos(latitude) * std::cos(longitude),
        (prime_vertical + height) * std::cos(latitude) * std::sin(longitude),
        (prime_vertical * (1.0 - eccentricity_sq) + height) * sin_lat};
    };
    // 用 rover 与 base 的 ECEF 差计算局部切平面坐标。
    const Point3 base = to_ecef(rtk_base_lat_, rtk_base_lon_, rtk_base_alt_);
    const Point3 rover = to_ecef(lat, lon, alt);
    const double dx = rover.x - base.x;
    const double dy = rover.y - base.y;
    const double dz = rover.z - base.z;
    const double lat0 = rtk_base_lat_ * M_PI / 180.0;
    const double lon0 = rtk_base_lon_ * M_PI / 180.0;
    return Point3{
      -std::sin(lon0) * dx + std::cos(lon0) * dy,
      -std::sin(lat0) * std::cos(lon0) * dx -
        std::sin(lat0) * std::sin(lon0) * dy + std::cos(lat0) * dz,
      std::cos(lat0) * std::cos(lon0) * dx +
        std::cos(lat0) * std::sin(lon0) * dy + std::sin(lat0) * dz};
  }

  /** @brief 将 RTK ENU 水平位置转换到 MAVROS 本地坐标。 */
  Point3 rtk_to_local(const Point3 & rtk) const {
    // 先相对 RTK 锚点平移，再施加配置的偏航旋转，最后加本地锚点。
    const double east = rtk.x - rtk_rover_anchor_.x;
    const double north = rtk.y - rtk_rover_anchor_.y;
    return Point3{
      mavros_local_anchor_.x +
        std::cos(rtk_to_local_yaw_) * east - std::sin(rtk_to_local_yaw_) * north,
      mavros_local_anchor_.y +
        std::sin(rtk_to_local_yaw_) * east + std::cos(rtk_to_local_yaw_) * north,
      home_.value_or(Point3{}).z + takeoff_alt_};
  }

  /** @brief 将 MAVROS 本地水平位置反变换到 RTK ENU。 */
  Point3 local_to_rtk_enu(const Point3 & local) const {
    const double dx = local.x - mavros_local_anchor_.x;
    const double dy = local.y - mavros_local_anchor_.y;
    return Point3{
      rtk_rover_anchor_.x +
        std::cos(rtk_to_local_yaw_) * dx + std::sin(rtk_to_local_yaw_) * dy,
      rtk_rover_anchor_.y -
        std::sin(rtk_to_local_yaw_) * dx + std::cos(rtk_to_local_yaw_) * dy,
      0.0};
  }

  /**
   * @brief 把搜索区域中心附近的局部偏移转换为飞行使用的本地坐标。
   *
   * RTK 可用时通过 center_rtk_enu 和区域 yaw 变换；否则通过
   * center_world、vehicle_spawn_world 和 home 构造出生点相对坐标。
   */
  Point3 area_offset_to_local(double offset_x, double offset_y) const {
    // RTK 路径：区域坐标 -> RTK ENU -> MAVROS 本地坐标。
    if (rtk_locked_ && coordinate_source_ != "spawn_relative") {
      const double c = std::cos(search_area_.world_to_rtk_yaw);
      const double s = std::sin(search_area_.world_to_rtk_yaw);
      const Point3 rtk{
        search_area_.center_rtk_enu.x + c * offset_x - s * offset_y,
        search_area_.center_rtk_enu.y + s * offset_x + c * offset_y,
        0.0};
      return rtk_to_local(rtk);
    }
    // 回退路径：世界坐标差值叠加到当前 MAVROS home。
    const auto home = home_.value_or(Point3{});
    return Point3{
      home.x + search_area_.center_world.x + offset_x - vehicle_spawn_world_.x,
      home.y + search_area_.center_world.y + offset_y - vehicle_spawn_world_.y,
      home.z + takeoff_alt_};
  }

  // ============================================================================
  // 覆盖路径规划与轨迹执行
  // ============================================================================

  /**
   * @brief 根据公开矩形区域生成往复式“割草”覆盖航线。
   *
   * 航线先按边界内缩，依据 lane_spacing 生成平行航带；随后从四种
   * 起始组合中选择离当前无人机最近的入口，并交替连接每条航带两端。
   */
  void start_coverage_scan() {
    scan_route_.clear();
    // 计算内缩后的可飞行矩形范围。
    const double half_x = search_area_.size_x * 0.5;
    const double half_y = search_area_.size_y * 0.5;
    const double inset_x = std::min(edge_inset_x_, half_x);
    const double inset_y = std::min(edge_inset_y_, half_y);
    const double x_min = -half_x + inset_x;
    const double x_max = half_x - inset_x;
    const double y_min = -half_y + inset_y;
    const double y_max = half_y - inset_y;
    const double x_span = std::max(0.0, x_max - x_min);
    // 航带数量确保最大相邻间距不超过 lane_spacing_。
    const std::size_t lane_count =
      x_span < 1e-6 ? 1U :
      static_cast<std::size_t>(std::ceil(x_span / lane_spacing_ - 1e-9)) + 1U;

    // 在 x_min 到 x_max 之间均匀放置每条扫描航带。
    std::vector<double> lane_offsets(lane_count, 0.0);
    for (std::size_t i = 0; i < lane_count; ++i) {
      lane_offsets[i] =
        lane_count == 1 ? 0.0 :
        x_min + x_span * static_cast<double>(i) / static_cast<double>(lane_count - 1);
    }

    // 比较左右两端、上下两个入口，共四种首航点组合。
    bool reverse_lanes = false;
    bool first_from_high_y = false;
    double best_entry_distance = std::numeric_limits<double>::infinity();
    const Point3 entry_reference = current_at_altitude();
    for (const bool reverse : {false, true}) {
      for (const bool high_y : {false, true}) {
        const double first_x = reverse ? lane_offsets.back() : lane_offsets.front();
        const Point3 candidate =
          area_offset_to_local(first_x, high_y ? y_max : y_min);
        const double distance = distance_xy(entry_reference, candidate);
        if (distance < best_entry_distance) {
          best_entry_distance = distance;
          reverse_lanes = reverse;
          first_from_high_y = high_y;
        }
      }
    }

    // 按奇偶航带交替飞向 y_min/y_max，形成连续蛇形覆盖。
    for (std::size_t order = 0; order < lane_count; ++order) {
      const std::size_t lane = reverse_lanes ? lane_count - 1U - order : order;
      const bool start_high = first_from_high_y ^ (order % 2U == 1U);
      const double first_y = start_high ? y_max : y_min;
      const double second_y = start_high ? y_min : y_max;
      scan_route_.push_back(
        CoverageWaypoint{area_offset_to_local(lane_offsets[lane], first_y), lane});
      if (std::abs(second_y - first_y) > 1e-6) {
        scan_route_.push_back(
          CoverageWaypoint{area_offset_to_local(lane_offsets[lane], second_y), lane});
      }
    }

    // 初始化扫描和视觉确认状态。
    scan_index_ = 0;
    detection_hold_active_ = false;
    detection_evidence_active_ = false;
    coverage_detection_enabled_ = false;
    reset_candidate();
    mission_start_time_ = now();
    mission_started_ = true;
    if (scan_route_.empty()) {
      start_return_home();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Coverage route planned from public area only: lanes=%zu waypoints=%zu "
      "spacing=%.2f m footprint=%.2fx%.2f m; random bucket positions remain unknown",
      lane_count, scan_route_.size(), lane_spacing_,
      camera_footprint_x_, camera_footprint_y_);
    for (std::size_t i = 0; i < scan_route_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(), "Coverage waypoint %zu/%zu lane=%zu local ENU=(%.2f,%.2f,%.2f)",
        i + 1, scan_route_.size(), scan_route_[i].lane_index + 1,
        scan_route_[i].point.x, scan_route_[i].point.y, scan_route_[i].point.z);
    }
    // 从当前位置平滑飞向首个覆盖航点。
    start_segment(current_at_altitude(), scan_route_.front().point);
    enter_state(MissionState::SCANNING);
  }

  /** @brief 更新覆盖航线或视觉确认插入段，并处理航点推进。 */
  void update_coverage_trajectory() {
    if (scan_index_ >= scan_route_.size()) {
      start_return_home();
      return;
    }

    // 视觉确认优先级高于原覆盖轨迹：先移动到投影目标上方并悬停。
    if (detection_hold_active_) {
      target_ = detection_hold_position_;
      if (!detection_evidence_active_) {
        // 到达悬停点后才开始计时和累计分类证据。
        if (distance_xyz(current_position_, detection_hold_position_) <= accept_radius_) {
          detection_evidence_active_ = true;
          detection_hold_start_time_ = now();
          reset_candidate();
          RCLCPP_INFO(
            get_logger(),
            "Visual target centered; accumulating full class evidence for %.1f s",
            detection_pause_s_);
        // 无法在规定时间内居中时放弃本次候选，继续原覆盖航点。
        } else if (seconds_since(detection_centering_start_time_) >=
          detection_centering_timeout_s_)
        {
          RCLCPP_INFO(
            get_logger(), "Visual centering timed out; resuming coverage route");
          detection_hold_active_ = false;
          reset_candidate();
          start_segment(current_at_altitude(), scan_route_[scan_index_].point);
        }
        return;
      }
      // 停留窗口结束后统一做候选判决，不论成功与否都恢复覆盖航线。
      if (seconds_since(detection_hold_start_time_) >= detection_pause_s_) {
        const bool confirmed = finalize_detection_candidate();
        if (!confirmed) {
          RCLCPP_INFO(
            get_logger(),
            "Detection pause expired without sufficient fused evidence; resuming coverage");
        }
        detection_hold_active_ = false;
        detection_evidence_active_ = false;
        reset_candidate();
        start_segment(current_at_altitude(), scan_route_[scan_index_].point);
      }
      return;
    }

    // 正常覆盖模式：按当前时间采样平滑轨迹设定点。
    target_ = sample_segment();
    update_yaw_to_target(target_);
    // 时间尚未走完时继续沿轨迹飞行。
    if (!segment_finished()) {
      return;
    }
    target_ = active_segment_.end;
    // 即使轨迹时间结束，也必须实际进入接受半径才推进到下一航点。
    if (distance_xyz(current_position_, active_segment_.end) > accept_radius_) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Coverage waypoint reached %zu/%zu lane=%zu",
      scan_index_ + 1, scan_route_.size(), scan_route_[scan_index_].lane_index + 1);
    // 只有真正到达首个区域内航点后，才允许记录视觉目标。
    if (!coverage_detection_enabled_) {
      coverage_detection_enabled_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Recognition gate enabled at the first public-area coverage waypoint");
    }
    ++scan_index_;
    if (scan_index_ >= scan_route_.size()) {
      RCLCPP_INFO(
        get_logger(),
        "Entire specified recognition area covered; observations=%zu",
        observations_.size());
      start_return_home();
      return;
    }
    start_segment(current_at_altitude(), scan_route_[scan_index_].point);
  }

  /** @brief 取消视觉确认，并规划到 home 上方巡航高度的返航段。 */
  void start_return_home() {
    const auto home = home_.value_or(Point3{});
    start_segment(
      current_at_altitude(), Point3{home.x, home.y, home.z + takeoff_alt_});
    detection_hold_active_ = false;
    enter_state(MissionState::RETURN_HOME);
  }

  /** @brief 执行返航轨迹；到达 home 上方后进入降落状态。 */
  void update_return_trajectory() {
    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (segment_finished() &&
      distance_xyz(current_position_, active_segment_.end) <= accept_radius_)
    {
      enter_state(MissionState::LANDING);
    }
  }

  /** @brief 根据距离和期望速度创建一段带最短时长约束的轨迹。 */
  void start_segment(const Point3 & start, const Point3 & end) {
    const double duration =
      std::max(min_segment_time_s_, distance_xyz(start, end) / trajectory_speed_);
    active_segment_ = Segment{start, end, duration, now()};
    target_ = start;
    update_yaw_to_target(end);
  }

  /**
   * @brief 以七次 smootherstep 对轨迹段进行时间插值。
   *
   * blend = 35t^4 - 84t^5 + 70t^6 - 20t^7，
   * 在 t=0 和 t=1 处速度、加速度和加加速度均为零，减少设定点突变。
   */
  Point3 sample_segment() const {
    // 将轨迹时间归一化为 tau ∈ [0, 1]。
    const double elapsed =
      std::clamp(seconds_since(active_segment_.start_time), 0.0, active_segment_.duration_s);
    const double tau = elapsed / active_segment_.duration_s;
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;
    // 使用同一标量混合系数分别插值 x、y、z。
    const double blend = 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;
    return Point3{
      active_segment_.start.x + (active_segment_.end.x - active_segment_.start.x) * blend,
      active_segment_.start.y + (active_segment_.end.y - active_segment_.start.y) * blend,
      active_segment_.start.z + (active_segment_.end.z - active_segment_.start.z) * blend};
  }

  /** @brief 判断当前轨迹段的计划持续时间是否结束。 */
  bool segment_finished() const {
    return seconds_since(active_segment_.start_time) >= active_segment_.duration_s;
  }

  /** @brief 返回当前水平位置与统一巡航高度组成的点。 */
  Point3 current_at_altitude() const {
    return Point3{
      current_position_.x, current_position_.y,
      home_.value_or(Point3{}).z + takeoff_alt_};
  }

  /** @brief 返回相对 home 的 MAVROS 本地高度。 */
  double relative_altitude() const {
    return current_position_.z - home_.value_or(Point3{}).z;
  }

  // ============================================================================
  // 设定点发布与 MAVROS 服务调用
  // ============================================================================

  /** @brief 发布位置和当前偏航角组成的 PoseStamped 设定点。 */
  void publish_setpoint(const Point3 & p) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = p.x;
    msg.pose.position.y = p.y;
    msg.pose.position.z = p.z;
    // 仅设置绕 z 轴的四元数分量，对应 roll=pitch=0。
    msg.pose.orientation.z = std::sin(current_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(current_yaw_ * 0.5);
    setpoint_pub_->publish(msg);
  }

  /** @brief 当水平距离足够大时，使机头朝向目标点。 */
  void update_yaw_to_target(const Point3 & p) {
    const double dx = p.x - current_position_.x;
    const double dy = p.y - current_position_.y;
    if (std::hypot(dx, dy) > 0.2) {
      current_yaw_ = std::atan2(dy, dx);
    }
  }

  /** @brief 非阻塞请求飞控切换模式；服务未就绪时等待下周期重试。 */
  void call_set_mode(const std::string & mode) {
    if (!set_mode_cli_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    pending_set_mode_future_ = set_mode_cli_->async_send_request(request).future.share();
  }

  /** @brief 非阻塞请求解锁或上锁。 */
  void call_arm(bool arm) {
    if (!arming_cli_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    pending_arm_future_ = arming_cli_->async_send_request(request).future.share();
  }

  /** @brief 非阻塞发送起飞命令；服务未就绪时允许重新发送。 */
  void call_takeoff(double altitude) {
    if (!takeoff_cli_->service_is_ready()) {
      takeoff_sent_ = false;
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(altitude);
    pending_takeoff_future_ = takeoff_cli_->async_send_request(request).future.share();
  }

  /** @brief 发送降落命令。当前实现不保存其异步返回值。 */
  void call_land() {
    if (!land_cli_->service_is_ready()) {
      land_sent_ = false;
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_cli_->async_send_request(request);
  }

  /**
   * @brief 轮询模式切换、解锁和起飞服务的异步结果。
   *
   * 请求被拒绝时清空 future，使状态机在后续周期重新发起请求。
   */
  void check_service_results() {
    if (pending_set_mode_future_.valid() &&
      pending_set_mode_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_set_mode_future_.get();
      if (!result->mode_sent) {
        RCLCPP_WARN(get_logger(), "Set mode rejected; retrying");
      }
      pending_set_mode_future_ = {};
    }
    if (pending_arm_future_.valid() &&
      pending_arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_arm_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Arming rejected; retrying");
      }
      pending_arm_future_ = {};
    }
    if (pending_takeoff_future_.valid() &&
      pending_takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_takeoff_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected; retrying");
        takeoff_sent_ = false;
      }
      pending_takeoff_future_ = {};
    }
  }

  // ============================================================================
  // 结果输出与通用辅助函数
  // ============================================================================

  /** @brief 将任务状态和目标观测写入 JSON 文件。 */
  void write_observations() {
    std::ofstream output(result_path_);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s", result_path_.c_str());
      return;
    }
    output << "{\n";
    output << "  \"seed\": " << scene_seed_ << ",\n";
    output << "  \"mission\": \"coverage_search_unknown_bucket_positions\",\n";
    // 明确记录状态机未读取目标 ground truth，便于后续审计。
    output << "  \"state_machine_has_ground_truth\": false,\n";
    output << "  \"localization\": \""
           << (rtk_locked_ ? "rtk" : "spawn_fallback") << "\",\n";
    output << "  \"coverage_waypoints\": " << scan_route_.size() << ",\n";
    output << "  \"coverage_completed\": "
           << (scan_index_ >= scan_route_.size() ? "true" : "false") << ",\n";
    output << "  \"observations\": [\n";
    for (std::size_t i = 0; i < observations_.size(); ++i) {
      const auto & observation = observations_[i];
      output << "    {\"class\": \"" << observation.predicted
             << "\", \"confidence\": " << std::fixed << std::setprecision(4)
             << observation.confidence << ", \"position_local\": ["
             << observation.local_position.x << ", " << observation.local_position.y
             << "], \"route_index\": " << observation.route_index
             << ", \"merged_views\": " << observation.merged_views;
      if (observation.has_rtk) {
        output << ", \"position_rtk_enu\": ["
               << observation.rtk_enu.x << ", " << observation.rtk_enu.y << "]";
      }
      output << "}";
      if (i + 1 < observations_.size()) {
        output << ',';
      }
      output << '\n';
    }
    output << "  ],\n";
    output << "  \"detected\": " << observations_.size() << "\n";
    output << "}\n";
  }

  /** @brief 切换任务状态，重置状态进入时间并打印日志。 */
  void enter_state(MissionState state) {
    if (mission_state_ == state) {
      return;
    }
    mission_state_ = state;
    state_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "Enter %s", state_name(state).c_str());
  }

  /** @brief 计算某个 ROS 时间点到当前时刻的秒数。 */
  double seconds_since(const rclcpp::Time & start) const {
    return (now() - start).seconds();
  }

  /** @brief 计算两点的水平欧氏距离。 */
  static double distance_xy(const Point3 & a, const Point3 & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  /** @brief 计算两点的三维欧氏距离。 */
  static double distance_xyz(const Point3 & a, const Point3 & b) {
    return std::hypot(distance_xy(a, b), a.z - b.z);
  }

  /** @brief 将枚举状态转换为日志使用的字符串。 */
  static std::string state_name(MissionState state) {
    switch (state) {
      case MissionState::WAITING_FCU: return "WAITING_FCU";
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

  // ============================================================================
  // 参数缓存
  // ============================================================================
  // 飞行与轨迹参数。
  double takeoff_alt_ = 1.5;
  double accept_radius_ = 0.3;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 300.0;
  double trajectory_speed_ = 0.65;
  double min_segment_time_s_ = 1.5;
  // 覆盖航线参数。
  double lane_spacing_ = 0.70;
  double edge_inset_x_ = 0.40;
  double edge_inset_y_ = 0.65;
  // 视觉确认与观测合并参数。
  double detection_pause_s_ = 3.0;
  double detection_centering_timeout_s_ = 5.0;
  double visual_min_confidence_ = 0.25;
  double visual_weak_confidence_ = 0.05;
  double visual_center_gate_fraction_ = 0.90;
  double observation_merge_radius_ = 0.65;
  double observation_same_class_merge_radius_ = 0.85;
  // 相机投影近似与定位参数。
  double camera_footprint_x_ = 1.02;
  double camera_footprint_y_ = 1.80;
  double camera_image_yaw_ = 0.0;
  double rtk_fix_timeout_s_ = 8.0;
  double rtk_to_local_yaw_ = 0.0;
  // 帧数、图像尺寸和场景标识。
  int visual_min_consecutive_ = 3;
  int visual_weak_min_consecutive_ = 8;
  int camera_width_ = 848;
  int camera_height_ = 480;
  int scene_seed_ = 0;

  // ============================================================================
  // 运行状态标志
  // ============================================================================
  bool have_local_position_ = false;
  bool have_valid_fix_ = false;
  bool have_rtk_reference_ = false;
  bool rtk_locked_ = false;
  bool using_spawn_fallback_ = false;
  bool publish_setpoints_ = true;
  bool takeoff_sent_ = false;
  bool land_sent_ = false;
  bool result_written_ = false;
  bool mission_started_ = false;
  bool detection_hold_active_ = false;
  bool detection_evidence_active_ = false;
  bool coverage_detection_enabled_ = false;

  // ============================================================================
  // 配置字符串与视觉候选累计量
  // ============================================================================
  std::string generated_scene_path_;
  std::string coordinate_source_;
  std::string result_path_;
  std::map<std::string, double> candidate_class_scores_;
  std::map<std::string, double> candidate_class_peaks_;
  int candidate_frame_count_ = 0;
  int candidate_position_samples_ = 0;
  double current_yaw_ = 0.0;
  double rtk_base_lat_ = 0.0;
  double rtk_base_lon_ = 0.0;
  double rtk_base_alt_ = 0.0;

  // ============================================================================
  // 最新消息、坐标锚点、任务路径和状态机数据
  // ============================================================================
  mavros_msgs::msg::State current_state_;
  sensor_msgs::msg::NavSatFix latest_fix_;
  Point3 current_position_;
  Point3 target_;
  Point3 vehicle_spawn_world_;
  Point3 rtk_rover_anchor_;
  Point3 mavros_local_anchor_;
  Point3 detection_hold_position_;
  Point3 candidate_position_;
  SearchArea search_area_;
  std::optional<Point3> home_;
  std::optional<rclcpp::Time> localization_wait_start_;
  std::vector<CoverageWaypoint> scan_route_;
  std::vector<Observation> observations_;
  std::size_t scan_index_ = 0;
  Segment active_segment_;
  MissionState mission_state_ = MissionState::WAITING_FCU;

  // 各任务阶段与视觉确认过程的计时起点。
  rclcpp::Time mission_start_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time last_state_log_;
  rclcpp::Time detection_hold_start_time_;
  rclcpp::Time detection_centering_start_time_;

  // ============================================================================
  // ROS 2 通信对象
  // ============================================================================
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<HazardDetection>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture pending_set_mode_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture pending_arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture pending_takeoff_future_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/** @brief 初始化 ROS 2，运行状态机节点，节点结束后清理资源。 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReconStateMachineCpp>());
  rclcpp::shutdown();
  return 0;
}
