/*
 * CUADC 室外定点起飞、双载荷顺序投放、降落状态机
 *
 * 用途：
 *   只验证实际飞机的起飞、通道 7、通道 8 顺序投放和降落，不执行航点、搜索或视觉任务。
 *
 * 操作约束：
 *   1. 飞机静止、未解锁时，使用 compass_hdg 锁定真实机头航向。
 *   2. 飞手用遥控器切入 GUIDED，程序只显示“GUIDED,开始任务”。
 *   3. 程序不会自动切换模式，也不会自动解锁；飞手确认环境安全后手动解锁。
 *   4. 检测到 GUIDED + armed 后，程序执行：
 *        起飞 -> 定点悬停 -> 通道 7 投放并收回 -> 通道 8 投放并收回 -> 降落。
 *   5. 任务中一旦人工切离 GUIDED，程序立即停止发布目标并退出，
 *      不与飞手争夺控制权。
 *   6. /mavros/local_position/odom 是唯一位置源。
 *   7. 起飞 CommandTOL 显式携带 compass_hdg 锁定航向。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
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

struct HeadingSample
{
  rclcpp::Time stamp;
  double yaw_enu = 0.0;
};

enum class State
{
  WAIT_FCU,
  WAIT_NAV_STABLE,
  PRESTREAM,
  WAIT_GUIDED,
  WAIT_ARM,
  TAKEOFF,
  HOLD,
  RELEASE,
  INTER_PAYLOAD_HOLD,
  POST_RELEASE_HOLD,
  LAND,
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
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future;
};

}  // namespace

class TakeoffDropLandNode final : public rclcpp::Node
{
public:
  TakeoffDropLandNode()
  : Node("takeoff_drop_land_node")
  {
    declare_parameters();
    load_parameters();

    const auto reliable_qos = rclcpp::QoS(10).reliable();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", reliable_qos,
      std::bind(&TakeoffDropLandNode::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&TakeoffDropLandNode::odom_callback, this, std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&TakeoffDropLandNode::compass_callback, this, std::placeholders::_1));

    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    command_client_ = create_client<mavros_msgs::srv::CommandLong>(
      "/mavros/cmd/command");

    state_enter_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_servo_attempt_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_status_time_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&TakeoffDropLandNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Ready: manual GUIDED + manual arm starts fixed-yaw takeoff/drop/land mission");
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("takeoff_alt_m", 3.0);
    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("hold_before_release_s", 2.0);
    declare_parameter<double>("inter_payload_hold_s", 0.5);
    declare_parameter<double>("post_release_hold_s", 1.0);
    declare_parameter<double>("takeoff_timeout_s", 45.0);
    declare_parameter<double>("mission_timeout_s", 90.0);
    declare_parameter<double>("odom_timeout_s", 0.75);
    declare_parameter<double>("compass_timeout_s", 1.0);
    declare_parameter<double>("heading_lock_stability_s", 2.0);
    declare_parameter<double>("heading_lock_max_variation_deg", 2.0);
    declare_parameter<double>("position_lock_stability_s", 2.0);
    declare_parameter<double>("stationary_speed_max_m_s", 0.15);
    declare_parameter<std::vector<int64_t>>(
      "payload_sequence", std::vector<int64_t>{0, 1});
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
  }

  void load_parameters()
  {
    takeoff_alt_m_ = std::max(0.5, get_parameter("takeoff_alt_m").as_double());
    prestream_s_ = std::max(1.0, get_parameter("prestream_hold_s").as_double());
    hold_before_release_s_ =
      std::max(0.0, get_parameter("hold_before_release_s").as_double());
    inter_payload_hold_s_ =
      std::max(0.0, get_parameter("inter_payload_hold_s").as_double());
    post_release_hold_s_ =
      std::max(0.0, get_parameter("post_release_hold_s").as_double());
    takeoff_timeout_s_ =
      std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ =
      std::max(30.0, get_parameter("mission_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    compass_timeout_s_ =
      std::max(0.2, get_parameter("compass_timeout_s").as_double());
    heading_stability_s_ =
      std::max(0.5, get_parameter("heading_lock_stability_s").as_double());
    heading_max_variation_rad_ =
      std::max(0.1, get_parameter("heading_lock_max_variation_deg").as_double()) *
      kPi / 180.0;
    position_stability_s_ =
      std::max(0.5, get_parameter("position_lock_stability_s").as_double());
    stationary_speed_m_s_ =
      std::max(0.02, get_parameter("stationary_speed_max_m_s").as_double());
    payload_sequence_raw_ = get_parameter("payload_sequence").as_integer_array();
    servo_channels_ = get_parameter("servo_channels").as_integer_array();
    stowed_pwm_ = get_parameter("servo_stowed_pwm").as_integer_array();
    release_pwm_ = get_parameter("servo_release_pwm").as_integer_array();
    release_duration_s_ =
      get_parameter("servo_release_duration_s").as_double_array();
    initialize_stowed_ = get_parameter("servo_initialize_stowed").as_bool();
    return_to_stowed_ = get_parameter("servo_return_to_stowed").as_bool();

    config_valid_ = !payload_sequence_raw_.empty();
    payload_sequence_.clear();
    for (const int64_t payload : payload_sequence_raw_) {
      if (payload < 0 ||
        static_cast<std::size_t>(payload) >= servo_channels_.size() ||
        static_cast<std::size_t>(payload) >= stowed_pwm_.size() ||
        static_cast<std::size_t>(payload) >= release_pwm_.size() ||
        static_cast<std::size_t>(payload) >= release_duration_s_.size())
      {
        config_valid_ = false;
        continue;
      }
      payload_sequence_.push_back(static_cast<std::size_t>(payload));
    }
    config_valid_ = config_valid_ &&
      payload_sequence_.size() == payload_sequence_raw_.size();
    if (!config_valid_) {
      RCLCPP_ERROR(get_logger(), "Invalid payload/servo array configuration");
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
      std::cout << "GUIDED,开始任务；请确认环境安全后手动解锁" << std::endl;
    }

    if (!guided_active_ && was_guided && is_automatic_flight_state(state_)) {
      publish_setpoint_ = false;
      if (state_ == State::RELEASE && release_started_ && !return_sent_ &&
        !return_command_pending_)
      {
        return_command_pending_ =
          send_servo(current_payload(), false, ServoPurpose::RETURN_STOWED);
      }
      std::cout << "已离开GUIDED,自动控制退出,飞手接管" << std::endl;
      enter(State::PILOT_OVERRIDE);
      return;
    }

    if (fcu_state_.armed && !was_armed && state_ == State::WAIT_ARM) {
      std::cout << "检测到飞控已解锁,开始自动起飞" << std::endl;
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
    have_odom_ = true;
    last_odom_time_ = now();

    if (!frame_locked_) {
      if (horizontal_speed_m_s_ <= stationary_speed_m_s_) {
        if (!position_stable_since_.has_value()) {
          position_stable_since_ = now();
        }
      } else {
        position_stable_since_.reset();
      }
    }
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    const double yaw_enu =
      normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    last_compass_time_ = now();
    have_compass_ = true;
    heading_samples_.push_back(HeadingSample{last_compass_time_, yaw_enu});
    while (!heading_samples_.empty() &&
      (last_compass_time_ - heading_samples_.front().stamp).seconds() >
      heading_stability_s_ + 0.5)
    {
      heading_samples_.pop_front();
    }
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
    for (const auto & sample : heading_samples_) {
      if (std::abs(normalize_angle(sample.yaw_enu - mean)) >
        heading_max_variation_rad_)
      {
        return false;
      }
    }
    return true;
  }

  bool navigation_ready_to_lock() const
  {
    return config_valid_ && fcu_state_.connected && !fcu_state_.armed &&
      odom_fresh() && compass_fresh() && heading_stable() &&
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
    target_ = *home_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    frame_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Heading/home locked: compass=%.2f deg yaw_enu=%.2f deg home=(%.3f,%.3f,%.3f)",
      locked_compass_deg_, mission_yaw_ * 180.0 / kPi,
      home_->x, home_->y, home_->z);
  }

  void tick()
  {
    check_service_results();
    check_servo_results();
    initialize_servos_if_ready();

    if (publish_setpoint_ && frame_locked_) {
      publish_setpoint();
    }

    if (mission_started_ && state_ != State::LAND && state_ != State::DONE &&
      state_ != State::PILOT_OVERRIDE && state_ != State::ABORT &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      abort_or_land("Mission timeout");
    }

    if ((now() - last_status_time_).seconds() >= 3.0) {
      last_status_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s mode=%s armed=%s rel_alt=%.2f compass=%.1f locked_yaw=%.1f",
        state_name(state_).c_str(), fcu_state_.mode.c_str(),
        fcu_state_.armed ? "true" : "false", relative_altitude(),
        current_compass_deg_, mission_yaw_ * 180.0 / kPi);
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_ = false;
        if (!config_valid_) {
          enter(State::ABORT);
        } else if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (navigation_ready_to_lock()) {
          lock_frame();
          publish_setpoint_ = true;
          enter(State::PRESTREAM);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting lock: odom=%s compass=%s heading=%s stationary=%s disarmed=%s",
            odom_fresh() ? "yes" : "no", compass_fresh() ? "yes" : "no",
            heading_stable() ? "yes" : "no",
            horizontal_speed_m_s_ <= stationary_speed_m_s_ ? "yes" : "no",
            fcu_state_.armed ? "no" : "yes");
        }
        break;

      case State::PRESTREAM:
        if (!odom_fresh() || !compass_fresh()) {
          abort_or_land("Navigation data stale during PRESTREAM");
        } else if ((now() - state_enter_time_).seconds() >= prestream_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        if (!fcu_state_.connected) {
          enter(State::ABORT);
        } else if (guided_active_) {
          if (!guided_announced_) {
            guided_announced_ = true;
            std::cout << "GUIDED,开始任务；请确认环境安全后手动解锁" << std::endl;
          }
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        if (!guided_active_) {
          guided_announced_ = false;
          enter(State::WAIT_GUIDED);
        } else if (fcu_state_.armed) {
          if (initialize_stowed_ && !servos_initialized_) {
            abort_or_land("Servo initialization has not been ACKed");
          } else {
            mission_started_ = true;
            mission_start_time_ = now();
            target_ = *home_;
            target_.z = home_->z + takeoff_alt_m_;
            publish_setpoint_ = false;
            enter(State::TAKEOFF);
          }
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "GUIDED active; waiting for pilot to arm");
        }
        break;

      case State::TAKEOFF:
        publish_setpoint_ = false;
        if (!fcu_state_.armed) {
          enter(State::ABORT);
          break;
        }
        if (!guided_active_) {
          enter(State::PILOT_OVERRIDE);
          break;
        }
        if (!takeoff_sent_) {
          request_takeoff();
        }
        if (relative_altitude() >= takeoff_alt_m_ * 0.90) {
          publish_setpoint_ = true;
          enter(State::HOLD);
        } else if (
          takeoff_sent_ &&
          (now() - state_enter_time_).seconds() > takeoff_timeout_s_)
        {
          abort_or_land("Takeoff timeout");
        }
        break;

      case State::HOLD:
        if (!flight_gate_ok()) {
          break;
        }
        if ((now() - state_enter_time_).seconds() >= hold_before_release_s_) {
          enter(State::RELEASE);
        }
        break;

      case State::RELEASE:
        if (!flight_gate_ok()) {
          break;
        }
        update_release();
        break;

      case State::INTER_PAYLOAD_HOLD:
        if (!flight_gate_ok()) {
          break;
        }
        if ((now() - state_enter_time_).seconds() >= inter_payload_hold_s_) {
          enter(State::RELEASE);
        }
        break;

      case State::POST_RELEASE_HOLD:
        if (!flight_gate_ok()) {
          break;
        }
        if ((now() - state_enter_time_).seconds() >= post_release_hold_s_) {
          publish_setpoint_ = false;
          enter(State::LAND);
        }
        break;

      case State::LAND:
        publish_setpoint_ = false;
        request_land();
        if (!fcu_state_.armed && relative_altitude() < 0.30) {
          enter(State::DONE);
        }
        break;

      case State::DONE:
        if (!success_logged_) {
          success_logged_ = true;
          std::cout << "任务完成：起飞、双投、降落、解锁" << std::endl;
          RCLCPP_INFO(
            get_logger(),
            "TAKEOFF_DROP_LAND_SUCCESS: released payloads 1 and 2, landed and disarmed");
        }
        if ((now() - state_enter_time_).seconds() >= 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_ = false;
        if ((now() - state_enter_time_).seconds() >= 0.5) {
          RCLCPP_WARN(get_logger(), "PILOT_OVERRIDE: node exits without sending LAND");
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          RCLCPP_ERROR(get_logger(), "TAKEOFF_DROP_LAND_ABORTED");
          rclcpp::shutdown();
        }
        break;
    }
  }

  bool flight_gate_ok()
  {
    if (!fcu_state_.armed) {
      enter(State::ABORT);
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    if (!odom_fresh() || !compass_fresh()) {
      abort_or_land("Navigation data stale during automatic flight");
      return false;
    }
    return true;
  }

  void update_release()
  {
    if (!release_started_ && !release_command_pending_) {
      release_command_pending_ =
        send_servo(current_payload(), true, ServoPurpose::RELEASE);
      if (!release_command_pending_) {
        abort_or_land("Unable to send payload release command");
      }
      return;
    }

    if (release_started_ && return_to_stowed_ && !return_sent_ &&
      !return_command_pending_ &&
      (now() - release_start_time_).seconds() >= release_duration())
    {
      return_command_pending_ =
        send_servo(current_payload(), false, ServoPurpose::RETURN_STOWED);
      return;
    }

    if (release_started_ && (!return_to_stowed_ || return_sent_)) {
      if (release_sequence_index_ + 1U < payload_sequence_.size()) {
        ++release_sequence_index_;
        release_command_pending_ = false;
        release_started_ = false;
        return_command_pending_ = false;
        return_sent_ = false;
        RCLCPP_INFO(
          get_logger(),
          "Payload %zu complete; preparing payload %zu on channel %ld",
          payload_sequence_[release_sequence_index_ - 1U] + 1U,
          current_payload() + 1U,
          static_cast<long>(servo_channels_[current_payload()]));
        enter(State::INTER_PAYLOAD_HOLD);
      } else {
        enter(State::POST_RELEASE_HOLD);
      }
    }

    if ((now() - state_enter_time_).seconds() > release_duration() + 8.0) {
      abort_or_land("Payload release/return ACK timeout");
    }
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
    for (std::size_t index = 0U; index < servo_channels_.size(); ++index) {
      if (index >= stowed_pwm_.size()) {
        initialization_failed_ = true;
        continue;
      }
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
      if (it->future.wait_for(0s) != std::future_status::ready) {
        ++it;
        continue;
      }
      const auto response = it->future.get();
      const bool accepted = response->success;
      RCLCPP_INFO(
        get_logger(), "DO_SET_SERVO ACK payload=%zu success=%s result=%u",
        it->payload + 1U, accepted ? "true" : "false", response->result);

      if (it->purpose == ServoPurpose::INITIALIZE) {
        if (initialization_pending_count_ > 0U) {
          --initialization_pending_count_;
        }
        initialization_failed_ = initialization_failed_ || !accepted;
      } else if (it->purpose == ServoPurpose::RELEASE) {
        release_command_pending_ = false;
        if (accepted) {
          release_started_ = true;
          release_start_time_ = now();
          std::cout << "飞控确认投放：通道" << servo_channels_[it->payload] <<
            "=" << release_pwm_[it->payload] << "us" << std::endl;
        } else {
          abort_or_land("FCU rejected payload release");
        }
      } else {
        return_command_pending_ = false;
        if (accepted) {
          return_sent_ = true;
          std::cout << "飞控确认舵机收回：通道" << servo_channels_[it->payload] <<
            "=" << stowed_pwm_[it->payload] << "us" << std::endl;
        } else {
          abort_or_land("FCU rejected servo return-to-stowed");
        }
      }
      it = pending_servo_commands_.erase(it);
    }

    if (initialization_started_ && initialization_pending_count_ == 0U) {
      initialization_started_ = false;
      if (initialization_failed_) {
        RCLCPP_ERROR(get_logger(), "Servo initialization ACK failed; retrying");
      } else {
        servos_initialized_ = true;
        std::cout << "舵机初始化确认：通道7=1100us,通道8=1100us" << std::endl;
      }
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
      get_logger(), "CommandTOL takeoff: altitude=%.1f explicit_yaw=%.2f deg",
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
  }

  bool request_allowed() const
  {
    return (now() - last_request_time_).seconds() >= 1.0;
  }

  void mark_request()
  {
    last_request_time_ = now();
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  double release_duration() const
  {
    const std::size_t payload = current_payload();
    if (payload >= release_duration_s_.size()) {
      return 0.7;
    }
    return std::max(0.1, release_duration_s_[payload]);
  }

  std::size_t current_payload() const
  {
    if (payload_sequence_.empty() ||
      release_sequence_index_ >= payload_sequence_.size())
    {
      return 0U;
    }
    return payload_sequence_[release_sequence_index_];
  }

  bool is_automatic_flight_state(State state) const
  {
    return state == State::WAIT_ARM || state == State::TAKEOFF ||
      state == State::HOLD || state == State::RELEASE ||
      state == State::INTER_PAYLOAD_HOLD ||
      state == State::POST_RELEASE_HOLD;
  }

  void abort_or_land(const std::string & reason)
  {
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
      case State::HOLD: return "HOLD";
      case State::RELEASE: return "RELEASE";
      case State::INTER_PAYLOAD_HOLD: return "INTER_PAYLOAD_HOLD";
      case State::POST_RELEASE_HOLD: return "POST_RELEASE_HOLD";
      case State::LAND: return "LAND";
      case State::DONE: return "DONE";
      case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
      case State::ABORT: return "ABORT";
    }
    return "UNKNOWN";
  }

  bool config_valid_ = false;
  bool guided_active_ = false;
  bool guided_announced_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_ = false;
  bool mission_started_ = false;
  bool takeoff_sent_ = false;
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

  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  Point3 position_;
  Point3 target_;
  std::optional<Point3> home_;
  std::optional<rclcpp::Time> position_stable_since_;
  std::deque<HeadingSample> heading_samples_;
  std::vector<PendingServoCommand> pending_servo_commands_;

  double takeoff_alt_m_ = 3.0;
  double prestream_s_ = 1.5;
  double hold_before_release_s_ = 2.0;
  double inter_payload_hold_s_ = 0.5;
  double post_release_hold_s_ = 1.0;
  double takeoff_timeout_s_ = 45.0;
  double mission_timeout_s_ = 90.0;
  double odom_timeout_s_ = 0.75;
  double compass_timeout_s_ = 1.0;
  double heading_stability_s_ = 2.0;
  double heading_max_variation_rad_ = 2.0 * kPi / 180.0;
  double position_stability_s_ = 2.0;
  double stationary_speed_m_s_ = 0.15;
  double horizontal_speed_m_s_ = 0.0;
  double current_compass_deg_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;

  std::size_t release_sequence_index_ = 0U;
  std::size_t initialization_pending_count_ = 0U;
  std::vector<int64_t> payload_sequence_raw_;
  std::vector<std::size_t> payload_sequence_;
  std::vector<int64_t> servo_channels_;
  std::vector<int64_t> stowed_pwm_;
  std::vector<int64_t> release_pwm_;
  std::vector<double> release_duration_s_;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_compass_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time release_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_servo_attempt_time_;
  rclcpp::Time last_status_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffDropLandNode>());
  rclcpp::shutdown();
  return 0;
}
