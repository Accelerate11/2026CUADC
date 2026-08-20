#ifndef CUADC_PAYLOAD__PAYLOAD_NODE_HPP_
#define CUADC_PAYLOAD__PAYLOAD_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cuadc_interfaces/action/release_payload.hpp"
#include "cuadc_interfaces/msg/payload_status.hpp"
#include "cuadc_interfaces/msg/safety_status.hpp"
#include "cuadc_interfaces/srv/payload_command.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace cuadc_payload
{

class PayloadNode final : public rclcpp::Node
{
public:
  explicit PayloadNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PayloadNode() override;

private:
  using PayloadCommand = cuadc_interfaces::srv::PayloadCommand;
  using PayloadStatus = cuadc_interfaces::msg::PayloadStatus;
  using SafetyStatus = cuadc_interfaces::msg::SafetyStatus;
  using ReleasePayload = cuadc_interfaces::action::ReleasePayload;
  using ReleaseGoalHandle = rclcpp_action::ServerGoalHandle<ReleasePayload>;
  using CommandLong = mavros_msgs::srv::CommandLong;

  struct ActionRecord
  {
    std::uint8_t payload_index{0U};
    bool started{false};
    bool terminal{false};
    std::shared_ptr<ReleasePayload::Result> result;
    std::vector<std::shared_ptr<ReleaseGoalHandle>> goal_handles;
  };

  enum class PendingPhase : std::uint8_t
  {
    NONE = 0,
    RELEASE = 1,
    STOW = 2,
  };

  static constexpr std::uint16_t kMavCmdDoSetServo = 183U;
  static constexpr std::uint8_t kMavResultAccepted = 0U;
  static constexpr std::uint8_t kPayloadOne = 1U;
  static constexpr std::uint8_t kPayloadTwo = 2U;
  static constexpr std::chrono::milliseconds kStowRetryDelay{100};
  static constexpr std::uint8_t kMaximumStowAttempts = 3U;

  void handle_payload_command(
    const std::shared_ptr<PayloadCommand::Request> request,
    std::shared_ptr<PayloadCommand::Response> response);
  bool valid_request_id(const std::string & request_id) const;
  std::size_t slot_for_payload(std::uint8_t payload_index) const;
  std::uint8_t channel_for_payload(std::uint8_t payload_index) const;
  std::uint16_t stow_pwm_for_payload(std::uint8_t payload_index) const;
  std::uint16_t release_pwm_for_payload(std::uint8_t payload_index) const;
  std::chrono::nanoseconds release_duration_for_payload(std::uint8_t payload_index) const;
  std::string action_key(std::uint64_t mission_epoch, const std::string & command_id) const;

  rclcpp_action::GoalResponse handle_release_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ReleasePayload::Goal> goal);
  rclcpp_action::CancelResponse handle_release_cancel(
    const std::shared_ptr<ReleaseGoalHandle> goal_handle);
  void handle_release_accepted(const std::shared_ptr<ReleaseGoalHandle> goal_handle);
  void start_action_release(
    const std::string & key,
    const std::shared_ptr<ReleaseGoalHandle> goal_handle);
  void publish_action_feedback(std::uint8_t stage);
  void complete_action(std::uint8_t outcome, bool success, const std::string & reason);
  void deliver_action_result(
    const std::shared_ptr<ReleaseGoalHandle> & goal_handle,
    const std::shared_ptr<ReleasePayload::Result> & result) const;

  void start_operation(std::uint8_t command);
  void send_servo_command(PendingPhase phase, std::uint16_t pwm);
  void handle_mavros_response(
    std::uint64_t generation,
    PendingPhase phase,
    rclcpp::Client<CommandLong>::SharedFuture future);
  void handle_ack_timeout(std::uint64_t generation, PendingPhase phase);
  void handle_command_result(
    PendingPhase phase,
    bool accepted,
    std::uint8_t mav_result,
    const std::string & reason);

  void schedule_release_hold();
  void begin_restow(const std::string & reason, bool operation_failed);
  void send_next_stow_attempt();
  void handle_stow_failure(const std::string & reason, std::uint8_t mav_result);
  void latch_fault(const std::string & reason, std::uint8_t mav_result);
  void cancel_timer(rclcpp::TimerBase::SharedPtr & timer);

  void publish_payload_status();
  void publish_safety_status();
  std::string safety_detail() const;

  rclcpp::Service<PayloadCommand>::SharedPtr command_service_;
  rclcpp_action::Server<ReleasePayload>::SharedPtr release_action_server_;
  rclcpp::Client<CommandLong>::SharedPtr mavros_command_client_;
  rclcpp::Publisher<PayloadStatus>::SharedPtr payload_status_publisher_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr safety_status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr ack_timeout_timer_;
  rclcpp::TimerBase::SharedPtr release_hold_timer_;
  rclcpp::TimerBase::SharedPtr stow_retry_timer_;

  bool flight_enable_{false};
  bool runtime_gate_open_{false};
  bool config_bundle_hash_valid_{false};
  bool allow_legacy_release_service_{false};
  bool busy_{false};
  bool interlock_active_{false};
  bool fault_latched_{false};
  bool operation_failed_{false};
  bool payload_one_consumed_{false};
  bool payload_two_consumed_{false};
  bool last_command_accepted_{false};
  bool active_operation_is_action_{false};
  bool action_cancel_requested_{false};
  bool action_committed_{false};
  bool action_release_ack_{false};
  bool action_stow_ack_{false};

  std::uint8_t payload_index_{0U};
  std::uint8_t servo_channel_{0U};
  std::uint8_t payload_state_{PayloadStatus::UNKNOWN};
  std::uint8_t stow_attempts_{0U};
  std::uint8_t last_mav_result_{255U};
  PendingPhase pending_phase_{PendingPhase::NONE};
  std::uint64_t operation_generation_{0U};
  std::string request_id_;
  std::string active_action_key_;
  std::string detail_{"Payload position is unknown; issue STOW after all gates pass."};
  std::unordered_map<std::string, ActionRecord> action_records_;
  std::unordered_set<std::string> seen_request_ids_;
  std::chrono::milliseconds ack_timeout_{2000};
  std::array<std::uint8_t, 2> servo_channels_{{0U, 0U}};
  std::array<std::uint16_t, 2> servo_stowed_pwm_{{0U, 0U}};
  std::array<std::uint16_t, 2> servo_release_pwm_{{0U, 0U}};
  std::array<std::chrono::nanoseconds, 2> servo_release_duration_{{
    std::chrono::nanoseconds::zero(), std::chrono::nanoseconds::zero()}};
};

}  // namespace cuadc_payload

#endif  // CUADC_PAYLOAD__PAYLOAD_NODE_HPP_
