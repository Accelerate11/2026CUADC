#include "cuadc_payload/payload_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cuadc_payload
{

using namespace std::chrono_literals;

PayloadNode::PayloadNode(const rclcpp::NodeOptions & options)
: Node("payload_node", options)
{
  flight_enable_ = declare_parameter<bool>("flight_enable", false);
  allow_legacy_release_service_ =
    declare_parameter<bool>("allow_legacy_release_service", false);
  const auto timeout_ms = declare_parameter<std::int64_t>("command_ack_timeout_ms", 2000);
  if (timeout_ms < 500 || timeout_ms > 10000) {
    throw std::invalid_argument("command_ack_timeout_ms must be in [500, 10000]");
  }
  ack_timeout_ = std::chrono::milliseconds(timeout_ms);

  const auto channels = declare_parameter<std::vector<std::int64_t>>(
    "servo_channels", std::vector<std::int64_t>());
  const auto stowed_pwm = declare_parameter<std::vector<std::int64_t>>(
    "servo_stowed_pwm", std::vector<std::int64_t>());
  const auto release_pwm = declare_parameter<std::vector<std::int64_t>>(
    "servo_release_pwm", std::vector<std::int64_t>());
  const auto release_duration = declare_parameter<std::vector<double>>(
    "servo_release_duration_s", std::vector<double>());
  if (channels.size() != 2U || stowed_pwm.size() != 2U || release_pwm.size() != 2U ||
    release_duration.size() != 2U)
  {
    throw std::invalid_argument(
            "servo_channels, servo_stowed_pwm, servo_release_pwm, and "
            "servo_release_duration_s must each contain exactly two values");
  }
  for (std::size_t index = 0U; index < 2U; ++index) {
    if (channels[index] < 1 || channels[index] > 16) {
      throw std::invalid_argument("each servo channel must be in [1, 16]");
    }
    if (stowed_pwm[index] < 800 || stowed_pwm[index] > 2200 ||
      release_pwm[index] < 800 || release_pwm[index] > 2200)
    {
      throw std::invalid_argument("each servo PWM must be in [800, 2200] microseconds");
    }
    if (stowed_pwm[index] == release_pwm[index]) {
      throw std::invalid_argument("stowed and release PWM must differ for each payload");
    }
    if (!std::isfinite(release_duration[index]) || release_duration[index] < 0.05 ||
      release_duration[index] > 5.0)
    {
      throw std::invalid_argument("each release duration must be finite and in [0.05, 5.0] seconds");
    }
    servo_channels_[index] = static_cast<std::uint8_t>(channels[index]);
    servo_stowed_pwm_[index] = static_cast<std::uint16_t>(stowed_pwm[index]);
    servo_release_pwm_[index] = static_cast<std::uint16_t>(release_pwm[index]);
    servo_release_duration_[index] = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(release_duration[index]));
  }
  if (servo_channels_[0] == servo_channels_[1]) {
    throw std::invalid_argument("the two payload servo channels must be different");
  }

  const char * runtime_gate = std::getenv("CUADC_PAYLOAD_RUNTIME_GATE");
  const char * config_bundle_hash = std::getenv("CUADC_CONFIG_BUNDLE_SHA256");
  const std::string config_bundle_hash_value =
    config_bundle_hash == nullptr ? std::string() : std::string(config_bundle_hash);
  config_bundle_hash_valid_ = config_bundle_hash_value.size() == 64U &&
    std::all_of(
    config_bundle_hash_value.begin(), config_bundle_hash_value.end(),
    [](const unsigned char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
  runtime_gate_open_ = runtime_gate != nullptr &&
    std::string(runtime_gate) == "CUADC_FLIGHT_LAUNCH_PREFLIGHT_PASSED" &&
    config_bundle_hash_valid_;

  mavros_command_client_ = create_client<CommandLong>("/mavros/cmd/command");

  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  payload_status_publisher_ = create_publisher<PayloadStatus>(
    "/cuadc/payload/status", status_qos);
  safety_status_publisher_ = create_publisher<SafetyStatus>(
    "/cuadc/payload/safety_status", status_qos);

  command_service_ = create_service<PayloadCommand>(
    "/cuadc/payload/command",
    std::bind(
      &PayloadNode::handle_payload_command, this,
      std::placeholders::_1, std::placeholders::_2));

  release_action_server_ = rclcpp_action::create_server<ReleasePayload>(
    this,
    "/cuadc/payload/release",
    std::bind(
      &PayloadNode::handle_release_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &PayloadNode::handle_release_cancel, this,
      std::placeholders::_1),
    std::bind(
      &PayloadNode::handle_release_accepted, this,
      std::placeholders::_1));

  status_timer_ = create_wall_timer(100ms, [this]() {
      publish_payload_status();
      publish_safety_status();
    });

  publish_payload_status();
  publish_safety_status();

  RCLCPP_INFO(
    get_logger(),
    "Payload profile: P1 CH%u stow=%u release=%u duration=%.3f s; "
    "P2 CH%u stow=%u release=%u duration=%.3f s",
    static_cast<unsigned int>(servo_channels_[0]),
    static_cast<unsigned int>(servo_stowed_pwm_[0]),
    static_cast<unsigned int>(servo_release_pwm_[0]),
    std::chrono::duration<double>(servo_release_duration_[0]).count(),
    static_cast<unsigned int>(servo_channels_[1]),
    static_cast<unsigned int>(servo_stowed_pwm_[1]),
    static_cast<unsigned int>(servo_release_pwm_[1]),
    std::chrono::duration<double>(servo_release_duration_[1]).count());
  if (!flight_enable_) {
    RCLCPP_WARN(get_logger(), "flight_enable is false; all payload commands are blocked");
  }
  if (!runtime_gate_open_) {
    RCLCPP_WARN(
      get_logger(),
      "Runtime gate is closed; validated launcher must inject the exact payload gate and lowercase config-bundle SHA-256");
  }
}

PayloadNode::~PayloadNode()
{
  if (!busy_ || (payload_index_ != kPayloadOne && payload_index_ != kPayloadTwo)) {
    return;
  }
  if (!mavros_command_client_ || !mavros_command_client_->service_is_ready()) {
    return;
  }

  auto request = std::make_shared<CommandLong::Request>();
  request->broadcast = false;
  request->command = kMavCmdDoSetServo;
  request->confirmation = 0U;
  request->param1 = static_cast<float>(servo_channel_);
  request->param2 = static_cast<float>(stow_pwm_for_payload(payload_index_));
  try {
    (void)mavros_command_client_->async_send_request(request);
  } catch (const std::exception &) {
    // The executor may already be stopping. This is only a best-effort final
    // stow; normal operation requires a positive MAVROS ACK before completion.
  }
}

bool PayloadNode::valid_request_id(const std::string & request_id) const
{
  if (request_id.empty() || request_id.size() > 64U) {
    return false;
  }
  return std::all_of(request_id.begin(), request_id.end(), [](const unsigned char character) {
      return std::isalnum(character) != 0 || character == '-' || character == '_' ||
             character == '.' || character == ':';
    });
}

std::size_t PayloadNode::slot_for_payload(const std::uint8_t payload_index) const
{
  if (payload_index == kPayloadOne) {
    return 0U;
  }
  if (payload_index == kPayloadTwo) {
    return 1U;
  }
  throw std::out_of_range("payload_index must be 1 or 2");
}

std::uint8_t PayloadNode::channel_for_payload(const std::uint8_t payload_index) const
{
  return servo_channels_.at(slot_for_payload(payload_index));
}

std::uint16_t PayloadNode::stow_pwm_for_payload(const std::uint8_t payload_index) const
{
  return servo_stowed_pwm_.at(slot_for_payload(payload_index));
}

std::uint16_t PayloadNode::release_pwm_for_payload(const std::uint8_t payload_index) const
{
  return servo_release_pwm_.at(slot_for_payload(payload_index));
}

std::chrono::nanoseconds PayloadNode::release_duration_for_payload(
  const std::uint8_t payload_index) const
{
  return servo_release_duration_.at(slot_for_payload(payload_index));
}

std::string PayloadNode::action_key(
  const std::uint64_t mission_epoch, const std::string & command_id) const
{
  return std::to_string(mission_epoch) + ":" + command_id;
}

rclcpp_action::GoalResponse PayloadNode::handle_release_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ReleasePayload::Goal> goal)
{
  (void)uuid;
  if (goal->mission_epoch == 0U || !valid_request_id(goal->command_id)) {
    RCLCPP_WARN(get_logger(), "Rejecting release action with invalid epoch/command_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->payload_index != kPayloadOne && goal->payload_index != kPayloadTwo) {
    RCLCPP_WARN(get_logger(), "Rejecting release action with invalid payload_index");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const std::string key = action_key(goal->mission_epoch, goal->command_id);
  const auto existing = action_records_.find(key);
  if (existing != action_records_.end()) {
    if (existing->second.payload_index != goal->payload_index) {
      RCLCPP_ERROR(
        get_logger(),
        "Idempotency conflict: action key '%s' was previously bound to payload %u",
        key.c_str(), static_cast<unsigned int>(existing->second.payload_index));
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Accepting idempotent duplicate action key '%s'", key.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  if (!flight_enable_ || !runtime_gate_open_ || busy_ || fault_latched_ ||
    !mavros_command_client_->service_is_ready())
  {
    RCLCPP_WARN(
      get_logger(), "Rejecting new release action '%s': %s",
      key.c_str(), safety_detail().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if ((goal->payload_index == kPayloadOne && payload_one_consumed_) ||
    (goal->payload_index == kPayloadTwo && payload_two_consumed_))
  {
    RCLCPP_WARN(get_logger(), "Rejecting release action: payload channel already consumed");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (action_records_.size() >= 1024U) {
    RCLCPP_ERROR(get_logger(), "Rejecting release action: idempotency ledger is full");
    return rclcpp_action::GoalResponse::REJECT;
  }

  ActionRecord record;
  record.payload_index = goal->payload_index;
  action_records_.emplace(key, std::move(record));
  busy_ = true;
  interlock_active_ = true;
  detail_ = "release action accepted and reserved; awaiting executor callback";
  publish_safety_status();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PayloadNode::handle_release_cancel(
  const std::shared_ptr<ReleaseGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const std::string key = action_key(goal->mission_epoch, goal->command_id);
  const auto record = action_records_.find(key);
  if (record == action_records_.end() || record->second.terminal ||
    key != active_action_key_)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  action_cancel_requested_ = true;
  detail_ = action_committed_ ?
    "cancel requested after release commit; mandatory restow continues" :
    "cancel requested before release ACK; command outcome will be resolved then restowed";
  publish_payload_status();
  publish_action_feedback(
    payload_state_ == PayloadStatus::RESTOWING ?
    ReleasePayload::Feedback::RESTOWING : ReleasePayload::Feedback::RELEASING);

  if (action_committed_ && payload_state_ == PayloadStatus::RELEASING &&
    pending_phase_ == PendingPhase::NONE)
  {
    begin_restow("cancel requested after committed release ACK", false);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PayloadNode::handle_release_accepted(
  const std::shared_ptr<ReleaseGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const std::string key = action_key(goal->mission_epoch, goal->command_id);
  auto record = action_records_.find(key);
  if (record == action_records_.end()) {
    auto result = std::make_shared<ReleasePayload::Result>();
    result->outcome = ReleasePayload::Result::CONFLICT;
    result->success = false;
    result->detail = "internal idempotency reservation is missing";
    goal_handle->abort(result);
    return;
  }
  if (record->second.terminal) {
    deliver_action_result(goal_handle, record->second.result);
    return;
  }

  record->second.goal_handles.push_back(goal_handle);
  if (record->second.started) {
    publish_action_feedback(
      payload_state_ == PayloadStatus::RESTOWING ?
      ReleasePayload::Feedback::RESTOWING : ReleasePayload::Feedback::RELEASING);
    return;
  }
  start_action_release(key, goal_handle);
}

void PayloadNode::start_action_release(
  const std::string & key,
  const std::shared_ptr<ReleaseGoalHandle> goal_handle)
{
  auto & record = action_records_.at(key);
  record.started = true;
  const auto goal = goal_handle->get_goal();

  active_operation_is_action_ = true;
  active_action_key_ = key;
  action_cancel_requested_ = false;
  action_committed_ = false;
  action_release_ack_ = false;
  action_stow_ack_ = false;
  payload_index_ = goal->payload_index;
  servo_channel_ = channel_for_payload(payload_index_);
  request_id_ = goal->command_id;
  operation_failed_ = false;
  fault_latched_ = false;
  busy_ = true;
  interlock_active_ = true;
  last_command_accepted_ = false;
  last_mav_result_ = 255U;
  stow_attempts_ = 0U;
  ++operation_generation_;

  if (payload_index_ == kPayloadOne) {
    payload_one_consumed_ = true;
  } else {
    payload_two_consumed_ = true;
  }

  publish_action_feedback(ReleasePayload::Feedback::QUEUED);
  start_operation(PayloadCommand::Request::RELEASE_SEQUENCE);
}

void PayloadNode::publish_action_feedback(const std::uint8_t stage)
{
  if (!active_operation_is_action_ || active_action_key_.empty()) {
    return;
  }
  const auto record = action_records_.find(active_action_key_);
  if (record == action_records_.end() || record->second.terminal) {
    return;
  }

  auto feedback = std::make_shared<ReleasePayload::Feedback>();
  feedback->stage = stage;
  feedback->committed = action_committed_;
  feedback->release_ack = action_release_ack_;
  feedback->stow_ack = action_stow_ack_;
  feedback->detail = detail_;
  for (const auto & handle : record->second.goal_handles) {
    if (handle && handle->is_active()) {
      handle->publish_feedback(feedback);
    }
  }
}

void PayloadNode::complete_action(
  const std::uint8_t outcome, const bool success, const std::string & reason)
{
  if (!active_operation_is_action_ || active_action_key_.empty()) {
    return;
  }
  const auto record = action_records_.find(active_action_key_);
  if (record == action_records_.end() || record->second.terminal) {
    return;
  }

  auto result = std::make_shared<ReleasePayload::Result>();
  result->outcome = outcome;
  result->success = success;
  result->committed = action_committed_;
  result->release_ack = action_release_ack_;
  result->stow_ack = action_stow_ack_;
  result->detail = reason;

  publish_action_feedback(ReleasePayload::Feedback::TERMINAL);
  record->second.terminal = true;
  record->second.result = result;
  const auto handles = record->second.goal_handles;

  active_operation_is_action_ = false;
  active_action_key_.clear();
  for (const auto & handle : handles) {
    deliver_action_result(handle, result);
  }
}

void PayloadNode::deliver_action_result(
  const std::shared_ptr<ReleaseGoalHandle> & goal_handle,
  const std::shared_ptr<ReleasePayload::Result> & result) const
{
  if (!goal_handle || !result) {
    return;
  }
  if (result->outcome == ReleasePayload::Result::CANCELED) {
    goal_handle->canceled(result);
  } else if (result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void PayloadNode::handle_payload_command(
  const std::shared_ptr<PayloadCommand::Request> request,
  std::shared_ptr<PayloadCommand::Response> response)
{
  response->accepted = false;
  response->request_id = request->request_id;

  if (!valid_request_id(request->request_id)) {
    response->message = "request_id must be 1..64 characters from [A-Za-z0-9._:-]";
    return;
  }
  if (request->payload_index != kPayloadOne && request->payload_index != kPayloadTwo) {
    response->message = "payload_index must be exactly 1 or 2";
    return;
  }
  if (request->command != PayloadCommand::Request::STOW &&
    request->command != PayloadCommand::Request::RELEASE_SEQUENCE)
  {
    response->message = "command must be STOW(0) or RELEASE_SEQUENCE(1)";
    return;
  }
  if (request->command == PayloadCommand::Request::RELEASE_SEQUENCE &&
    !allow_legacy_release_service_)
  {
    response->message =
      "blocked: formal release requires /cuadc/payload/release action; legacy service release is disabled";
    return;
  }
  if (!flight_enable_) {
    response->message = "blocked: flight_enable is false";
    return;
  }
  if (!runtime_gate_open_) {
    response->message =
      "blocked: exact payload runtime gate and lowercase 64-hex config-bundle SHA-256 are required";
    return;
  }
  if (busy_) {
    response->message = "blocked: payload actuator is busy and mutually interlocked";
    return;
  }
  if (fault_latched_ && request->command == PayloadCommand::Request::RELEASE_SEQUENCE) {
    response->message = "blocked: fault is latched; only an explicit STOW recovery is allowed";
    return;
  }
  if (!mavros_command_client_->service_is_ready()) {
    response->message = "blocked: /mavros/cmd/command is not ready";
    return;
  }
  if (seen_request_ids_.find(request->request_id) != seen_request_ids_.end()) {
    response->message = "blocked: request_id was already accepted by this process";
    return;
  }
  if (seen_request_ids_.size() >= 4096U) {
    response->message = "blocked: request replay ledger is full; restart only after aircraft is safe";
    return;
  }
  if (request->command == PayloadCommand::Request::RELEASE_SEQUENCE &&
    ((request->payload_index == kPayloadOne && payload_one_consumed_) ||
    (request->payload_index == kPayloadTwo && payload_two_consumed_)))
  {
    response->message = "blocked: this payload channel has already consumed its one release attempt";
    return;
  }

  payload_index_ = request->payload_index;
  servo_channel_ = channel_for_payload(payload_index_);
  request_id_ = request->request_id;
  seen_request_ids_.insert(request_id_);
  busy_ = true;
  interlock_active_ = true;
  operation_failed_ = false;
  last_command_accepted_ = false;
  last_mav_result_ = 255U;
  stow_attempts_ = 0U;
  ++operation_generation_;
  active_operation_is_action_ = false;
  active_action_key_.clear();
  action_cancel_requested_ = false;
  action_committed_ = false;
  action_release_ack_ = false;
  action_stow_ack_ = false;

  if (request->command == PayloadCommand::Request::RELEASE_SEQUENCE) {
    if (payload_index_ == kPayloadOne) {
      payload_one_consumed_ = true;
    } else {
      payload_two_consumed_ = true;
    }
  }

  response->accepted = true;
  response->message =
    "queued; wait for matching /cuadc/payload/status request_id to reach STOWED or FAULT";

  start_operation(request->command);
}

void PayloadNode::start_operation(const std::uint8_t command)
{
  cancel_timer(ack_timeout_timer_);
  cancel_timer(release_hold_timer_);
  cancel_timer(stow_retry_timer_);
  pending_phase_ = PendingPhase::NONE;

  if (command == PayloadCommand::Request::STOW) {
    const auto stow_pwm = stow_pwm_for_payload(payload_index_);
    payload_state_ = PayloadStatus::STOWING;
    detail_ = "STOW accepted; requesting profile PWM " + std::to_string(stow_pwm) + " us";
    publish_payload_status();
    publish_safety_status();
    send_servo_command(PendingPhase::STOW, stow_pwm);
    return;
  }

  const auto release_pwm = release_pwm_for_payload(payload_index_);
  payload_state_ = PayloadStatus::RELEASING;
  detail_ = "RELEASE_SEQUENCE accepted; requesting profile PWM " +
    std::to_string(release_pwm) + " us";
  publish_payload_status();
  publish_safety_status();
  publish_action_feedback(ReleasePayload::Feedback::RELEASING);
  send_servo_command(PendingPhase::RELEASE, release_pwm);
}

void PayloadNode::send_servo_command(const PendingPhase phase, const std::uint16_t pwm)
{
  pending_phase_ = phase;
  last_command_accepted_ = false;
  const std::uint64_t generation = operation_generation_;

  cancel_timer(ack_timeout_timer_);
  ack_timeout_timer_ = create_wall_timer(ack_timeout_, [this, generation, phase]() {
      handle_ack_timeout(generation, phase);
    });

  if (!mavros_command_client_->service_is_ready()) {
    cancel_timer(ack_timeout_timer_);
    handle_command_result(phase, false, 255U, "/mavros/cmd/command became unavailable");
    return;
  }

  auto request = std::make_shared<CommandLong::Request>();
  request->broadcast = false;
  request->command = kMavCmdDoSetServo;
  request->confirmation = 0U;
  request->param1 = static_cast<float>(servo_channel_);
  request->param2 = static_cast<float>(pwm);
  request->param3 = 0.0F;
  request->param4 = 0.0F;
  request->param5 = 0.0F;
  request->param6 = 0.0F;
  request->param7 = 0.0F;

  try {
    (void)mavros_command_client_->async_send_request(
      request,
      [this, generation, phase](rclcpp::Client<CommandLong>::SharedFuture future) {
        handle_mavros_response(generation, phase, std::move(future));
      });
  } catch (const std::exception & exception) {
    cancel_timer(ack_timeout_timer_);
    handle_command_result(
      phase, false, 255U,
      std::string("failed to send MAV_CMD_DO_SET_SERVO: ") + exception.what());
  }
}

void PayloadNode::handle_mavros_response(
  const std::uint64_t generation,
  const PendingPhase phase,
  rclcpp::Client<CommandLong>::SharedFuture future)
{
  if (generation != operation_generation_ || phase != pending_phase_) {
    return;
  }

  try {
    const auto response = future.get();
    const bool accepted = response->success && response->result == kMavResultAccepted;
    handle_command_result(
      phase, accepted, response->result,
      accepted ? "MAVROS COMMAND_ACK accepted" : "MAVROS COMMAND_ACK rejected");
  } catch (const std::exception & exception) {
    handle_command_result(
      phase, false, 255U,
      std::string("MAVROS command response exception: ") + exception.what());
  }
}

void PayloadNode::handle_ack_timeout(
  const std::uint64_t generation, const PendingPhase phase)
{
  if (generation != operation_generation_ || phase != pending_phase_) {
    return;
  }
  cancel_timer(ack_timeout_timer_);
  handle_command_result(phase, false, 255U, "MAVROS COMMAND_ACK timeout");
}

void PayloadNode::handle_command_result(
  const PendingPhase phase,
  const bool accepted,
  const std::uint8_t mav_result,
  const std::string & reason)
{
  if (phase != pending_phase_) {
    return;
  }
  cancel_timer(ack_timeout_timer_);
  pending_phase_ = PendingPhase::NONE;
  last_mav_result_ = mav_result;

  if (phase == PendingPhase::RELEASE) {
    if (active_operation_is_action_) {
      action_release_ack_ = accepted;
      action_committed_ = accepted;
    }
    if (accepted) {
      last_command_accepted_ = true;
      detail_ = reason + "; holding release PWM for profile duration " +
        std::to_string(
        std::chrono::duration<double>(release_duration_for_payload(payload_index_)).count()) +
        " s before automatic restow";
      publish_payload_status();
      publish_safety_status();
      if (action_cancel_requested_) {
        begin_restow("cancel requested; committed release will still be safely restowed", false);
      } else {
        schedule_release_hold();
      }
    } else {
      last_command_accepted_ = false;
      begin_restow(reason + "; release outcome is unsafe/unknown", true);
    }
    return;
  }

  if (accepted) {
    if (active_operation_is_action_) {
      action_stow_ack_ = true;
    }
    last_command_accepted_ = !operation_failed_;
    if (operation_failed_) {
      latch_fault(
        "profile restow PWM ACK accepted, but the release phase failed: " + reason,
        mav_result);
    } else {
      payload_state_ = PayloadStatus::STOWED;
      busy_ = false;
      interlock_active_ = false;
      fault_latched_ = false;
      detail_ = reason + "; payload is STOWED at profile PWM " +
        std::to_string(stow_pwm_for_payload(payload_index_)) + " us";
      publish_payload_status();
      publish_safety_status();
      if (active_operation_is_action_) {
        const bool canceled = action_cancel_requested_;
        complete_action(
          canceled ? ReleasePayload::Result::CANCELED : ReleasePayload::Result::SUCCEEDED,
          !canceled,
          canceled ?
          "cancel acknowledged only after mandatory stow ACK; payload is safely STOWED" :
          detail_);
      }
    }
  } else {
    handle_stow_failure(reason, mav_result);
  }
}

void PayloadNode::schedule_release_hold()
{
  const std::uint64_t generation = operation_generation_;
  cancel_timer(release_hold_timer_);
  publish_action_feedback(ReleasePayload::Feedback::HOLDING);
  release_hold_timer_ = create_wall_timer(
    release_duration_for_payload(payload_index_), [this, generation]() {
      if (generation != operation_generation_ || payload_state_ != PayloadStatus::RELEASING) {
        return;
      }
      cancel_timer(release_hold_timer_);
      begin_restow("profile release hold duration elapsed", false);
    });
}

void PayloadNode::begin_restow(const std::string & reason, const bool operation_failed)
{
  cancel_timer(release_hold_timer_);
  operation_failed_ = operation_failed_ || operation_failed;
  payload_state_ = PayloadStatus::RESTOWING;
  stow_attempts_ = 0U;
  detail_ = reason + "; automatic restow is interlocked and mandatory";
  publish_payload_status();
  publish_safety_status();
  publish_action_feedback(ReleasePayload::Feedback::RESTOWING);
  send_next_stow_attempt();
}

void PayloadNode::send_next_stow_attempt()
{
  cancel_timer(stow_retry_timer_);
  ++stow_attempts_;
  const auto stow_pwm = stow_pwm_for_payload(payload_index_);
  detail_ = "requesting profile restow PWM " + std::to_string(stow_pwm) +
    " us; attempt " +
    std::to_string(static_cast<unsigned int>(stow_attempts_)) + "/" +
    std::to_string(static_cast<unsigned int>(kMaximumStowAttempts));
  publish_payload_status();
  publish_action_feedback(ReleasePayload::Feedback::RESTOWING);
  send_servo_command(PendingPhase::STOW, stow_pwm);
}

void PayloadNode::handle_stow_failure(
  const std::string & reason, const std::uint8_t mav_result)
{
  last_command_accepted_ = false;
  if (active_operation_is_action_) {
    action_stow_ack_ = false;
  }
  if (stow_attempts_ < kMaximumStowAttempts) {
    detail_ = reason + "; scheduling bounded profile restow PWM retry";
    publish_payload_status();
    publish_safety_status();
    const std::uint64_t generation = operation_generation_;
    cancel_timer(stow_retry_timer_);
    stow_retry_timer_ = create_wall_timer(kStowRetryDelay, [this, generation]() {
        if (generation != operation_generation_ || !busy_) {
          return;
        }
        cancel_timer(stow_retry_timer_);
        send_next_stow_attempt();
      });
    return;
  }
  latch_fault(
    reason + "; all bounded profile restow PWM attempts failed",
    mav_result);
}

void PayloadNode::latch_fault(const std::string & reason, const std::uint8_t mav_result)
{
  cancel_timer(ack_timeout_timer_);
  cancel_timer(release_hold_timer_);
  cancel_timer(stow_retry_timer_);
  pending_phase_ = PendingPhase::NONE;
  last_mav_result_ = mav_result;
  payload_state_ = PayloadStatus::FAULT;
  busy_ = false;
  interlock_active_ = true;
  fault_latched_ = true;
  last_command_accepted_ = false;
  detail_ = reason + "; RELEASE remains blocked; explicit STOW recovery is permitted";
  publish_payload_status();
  publish_safety_status();
  RCLCPP_ERROR(
    get_logger(), "Payload fault for request '%s': %s",
    request_id_.c_str(), detail_.c_str());
  if (active_operation_is_action_) {
    complete_action(ReleasePayload::Result::FAILED, false, detail_);
  }
}

void PayloadNode::cancel_timer(rclcpp::TimerBase::SharedPtr & timer)
{
  if (timer) {
    timer->cancel();
    timer.reset();
  }
}

void PayloadNode::publish_payload_status()
{
  PayloadStatus message;
  message.header.stamp = now();
  message.header.frame_id = "payload_actuator";
  message.payload_index = payload_index_;
  message.state = payload_state_;
  message.busy = busy_;
  message.last_command_accepted = last_command_accepted_;
  message.request_id = request_id_;
  message.detail = detail_;
  payload_status_publisher_->publish(message);
}

std::string PayloadNode::safety_detail() const
{
  if (fault_latched_) {
    return "payload fault latched; RELEASE blocked; explicit STOW recovery only";
  }
  if (!flight_enable_) {
    return "flight_enable=false; all payload commands blocked";
  }
  if (!runtime_gate_open_) {
    return config_bundle_hash_valid_ ?
           "runtime payload preflight gate is closed" :
           "CUADC_CONFIG_BUNDLE_SHA256 is missing or not 64 lowercase hex characters";
  }
  if (!mavros_command_client_->service_is_ready()) {
    return "/mavros/cmd/command is not ready";
  }
  if (busy_) {
    return "payload operation active; dual-channel mutual interlock engaged";
  }
  return "payload command path ready";
}

void PayloadNode::publish_safety_status()
{
  SafetyStatus message;
  message.header.stamp = now();
  message.header.frame_id = "payload_actuator";
  message.flight_enable = flight_enable_;
  message.runtime_gate_open = runtime_gate_open_;
  message.mavros_command_ready = mavros_command_client_->service_is_ready();
  message.busy = busy_;
  message.interlock_active = interlock_active_;
  message.fault_latched = fault_latched_;
  if (fault_latched_) {
    message.level = SafetyStatus::FAULT;
  } else if (!flight_enable_ || !runtime_gate_open_ || !message.mavros_command_ready) {
    message.level = SafetyStatus::BLOCKED;
  } else if (busy_) {
    message.level = SafetyStatus::DEGRADED;
  } else {
    message.level = SafetyStatus::SAFE;
  }
  message.detail = safety_detail();
  safety_status_publisher_->publish(message);
}

}  // namespace cuadc_payload
