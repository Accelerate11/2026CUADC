#pragma once

// GUIDED profile transactions are intentionally sequential. A PARAM_SET ACK
// proves the returned parameter value, not physical stability or atomicity.
// Ordinary FCU PARAM_SET may persist across reboot. The mission owns its own
// setpoint stream and flight-response verification while this helper runs.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace cuadc_guided
{
struct FcuProfile
{
  std::string name;
  double speed_xy_m_s{1.0};
  std::vector<std::string> param_ids;
  std::vector<double> param_values;
};

namespace detail
{
struct TransactionConfig
{
  bool enabled{false};
  bool param_switch_enabled{false};
  // An operator confirmation of the exact tested release, e.g. "4.5.7".
  // This is NOT detection or verification of firmware on the connected FCU.
  std::string confirmed_firmware;
  double timeout_s{15.0};
  int retries{1};
  double tolerance{1.0e-5};
  std::vector<std::string> allowlist;
};

enum class OperationKind {Pull, Snapshot, Write, Command, Rollback};
struct Operation
{
  OperationKind kind;
  uint64_t generation;
  uint64_t sequence;
  std::size_t index{0};
  std::string param_id;
  double value{0.0};
};
struct OperationResult
{
  bool success{false};
  bool outcome_known{true};  // ParamSetV2 false does not give a failure reason.
  bool accepted{false};  // COMMAND_ACK MAV_RESULT_ACCEPTED (0).
  bool is_double{false};
  double value{0.0};
  std::vector<double> snapshot;
};

// Pure transaction engine used by the ROS adapter and the offline tests.
// All methods must be called from the same mission timer/callback group.
class ProfileTransaction
{
public:
  explicit ProfileTransaction(TransactionConfig config) : config_(std::move(config)) {}

  void request(FcuProfile profile)
  {
    if (uncertain_) {
      fail("late FCU mutation possible; restart only after manual reconciliation");
      return;
    }
    if (failed()) {return;}  // Transaction failure aborts the mission session.
    if (busy()) {cancel(); return;}
    ++generation_;
    profile_ = std::move(profile);
    pending_.reset();
    touched_.clear();
    snapshot_.clear();
    index_ = 0;
    rollback_index_ = 0;
    attempts_ = 0;
    any_dispatched_ = false;
    if (!config_.enabled) {state_ = State::Ready; message_ = "FCU switching disabled"; return;}
    if (!valid_config()) {return;}
    if (profile_.name.empty() || !std::isfinite(profile_.speed_xy_m_s) ||
      profile_.speed_xy_m_s <= 0.0 || profile_.speed_xy_m_s > 20.0)
    {
      fail("invalid profile name or speed (required 0 < speed <= 20 m/s)"); return;
    }
    if (profile_.param_ids.size() != profile_.param_values.size()) {
      fail("parameter names and values have different lengths"); return;
    }
    std::set<std::string> unique;
    for (std::size_t i = 0; i < profile_.param_ids.size(); ++i) {
      const auto & name = profile_.param_ids[i];
      if (!config_.param_switch_enabled || !runtime_gain(name) ||
        std::find(config_.allowlist.begin(), config_.allowlist.end(), name) ==
        config_.allowlist.end() || !unique.insert(name).second ||
        !std::isfinite(profile_.param_values[i]) || profile_.param_values[i] < 0.0)
      {
        fail("unsafe, duplicate, disabled or non-allowlisted parameter: " + name); return;
      }
    }
    state_ = profile_.param_ids.empty() ? State::Command : State::Pull;
    message_ = "waiting for authority and settled hold";
  }

  // Observing lost authority after a dispatch cancels the whole transaction.
  // Losing settled hold merely pauses subsequent dispatches, with a deadline.
  void observe(double now_s, bool authority_ok, bool settled)
  {
    authority_ok_ = authority_ok;
    settled_ = settled;
    if (!busy()) {return;}
    if (!authority_ok && any_dispatched_) {cancel(); return;}
    expire(now_s);
  }

  std::optional<Operation> next(double now_s, bool authority_ok, bool settled)
  {
    observe(now_s, authority_ok, settled);
    if (!busy()) {return std::nullopt;}
    if (!pending_) {
      pending_ = Operation{operation_kind(), generation_, ++sequence_, index_, "", 0.0};
      if (state_ == State::Write) {
        pending_->param_id = profile_.param_ids[index_];
        pending_->value = profile_.param_values[index_];
      } else if (state_ == State::Rollback) {
        pending_->index = touched_[rollback_index_];
        pending_->param_id = profile_.param_ids[pending_->index];
        pending_->value = snapshot_[pending_->index];
      } else if (state_ == State::Command) {
        pending_->value = profile_.speed_xy_m_s;
      }
      deadline_s_ = now_s + config_.timeout_s;
      dispatched_ = false;
    }
    expire(now_s);
    if (!pending_ || dispatched_ || !authority_ok || !settled) {return std::nullopt;}
    return pending_;
  }

  bool dispatched(const Operation & op, double now_s)
  {
    if (!matches(op)) {return false;}
    dispatched_ = true;
    any_dispatched_ = true;
    if (op.kind == OperationKind::Write &&
      std::find(touched_.begin(), touched_.end(), op.index) == touched_.end())
    {
      // Even an explicit rejection may accompany a changed FCU parameter.
      touched_.push_back(op.index);
    }
    if (now_s >= deadline_s_) {expire(now_s); return false;}
    message_ = "awaiting " + kind_name(op.kind) + " ACK";
    return true;
  }

  void complete(const Operation & op, const OperationResult & result, double now_s)
  {
    expire(now_s);  // A response arriving after deadline cannot release the gate.
    if (!matches(op) || !dispatched_) {return;}
    if (!result.outcome_known && mutating(op.kind)) {
      transport_error(op); return;
    }
    bool good = result.success;
    if (op.kind == OperationKind::Command) {good = good && result.accepted;}
    if (op.kind == OperationKind::Write || op.kind == OperationKind::Rollback) {
      good = good && result.is_double && equal(result.value, op.value);
    }
    if (op.kind == OperationKind::Snapshot) {
      good = good && result.snapshot.size() == profile_.param_ids.size() &&
        std::all_of(result.snapshot.begin(), result.snapshot.end(),
          [](double value) {return std::isfinite(value);});
    }
    pending_.reset();
    dispatched_ = false;
    if (!good) {
      if (attempts_++ < config_.retries) {message_ = "explicit failure; bounded retry"; return;}
      attempts_ = 0;
      if (state_ == State::Rollback) {
        rollback_failed_ = true;
        ++rollback_index_;  // Try restoring every possibly touched parameter.
        if (rollback_index_ == touched_.size()) {finish_rollback();}
      } else if (!touched_.empty()) {
        failure_reason_ = kind_name(op.kind) + " rejected or returned mismatched value";
        state_ = State::Rollback;
        rollback_index_ = 0;
        rollback_failed_ = false;
        message_ = "restoring all possibly touched parameters";
      } else {
        fail(kind_name(op.kind) + " rejected or returned invalid value");
      }
      return;
    }
    attempts_ = 0;
    switch (state_) {
      case State::Pull: state_ = State::Snapshot; break;
      case State::Snapshot: snapshot_ = result.snapshot; state_ = State::Write; break;
      case State::Write:
        if (++index_ == profile_.param_ids.size()) {state_ = State::Command;}
        break;
      case State::Command: state_ = State::Ready; message_ = "all ACKs verified; verify flight response"; break;
      case State::Rollback:
        if (++rollback_index_ == touched_.size()) {finish_rollback();}
        break;
      default: break;
    }
  }

  // A local transport exception after send may still mean the FCU saw it.
  void transport_error(const Operation & op)
  {
    if (!matches(op)) {return;}
    uncertain_ = mutating(op.kind);
    ++generation_;
    pending_.reset();
    fail(uncertain_ ? "transport exception; FCU mutation outcome unknown" : "transport exception");
  }

  void cancel(const std::string & reason = {})
  {
    if (!busy()) {return;}
    if (pending_ && dispatched_ && mutating(pending_->kind)) {uncertain_ = true;}
    ++generation_;  // Every stale response is invalidated, including read-only pulls.
    pending_.reset();
    fail((reason.empty() ? "cancelled" : reason) +
      (uncertain_ ? "; late FCU mutation possible" : "; no further FCU writes"));
  }

  bool ready() const {return state_ == State::Ready;}
  bool failed() const {return state_ == State::Failed;}
  bool busy() const {return state_ != State::Idle && !ready() && !failed();}
  bool pending_matches(const Operation & op) const {return matches(op);}
  bool uncertain() const {return uncertain_;}
  const std::vector<std::string> & param_ids() const {return profile_.param_ids;}
  std::string status() const {return profile_.name + ": " + message_;}

private:
  enum class State {Idle, Pull, Snapshot, Write, Command, Rollback, Ready, Failed};
  static bool runtime_gain(const std::string & name)
  {
    static const std::set<std::string> gains{
      "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
      "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
      "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D"};
    return gains.count(name) != 0;
  }
  static bool mutating(OperationKind kind)
  {return kind == OperationKind::Write || kind == OperationKind::Command || kind == OperationKind::Rollback;}
  static std::string kind_name(OperationKind kind)
  {
    switch (kind) {
      case OperationKind::Pull: return "forced pull";
      case OperationKind::Snapshot: return "snapshot cache read";
      case OperationKind::Write: return "parameter write";
      case OperationKind::Command: return "DO_CHANGE_SPEED";
      case OperationKind::Rollback: return "rollback";
    }
    return "unknown";
  }
  bool valid_config()
  {
    if (!std::isfinite(config_.timeout_s) || config_.timeout_s <= 0.0 ||
      config_.retries < 0 || config_.retries > 3 || !std::isfinite(config_.tolerance) ||
      config_.tolerance <= 0.0 || config_.tolerance > 0.01)
    {fail("invalid timeout, retries (0..3), or tolerance (0..0.01]"); return false;}
    if (config_.param_switch_enabled &&
      !std::regex_match(config_.confirmed_firmware, std::regex("4\\.5\\.[0-9]+")))
    {fail("PID switching requires exact tested firmware 4.5.<patch>"); return false;}
    return true;
  }
  OperationKind operation_kind() const
  {
    switch (state_) {
      case State::Pull: return OperationKind::Pull;
      case State::Snapshot: return OperationKind::Snapshot;
      case State::Write: return OperationKind::Write;
      case State::Rollback: return OperationKind::Rollback;
      default: return OperationKind::Command;
    }
  }
  bool equal(double a, double b) const
  {return std::isfinite(a) && std::abs(a - b) <= config_.tolerance;}
  bool matches(const Operation & op) const
  {return pending_ && op.generation == generation_ && op.sequence == pending_->sequence;}
  void expire(double now_s)
  {
    if (!pending_ || now_s < deadline_s_) {return;}
    const auto kind = pending_->kind;
    uncertain_ = dispatched_ && mutating(kind);
    ++generation_;
    pending_.reset();
    // A timeout does not establish FCU rejection. Never retry or roll back a
    // dispatched mutation: a late service handler could overwrite the restore.
    fail(uncertain_ ? kind_name(kind) + " timeout; outcome unknown; no rollback or resume" :
      kind_name(kind) + " availability/read timeout; mission aborted");
  }
  void fail(std::string reason) {state_ = State::Failed; message_ = std::move(reason);}
  void finish_rollback()
  {
    fail(failure_reason_ + (rollback_failed_ ? "; rollback incomplete; abort mission" :
      "; rollback ACKs verified; abort mission"));
  }

  TransactionConfig config_;
  FcuProfile profile_;
  State state_{State::Idle};
  std::optional<Operation> pending_;
  std::vector<double> snapshot_;
  std::vector<std::size_t> touched_;
  std::size_t index_{0}, rollback_index_{0};
  uint64_t generation_{0}, sequence_{0};
  double deadline_s_{0.0};
  int attempts_{0};
  bool dispatched_{false}, any_dispatched_{false}, uncertain_{false};
  bool authority_ok_{false}, settled_{false}, rollback_failed_{false};
  std::string message_{"idle"}, failure_reason_;
};
}  // namespace detail
}  // namespace cuadc_guided

#ifndef CUADC_GUIDED_PROFILE_TRANSACTION_ONLY
#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/param_pull.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>

namespace cuadc_guided
{
class GuidedProfileClient
{
public:
  explicit GuidedProfileClient(rclcpp::Node & node)
  : node_(node), transaction_(configuration(node))
  {
    command_client_ = node_.create_client<Command>("/mavros/cmd/command");
    pull_client_ = node_.create_client<Pull>("/mavros/param/pull");
    set_client_ = node_.create_client<Set>("/mavros/param/set");
    create_parameter_client();
  }
  ~GuidedProfileClient() {clear_pending(false);}
  GuidedProfileClient(const GuidedProfileClient &) = delete;
  GuidedProfileClient & operator=(const GuidedProfileClient &) = delete;

  void request(FcuProfile profile) {transaction_.request(std::move(profile)); clear_if_stale();}
  void tick(bool authority_ok, bool settled)
  {
    const double time = monotonic_seconds();
    transaction_.observe(time, authority_ok, settled);
    clear_if_stale();
    if (active_) {poll(time);}
    clear_if_stale();
    const auto op = transaction_.next(time, authority_ok, settled);
    if (op) {dispatch(*op, time);}
  }
  bool ready() const {return transaction_.ready();}
  bool failed() const {return transaction_.failed();}
  bool busy() const {return transaction_.busy();}
  void cancel(const std::string & reason = {}) {transaction_.cancel(reason); clear_if_stale();}
  std::string status() const {return transaction_.status();}

private:
  using Command = mavros_msgs::srv::CommandLong;
  using Pull = mavros_msgs::srv::ParamPull;
  using Set = mavros_msgs::srv::ParamSetV2;
  using Kind = detail::OperationKind;
  template<typename T> static T parameter(rclcpp::Node & n, const std::string & name, T value)
  {
    if (!n.has_parameter(name)) {return n.declare_parameter<T>(name, value);}
    return n.get_parameter(name).get_value<T>();
  }
  static detail::TransactionConfig configuration(rclcpp::Node & node)
  {
    detail::TransactionConfig c;
    c.enabled = parameter(node, "guided_fcu_enabled", false);
    c.param_switch_enabled = parameter(node, "guided_fcu_param_switch_enabled", false);
    c.confirmed_firmware = parameter(node, "guided_fcu_confirmed_firmware", std::string{});
    c.timeout_s = parameter(node, "guided_fcu_timeout_s", 15.0);
    c.retries = static_cast<int>(parameter(node, "guided_fcu_retries", int64_t{1}));
    c.tolerance = parameter(node, "guided_fcu_tolerance", 1.0e-5);
    c.allowlist = parameter(node, "guided_fcu_allowlist", std::vector<std::string>{});
    return c;
  }
  static double monotonic_seconds()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void create_parameter_client()
  {parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/param");}
  void dispatch(const detail::Operation & op, double time)
  {
    try {
      if (op.kind == Kind::Pull) {
        if (!pull_client_->service_is_ready()) {return;}
        auto request = std::make_shared<Pull::Request>();
        request->force_pull = true;
        auto call = pull_client_->async_send_request(request);
        request_id_ = call.request_id;
        pull_future_ = call.future.share();
      } else if (op.kind == Kind::Snapshot) {
        if (!parameter_client_) {create_parameter_client();}
        if (!parameter_client_->service_is_ready()) {return;}
        // MAVROS get_parameters reads its ROS parameter cache. Only a successful
        // force_pull above refreshes that cache from the device first.
        snapshot_future_ = parameter_client_->get_parameters(transaction_.param_ids());
        request_id_ = -1;  // AsyncParametersClient does not expose request IDs.
      } else if (op.kind == Kind::Command) {
        if (!command_client_->service_is_ready()) {return;}
        auto request = std::make_shared<Command::Request>();
        request->broadcast = false;
        request->command = 178;  // MAV_CMD_DO_CHANGE_SPEED.
        request->confirmation = 0;
        request->param1 = 1.0F;  // ground speed, param2 is m/s.
        request->param2 = static_cast<float>(op.value);
        request->param3 = -1.0F;
        request->param4 = request->param5 = request->param6 = request->param7 = 0.0F;
        auto call = command_client_->async_send_request(request);
        request_id_ = call.request_id;
        command_future_ = call.future.share();
      } else {
        if (!set_client_->service_is_ready()) {return;}
        auto request = std::make_shared<Set::Request>();
        request->force_set = false;
        request->param_id = op.param_id;
        request->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        request->value.double_value = op.value;
        auto call = set_client_->async_send_request(request);
        request_id_ = call.request_id;
        set_future_ = call.future.share();
      }
      active_ = op;
      transaction_.dispatched(op, time);
    } catch (const std::exception &) {
      transaction_.transport_error(op);
      clear_pending();
    }
  }
  template<typename T> static bool available(const std::shared_future<T> & future)
  {return future.valid() && future.wait_for(std::chrono::seconds{0}) == std::future_status::ready;}
  void poll(double time)
  {
    detail::OperationResult result;
    try {
      switch (active_->kind) {
        case Kind::Pull: {
          if (!available(pull_future_)) {return;}
          const auto response = pull_future_.get();
          result.success = response->success && response->param_received > 0;
          break;
        }
        case Kind::Snapshot: {
          if (!available(snapshot_future_)) {return;}
          const auto parameters = snapshot_future_.get();
          result.success = parameters.size() == transaction_.param_ids().size();
          for (const auto & p : parameters) {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.success = false; break;
            }
            result.snapshot.push_back(p.as_double());
          }
          break;
        }
        case Kind::Command: {
          if (!available(command_future_)) {return;}
          const auto response = command_future_.get();
          result.success = response->success;
          result.accepted = response->result == 0;  // MAV_RESULT_ACCEPTED.
          // Plugin timeouts leave result=0, and IN_PROGRESS is not a final ACK.
          result.outcome_known = (response->success && result.accepted) ||
            (response->result >= 1 && response->result <= 4) ||
            (response->result >= 6 && response->result <= 9);
          break;
        }
        default: {
          if (!available(set_future_)) {return;}
          const auto response = set_future_.get();
          result.success = response->success;
          // This interface cannot distinguish rejection from plugin ACK timeout.
          result.outcome_known = response->success;
          result.is_double = response->value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          result.value = response->value.double_value;
          break;
        }
      }
      transaction_.complete(*active_, result, time);
      clear_pending();
    } catch (const std::exception &) {
      transaction_.transport_error(*active_);
      clear_pending();
    }
  }
  void clear_if_stale()
  {if (active_ && !transaction_.pending_matches(*active_)) {clear_pending();}}
  void clear_pending(bool recreate = true) noexcept
  {
    if (!active_) {return;}
    try {switch (active_->kind) {
      case Kind::Pull: pull_client_->remove_pending_request(request_id_); break;
      case Kind::Command: command_client_->remove_pending_request(request_id_); break;
      case Kind::Write: case Kind::Rollback: set_client_->remove_pending_request(request_id_); break;
      case Kind::Snapshot:
        // This Humble API gives no request ID. Recreate its read-only clients
        // on timeout/cancellation to release outstanding cache-read resources.
        if (!available(snapshot_future_)) {
          parameter_client_.reset();
          if (recreate) {create_parameter_client();}
        }
        break;
    }} catch (const std::exception &) {}
    active_.reset();
    command_future_ = {};
    pull_future_ = {};
    set_future_ = {};
    snapshot_future_ = {};
    request_id_ = -1;
    // Removing ROS pending requests only frees local resources; it cannot
    // retract a MAVLink packet or cancel an FCU change already in progress.
  }

  rclcpp::Node & node_;
  detail::ProfileTransaction transaction_;
  rclcpp::Client<Command>::SharedPtr command_client_;
  rclcpp::Client<Pull>::SharedPtr pull_client_;
  rclcpp::Client<Set>::SharedPtr set_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
  std::optional<detail::Operation> active_;
  int64_t request_id_{-1};
  std::shared_future<Command::Response::SharedPtr> command_future_;
  std::shared_future<Pull::Response::SharedPtr> pull_future_;
  std::shared_future<Set::Response::SharedPtr> set_future_;
  std::shared_future<std::vector<rclcpp::Parameter>> snapshot_future_;
};
}  // namespace cuadc_guided
#endif  // CUADC_GUIDED_PROFILE_TRANSACTION_ONLY
