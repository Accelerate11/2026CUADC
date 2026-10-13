// Real ROS service transport against local fakes only. CTest isolates DDS
// using ROS_DOMAIN_ID=187 and ROS_LOCALHOST_ONLY=1. No MAVROS process or FCU.
#include "../src/guided_profile_client.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
void check(bool good, const char * message)
{if (!good) {throw std::runtime_error(message);}}

struct Fixture
{
  using Command = mavros_msgs::srv::CommandLong;
  using Pull = mavros_msgs::srv::ParamPull;
  using Set = mavros_msgs::srv::ParamSetV2;
  enum class WriteReply {ACCEPT, MISMATCH, UNKNOWN};

  static rclcpp::NodeOptions options(bool enabled, bool pid)
  {
    rclcpp::NodeOptions result;
    result.parameter_overrides({
      rclcpp::Parameter("guided_fcu_enabled", enabled),
      rclcpp::Parameter("guided_fcu_param_switch_enabled", pid),
      rclcpp::Parameter("guided_fcu_confirmed_firmware", "4.5.7"),
      rclcpp::Parameter("guided_fcu_timeout_s", 5.0),
      rclcpp::Parameter("guided_fcu_retries", int64_t{0}),
      rclcpp::Parameter("guided_fcu_allowlist", std::vector<std::string>{"ATC_RAT_RLL_P"})});
    return result;
  }

  explicit Fixture(bool enabled = true, bool pid = false)
  : caller(std::make_shared<rclcpp::Node>("guided_adapter_test", options(enabled, pid))),
    parameter_server(std::make_shared<rclcpp::Node>("param", "/mavros")),
    client(std::make_unique<cuadc_guided::GuidedProfileClient>(*caller))
  {
    parameter_server->declare_parameter<double>("ATC_RAT_RLL_P", 0.1);
    command = parameter_server->create_service<Command>("/mavros/cmd/command",
      [this](const Command::Request::SharedPtr request, Command::Response::SharedPtr response) {
        ++commands;
        check(request->command == 178 && !request->broadcast && request->confirmation == 0,
          "wrong command transport flags");
        check(request->param1 == 1.0F && std::abs(request->param2 - 1.25F) < 1e-6F &&
          request->param3 == -1.0F && request->param4 == 0.0F,
          "speed command units or fields incorrect");
        response->success = true;
        response->result = 0;
      });
    pull = parameter_server->create_service<Pull>("/mavros/param/pull",
      [this](const Pull::Request::SharedPtr request, Pull::Response::SharedPtr response) {
        ++pulls;
        check(request->force_pull, "snapshot did not force a device refresh");
        response->success = true;
        response->param_received = 1;
      });
    set = parameter_server->create_service<Set>("/mavros/param/set",
      [this](const Set::Request::SharedPtr request, Set::Response::SharedPtr response) {
        check(pulls == 1, "write before fresh snapshot");
        check(!request->force_set && request->param_id == "ATC_RAT_RLL_P" &&
          request->value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
          "wrong parameter write type or flags");
        writes.push_back(request->value.double_value);
        const bool restoration = std::abs(request->value.double_value - 0.1) < 1e-8;
        const double applied = reply == WriteReply::MISMATCH && !restoration ?
          0.3 : request->value.double_value;
        parameter_server->set_parameter(rclcpp::Parameter(request->param_id, applied));
        response->success = reply != WriteReply::UNKNOWN;
        response->value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        response->value.double_value = applied;
      });
    executor.add_node(caller);
    executor.add_node(parameter_server);
  }

  void finish()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{8};
    while (!client->ready() && !client->failed() && std::chrono::steady_clock::now() < deadline) {
      client->tick(true, true);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    check(client->ready() || client->failed(), "fake ROS adapter transaction never terminated");
  }

  std::shared_ptr<rclcpp::Node> caller;
  std::shared_ptr<rclcpp::Node> parameter_server;
  std::unique_ptr<cuadc_guided::GuidedProfileClient> client;
  rclcpp::Service<Command>::SharedPtr command;
  rclcpp::Service<Pull>::SharedPtr pull;
  rclcpp::Service<Set>::SharedPtr set;
  rclcpp::executors::SingleThreadedExecutor executor;
  int commands = 0;
  int pulls = 0;
  std::vector<double> writes;
  WriteReply reply = WriteReply::ACCEPT;
};

void run()
{
  {
    Fixture f(false);
    f.client->request({"disabled", 1.25, {}, {}});
    f.client->tick(true, true);
    f.executor.spin_some();
    check(f.client->ready() && f.commands == 0 && f.pulls == 0 && f.writes.empty(),
      "disabled helper sent a hardware request");
  }
  {
    Fixture f;
    f.client->request({"speed_only", 1.25, {}, {}});
    for (int i = 0; i < 20; ++i) {
      f.client->tick(true, false);
      f.executor.spin_some();
    }
    check(f.commands == 0 && !f.client->ready(), "unsettled hold issued command");
    f.finish();
    check(f.client->ready() && f.commands == 1 && f.writes.empty(), "command ACK not accepted");
  }
  {
    Fixture f(true, true);
    f.client->request({"gain", 1.25, {"ATC_RAT_RLL_P"}, {0.2}});
    f.finish();
    check(f.client->ready() && f.pulls == 1 && f.commands == 1 && f.writes.size() == 1,
      "pull snapshot write command sequence failed");
    check(std::abs(f.parameter_server->get_parameter("ATC_RAT_RLL_P").as_double() - 0.2) < 1e-8,
      "accepted double did not match");
  }
  {
    Fixture f(true, true);
    f.reply = Fixture::WriteReply::MISMATCH;
    f.client->request({"mismatch", 1.25, {"ATC_RAT_RLL_P"}, {0.2}});
    f.finish();
    check(f.client->failed() && f.commands == 0 && f.writes.size() == 2 &&
      std::abs(f.writes.back() - 0.1) < 1e-8, "echo mismatch did not restore snapshot and abort");
    check(std::abs(f.parameter_server->get_parameter("ATC_RAT_RLL_P").as_double() - 0.1) < 1e-8,
      "rollback failed to restore fake parameter");
  }
  {
    Fixture f(true, true);
    f.reply = Fixture::WriteReply::UNKNOWN;
    f.client->request({"unknown", 1.25, {"ATC_RAT_RLL_P"}, {0.2}});
    f.finish();
    f.client->request({"resume_denied", 1.25, {}, {}});
    for (int i = 0; i < 20; ++i) {f.client->tick(true, true); f.executor.spin_some();}
    check(f.client->failed() && f.commands == 0 && f.writes.size() == 1,
      "unknown mutation was retried, rolled back, or resumed");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    run();
    std::cout << "fake ROS services: disabled, settled command, fresh snapshot, rollback, unknown passed\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
