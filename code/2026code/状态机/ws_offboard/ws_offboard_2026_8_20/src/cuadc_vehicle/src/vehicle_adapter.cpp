#include <cuadc_vehicle/vehicle_adapter.hpp>

#include <stdexcept>
#include <utility>

namespace cuadc_vehicle
{

VehicleAdapter::VehicleAdapter(rclcpp::Node & node, VehicleCallbacks callbacks)
: node_(node)
{
  if (!callbacks.state || !callbacks.odometry || !callbacks.extended_state ||
    !callbacks.compass_heading)
  {
    throw std::invalid_argument("VehicleAdapter requires all telemetry callbacks");
  }
  state_sub_ = node.create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", rclcpp::QoS(10).reliable(), std::move(callbacks.state));
  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
    std::move(callbacks.odometry));
  extended_state_sub_ = node.create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", rclcpp::SensorDataQoS(),
    std::move(callbacks.extended_state));
  compass_sub_ = node.create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
    std::move(callbacks.compass_heading));
  setpoint_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());
  takeoff_client_ = node.create_client<mavros_msgs::srv::CommandTOL>(
    "/mavros/cmd/takeoff");
  land_client_ = node.create_client<mavros_msgs::srv::CommandTOL>(
    "/mavros/cmd/land");
  arm_client_ = node.create_client<mavros_msgs::srv::CommandBool>(
    "/mavros/cmd/arming");
}

bool VehicleAdapter::grant_mission_authority() noexcept
{
  return command_authority_.grant_mission();
}

void VehicleAdapter::revoke_for_pilot() noexcept
{
  command_authority_.revoke_for_pilot();
}

void VehicleAdapter::enter_fault_landing() noexcept
{
  command_authority_.revoke_for_fault();
}

void VehicleAdapter::reset_disarmed() noexcept
{
  command_authority_.reset_disarmed();
}

bool VehicleAdapter::may_takeoff() const noexcept
{
  return command_authority_.mission_may_command();
}

bool VehicleAdapter::may_land() const noexcept
{
  return command_authority_.mission_may_command() ||
         command_authority_.fault_landing_may_command();
}

bool VehicleAdapter::may_arm() const noexcept
{
  return command_authority_.mission_may_command();
}

ControlAuthority VehicleAdapter::authority() const noexcept
{
  return command_authority_.current();
}

bool VehicleAdapter::publish_position_setpoint(
  const geometry_msgs::msg::PoseStamped & message)
{
  if (!command_authority_.mission_may_command()) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 2000,
      "Rejected position setpoint: mission does not own command authority");
    return false;
  }
  setpoint_pub_->publish(message);
  return true;
}

rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr
VehicleAdapter::takeoff_client() const
{
  return takeoff_client_;
}

rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr
VehicleAdapter::land_client() const
{
  return land_client_;
}

rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr
VehicleAdapter::arm_client() const
{
  return arm_client_;
}

}  // namespace cuadc_vehicle
