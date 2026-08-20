#pragma once

#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cuadc_vehicle/command_authority.hpp>

namespace cuadc_vehicle
{

struct VehicleCallbacks
{
  std::function<void(mavros_msgs::msg::State::SharedPtr)> state;
  std::function<void(nav_msgs::msg::Odometry::SharedPtr)> odometry;
  std::function<void(mavros_msgs::msg::ExtendedState::SharedPtr)> extended_state;
  std::function<void(std_msgs::msg::Float64::SharedPtr)> compass_heading;
};

class VehicleAdapter
{
public:
  VehicleAdapter(rclcpp::Node & node, VehicleCallbacks callbacks);
  VehicleAdapter(const VehicleAdapter &) = delete;
  VehicleAdapter & operator=(const VehicleAdapter &) = delete;

  bool grant_mission_authority() noexcept;
  void revoke_for_pilot() noexcept;
  void enter_fault_landing() noexcept;
  void reset_disarmed() noexcept;

  bool may_takeoff() const noexcept;
  bool may_land() const noexcept;
  bool may_arm() const noexcept;
  ControlAuthority authority() const noexcept;

  // Returns false and publishes nothing unless MISSION owns command authority.
  bool publish_position_setpoint(const geometry_msgs::msg::PoseStamped & message);

  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client() const;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client() const;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client() const;

private:
  rclcpp::Node & node_;
  CommandAuthority command_authority_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
};

}  // namespace cuadc_vehicle
