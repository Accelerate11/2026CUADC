#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class SimpleMissionDemo : public rclcpp::Node {
public:
  SimpleMissionDemo() : Node("simple_mission_demo_node") {
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    release_cli_ = create_client<std_srvs::srv::Trigger>("/drop_controller/release");

    waypoints_ = {{
      {0.0, 0.0, 5.0},
      {30.0, -1.2, 5.0},
      {30.0, 0.0, 5.0},
      {55.0, 0.0, 5.0},
      {0.0, 0.0, 5.0},
      {0.0, 0.0, 1.0},
    }};

    timer_ = create_wall_timer(100ms, std::bind(&SimpleMissionDemo::tick, this));
    stage_start_ = now();
    RCLCPP_INFO(get_logger(), "Simple mission demo publishes setpoints only; arm/takeoff remains external.");
  }

private:
  void tick() {
    if (stage_ >= waypoints_.size()) {
      return;
    }

    const auto &wp = waypoints_[stage_];
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = wp[0];
    msg.pose.position.y = wp[1];
    msg.pose.position.z = wp[2];
    msg.pose.orientation.w = 1.0;
    setpoint_pub_->publish(msg);

    const auto elapsed = (now() - stage_start_).seconds();
    if (elapsed < 8.0) {
      return;
    }

    if ((stage_ == 1 || stage_ == 2) && release_cli_->service_is_ready()) {
      release_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    stage_++;
    stage_start_ = now();
    RCLCPP_INFO(get_logger(), "Mission demo advanced to stage %zu", stage_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::array<double, 3>> waypoints_;
  size_t stage_ = 0;
  rclcpp::Time stage_start_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMissionDemo>());
  rclcpp::shutdown();
  return 0;
}
