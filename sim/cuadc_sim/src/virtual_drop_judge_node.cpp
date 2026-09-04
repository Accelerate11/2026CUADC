#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

struct Bucket {
  double x;
  double y;
  double radius;
  int score;
};

class VirtualDropJudge : public rclcpp::Node {
public:
  VirtualDropJudge() : Node("virtual_drop_judge_node") {
    buckets_ = {
      {"drop_1", {30.0, -1.2, 0.075, 500}},
      {"drop_2", {30.0, 0.0, 0.10, 300}},
      {"drop_3", {30.0, 1.2, 0.125, 100}},
    };

    target_bucket_id_ = declare_parameter<std::string>("target_bucket_id", "drop_1");
    b_zone_radius_ = declare_parameter<double>("b_zone_radius", 0.5);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        have_odom_ = true;
      });

    release_srv_ = create_service<std_srvs::srv::Trigger>(
      "/drop_controller/release",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_release(response);
      });

    RCLCPP_INFO(get_logger(), "Virtual drop judge ready on /drop_controller/release");
  }

private:
  void handle_release(const std_srvs::srv::Trigger::Response::SharedPtr &response) {
    release_count_++;
    target_bucket_id_ = get_parameter("target_bucket_id").as_string();

    if (!have_odom_) {
      response->success = false;
      response->message = "No odometry received yet";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto it = buckets_.find(target_bucket_id_);
    if (it == buckets_.end()) {
      response->success = false;
      response->message = "Unknown target bucket: " + target_bucket_id_;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto &bucket = it->second;
    const double dx = x_ - bucket.x;
    const double dy = y_ - bucket.y;
    const double error = std::hypot(dx, dy);

    std::string zone = "invalid";
    int score = 0;
    if (error <= bucket.radius) {
      zone = "A";
      score = bucket.score;
    } else if (error <= b_zone_radius_) {
      zone = "B";
      score = 50;
    }

    std::ostringstream msg;
    msg << "release=" << release_count_
        << " target=" << target_bucket_id_
        << " drone=(" << x_ << "," << y_ << ")"
        << " bucket=(" << bucket.x << "," << bucket.y << ")"
        << " error=" << error
        << " zone=" << zone
        << " score=" << score;

    response->success = score > 0;
    response->message = msg.str();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;

  std::map<std::string, Bucket> buckets_;
  std::string target_bucket_id_;
  double b_zone_radius_ = 0.5;
  double x_ = 0.0;
  double y_ = 0.0;
  bool have_odom_ = false;
  int release_count_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualDropJudge>());
  rclcpp::shutdown();
  return 0;
}
