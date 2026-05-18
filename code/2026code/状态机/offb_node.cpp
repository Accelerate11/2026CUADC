/**
 * @file offb_node.cpp
 * @brief Pure takeoff solution – no setpoint during takeoff to avoid conflict.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

enum class MissionState {
    WAITING_FCU,
    SETTING_GUIDED,
    ARMING,
    TAKEOFF,
    FLYING_WAYPOINTS,
    LANDING,
    DONE
};

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offb_node") {
        // QoS compatible with mavros
        rclcpp::QoS odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        rclcpp::QoS state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", state_qos,
            std::bind(&OffboardControl::state_cb, this, std::placeholders::_1));

        local_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom", odom_qos,
            std::bind(&OffboardControl::local_pos_cb, this, std::placeholders::_1));

        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());

        arming_cli_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_cli_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_cli_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
        land_cli_   = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControl::control_loop, this));

        // 初始目标点（原点）
        current_target_ = {0.0, 0.0, 0.0};
        publish_setpoint_enabled_ = true;   // 初始化时允许发布

        RCLCPP_INFO(this->get_logger(), "Offboard node started (pure takeoff mode)");
    }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void local_pos_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_altitude_ = msg->pose.pose.position.z;
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
    }

    void publish_setpoint(double x, double y, double z) {
        if (!publish_setpoint_enabled_) return;  // 关键：不允许发布时直接返回
        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp = this->now();
        sp.header.frame_id = "map";
        sp.pose.position.x = x;
        sp.pose.position.y = y;
        sp.pose.position.z = z;
        sp.pose.orientation.w = 1.0;
        setpoint_pub_->publish(sp);
        current_target_ = {x, y, z};
    }

    void call_set_mode(const std::string& mode) {
        if (!set_mode_cli_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode;
        auto future = set_mode_cli_->async_send_request(req);
        pending_set_mode_future_ = future.future.share();
        RCLCPP_INFO(get_logger(), "Requested mode: %s", mode.c_str());
    }

    void call_arm(bool arm) {
        if (!arming_cli_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = arm;
        auto future = arming_cli_->async_send_request(req);
        pending_arm_future_ = future.future.share();
        RCLCPP_INFO(get_logger(), "Requested arming: %s", arm ? "true" : "false");
    }

    void call_takeoff(float alt) {
        if (!takeoff_cli_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Takeoff service not ready");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = alt;
        auto future = takeoff_cli_->async_send_request(req);
        pending_takeoff_future_ = future.future.share();
        RCLCPP_INFO(get_logger(), "Takeoff command sent to %.1f m", alt);
    }

    void call_land() {
        if (!land_cli_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        land_cli_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "Land command sent");
    }

    void control_loop() {
        // 仅在允许发布时才发布 setpoint
        if (publish_setpoint_enabled_) {
            publish_setpoint(current_target_.x, current_target_.y, current_target_.z);
        }

        // 检查异步服务响应
        if (pending_set_mode_future_.valid() &&
            pending_set_mode_future_.wait_for(0s) == std::future_status::ready) {
            auto result = pending_set_mode_future_.get();
            if (result->mode_sent) {
                RCLCPP_INFO(get_logger(), "GUIDED mode confirmed");
                guided_mode_confirmed_ = true;
            } else {
                RCLCPP_WARN(get_logger(), "Set mode failed, will retry");
            }
            pending_set_mode_future_ = {};
        }

        if (pending_arm_future_.valid() &&
            pending_arm_future_.wait_for(0s) == std::future_status::ready) {
            auto result = pending_arm_future_.get();
            if (result->success) {
                RCLCPP_INFO(get_logger(), "Arming confirmed");
                armed_confirmed_ = true;
                // 解锁成功后，可以预先设置目标点（但不发布，因为 takeoff 状态会禁用发布）
                // 等起飞完成后再启用发布时使用该目标点
                current_target_ = {0.0, 0.0, TAKEOFF_ALT};
                RCLCPP_INFO(get_logger(), "Setpoint target set to (0,0,%.1f) (will be published after takeoff)", TAKEOFF_ALT);
            } else {
                RCLCPP_WARN(get_logger(), "Arming failed, will retry");
            }
            pending_arm_future_ = {};
        }

        if (pending_takeoff_future_.valid() &&
            pending_takeoff_future_.wait_for(0s) == std::future_status::ready) {
            auto result = pending_takeoff_future_.get();
            if (result->success) {
                RCLCPP_INFO(get_logger(), "Takeoff command accepted by FCU");
            } else {
                RCLCPP_WARN(get_logger(), "Takeoff command rejected");
            }
            pending_takeoff_future_ = {};
        }

        // 状态机
        switch (mission_state_) {
            case MissionState::WAITING_FCU:
                if (current_state_.connected) {
                    RCLCPP_INFO(get_logger(), "FCU connected");
                    mission_state_ = MissionState::SETTING_GUIDED;
                }
                break;

            case MissionState::SETTING_GUIDED:
                if (current_state_.mode == "GUIDED") {
                    guided_mode_confirmed_ = true;
                }
                if (!guided_mode_confirmed_ && !pending_set_mode_future_.valid()) {
                    call_set_mode("GUIDED");
                } else if (guided_mode_confirmed_) {
                    mission_state_ = MissionState::ARMING;
                }
                break;

            case MissionState::ARMING:
                if (current_state_.armed) {
                    armed_confirmed_ = true;
                }
                if (!armed_confirmed_ && !pending_arm_future_.valid()) {
                    call_arm(true);
                } else if (armed_confirmed_) {
                    // 解锁成功，进入起飞状态
                    mission_state_ = MissionState::TAKEOFF;
                }
                break;

            case MissionState::TAKEOFF: {
                static bool takeoff_sent = false;
                // 关键：起飞期间禁用 setpoint 发布
                publish_setpoint_enabled_ = false;

                if (!takeoff_sent) {
                    call_takeoff(TAKEOFF_ALT);
                    takeoff_sent = true;
                    takeoff_start_time_ = this->now();
                }

                // 检查高度是否达到目标
                if (current_altitude_ >= TAKEOFF_ALT * 0.95) {
                    RCLCPP_INFO(get_logger(), "Takeoff reached target altitude (%.2f m)", current_altitude_);
                    // 起飞完成，重新允许发布 setpoint
                    publish_setpoint_enabled_ = true;
                    mission_state_ = MissionState::FLYING_WAYPOINTS;
                    takeoff_sent = false;
                    waypoint_index_ = 0;
                    last_waypoint_time_ = this->now();
                    // 确保当前目标点为起飞点（后续航点会更新）
                    current_target_ = {0.0, 0.0, TAKEOFF_ALT};
                } else if ((this->now() - takeoff_start_time_).seconds() > 60.0) {
                    RCLCPP_WARN(get_logger(), "Takeoff timeout (60s), forcing to waypoints");
                    publish_setpoint_enabled_ = true;   // 超时后也恢复发布
                    mission_state_ = MissionState::FLYING_WAYPOINTS;
                    takeoff_sent = false;
                }
                break;
            }

            case MissionState::FLYING_WAYPOINTS: {
                const std::vector<std::array<double, 2>> waypoints = {
                    {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}, {0.0, 0.0}
                };
                double z = TAKEOFF_ALT;

                if (waypoint_index_ < waypoints.size()) {
                    double target_x = waypoints[waypoint_index_][0];
                    double target_y = waypoints[waypoint_index_][1];
                    // 更新期望位置
                    current_target_ = {target_x, target_y, z};

                    double dx = current_position_.x - target_x;
                    double dy = current_position_.y - target_y;
                    double dist = std::hypot(dx, dy);
                    if (dist < 0.5) {
                        if ((this->now() - last_waypoint_time_).seconds() > 2.0) {
                            RCLCPP_INFO(get_logger(), "Waypoint %zu reached", waypoint_index_+1);
                            waypoint_index_++;
                            last_waypoint_time_ = this->now();
                        }
                    } else {
                        last_waypoint_time_ = this->now();
                    }
                } else {
                    mission_state_ = MissionState::LANDING;
                }
                break;
            }

            case MissionState::LANDING: {
                static bool land_sent = false;
                if (!land_sent) {
                    call_land();
                    land_sent = true;
                    // 降落过程中仍可发布 setpoint（目标地面点）
                    current_target_ = {current_position_.x, current_position_.y, 0.0};
                }
                if (current_altitude_ < 0.2) {
                    RCLCPP_INFO(get_logger(), "Landed, mission complete");
                    mission_state_ = MissionState::DONE;
                }
                break;
            }

            case MissionState::DONE:
                RCLCPP_INFO(get_logger(), "Shutting down");
                rclcpp::shutdown();
                break;
        }
    }

    // 成员变量
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_pos_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    double current_altitude_ = 0.0;
    struct { double x=0, y=0; } current_position_;
    MissionState mission_state_ = MissionState::WAITING_FCU;

    struct Target { double x=0, y=0, z=0; } current_target_;
    bool publish_setpoint_enabled_ = true;

    const double TAKEOFF_ALT = 2.0;
    size_t waypoint_index_ = 0;
    rclcpp::Time last_waypoint_time_;
    rclcpp::Time takeoff_start_time_;

    std::shared_future<std::shared_ptr<mavros_msgs::srv::SetMode::Response>> pending_set_mode_future_;
    std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandBool::Response>> pending_arm_future_;
    std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandTOL::Response>> pending_takeoff_future_;
    bool guided_mode_confirmed_ = false;
    bool armed_confirmed_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
