/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
/*代码功能，在apm固件环境下实现自动飞矩形
是否仿真验证：是
是否实飞验证：是
实飞现象：先起了1m,又起了1m,在去往第一个点过程中，出现自旋，原因是本代码go_to_pub坐标系为东北天，即东为x,北为y,右手系，而apm疑似未声明的情况下需要保持朝向为x正向
之前使用起飞订阅来完成take off ，坐标系与地面站显示一致，导致飞高高
更新：V1.1的目的是验证eigen自动坐标系转换
V1.2 验证yaml获取参数
V1.3 将除起飞以外的时间判断改为距离判断走点切换，感觉效果一般，但是善


code1.3将除起飞以外的时间判断改为距离判断走点切换（第312行）。原来用的状态切换都是靠计时的。但是实际飞行过程中，很难把控飞机多长时间会到达预期位置，因为我们比赛场地很大。采用期望位置和当前位置的差距作为切换条件，很好。但是也需要我们合理设定差值的误差范围，同时也必须引入超时强制切换，这个我们后面再谈。
*/
 #include <ros/ros.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/CommandLong.h>
 #include <mavros_msgs/CommandTOL.h>
 #include <mavros_msgs/SetMode.h>
 #include <mavros_msgs/State.h>
 #include <mavros_msgs/OverrideRCIn.h>
 #include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

 enum mission_state
 {
     READY_TO_FLY,
     TAKE_OFF,
     GO_TO_POINT,
     LAND,
     DONE
 };
 
 struct position {
    double x;
    double y;
};

int waypoint_num;
double err_without_z,err_z_takeoff;
 int wp_index = 0;

 // 飞行状态初始化
 enum mission_state ms = READY_TO_FLY;
 

 //  获取飞控状态
 mavros_msgs::State current_state;
 void state_cb(const mavros_msgs::State::ConstPtr& msg) {
     current_state = *msg;
 }
 
 //  获取当前位置
geometry_msgs::PoseStamped local_pos;
void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg; return;
}
geometry_msgs::PoseStamped start_pos;

Eigen::Quaterniond start_q_eigen;
Eigen::Quaterniond expect_q_eigen;
Eigen::Vector3d manual_pos_eigen;
Eigen::Vector3d start_pos_eigen;
Eigen::Vector3d expect_pos_eigen;

std::string frame;
//位置判断函数
//  判断水平方向误差
bool local_pos_check_without_z(const geometry_msgs::PoseStamped tar, double error) {
    // std::cout << "error x is :" << tar.pose.position.x - local_pos.pose.position.x << std::endl
            //   << "error y is :" << tar.pose.position.y - local_pos.pose.position.y << std::endl;
    if ((tar.pose.position.x - local_pos.pose.position.x) * (tar.pose.position.x - local_pos.pose.position.x) 
      + (tar.pose.position.y - local_pos.pose.position.y) * (tar.pose.position.y - local_pos.pose.position.y) 
      < error * error) return 1;
    return 0;
}
//  判断全方向误差
bool local_pos_check_include_z(const geometry_msgs::PoseStamped tar, double error) {
    //  std::cout << "error x is :" << tar.pose.position.x - local_pos.pose.position.x << std::endl
            //    << "error y is :" << tar.pose.position.y - local_pos.pose.position.y << std::endl
            //    << "error z is :" << tar.pose.position.z - local_pos.pose.position.z << std::endl;
    if ((tar.pose.position.x - local_pos.pose.position.x) * (tar.pose.position.x - local_pos.pose.position.x) 
      + (tar.pose.position.y - local_pos.pose.position.y) * (tar.pose.position.y - local_pos.pose.position.y) 
      + (tar.pose.position.z - local_pos.pose.position.z) * (tar.pose.position.z - local_pos.pose.position.z) 
      < error * error) return 1;
    return 0;
}
//  判断竖直方向误差
bool local_pos_check_only_z(double expect_height, double error)
{
    // std::cout << "error z is :" << expect_height - local_pos.pose.position.z << std::endl;
    if (abs(expect_height - local_pos.pose.position.z) < error) return 1;
    return 0;
}

 ros::Publisher expect_pos_pub;
 geometry_msgs::PoseStamped expect_pos;
 void go_to_pub(double x, double y, double z) {
     expect_pos.pose.position.x = x;
     expect_pos.pose.position.y = y;
     expect_pos.pose.position.z = z;
     expect_q_eigen.x() = 0;
     expect_q_eigen.y() = 0;
     expect_q_eigen.z() = 0;
     expect_q_eigen.w() = 1;
     expect_q_eigen = start_q_eigen * expect_q_eigen;
     expect_pos.pose.orientation.x = expect_q_eigen.x();
     expect_pos.pose.orientation.y = expect_q_eigen.y();
     expect_pos.pose.orientation.z = expect_q_eigen.z();
    expect_pos.pose.orientation.w = expect_q_eigen.w();
     expect_pos.header.frame_id = frame;
     expect_pos_pub.publish(expect_pos);
     ROS_DEBUG("Publishing target position: [%.2f, %.2f, %.2f]", x, y, z); // 新增调试信息
 }
 
 void end_go_to_pub(double destination_z)
{
    // 保持终点水平位置
    go_to_pub(expect_pos.pose.position.x, expect_pos.pose.position.y, -200);
    expect_pos.pose.position.z = destination_z;
}

int takeoff_height = 2;

geometry_msgs::Point take_off_pos;

 int main(int argc, char **argv) {
     ros::init(argc, argv, "ceshi");
     ros::NodeHandle nh("~");
    
    //获取参数
    nh.getParam("waypoint_num", waypoint_num);

    position map[4];
    nh.getParam("position0_x", map[0].x);
    map[0].x = 1;
    nh.getParam("postion0_y", map[0].y);
    map[0].y = 0;
    nh.getParam("position1_x", map[1].x);
    map[1].x = 1;
    nh.getParam("position1_y", map[1].y);
    map[1].y = 1;
    nh.getParam("position2_x", map[2].x);
    map[2].x = 0;
    nh.getParam("position2_y", map[2].y);
    map[2].y = 1;
    nh.getParam("position3_x", map[3].x);
    map[3].x = 0;
    nh.getParam("position3_y", map[3].y);
    map[3].y = 0;

    nh.getParam("err_without_z", err_without_z);
    nh.getParam("err_z_takeoff", err_z_takeoff);
    //订阅当前位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
    
     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
         ("/mavros/state", 10, state_cb);
     expect_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
         ("/mavros/setpoint_position/local", 10);
     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
         ("/mavros/cmd/arming");
     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
         ("/mavros/set_mode");
 
     ros::Publisher override_rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
         ("/mavros/rc/override", 10);
     mavros_msgs::OverrideRCIn override_rc;
     override_rc.channels[3] = 1500;
 
     ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
         ("/mavros/cmd/takeoff");
     mavros_msgs::CommandTOL takeoff_command;
     takeoff_command.request.altitude = takeoff_height;
     bool takeoff_initiated = false;
    bool takeoff_completed = false;

     ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
         ("/mavros/cmd/land");
     mavros_msgs::CommandTOL land_command;
     land_command.request.longitude = 0;
     land_command.request.latitude = 0;
     land_command.request.altitude = 0;
 
     // the setpoint publishing rate MUST be faster than 2Hz
     ros::Rate rate(10.0);
 
     // wait for FCU connection
     ROS_INFO("Waiting for FCU connection...");
     int connection_counter = 0;
     while (ros::ok() && !current_state.connected) {
         ros::spinOnce();
         rate.sleep();
         if (++connection_counter % 50 == 0) {  // 每2.5秒提示一次
             ROS_WARN_STREAM_THROTTLE(2.5, "Waiting for FCU connection...");
         }
     }
     ROS_INFO("FCU connected!");
 
     // 初始化目标位置
     expect_pos.pose.position.x = 0;
     expect_pos.pose.position.y = 0;
     expect_pos.pose.position.z = takeoff_height;
 
     
     // 切换到 GUIDED 模式
     mavros_msgs::SetMode GUIDED_set_mode;
     GUIDED_set_mode.request.custom_mode = "GUIDED";
 
     mavros_msgs::SetMode AUTO_set_mode;
     AUTO_set_mode.request.custom_mode = "AUTO";
 
     mavros_msgs::CommandBool arm_cmd;
     arm_cmd.request.value = true;
 
     char start_cmd = '\0';
     ROS_INFO("Waiting for start command (press Enter)...");
     for (start_cmd = getchar(); start_cmd != '\n'; start_cmd = getchar());
     ROS_INFO("Launch command received!");
     ros::Time last_time = ros::Time::now();
 
     while (ros::ok()) {
         ROS_DEBUG_THROTTLE(1.0, "Current state: %d", ms);  // 限速状态打印
 
         switch (ms) {
          case READY_TO_FLY:
             if (current_state.mode != "GUIDED") {
                 ROS_WARN("Attempting to switch from %s to GUIDED", current_state.mode.c_str());
                 if (set_mode_client.call(GUIDED_set_mode)) {
                     if (GUIDED_set_mode.response.mode_sent) {
                         ROS_WARN("GUIDED enabled");
                         last_time = ros::Time::now();
                     } else {
                         ROS_ERROR("Failed to set GUIDED mode");
                     }
                 } else {
                     ROS_ERROR("GUIDED mode service call failed");
                 }
             } 
             else {
                 ROS_INFO("Current mode: GUIDED");
                 if (!current_state.armed) {
                     ROS_INFO_THROTTLE(1.0, "Waiting for arming...");
                     if (ros::Time::now() - last_time > ros::Duration(0.5)) {
                         ROS_WARN("Attempting arming...");
                         if (arming_client.call(arm_cmd)) {
                             if (arm_cmd.response.success) {
                                 ROS_WARN("Vehicle armed");
                                 ms = TAKE_OFF;
                                 start_pos = local_pos;
                                 start_q_eigen.x() = start_pos.pose.orientation.x;
                                 start_q_eigen.y() = start_pos.pose.orientation.y;
                                 start_q_eigen.z() = start_pos.pose.orientation.z;
                                 start_q_eigen.w() = start_pos.pose.orientation.w;
                             } else {
                                 ROS_ERROR("Arming command rejected");
                             }
                         } else {
                             ROS_ERROR("Arming service call failed");
                         }
                         last_time = ros::Time::now();
                     }
                 }
             }
             break;
 
         case TAKE_OFF:
             ROS_INFO("Attempting takeoff to %.1fm", takeoff_command.request.altitude);
             if (!takeoff_command.response.success) {
                 if (ros::Time::now() - last_time > ros::Duration(1.0)) {
                     if (takeoff_client.call(takeoff_command)) {
                         if (takeoff_command.response.success) {
                             ROS_WARN("Takeoff initiated!");
                             ROS_INFO("Ascending to %.1fm (waiting 10s)...", takeoff_command.request.altitude);
                             ros::Duration(10.0).sleep();
                             ms = GO_TO_POINT;
                            
                         } else {
                             ROS_ERROR("Takeoff command rejected");
                         }
                     } else {
                         ROS_ERROR("Takeoff service call failed");
                     }
                     last_time = ros::Time::now();
                 }
             }
             break;
 
         case GO_TO_POINT:
         {
             if (wp_index < waypoint_num) {
                    // 计算目标位置（相对于起飞点）
                    manual_pos_eigen = Eigen::Vector3d(
                        map[wp_index].x, 
                        map[wp_index].y, 
                        takeoff_height
                    );
                    
                    // 应用初始旋转
                    expect_pos_eigen = start_q_eigen * manual_pos_eigen;
                    
                    // 转换为全局坐标系
                     expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
                     expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
                     expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
                    
                    // 发布目标位置
                    go_to_pub(expect_pos.pose.position.x,expect_pos.pose.position.y, expect_pos.pose.position.z);
                    
                    // 检查是否到达航点
                    if (local_pos_check_without_z(expect_pos, err_without_z) && 
                        local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                        ROS_INFO("Navigating to waypoint %d/%d [%.1f, %.1f, %.1f]", 
                         wp_index+1, waypoint_num,expect_pos.pose.position.x, expect_pos.pose.position.y, expect_pos.pose.position.z);
                        ROS_INFO("Reached waypoint %d/%d", wp_index+1, waypoint_num);
                        wp_index++;
                    }
                } 
                // 所有航点完成
                else {
                    ROS_INFO("All waypoints completed");
                    ms = LAND;
                }
                break;
         }
         case LAND:
             ROS_WARN("Initiating landing sequence");
             if (ros::Time::now() - last_time > ros::Duration(1.0)) {
                 if (land_client.call(land_command)) {
                     if (land_command.response.success) {
                         ROS_WARN("Landing command accepted");
                         ROS_INFO("Descending... (check actual altitude)");
                         last_time = ros::Time::now();
                         ms = DONE;
                     } else {
                         ROS_ERROR("Landing command rejected");
                     }
                 } else {
                     ROS_ERROR("Land service call failed");
                 }
             }
             break;
 
         default:
             ROS_WARN("Unknown state");
             break;
         }
 
         if (ms == DONE) {
             ROS_WARN("Mission complete");
             break;
         }
 
         // 持续发布目标位置
         expect_pos_pub.publish(expect_pos);
 
         ros::spinOnce();
         rate.sleep();
     }
     ROS_INFO("Shutting down");
     return 0;
 }
