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
*/
 #include <ros/ros.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/CommandLong.h>
 #include <mavros_msgs/CommandTOL.h>
 #include <mavros_msgs/SetMode.h>
 #include <mavros_msgs/State.h>
 #include <mavros_msgs/OverrideRCIn.h>
 
 enum mission_state
 {
     READY_TO_FLY,
     TAKE_OFF,
     GO_TO_POINT,
     LAND,
     DONE
 };
 
 // 飞行状态初始化
 enum mission_state ms = READY_TO_FLY;
 
 mavros_msgs::State current_state;
 void state_cb(const mavros_msgs::State::ConstPtr& msg) {
     current_state = *msg;
 }
 
 ros::Publisher expect_pos_pub;//这个变量表示目标位置坐标这一话题
 geometry_msgs::PoseStamped expect_pos;
 void go_to_pub(double x, double y, double z) {
     expect_pos.pose.position.x = x;
     expect_pos.pose.position.y = y;
     expect_pos.pose.position.z = z;
     expect_pos.header.frame_id = "map";
     expect_pos_pub.publish(expect_pos);//将expect_pos发布到mavros/setpoint_position/local话题
     ROS_DEBUG("Publishing target position: [%.2f, %.2f, %.2f]", x, y, z); // 新增调试信息
 }
 
 int main(int argc, char **argv) {
     ros::init(argc, argv, "offb_node");
     ros::NodeHandle nh("~");
 
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
     takeoff_command.request.altitude = 1;
 
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
     expect_pos.pose.position.z = 1.0;
 
     // 预发送一些目标位置，确保飞控进入 OFFBOARD 模式
     ROS_INFO("Initializing setpoints...");
     for (int i = 0; i < 100 && ros::ok(); ++i) {
         expect_pos_pub.publish(expect_pos);
         if (i % 20 == 0) {  // 每秒打印一次
             ROS_DEBUG("Sending initial setpoint [%d/100]", i+1);
         }
         ros::spinOnce();
         rate.sleep();
     }
     
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
             ROS_WARN("Starting waypoint navigation");
             size_t waypoint_num = 5;
             double x[] = {0, 1, 1, 0, 0};
             double y[] = {0, 0, 1, 1, 0};
             double z = 1.0;
             
             for (size_t i = 0; i < waypoint_num; ++i) {
                 ROS_INFO("Navigating to waypoint %zu/%zu [%.1f, %.1f, %.1f]", 
                         i+1, waypoint_num, x[i], y[i], z);
                 go_to_pub(x[i], y[i], z);
                 ros::Duration(10.0).sleep(); // 等待10秒到达下一个点
                 ROS_INFO("Reached waypoint %zu (waiting 10s)", i+1);
             }
             ms = LAND;
             last_time = ros::Time::now();
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
