/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
/*代码功能，在apm固件环境下实现自动投靶
是否仿真验证：否
是否实飞验证：是，这个PWM是相对的，也就是当前舵机状态不一样，同样的PWM会出现不一样效果，当前数值经过验证可行，在初始舵机关闭状态，将PWM设为1950即可打开投放
code2 主要是用来测试舵机控制的。具体的PWM需要实际测试。取决于舵机摇臂安装角。
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
     PUT,
     GO_TO_POINT,
     LAND,
     DONE
 };
 

// 舵机控制相关参数
#define SERVO_NUM_1 10          // 舵机通道号（根据实际接线修改）
#define SERVO_NUM_2 9          // 舵机通道号（根据实际接线修改）
#define SERVO_NEUTRAL 1300   // 舵机中位值
#define SERVO_MIN 1100        // 舵机最小PWM
#define SERVO_MAX 1950        // 舵机最大PWM

bool control_servo(ros::ServiceClient& servo_client, int servo_id, int pwm_value)
{
    mavros_msgs::CommandLong servo_cmd;
    servo_cmd.request.command = 183; // DO_SET_SERVO命令
    servo_cmd.request.param1 = servo_id; // 舵机ID
    servo_cmd.request.param2 = pwm_value; // PWM值
    
    if (servo_client.call(servo_cmd)) {
        ROS_INFO("Servo command sent: channel=%d, pwm=%d", servo_id, pwm_value);
        return servo_cmd.response.success;
    } else {
        ROS_ERROR("Failed to call servo command service");
        return false;
    }
}
    bool servo_action_completed = false;


 // 飞行状态初始化
 enum mission_state ms = READY_TO_FLY;
 
 mavros_msgs::State current_state;
 void state_cb(const mavros_msgs::State::ConstPtr& msg) {
     current_state = *msg;
 }
 
 ros::Publisher expect_pos_pub;
 geometry_msgs::PoseStamped expect_pos;
 void go_to_pub(double x, double y, double z) {
     expect_pos.pose.position.x = x;
     expect_pos.pose.position.y = y;
     expect_pos.pose.position.z = z;
     expect_pos.header.frame_id = "map";
     expect_pos_pub.publish(expect_pos);
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
    
     ros::ServiceClient servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
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
           
            ms = PUT;
             break;
 
         case PUT:
             ROS_WARN("Performing servo action");
            

            if (!servo_action_completed) {

                
            
                control_servo(servo_client, SERVO_NUM_1, SERVO_MAX);
                ros::Duration(2.0).sleep();
                

                control_servo(servo_client, SERVO_NUM_2, SERVO_MAX);
                ros::Duration(1.0).sleep();
                
                servo_action_completed = true;
                ROS_INFO("Servo action completed");
            }
            
            // 返回航点导航，继续剩余航点
            ms = LAND;
            break;
 

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