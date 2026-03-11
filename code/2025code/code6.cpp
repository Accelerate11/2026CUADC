/*代码功能，单独验证投放瞄准部分
是否仿真验证：是
是否实飞验证：是

code6,单独验证投放瞄准部分。区赛后发愤图强，开始专攻TAR_FIND，GO_TO_TAR，PUT_AIM三个重要状态。发现飞机两点距离较大时，行动过程飞机姿态变化太大，且速度较大，影响目标定位。因此决定多设立几个点，来限制速度和姿态（手动微分这一块），其实有更好的方法实现这一目的，直接在gotopub函数中进行限制，但是笔者愚钝。
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
#include <std_msgs/Float64.h>

enum mission_state
{
    READY_TO_FLY,
    TAKE_OFF,
    GO_TO_PUT,
    TAR_FIND,
    GO_TO_TAR_1,
    PUT_AIM_1,
    PUT_1,
    GO_TO_TAR_2,
    PUT_AIM_2,
    PUT_2,
    RETURN,
    LAND_AIM,
    LAND,
    DONE
};

Eigen::Vector3d guidance_global;
Eigen::Vector3d guidance_local;
//bool is_guidance_valid = false;
double current_d = 0;

struct SpeedProfile {
    double takeoff_speed;         // 起飞速度 (m/s)
    double waypoint_speed;        // 航点飞行速度 (m/s)
    double search_speed;          // 目标搜索速度 (m/s)
    double target_approach_speed; // 目标接近速度 (m/s)
    double precise_aim_speed;     // 精确瞄准速度 (m/s)
    double return_speed;          // 返航速度 (m/s)
};
SpeedProfile speed_profile;
// 引导向量回调函数
void guidance_cb(const geometry_msgs::Point::ConstPtr& msg) {

    guidance_local = Eigen::Vector3d(-(msg->y), -(msg->x), msg->z);
    current_d = guidance_local(2);
}
bool is_stable_arrival(const Eigen::Vector3d& guidance, double threshold, double duration) {
    static ros::Time stable_start_time;
    static bool in_stable = false;

    double offset = std::sqrt(guidance(0) * guidance(0) + guidance(1) * guidance(1));
    
    if (offset < threshold) {
        if (!in_stable) {
            stable_start_time = ros::Time::now();
            in_stable = true;
        }
        if ((ros::Time::now() - stable_start_time).toSec() >= duration) {
            return true;
        }
    } 
    else {
        in_stable = false;
    }
    return false; 
}

// 舵机控制相关参数
#define SERVO_NUM_1 10          // 舵机通道号（根据实际接线修改）
#define SERVO_NUM_2 9           // 舵机通道号（根据实际接线修改）
#define SERVO_NEUTRAL 1300      // 舵机中位值
#define SERVO_MIN 1100          // 舵机最小PWM
#define SERVO_MAX 1950          // 舵机最大PWM

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

struct position {
    double x;
    double y;
};

int waypoint_num;
double err_without_z, err_z_takeoff, det_height, put_height;
int wp_index = 0;
double max_wait_time[15];

double tar_d[4];
position pos_tar[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
bool tar_found[4] = {false, false, false, false};
int put_tar_cnt = 0;
bool end_find_tar = false;
int rank[3] = {3, 2, 1}; // 桶的优先级顺序
bool GO_TO_TAR_1_First = true, GO_TO_TAR_2_First = true;
double camera_offset1_x, camera_offset1_y; // 水瓶1相对于摄像头的偏移，即摄像头指向水瓶1的向量
double camera_offset2_x, camera_offset2_y; 
int current_tar = 0;
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
    local_pos = *msg;
    return;
}

geometry_msgs::PoseStamped start_pos;
Eigen::Quaterniond start_q_eigen = Eigen::Quaterniond::Identity();
geometry_msgs::PoseStamped better_pos[4];

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
geometry_msgs::PoseStamped interp_pos, last_pos;
double dt = 0.1, wp_time, scale, distance;
int interp_cnt = 0;
double ratio1;

void go_to_pub() {
    expect_pos.pose.orientation.x = start_q_eigen.x();
    expect_pos.pose.orientation.y = start_q_eigen.y();
    expect_pos.pose.orientation.z = start_q_eigen.z();
    expect_pos.pose.orientation.w = start_q_eigen.w();

    expect_pos.header.frame_id = frame;

    expect_pos_pub.publish(expect_pos);
    //ROS_DEBUG("Publishing target position: [%.2f, %.2f, %.2f]", x, y, z); // 新增调试信息
}

// void end_go_to_pub(double destination_z)
// {
//     // 保持终点水平位置
//     go_to_pub(expect_pos.pose.position.x, expect_pos.pose.position.y, -200);
//     expect_pos.pose.position.z = destination_z;
// }

double takeoff_height;

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    //获取参数
    nh.getParam("waypoint_num", waypoint_num);

    position map[20];
    nh.getParam("position0_x", map[0].x);
    nh.getParam("position0_y", map[0].y);
    nh.getParam("position1_x", map[1].x);
    nh.getParam("position1_y", map[1].y);
    nh.getParam("position2_x", map[2].x);
    nh.getParam("position2_y", map[2].y);
    nh.getParam("position3_x", map[3].x);
    nh.getParam("position3_y", map[3].y);
    nh.getParam("position4_x", map[4].x);
    nh.getParam("position4_y", map[4].y);
    nh.getParam("position5_x", map[5].x);
    nh.getParam("position5_y", map[5].y);
    nh.getParam("position6_x", map[6].x);
    nh.getParam("position6_y", map[6].y);
    nh.getParam("position7_x", map[7].x);
    nh.getParam("position7_y", map[7].y);
    nh.getParam("position8_x", map[8].x);
    nh.getParam("position8_y", map[8].y);
    nh.getParam("position9_x", map[9].x);
    nh.getParam("position9_y", map[9].y);
    nh.getParam("position10_x", map[10].x);
    nh.getParam("position10_y", map[10].y);
    nh.getParam("position11_x", map[11].x);
    nh.getParam("position11_y", map[11].y);
    nh.getParam("position12_x", map[12].x);
    nh.getParam("position12_y", map[12].y);
    nh.getParam("position13_x", map[13].x);
    nh.getParam("position13_y", map[13].y);
    nh.getParam("position14_x", map[14].x);
    nh.getParam("position14_y", map[14].y);
    nh.getParam("position15_x", map[15].x);
    nh.getParam("position15_y", map[15].y);
    nh.getParam("position16_x", map[16].x);
    nh.getParam("position16_y", map[16].y);
    nh.getParam("position17_x", map[17].x);
    nh.getParam("position17_y", map[17].y);
    nh.getParam("position18_x", map[18].x);
    nh.getParam("position18_y", map[18].y);

    nh.getParam("err_without_z", err_without_z);
    nh.getParam("err_z_takeoff", err_z_takeoff);

    nh.getParam("frame", frame);
    nh.getParam("max_wait_time0", max_wait_time[0]);
    nh.getParam("max_wait_time1", max_wait_time[1]);
    nh.getParam("max_wait_time2", max_wait_time[2]);
    nh.getParam("max_wait_time3", max_wait_time[3]);
    nh.getParam("max_wait_time4", max_wait_time[4]);
    nh.getParam("max_wait_time5", max_wait_time[5]);
    nh.getParam("max_wait_time6", max_wait_time[6]);
    nh.getParam("max_wait_time7", max_wait_time[7]);
    nh.getParam("max_wait_time8", max_wait_time[8]);
    nh.getParam("max_wait_time9", max_wait_time[9]);
    nh.getParam("max_wait_time10", max_wait_time[10]);
    nh.getParam("max_wait_time11", max_wait_time[11]);

    nh.getParam("takeoff_height", takeoff_height);
    nh.getParam("det_height", det_height);
    nh.getParam("put_height", put_height);
    nh.getParam("tar_d1", tar_d[1]);
    nh.getParam("tar_d2", tar_d[2]);
    nh.getParam("tar_d3", tar_d[3]);
    nh.getParam("camera_offset1_x", camera_offset1_x);
    nh.getParam("camera_offset1_y", camera_offset1_y);
    nh.getParam("camera_offset2_x", camera_offset2_x);
    nh.getParam("camera_offset2_y", camera_offset2_y);

    nh.getParam("ratio1", ratio1);

    ros::Subscriber guidance_sub;
    guidance_sub = nh.subscribe<geometry_msgs::Point>("/h_detect/xy", 10, guidance_cb);//获取引导向量

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
    
    ros::ServiceClient servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("/mavros/cmd/land");
    mavros_msgs::CommandTOL land_command;
    land_command.request.longitude = 0;
    land_command.request.latitude = 0;
    land_command.request.altitude = 0;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);
    dt = 1.0 / 10.0;

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
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ROS_DEBUG_THROTTLE(1.0, "Current state: %d", ms);  // 限速状态打印

        switch (ms) {
        case READY_TO_FLY:
        {
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
        }
        case TAKE_OFF:
        {
            ROS_INFO_THROTTLE(0.5, "Attempting takeoff to %.1fm", takeoff_command.request.altitude);
            if (!takeoff_command.response.success) {
                if (ros::Time::now() - last_time > ros::Duration(1.0)) {
                    if (takeoff_client.call(takeoff_command)) {
                        if (takeoff_command.response.success) {
                            ROS_WARN("Takeoff initiated!");
                            ROS_INFO("Ascending to %.1fm (waiting 10s)...", takeoff_command.request.altitude);
                            ros::Duration(10.0).sleep();
                            ms = GO_TO_PUT;
                            start_time = ros::Time::now();
                            manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, takeoff_height);
                            expect_pos_eigen = start_q_eigen * manual_pos_eigen;
                            expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
                            expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
                            expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
                            go_to_pub(); // 发布目标位置
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
        }
        case GO_TO_PUT://投水高度和寻找筒的高度和侦察高度？？？
        {
            // 检查是否到达航点
            if (local_pos_check_without_z(expect_pos, err_without_z) &&
                local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ms = TAR_FIND;
                start_time = ros::Time::now();
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[0])) {
                ROS_ERROR("GO_TO_PUT Timeout");
                ms = TAR_FIND;
                start_time = ros::Time::now();
            }
            break;
        }
        case TAR_FIND:
        {
            if (put_tar_cnt == 3) {
                ROS_INFO("Found 3 targets, GO_TO_TAR_1...");
                end_find_tar = true; 
            }
            else if (put_tar_cnt == 2) {
                if (ros::Time::now() - start_time > ros::Duration(max_wait_time[1])) {
                    ROS_WARN("Found 2 targets, GO_TO_TAR_1...");
                    end_find_tar = true;
                }
            }
            else if (put_tar_cnt == 1) {
                if (ros::Time::now() - start_time > ros::Duration(max_wait_time[2])) {
                    ROS_WARN("Found 1 target, GO_TO_TAR_1...");
                    end_find_tar = true;
                }
            }
            else if (put_tar_cnt == 0) {
                if (ros::Time::now() - start_time > ros::Duration(max_wait_time[3])) {
                    ROS_WARN("Found 0 target, GO_TO_TAR_1...");
                    end_find_tar = true; 
                }
            }

            if (end_find_tar) {
                ms = GO_TO_TAR_1;
                wp_index = 18;
                start_time = ros::Time::now();
                break;
            }
            
            if (local_pos_check_without_z(expect_pos, err_without_z) && local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff))
            {
                // 切换到下一个目标点
                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                            local_pos.pose.position.y - start_pos.pose.position.y,
                                            local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                Eigen::Vector3d expect_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
                ROS_INFO("finding...\n waypoint_num: [%d]\n expect_pos_body: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]", wp_index, 
                         expect_pos_body(0), expect_pos_body(1), expect_pos_body(2),
                         local_pos_body(0), local_pos_body(1), local_pos_body(2));
                         
                if (wp_index > 8) wp_index = 1;
                manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, det_height);
                expect_pos_eigen = start_q_eigen * manual_pos_eigen;
                expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
                expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
                expect_pos.pose.position.z = det_height + start_pos.pose.position.z;
                go_to_pub();
                wp_index++;
            }

            guidance_local(2) = 0;
            guidance_global = start_q_eigen * guidance_local;
            double guidance_norm2 = guidance_global(0) * guidance_global(0) + guidance_global(1) * guidance_global(1);
            static double min_norm2_tar1 = 1000, min_norm2_tar2 = 1000, min_norm2_tar3 = 1000;

            if(fabs(current_d - tar_d[1])<2.4)
            {
                if(guidance_norm2 < min_norm2_tar1) {
                    min_norm2_tar1 = guidance_norm2;
                    pos_tar[1].x = local_pos.pose.position.x + guidance_global(0);//引导向量和local_pos不同步,是否引入修正偏移？
                    pos_tar[1].y = local_pos.pose.position.y + guidance_global(1);
                    better_pos[1] = local_pos;
                }
                if (!tar_found[1]) {
                    tar_found[1] = true;
                    put_tar_cnt++;
                }
                ROS_INFO("15cm has been found!\n pos_tar1: [%.2f, %.2f]", pos_tar[1].x, pos_tar[1].y);
            }
            else if(fabs(current_d - tar_d[2])<2.4)
            {
                if(guidance_norm2 < min_norm2_tar2) {
                    min_norm2_tar2 = guidance_norm2;
                    pos_tar[2].x = local_pos.pose.position.x + guidance_global(0);//引导向量和local_pos不同步,是否引入修正偏移？
                    pos_tar[2].y = local_pos.pose.position.y + guidance_global(1);
                    better_pos[2] = local_pos;
                }
                if (!tar_found[2]) {
                    tar_found[2] = true;
                    put_tar_cnt++;
                }
                ROS_INFO("20cm has been found!\n pos_tar1: [%.2f, %.2f]", pos_tar[2].x, pos_tar[2].y);
            }
            else if (fabs(current_d - tar_d[3])<2.4)
            {
                if(guidance_norm2 < min_norm2_tar3) {
                    min_norm2_tar3 = guidance_norm2;
                    pos_tar[3].x = local_pos.pose.position.x + guidance_global(0);//引导向量和local_pos不同步,是否引入修正偏移？
                    pos_tar[3].y = local_pos.pose.position.y + guidance_global(1);
                    better_pos[3] = local_pos;
                }
                if (!tar_found[3]) {
                    tar_found[3] = true;
                    put_tar_cnt++;
                }
                ROS_INFO("25cm has been found!\n pos_tar1: [%.2f, %.2f]", pos_tar[3].x, pos_tar[3].y);
            }
            break;
        }
        //
        case GO_TO_TAR_1:
        {
            if (GO_TO_TAR_1_First) {
                GO_TO_TAR_1_First = false; 
                int i = 0;
                for (i = 0; i < 3; i++) {
                    if (tar_found[rank[i]]) {
                        expect_pos.pose.position.x = pos_tar[rank[i]].x;
                        expect_pos.pose.position.y = pos_tar[rank[i]].y;
                        expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
                        current_tar = rank[i];
                        go_to_pub();
                        tar_found[rank[i]] = false; // 清除桶标志
                        break;
                    }
                }
                if (i == 3) {
                    ROS_WARN("No target, go to PUT_1");
                    ms = PUT_1; // 如果没有找到目标，直接进入PUT_1状态
                    break;
                }
            }
            // 检查是否到达航点
            if (local_pos_check_without_z(expect_pos, err_without_z) &&
                local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {

                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                               local_pos.pose.position.y - start_pos.pose.position.y,
                                               local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                Eigen::Vector3d expect_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
                ROS_INFO("reached tar1\n expect_pos_body: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]", 
                         expect_pos_body(0), expect_pos_body(1), expect_pos_body(2),
                         local_pos_body(0), local_pos_body(1), local_pos_body(2));

                ms = PUT_AIM_1;
                start_time = ros::Time::now();
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[4])) {
                ROS_ERROR("GO_TO_TAR_1 Timeout");
                ms = PUT_AIM_1; // 超时后切换到下一状态
                start_time = ros::Time::now();
            }
            break;
        }
        //把上一次识别到圆的位置保存下来，若没有识别到圆，则使用上一次识别到圆的位置
        case PUT_AIM_1:
        {
            static double min_norm2 = 1000;

            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[5])) {
                ROS_WARN("PUT_AIM_1 Timeout");
                ms = PUT_1; // 超时后切换到下一状态
                start_time = ros::Time::now();
            }

            if (current_d == 0 || fabs(current_d - tar_d[current_tar]) > 5) {
                expect_pos = better_pos[current_tar];
                expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
                Eigen::Vector3d better_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                better_pos_body = start_q_eigen.inverse() * better_pos_body;
                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                               local_pos.pose.position.y - start_pos.pose.position.y,
                                               local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                ROS_INFO("PUT_AIM_1\n no circle or not right circle, guidance_local:[%.2f, %.2f, %.2f]\n current_d: %.2f", 
                better_pos_body(0)-local_pos_body(0), better_pos_body(1)-local_pos_body(1), better_pos_body(2)-local_pos_body(2), current_d);
                go_to_pub();
                break;//是否设置四处找找的函数？
            }

            guidance_local(0) -= camera_offset1_x;
            guidance_local(1) -= camera_offset1_y;
            guidance_local(2) = 0;

            double norm2 = guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1);
            if (norm2 < min_norm2) {
                min_norm2 = norm2;
                better_pos[current_tar] = local_pos;
            }

            guidance_global = start_q_eigen * guidance_local;
            expect_pos.pose.position.x = local_pos.pose.position.x + guidance_global(0) * ratio1;
            expect_pos.pose.position.y = local_pos.pose.position.y + guidance_global(1) * ratio1;
            expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
            //日志
            Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                            local_pos.pose.position.y - start_pos.pose.position.y,
                                            local_pos.pose.position.z - start_pos.pose.position.z);
            local_pos_body = start_q_eigen.inverse() * local_pos_body;
            Eigen::Vector3d expect_pos_body(local_pos.pose.position.x + guidance_global(0) - start_pos.pose.position.x,
                                            local_pos.pose.position.y + guidance_global(1) - start_pos.pose.position.y,
                                            expect_pos.pose.position.z - start_pos.pose.position.z);
            expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
            ROS_INFO_THROTTLE(0.5,
                "PUT_AIM_1\n guidance_local: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]\n expect_pos_body: [%.2f, %.2f, %.2f]",
                guidance_local(0), guidance_local(1), guidance_local(2),
                local_pos_body(0), local_pos_body(1), local_pos_body(2),
                expect_pos_body(0), expect_pos_body(1), expect_pos_body(2));
            // ROS_INFO("PUT_AIM_1\n guidance_local: [%.2f, %.2f, %.2f]", guidance_local(0), guidance_local(1), guidance_local(2));
            // 发布目标位置
            go_to_pub();
            // 依靠图像检查是否到达航点
            if (is_stable_arrival(guidance_local, 0.08, 1)) {
                ms = PUT_1;
                start_time = ros::Time::now();
            }
            break;
        }
        case PUT_1:
        {
            ROS_WARN("Performing servo action");
            if (control_servo(servo_client, SERVO_NUM_1, SERVO_MAX)) {
                ros::Duration(2.0).sleep();
                ROS_INFO("PUT_1 completed");
            }
            else {
                ROS_ERROR("Failed to control servo");
            }
            ms = GO_TO_TAR_2;
            break;
        }

        case GO_TO_TAR_2:
        {
            if (GO_TO_TAR_2_First) {
                GO_TO_TAR_2_First = false;
                int i = 0;
                for (i = 0; i < 3; i++) {
                    if (tar_found[rank[i]]) {
                        expect_pos.pose.position.x = pos_tar[rank[i]].x;
                        expect_pos.pose.position.y = pos_tar[rank[i]].y; 
                        expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
                        current_tar = rank[i];
                        go_to_pub();
                        tar_found[rank[i]] = false; // 清除桶标志
                        break;
                    }
                }
                if (i == 3) {
                    ROS_WARN("No target, go to PUT_2");
                    ms = PUT_2; // 如果没有找到目标，直接进入PUT_2状态
                    break;
                }
            }
            // 检查是否到达航点
            if (local_pos_check_without_z(expect_pos, err_without_z) &&
                local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ROS_INFO("reached tar2\n expect_pos: [%.2f, %.2f, %.2f]\n local_pos: [%.2f, %.2f, %.2f]", 
                         expect_pos.pose.position.x, expect_pos.pose.position.y, expect_pos.pose.position.z,
                         local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z);
                ms = PUT_AIM_2;
                start_time = ros::Time::now();
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[6])) {
                ROS_ERROR("GO_TO_TAR_2 Timeout");
                ms = PUT_AIM_2;
                start_time = ros::Time::now();
            }
            break;
        }
        case PUT_AIM_2:
        {
            static double min_norm2 = 1000;

            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[5])) {
                ROS_WARN("PUT_AIM_2 Timeout");
                ms = PUT_2; // 超时后切换到下一状态
                start_time = ros::Time::now();
            }

            if (current_d == 0 || fabs(current_d - tar_d[current_tar]) > 2.5) {
                expect_pos = better_pos[current_tar];
                expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
                Eigen::Vector3d better_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                better_pos_body = start_q_eigen.inverse() * better_pos_body;
                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                               local_pos.pose.position.y - start_pos.pose.position.y,
                                               local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                ROS_INFO("PUT_AIM_1\n no circle or not right circle, guidance_local:[%.2f, %.2f, %.2f]\n current_d: %.2f", 
                better_pos_body(0)-local_pos_body(0), better_pos_body(1)-local_pos_body(1), better_pos_body(2)-local_pos_body(2), current_d);
                go_to_pub();
                break;
            }
            
            guidance_local(0) -= camera_offset2_x;
            guidance_local(1) -= camera_offset2_y;
            guidance_local(2) = 0;

            double norm2 = guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1);
            if (norm2 < min_norm2) {
                min_norm2 = norm2;
                better_pos[current_tar] = local_pos;
            }

            guidance_global = start_q_eigen * guidance_local;
            expect_pos.pose.position.x = local_pos.pose.position.x + guidance_global(0) * ratio1;
            expect_pos.pose.position.y = local_pos.pose.position.y + guidance_global(1) * ratio1;
            expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
            //日志
            Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                            local_pos.pose.position.y - start_pos.pose.position.y,
                                            local_pos.pose.position.z - start_pos.pose.position.z);
            local_pos_body = start_q_eigen.inverse() * local_pos_body;
            Eigen::Vector3d expect_pos_body(local_pos.pose.position.x + guidance_global(0) - start_pos.pose.position.x,
                                            local_pos.pose.position.y + guidance_global(1) - start_pos.pose.position.y,
                                            expect_pos.pose.position.z - start_pos.pose.position.z);
            expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
            ROS_INFO_THROTTLE(0.5,
                "PUT_AIM_2\n guidance_local: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]\n expect_pos_body: [%.2f, %.2f, %.2f]",
                guidance_local(0), guidance_local(1), guidance_local(2),
                local_pos_body(0), local_pos_body(1), local_pos_body(2),
                expect_pos_body(0), expect_pos_body(1), expect_pos_body(2));
            //ROS_INFO("PUT_AIM_2\n guidance_local: [%.2f, %.2f, %.2f]", guidance_local(0), guidance_local(1), guidance_local(2));
            // 发布目标位置
            go_to_pub();
            // 依靠图像检查是否到达航点
            if (is_stable_arrival(guidance_local, 0.05, 0.5)) {
                ms = PUT_1;
                start_time = ros::Time::now();
            }
            break;
        }
        case PUT_2:
        {
            ROS_WARN("Performing servo action");
            if (control_servo(servo_client, SERVO_NUM_2, SERVO_MAX)) {
                ros::Duration(2.0).sleep();
                ROS_INFO("PUT_2 completed");
            }
            else {
                ROS_ERROR("Failed to control servo");
            }
            ms = RETURN;
            start_time = ros::Time::now();
            manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, det_height);
            expect_pos_eigen = start_q_eigen * manual_pos_eigen;
            expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
            expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
            expect_pos.pose.position.z = det_height + start_pos.pose.position.z;
            go_to_pub(); // 发布目标位置
            break;
        }

        case RETURN:
        {
            manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, takeoff_height);
            expect_pos_eigen = start_q_eigen * manual_pos_eigen;
            expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
            expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
            expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
            go_to_pub(); // 发布目标位置
            // 检查是否到达航点
            if (local_pos_check_without_z(expect_pos, err_without_z) &&
                local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ms = LAND_AIM;
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[10])) {
                ROS_ERROR("RETURN Timeout");
                ms = LAND_AIM; // 超时后切换到降落状态
            }
            break;
        }
        case LAND_AIM:
        {
            static double min_norm2 = 1000;
            static geometry_msgs::PoseStamped better_pos_land = start_pos;

            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[11])) {
                ROS_WARN("LAND_AIM Timeout");
                ms = LAND; // 超时后切换到下一状态
                start_time = ros::Time::now();
            }

            if (current_d == 0) {
                expect_pos = better_pos_land;
                expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
                //日志
                Eigen::Vector3d better_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                better_pos_body = start_q_eigen.inverse() * better_pos_body;
                ROS_INFO("LAND_AIM\n no circle, better_pos_body:[%.2f, %.2f, %.2f]", 
                better_pos_body(0), better_pos_body(1), better_pos_body(2));
                go_to_pub();
                break;
            }

            double norm2 = guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1);
            if (norm2 < min_norm2) {
                min_norm2 = norm2;
                better_pos_land = local_pos;
            }

            guidance_global = start_q_eigen * guidance_local;
            expect_pos.pose.position.x = local_pos.pose.position.x + guidance_global(0) * ratio1;
            expect_pos.pose.position.y = local_pos.pose.position.y + guidance_global(1) * ratio1;
            expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
            //日志
            Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                            local_pos.pose.position.y - start_pos.pose.position.y,
                                            local_pos.pose.position.z - start_pos.pose.position.z);
            local_pos_body = start_q_eigen.inverse() * local_pos_body;
            Eigen::Vector3d expect_pos_body(local_pos.pose.position.x + guidance_global(0) - start_pos.pose.position.x,
                                            local_pos.pose.position.y + guidance_global(1) - start_pos.pose.position.y,
                                            expect_pos.pose.position.z - start_pos.pose.position.z);
            expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
            ROS_INFO_THROTTLE(0.5,
                "LAND_AIM\n guidance_local: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]\n expect_pos_body: [%.2f, %.2f, %.2f]",
                guidance_local(0), guidance_local(1), guidance_local(2),
                local_pos_body(0), local_pos_body(1), local_pos_body(2),
                expect_pos_body(0), expect_pos_body(1), expect_pos_body(2));
            // ROS_INFO("PUT_AIM_1\n guidance_local: [%.2f, %.2f, %.2f]", guidance_local(0), guidance_local(1), guidance_local(2));
            // 发布目标位置
            go_to_pub();
            // 依靠图像检查是否到达航点
            if (is_stable_arrival(guidance_local, 0.2, 0.5)) {
                ms = LAND;
            }
            break;
        }
        case LAND:
        {
            ROS_WARN("Initiating landing sequence");
            if (ros::Time::now() - last_time > ros::Duration(1.0)) {
                if (land_client.call(land_command)) {
                    if (land_command.response.success) {
                        ROS_WARN("Landing command accepted");
                        ROS_INFO("Descending... (check actual altitude)");
                        ms = DONE;
                    } else {
                        ROS_ERROR("Landing command rejected");
                    }
                } else {
                    ROS_ERROR("Land service call failed");
                }
                last_time = ros::Time::now();
            }
            break;
        }
        default:
            ROS_WARN("Unknown state");
            break;
        }//switch结束

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
