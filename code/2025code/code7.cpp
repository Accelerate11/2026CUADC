/*代码功能，全流程最终版本1，瞄准阶段加入低通滤波
是否仿真验证：是
是否实飞验证：是

存在坐标系偏移现象， 系由于罗盘干扰。决定采用经纬度发点方式新路径。

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
    GO_TO_DET,
    DET,
    RETURN,
    LAND_AIM,
    LAND,
    DONE
};

Eigen::Vector3d guidance_global;
Eigen::Vector3d guidance_local;
//bool is_guidance_valid = false;
double current_d = 0;
bool guidance_exist = false;
double max_tol = 0.1;

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
    double norm = sqrt(guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1));
    guidance_exist = true;
    if(norm > 2000) {
        guidance_exist = false;
        current_d = 0;
    }
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

double limitCorrection(double correction, double guidance) {
    // 确保绝对值至少为 0.01
    if (fabs(correction) < 0.015) {
        correction = (correction >= 0 ? 1 : -1) * 0.015;
    }
    
    // 确保绝对值不超过 guidance 的绝对值
    if (fabs(correction) > fabs(guidance)) {
        correction = guidance;
    }
    
    return correction;
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
int aim_height_cnt=0;
int tar1_cnt,tar2_cnt,tar3_cnt;
double err_without_z, err_without_z2, err_z_takeoff, det_height, put_height, ready_put_height,origin_put_height,height_reduce_norm;
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
Eigen::Quaterniond body_q_eigen = Eigen::Quaterniond::Identity();
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
int camera_offline_cnt = 0;

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

void attitude_correction(int drone_height)
{
    body_q_eigen.x() = local_pos.pose.orientation.x;
    body_q_eigen.y() = local_pos.pose.orientation.y;
    body_q_eigen.z() = local_pos.pose.orientation.z;
    body_q_eigen.w() = local_pos.pose.orientation.w;
    Eigen::Vector3d target_proj_enu = body_q_eigen * Eigen::Vector3d(guidance_local(0), guidance_local(1), -(drone_height - 0.26 + 0.55));//cam_install_height为相机到rtk天线的距离
    target_proj_enu(0) += local_pos.pose.position.x;
    target_proj_enu(1) += local_pos.pose.position.y;
    target_proj_enu(2) += local_pos.pose.position.z;
    Eigen::Vector3d v_enu = body_q_eigen * Eigen::Vector3d(0, 0, -1);
    double L = (target_proj_enu(2) - (-0.3 + start_pos.pose.position.z)) / v_enu(2);// -0.3为桶口中心相对于rtk天线的高度
    Eigen::Vector3d target_enu = target_proj_enu + L * v_enu;
    guidance_global(0) = target_enu(0) - local_pos.pose.position.x;
    guidance_global(1) = target_enu(1) - local_pos.pose.position.y;
    guidance_global(2) = 0;
}

Eigen::Quaterniond createHorizontalFrame(const Eigen::Vector3d& world_forward_flat) {
    Eigen::Vector3d x_axis = world_forward_flat.normalized();
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis; 
    rotation_matrix.col(2) = z_axis;
    Eigen::Quaterniond local_to_global(rotation_matrix);//机体系到世界系的旋转
    return local_to_global.normalized(); // 返回归一化的四元数
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    //获取参数
    nh.getParam("waypoint_num", waypoint_num);

    position map[30];
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
    nh.getParam("position19_x", map[19].x);
    nh.getParam("position19_y", map[19].y);
    nh.getParam("position20_x", map[20].x);
    nh.getParam("position20_y", map[20].y);
    nh.getParam("position21_x", map[21].x);
    nh.getParam("position21_y", map[21].y);
    nh.getParam("position22_x", map[22].x);
    nh.getParam("position22_y", map[22].y);
    nh.getParam("position23_x", map[23].x);
    nh.getParam("position23_y", map[23].y);
    nh.getParam("position24_x", map[24].x);
    nh.getParam("position24_y", map[24].y);
    nh.getParam("position25_x", map[25].x);
    nh.getParam("position25_y", map[25].y);
    nh.getParam("position26_x", map[26].x);
    nh.getParam("position26_y", map[26].y);
    nh.getParam("position27_x", map[27].x);
    nh.getParam("position27_y", map[27].y);
    nh.getParam("position28_x", map[28].x);
    nh.getParam("position28_y", map[28].y);
    nh.getParam("position29_x", map[29].x);
    nh.getParam("position29_y", map[29].y);

    nh.getParam("err_without_z", err_without_z);
    nh.getParam("err_without_z2", err_without_z2);
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
    nh.getParam("ready_put_height", ready_put_height);
    nh.getParam("height_reduce_norm", height_reduce_norm);
    nh.getParam("tar_d1", tar_d[1]);
    nh.getParam("tar_d2", tar_d[2]);
    nh.getParam("tar_d3", tar_d[3]);
    nh.getParam("camera_offset1_x", camera_offset1_x);
    nh.getParam("camera_offset1_y", camera_offset1_y);
    nh.getParam("camera_offset2_x", camera_offset2_x);
    nh.getParam("camera_offset2_y", camera_offset2_y);

    origin_put_height = 2;

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
    ROS_INFO("battery!battery!battery!battery!battery!Waiting for start command (press Enter)...");
    for (start_cmd = getchar(); start_cmd != '\n'; start_cmd = getchar());
    ROS_INFO("Launch command received!");
    ros::Time last_time = ros::Time::now();
    ros::Time start_time = ros::Time::now();

    bool camera_offline_flag = false;

    while (ros::ok()) {
        ROS_DEBUG_THROTTLE(1.0, "Current state: %d", ms);  // 限速状态打印

        if(!camera_offline_flag && camera_offline_cnt > 100) {
            camera_offline_flag = true;
            ms = RETURN;
            wp_index = 29;
            start_time = ros::Time::now();
            ROS_ERROR("Camera offline for too long, switching to RETURN state");
        }
        else {
            if(!guidance_exist) camera_offline_cnt++;
            else {
                camera_offline_cnt = 0;
                camera_offline_flag = false;
            }
        }

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
                                ROS_INFO("start_q_eigen: [%.2f, %.2f, %.2f, %.2f]", start_q_eigen.x(), start_q_eigen.y(), start_q_eigen.z(), start_q_eigen.w());
                                Eigen::Vector3d body_forward(1.0, 0.0, 0.0); // 机体系前向向量 (X轴)
                                Eigen::Vector3d world_forward = start_q_eigen * body_forward; // 转换到世界系
                                world_forward.z() = 0.0; // 投影到水平面
                                if (world_forward.norm() > 1e-3) { // 避免除以零
                                    world_forward.normalize();
                                    start_q_eigen = createHorizontalFrame(world_forward);
                                    ROS_INFO("start_q_eigen (after adjustment): [%.2f, %.2f, %.2f, %.2f]", start_q_eigen.x(), start_q_eigen.y(), start_q_eigen.z(), start_q_eigen.w());
                                } else {
                                    ROS_ERROR("机头方向接近垂直");
                                }
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
                wp_index = 21;
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
                         
                if (wp_index > 20) wp_index = 1;
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
            attitude_correction(det_height);

            if(fabs(current_d - tar_d[1])<1.5)
            {
                tar1_cnt++;
                tar2_cnt=0;
                tar3_cnt=0;
                if(tar1_cnt>2){
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
            }
            else if(fabs(current_d - tar_d[2])<1.5)
            {
                tar2_cnt++;
                tar1_cnt=0;
                tar3_cnt=0;
                if(tar2_cnt>2){
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
            }
            else if (fabs(current_d - tar_d[3])<1.5)
            {
                tar3_cnt++;
                tar2_cnt=0;
                tar1_cnt=0;
                if(tar3_cnt>2){
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
            if (local_pos_check_without_z(expect_pos, err_without_z2) &&
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
             if (ros::Time::now() - start_time > ros::Duration(max_wait_time[5])) {
                ROS_WARN("PUT_AIM_1 Timeout");
                ms = PUT_1; // 超时后切换到下一状态
                put_height =1.5;
                aim_height_cnt = 0;
                start_time = ros::Time::now();
            }

            static double min_norm = 1000;
            static int bpos_counter = 0;
            static Eigen::Vector3d filtered_guidance(0, 0, 0);
            const double alpha = 0.3; // 滤波系数，可根据需要调整

            if (guidance_exist) {
                filtered_guidance = alpha * guidance_local + (1 - alpha) * filtered_guidance;
            } else {
                // 当没有新数据时，逐渐减小滤波后的值
                filtered_guidance = (1 - alpha/2) * filtered_guidance;
            }
            ROS_INFO("before LPF ,guidance_local [%.2f, %.2f]", guidance_local(0), guidance_local(1));
            // 使用滤波后的引导向量
            guidance_local = filtered_guidance;
            ROS_INFO("after LPF ,guidance_local [%.2f, %.2f]", guidance_local(0), guidance_local(1));
            guidance_local(0) -= camera_offset1_x;
            guidance_local(1) -= camera_offset1_y;
            guidance_local(2) = 0;
            guidance_global = start_q_eigen * guidance_local;

            double norm = sqrt(guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1));
            if (current_d != 0 && norm < min_norm) {
                min_norm = norm;
                better_pos[current_tar].pose.position.x = local_pos.pose.position.x + guidance_global(0);
                better_pos[current_tar].pose.position.y = local_pos.pose.position.y + guidance_global(1);
                better_pos[current_tar].pose.position.z = put_height + start_pos.pose.position.z;
            }

            if (current_d == 0) {
                aim_height_cnt++;
                if(aim_height_cnt > 20){
                //expect_pos = better_pos[current_tar];
                expect_pos.pose.position.z = origin_put_height + start_pos.pose.position.z;
                //日志
                Eigen::Vector3d better_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                better_pos_body = start_q_eigen.inverse() * better_pos_body;
                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                               local_pos.pose.position.y - start_pos.pose.position.y,
                                               local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                ROS_INFO("PUT_AIM_1\n go to better_pos, guidance_local:[%.2f, %.2f, %.2f]\n NORM: [%.3f]\n MIN_NORM: [%.3f]\n current_d: %.2f", 
                better_pos_body(0)-local_pos_body(0), better_pos_body(1)-local_pos_body(1), better_pos_body(2)-local_pos_body(2), 
                norm, min_norm, current_d);

                go_to_pub();
                }
                // 依靠图像检查是否到达航点
                if (is_stable_arrival(guidance_local, 0.05, 0.7)&&local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                    ms = PUT_1;
                    put_height =1.5;
                    aim_height_cnt = 0;
                    start_time = ros::Time::now();
                }
                break;
            }
            aim_height_cnt = 0;

            double correction_x = guidance_global(0) * ratio1;
            double correction_y = guidance_global(1) * ratio1;
            correction_x = limitCorrection(correction_x, guidance_global(0));
            correction_y = limitCorrection(correction_y, guidance_global(1));
            
            if (is_stable_arrival(guidance_local, height_reduce_norm, 0.2)) {
                put_height = ready_put_height;
                ROS_WARN("reduce height");
            }
            expect_pos.pose.position.x = local_pos.pose.position.x + correction_x;
            expect_pos.pose.position.y = local_pos.pose.position.y + correction_y;
            expect_pos.pose.position.z = put_height + start_pos.pose.position.z;
            
            //日志
            // Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
            //                                 local_pos.pose.position.y - start_pos.pose.position.y,
            //                                 local_pos.pose.position.z - start_pos.pose.position.z);
            // local_pos_body = start_q_eigen.inverse() * local_pos_body;
            // Eigen::Vector3d expect_pos_body(local_pos.pose.position.x + guidance_global(0) - start_pos.pose.position.x,
            //                                 local_pos.pose.position.y + guidance_global(1) - start_pos.pose.position.y,
            //                                 expect_pos.pose.position.z - start_pos.pose.position.z);
            // expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
            // ROS_INFO_THROTTLE(0.5,
            //     "PUT_AIM_1\n guidance_local: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]\n expect_pos_body: [%.2f, %.2f, %.2f]",
            //     guidance_local(0), guidance_local(1), guidance_local(2),
            //     local_pos_body(0), local_pos_body(1), local_pos_body(2),
            //     expect_pos_body(0), expect_pos_body(1), expect_pos_body(2));
            ROS_INFO("PUT_AIM_1\n guidance_local: [%.2f, %.2f, %.2f]\n NORM: [%.3f]", guidance_local(0), guidance_local(1), guidance_local(2), norm);
            // 发布目标位置
            go_to_pub();
            // 依靠图像检查是否到达航点
            if (is_stable_arrival(guidance_local, 0.05, 1)&&local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ms = PUT_1;
                put_height = 1.5;
                aim_height_cnt = 0;
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
            if (local_pos_check_without_z(expect_pos, err_without_z2) &&
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
             if (ros::Time::now() - start_time > ros::Duration(max_wait_time[7])) {
                ROS_WARN("PUT_AIM_2 Timeout");
                ms = PUT_2; // 超时后切换到下一状态
                put_height = origin_put_height;
                start_time = ros::Time::now();
            }

            static double min_norm = 1000;
            static Eigen::Vector3d filtered_guidance(0, 0, 0);
            const double alpha = 0.3; // 滤波系数，可根据需要调整

            if (guidance_exist) {
                filtered_guidance = alpha * guidance_local + (1 - alpha) * filtered_guidance;
            } else {
                // 当没有新数据时，逐渐减小滤波后的值
                filtered_guidance = (1 - alpha/2) * filtered_guidance;
            }
            ROS_INFO("before LPF ,guidance_local [%.2f, %.2f]", guidance_local(0), guidance_local(1));
            // 使用滤波后的引导向量
            guidance_local = filtered_guidance;
            ROS_INFO("after LPF ,guidance_local [%.2f, %.2f]", guidance_local(0), guidance_local(1));
            guidance_local(0) -= camera_offset2_x;
            guidance_local(1) -= camera_offset2_y;
            guidance_local(2) = 0;
            guidance_global = start_q_eigen * guidance_local;

            double norm = sqrt(guidance_local(0) * guidance_local(0) + guidance_local(1) * guidance_local(1));
            if (current_d != 0 && norm < min_norm) {
                min_norm = norm;
                better_pos[current_tar].pose.position.x = local_pos.pose.position.x + guidance_global(0);
                better_pos[current_tar].pose.position.y = local_pos.pose.position.y + guidance_global(1);
                better_pos[current_tar].pose.position.z = put_height + start_pos.pose.position.z;
            }

            if (current_d == 0) {
                aim_height_cnt++;
                if(aim_height_cnt > 20){
                //expect_pos = better_pos[current_tar];
                expect_pos.pose.position.z = origin_put_height + start_pos.pose.position.z;
                //日志
                Eigen::Vector3d better_pos_body(expect_pos.pose.position.x - start_pos.pose.position.x,
                                                expect_pos.pose.position.y - start_pos.pose.position.y,
                                                expect_pos.pose.position.z - start_pos.pose.position.z);
                better_pos_body = start_q_eigen.inverse() * better_pos_body;
                Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
                                               local_pos.pose.position.y - start_pos.pose.position.y,
                                               local_pos.pose.position.z - start_pos.pose.position.z);
                local_pos_body = start_q_eigen.inverse() * local_pos_body;
                ROS_INFO("PUT_AIM_2\n go to better_pos, guidance_local:[%.2f, %.2f, %.2f]\n NORM: [%.3f]\n MIN_NORM: [%.3f]\n current_d: %.2f", 
                better_pos_body(0)-local_pos_body(0), better_pos_body(1)-local_pos_body(1), better_pos_body(2)-local_pos_body(2), 
                norm, min_norm, current_d);

                go_to_pub();
                }
                // 依靠图像检查是否到达航点
                if (is_stable_arrival(guidance_local, 0.05, 1)&&local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                    ms = PUT_2;
                    put_height = origin_put_height;
                    start_time = ros::Time::now();
                }
                break;
            }
            aim_height_cnt = 0;
            
            double correction_x = guidance_global(0) * ratio1;
            double correction_y = guidance_global(1) * ratio1;
            correction_x = limitCorrection(correction_x, guidance_global(0));
            correction_y = limitCorrection(correction_y, guidance_global(1));
            
            if (is_stable_arrival(guidance_local, height_reduce_norm, 0.2)) {
                put_height = ready_put_height;
                ROS_WARN("reduce height");
                
            }
            expect_pos.pose.position.x = local_pos.pose.position.x + correction_x;
            expect_pos.pose.position.y = local_pos.pose.position.y + correction_y;
            expect_pos.pose.position.z = put_height + start_pos.pose.position.z;

            //日志
            // Eigen::Vector3d local_pos_body(local_pos.pose.position.x - start_pos.pose.position.x,
            //                                 local_pos.pose.position.y - start_pos.pose.position.y,
            //                                 local_pos.pose.position.z - start_pos.pose.position.z);
            // local_pos_body = start_q_eigen.inverse() * local_pos_body;
            // Eigen::Vector3d expect_pos_body(local_pos.pose.position.x + guidance_global(0) - start_pos.pose.position.x,
            //                                 local_pos.pose.position.y + guidance_global(1) - start_pos.pose.position.y,
            //                                 expect_pos.pose.position.z - start_pos.pose.position.z);
            // expect_pos_body = start_q_eigen.inverse() * expect_pos_body;
            // ROS_INFO_THROTTLE(0.5,
            //     "PUT_AIM_2\n guidance_local: [%.2f, %.2f, %.2f]\n local_pos_body: [%.2f, %.2f, %.2f]\n expect_pos_body: [%.2f, %.2f, %.2f]",
            //     guidance_local(0), guidance_local(1), guidance_local(2),
            //     local_pos_body(0), local_pos_body(1), local_pos_body(2),
            //     expect_pos_body(0), expect_pos_body(1), expect_pos_body(2));
            ROS_INFO("PUT_AIM_2\n guidance_local: [%.2f, %.2f, %.2f]\n NORM: [%.3f]", guidance_local(0), guidance_local(1), guidance_local(2), norm);
            // 发布目标位置
            go_to_pub();
            // 依靠图像检查是否到达航点
            if (is_stable_arrival(guidance_local, 0.05, 0.7)&&local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ms = PUT_2;
                put_height = origin_put_height;
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
            ms = GO_TO_DET;
            wp_index = 21;
            start_time = ros::Time::now();
            manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, det_height);
            expect_pos_eigen = start_q_eigen * manual_pos_eigen;
            expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
            expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
            expect_pos.pose.position.z = det_height + start_pos.pose.position.z;
            go_to_pub(); // 发布目标位置
            break;
        }

           case GO_TO_DET:
        {
            // 检查是否到达航点
            if (local_pos_check_without_z(expect_pos, err_without_z) &&
                local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff)) {
                ms = DET;
                start_time = ros::Time::now();
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[8])) {
                ROS_ERROR("Timeout: GO_TO_DET");
                ms = DET;
                start_time = ros::Time::now();
            }
            break;
        }

        case DET://盯帧
        {
            if (local_pos_check_without_z(expect_pos, err_without_z) && local_pos_check_only_z(expect_pos.pose.position.z, err_z_takeoff))
            {
                ROS_INFO("DET");
                // 切换到下一个目标点并发布运动指令
                wp_index++;//wp_index:10-17
                if (wp_index > 28) {
                    ms = RETURN;
                    wp_index = 29;
                    start_time = ros::Time::now();
                    break; 
                }
                manual_pos_eigen = Eigen::Vector3d(map[wp_index].x, map[wp_index].y, det_height);
                expect_pos_eigen = start_q_eigen * manual_pos_eigen;
                expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
                expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
                expect_pos.pose.position.z = det_height + start_pos.pose.position.z;
                go_to_pub(); // 发布目标位置
            }
            if (ros::Time::now() - start_time > ros::Duration(max_wait_time[9])) {
                ROS_ERROR("waypoint %d : DET Timeout", wp_index);
                ms = RETURN; // 超时后切换到返回状态
                wp_index = 29; // 确保返回状态时使用正确的航点索引
                start_time = ros::Time::now();
            }
            break;
        }

        case RETURN:
        {
            wp_index = 29;
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

        guidance_exist = false;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Shutting down");
    return 0;
}


/*
waypoint_num: 19 
#position0_x: 31 #投靶区为0—8
#position0_y: 0
#position1_x: 32
#position1_y: 2.5
#position2_x: 33
#position2_y: -2.5
#position3_x: 33.5
#position3_y: 2.5
#position4_x: 34
#position4_y: -2.5
#position5_x: 34
#position5_y: 2.5
#position6_x: 33.5
#position6_y: -2.5
#position7_x: 32.5
#position7_y: 2.5
#position8_x: 31.5
#position8_y: -2.5
#position9_x: 56 #侦察区为9—17
#position9_y: 0
#position10_x: 57
#position10_y: 2
#position11_x: 58
#position11_y: -2
#position12_x: 58.5
#position12_y: 2
#position13_x: 59
#position13_y: -2
#position14_x: 59
#position14_y: 2
#position15_x: 58
#position15_y: -2
#position16_x: 57
#position16_y: 2
#position17_x: 56
#position17_y: -2
#position18_x: 0
#position18_y: 0

position0_x: 31 #投靶区为0—20
position0_y: 0
position1_x: 31
position1_y: 2.5
position2_x: 32
position2_y: 2.5
position3_x: 33
position3_y: 2.5
position4_x: 34
position4_y: 2.5
position5_x: 33
position5_y: 1.25
position6_x: 32
position6_y: 1.25
position7_x: 31
position7_y: 1.25
position8_x: 32
position8_y: 0
position9_x: 33 
position9_y: 0
position10_x: 34
position10_y: 0
position11_x: 33
position11_y: -1.25
position12_x: 32
position12_y: -1.25
position13_x: 31
position13_y: -1.25
position14_x: 32
position14_y: -2.5
position15_x: 33
position15_y: -2.5
position16_x: 34
position16_y: -2.5
position17_x: 32.5
position17_y: -1.5
position18_x: 31.5
position18_y: 1
position19_x: 33.5
position19_y: -1
position20_x: 33
position20_y: 1
position21_x: 46 #侦察区为21—28
position21_y: 2.5
position22_x: 47
position22_y: -2.5
position23_x: 48
position23_y: 2.5
position24_x: 49
position24_y: -2.5
position25_x: 48
position25_y: -2.5
position26_x: 47
position26_y: 2.5
position27_x: 46
position27_y: -2.5
position28_x: 46
position28_y: 0
position29_x: 0
position29_y: 0



global0_lat: 45.7407061
global0_lon: 126.6246417



max_wait_time0: 10 #去投靶区的时间
max_wait_time1: 1 #寻找到两个目标的时间
max_wait_time2: 1 #寻找到一个目标的时间
max_wait_time3: 1 #寻找到零个目标的时间
max_wait_time4: 1 #去第一个目标的时间
max_wait_time5: 2 #瞄准第一个目标的时间
max_wait_time6: 1 #去第二个目标的时间
max_wait_time7: 2 #瞄准第二个目标的时间
max_wait_time8: 2 #去侦察区的时间
max_wait_time9: 2 #离开侦察区的时间
max_wait_time10: 10 #回降落点的时间
max_wait_time11: 5 #瞄准降落的时间

camera_offset1_x: -0.055 #相机偏移量
camera_offset1_y: -0.03
camera_offset2_x: 0.055
camera_offset2_y: 0.03

tar_d1: 25
tar_d2: 20
tar_d3: 15
det_height: 2
put_height: 1.5
ready_put_height: 1.5
height_reduce_norm: 0.40
takeoff_height: 2.5   
err_without_z: 0.15  #不含z方向的误差
err_without_z2: 0.07 #gototar误差
err_z_takeoff: 0.2  #起飞点z方向误差

speed_takeoff: 4
speed_waypoint: 6
speed_search: 2
speed_target_approach: 2
speed_precise_aim: 1
speed_return: 6

ratio1: 0.27

*/