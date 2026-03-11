/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
/*代码功能，在code1.3的基础上，将发点改为经纬度坐标，借以验证
是否仿真验证：是
是否实飞验证：是
实飞现象
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geographic_msgs/GeoPoseStamped.h>

enum mission_state
{
    READY_TO_FLY,
    TAKE_OFF,
    GO_TO_POINT,
    RETURN,
    LAND,
    DONE
};

struct position {
    double lat;
    double lon;
};

int waypoint_num;
double err_without_z, err_z_takeoff;
int wp_index = 0;

// ����״̬��ʼ��
enum mission_state ms = READY_TO_FLY;

// ��ȡ�ɿ�״̬
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// ��ȡ��ǰλ��
sensor_msgs::NavSatFix global_pos;
void global_pos_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_pos = *msg;
    return;
}

// ��¼��ɵ�λ��
sensor_msgs::NavSatFix start_global_pos;

// λ���жϺ���
// �ж�ˮƽ������ʹ�� Haversine ��ʽ������룩
bool global_pos_check_without_z(double target_lat, double target_lon, double error) {
    double dlat = target_lat - global_pos.latitude;
    double dlon = target_lon - global_pos.longitude;
    if(fabs(dlat)<0.0000005&&fabs(dlon)<0.0000005) return true;
    else return false;
}

// �жϸ߶���ʹ����Ը߶ȣ�
bool altitude_check(double expect_height, double error) {
    // ���������Ѿ�֪����ɵ�߶ȣ�ʹ����Ը߶�
    double current_alt = global_pos.altitude - start_global_pos.altitude;
    ROS_DEBUG("Altitude error: %.2fm", fabs(expect_height - current_alt));
    return fabs(expect_height - current_alt) < error;
}

ros::Publisher expect_global_pos_pub;
mavros_msgs::GlobalPositionTarget expect_global_pos;

void go_to_global_pub(double lat, double lon, double alt) {
    expect_global_pos.header.stamp = ros::Time::now();
    expect_global_pos.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    expect_global_pos.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    
    expect_global_pos.latitude = lat;
    expect_global_pos.longitude = lon;
    expect_global_pos.altitude = alt; // ʹ����Ը߶�
    
    expect_global_pos_pub.publish(expect_global_pos);
    ROS_DEBUG("Publishing target position: [%.6f, %.6f, %.2f]", lat, lon, alt);
}

double takeoff_height = 2;

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");
    
    // ��ȡ����
    nh.getParam("waypoint_num", waypoint_num);
    waypoint_num = 4;
    position map[4];
    nh.getParam("position0_lat", map[0].lat);
    nh.getParam("position0_lon", map[0].lon);
    nh.getParam("position1_lat", map[1].lat);
    nh.getParam("position1_lon", map[1].lon);
    nh.getParam("position2_lat", map[2].lat);
    nh.getParam("position2_lon", map[2].lon);
    nh.getParam("position3_lat", map[3].lat);
    nh.getParam("position3_lon", map[3].lon);

    nh.getParam("err_without_z", err_without_z);
    nh.getParam("err_z_takeoff", err_z_takeoff);
    
    // ����ȫ��λ����Ϣ
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, global_pos_callback);
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, state_cb);
    
    expect_global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);

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
        if (++connection_counter % 50 == 0) {  // ÿ2.5����ʾһ��
            ROS_WARN_STREAM_THROTTLE(2.5, "Waiting for FCU connection...");
        }
    }
    ROS_INFO("FCU connected!");

    // ��ʼ��Ŀ��λ��
    expect_global_pos.latitude = 45.7406205;
    expect_global_pos.longitude = 126.6245719;
    expect_global_pos.altitude = 2;
    expect_global_pos.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    expect_global_pos.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                                  mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    // �л��� GUIDED ģʽ
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
        ROS_DEBUG_THROTTLE(1.0, "Current state: %d", ms);  // ����״̬��ӡ

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
                                start_global_pos = global_pos; // ��¼��ɵ�λ��
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
                            expect_global_pos.altitude = start_global_pos.altitude + takeoff_height;
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
                // ����Ŀ��λ�ã���γ�ߣ�
                go_to_global_pub(map[wp_index].lat, map[wp_index].lon, takeoff_height);
                ROS_WARN("GO_TO_POINT,%f,%f,%f",expect_global_pos.latitude, expect_global_pos.longitude, expect_global_pos.altitude);
                // ����Ƿ񵽴ﺽ��
                if (global_pos_check_without_z(map[wp_index].lat, map[wp_index].lon, err_without_z) && 
                    altitude_check(takeoff_height, err_z_takeoff)) {
                    ROS_INFO("Navigating to waypoint %d/%d [%.6f, %.6f, %.1f]", 
                        wp_index+1, waypoint_num, map[wp_index].lat, map[wp_index].lon, takeoff_height);
                    ROS_INFO("Reached waypoint %d/%d", wp_index+1, waypoint_num);
                    wp_index++;
                }
            } 
            // ���к������
            else {
                ROS_INFO("All waypoints completed");
                ms = RETURN;
            }
            break;
        }

        case RETURN:
        
                // ����Ŀ��λ�ã���γ�ߣ�
                go_to_global_pub(start_global_pos.latitude,start_global_pos.longitude, takeoff_height);
                ROS_INFO("RETURN HOME %f ,%f, %f",expect_global_pos.latitude, expect_global_pos.longitude, expect_global_pos.altitude );
                
                // ����Ƿ񵽴ﺽ��
                if (global_pos_check_without_z(start_global_pos.latitude, start_global_pos.longitude, err_without_z) && 
                    altitude_check(takeoff_height, err_z_takeoff)) {

                        ms = LAND;
            } 
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

        // ��������Ŀ��λ��
        expect_global_pos_pub.publish(expect_global_pos);

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Shutting down");
    return 0;
}