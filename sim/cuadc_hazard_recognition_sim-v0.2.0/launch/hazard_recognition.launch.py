from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cuadc_hazard_recognition_sim")
    world = os.path.join(pkg_share, "worlds", "hazard_recognition_single.sdf")
    generated_scene = os.path.join(pkg_share, "config", "generated_scene.yaml")
    model_path = os.path.join(pkg_share, "models")
    show_gazebo_gui = LaunchConfiguration("show_gazebo_gui")
    return LaunchDescription([
        DeclareLaunchArgument("use_bridge", default_value="true"),
        DeclareLaunchArgument("start_state_machine", default_value="false"),
        DeclareLaunchArgument("start_vision", default_value="true"),
        DeclareLaunchArgument("show_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("show_vision_window", default_value="true"),
        DeclareLaunchArgument("generated_scene_path", default_value=generated_scene),
        DeclareLaunchArgument("coordinate_source", default_value="auto_rtk"),
        DeclareLaunchArgument(
            "vision_model",
            default_value=EnvironmentVariable("CUADC_HAZARD_MODEL", default_value=""),
        ),
        DeclareLaunchArgument("gz_image_topic", default_value="/hazard_d435i/image"),
        DeclareLaunchArgument(
            "vision_image_topic", default_value="/uav/d435i/color/image"
        ),
        DeclareLaunchArgument(
            "result_path",
            default_value=EnvironmentVariable(
                "CUADC_RESULT_PATH",
                default_value=os.path.expanduser("~/cuadc_outputs/hazard_search_observations.json"),
            ),
        ),
        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
        SetEnvironmentVariable(name="QT_XCB_GL_INTEGRATION", value="xcb_glx"),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                model_path,
                ":",
                EnvironmentVariable("ARDUPILOT_GAZEBO_DIR", default_value=os.path.expanduser("~/ardupilot_gazebo")),
                "/models:",
                EnvironmentVariable("ARDUPILOT_GAZEBO_DIR", default_value=os.path.expanduser("~/ardupilot_gazebo")),
                "/worlds:",
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=[
                EnvironmentVariable("ARDUPILOT_GAZEBO_DIR", default_value=os.path.expanduser("~/ardupilot_gazebo")),
                "/build:",
                EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            ],
        ),
        # Keep the simulation server independent from the desktop client.
        # A GUI/OpenGL failure must never take down SITL, sensors, or flight control.
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-v4", "-r", world],
            output="screen",
        ),
        TimerAction(
            period=2.0,
            condition=IfCondition(show_gazebo_gui),
            actions=[
                LogInfo(
                    msg=(
                        "Gazebo GUI requested: connecting an independent client "
                        "to the randomized recognition-area server"
                    ),
                ),
                ExecuteProcess(
                    cmd=["gz", "sim", "-g", "-v4"],
                    output="screen",
                ),
            ],
        ),
        Node(
            package="cuadc_hazard_recognition_sim",
            executable="recon_state_machine_cpp",
            parameters=[{
                "generated_scene_path": LaunchConfiguration("generated_scene_path"),
                "coordinate_source": LaunchConfiguration("coordinate_source"),
                "result_path": LaunchConfiguration("result_path"),
                "takeoff_alt": 1.5,
                "trajectory_speed": 0.65,
                "waypoint_accept_radius": 0.30,
                "coverage_lane_spacing_m": 0.70,
                "coverage_edge_inset_x_m": 0.40,
                "coverage_edge_inset_y_m": 0.65,
                "detection_pause_s": 3.0,
                "detection_centering_timeout_s": 5.0,
                "visual_min_confidence": 0.25,
                "visual_min_consecutive": 3,
                "visual_weak_confidence": 0.05,
                "visual_weak_min_consecutive": 8,
                "visual_center_gate_fraction": 0.90,
                "observation_merge_radius_m": 0.65,
                "observation_same_class_merge_radius_m": 0.85,
                "camera_footprint_x_m": 1.02,
                "camera_footprint_y_m": 1.80,
                "camera_image_yaw_deg": 0.0,
            }],
            condition=IfCondition(LaunchConfiguration("start_state_machine")),
            output="screen",
        ),
        Node(
            package="cuadc_hazard_recognition_sim",
            executable="static_camera_roi_node.py",
            parameters=[{
                "generated_scene_path": LaunchConfiguration("generated_scene_path"),
                "gz_topic": LaunchConfiguration("gz_image_topic"),
                "ros_topic": LaunchConfiguration("vision_image_topic"),
            }],
            condition=IfCondition(LaunchConfiguration("use_bridge")),
            output="screen",
        ),
        Node(
            package="cuadc_hazard_recognition_sim",
            executable="hazard_vision_node.py",
            parameters=[{
                "model_path": LaunchConfiguration("vision_model"),
                "image_topic": LaunchConfiguration("vision_image_topic"),
                "output_topic": "/perception/hazard_detection",
                "confidence": 0.05,
                "min_consecutive_confirm": 3,
                "show_window": ParameterValue(
                    LaunchConfiguration("show_vision_window"), value_type=bool
                ),
                "publish_annotated": True,
                "annotated_topic": "/perception/hazard_annotated",
            }],
            condition=IfCondition(LaunchConfiguration("start_vision")),
            output="screen",
        ),
    ])
