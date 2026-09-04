from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cuadc_hazard_recognition_sim")
    world = os.path.join(pkg_share, "worlds", "hazard_recognition_single.sdf")
    bridge_config = os.path.join(pkg_share, "config", "camera_bridge.yaml")
    model_path = os.path.join(pkg_share, "models")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_bridge",
            default_value="false",
            description="Start ros_gz_bridge for camera topics when installed.",
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                model_path,
                ":",
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-v4", "-r", world],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
            condition=IfCondition(LaunchConfiguration("use_bridge")),
            output="screen",
        ),
    ])
