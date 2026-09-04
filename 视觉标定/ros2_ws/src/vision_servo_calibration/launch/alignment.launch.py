from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    start_template = LaunchConfiguration("start_template")
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("start_template", default_value="false"),
        Node(
            package="vision_servo_calibration",
            executable="vision_provider_template",
            name="vision_provider_template",
            parameters=[params_file],
            condition=IfCondition(start_template),
            output="screen",
        ),
        Node(
            package="vision_servo_calibration",
            executable="alignment_viewer",
            name="alignment_viewer",
            parameters=[params_file],
            output="screen",
        ),
    ])
