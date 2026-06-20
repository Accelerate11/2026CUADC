from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control',
            executable='offb_node',
            name='offb_node',
            output='screen',
        ),
    ])
