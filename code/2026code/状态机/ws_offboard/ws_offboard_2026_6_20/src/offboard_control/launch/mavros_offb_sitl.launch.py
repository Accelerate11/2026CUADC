from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://:14550@',
            description='SITL MAVLink endpoint. ArduPilot SITL usually uses udp://:14550@'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{'fcu_url': fcu_url}],
        ),

        Node(
            package='offboard_control',
            executable='offb_node',
            name='offb_node',
            output='screen',
        ),
    ])
