from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial/by-id/usb-ArduPilot_CUAVv5Nano_410035001451333035363236-if00:57600',
            description='CUAV v5 Nano USB MAVLink serial port'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{'fcu_url': fcu_url}],
        ),
    ])
