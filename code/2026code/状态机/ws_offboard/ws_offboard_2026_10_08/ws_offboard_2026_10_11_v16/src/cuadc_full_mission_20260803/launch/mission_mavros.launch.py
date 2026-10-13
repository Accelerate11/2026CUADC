"""One MAVROS with unchanged APM defaults plus mission clock settings."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros = get_package_share_directory('mavros')
    mission = get_package_share_directory('cuadc_full_mission_20260803')
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('namespace', default_value='mavros'),
        Node(package='mavros', executable='mavros_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[
                 os.path.join(mavros, 'launch', 'apm_pluginlists.yaml'),
                 os.path.join(mavros, 'launch', 'apm_config.yaml'),
                 os.path.join(mission, 'config', 'mavros_time_sync.yaml'),
                 {'fcu_url': LaunchConfiguration('fcu_url'),
                  'gcs_url': LaunchConfiguration('gcs_url'),
                  'tgt_system': 1, 'tgt_component': 1,
                  'fcu_protocol': 'v2.0'}]),
    ])
