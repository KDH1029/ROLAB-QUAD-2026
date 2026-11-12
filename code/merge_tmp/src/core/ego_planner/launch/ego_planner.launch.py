import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ego_planner')
    default_config = os.path.join(pkg_share, 'config', 'ego_planner_params.yaml')

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),

        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            parameters=[config_file],
            output='screen'
        )
    ])

