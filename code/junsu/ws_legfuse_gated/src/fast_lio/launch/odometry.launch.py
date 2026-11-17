import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio')
    default_config = os.path.join(pkg_share, 'config', 'go2_xt16_odometry.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_odometry',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])

