import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_bridge')
    default_params = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')

    params_file = LaunchConfiguration('params_file')
    odom_topic = LaunchConfiguration('odom_topic')
    use_kinematics_filter = LaunchConfiguration('use_kinematics_filter')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to go2_bridge parameter YAML file'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/utlidar/robot_odom',
            description='Odometry topic to bridge to TF'),
        DeclareLaunchArgument(
            'use_kinematics_filter',
            default_value='true',
            description='Enable forward-kinematics + gyro complementary filter for base_stab'),

        # 1. Odometry TF Bridge Node (odom -> base_link)
        Node(
            package='go2_bridge',
            executable='odom_tf_node',
            name='go2_odom_tf',
            output='screen',
            parameters=[params_file, {'odom_topic': odom_topic}]
        ),

        # 2. IMU & Gravity/Hill-Stabilized Frame Node (base_link -> base_stab, static transforms)
        Node(
            package='go2_bridge',
            executable='imu_bridge_node',
            name='go2_imu_bridge',
            output='screen',
            parameters=[params_file, {'use_kinematics_filter': use_kinematics_filter}]
        ),

        # 3. 3D PointCloud & Hill Terrain-Aware Scan Bridge (/lidar_points -> /velodyne_points & /scan)
        Node(
            package='go2_bridge',
            executable='cloud_bridge_node',
            name='go2_cloud_bridge',
            output='screen',
            parameters=[params_file]
        ),

        # 4. CmdVel to Go2 Sport Motor Bridge
        Node(
            package='go2_bridge',
            executable='cmd_vel_to_sport_node',
            name='go2_cmd_vel_to_sport',
            output='screen',
            parameters=[params_file]
        ),
    ])
