import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_localization')

    # Dynamically resolve workspace maps directory without hardcoded username/paths
    ws_root = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    default_map = os.path.join(ws_root, 'maps', 'map_default.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.expanduser('~/go2_nav_ws/maps/map_default.yaml')

    default_amcl_params = os.path.join(pkg_dir, 'config', 'amcl_params.yaml')
    default_p2l_params = os.path.join(pkg_dir, 'config', 'p2l_params.yaml')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_params = LaunchConfiguration('amcl_params')
    p2l_params = LaunchConfiguration('p2l_params')
    cloud_topic = LaunchConfiguration('cloud_topic')
    use_external_scan = LaunchConfiguration('use_external_scan')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to 2D map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        DeclareLaunchArgument('amcl_params', default_value=default_amcl_params, description='AMCL parameter file'),
        DeclareLaunchArgument('p2l_params', default_value=default_p2l_params, description='PointCloud to LaserScan param file'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points', description='Input 3D point cloud topic'),
        DeclareLaunchArgument('use_external_scan', default_value='true', description='If true, cloud_bridge_node provides hill-filtered /scan'),

        # 1. 2D Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_file,
                         'topic_name': 'map',
                         'frame_id': 'map'}]
        ),

        # 2. AMCL Global Localization Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params, {'use_sim_time': use_sim_time}],
            remappings=[('scan', '/scan')]
        ),

        # 3. Lifecycle Manager for Map Server and AMCL
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['map_server', 'amcl']}]
        ),

        # 4. Fallback PointCloud to LaserScan Node (only launched if use_external_scan is false)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[p2l_params, {'use_sim_time': use_sim_time}],
            remappings=[('cloud_in', cloud_topic), ('scan', '/scan')],
            condition=UnlessCondition(use_external_scan)
        )
    ])
