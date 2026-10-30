import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():
    nav_pkg = get_package_share_directory('go2_navigation')
    loc_pkg = get_package_share_directory('go2_localization')
    bridge_pkg = get_package_share_directory('go2_bridge')

    # Dynamically resolve workspace maps directory without hardcoded username/paths
    ws_root = os.path.abspath(os.path.join(nav_pkg, '..', '..', '..', '..'))
    default_map = os.path.join(ws_root, 'maps', 'map_default.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.expanduser('~/go2_nav_ws/maps/map_default.yaml')

    default_nav_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_bridge_params = os.path.join(bridge_pkg, 'config', 'bridge_params.yaml')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_mode = LaunchConfiguration('sim_mode')
    nav_params = LaunchConfiguration('nav_params')
    bridge_params = LaunchConfiguration('bridge_params')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to 2D map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('sim_mode', default_value='false', description='Simulation/Mock mode without physical Go2 robot'),
        DeclareLaunchArgument('nav_params', default_value=default_nav_params, description='Nav2 parameter file'),
        DeclareLaunchArgument('bridge_params', default_value=default_bridge_params, description='go2_bridge parameter file'),
        DeclareLaunchArgument('odom_topic', default_value='/utlidar/robot_odom', description='Odometry topic to bridge to TF'),

        # 1. Hardware Bridge Subsystem (odom_tf + imu_bridge with FK + cmd_vel_to_sport)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_pkg, 'launch', 'bridge.launch.py')
            ),
            launch_arguments={
                'params_file': bridge_params,
                'odom_topic': odom_topic
            }.items(),
            condition=UnlessCondition(sim_mode)
        ),

        # 2. Localization Subsystem (map_server + amcl + pointcloud_to_laserscan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_pkg, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # 3. Nav2 Path Planning Subsystem (controller, planner, behaviors, bt_navigator)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'params': nav_params,
                'use_sim_time': use_sim_time
            }.items()
        )
    ])
