"""Go2 full stack in one launch, brought up in order by one lifecycle manager.

mode:=nav     bridges + FAST-LIO + map_server/AMCL + EGO-Planner (+ motor bridge)
mode:=patrol  bridges + RANSAC wall runner (+ motor bridge)   -- fallback when SLAM/AMCL fail
motor:=false  never starts cmd_vel_to_sport: nothing can move the robot
route:=<file> + route_runner_node (saved points -> EGO goals, stop line / parking); idle until
              START on /route/cmd (scripts/go2_route.py start)

Frames (nav): map -AMCL-> odom -leg odom-> base_link -imu_bridge-> base_stab
FAST-LIO keeps its own camera_init tree on /tf_lio; EGO-Planner reads its /Odometry and maps
goals given in "map" through AMCL (see ego_planner_params.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def share(pkg, *p):
    return os.path.join(get_package_share_directory(pkg), *p)


def stack(context):
    mode = LaunchConfiguration('mode').perform(context)
    motor = LaunchConfiguration('motor').perform(context).lower() == 'true'
    map_yaml = LaunchConfiguration('map').perform(context)
    fastlio_cfg = LaunchConfiguration('fastlio_config').perform(context)
    bridge_params = share('go2_bridge', 'config', 'bridge_params.yaml')

    def lc(pkg, exe, name, params=(), remaps=()):
        return LifecycleNode(package=pkg, executable=exe, name=name, namespace='',
                             output='screen', parameters=list(params), remappings=list(remaps))

    nodes, managed = [], []

    nodes.append(lc('go2_bridge', 'imu_bridge_node', 'go2_imu_bridge', [bridge_params]))
    managed.append('go2_imu_bridge')
    nodes.append(lc('go2_bridge', 'cloud_bridge_node', 'go2_cloud_bridge', [bridge_params]))
    managed.append('go2_cloud_bridge')

    if mode == 'nav':
        if not map_yaml:
            raise RuntimeError('mode:=nav needs map:=<map.yaml>')
        nodes.append(lc('go2_bridge', 'odom_tf_node', 'go2_odom_tf', [bridge_params]))
        managed.append('go2_odom_tf')
        # FAST-LIO is not a lifecycle node; its TF goes to /tf_lio so camera_init->body does not
        # fight base_link->body from imu_bridge.
        nodes.append(Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_odometry',
                          output='screen', parameters=[fastlio_cfg],
                          remappings=[('/tf', '/tf_lio'), ('/tf_static', '/tf_static_lio')]))
        nodes.append(lc('nav2_map_server', 'map_server', 'map_server',
                        [{'yaml_filename': map_yaml, 'frame_id': 'map'}]))
        nodes.append(lc('nav2_amcl', 'amcl', 'amcl',
                        [share('go2_localization', 'config', 'amcl_params.yaml')]))
        managed += ['map_server', 'amcl']
        nodes.append(lc('ego_planner', 'ego_planner_node', 'ego_planner_node',
                        [share('ego_planner', 'config', 'ego_planner_params.yaml')]))
        managed.append('ego_planner_node')
        if LaunchConfiguration('cones').perform(context).lower() == 'true':
            # Cone tracker (YOLO runs on the robot PC and publishes /mission/cone_detections)
            nodes.append(lc('mission_vision', 'cone_tracker_node', 'cone_tracker_node',
                            [share('mission_vision', 'config', 'cone_tracker_params.yaml'),
                             share('mission_vision', 'config', 'camera_extrinsics.yaml')]))
            managed.append('cone_tracker_node')
        route = LaunchConfiguration('route').perform(context)
        if route:
            if not os.path.isfile(route):
                raise RuntimeError(f'route file not found: {route}')
            nodes.append(lc('mission_runner', 'route_runner_node', 'route_runner_node', [route]))
            managed.append('route_runner_node')
    elif mode == 'patrol':
        nodes.append(lc('go2_wall_patrol', 'wall_runner_node', 'wall_runner_node',
                        [share('go2_wall_patrol', 'config', 'patrol_params.yaml')]))
        managed.append('wall_runner_node')
        if motor:
            # Corner SIT/STAND postures (StandDown 1005 -> StandUp 1004 -> BalanceStand 1002)
            nodes.append(Node(package='go2_wall_patrol', executable='go2_sport_bridge.py',
                              name='go2_sport_bridge', output='screen'))
    else:
        raise RuntimeError(f'unknown mode {mode} (nav | patrol)')

    if motor:
        # Last in the list: the motor bridge only becomes active after everything else did.
        nodes.append(lc('go2_bridge', 'cmd_vel_to_sport_node', 'go2_cmd_vel_to_sport', [bridge_params]))
        managed.append('go2_cmd_vel_to_sport')

    # bond_timeout 0: our nodes are plain rclcpp_lifecycle nodes without nav2 bonds
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_go2', output='screen',
                      parameters=[{'autostart': True, 'node_names': managed, 'bond_timeout': 0.0}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav', description='nav | patrol'),
        DeclareLaunchArgument('motor', default_value='true', description='false: no cmd_vel_to_sport'),
        DeclareLaunchArgument('map', default_value='', description='map yaml (mode:=nav)'),
        DeclareLaunchArgument('cones', default_value='false', description='true: cone_tracker_node'),
        DeclareLaunchArgument('route', default_value='', description='route file: route_runner_node (mode:=nav)'),
        DeclareLaunchArgument('fastlio_config', default_value='',
                              description='FAST-LIO parameter file (mode:=nav)'),
        OpaqueFunction(function=stack),
    ])
