"""Go2 full stack in one launch, brought up in order by one lifecycle manager.

mode:=nav     bridges + FAST-LIO + map_server/AMCL + EGO-Planner (+ motor bridge)
mode:=patrol  bridges + RANSAC wall runner (+ motor bridge)   -- fallback when SLAM/AMCL fail
motor:=false  never starts cmd_vel_to_sport: nothing can move the robot
cones:=true   + cone_tracker_node (LiDAR only); cone_regions:="x0 y0 x1 y1 ..."

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
        # AMCL odometry = FAST-LIO (/Odometry), not the leg odometry: 11-05 replays (3 arena bags) AMCL median
        # 9-14 cm -> 3-3.6 cm, scan match 92-96 -> 97-99 %. EGO already drives on FAST-LIO, so leg-odom AMCL
        # bought no real independence. bridge_params keeps the
        # leg odometry for launches without FAST-LIO (bridge.launch / bringup.launch).
        nodes.append(lc('go2_bridge', 'odom_tf_node', 'go2_odom_tf',
                        [bridge_params, {'odom_topic': LaunchConfiguration('amcl_odom').perform(context)}]))
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
        ego_over = {}
        plan_map = LaunchConfiguration('plan_map').perform(context)
        if plan_map:
            if not os.path.isfile(plan_map):
                raise RuntimeError(f'plan_map not found: {plan_map}')
            # closed-wall planning layer (scripts/go2_plan_map.py), AMCL map frame
            ego_over = {'use_prior_map': True, 'static_map_path': plan_map, 'prior_map.frame': 'map'}
        nodes.append(lc('ego_planner', 'ego_planner_node', 'ego_planner_node',
                        [share('ego_planner', 'config', 'ego_planner_params.yaml'), ego_over]))
        managed.append('ego_planner_node')
        if LaunchConfiguration('cones').perform(context).lower() == 'true':
            # Cone tracker (LiDAR only)
            over = {}
            reg = LaunchConfiguration('cone_regions').perform(context).replace(',', ' ').split()
            if reg:
                if len(reg) % 4:
                    raise RuntimeError('cone_regions needs x0 y0 x1 y1 per rectangle')
                over['lidar_only.regions'] = [float(v) for v in reg]
            nodes.append(lc('mission_vision', 'cone_tracker_node', 'cone_tracker_node',
                            [share('mission_vision', 'config', 'cone_tracker_params.yaml'),
                             over]))
            managed.append('cone_tracker_node')
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
        DeclareLaunchArgument('plan_map', default_value='',
                              description='closed-wall planning layer .bin (map frame) for EGO-Planner, e.g. maps/arena1459_plan.bin'),
        DeclareLaunchArgument('amcl_odom', default_value='/Odometry',
                              description='odometry under AMCL: /Odometry (FAST-LIO, default) | /utlidar/robot_odom (legs)'),
        DeclareLaunchArgument('cone_regions', default_value='',
                              description='LiDAR-only cone rectangles "x0 y0 x1 y1 ..." (map frame)'),
        DeclareLaunchArgument('fastlio_config', default_value='',
                              description='FAST-LIO parameter file (mode:=nav)'),
        OpaqueFunction(function=stack),
    ])
