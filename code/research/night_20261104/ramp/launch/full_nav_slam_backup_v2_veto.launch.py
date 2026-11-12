# night_20261104 — 백업 v2 런치(backup_v2/launch/full_nav_slam_backup_v2.launch.py)를 그대로 포함하고
#   ① 플래너 설정만 ego_planner_veto.yaml(cloud_topic=/cloud_planner) 로 바꾸고
#   ② terrain_veto 노드(/cloud_active + /Odometry_mux + L1 + D435i 깊이 → /cloud_planner)를 더한다.
# veto:=false 면 terrain_veto 를 지우지 않는 통과 모드(max_step 0 → 통과면 없음)로 띄워 비교용으로 쓴다.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

N = os.path.expanduser('~/ws/night_20261104')


def generate_launch_description():
    veto = LaunchConfiguration('veto')
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(N + '/backup_v2/launch/full_nav_slam_backup_v2.launch.py'),
        launch_arguments={'planner_config': N + '/ramp/config/ego_planner_veto.yaml'}.items())
    common = {'cloud_topic': '/cloud_active', 'odom_topic': '/Odometry_mux', 'out_topic': '/cloud_planner',
              'depth_topic': LaunchConfiguration('depth_topic'), 'l1_topic': LaunchConfiguration('l1_topic')}
    return LaunchDescription([
        DeclareLaunchArgument('veto', default_value='true'),
        DeclareLaunchArgument('depth_topic', default_value=''),   # 11-04: XT16+L1 이 가장 좋음(4.3%). 깊이를 격자에 더하면 5.3% — 콘 녹화로 다시 볼 때만 켤 것
        DeclareLaunchArgument('l1_topic', default_value='/utlidar/cloud_base'),
        base,
        Node(package='terrain_veto', executable='terrain_veto_node', name='terrain_veto', output='screen',
             parameters=[common], condition=IfCondition(veto)),
        Node(package='terrain_veto', executable='terrain_veto_node', name='terrain_veto', output='screen',
             parameters=[dict(common, max_step=0.0, seed_tol=-1.0)], condition=UnlessCondition(veto)),
    ])
