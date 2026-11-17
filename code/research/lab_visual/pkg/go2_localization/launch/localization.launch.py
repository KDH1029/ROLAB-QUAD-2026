"""고정 사전 지도에 대고 위치추정만 돌린다. 지도는 절대 바뀌지 않는다.

  ros2 launch go2_localization localization.launch.py
  ros2 launch go2_localization localization.launch.py map:=/경로/지도.pcd pose:=1.2,-4.8,8

【무엇을 띄우나】
  icp_localization      스캔 -> 지도 정합 -> TF map->camera_init
  odometry_map_adapter  그 TF + /Odometry -> /Odometry_map  (계획기가 받는 모양)
  odometry_monitor      발산 감시 (구독만, odometry/health 만 발행)
  map_publisher         지도를 map/points 로 (화면 확인용)

【mapping 과 무엇이 다른가】
  mapping 은 지도를 계속 고쳐 쓴다. 그래서 틀어지면 지도까지 같이 망가진다.
  여기서는 지도가 **읽기 전용**이다. 바뀌는 것은 map->odom 보정 하나뿐이고,
  그게 틀어져도 지도는 그대로라 다시 잡을 수 있다.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_map = os.path.expanduser('~/gosu/lab_visual/maps/go2_prior_map_v05.pcd')
    m = LaunchConfiguration('map')
    odom_frame = LaunchConfiguration('odom_frame')
    hz = LaunchConfiguration('hz')
    pts = LaunchConfiguration('scan_pts')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='사전 지도 PCD. 읽기만 한다'),
        DeclareLaunchArgument('odom_frame', default_value='camera_init',
                              description='FAST-LIO 의 고정 프레임. odom 이 아니다'),
        DeclareLaunchArgument('hz', default_value='5.0'),
        DeclareLaunchArgument('scan_pts', default_value='800'),
        Node(package='go2_localization', executable='icp_localization',
             name='icp_localization', output='screen',
             arguments=['--map', m, '--odom-frame', odom_frame,
                        '--hz', hz, '--scan-pts', pts, '--wide']),
        Node(package='go2_localization', executable='odometry_map_adapter',
             name='odometry_map_adapter', output='screen',
             arguments=['--odom-frame', odom_frame]),
        Node(package='go2_localization', executable='odometry_monitor',
             name='odometry_monitor', output='screen',
             arguments=['--report-every', '10']),
        Node(package='go2_localization', executable='map_publisher',
             name='map_publisher', output='screen',
             arguments=['--map', m, '--voxel', '0.12', '--band']),
    ])
