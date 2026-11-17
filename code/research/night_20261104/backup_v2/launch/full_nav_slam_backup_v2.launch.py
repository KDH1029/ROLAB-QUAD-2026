# v2 사본 — 원본 ~/ws/v_gosu/backup/launch/full_nav_slam_backup.launch.py 와 다른 곳은 백업 노드(v2 + 야간 venv 파이썬) 한 줄뿐
# 11-04 07:40: 전환 장치 두 줄(전환 / --no-switch)도 v2 사본 odom_mux_node_v2.py 로 — 백업 health 가 ok 일 때만 전환, 종료 Traceback 잡음
"""통째 SLAM 주행 + 백업 위치추정 (v_gosu 사본, 원본 ~/ws/gosu_slam/launch/full_nav_slam.launch.py 는 그대로).

【원본과 다른 점 — 이것만】
  + backup_loc_node.py : 다리 오도메트리 + 3D 지도 정합(수평 세우기) → /backup/odometry, /backup/cloud
  + odom_mux_node.py   : FAST-LIO 이상(0.5초 무소식 / 2 m/s 초과 / 백업과 0.5 m·1초) 시 플래너 입력을 백업으로 전환
  · odom_filter 입력   /Odometry → /Odometry_mux    · odom_to_tf 입력  /Odometry → /Odometry_mux
  · 플래너 점구름     /cloud_registered → /cloud_active  (config/ego_planner_backup.yaml)
  인자 backup_map (사전 지도 .npz/.pcd, 출발 자세 = 원점으로 만든 것), backup:=false 면 원본과 같은 구성

통째 SLAM 주행 — 사전 지도 없음, AMCL 없음.

팀 공용 full_nav_live.launch.py 에서 **AMCL 위치추정 층만 뺀 것**이다.
팀 파일은 건드리지 않았다. 원본은 그대로 살아있다.

【왜 빼는가 — 전부 실측】
  FAST-LIO 프레임 표류   20초 주행 중앙 1.4 mm, p90 11.3 mm, 최악 44.6 mm
  ego_planner goal_tol   250 mm         -> 표류는 허용치의 0.6%
  ego_planner 는 이미 사전 지도를 안 쓴다 (use_prior_map: false, map_frame: camera_init)
  AMCL 이 플래너에 닿는 통로는 goal_relay 하나뿐이었다.
  전역 재정위는 직사각 경기장이라 원리적으로 불가 (1등-2등 일치율 격차 0~2.2%p).

【뺀 것】 map_server, AMCL, pointcloud_to_laserscan(3D->2D 버림),
         nav2 lifecycle_manager, goal_relay
【남긴 것】 odom_to_tf — AMCL 층에 있었지만 위치추정이 아니라 TF 사슬(odom->base_link)
           담당이라 필수다.

【목표 지정 — 반드시 읽을 것】
  camera_init(= 출발 자세) 기준으로 /goal_pose 에 직접 준다.
  대회는 지정 코너에서 출발하므로 코너 좌표가 출발점 기준 상수가 된다.
  예전 map 좌표 웨이포인트를 그대로 넣으면 엉뚱한 곳으로 간다.
  ★ 주행 전 무모터(motor:=false, 기본)로 목표 좌표계를 반드시 확인할 것.

인자
  lidar      true 면 hesai 드라이버를 띄운다(실기). 재생 검증 때는 false.  기본 true
  motor      true 면 /cmd_vel 이 로봇으로 나간다.  기본 false (안전)
  deskew     true 면 liosam 디스큐 브리지, false 면 팀 브리지.  기본 false
             ※ true 로 켤 때는 imu_bridge 의 publish_stab 도 꺼야 한다(같은 TF 충돌)
  foxglove   기본 true (포트 8765)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    odom_cfg = os.path.expanduser(
        '~/ws/gosu_slam/config/go2_slam_odometry.yaml')
    planner_default = os.path.expanduser(
        '~/ws/v_gosu/backup/config/ego_planner_backup.yaml')
    bdir = os.path.expanduser('~/ws/v_gosu/backup')
    v2dir = os.path.expanduser('~/ws/night_20261104/backup_v2')   # v2: GICP + 견고 지면표 (전환 장치·플래너 설정은 v_gosu 그대로)
    backup = LaunchConfiguration('backup')
    lidar = LaunchConfiguration('lidar')
    motor = LaunchConfiguration('motor')
    deskew = LaunchConfiguration('deskew')
    foxglove = LaunchConfiguration('foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('motor', default_value='false'),
        DeclareLaunchArgument('deskew', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry_mux'),
        DeclareLaunchArgument('backup', default_value='true'),
        DeclareLaunchArgument('backup_map', default_value=os.path.expanduser('~/ws/v_gosu/eval/out/gosu1030v2_map.npz')),
        DeclareLaunchArgument('backup_hz', default_value='5.0'),
        DeclareLaunchArgument('planner_config', default_value=planner_default),

        # ── 센서 ──────────────────────────────────────────────────────────────
        # 실기에서만. 재생 검증은 lidar:=false (점군은 bag 이 준다)
        Node(namespace='hesai_ros_driver', package='hesai_ros_driver',
             executable='hesai_ros_driver_node', name='hesai_ros_driver_node',
             output='screen', condition=IfCondition(lidar)),

        # ── 브리지 ────────────────────────────────────────────────────────────
        Node(package='go2_bridge', executable='imu_bridge_node', name='go2_imu_bridge',
             output='screen',
             parameters=[{'input_topic': '/lowstate', 'output_topic': '/imu/body',
                          'frame_id': 'imu_link', 'publish_static_tf': True,
                          'publish_stab': True}]),
        # 디스큐 없음 (팀 브리지) — 기본
        Node(package='go2_bridge', executable='cloud_bridge_node', name='go2_cloud_bridge',
             output='screen', condition=UnlessCondition(deskew),
             parameters=[{'input_topic': '/lidar_points', 'output_topic': '/velodyne_points',
                          'frame_id': 'hesai_lidar', 'stride': 1}]),
        # 디스큐 있음 (liosam 브리지) — deskew:=true
        Node(package='go2_liosam_bridge', executable='cloud_bridge', name='go2_cloud_bridge',
             output='screen', condition=IfCondition(deskew),
             parameters=[{'input_topic': '/lidar_points', 'output_topic': '/velodyne_points',
                          'frame_id': 'hesai_lidar', 'stride': 1, 'deskew': True}]),

        # ── SLAM (오도메트리 겸 지도) ─────────────────────────────────────────
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_odometry',
             parameters=[odom_cfg], output='screen'),
        Node(package='go2_bridge', executable='odom_filter_node', name='go2_odom_filter',
             output='screen',
             parameters=[{'input_topic': '/Odometry_mux', 'output_topic': '/Odometry_filtered',
                          'alpha_pos': 0.65, 'alpha_rot': 0.60, 'max_step_dist': 0.20}]),
        Node(package='go2_amcl_layer', executable='odom_to_tf', name='slam_odom_to_tf',
             output='screen',
             parameters=[{'odom_topic': LaunchConfiguration('odom_topic'),
                          'parent_frame': 'odom', 'child_frame': 'base_link'}]),

        # ── 백업 위치추정 v2 (night_20261104) + 전환 장치 v2 (night_20261104, v_gosu 사본) ──────────────
        ExecuteProcess(cmd=['nice', '-n', '5', os.path.expanduser('~/ws/night_20261104/venv/bin/python'), v2dir + '/backup_loc_node_v2.py', '--map', LaunchConfiguration('backup_map'),
                            '--hz', LaunchConfiguration('backup_hz')], output='screen', condition=IfCondition(backup)),
        # 전환 장치는 항상 띄운다 — backup:=false 면 중계만(원본과 같은 동작)
        ExecuteProcess(cmd=['python3', v2dir + '/odom_mux_node_v2.py'], output='screen', condition=IfCondition(backup)),
        ExecuteProcess(cmd=['python3', v2dir + '/odom_mux_node_v2.py', '--no-switch'], output='screen', condition=UnlessCondition(backup)),

        # ── 계획 / 제어 ───────────────────────────────────────────────────────
        Node(package='ego_planner', executable='ego_planner_node', name='ego_planner_node',
             parameters=[LaunchConfiguration('planner_config')], output='screen'),
        Node(package='go2_bridge', executable='udp_goal_bridge_node', name='udp_goal_bridge_node',
             output='screen',
             parameters=[{'odom_topic': '/Odometry_filtered', 'goal_topic': '/goal_pose',
                          'path_topic': '/ego_planner/trajectory',
                          'cloud_topic': '/cloud_registered', 'udp_port': 9870}]),
        # ★ 모터 — motor:=true 일 때만. 기본은 꺼짐.
        Node(package='go2_bridge', executable='cmd_vel_to_sport_node',
             name='go2_cmd_vel_to_sport', output='screen',
             parameters=[{'cmd_vel_topic': '/cmd_vel', 'sport_req_topic': '/api/sport/request',
                          'max_vx': 0.8, 'max_vy': 0.3, 'max_wz': 1.5, 'deadman_s': 0.5}],
             condition=IfCondition(motor)),

        # ── 보기용 ────────────────────────────────────────────────────────────
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             parameters=[{'port': 8765}], output='log', condition=IfCondition(foxglove)),
    ])
