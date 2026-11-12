import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_wall_patrol')
    default_params_file = os.path.join(pkg_share, 'config', 'patrol_params.yaml')

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS 2 parameters file'
    )

    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='true',
        description='Whether to launch real robot hardware bridges (hesai lidar, imu, cmd_vel sport bridge)'
    )

    hesai_arg = DeclareLaunchArgument(
        'hesai',
        default_value='true',
        description='Whether to launch hesai_ros_driver_node'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Whether to auto start patrol immediately or stay in tuning mode'
    )

    hardware_cond = IfCondition(LaunchConfiguration('hardware'))
    hesai_cond = IfCondition(LaunchConfiguration('hesai'))

    # 1. Hesai LiDAR Driver (Hardware Lidar -> /lidar_points)
    hesai_node = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        condition=hesai_cond
    )

    # 2. Go2 IMU Bridge (/lowstate -> /imu/body)
    imu_bridge_node = Node(
        package='go2_bridge',
        executable='imu_bridge_node',
        name='go2_imu_bridge',
        output='screen',
        condition=hardware_cond,
        parameters=[{
            'input_topic': '/lowstate',
            'output_topic': '/imu/body',
            'frame_id': 'imu_link',
            'publish_static_tf': True,
            'publish_stab': True,
        }]
    )

    # 3. Hesai Cloud Bridge (/lidar_points -> /velodyne_points)
    cloud_bridge_node = Node(
        package='go2_bridge',
        executable='cloud_bridge_node',
        name='go2_cloud_bridge',
        output='screen',
        condition=hardware_cond,
        parameters=[{
            'input_topic': '/lidar_points',
            'output_topic': '/velodyne_points',
            'frame_id': 'hesai_lidar',
            'stride': 1,
        }]
    )

    # 4. CmdVel to Go2 Sport API Bridge (/cmd_vel -> /api/sport/request API 1008)
    cmd_vel_bridge_node = Node(
        package='go2_bridge',
        executable='cmd_vel_to_sport_node',
        name='go2_cmd_vel_bridge',
        output='screen',
        condition=hardware_cond,
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'sport_req_topic': '/api/sport/request',
            'max_vx': 0.8,
            'max_vy': 0.4,
            'max_wz': 1.2,
            'max_ax': 0.6,
        }]
    )

    # 5. Go2 Sport Bridge (Posture SIT / STAND -> /api/sport/request API 1005/1009 & 1004/1010)
    sport_bridge_node = Node(
        package='go2_wall_patrol',
        executable='go2_sport_bridge.py',
        name='go2_sport_bridge',
        output='screen'
    )

    # 6. Wall Runner Node (Core autonomous outer-wall following & corner sit/stand logic)
    wall_runner_node = Node(
        package='go2_wall_patrol',
        executable='wall_runner_node',
        name='wall_runner_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'auto_start': LaunchConfiguration('auto_start')}
        ]
    )

    # 7. Patrol UDP Bridge (Telemetry & Control for PC GUI over UDP Port 9880)
    udp_bridge_node = Node(
        package='go2_wall_patrol',
        executable='patrol_udp_bridge.py',
        name='patrol_udp_bridge',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(hardware_arg)
    ld.add_action(hesai_arg)
    ld.add_action(auto_start_arg)

    ld.add_action(hesai_node)
    ld.add_action(imu_bridge_node)
    ld.add_action(cloud_bridge_node)
    ld.add_action(cmd_vel_bridge_node)
    ld.add_action(sport_bridge_node)
    ld.add_action(udp_bridge_node)
    ld.add_action(wall_runner_node)

    return ld
