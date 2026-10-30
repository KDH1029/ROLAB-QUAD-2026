import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mission_vision')
    config_file = os.path.join(pkg_dir, 'config', 'yolo_params.yaml')

    return LaunchDescription([
        Node(
            package='mission_vision',
            executable='yolo_detector_node.py',
            name='yolo_detector_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
