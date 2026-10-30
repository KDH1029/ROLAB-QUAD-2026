from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_vision',
            executable='traffic_light_node',
            name='traffic_light_node',
            output='screen'
        ),
        Node(
            package='mission_vision',
            executable='parking_marker_node',
            name='parking_marker_node',
            output='screen'
        ),
        Node(
            package='mission_vision',
            executable='cone_detector_node',
            name='cone_detector_node',
            output='screen'
        ),
    ])
