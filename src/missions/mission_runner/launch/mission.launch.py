from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_relative_route = LaunchConfiguration('use_relative_route')
    reference_frame = LaunchConfiguration('reference_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_relative_route', default_value='false'),
        DeclareLaunchArgument('reference_frame', default_value='map'),

        # Vision (traffic light, parking sign) is the teammate's, not in this repo.
        Node(
            package='mission_runner',
            executable='mission_fsm_node',
            name='mission_fsm_node',
            output='screen',
            parameters=[{
                'use_relative_route': use_relative_route,
                'reference_frame': reference_frame,
            }]
        ),
    ])
