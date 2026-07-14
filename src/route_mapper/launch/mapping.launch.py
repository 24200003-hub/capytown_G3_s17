from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='route_mapper',
            executable='route_mapper_node',
            name='route_mapper_node',
            output='screen',
        )
    ])
