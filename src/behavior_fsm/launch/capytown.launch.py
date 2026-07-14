"""
capytown.launch.py
------------------
Lanzamiento COMPLETO del reto: driver LiDAR + detector (Parte A) + guardian (Parte B)
+ logger de metricas + route_mapper.

    ros2 launch behavior_fsm capytown.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_det = os.path.join(
        get_package_share_directory('box_detector'), 'config', 'params.yaml')
    params_fsm = os.path.join(
        get_package_share_directory('behavior_fsm'), 'config', 'params.yaml')

    # Driver del LiDAR (Yahboom MS200). Ajusta package/executable a tu driver real.
    driver_lidar = Node(
        package='ms200_driver',          # <-- cambia segun tu driver
        executable='ms200_driver_node',  # <-- cambia segun tu driver
        name='lidar_driver',
        output='screen',
    )

    detector = Node(
        package='box_detector',
        executable='box_detector',
        name='box_detector',
        output='screen',
        parameters=[params_det],
    )

    metrics = Node(
        package='box_detector',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[params_det],
    )

    guardian = Node(
        package='behavior_fsm',
        executable='behavior_fsm',
        name='behavior_fsm',
        output='screen',
        parameters=[params_fsm],
    )

    mapper = Node(
        package='route_mapper',
        executable='route_mapper_node',
        name='route_mapper_node',
        output='screen',
    )

    return LaunchDescription([
        # driver_lidar,   # <-- descomenta cuando tengas el driver correcto
        detector,
        metrics,
        guardian,
        mapper,
    ])
