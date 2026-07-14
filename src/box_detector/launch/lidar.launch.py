"""
lidar.launch.py
---------------
Lanza el driver del LiDAR (MS200) y el detector de cajas.
Sirve para VERIFICAR que /scan llega y que el censo funciona, sin la FSM.

    ros2 launch box_detector lidar.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('box_detector'), 'config', 'params.yaml')

    # NOTA: ajusta 'package'/'executable' al driver real de tu LiDAR (Yahboom MS200).
    # Si ya tienes el driver corriendo aparte, comenta este nodo.
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
        parameters=[params],
    )

    return LaunchDescription([
        # driver_lidar,   # <-- descomenta cuando tengas el driver correcto
        detector,
    ])
