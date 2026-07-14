#!/usr/bin/env python3
"""Nodo opcional del mapper.

El ejecutable principal ya integra RouteMemoryMapper directamente para dibujar en GUI.
Este nodo queda como carpeta/paquete separado para que el proyecto sea modular y pueda
crecer después publicando mapa o métricas.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .path_mapper import RouteMemoryMapper


class RouteMapperNode(Node):
    def __init__(self):
        super().__init__('route_mapper_node')
        self.mapper = RouteMemoryMapper()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.tengo_odom = False

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_lidar)
        self.create_timer(2.0, self.log_resumen)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = self.quaternion_a_yaw(q.x, q.y, q.z, q.w)
        self.tengo_odom = True
        self.mapper.actualizar_pose(self.odom_x, self.odom_y, self.odom_yaw)

    def scan_callback(self, _msg):
        # El procesamiento completo de puntos se hace en el ejecutable integrado,
        # donde ya existe radar_utils. Este nodo queda listo para expansión.
        pass

    def quaternion_a_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def log_resumen(self):
        resumen = self.mapper.obtener_estado_resumen()
        self.get_logger().info(
            f"Mapa ruta: {resumen['path_points']} puntos, "
            f"dist={resumen['distance']:.2f} m, vueltas={resumen['laps']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RouteMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
