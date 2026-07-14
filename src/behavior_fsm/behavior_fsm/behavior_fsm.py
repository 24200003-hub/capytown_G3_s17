#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from behavior_fsm.wall_follower import WallFollowerController
from box_detector.lidar_utils import segmentar_lidar


class BehaviorFSMNode(Node):
    def __init__(self):
        super().__init__('behavior_fsm_node')

        self.controlador = WallFollowerController()

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.control_loop_callback,
            qos_profile
        )
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.estado_actual = "AVANZAR_CENTRADO"
        self.t_inicio_estado = time.time()
        self.t_ultima_esquina = 0.0
        self.t_inicio_avance_post = time.time()

        # Antihorario: las esquinas del circuito se toman hacia la izquierda.
        self.contador_esquinas = 0
        self.indice_esquina_actual = 0
        self.giro_esquina_base = 0.22
        self.avance_giro = 0.014
        self.detencion_esquina = 0.26
        self.cooldown_esquina = 0.75
        self.bloqueo_reentrada_seg = 0.85
        self.max_tiempo_giro_por_esquina = (3.40, 2.50, 3.10, 3.10)
        self.factor_giro_por_esquina = (1.00, 0.82, 0.92, 0.92)
        self.max_tiempo_alinear = 1.0
        self.avance_post_seg = 0.55
        self.avance_post_max_seg = 1.55

        self.get_logger().info('Behavior FSM por esquinas: antihorario con bloqueo post-giro.')

    def cambiar_estado(self, nuevo):
        if self.estado_actual != nuevo:
            self.estado_actual = nuevo
            self.t_inicio_estado = time.time()
            self.get_logger().info(f"Estado -> {nuevo}")

    def valor_esquina(self, valores, defecto):
        try:
            return float(valores[self.indice_esquina_actual % len(valores)])
        except Exception:
            return float(defecto)

    def deteccion_bloqueada(self, ahora, frente):
        dt = ahora - self.t_ultima_esquina
        if frente < 0.22 and dt > 0.35:
            return False
        return dt < self.cooldown_esquina or dt < self.bloqueo_reentrada_seg

    def control_loop_callback(self, msg):
        sectores = segmentar_lidar(msg.ranges, msg.angle_min, msg.angle_increment)
        cmd_vel = Twist()
        ahora = time.time()
        en_cooldown = self.deteccion_bloqueada(ahora, sectores['frente'])

        if self.estado_actual == "AVANZAR_CENTRADO":
            if sectores.get('esquina', False) and sectores['frente'] < 0.55 and not en_cooldown:
                self.indice_esquina_actual = self.contador_esquinas % 4
                self.get_logger().info(f"Preparando esquina #{self.indice_esquina_actual + 1}")
                self.cambiar_estado("ACERCAR_ESQUINA")
            elif sectores.get('pasadizo', False):
                cmd_vel.linear.x = self.controlador.ajustar_velocidad_lineal(sectores['frente'], velocidad_base=0.18)
                cmd_vel.angular.z = self.controlador.calcular_giro(sectores['izquierda'])
            elif sectores['frente'] < 0.22 and not en_cooldown:
                self.indice_esquina_actual = self.contador_esquinas % 4
                self.cambiar_estado("GIRAR_ESQUINA")
            else:
                cmd_vel.linear.x = self.controlador.ajustar_velocidad_lineal(sectores['frente'], velocidad_base=0.16)
                cmd_vel.angular.z = self.controlador.calcular_giro(sectores['izquierda'])

        if self.estado_actual == "ACERCAR_ESQUINA":
            if sectores.get('pasadizo', False) and sectores['frente'] > 0.45:
                self.cambiar_estado("AVANZAR_CENTRADO")
            elif sectores['frente'] <= self.detencion_esquina:
                self.cambiar_estado("GIRAR_ESQUINA")
            else:
                cmd_vel.linear.x = 0.04
                cmd_vel.angular.z = max(-0.16, min(0.16, self.controlador.calcular_giro(sectores['izquierda'])))

        if self.estado_actual == "GIRAR_ESQUINA":
            tiempo = ahora - self.t_inicio_estado
            lateral = sectores['izquierda'] < 1.15 or sectores['derecha'] < 1.15
            max_t = self.valor_esquina(self.max_tiempo_giro_por_esquina, 3.2)
            factor = self.valor_esquina(self.factor_giro_por_esquina, 1.0)
            if (sectores['frente'] > 0.36 and lateral) or tiempo > max_t:
                self.t_ultima_esquina = ahora
                self.contador_esquinas += 1
                self.cambiar_estado("ALINEAR_POST_GIRO")
            else:
                cmd_vel.linear.x = self.avance_giro if sectores['frente'] > 0.18 else 0.0
                cmd_vel.angular.z = self.giro_esquina_base * factor  # izquierda

        if self.estado_actual == "ALINEAR_POST_GIRO":
            tiempo = ahora - self.t_inicio_estado
            if sectores.get('pasadizo', False) or tiempo > self.max_tiempo_alinear:
                self.t_inicio_avance_post = ahora
                self.cambiar_estado("AVANZAR_POST_ESQUINA")
            else:
                # Corrección corta, sin permitir otra vuelta completa.
                cmd_vel.linear.x = 0.030 if sectores['frente'] > 0.22 else 0.0
                cmd_vel.angular.z = 0.055

        if self.estado_actual == "AVANZAR_POST_ESQUINA":
            tiempo = ahora - self.t_inicio_estado
            if (tiempo > self.avance_post_seg and sectores['frente'] > 0.24) or tiempo > self.avance_post_max_seg:
                self.cambiar_estado("AVANZAR_CENTRADO")
            elif sectores['frente'] < 0.20 and tiempo > 0.35:
                # Tramo corto: permite tomar la siguiente esquina sin hacer vuelta completa.
                self.cambiar_estado("AVANZAR_CENTRADO")
            else:
                cmd_vel.linear.x = 0.045 if sectores['frente'] > 0.20 else 0.0
                cmd_vel.angular.z = max(-0.10, min(0.10, self.controlador.calcular_giro(sectores['izquierda'])))

        self.vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo el cerebro del robot...')
    finally:
        parar_vel = Twist()
        node.vel_pub.publish(parar_vel)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
