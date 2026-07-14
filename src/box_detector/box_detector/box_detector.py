#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# Importamos la función matemática que creamos en el paso anterior
from box_detector.lidar_utils import segmentar_lidar, detectar_escalon_caja

class BoxDetectorNode(Node):
    def __init__(self):
        super().__init__('box_detector_node')
        
        # 1. Configuración de QoS (Quality of Service) ideal para sensores (LiDAR)
        # 'best_effort' evita retrasos en la red si se pierden paquetes del escaneo
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Suscriptor al tópico del LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        
        # 3. Variables de estado del entorno
        self.distancia_pared_ideal = 0.60  # El ancho libre del pasillo según el plano es 60cm
        self.contador_cajas = 0
        self.viendo_caja_actualmente = False

        self.get_logger().info('¡Nodo BoxDetector inicializado y escuchando en /scan!')

    def lidar_callback(self, msg):
        """
        Esta función se ejecuta automáticamente cada vez que el LiDAR genera un nuevo escaneo.
        """
        # Procesamos los rangos crudos usando nuestra librería auxiliar
        sectores = segmentar_lidar(msg.ranges, msg.angle_min, msg.angle_increment)
        
        # Imprimimos telemetría básica en la terminal para telemetría/depuración
        self.get_logger().info(
            f"Frente: {sectores['frente']:.2f}m | Izq: {sectores['izquierda']:.2f}m | Der: {sectores['derecha']:.2f}m"
        )

        # --- LÓGICA DEL CENSO (CONTADOR DE CAJAS) ---
        # Analizamos el sector izquierdo (o el derecho, dependiendo de hacia dónde apunte tu circuito)
        hay_objeto_cerca = detectar_escalon_caja(
            sectores['izquierda'], 
            self.distancia_pared_ideal
        )

        # Máquina de estados simple para no contar múltiples veces la misma caja mientras pasamos al lado
        if hay_objeto_cerca and not self.viendo_caja_actualmente:
            self.viendo_caja_actualmente = True
            self.contador_cajas += 1
            self.get_logger().warn(f"¡Caja detectada! Conteo actual: {self.contador_cajas}")
            
        elif not hay_objeto_cerca and self.viendo_caja_actualmente:
            # El robot ya terminó de pasar la caja completamente
            self.viendo_caja_actualmente = False

    # NOTA PARA EL FUTURO: Aquí agregaremos un Publisher para enviarle los sectores limpios
    # directamente al cerebro (FSM) de forma empaquetada.

def main(args=None):
    rclpy.init(args=args)
    node = BoxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Apagando nodo BoxDetector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()