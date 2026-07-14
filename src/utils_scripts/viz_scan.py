#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class VisualizadorScan(Node):
    def __init__(self):
        super().__init__('visualizador_scan_node')
        # Se suscribe al canal del escáner láser del carro
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Almacenamiento local de los datos del sensor
        self.last_angles = None
        self.last_ranges = None
        
        # Configuración de la ventana gráfica en 2D (Matplotlib)
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_title("Escáner Láser (LiDAR) del Robot en Tiempo Real", fontsize=12, pad=15)
        self.ax.set_rmax(3.5)  # Rango máximo visual en metros
        
        # Inicialización del gráfico de puntos (Puntitos rojos para simular el láser)
        self.scan_plot, = self.ax.plot([], [], 'ro', markersize=1.5, label='Paredes / Cajas')
        self.ax.legend(loc='upper right')
        
        self.get_logger().info('Nodo Visualizador de Láser listo. Esperando datos...')

    def scan_callback(self, msg):
        """ Captura los datos crudos del LiDAR y los convierte a coordenadas polares """
        ranges = np.array(msg.ranges)
        # Limpieza de lecturas inválidas o infinitas usando NumPy
        ranges[(ranges == 0.0) | (np.isinf(ranges)) | (np.isnan(ranges))] = 3.5
        
        # Reconstrucción geométrica de los ángulos de cada haz láser
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        self.last_angles = angles
        self.last_ranges = ranges

    def update_plot(self, frame):
        """ Función de refresco dinámico de la pantalla """
        if self.last_angles is not None and self.last_ranges is not None:
            # Actualiza la posición de los puntos en el gráfico polar
            self.scan_plot.set_data(self.last_angles, self.last_ranges)
        return self.scan_plot,

def main(args=None):
    rclpy.init(args=args)
    nodo_viz = VisualizadorScan()
    
    # Animación integrada de Matplotlib que se refresca cada 50ms (20 FPS)
    ani = FuncAnimation(nodo_viz.fig, nodo_viz.update_plot, blit=True, interval=50)
    
    # Hilo secundario para que ROS 2 reciba datos mientras Matplotlib dibuja en el hilo principal
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(nodo_viz), daemon=True)
    ros_thread.start()
    
    # Despliega la ventana gráfica en pantalla
    plt.show()
    
    # Cierre ordenado al cerrar la ventana
    nodo_viz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()