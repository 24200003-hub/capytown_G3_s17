#!/usr/bin/env python3
import sys
import os
import time
import math
import threading
import json
import csv
import glob
import heapq
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# OpenCV es opcional: si no está instalado, el robot sigue usando LiDAR.
try:
    import cv2
    CV2_DISPONIBLE = True
except Exception:
    cv2 = None
    CV2_DISPONIBLE = False

# Inyectar dinámicamente la ruta src de Reto_03
ruta_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '../src'))
if ruta_src not in sys.path:
    sys.path.append(ruta_src)

# Ruta extra para el paquete route_mapper agregado en src/route_mapper/route_mapper
ruta_route_mapper = os.path.abspath(os.path.join(ruta_src, 'route_mapper'))
if ruta_route_mapper not in sys.path:
    sys.path.append(ruta_route_mapper)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from utils_scripts.control_config import ControlConfig
from utils_scripts.radar_utils import procesar_escaneo_lidar
from utils_scripts.radar_interface import RadarInterface
from utils_scripts.box_avoidance import BoxAvoidanceFSM
from route_mapper.path_mapper import RouteMemoryMapper


class SistemaControlBorde(Node):
    """
    Nodo integrado y modular:
    1) Al ejecutar, abre la interfaz y deja motores en 0.
    2) Al presionar INICIAR, conduce por el centro entre pared derecha e izquierda.
    3) La pared derecha/izquierda se valida por clusters conectados al lateral ±90°.
       Así la pared del frente NO se toma como derecha.
    4) Si solo ve una pared lateral, usa seguimiento lateral suave como respaldo.
    5) Integra memoria de recorrido: ruta, paredes/puntos detectados y control anti-retorno.
    6) La velocidad lineal y angular se cambian desde la GUI; la batería se lee correctamente.
    """

    def __init__(self):
        super().__init__('sistema_control_centrado_modular')

        self.cfg = ControlConfig()

        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_lidar)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_sub = self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.ui = RadarInterface(
            callback_iniciar=self.handler_iniciar,
            callback_detener=self.handler_detener,
            callback_salir=self.handler_salir,
            callback_vel_lenta=self.handler_vel_lenta,
            callback_vel_media=self.handler_vel_media,
            callback_vel_rapida=self.handler_vel_rapida,
            callback_ang_menos=self.handler_ang_menos,
            callback_ang_mas=self.handler_ang_mas,
            callback_cargar_mapa=self.handler_cargar_mapa,
            callback_usar_mapa=self.handler_usar_mapa,
        )
        self.fig = self.ui.fig

        # Telemetría LiDAR
        self.datos_filtrados = None
        self.dist_frente = float('inf')
        self.dist_izq = float('inf')
        self.dist_der = float('inf')
        self.dist_diag_der = float('inf')
        self.dist_diag_izq = float('inf')

        # Distancias laterales filtradas por clusters conectados.
        self.dist_der_pared = float('inf')
        self.dist_izq_pared = float('inf')
        self.pendiente_pared_der = 0.0
        self.pendiente_pared_izq = 0.0
        self.pared_der_valida = False
        self.pared_izq_valida = False
        self.puntos_pared_der = 0
        self.puntos_pared_izq = 0
        self.ancho_pasillo = float('inf')
        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        self.total_puntos = 0

        # Odometría
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.tengo_odom = False

        # Control y estados
        self.estado_actual = "ESPERANDO INICIO"
        self.robot_habilitado = False
        self.robot_pausado = False
        self.solicitud_salir = False
        self.ciclos_estable = 0
        self.error_anterior = 0.0
        self.t_anterior = time.time()
        self.t_estado = time.time()
        self.estado_post_pausa = None
        self.sentido_giro_frente = self.cfg.sentido_giro_esquina
        self.t_inicio_giro_frente = time.time()
        self.t_inicio_alinear_post_giro = time.time()
        self.t_ultima_esquina = 0.0
        self.yaw_inicio_giro = None
        self.ultimo_lado_pared = "izquierda"  # antihorario: se prioriza pared izquierda / isla central

        # Bloqueo temporal de referencia lateral.
        # Evita que, antes/durante una esquina, el robot pierda la pared derecha
        # y tome la pared izquierda como nueva referencia, provocando una vuelta.
        self.referencia_estable = self.cfg.referencia_preferida_esquina
        self.referencia_candidata = None
        self.frames_referencia_candidata = 0
        self.referencia_bloqueada = None
        self.t_bloqueo_referencia = 0.0
        self.t_ultima_pared_der = 0.0
        self.t_ultima_pared_izq = 0.0

        # Control por esquinas: evita que una esquina corta dispare otro giro completo.
        self.contador_esquinas = 0
        self.indice_esquina_actual = 0
        self.esquina_en_proceso = False
        self.t_inicio_avance_post_esquina = time.time()
        self.x_inicio_avance_post_esquina = 0.0
        self.y_inicio_avance_post_esquina = 0.0
        self.x_fin_ultima_esquina = 0.0
        self.y_fin_ultima_esquina = 0.0
        self.lateral_estable_post_giro = 0

        # Salida protegida de esquina:
        # después de girar, el robot avanza recto unos cm sin volver a
        # analizar otra esquina. Esto evita que la 4ta esquina acumule giros
        # y termine regresando por donde vino.
        self.yaw_salida_post_esquina = None
        self.sentido_salida_post_esquina = 1.0
        self.indice_salida_post_esquina = 0

        # Análisis frontal: pasadizo vs esquina.
        self.tipo_frente = "SIN_DATOS"
        self.frente_es_pasillo = False
        self.frente_es_esquina = False
        self.frente_conecta_der = False
        self.frente_conecta_izq = False
        self.frente_conecta_ambos = False
        self.frente_confianza = 0.0
        self.front_class_sm = "NONE"
        self.front_ang_width = 0.0
        self.front_dist_sm = float('inf')
        self.alpha_pared_der = 0.0
        self.alpha_pared_izq = 0.0
        self.len_pared_der = 0.0
        self.len_pared_izq = 0.0

        # Persistencia tipo CapyGuardian: una lectura mala no cambia de estado.
        self._persist = {}

        # Conteo simple de cajas detectadas por arco frontal corto.
        self.contador_cajas = 0
        self.t_ultima_caja = 0.0

        # Control de carril central / hueco libre.
        # Esta capa evita que el robot se pegue a la pared exterior cuando el
        # espacio es amplio: busca el centro del hueco libre frontal y solo usa
        # paredes como límites de seguridad.
        self.gap_error_anterior = 0.0
        self.t_gap_anterior = time.time()
        self.gap_target_deg = 0.0
        self.gap_width_deg = 0.0
        self.gap_clearance = float('inf')

        # FSM modular de rodeo de caja: giro IZQ -> DER -> DER -> IZQ.
        self.rodeo_caja = BoxAvoidanceFSM(self.cfg, logger=self.get_logger())

        # Selector de 3 velocidades lineales desde la GUI.
        self.modo_velocidad = self.cfg.modo_velocidad_inicial
        self.factor_velocidad = self.obtener_factor_velocidad(self.modo_velocidad)
        self.nombre_modo_velocidad = self.obtener_nombre_velocidad(self.modo_velocidad)

        # Control angular independiente desde GUI.
        self.factor_angular = self.cfg.factor_angular_inicial

        # Velocidades publicadas.
        self.vel_lineal = 0.0
        self.vel_angular = 0.0

        # Anti-choque global: capa superior que se aplica justo antes de
        # publicar cmd_vel. No cambia la estrategia de navegación, solo impide
        # avanzar si el LiDAR ve riesgo real de choque.
        self.antichoque_activo = False
        self.antichoque_motivo = "OK"

        # Modo aspiradora: navegación simple y robusta.
        # Avanza por zona libre, gira sin avanzar cuando el frente se cierra,
        # sigue una pared con distancia prudente y no cambia de decisión cada frame.
        self.asp_turn_dir = float(self.cfg.aspiradora_default_turn)
        self.asp_turn_t0 = time.time()
        self.asp_decision_t0 = 0.0
        self.asp_estado_anterior = ""
        self.asp_error_prev = 0.0
        self.asp_t_prev = time.time()
        self.asp_backoff_t0 = 0.0
        self.asp_last_map_update = 0.0

        # Modo laberinto / micromouse simple.
        # Decide en intersecciones, bloquea la decisión, sigue pared izquierda
        # y usa el mapa LiDAR 360° como memoria visual, no como giro libre.
        self.lab_turn_dir = float(getattr(self.cfg, "lab_default_turn", 1.0))
        self.lab_turn_t0 = time.time()
        self.lab_decision_t0 = 0.0
        self.lab_backoff_t0 = 0.0
        self.lab_exit_t0 = 0.0
        self.lab_error_prev = 0.0
        self.lab_t_prev = time.time()
        self.lab_estado_anterior = ""
        self.lab_ultimo_nodo = None
        self.lab_intersecciones = deque(maxlen=120)
        self.lab_callejones = deque(maxlen=80)
        self.lab_decision_actual = "NINGUNA"

        # Capybot K: exploración estable tipo laberinto.
        # Evita quedarse pensando: si no hay progreso, ejecuta una maniobra
        # de escape completa sin volver a decidir a cada frame.
        self.lab_escape_t0 = 0.0
        self.lab_escape_dir = float(getattr(self.cfg, "lab_default_turn", 1.0))
        self.lab_escape_motivo = ""
        self.lab_last_progress_t = time.time()
        self.lab_last_progress_distance = 0.0
        self.lab_last_progress_pose = (0.0, 0.0)
        self.lab_last_intersection_t = 0.0
        self.lab_last_deadend_t = 0.0
        self.lab_frontier_target = "frente"

        # Modo experimental AC: usar mapa aprendido.
        # Esta capa se activa SOLO por botón; no modifica el modo exploración estable.
        self.mapa_aprendido_cargado = False
        self.usar_mapa_aprendido_activo = False
        self.mapa_aprendido_path = None
        self.mapa_aprendido_data = None
        self.mapa_aprendido_waypoints = []
        self.mapa_aprendido_waypoint_idx = 0
        self.mapa_aprendido_estado = "SIN_CARGAR"
        self.mapa_aprendido_fuente = ""

        # Capybot AG: usar el mapa aprendido como memoria de decisiones,
        # no como coordenadas exactas. Esto evita que A* o waypoints sobre
        # cmd_vel deformado manden al robot contra paredes.
        self.usar_decisiones_aprendidas_activo = False
        self.mapa_decisiones_aprendidas = []
        self.mapa_decision_idx = 0
        self.mapa_decision_ultima_t = 0.0
        self.mapa_decision_fuente = ""

        self.mapa_runtime_shift_x = 0.0
        self.mapa_runtime_shift_y = 0.0
        self.mapa_progress_last_dist = float("inf")
        self.mapa_progress_last_t = time.time()
        self.mapa_progress_last_idx = 0
        self.mapa_front_block_t = None
        self.mapa_recovery_until = 0.0

        # Batería real: se lee desde /battery. Si no llega, se muestra SIN DATOS.
        self.voltaje_bateria = float('nan')
        self.porcentaje_bateria = None
        self.bateria_fuente = "SIN DATOS"
        self.bateria_real_recibida = False

        # Cámara / señal PARE.
        # IMPORTANTE: esta capa NO cambia la lógica del laberinto. Solo actúa
        # como arbitraje superior: si la cámara confirma rojo tipo PARE,
        # fuerza motores en 0 durante 3 segundos y luego devuelve el control
        # a la navegación que ya funcionaba.
        self.camera_enabled = bool(getattr(self.cfg, "usar_camara_pare", True))
        self.camera_running = False
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        self.pare_detectado = False
        self.pare_activo_hasta = 0.0
        self.pare_cooldown_hasta = 0.0
        self.pare_frames_ok = 0
        self.pare_frames_miss = 0
        self.pare_area_ratio = 0.0
        self.pare_bbox = None
        self.pare_ultimo_t = 0.0
        self.pare_estado_previo = None
        self.pare_detenciones = 0
        self.pare_falsos = 0
        # Capa pasiva de aprendizaje: posiciones aproximadas donde se detectó PARE.
        # No decide movimiento; solo se guarda en mapa_laberinto.json/csv.
        self.pare_eventos = []
        self.pare_fuente = "OFF"
        # Vista de cámara para la interfaz. No afecta navegación: solo guarda
        # el último frame anotado para mostrar qué está viendo el robot.
        self.camera_frame_rgb = None
        self.camera_frame_t = 0.0
        self.camera_preview_estado = "SIN_CAMARA"

        # Meta conocida del reto: esquina superior derecha del laberinto oficial.
        # Capa pasiva: solo avisa/guarda cuando cree estar cerca de META.
        # No modifica la navegación que ya funciona.
        self.meta_cree_llegar = False
        self.meta_confirmada_t = 0.0
        self.meta_distancia = float('inf')
        self.meta_x = float(getattr(self.cfg, "meta_x_m", 3.60))
        self.meta_y = float(getattr(self.cfg, "meta_y_m", 2.40))
        self.meta_radio = float(getattr(self.cfg, "meta_radio_m", 0.48))
        self.meta_eventos = []

        # Meta por cámara: detector verde independiente del PARE.
        # No depende de la odometría estimada: si ve el cartel verde META,
        # alinea un poco por visión, avanza corto y recién marca llegada.
        self.meta_vision_detectada = False
        self.meta_vision_bbox = None
        self.meta_vision_area_ratio = 0.0
        self.meta_vision_error_x = 0.0
        self.meta_vision_frames_ok = 0
        self.meta_vision_frames_miss = 0
        self.meta_vision_ultimo_t = 0.0
        self.meta_vision_detectadas = 0
        self.meta_vision_activa = False
        self.meta_vision_avanzar_hasta = 0.0
        self.meta_vision_finalizada = False

        # Métricas visibles del reto: solo lectura/reportes, no afectan navegación.
        self.t_inicio_corrida = None
        self.t_meta_alcanzada = None
        self.pare_respetados = 0
        self.colisiones_estimadas = 0
        self._colision_cooldown_hasta = 0.0

        # Memoria/mapa del recorrido y puntos detectados por LiDAR.
        self.mapa_ruta = RouteMemoryMapper()
        # Capybot N: el mapa tipo laberinto NO debe guardar puntos mientras
        # el robot gira/escapa, porque eso crea nubes circulares. Además se
        # limita el rango a paredes cercanas del circuito, no a objetos lejanos
        # del laboratorio.
        try:
            self.mapa_ruta.max_map_range = float(getattr(self.cfg, "mapa_max_range_laberinto", self.mapa_ruta.max_map_range))
            self.mapa_ruta.grid_resolution = float(getattr(self.cfg, "mapa_grid_res_laberinto", self.mapa_ruta.grid_resolution))
            self.mapa_ruta.lidar_stride = int(getattr(self.cfg, "mapa_lidar_stride", self.mapa_ruta.lidar_stride))
            self.mapa_ruta.occ_hit_threshold = int(getattr(self.cfg, "mapa_occ_hit_threshold", self.mapa_ruta.occ_hit_threshold))
        except Exception:
            pass
        self.evitar_retorno_activo = False

        # Historial para reporte al pausar.
        # Se usa para plotear recorrido + factores relevantes sin frenar el control.
        self.historial_control = deque(maxlen=3500)
        self.ultimo_reporte_pausa = None

        # Trayectoria independiente SOLO PARA EL PLOT.
        # No modifica la lógica de navegación ni los comandos del robot.
        # Si /odom no se mueve o no llega, se estima la ruta integrando cmd_vel
        # para que al pausar aparezca el recorrido como en el dashboard de referencia.
        self.plot_x = 0.0
        self.plot_y = 0.0
        self.plot_yaw = 0.0
        self.plot_total_distance = 0.0
        self.plot_last_t = None
        self.plot_path = deque(maxlen=5000)
        self.plot_path.append((0.0, 0.0))
        self.plot_source = "cmd_vel estimado"

        # Estadísticas de procesamiento.
        self.tiempos_proc = deque(maxlen=80)
        self.tiempos_loop = deque(maxlen=80)
        self.ultimo_scan_time = None
        self.t_proc_actual = 0.0

        # Timer de seguridad: aunque no llegue scan, publica 0 al inicio/pausa.
        self.timer_seguridad = self.create_timer(0.20, self.timer_seguridad_callback)

        self.iniciar_detector_pare_camara()

        self.get_logger().info("Interfaz lista. El robot NO se moverá hasta presionar INICIAR.")

    # ==========================================================
    # BOTONES DE INTERFAZ
    # ==========================================================
    def handler_iniciar(self, event):
        self.robot_habilitado = True
        self.robot_pausado = False
        # Inicio de cronómetro de la corrida. No modifica la lógica de control.
        if self.t_inicio_corrida is None or self.estado_actual in {"ESPERANDO INICIO", "META_ALCANZADA"}:
            self.t_inicio_corrida = time.time()
            self.t_meta_alcanzada = None
        self.plot_last_t = time.time()
        if getattr(self.cfg, "usar_modo_laberinto", False):
            self.cambiar_estado("LAB_AVANZAR")
            self.lab_decision_actual = "AVANZAR"
            self.get_logger().info("INICIAR presionado: modo laberinto/micromouse con mapa LiDAR 360.")
        elif getattr(self.cfg, "usar_modo_aspiradora", False):
            self.cambiar_estado("ASPIRADORA_AVANZAR")
            self.get_logger().info("INICIAR presionado: modo aspiradora con mapa + LiDAR 360.")
        else:
            self.cambiar_estado("CENTRAR_PASILLO")
            self.get_logger().info("INICIAR presionado: conduciendo por el centro del pasillo.")

    def handler_detener(self, event):
        if not self.robot_habilitado:
            self.detener_robot()
            self.estado_actual = "ESPERANDO INICIO"
            return

        self.robot_pausado = not self.robot_pausado
        if self.robot_pausado:
            self.estado_post_pausa = self.estado_actual
            self.detener_robot()
            self.estado_actual = "PAUSA MANUAL"
            self.get_logger().warn("PAUSA: motores detenidos. Generando reporte del recorrido...")
            self.generar_reporte_pausa()
            self.guardar_mapa_aprendido(motivo="pausa")
        else:
            self.plot_last_t = time.time()
            self.cambiar_estado(self.estado_post_pausa or "CENTRAR_PASILLO")
            self.get_logger().info("REANUDAR: control activo.")

    def handler_salir(self, event):
        self.get_logger().info("Cerrando aplicación por interfaz gráfica...")
        self.solicitud_salir = True
        self.camera_running = False
        self.detener_robot()
        self.guardar_mapa_aprendido(motivo="salir")
        plt.close(self.fig)

    def handler_cargar_mapa(self, event):
        """Botón CARGAR MAPA.

        Solo carga el mapa aprendido y prepara la ruta A*. No mueve el robot.
        Mantiene intacto el modo normal de navegación.
        """
        ok = self.cargar_mapa_aprendido()
        if ok:
            self.usar_mapa_aprendido_activo = False
            self.robot_pausado = False
            self.cambiar_estado("MAPA CARGADO")
            try:
                self.get_logger().warn(
                    f"MAPA CARGADO: {self.mapa_decision_fuente or self.mapa_aprendido_fuente}, "
                    f"decisiones={len(getattr(self, 'mapa_decisiones_aprendidas', []))}, "
                    f"waypoints={len(self.mapa_aprendido_waypoints)}"
                )
            except Exception:
                pass
        else:
            self.detener_robot()
            self.cambiar_estado("MAPA NO CARGADO")

    def handler_usar_mapa(self, event):
        """Botón USAR MAPA.

        En AG ya no se sigue el mapa como coordenadas exactas. El robot
        conserva la navegación estable que ya llega a meta, pero usa el mapa
        aprendido como memoria de decisiones: en intersecciones/cierres intenta
        repetir la secuencia de giros que funcionó en la corrida aprendida.
        """
        if not self.mapa_aprendido_cargado:
            if not self.cargar_mapa_aprendido():
                self.detener_robot()
                self.cambiar_estado("MAPA SIN RUTA")
                return

        if not getattr(self, "mapa_decisiones_aprendidas", []):
            self.detener_robot()
            self.cambiar_estado("MAPA SIN DECISIONES")
            try:
                self.get_logger().warn("USAR MAPA: no hay decisiones aprendidas válidas; usa INICIAR para modo estable.")
            except Exception:
                pass
            return

        # Este modo NO activa control por waypoints/A*. Activa decisiones aprendidas
        # dentro de la misma FSM de laberinto que ya funciona.
        self.usar_mapa_aprendido_activo = False
        self.usar_decisiones_aprendidas_activo = True
        self.mapa_decision_idx = 0
        self.mapa_decision_ultima_t = 0.0
        self.robot_habilitado = True
        self.robot_pausado = False
        if self.t_inicio_corrida is None:
            self.t_inicio_corrida = time.time()
        self.plot_last_t = time.time()
        self.cambiar_estado("USAR_MAPA_DECISIONES")
        self.lab_decision_actual = f"MAPA_DEC 0/{len(self.mapa_decisiones_aprendidas)}"
        try:
            self.get_logger().warn(
                f"USAR MAPA DECISIONES: {len(self.mapa_decisiones_aprendidas)} decisiones desde "
                f"{self.mapa_decision_fuente or os.path.basename(str(self.mapa_aprendido_path))}"
            )
        except Exception:
            pass

    def handler_vel_lenta(self, event):
        self.set_velocidad(1)

    def handler_vel_media(self, event):
        self.set_velocidad(2)

    def handler_vel_rapida(self, event):
        self.set_velocidad(3)

    def handler_ang_menos(self, event):
        self.set_factor_angular(self.factor_angular - self.cfg.factor_angular_paso)

    def handler_ang_mas(self, event):
        self.set_factor_angular(self.factor_angular + self.cfg.factor_angular_paso)

    def set_velocidad(self, modo):
        self.modo_velocidad = modo
        self.factor_velocidad = self.obtener_factor_velocidad(modo)
        self.nombre_modo_velocidad = self.obtener_nombre_velocidad(modo)
        self.get_logger().info(
            f"Velocidad lineal seleccionada: {self.nombre_modo_velocidad} "
            f"(factor {self.factor_velocidad:.2f})"
        )

    def set_factor_angular(self, factor):
        self.factor_angular = max(self.cfg.factor_angular_min, min(self.cfg.factor_angular_max, float(factor)))
        self.get_logger().info(f"Factor de velocidad angular: {self.factor_angular:.2f}x")

    def porcentaje_a_voltaje(self, porcentaje):
        porcentaje = max(0, min(100, porcentaje))
        return 10.5 + (porcentaje / 100.0) * 2.1

    def obtener_factor_velocidad(self, modo):
        if modo == 1:
            return self.cfg.factor_lento
        if modo == 3:
            return self.cfg.factor_rapido
        return self.cfg.factor_medio

    def obtener_nombre_velocidad(self, modo):
        if modo == 1:
            return "LENTO"
        if modo == 3:
            return "RÁPIDO"
        return "MEDIO"


    # ==========================================================
    # CÁMARA — DETECCIÓN DE PARE ROJO
    # ==========================================================
    def iniciar_detector_pare_camara(self):
        """Arranca un hilo liviano para detectar rojo/PARE con OpenCV.

        No depende de ROS ni de cv_bridge. Usa /dev/video0 por defecto.
        Si OpenCV o la cámara no están disponibles, avisa y la navegación LiDAR
        continúa exactamente igual.
        """
        if not self.camera_enabled:
            self.pare_fuente = "DESACTIVADO"
            self.camera_preview_estado = "CAMARA DESACTIVADA"
            return
        if not CV2_DISPONIBLE:
            self.pare_fuente = "SIN_OPENCV"
            self.camera_preview_estado = "SIN_OPENCV"
            try:
                self.get_logger().warn("Detector PARE no iniciado: OpenCV/cv2 no está disponible.")
            except Exception:
                pass
            return
        self.camera_running = True
        self.camera_thread = threading.Thread(target=self._camera_pare_loop, daemon=True)
        self.camera_thread.start()
        self.pare_fuente = "CAMARA"
        self.get_logger().info("Detector PARE por cámara iniciado sin tocar la lógica LiDAR.")

    def _camera_pare_loop(self):
        idx = int(getattr(self.cfg, "camara_indice", 0))
        cap = None
        try:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                with self.camera_lock:
                    self.pare_fuente = "CAMARA_NO_ABRE"
                    self.camera_preview_estado = "CAMARA_NO_ABRE"
                self.get_logger().warn(f"No se pudo abrir cámara índice {idx}. La navegación LiDAR sigue normal.")
                return

            # Forzar conversión RGB/MJPG evita que algunas cámaras USB entreguen
            # frames YUYV/Bayer mal interpretados en VNC/Matplotlib.
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(getattr(self.cfg, "camara_fourcc", "MJPG"))))
            except Exception:
                pass
            try:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            except Exception:
                pass
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(getattr(self.cfg, "camara_width", 320)))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(getattr(self.cfg, "camara_height", 240)))
            cap.set(cv2.CAP_PROP_FPS, int(getattr(self.cfg, "camara_fps", 15)))
            salto = max(1, int(getattr(self.cfg, "pare_procesar_cada_n_frames", 1)))
            frame_i = 0
            while self.camera_running and not self.solicitud_salir:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.03)
                    continue
                frame_i += 1

                frame_bgr = self._normalizar_frame_camara_bgr(frame)
                if frame_bgr is None:
                    time.sleep(0.01)
                    continue

                # La cámara corre en su propio hilo: detecta PARE aunque el
                # robot esté avanzando, girando, escapando o "pensando".
                # El detector trabaja internamente en HSV, pero la interfaz
                # muestra SIEMPRE la imagen normal de la cámara.
                if frame_i % salto == 0:
                    self._procesar_frame_pare(frame_bgr)
                    self._procesar_frame_meta_verde(frame_bgr)
                self._guardar_preview_camara(frame_bgr)
                time.sleep(0.001)
        except Exception as e:
            with self.camera_lock:
                self.pare_fuente = "ERROR_CAMARA"
                self.camera_preview_estado = "ERROR_CAMARA"
            try:
                self.get_logger().warn(f"Detector PARE detenido por error: {e}")
            except Exception:
                pass
        finally:
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass

    def _normalizar_frame_camara_bgr(self, frame):
        """Devuelve un frame BGR normal para detección y vista previa.

        Algunas cámaras USB en Raspberry/VNC entregan YUYV, BGRA o un
        formato no contiguo. Si eso se manda directo a Matplotlib, la vista
        puede verse como máscara/imagen solarizada. Esta función normaliza
        el frame antes de procesar HSV o mostrarlo.
        """
        if cv2 is None or frame is None:
            return None
        try:
            arr = np.asarray(frame)
            if arr.size == 0:
                return None
            if arr.ndim == 2:
                return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            if arr.ndim == 3 and arr.shape[2] == 4:
                return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            if arr.ndim == 3 and arr.shape[2] == 2:
                return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)
            if arr.ndim == 3 and arr.shape[2] >= 3:
                bgr = arr[:, :, :3]
                if bool(getattr(self.cfg, "camara_swap_rb", False)):
                    bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
                return np.ascontiguousarray(bgr)
        except Exception:
            return None
        return None

    def _guardar_preview_camara(self, frame):
        """Guarda un frame anotado para verlo dentro de la interfaz.

        No usa cv2.imshow porque en VNC/Matplotlib es más estable mostrarlo
        dentro de la misma ventana. Este método no decide movimientos.
        """
        if cv2 is None or frame is None:
            return
        try:
            vis = frame.copy()
            h, w = vis.shape[:2]
            y0 = int(h * float(getattr(self.cfg, "pare_roi_y0", 0.05)))
            y1 = int(h * float(getattr(self.cfg, "pare_roi_y1", 0.88)))
            x0 = int(w * float(getattr(self.cfg, "pare_roi_x0", 0.08)))
            x1 = int(w * float(getattr(self.cfg, "pare_roi_x1", 0.92)))

            with self.camera_lock:
                bbox = self.pare_bbox
                detectado = bool(self.pare_detectado)
                activo = bool(self.pare_en_detencion_activa())
                area = float(self.pare_area_ratio)
                fuente = str(self.pare_fuente)
                pares_detectados = int(self.pare_detenciones)
                frames_ok = int(self.pare_frames_ok)
                meta_bbox = getattr(self, "meta_vision_bbox", None)
                meta_detectada = bool(getattr(self, "meta_vision_detectada", False))
                meta_activa = bool(getattr(self, "meta_vision_activa", False))
                meta_final = bool(getattr(self, "meta_vision_finalizada", False))
                meta_area = float(getattr(self, "meta_vision_area_ratio", 0.0))
                meta_count = int(getattr(self, "meta_vision_detectadas", 0))
                meta_err = float(getattr(self, "meta_vision_error_x", 0.0))

            # ROI azul: zona donde el detector busca la señal PARE.
            cv2.rectangle(vis, (max(0, x0), max(0, y0)), (min(w - 1, x1), min(h - 1, y1)), (255, 180, 0), 1)

            if bbox is not None:
                bx, by, bw, bh = bbox
                color = (0, 0, 255) if detectado or activo else (0, 180, 255)
                cv2.rectangle(vis, (int(bx), int(by)), (int(bx + bw), int(by + bh)), color, 2)

            if meta_bbox is not None:
                gx, gy, gw, gh = meta_bbox
                color_meta = (0, 255, 0) if (meta_detectada or meta_activa or meta_final) else (0, 160, 0)
                cv2.rectangle(vis, (int(gx), int(gy)), (int(gx + gw), int(gy + gh)), color_meta, 2)
                cv2.putText(vis, "META", (int(gx), max(14, int(gy) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color_meta, 2, cv2.LINE_AA)

            if activo:
                texto = f"PARE: DETENIDO 3s | PAREs detectados: {pares_detectados}"
                color_texto = (0, 0, 255)
            elif meta_final:
                texto = f"META ALCANZADA | verdes detectados: {meta_count}"
                color_texto = (0, 255, 0)
            elif meta_activa:
                texto = f"META: avanzando por vision err={meta_err:+.2f} | det={meta_count}"
                color_texto = (0, 255, 0)
            elif meta_detectada:
                texto = f"META candidata area={meta_area:.3f} | det={meta_count}"
                color_texto = (0, 255, 0)
            elif detectado:
                texto = f"PARE candidato {frames_ok}f area={area:.3f} | detectados: {pares_detectados}"
                color_texto = (0, 0, 255)
            else:
                texto = f"Camara {fuente} | PAREs detectados: {pares_detectados} | META det={meta_count}"
                color_texto = (0, 255, 255)

            cv2.rectangle(vis, (0, 0), (w, 24), (0, 0, 0), -1)
            cv2.putText(vis, texto, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color_texto, 1, cv2.LINE_AA)

            # Aviso de meta conocida: se dibuja en la vista de cámara para que
            # puedas saber cuándo el robot cree estar llegando a la META.
            try:
                meta_on = bool(getattr(self, "meta_cree_llegar", False)) or bool(getattr(self, "meta_vision_finalizada", False))
                meta_txt = self.estado_meta_texto()
                if meta_on:
                    cv2.rectangle(vis, (0, max(25, h - 34)), (w, h), (0, 110, 0), -1)
                    cv2.putText(vis, "META ALCANZADA", (8, h - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
                elif bool(getattr(self.cfg, "meta_mostrar_distancia", True)):
                    cv2.putText(vis, meta_txt, (8, min(h - 8, 42)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)
            except Exception:
                pass

            # Matplotlib espera RGB.
            rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            with self.camera_lock:
                self.camera_frame_rgb = rgb
                self.camera_frame_t = time.time()
                self.camera_preview_estado = texto
        except Exception:
            # La vista de cámara no debe afectar al robot.
            pass

    def _procesar_frame_pare(self, frame):
        """Detecta rojo en HSV + forma/área para confirmar una señal PARE.

        Regla contra falsos positivos: debe haber rojo persistente, área mínima,
        contorno con forma razonable y no estar en cooldown tras una detención.
        """
        if cv2 is None:
            return
        ahora = time.time()
        if ahora < self.pare_cooldown_hasta:
            return

        h, w = frame.shape[:2]
        # Zona de atención: parte central/superior de la imagen. Así reducimos
        # falsos rojos del piso o de cables/laterales.
        y0 = int(h * float(getattr(self.cfg, "pare_roi_y0", 0.05)))
        y1 = int(h * float(getattr(self.cfg, "pare_roi_y1", 0.88)))
        x0 = int(w * float(getattr(self.cfg, "pare_roi_x0", 0.08)))
        x1 = int(w * float(getattr(self.cfg, "pare_roi_x1", 0.92)))
        roi = frame[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # Rojo cruza el 0° en HSV, por eso se usan dos rangos.
        lower1 = np.array([int(getattr(self.cfg, "pare_h1_min", 0)),
                           int(getattr(self.cfg, "pare_s_min", 80)),
                           int(getattr(self.cfg, "pare_v_min", 70))], dtype=np.uint8)
        upper1 = np.array([int(getattr(self.cfg, "pare_h1_max", 12)), 255, 255], dtype=np.uint8)
        lower2 = np.array([int(getattr(self.cfg, "pare_h2_min", 165)),
                           int(getattr(self.cfg, "pare_s_min", 80)),
                           int(getattr(self.cfg, "pare_v_min", 70))], dtype=np.uint8)
        upper2 = np.array([int(getattr(self.cfg, "pare_h2_max", 179)), 255, 255], dtype=np.uint8)

        mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_roi = float(roi.shape[0] * roi.shape[1])
        mejor = None
        mejor_score = 0.0
        for c in contornos:
            area = float(cv2.contourArea(c))
            if area < float(getattr(self.cfg, "pare_area_min_px", 160)):
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw <= 0 or bh <= 0:
                continue
            ratio = bw / float(bh)
            if ratio < float(getattr(self.cfg, "pare_ratio_min", 0.55)) or ratio > float(getattr(self.cfg, "pare_ratio_max", 1.80)):
                continue
            area_ratio = area / max(1.0, area_roi)
            if area_ratio < float(getattr(self.cfg, "pare_area_ratio_min", 0.0022)):
                continue
            if area_ratio > float(getattr(self.cfg, "pare_area_ratio_max", 0.045)):
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.045 * peri, True) if peri > 0 else c
            lados = len(approx)
            # Una señal PARE puede verse como octágono, pero por perspectiva
            # aceptamos 5..10 lados. Si no cumple, igual puede pasar con mayor área.
            forma_ok = 5 <= lados <= 10 or area_ratio > float(getattr(self.cfg, "pare_area_ratio_fuerte", 0.009))
            if not forma_ok:
                continue
            score = area_ratio * 100.0 + min(10.0, area / 250.0)
            if score > mejor_score:
                mejor_score = score
                mejor = (x + x0, y + y0, bw, bh, area_ratio, lados)

        detectado = mejor is not None
        with self.camera_lock:
            if detectado:
                self.pare_frames_ok += 1
                self.pare_frames_miss = 0
                self.pare_detectado = True
                self.pare_bbox = mejor[:4]
                self.pare_area_ratio = float(mejor[4])
                self.pare_ultimo_t = ahora
            else:
                self.pare_frames_miss += 1
                if self.pare_frames_miss >= int(getattr(self.cfg, "pare_frames_perdida", 4)):
                    self.pare_frames_ok = 0
                    self.pare_detectado = False
                    self.pare_bbox = None
                    self.pare_area_ratio = 0.0

            if self.pare_frames_ok >= int(getattr(self.cfg, "pare_frames_confirmacion", 3)):
                self.pare_activo_hasta = max(self.pare_activo_hasta, ahora + float(getattr(self.cfg, "pare_stop_seg", 3.0)))
                self.pare_cooldown_hasta = ahora + float(getattr(self.cfg, "pare_cooldown_seg", 6.0))
                self.pare_frames_ok = 0
                self.pare_detenciones += 1
                # Guardado pasivo: ubicación aproximada del PARE detectado.
                # No altera velocidad, estados ni navegación.
                try:
                    px, py, pyaw, fuente = self.pose_para_mapa()
                    self.pare_eventos.append({
                        "t": ahora,
                        "x": float(px),
                        "y": float(py),
                        "yaw": float(pyaw),
                        "fuente": str(fuente),
                        "area_ratio": float(self.pare_area_ratio),
                    })
                except Exception:
                    pass
                self.get_logger().warn("PARE detectado por cámara: detención obligatoria de 3 s.")
                # Freno inmediato desde el hilo de cámara. No cambia la ruta ni
                # agrega estados de prealerta; solo evita que el robot se pase
                # una señal detectada mientras está en marcha o girando.
                try:
                    if self.robot_habilitado and not self.robot_pausado and not self.solicitud_salir:
                        self.detener_robot()
                except Exception:
                    pass

    def _procesar_frame_meta_verde(self, frame):
        """Detecta cartel verde de META.

        Diferencia con la meta por coordenada: esta usa la cámara. Cuando se
        confirma, no declara llegada de inmediato: primero fuerza un avance
        corto alineado por el centro del cartel, para entrar al área de meta.
        """
        if cv2 is None or frame is None:
            return
        if not bool(getattr(self.cfg, "usar_camara_meta_verde", True)):
            return
        if bool(getattr(self, "meta_vision_finalizada", False)):
            return

        ahora = time.time()
        h, w = frame.shape[:2]
        y0 = int(h * float(getattr(self.cfg, "meta_roi_y0", 0.02)))
        y1 = int(h * float(getattr(self.cfg, "meta_roi_y1", 0.78)))
        x0 = int(w * float(getattr(self.cfg, "meta_roi_x0", 0.04)))
        x1 = int(w * float(getattr(self.cfg, "meta_roi_x1", 0.96)))
        roi = frame[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([int(getattr(self.cfg, "meta_h_min", 35)),
                          int(getattr(self.cfg, "meta_s_min", 65)),
                          int(getattr(self.cfg, "meta_v_min", 65))], dtype=np.uint8)
        upper = np.array([int(getattr(self.cfg, "meta_h_max", 92)), 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_roi = float(roi.shape[0] * roi.shape[1])
        mejor = None
        mejor_score = 0.0
        for c in contornos:
            area = float(cv2.contourArea(c))
            if area < float(getattr(self.cfg, "meta_area_min_px", 240)):
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bw <= 0 or bh <= 0:
                continue
            ratio = bw / float(bh)
            if ratio < float(getattr(self.cfg, "meta_ratio_min", 1.05)) or ratio > float(getattr(self.cfg, "meta_ratio_max", 5.5)):
                continue
            area_ratio = area / max(1.0, area_roi)
            if area_ratio < float(getattr(self.cfg, "meta_area_ratio_min", 0.0030)):
                continue
            if area_ratio > float(getattr(self.cfg, "meta_area_ratio_max", 0.22)):
                continue
            # Preferimos carteles horizontales/anchos y ubicados arriba/centro.
            score = area_ratio * 100.0 + min(5.0, ratio)
            if score > mejor_score:
                mejor_score = score
                mejor = (x + x0, y + y0, bw, bh, area_ratio)

        detectado = mejor is not None
        with self.camera_lock:
            if detectado:
                gx, gy, gw, gh, ar = mejor
                cx = gx + gw / 2.0
                err_x = (cx - (w / 2.0)) / max(1.0, (w / 2.0))
                self.meta_vision_frames_ok += 1
                self.meta_vision_frames_miss = 0
                self.meta_vision_detectada = True
                self.meta_vision_bbox = (int(gx), int(gy), int(gw), int(gh))
                self.meta_vision_area_ratio = float(ar)
                self.meta_vision_error_x = float(max(-1.0, min(1.0, err_x)))
                self.meta_vision_ultimo_t = ahora
            else:
                self.meta_vision_frames_miss += 1
                if self.meta_vision_frames_miss >= int(getattr(self.cfg, "meta_frames_perdida", 5)):
                    self.meta_vision_frames_ok = 0
                    self.meta_vision_detectada = False
                    self.meta_vision_bbox = None
                    self.meta_vision_area_ratio = 0.0
                    self.meta_vision_error_x = 0.0

            if (not self.meta_vision_activa
                    and not self.meta_vision_finalizada
                    and self.meta_vision_frames_ok >= int(getattr(self.cfg, "meta_frames_confirmacion", 2))):
                self.meta_vision_activa = True
                self.meta_vision_avanzar_hasta = ahora + float(getattr(self.cfg, "meta_avance_seg", 1.15))
                self.meta_vision_detectadas += 1
                self.meta_cree_llegar = True
                try:
                    px, py, pyaw, fuente = self.pose_para_mapa()
                    self.meta_eventos.append({
                        "t": float(ahora),
                        "x": float(px),
                        "y": float(py),
                        "yaw": float(pyaw),
                        "fuente": str(fuente),
                        "modo": "camara_verde",
                        "area_ratio": float(self.meta_vision_area_ratio),
                        "error_x": float(self.meta_vision_error_x),
                    })
                except Exception:
                    pass
                try:
                    self.get_logger().warn("META verde detectada: alineando y avanzando por visión.")
                except Exception:
                    pass

    def aplicar_meta_si_corresponde(self):
        """Prioridad de llegada: si la cámara vio META, alinea y avanza corto.

        No se declara llegada apenas ve verde. Primero avanza un poco guiado
        por el error horizontal del cartel para realmente entrar a la meta.
        """
        if not bool(getattr(self.cfg, "usar_camara_meta_verde", True)):
            return False
        ahora = time.time()
        if bool(getattr(self, "meta_vision_finalizada", False)):
            self.cambiar_estado("META_ALCANZADA")
            self.detener_robot()
            return True
        if not bool(getattr(self, "meta_vision_activa", False)):
            return False

        if ahora < float(getattr(self, "meta_vision_avanzar_hasta", 0.0)):
            err = float(getattr(self, "meta_vision_error_x", 0.0))
            kp = float(getattr(self.cfg, "meta_kp_vision", 0.42))
            wmax = float(getattr(self.cfg, "meta_w_max", 0.28))
            # err>0 significa cartel a la derecha; angular.z negativo gira a la derecha.
            w = max(-wmax, min(wmax, -kp * err))
            v = float(getattr(self.cfg, "meta_vel_avance", 0.065))
            # Seguridad: si el frente está demasiado cerca, no empujar.
            if math.isfinite(self.dist_frente) and self.dist_frente < float(getattr(self.cfg, "meta_frente_stop", 0.22)):
                v = 0.0
            self.cambiar_estado("META_AVANZAR_VISION")
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.vel_lineal = v
            self.vel_angular = w
            self.publisher.publish(cmd)
            return True

        self.meta_vision_activa = False
        self.meta_vision_finalizada = True
        self.meta_cree_llegar = True
        self.meta_confirmada_t = ahora
        self.t_meta_alcanzada = ahora
        self.cambiar_estado("META_ALCANZADA")
        self.detener_robot()
        try:
            self.get_logger().warn("META alcanzada por cámara verde: robot detenido.")
        except Exception:
            pass
        return True

    def pare_en_detencion_activa(self):
        return bool(getattr(self.cfg, "usar_camara_pare", True)) and time.time() < self.pare_activo_hasta

    def aplicar_parada_pare_si_corresponde(self):
        """Prioridad máxima: cámara PARE manda sobre LiDAR y navegación."""
        if self.pare_en_detencion_activa():
            if self.estado_actual != "PARAR_PARE":
                self.pare_estado_previo = self.estado_actual
                self.cambiar_estado("PARAR_PARE")
            self.detener_robot()
            return True

        if self.estado_actual == "PARAR_PARE":
            # Terminó la espera de 3 s: se cuenta como PARE respetado.
            try:
                self.pare_respetados += 1
            except Exception:
                self.pare_respetados = 1
            # Terminó la espera de 3 s. Para no quedarse detenido frente al
            # mismo PARE ni volver a detectarlo de inmediato, fuerza una salida
            # corta y bloqueada antes de volver a analizar el laberinto.
            previo = self.pare_estado_previo or "LAB_AVANZAR"
            self.pare_estado_previo = None
            self.pare_detectado = False
            self.pare_frames_ok = 0
            self.pare_frames_miss = 0
            self.pare_cooldown_hasta = max(
                self.pare_cooldown_hasta,
                time.time() + float(getattr(self.cfg, "pare_cooldown_seg", 10.0))
            )
            if getattr(self.cfg, "usar_modo_laberinto", False):
                self.lab_exit_t0 = time.time()
                self.cambiar_estado("LAB_SALIDA_GIRO")
            else:
                self.cambiar_estado(previo)
        return False

    # ==========================================================
    # SENSORES
    # ==========================================================
    def battery_callback(self, msg):
        """Actualiza indicador de batería desde sensor_msgs/BatteryState.

        Corrección importante:
        - percentage en ROS suele venir entre 0.0 y 1.0.
        - si no hay percentage válido, se estima por voltaje.
        - ya no hay botones ni simulación manual de batería.
        """
        self.bateria_real_recibida = True

        volt = float(msg.voltage) if math.isfinite(float(msg.voltage)) and msg.voltage > 0.0 else float('nan')
        self.voltaje_bateria = volt

        pct = None
        try:
            raw_pct = float(msg.percentage)
            if math.isfinite(raw_pct) and raw_pct >= 0.0:
                pct = raw_pct * 100.0 if raw_pct <= 1.0 else raw_pct
        except Exception:
            pct = None

        if pct is None and math.isfinite(volt):
            pct = ((volt - self.cfg.bateria_voltaje_min) /
                   (self.cfg.bateria_voltaje_max - self.cfg.bateria_voltaje_min)) * 100.0
            self.bateria_fuente = "EST. VOLTAJE"
        else:
            self.bateria_fuente = "REAL"

        if pct is not None:
            self.porcentaje_bateria = int(max(0, min(100, round(pct))))

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = self.quaternion_a_yaw(q.x, q.y, q.z, q.w)
        self.tengo_odom = True
        self.mapa_ruta.actualizar_pose(self.odom_x, self.odom_y, self.odom_yaw, source="/odom")

    def quaternion_a_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ==========================================================
    # CALLBACK PRINCIPAL DEL LiDAR
    # ==========================================================
    def lidar_callback(self, msg):
        t0 = time.perf_counter()
        res = procesar_escaneo_lidar(msg)
        self.datos_filtrados = res

        self.dist_frente = res['dist_frente']
        self.dist_izq = res['dist_izq']
        self.dist_der = res['dist_der']
        self.dist_diag_der = res['dist_diag_der']
        self.dist_diag_izq = res['dist_diag_izq']
        self.dist_der_pared = res['dist_der_pared']
        self.dist_izq_pared = res['dist_izq_pared']
        self.pendiente_pared_der = res['pared_der_pendiente']
        self.pendiente_pared_izq = res['pared_izq_pendiente']
        self.pared_der_valida = res['pared_der_valida']
        self.pared_izq_valida = res['pared_izq_valida']
        self.puntos_pared_der = res['puntos_pared_der']
        self.puntos_pared_izq = res['puntos_pared_izq']
        self.ancho_pasillo = res['ancho_pasillo']
        self.error_centro_actual = res['error_centro']
        self.tipo_frente = res.get('tipo_frente', 'INDEFINIDO')
        self.frente_es_pasillo = bool(res.get('frente_es_pasillo', False))
        self.frente_es_esquina = bool(res.get('frente_es_esquina', False))
        self.frente_conecta_der = bool(res.get('frente_conecta_der', False))
        self.frente_conecta_izq = bool(res.get('frente_conecta_izq', False))
        self.frente_conecta_ambos = bool(res.get('frente_conecta_ambos', False))
        self.frente_confianza = float(res.get('frente_confianza', 0.0))
        self.front_class_sm = res.get('front_class_sm', 'NONE')
        self.front_ang_width = float(res.get('front_ang_width', 0.0))
        self.front_dist_sm = float(res.get('front_dist_sm', self.dist_frente))
        self.alpha_pared_der = float(res.get('pared_der_alpha', self.pendiente_pared_der))
        self.alpha_pared_izq = float(res.get('pared_izq_alpha', self.pendiente_pared_izq))
        self.len_pared_der = float(res.get('pared_der_len', 0.0))
        self.len_pared_izq = float(res.get('pared_izq_len', 0.0))
        self.total_puntos = res['num_puntos']
        self.t_proc_actual = res['tiempo_proc_ms']
        self.tiempos_proc.append(self.t_proc_actual)

        ahora_ref = time.time()
        if self.pared_der_valida:
            self.t_ultima_pared_der = ahora_ref
        if self.pared_izq_valida:
            self.t_ultima_pared_izq = ahora_ref
        self.actualizar_referencia_estable()

        # Mapa simple: usa /odom si se mueve; si /odom está en cero o no llega,
        # usa la trayectoria estimada por cmd_vel. Esto permite dibujar paredes
        # con LiDAR 360° aunque el tópico /odom no funcione.
        self.actualizar_mapa_lidar(res)
        self.registrar_colision_estimada()

        ahora = time.time()
        if self.ultimo_scan_time is not None:
            dt_scan = ahora - self.ultimo_scan_time
            if dt_scan > 0:
                self.tiempos_loop.append(dt_scan)
        self.ultimo_scan_time = ahora

        # Estado inicial o pausa: interfaz activa, motores quietos.
        if not self.robot_habilitado:
            self.estado_actual = "ESPERANDO INICIO"
            self.detener_robot()
            return

        if self.robot_pausado:
            self.detener_robot()
            return

        # Fusión cámara + LiDAR: si la cámara confirma PARE, manda la cámara.
        # Se detiene 3 s sin modificar la lógica de navegación ya lograda.
        if self.aplicar_parada_pare_si_corresponde():
            self.registrar_historial_control()
            return

        # META verde por cámara: una vez confirmada, alinea por visión,
        # avanza corto dentro del área de meta y detiene el robot.
        if self.aplicar_meta_si_corresponde():
            self.registrar_historial_control()
            return

        cmd = self.calcular_comando()
        self.vel_lineal = cmd.linear.x
        self.vel_angular = cmd.angular.z
        self.publisher.publish(cmd)
        self.registrar_historial_control()

        t_loop_ms = (time.perf_counter() - t0) * 1000.0
        self.tiempos_proc.append(max(self.t_proc_actual, t_loop_ms))


    # ==========================================================
    # MAPA SIMPLE + MODO ASPIRADORA
    # ==========================================================
    def odom_parece_util(self):
        """True si /odom realmente se mueve; si está fijo en 0, usar cmd_vel estimado."""
        if not self.tengo_odom:
            return False
        if abs(self.odom_x) > 0.025 or abs(self.odom_y) > 0.025:
            return True
        try:
            return self.mapa_ruta.total_distance > 0.05
        except Exception:
            return False

    def _map_snap_yaw(self, yaw):
        """Ajusta el yaw estimado a direcciones de laberinto (0/90/180/270).

        Como la Raspberry no está entregando /odom real, el mapa se arma con
        cmd_vel. Si durante los giros integramos yaw continuo, las paredes se
        dibujan como círculos. Para un circuito tipo laberinto, las paredes son
        principalmente rectas; por eso el mapa usa un yaw cuantizado SOLO para
        dibujar el plano. Esto no cambia la lógica de movimiento.
        """
        try:
            paso = math.radians(float(getattr(self.cfg, "mapa_snap_yaw_deg", 90.0)))
            if paso <= 0:
                return yaw
            return round(float(yaw) / paso) * paso
        except Exception:
            return yaw

    def pose_para_mapa(self):
        if self.odom_parece_util():
            return float(self.odom_x), float(self.odom_y), float(self.odom_yaw), "/odom"
        yaw = float(self.plot_yaw)
        fuente = "cmd_vel"
        if bool(getattr(self.cfg, "mapa_snap_yaw_cmd_vel", True)):
            yaw = self._map_snap_yaw(yaw)
            fuente = "cmd_vel_manhattan"
        return float(self.plot_x), float(self.plot_y), yaw, fuente

    def evaluar_meta_conocida(self):
        """Evalúa si el robot cree estar cerca de la META oficial.

        El reto define INICIO en la esquina inferior izquierda y META en la
        esquina superior derecha de una pista de 3.60 x 2.40 m. Esta función
        usa la pose estimada del mapa/cmd_vel para generar un aviso visual.
        No altera estados, velocidades ni decisiones del robot.
        """
        try:
            if not bool(getattr(self.cfg, "meta_avisar_en_frame", True)):
                return False
            x, y, yaw, fuente = self.pose_para_mapa()
            mx = float(getattr(self.cfg, "meta_x_m", self.meta_x))
            my = float(getattr(self.cfg, "meta_y_m", self.meta_y))
            r = float(getattr(self.cfg, "meta_radio_m", self.meta_radio))
            d = math.hypot(float(x) - mx, float(y) - my)
            self.meta_distancia = d
            ok_dist = d <= r
            ok_recorrido = True
            try:
                ok_recorrido = float(self.mapa_ruta.total_distance) >= float(getattr(self.cfg, "meta_min_recorrido_m", 1.20))
            except Exception:
                pass
            nuevo_estado = bool(ok_dist and ok_recorrido)
            if nuevo_estado and not self.meta_cree_llegar:
                self.meta_confirmada_t = time.time()
                self.meta_eventos.append({
                    "t": float(self.meta_confirmada_t),
                    "x": float(x),
                    "y": float(y),
                    "yaw": float(yaw),
                    "meta_x": mx,
                    "meta_y": my,
                    "distancia_m": float(d),
                    "fuente": str(fuente),
                })
                try:
                    self.get_logger().warn(f"META probable: robot cerca de ({mx:.2f}, {my:.2f}), d={d:.2f} m")
                except Exception:
                    pass
            self.meta_cree_llegar = nuevo_estado
            return self.meta_cree_llegar
        except Exception:
            return False

    def estado_meta_texto(self):
        try:
            if bool(getattr(self, "meta_vision_finalizada", False)):
                return "META ALCANZADA camara"
            if bool(getattr(self, "meta_vision_activa", False)):
                restante = max(0.0, float(getattr(self, "meta_vision_avanzar_hasta", 0.0)) - time.time())
                return f"META verde avanzando {restante:.1f}s"
            if bool(getattr(self, "meta_vision_detectada", False)):
                return f"META verde candidata {self.meta_vision_frames_ok}f"
            if self.meta_cree_llegar:
                return f"META PROBABLE d={self.meta_distancia:.2f}m"
            return f"meta d={self.meta_distancia:.2f}m" if math.isfinite(self.meta_distancia) else "meta pendiente"
        except Exception:
            return "meta pendiente"

    def actualizar_mapa_lidar(self, res):
        """Actualiza ruta + paredes con LiDAR 360° sin usarlo para mandar el control.

        Versión N: para que el dibujo se parezca a un plano de laberinto, el
        mapa SOLO agrega paredes cuando el robot avanza. Durante giros/escape
        solo actualiza la pose/ruta, pero NO acumula puntos LiDAR. Eso elimina
        los círculos marrones que veías en el reporte.
        """
        if not (getattr(self.cfg, "lab_mapa_lidar_360", False) or getattr(self.cfg, "aspiradora_mapa_lidar_360", True)):
            return
        try:
            mx, my, myaw, fuente = self.pose_para_mapa()
            self.mapa_ruta.actualizar_pose(mx, my, myaw, source=fuente)
            self.evaluar_meta_conocida()

            # No registrar paredes si está rotando o escapando: eso genera
            # mapas circulares, porque el LiDAR barre la misma zona desde casi
            # el mismo punto con muchos yaw diferentes.
            if bool(getattr(self.cfg, "mapa_solo_lidar_en_avance", True)):
                estado = str(self.estado_actual).upper()
                estados_sin_pared = ("GIRAR", "ESCAPE", "RETRO", "BACKTRACK", "PAUSA", "ESPERANDO")
                giro_alto = abs(float(getattr(self, "vel_angular", 0.0))) > float(getattr(self.cfg, "mapa_w_max_para_guardar", 0.16))
                avance_bajo = abs(float(getattr(self, "vel_lineal", 0.0))) < float(getattr(self.cfg, "mapa_v_min_para_guardar", 0.018))
                estado_giro = any(k in estado for k in estados_sin_pared)
                if estado_giro or giro_alto or avance_bajo:
                    self.plot_source = fuente + " / solo ruta"
                    return

            self.mapa_ruta.actualizar_lidar(
                res,
                mx,
                my,
                myaw,
                pared_der_valida=True,
                pared_izq_valida=True,
            )
            self.plot_source = fuente
        except Exception as e:
            try:
                self.get_logger().warn(f"Mapa LiDAR no actualizado: {e}")
            except Exception:
                pass

    def _asp_elegir_lado_libre(self):
        """Elige lado para girar. +1 = izquierda, -1 = derecha."""
        izq = self._distancia_lateral_segura("izquierda")
        der = self._distancia_lateral_segura("derecha")
        diag_izq = self.dist_diag_izq if math.isfinite(self.dist_diag_izq) else izq
        diag_der = self.dist_diag_der if math.isfinite(self.dist_diag_der) else der
        # Promedio ponderado: las diagonales importan para curvas.
        score_izq = 0.55 * izq + 0.45 * diag_izq
        score_der = 0.55 * der + 0.45 * diag_der
        if not math.isfinite(score_izq):
            score_izq = -1.0
        if not math.isfinite(score_der):
            score_der = -1.0
        if abs(score_izq - score_der) < 0.05:
            return float(self.cfg.aspiradora_default_turn)
        return 1.0 if score_izq > score_der else -1.0

    def _asp_iniciar_giro(self, motivo="frente"):
        ahora = time.time()
        # Bloqueo de decisión: si ya eligió un lado hace poco, no cambiarlo.
        if (ahora - self.asp_decision_t0) > self.cfg.aspiradora_decision_lock_seg:
            self.asp_turn_dir = self._asp_elegir_lado_libre()
            self.asp_decision_t0 = ahora
        self.asp_turn_t0 = ahora
        self.asp_estado_anterior = self.estado_actual
        self.cambiar_estado("ASPIRADORA_GIRAR")
        self.get_logger().warn(f"Aspiradora: giro bloqueado {self.asp_turn_dir:+.0f} por {motivo}")

    def _asp_control_pared(self, lado):
        """Seguimiento lateral simple, sin activar esquinas viejas ni DFS."""
        ahora = time.time()
        dt = max(0.01, ahora - self.asp_t_prev)
        target = self.cfg.aspiradora_dist_pared

        if lado == "izquierda":
            dist = self.dist_izq_pared if self.pared_izq_valida else self.dist_izq
            alpha = self.alpha_pared_izq if self.pared_izq_valida else 0.0
            # lejos de izquierda -> girar izquierda; cerca -> girar derecha
            err = dist - target if math.isfinite(dist) else 0.0
            w = (self.cfg.aspiradora_kp_pared * err
                 + self.cfg.aspiradora_kd_pared * ((err - self.asp_error_prev) / dt)
                 - self.cfg.aspiradora_kp_alpha * alpha)
        else:
            dist = self.dist_der_pared if self.pared_der_valida else self.dist_der
            alpha = self.alpha_pared_der if self.pared_der_valida else 0.0
            # lejos de derecha -> girar derecha; cerca -> girar izquierda
            err = dist - target if math.isfinite(dist) else 0.0
            w = (-self.cfg.aspiradora_kp_pared * err
                 - self.cfg.aspiradora_kd_pared * ((err - self.asp_error_prev) / dt)
                 - self.cfg.aspiradora_kp_alpha * alpha)

        self.asp_error_prev = err
        self.asp_t_prev = ahora
        w = self.saturar(w, self.cfg.aspiradora_w_max)
        v = self.cfg.aspiradora_vel_avance
        if self.dist_frente < self.cfg.frente_freno_suave:
            v = min(v, self.cfg.aspiradora_vel_lenta)
        self.error_centro_actual = err
        self.error_ang_actual = alpha
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def _asp_control_gap_suave(self):
        """Avance libre: apunta al hueco frontal sin perder tiempo pensando."""
        target_ang, width, clearance = self._calcular_gap_frontal()
        # positivo target = hueco a derecha; angular positivo gira a izquierda.
        w = -self.cfg.aspiradora_kp_gap * target_ang
        w = self.saturar(w, self.cfg.aspiradora_w_max * 0.80)
        v = self.cfg.aspiradora_vel_avance
        if self.dist_frente < self.cfg.frente_freno_suave:
            v = min(v, self.cfg.aspiradora_vel_lenta)
        self.gap_target_deg = math.degrees(target_ang)
        self.gap_width_deg = math.degrees(width)
        self.gap_clearance = clearance
        self.error_centro_actual = target_ang
        self.error_ang_actual = 0.0
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def control_aspiradora(self):
        """Navegación tipo aspiradora.

        Prioridades:
        1) Anti-choque global siempre se aplica en cmd_vel.
        2) Si el frente se cierra, detiene avance y gira con una decisión bloqueada.
        3) Si hay pared izquierda útil, la sigue a distancia prudente.
        4) Si no hay izquierda, usa derecha como respaldo.
        5) Si no hay pared, avanza hacia hueco libre frontal.
        """
        frente = self.dist_frente if math.isfinite(self.dist_frente) else float('inf')
        der = self._distancia_lateral_segura("derecha")
        izq = self._distancia_lateral_segura("izquierda")

        # Atasco fuerte: si está encerrado, retrocede poquito y luego gira.
        if self.estado_actual == "ASPIRADORA_RETROCEDER":
            if (time.time() - self.asp_backoff_t0) < self.cfg.aspiradora_backoff_seg:
                return self.cmd_vel(self.cfg.aspiradora_vel_retroceso, 0.0)
            self._asp_iniciar_giro("salida de retroceso")
            return self.cmd_vel(0.0, self.cfg.signo_giro * self.asp_turn_dir * self.cfg.aspiradora_w_giro)

        # Giro bloqueado: no cambia izquierda/derecha en cada frame.
        if self.estado_actual == "ASPIRADORA_GIRAR":
            dt = time.time() - self.asp_turn_t0
            frente_libre = frente > self.cfg.aspiradora_frente_salida
            min_ok = dt >= self.cfg.aspiradora_turn_min_seg
            max_ok = dt >= self.cfg.aspiradora_turn_max_seg
            if (min_ok and frente_libre) or max_ok:
                self.cambiar_estado("ASPIRADORA_AVANZAR")
                return self.control_aspiradora()
            return self.cmd_vel(self.cfg.aspiradora_vel_escape,
                                self.cfg.signo_giro * self.asp_turn_dir * self.cfg.aspiradora_w_giro)

        # Si el frente está cerrado, no avanzar: elegir y bloquear giro.
        if frente <= self.cfg.frente_emergencia and der <= self.cfg.lateral_stop_global and izq <= self.cfg.lateral_stop_global:
            self.asp_backoff_t0 = time.time()
            self.cambiar_estado("ASPIRADORA_RETROCEDER")
            return self.cmd_vel(self.cfg.aspiradora_vel_retroceso, 0.0)

        if frente <= self.cfg.aspiradora_frente_giro or (self.front_class_sm == "CORNER" and frente < self.cfg.frente_esquina_detect):
            self._asp_iniciar_giro("frente cerrado")
            return self.cmd_vel(self.cfg.aspiradora_vel_escape,
                                self.cfg.signo_giro * self.asp_turn_dir * self.cfg.aspiradora_w_giro)

        # Seguridad lateral preventiva: si roza, alejar sin detenerse totalmente.
        if der < self.cfg.aspiradora_lateral_seguro and der < izq:
            self.cambiar_estado("ASPIRADORA_AVANZAR")
            return self.cmd_vel(self.cfg.aspiradora_vel_lenta, abs(self.cfg.w_escape_lateral))
        if izq < self.cfg.aspiradora_lateral_seguro and izq < der:
            self.cambiar_estado("ASPIRADORA_AVANZAR")
            return self.cmd_vel(self.cfg.aspiradora_vel_lenta, -abs(self.cfg.w_escape_lateral))

        # Preferencia: seguir pared izquierda si existe. Eso da un comportamiento
        # parecido a aspiradora y a la ruta verde que dibujaste.
        prefer = str(self.cfg.aspiradora_preferir_pared).lower()
        if prefer == "izquierda" and (self.pared_izq_valida or math.isfinite(self.dist_izq)) and self.dist_izq < 0.75:
            self.cambiar_estado("ASPIRADORA_SEGUIR_PARED")
            return self._asp_control_pared("izquierda")
        if prefer == "derecha" and (self.pared_der_valida or math.isfinite(self.dist_der)) and self.dist_der < 0.75:
            self.cambiar_estado("ASPIRADORA_SEGUIR_PARED")
            return self._asp_control_pared("derecha")

        # Respaldo si aparece solo la otra pared.
        if self.pared_izq_valida and self.dist_izq < 0.75:
            self.cambiar_estado("ASPIRADORA_SEGUIR_PARED")
            return self._asp_control_pared("izquierda")
        if self.pared_der_valida and self.dist_der < 0.75:
            self.cambiar_estado("ASPIRADORA_SEGUIR_PARED")
            return self._asp_control_pared("derecha")

        self.cambiar_estado("ASPIRADORA_AVANZAR")
        return self._asp_control_gap_suave()

    # ==========================================================
    # MODO LABERINTO / MICROMOUSE SIMPLE
    # ==========================================================
    def _lab_dist(self, lado):
        return self._distancia_lateral_segura(lado)

    def _lab_pose_actual(self):
        """Pose usada para marcar decisiones en el mapa."""
        if self.odom_parece_util():
            return float(self.odom_x), float(self.odom_y), float(self.odom_yaw)
        return float(self.plot_x), float(self.plot_y), float(self.plot_yaw)

    def _lab_marcar_interseccion(self, tipo="interseccion"):
        """Marca intersecciones con persistencia espacial/temporal.

        Antes se guardaban demasiados eventos en pocos metros y eso hacía que
        el robot creyera que cada pequeño ruido del LiDAR era una decisión nueva.
        Ahora solo se marca si pasó suficiente tiempo o distancia desde la última.
        """
        if not getattr(self.cfg, "lab_marcar_intersecciones", True):
            return
        try:
            x, y, yaw = self._lab_pose_actual()
            ahora = time.time()
            if self.lab_intersecciones:
                lx, ly, _, lt = self.lab_intersecciones[-1]
                if (ahora - lt) < self.cfg.lab_evento_min_seg:
                    return
                if math.hypot(x - lx, y - ly) < self.cfg.lab_evento_min_dist:
                    return
            self.lab_intersecciones.append((x, y, tipo, ahora))
            self.lab_last_intersection_t = ahora
        except Exception:
            pass

    def _lab_marcar_callejon(self):
        """Marca callejón sin salida evitando duplicados por ruido."""
        try:
            x, y, yaw = self._lab_pose_actual()
            ahora = time.time()
            if self.lab_callejones:
                lx, ly, lt = self.lab_callejones[-1]
                if (ahora - lt) < self.cfg.lab_evento_min_seg:
                    return
                if math.hypot(x - lx, y - ly) < self.cfg.lab_evento_min_dist:
                    return
            self.lab_callejones.append((x, y, ahora))
            self.lab_last_deadend_t = ahora
        except Exception:
            pass

    def _lab_caminos_disponibles(self):
        """Evalúa caminos locales tipo micromouse: frente, izquierda, derecha.

        Retorna diccionario con True/False. No usa atrás para avanzar normal;
        atrás queda reservado para backtracking cuando no hay salida.
        """
        frente = self.dist_frente if math.isfinite(self.dist_frente) else float('inf')
        izq = self._lab_dist("izquierda")
        der = self._lab_dist("derecha")
        diag_izq = self.dist_diag_izq if math.isfinite(self.dist_diag_izq) else izq
        diag_der = self.dist_diag_der if math.isfinite(self.dist_diag_der) else der
        return {
            "frente": frente > self.cfg.lab_frente_giro,
            "izquierda": max(izq, diag_izq) > self.cfg.lab_izq_libre_para_girar,
            "derecha": max(der, diag_der) > self.cfg.lab_der_libre_para_girar,
            "frente_dist": frente,
            "izq_score": max(izq, diag_izq),
            "der_score": max(der, diag_der),
        }

    def _lab_total_distancia(self):
        try:
            return float(self.mapa_ruta.total_distance)
        except Exception:
            return 0.0

    def _lab_actualizar_progreso(self):
        """Actualiza contador de progreso usando el mapa/cmd_vel estimado.

        Si el robot gira mucho o se queda casi en el mismo punto por varios
        segundos, el modo escape toma el control.
        """
        total = self._lab_total_distancia()
        x, y, _ = self._lab_pose_actual()
        if total - self.lab_last_progress_distance >= self.cfg.lab_progress_min_m:
            self.lab_last_progress_distance = total
            self.lab_last_progress_pose = (x, y)
            self.lab_last_progress_t = time.time()
            return True
        # Respaldo si total_distance no se actualiza pero la pose sí cambió.
        if math.hypot(x - self.lab_last_progress_pose[0], y - self.lab_last_progress_pose[1]) >= self.cfg.lab_progress_min_m:
            self.lab_last_progress_pose = (x, y)
            self.lab_last_progress_distance = total
            self.lab_last_progress_t = time.time()
            return True
        return False

    def _lab_direccion_a_punto(self, ang_rel, distancia):
        """Punto del mapa mirando a un ángulo relativo del robot."""
        x, y, yaw = self._lab_pose_actual()
        a = yaw + ang_rel
        return x + distancia * math.cos(a), y + distancia * math.sin(a)

    def _lab_score_frontera(self, direccion):
        """Puntúa si una dirección lleva a zona nueva/no visitada.

        No hace navegación pesada. Solo mira celdas del mapa delante de cada
        candidato para preferir áreas libres/desconocidas y evitar zonas visitadas.
        """
        angs = {
            "frente": 0.0,
            "izquierda": math.radians(72.0),
            "derecha": math.radians(-72.0),
        }
        ang = angs.get(direccion, 0.0)
        try:
            occ = self.mapa_ruta.obtener_mapa_ocupacion()
            r = float(occ.get("resolution", 0.10))
            occ_set = set(tuple(c) for c in occ.get("occupied_cells", []))
            free_set = set(tuple(c) for c in occ.get("free_cells", []))
            visited_set = set(tuple(c) for c in occ.get("visited_cells", []))
            score = 0.0
            for d in (0.28, 0.42, 0.58, 0.74):
                px, py = self._lab_direccion_a_punto(ang, d)
                cell = (int(round(px / r)), int(round(py / r)))
                if cell in occ_set:
                    score -= self.cfg.lab_frontier_occ_penalty
                elif cell in visited_set:
                    score -= self.cfg.lab_frontier_visit_penalty
                elif cell in free_set:
                    score += self.cfg.lab_frontier_free_bonus
                else:
                    score += self.cfg.lab_frontier_unknown_bonus
            return score
        except Exception:
            return 0.0

    def _lab_debe_escapar(self):
        if not getattr(self.cfg, "lab_usar_escape_atorado", True):
            return False
        if self.estado_actual in {"LAB_ESCAPE_ATORADO", "LAB_RETROCEDER", "PAUSA MANUAL", "ESPERANDO INICIO"}:
            return False
        # Durante un giro normal damos un margen corto. Si se excede mucho, escape.
        ahora = time.time()
        if self.estado_actual == "LAB_GIRAR" and (ahora - self.lab_turn_t0) > self.cfg.lab_giro_stuck_seg:
            return True
        if (ahora - self.lab_last_progress_t) > self.cfg.lab_stuck_timeout_seg:
            return True
        return False

    def _lab_iniciar_escape(self, motivo="atasco"):
        izq = self._lab_dist("izquierda")
        der = self._lab_dist("derecha")
        diag_izq = self.dist_diag_izq if math.isfinite(self.dist_diag_izq) else izq
        diag_der = self.dist_diag_der if math.isfinite(self.dist_diag_der) else der
        score_izq = max(izq, diag_izq) if math.isfinite(max(izq, diag_izq)) else 0.0
        score_der = max(der, diag_der) if math.isfinite(max(der, diag_der)) else 0.0
        self.lab_escape_dir = 1.0 if score_izq >= score_der else -1.0
        self.lab_escape_t0 = time.time()
        self.lab_escape_motivo = str(motivo)
        self.lab_decision_actual = "ESCAPE"
        self.cambiar_estado("LAB_ESCAPE_ATORADO")
        self.get_logger().warn(
            f"ESCAPE_ATORADO: {motivo}. Giro dir={self.lab_escape_dir:+.0f} "
            f"L={score_izq:.2f} R={score_der:.2f}"
        )

    def _lab_control_escape(self):
        """Maniobra cerrada para salir de un bloqueo sin redecidir."""
        t = time.time() - self.lab_escape_t0
        stop = self.cfg.lab_escape_stop_seg
        back = stop + self.cfg.lab_escape_backoff_seg
        giro = back + self.cfg.lab_escape_turn_seg
        salida = giro + self.cfg.lab_escape_forward_seg
        if t < stop:
            return self.cmd_vel(0.0, 0.0)
        if t < back:
            return self.cmd_vel(self.cfg.lab_vel_retroceso, 0.0)
        if t < giro:
            return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_escape_dir * self.cfg.lab_escape_w)
        if t < salida:
            # Avanza lento para salir de la celda conflictiva.
            return self.cmd_vel(self.cfg.lab_vel_lenta, 0.0)

        self.lab_last_progress_t = time.time()
        self.lab_last_progress_distance = self._lab_total_distancia()
        self.lab_decision_t0 = 0.0
        self.cambiar_estado("LAB_AVANZAR")
        self.lab_decision_actual = "AVANZAR"
        return self._lab_control_gap_frontal()

    def _lab_iniciar_giro_rapido(self, motivo="frente urgente"):
        """Giro inmediato para no quedarse pensando cuando el frente está cerca.

        Se usa en zonas como la señal PARE o una esquina: si el frente baja
        demasiado, el robot elige el lado más libre y ejecuta el giro sin
        esperar puntajes ni bloqueo anterior.
        """
        self.lab_turn_dir = self._lado_mas_libre_para_escapar()
        self.lab_decision_actual = "GIRO_RAPIDO_IZQ" if self.lab_turn_dir > 0 else "GIRO_RAPIDO_DER"
        self.lab_decision_t0 = time.time()
        self.lab_turn_t0 = time.time()
        self.lab_estado_anterior = self.estado_actual
        self._lab_marcar_interseccion(self.lab_decision_actual)
        self.cambiar_estado("LAB_GIRAR")
        return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_turn_dir * min(self.cfg.lab_w_max, self.cfg.lab_w_giro * 1.25))

    def _lab_elegir_decision(self, motivo="frente cerrado"):
        """Elige UNA dirección y la bloquea con criterio de frontera.

        No basta escoger el hueco más grande: se pondera si esa dirección parece
        llevar a zona nueva del mapa, si ya fue visitada y si implica volver.
        Atrás/backtrack solo aparece cuando no hay izquierda/derecha/frente útiles.
        """
        ahora = time.time()
        if (ahora - self.lab_decision_t0) < self.cfg.lab_decision_lock_seg:
            return self.lab_turn_dir

        c = self._lab_caminos_disponibles()
        candidatos = []
        if c["izquierda"]:
            candidatos.append((
                "IZQUIERDA", 1.0,
                c["izq_score"] * self.cfg.lab_score_espacio
                + self._lab_score_frontera("izquierda")
                + self.cfg.lab_score_prefer_izq
            ))
        if c["frente"] and c["frente_dist"] > self.cfg.lab_frente_salida:
            candidatos.append((
                "FRENTE", 0.0,
                c["frente_dist"] * self.cfg.lab_score_espacio
                + self._lab_score_frontera("frente")
                + self.cfg.lab_score_prefer_frente
            ))
        if c["derecha"]:
            candidatos.append((
                "DERECHA", -1.0,
                c["der_score"] * self.cfg.lab_score_espacio
                + self._lab_score_frontera("derecha")
                + self.cfg.lab_score_prefer_der
            ))

        aprendida = self._tomar_decision_aprendida(c, motivo=motivo)
        if aprendida is not None:
            nombre, direction = aprendida
            score = 999.0
            self.lab_decision_actual = f"MAPA_{nombre}"
            self.lab_turn_dir = direction
        elif candidatos:
            candidatos.sort(key=lambda t: t[2], reverse=True)
            nombre, direction, score = candidatos[0]
            self.lab_decision_actual = nombre
            if direction != 0.0:
                self.lab_turn_dir = direction
            else:
                # Si ganó frente pero el frente está cerrándose, usa el lado libre mayor.
                self.lab_turn_dir = 1.0 if c["izq_score"] >= c["der_score"] else -1.0
        else:
            self.lab_turn_dir = float(self.cfg.lab_default_turn)
            self.lab_decision_actual = "BACKTRACK"
            self._lab_marcar_callejon()

        self.lab_decision_t0 = ahora
        self._lab_marcar_interseccion(self.lab_decision_actual)
        self.get_logger().warn(
            f"Laberinto: decisión {self.lab_decision_actual} dir={self.lab_turn_dir:+.0f} "
            f"por {motivo} | L={c['izq_score']:.2f} F={c['frente_dist']:.2f} R={c['der_score']:.2f} "
            f"frontier L/F/R={self._lab_score_frontera('izquierda'):.2f}/"
            f"{self._lab_score_frontera('frente'):.2f}/{self._lab_score_frontera('derecha'):.2f}"
        )
        return self.lab_turn_dir

    def _lab_iniciar_giro(self, motivo="frente cerrado"):
        self._lab_elegir_decision(motivo)
        self.lab_turn_t0 = time.time()
        self.lab_estado_anterior = self.estado_actual
        self.cambiar_estado("LAB_GIRAR")

    def _lab_control_pared(self, lado="izquierda"):
        """Seguimiento de pared con distancia prudente, sin cambiar de referencia por ruido."""
        ahora = time.time()
        dt = max(0.01, ahora - self.lab_t_prev)
        target = self.cfg.lab_dist_pared

        if lado == "izquierda":
            dist = self.dist_izq_pared if self.pared_izq_valida else self.dist_izq
            alpha = self.alpha_pared_izq if self.pared_izq_valida else 0.0
            err = dist - target if math.isfinite(dist) else 0.0
            # lejos de izquierda => girar izquierda; cerca => girar derecha
            w = (self.cfg.lab_kp_pared * err
                 + self.cfg.lab_kd_pared * ((err - self.lab_error_prev) / dt)
                 - self.cfg.lab_kp_alpha * alpha)
        else:
            dist = self.dist_der_pared if self.pared_der_valida else self.dist_der
            alpha = self.alpha_pared_der if self.pared_der_valida else 0.0
            err = dist - target if math.isfinite(dist) else 0.0
            # lejos de derecha => girar derecha; cerca => girar izquierda
            w = (-self.cfg.lab_kp_pared * err
                 - self.cfg.lab_kd_pared * ((err - self.lab_error_prev) / dt)
                 - self.cfg.lab_kp_alpha * alpha)

        self.lab_error_prev = err
        self.lab_t_prev = ahora
        self.error_centro_actual = err
        self.error_ang_actual = alpha
        v = self.cfg.lab_vel_avance
        if self.dist_frente < self.cfg.frente_freno_suave:
            v = min(v, self.cfg.lab_vel_lenta)
        return self.cmd_vel(v, self.cfg.signo_giro * self.saturar(w, self.cfg.lab_w_max))

    def _lab_control_gap_frontal(self):
        """Avance por hueco frontal, usado solo si no hay pared izquierda útil."""
        target_ang, width, clearance = self._calcular_gap_frontal()
        w = -self.cfg.lab_kp_gap * target_ang
        w = self.saturar(w, self.cfg.lab_w_max * 0.75)
        v = self.cfg.lab_vel_avance
        if self.dist_frente < self.cfg.frente_freno_suave:
            v = min(v, self.cfg.lab_vel_lenta)
        self.gap_target_deg = math.degrees(target_ang)
        self.gap_width_deg = math.degrees(width)
        self.gap_clearance = clearance
        self.error_centro_actual = target_ang
        self.error_ang_actual = 0.0
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    # ==========================================================
    # MODO AC — USAR MAPA APRENDIDO
    # ==========================================================
    def _mapa_aprendido_dir(self):
        carpeta = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "mapas_aprendidos"))
        os.makedirs(carpeta, exist_ok=True)
        return carpeta

    def _buscar_mapa_aprendido(self):
        carpeta = self._mapa_aprendido_dir()
        candidatos = []
        base = os.path.join(carpeta, "mapa_laberinto.json")
        if os.path.exists(base):
            candidatos.append(base)
        candidatos.extend(sorted(glob.glob(os.path.join(carpeta, "mapa_laberinto_*.json"))))
        if not candidatos:
            return None
        # Preferir mapas que hayan llegado a META y que tengan una ruta más corta.
        # Antes se priorizaba la ruta más larga; eso podía cargar mapas con más vueltas
        # y producir una ruta equivocada al usar A*.
        def score(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    d = json.load(f)
                meta = d.get("meta_estimada", {}) or {}
                eventos_meta = meta.get("eventos", []) or []
                cam = meta.get("camara_verde", {}) or {}
                dist = float(d.get("distancia_recorrida_m", 9999.0) or 9999.0)
                ruta = len(d.get("ruta_recorrida", []) or [])
                meta_ok = 1 if (eventos_meta or cam.get("finalizada")) else 0
                # meta_ok primero, luego menor distancia, luego ruta suficiente, luego reciente.
                return (meta_ok, -dist, ruta, os.path.getmtime(path))
            except Exception:
                return (0, -9999, 0, 0)
        return max(candidatos, key=score)

    def _mapa_cell(self, x, y, res):
        return (int(round(float(x) / float(res))), int(round(float(y) / float(res))))

    def _mapa_center(self, cell, res):
        return (float(cell[0]) * float(res), float(cell[1]) * float(res))

    def _nearest_cell(self, target, cells):
        if not cells:
            return target
        tx, ty = target
        return min(cells, key=lambda c: (c[0] - tx) ** 2 + (c[1] - ty) ** 2)

    def _astar_grid(self, start, goal, libres, ocupadas):
        if start is None or goal is None:
            return None
        openq = []
        heapq.heappush(openq, (0.0, start))
        came = {}
        gscore = {start: 0.0}
        closed = set()
        def h(c):
            return abs(c[0] - goal[0]) + abs(c[1] - goal[1])
        while openq:
            _, cur = heapq.heappop(openq)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cur[0] + dx, cur[1] + dy)
                if nb in ocupadas:
                    continue
                if nb not in libres:
                    continue
                ng = gscore[cur] + 1.0
                if ng < gscore.get(nb, 1e9):
                    came[nb] = cur
                    gscore[nb] = ng
                    heapq.heappush(openq, (ng + h(nb), nb))
        return None

    def _simplificar_waypoints(self, pts, min_dist=0.28):
        if not pts:
            return []
        out = [pts[0]]
        last = pts[0]
        last_dir = None
        for p in pts[1:]:
            dx = float(p[0]) - float(last[0])
            dy = float(p[1]) - float(last[1])
            d = math.hypot(dx, dy)
            direction = None
            if d > 1e-6:
                direction = (round(dx / d), round(dy / d))
            if d >= min_dist or (last_dir is not None and direction != last_dir):
                out.append(p)
                last = p
                last_dir = direction
        if math.hypot(float(out[-1][0]) - float(pts[-1][0]), float(out[-1][1]) - float(pts[-1][1])) > 0.05:
            out.append(pts[-1])
        return out

    def _normalizar_decision_aprendida(self, raw):
        """Normaliza decisiones guardadas a giros ejecutables.

        Para el modo aprendido no usamos FRENTE/BACKTRACK como órdenes
        absolutas. Solo guardamos giros claros que el robot realmente hizo.
        """
        d = str(raw or "").upper().strip()
        if "IZQ" in d or "LEFT" in d:
            return "IZQUIERDA", 1.0
        if "DER" in d or "RIGHT" in d:
            return "DERECHA", -1.0
        # FRENTE, BACKTRACK, ESCAPE, SEGUIR_IZQ, RESPALDO, etc. no son giros
        # confiables para repetir como ruta óptima.
        return None, None

    def _extraer_decisiones_aprendidas(self, data):
        """Extrae una lista compacta de giros desde el mapa aprendido.

        La ruta aprendida se usa como memoria de intersecciones: cuando el robot
        vuelva a encontrar una decisión local, intentará aplicar el siguiente
        giro de esta lista si el LiDAR confirma que ese lado está libre.
        """
        eventos = data.get("intersecciones", []) or []
        eventos = sorted(eventos, key=lambda e: float(e.get("t", 0.0)))
        decisiones = []
        last = None
        last_t = -1e9
        for ev in eventos:
            nombre, direction = self._normalizar_decision_aprendida(ev.get("decision", ""))
            if nombre is None:
                continue
            try:
                t = float(ev.get("t", 0.0))
            except Exception:
                t = last_t + 1.0
            # Evita duplicados creados por ruido en la misma maniobra.
            if last == nombre and (t - last_t) < 1.2:
                continue
            decisiones.append({
                "decision": nombre,
                "dir": float(direction),
                "x": float(ev.get("x", 0.0)),
                "y": float(ev.get("y", 0.0)),
                "t": t,
            })
            last = nombre
            last_t = t
        return decisiones

    def _decision_aprendida_disponible(self, decision, caminos):
        if decision == "IZQUIERDA":
            return bool(caminos.get("izquierda", False))
        if decision == "DERECHA":
            return bool(caminos.get("derecha", False))
        return False

    def _tomar_decision_aprendida(self, caminos, motivo=""):
        """Intenta usar la siguiente decisión aprendida sin forzar choques.

        Si el lado aprendido no está libre según el LiDAR actual, NO se fuerza.
        Se mira unas pocas decisiones siguientes para tolerar desfase entre el
        mapa aprendido y la posición real. Si ninguna es segura, retorna None y
        la lógica estable decide como siempre.
        """
        if not getattr(self, "usar_decisiones_aprendidas_activo", False):
            return None
        seq = getattr(self, "mapa_decisiones_aprendidas", []) or []
        if not seq:
            return None
        idx0 = int(getattr(self, "mapa_decision_idx", 0))
        if idx0 >= len(seq):
            return None
        ahora = time.time()
        # Evita gastar varias decisiones dentro de la misma esquina.
        if (ahora - float(getattr(self, "mapa_decision_ultima_t", 0.0))) < float(getattr(self.cfg, "mapa_decision_min_seg", 0.75)):
            return None
        max_look = int(getattr(self.cfg, "mapa_decision_lookahead", 4))
        for j in range(idx0, min(len(seq), idx0 + max_look)):
            item = seq[j]
            nombre = item.get("decision", "")
            if self._decision_aprendida_disponible(nombre, caminos):
                self.mapa_decision_idx = j + 1
                self.mapa_decision_ultima_t = ahora
                try:
                    self.get_logger().warn(
                        f"MAPA_DECISION: {nombre} ({self.mapa_decision_idx}/{len(seq)}) "
                        f"por {motivo}; skipped={j-idx0}"
                    )
                except Exception:
                    pass
                return nombre, float(item.get("dir", 1.0))
        return None

    def _lab_intentar_giro_aprendido_en_interseccion(self):
        """Ejecuta una decisión aprendida cuando aparece una apertura lateral.

        Esto convierte el mapa aprendido en una guía de intersecciones: el robot
        sigue navegando con su lógica estable, pero cuando detecta una bifurcación
        intenta repetir el siguiente giro aprendido, siempre validando con LiDAR.
        """
        if not getattr(self, "usar_decisiones_aprendidas_activo", False):
            return None
        if self.estado_actual in {"LAB_GIRAR", "LAB_SALIDA_GIRO", "LAB_RETROCEDER", "LAB_ESCAPE_ATORADO", "PARAR_PARE"}:
            return None
        c = self._lab_caminos_disponibles()
        umbral = float(getattr(self.cfg, "mapa_interseccion_lateral_min", 0.58))
        hay_apertura = bool(c.get("izq_score", 0.0) > umbral or c.get("der_score", 0.0) > umbral)
        if not hay_apertura:
            return None
        aprendida = self._tomar_decision_aprendida(c, motivo="intersección aprendida")
        if aprendida is None:
            return None
        nombre, direction = aprendida
        self.lab_turn_dir = float(direction)
        self.lab_decision_actual = f"MAPA_{nombre}"
        self.lab_decision_t0 = time.time()
        self.lab_turn_t0 = time.time()
        self.lab_estado_anterior = self.estado_actual
        self._lab_marcar_interseccion(self.lab_decision_actual)
        self.cambiar_estado("LAB_GIRAR")
        return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_turn_dir * self.cfg.lab_w_giro)

    def _ruta_aprendida_fallback(self, data):
        """Convierte la corrida que SÍ llegó a META en una ruta tipo migas de pan.

        Para el modo experimental conviene seguir la trayectoria aprendida que ya
        funcionó, no una ruta A* calculada sobre una grilla ruidosa de cmd_vel.
        Si hay evento de META verde, recorta la ruta hasta el punto más cercano
        a esa detección para no seguir vueltas posteriores.
        """
        ruta = data.get("ruta_recorrida", []) or []
        pts = []
        for item in ruta:
            try:
                pts.append((float(item.get("x", 0.0)), float(item.get("y", 0.0))))
            except Exception:
                pass
        if not pts:
            return []
        try:
            eventos = ((data.get("meta_estimada", {}) or {}).get("eventos", []) or [])
            if eventos:
                ev = eventos[-1]
                mx, my = float(ev.get("x", pts[-1][0])), float(ev.get("y", pts[-1][1]))
                k = min(range(len(pts)), key=lambda i: (pts[i][0]-mx)**2 + (pts[i][1]-my)**2)
                pts = pts[:max(2, min(len(pts), k + 8))]
        except Exception:
            pass
        return self._simplificar_waypoints(pts, min_dist=float(getattr(self.cfg, "mapa_wp_min_dist", 0.30)))

    def _mapa_wp_runtime(self, idx):
        tx, ty = self.mapa_aprendido_waypoints[int(idx)]
        return (float(tx) + float(getattr(self, "mapa_runtime_shift_x", 0.0)),
                float(ty) + float(getattr(self, "mapa_runtime_shift_y", 0.0)))

    def cargar_mapa_aprendido(self, ruta=None):
        try:
            ruta = ruta or self._buscar_mapa_aprendido()
            if not ruta or not os.path.exists(ruta):
                self.mapa_aprendido_estado = "NO_ENCONTRADO"
                return False
            with open(ruta, "r", encoding="utf-8") as f:
                data = json.load(f)
            grid = data.get("grid", {}) or {}
            res = float(grid.get("resolution_m", 0.10) or 0.10)
            occ = {tuple(map(int, c)) for c in (grid.get("occupied_cells", []) or [])}
            libres = {tuple(map(int, c)) for c in (grid.get("free_cells", []) or [])}
            libres |= {tuple(map(int, c)) for c in (grid.get("visited_cells", []) or [])}
            # Inicio desde primera celda visitada o primer punto de ruta.
            ruta_rec = data.get("ruta_recorrida", []) or []
            if ruta_rec:
                start = self._mapa_cell(ruta_rec[0].get("x", 0.0), ruta_rec[0].get("y", 0.0), res)
            else:
                start = (0, 0)
            start = self._nearest_cell(start, libres)
            # Meta: preferir evento de cámara verde; si no, último punto de ruta.
            meta = None
            eventos = ((data.get("meta_estimada", {}) or {}).get("eventos", []) or [])
            if eventos:
                ev = eventos[-1]
                meta = self._mapa_cell(ev.get("x", 0.0), ev.get("y", 0.0), res)
            elif ruta_rec:
                meta = self._mapa_cell(ruta_rec[-1].get("x", 0.0), ruta_rec[-1].get("y", 0.0), res)
            else:
                meta = (0, 0)
            goal = self._nearest_cell(meta, libres)
            # Modo AF: usar la ruta aprendida que ya llegó a META como guía principal.
            # El A* sobre el mapa de cmd_vel puede elegir atajos falsos si el mapa quedó
            # deformado; por eso A* queda solo como respaldo si no hay ruta grabada.
            waypoints = self._ruta_aprendida_fallback(data)
            fuente = "ruta_aprendida"
            if len(waypoints) < 2:
                path_cells = self._astar_grid(start, goal, libres, occ)
                if path_cells:
                    pts = [self._mapa_center(c, res) for c in path_cells]
                    waypoints = self._simplificar_waypoints(pts, min_dist=float(getattr(self.cfg, "mapa_wp_min_dist", 0.30)))
                    fuente = "A*_respaldo"
            decisiones = self._extraer_decisiones_aprendidas(data)
            if len(waypoints) < 2 and not decisiones:
                self.mapa_aprendido_estado = "SIN_RUTA_NI_DECISIONES"
                return False
            self.mapa_aprendido_cargado = True
            self.mapa_aprendido_path = ruta
            self.mapa_aprendido_data = data
            self.mapa_aprendido_waypoints = waypoints
            self.mapa_aprendido_waypoint_idx = 1 if len(waypoints) > 1 else 0
            self.mapa_decisiones_aprendidas = decisiones
            self.mapa_decision_idx = 0
            self.mapa_aprendido_estado = "CARGADO"
            self.mapa_aprendido_fuente = fuente
            self.mapa_decision_fuente = f"decisiones_intersecciones:{os.path.basename(ruta)}"
            try:
                self.get_logger().warn(
                    f"Mapa aprendido listo: decisiones={len(decisiones)}, waypoints={len(waypoints)}, "
                    f"fuente={fuente}, archivo={os.path.basename(ruta)}"
                )
            except Exception:
                pass
            return True
        except Exception as e:
            self.mapa_aprendido_estado = f"ERROR: {e}"
            try:
                self.get_logger().error(f"Error cargando mapa aprendido: {e}")
            except Exception:
                pass
            return False

    def control_usar_mapa_aprendido(self):
        if not self.mapa_aprendido_cargado or not self.mapa_aprendido_waypoints:
            self.cambiar_estado("MAPA_SIN_CARGAR")
            return self.cmd_vel(0.0, 0.0)
        if self.meta_vision_finalizada:
            self.cambiar_estado("META_ALCANZADA")
            return self.cmd_vel(0.0, 0.0)
        # Si el modo mapa se bloquea, usar temporalmente la lógica estable del laberinto
        # en vez de quedarse girando contra una pared.
        now = time.time()
        if now < float(getattr(self, "mapa_recovery_until", 0.0)):
            self.cambiar_estado("USAR_MAPA_RECUPERAR")
            self.lab_decision_actual = "RECUPERAR_CON_LOGICA_ESTABLE"
            return self.control_laberinto()

        x, y, yaw, fuente = self.pose_para_mapa()
        idx = int(self.mapa_aprendido_waypoint_idx)
        # Avanzar waypoints ya alcanzados.
        radio = float(getattr(self.cfg, "mapa_wp_radio", 0.24))
        while idx < len(self.mapa_aprendido_waypoints):
            tx, ty = self._mapa_wp_runtime(idx)
            if math.hypot(float(tx) - x, float(ty) - y) <= radio:
                idx += 1
            else:
                break
        self.mapa_aprendido_waypoint_idx = idx
        if idx >= len(self.mapa_aprendido_waypoints):
            self.cambiar_estado("MAPA_RUTA_COMPLETA")
            # Si todavía no ve META verde, sigue suave hacia adelante; la cámara es la confirmación final.
            return self.cmd_vel(float(getattr(self.cfg, "mapa_vel_llegada", 0.045)), 0.0)
        tx, ty = self._mapa_wp_runtime(idx)
        dx = float(tx) - x
        dy = float(ty) - y
        dist = math.hypot(dx, dy)

        # Detector de atasco del modo mapa: si no reduce distancia al waypoint o si
        # el frente se mantiene cerrado, cede el control unos segundos a la lógica
        # estable que ya sabemos que recorre bien.
        try:
            if idx != int(getattr(self, "mapa_progress_last_idx", idx)) or dist < float(getattr(self, "mapa_progress_last_dist", 1e9)) - 0.045:
                self.mapa_progress_last_idx = idx
                self.mapa_progress_last_dist = dist
                self.mapa_progress_last_t = now
            elif now - float(getattr(self, "mapa_progress_last_t", now)) > float(getattr(self.cfg, "mapa_stuck_seg", 3.2)):
                self.mapa_recovery_until = now + float(getattr(self.cfg, "mapa_recovery_seg", 2.8))
                self.mapa_progress_last_t = now
                self.mapa_progress_last_dist = dist
                self.cambiar_estado("USAR_MAPA_ATASCO")
                return self.control_laberinto()
        except Exception:
            pass
        ang_obj = math.atan2(dy, dx)
        err = self.normalizar_angulo(ang_obj - yaw)
        kp = float(getattr(self.cfg, "mapa_kp_yaw", 1.18))
        w = self.saturar(kp * err, float(getattr(self.cfg, "mapa_w_max", 0.46)))
        v_base = float(getattr(self.cfg, "mapa_vel_avance", 0.105))
        if abs(err) > float(getattr(self.cfg, "mapa_ang_giro_lento", 0.70)):
            v_base = float(getattr(self.cfg, "mapa_vel_giro", 0.035))
        elif dist < 0.35:
            v_base = min(v_base, float(getattr(self.cfg, "mapa_vel_lenta", 0.070)))
        # Si el frente se cierra, no perder tiempo recalculando: girar hacia el waypoint sin avanzar.
        if self.dist_frente < float(getattr(self.cfg, "mapa_frente_stop", 0.28)):
            v_base = 0.0
            if abs(w) < 0.12:
                w = 0.18 if self.dist_izq > self.dist_der else -0.18
            try:
                if self.mapa_front_block_t is None:
                    self.mapa_front_block_t = now
                elif now - self.mapa_front_block_t > float(getattr(self.cfg, "mapa_front_block_seg", 1.25)):
                    self.mapa_recovery_until = now + float(getattr(self.cfg, "mapa_recovery_seg", 2.8))
                    self.mapa_front_block_t = None
                    self.cambiar_estado("USAR_MAPA_BLOQUEADO")
                    return self.control_laberinto()
            except Exception:
                pass
        else:
            self.mapa_front_block_t = None
        self.cambiar_estado(f"USAR_MAPA {idx}/{len(self.mapa_aprendido_waypoints)-1}")
        self.lab_decision_actual = f"WP {idx}/{len(self.mapa_aprendido_waypoints)-1} {self.mapa_aprendido_fuente}"
        return self.cmd_vel(v_base, w)

    def control_laberinto(self):
        """Lógica tipo micromouse/laberinto para el reto.

        No hace DFS libre ni cambia de dirección cada frame. La estrategia es:
        avanzar seguro -> detectar cierre -> decidir y bloquear giro ->
        salir unos cm -> seguir pared izquierda con distancia segura.
        """
        frente = self.dist_frente if math.isfinite(self.dist_frente) else float('inf')
        izq = self._lab_dist("izquierda")
        der = self._lab_dist("derecha")

        self._lab_actualizar_progreso()
        if self.estado_actual == "LAB_ESCAPE_ATORADO":
            return self._lab_control_escape()
        if self._lab_debe_escapar():
            self._lab_iniciar_escape("sin progreso / giro prolongado")
            return self._lab_control_escape()

        # Acción concreta cuando el frente está muy cerca: no seguir
        # evaluando mapa/intersecciones, elegir lado libre y girar.
        # Evita quedarse pensando frente a PARE/esquina.
        if (self.estado_actual not in {"LAB_GIRAR", "LAB_SALIDA_GIRO", "LAB_RETROCEDER"}
                and frente <= float(getattr(self.cfg, "lab_frente_giro_rapido", 0.30))):
            return self._lab_iniciar_giro_rapido("frente urgente")

        if self.estado_actual == "LAB_RETROCEDER":
            if (time.time() - self.lab_backoff_t0) < self.cfg.lab_backoff_seg:
                return self.cmd_vel(self.cfg.lab_vel_retroceso, 0.0)
            self._lab_iniciar_giro("fin de retroceso")
            return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_turn_dir * self.cfg.lab_w_giro)

        if self.estado_actual == "LAB_GIRAR":
            dt = time.time() - self.lab_turn_t0
            frente_libre = frente > self.cfg.lab_frente_salida
            min_ok = dt >= self.cfg.lab_turn_min_seg
            max_ok = dt >= self.cfg.lab_turn_max_seg
            if max_ok and not frente_libre:
                self._lab_iniciar_escape("giro sin salida frontal")
                return self._lab_control_escape()
            if (min_ok and frente_libre) or max_ok:
                self.lab_exit_t0 = time.time()
                self.cambiar_estado("LAB_SALIDA_GIRO")
                return self.cmd_vel(self.cfg.lab_vel_lenta, 0.0)
            return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_turn_dir * self.cfg.lab_w_giro)

        if self.estado_actual == "LAB_SALIDA_GIRO":
            # Avanza un instante sin redecidir para no quedarse pensando en la esquina.
            if frente <= self.cfg.frente_emergencia:
                self._lab_iniciar_escape("salida de giro bloqueada")
                return self._lab_control_escape()
            if (time.time() - self.lab_exit_t0) < self.cfg.lab_salida_giro_seg:
                return self.cmd_vel(self.cfg.lab_vel_lenta, 0.0)
            self.cambiar_estado("LAB_SEGUIR_IZQUIERDA")

        # Atasco muy fuerte: retroceso pequeño.
        if frente <= self.cfg.frente_emergencia and der <= self.cfg.lateral_stop_global and izq <= self.cfg.lateral_stop_global:
            self.lab_backoff_t0 = time.time()
            self.cambiar_estado("LAB_RETROCEDER")
            return self.cmd_vel(self.cfg.lab_vel_retroceso, 0.0)

        # Si el frente se cierra, no avanzar: decisión concreta y giro bloqueado.
        if frente <= self.cfg.lab_frente_giro or (self.front_class_sm == "CORNER" and frente < self.cfg.frente_esquina_detect):
            self._lab_iniciar_giro("frente/esquina")
            return self.cmd_vel(0.0, self.cfg.signo_giro * self.lab_turn_dir * self.cfg.lab_w_giro)

        # En modo aprendido, si hay una apertura lateral clara, intenta aplicar
        # el siguiente giro aprendido. Si no es seguro, no fuerza nada.
        cmd_aprendido = self._lab_intentar_giro_aprendido_en_interseccion()
        if cmd_aprendido is not None:
            return cmd_aprendido

        # Anti-roce preventivo local.
        if der < self.cfg.lab_lateral_seguro and der < izq:
            self.cambiar_estado("LAB_AVANZAR")
            return self.cmd_vel(self.cfg.lab_vel_lenta, abs(self.cfg.w_escape_lateral))
        if izq < self.cfg.lab_lateral_seguro and izq < der:
            self.cambiar_estado("LAB_AVANZAR")
            return self.cmd_vel(self.cfg.lab_vel_lenta, -abs(self.cfg.w_escape_lateral))

        # Estrategia principal después del primer giro: seguir pared izquierda.
        if (self.pared_izq_valida or math.isfinite(self.dist_izq)) and self.dist_izq < 0.85:
            self.cambiar_estado("LAB_SEGUIR_IZQUIERDA")
            self.lab_decision_actual = "SEGUIR_IZQ"
            return self._lab_control_pared("izquierda")

        # Si no existe pared izquierda cercana, usa derecha como respaldo, pero solo temporal.
        if (self.pared_der_valida or math.isfinite(self.dist_der)) and self.dist_der < 0.55:
            self.cambiar_estado("LAB_SEGUIR_DERECHA_RESPALDO")
            self.lab_decision_actual = "RESPALDO_DER"
            return self._lab_control_pared("derecha")

        # Zona abierta: avanzar por hueco frontal, con preferencia a seguir adelante.
        self.cambiar_estado("LAB_AVANZAR")
        self.lab_decision_actual = "AVANZAR"
        return self._lab_control_gap_frontal()

    # ==========================================================
    # FSM MODULAR DE MOVIMIENTO
    # ==========================================================
    def calcular_comando(self):
        if getattr(self, "usar_mapa_aprendido_activo", False):
            return self.control_usar_mapa_aprendido()

        if getattr(self.cfg, "usar_modo_laberinto", False):
            return self.control_laberinto()

        if getattr(self.cfg, "usar_modo_aspiradora", False):
            return self.control_aspiradora()

        if self.rodeo_caja.es_estado_rodeo(self.estado_actual):
            return self.rodear_obstaculo()

        if self.estado_actual == "ACERCAR_ESQUINA":
            return self.acercar_esquina()

        if self.estado_actual == "GIRO_EVITAR_FRENTE":
            return self.girar_por_frente()

        if self.estado_actual == "ALINEAR_POST_GIRO":
            return self.alinear_post_giro()

        if self.estado_actual == "AVANZAR_POST_ESQUINA":
            return self.avanzar_post_esquina()

        if self.estado_actual in {"CENTRAR_PASILLO", "CARRIL_CENTRAL", "ACERCARSE_DERECHA", "ALINEAR_PARED", "SEGUIR_PARED", "RECUPERAR_PARED", "SEGUIR_PARED_DERECHA_SUAVE", "SEGUIR_PARED_IZQUIERDA_SUAVE"}:
            return self.navegar_por_centro()

        if self.estado_actual == "BUSCAR_DERECHA":
            return self.buscar_referencia_lateral()

        if self.estado_actual == "REFERENCIA_BLOQUEADA":
            return self.control_con_referencia_bloqueada()

        return self.cmd_vel(0.0, 0.0)

    def persist_check(self, clave, condicion, frames=None):
        """Exige varios scans consecutivos antes de disparar una maniobra."""
        n = int(frames if frames is not None else self.cfg.persist_frames)
        c = self._persist.get(clave, 0)
        c = c + 1 if condicion else 0
        self._persist[clave] = c
        return c >= max(1, n)

    def reset_persistencia(self):
        self._persist.clear()

    # ----------------------------------------------------------
    # Bloqueo temporal de referencia lateral en esquinas
    # ----------------------------------------------------------
    def estados_maniobra_esquina(self):
        return {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}

    def zona_pre_esquina_activa(self):
        """Zona donde NO conviene cambiar de referencia lateral.

        Si el frente empieza a cerrarse o el clasificador ve esquina, mantener la
        referencia anterior evita que la pared izquierda sea tomada como guía falsa.
        """
        return (
            self.estado_actual in self.estados_maniobra_esquina()
            or self.dist_frente < self.cfg.frente_zona_bloqueo_referencia
            or self.frente_es_esquina
            or self.front_class_sm == "CORNER"
        )

    def actualizar_referencia_estable(self):
        """Actualiza la referencia estable con histéresis.

        La referencia NO cambia dentro de zona de esquina. Fuera de esa zona,
        permite cambiar solo si el nuevo lado aparece varios frames seguidos.
        """
        if not self.cfg.bloqueo_referencia_esquinas:
            return
        if self.referencia_bloqueada is not None:
            return

        lado = None
        if self.pared_der_valida and not self.pared_izq_valida:
            lado = "derecha"
        elif self.pared_izq_valida and not self.pared_der_valida:
            lado = "izquierda"
        elif self.pared_der_valida and self.pared_izq_valida:
            # En pasillo con ambas paredes se conserva la referencia anterior.
            if self.referencia_estable not in {"derecha", "izquierda"}:
                self.referencia_estable = self.cfg.referencia_preferida_esquina
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            return

        if lado is None:
            return

        # Cerca de esquina no se cambia a otro lado; solo se mantiene memoria.
        if self.zona_pre_esquina_activa() and lado != self.referencia_estable:
            return

        if lado == self.referencia_estable:
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            return

        if self.referencia_candidata == lado:
            self.frames_referencia_candidata += 1
        else:
            self.referencia_candidata = lado
            self.frames_referencia_candidata = 1

        if self.frames_referencia_candidata >= self.cfg.frames_cambio_referencia:
            self.referencia_estable = lado
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            self.get_logger().info(f"Referencia estable -> {self.referencia_estable}")

    def lado_con_memoria_reciente(self, lado):
        ahora = time.time()
        if lado == "derecha":
            return (ahora - self.t_ultima_pared_der) <= self.cfg.ref_memoria_lateral_seg
        if lado == "izquierda":
            return (ahora - self.t_ultima_pared_izq) <= self.cfg.ref_memoria_lateral_seg
        return False

    def bloquear_referencia_para_esquina(self):
        """Congela una referencia para toda la maniobra de esquina."""
        if not self.cfg.bloqueo_referencia_esquinas:
            return

        preferida = self.cfg.referencia_preferida_esquina

        # Para este circuito se prefiere derecha para evitar que la izquierda
        # capture el control antes de la 4ta esquina. Si alguna vez se desea el
        # otro sentido, cambia referencia_preferida_esquina en control_config.py.
        if preferida in {"derecha", "izquierda"}:
            ref = preferida
        elif self.referencia_estable in {"derecha", "izquierda"}:
            ref = self.referencia_estable
        elif self.pared_der_valida or self.lado_con_memoria_reciente("derecha"):
            ref = "derecha"
        elif self.pared_izq_valida or self.lado_con_memoria_reciente("izquierda"):
            ref = "izquierda"
        else:
            ref = "derecha"

        self.referencia_bloqueada = ref
        self.t_bloqueo_referencia = time.time()
        self.referencia_estable = ref
        self.get_logger().info(f"Referencia bloqueada en esquina -> {ref}")

    def liberar_bloqueo_referencia(self, motivo=""):
        if self.referencia_bloqueada is not None:
            ref = self.referencia_bloqueada
            self.referencia_bloqueada = None
            self.referencia_candidata = None
            self.frames_referencia_candidata = 0
            txt = f"Referencia liberada ({ref})"
            if motivo:
                txt += f": {motivo}"
            self.get_logger().info(txt)

    def bloqueo_referencia_activo(self):
        if not self.cfg.bloqueo_referencia_esquinas:
            return False
        if self.referencia_bloqueada is None:
            return False
        if (time.time() - self.t_bloqueo_referencia) > self.cfg.max_tiempo_bloqueo_referencia:
            self.liberar_bloqueo_referencia("tiempo máximo")
            return False
        return True

    def debe_proteger_cambio_referencia_pre_esquina(self):
        """Evita saltar de derecha a izquierda justo antes de una esquina."""
        if not self.cfg.bloqueo_referencia_pre_esquina:
            return False
        if not self.zona_pre_esquina_activa():
            return False

        # Caso que estaba fallando: derecha perdida, izquierda visible, frente cerca.
        if self.referencia_estable == "derecha" and not self.pared_der_valida and self.pared_izq_valida:
            return True
        if self.referencia_estable == "izquierda" and not self.pared_izq_valida and self.pared_der_valida:
            return True
        return False

    def control_con_referencia_bloqueada(self):
        """Control seguro cuando no se permite cambiar de referencia.

        Si la referencia bloqueada está visible, la sigue. Si no está visible,
        avanza recto lento e ignora el lado contrario para no darse vuelta.
        """
        ref = self.referencia_bloqueada or self.referencia_estable or self.cfg.referencia_preferida_esquina

        # Si ya está muy cerca del frente, no seguir avanzando: preparar esquina.
        if self.dist_frente <= self.cfg.distancia_detencion_esquina or self.dist_frente <= self.cfg.frente_critico:
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        if ref == "derecha" and self.pared_der_valida:
            self.cambiar_estado("SEGUIR_PARED_DERECHA_SUAVE")
            return self.seguir_derecha_suave()

        if ref == "izquierda" and self.pared_izq_valida:
            self.cambiar_estado("SEGUIR_PARED_IZQUIERDA_SUAVE")
            return self.seguir_izquierda_suave()

        # Referencia perdida: no tomar el otro lado como guía.
        self.cambiar_estado("REFERENCIA_BLOQUEADA")
        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        v = min(self.cfg.avance_recto_ref_bloqueada, self.cfg.vel_lenta)
        return self.cmd_vel(v, 0.0)

    def navegar_por_centro(self):
        """
        Control principal por prioridades:
        1) seguridad frontal / caja / esquina,
        2) carril central cuando el espacio es amplio,
        3) centrado entre paredes si el pasillo es estrecho,
        4) seguimiento lateral solo como respaldo.
        """
        # Primero diferencia PASADIZO vs ESQUINA.
        # Pasadizo: paredes a ambos lados y frente libre -> pasar por el medio.
        # Esquina: frontal conectado a lateral -> acercarse, detenerse y girar controlado.
        en_cooldown = self.deteccion_esquina_bloqueada()

        # Prioridad nueva: LiDAR manda la ruta.
        # Si existe un hueco frontal útil y todavía no estamos en zona crítica,
        # no dejamos que el IMU/odom ni una pared lateral vieja obliguen a seguir recto
        # o a pegarse a la pared exterior. El robot apunta al centro del hueco libre.
        if (
            getattr(self.cfg, "gap_prioridad_lidar", True)
            and self.debe_usar_carril_central()
            and self.dist_frente > self.cfg.frente_stop_global
        ):
            self.ultimo_lado_pared = "gap_lidar"
            self.cambiar_estado("CARRIL_CENTRAL")
            return self.control_carril_central()

        # Clasificador fusionado: arco frontal corto = caja; arco largo/conectado = esquina.
        # Además usa persistencia para que un scan aislado no active maniobras.
        ahora = time.time()
        caja_candidata = (
            self.front_class_sm == "BOX"
            and self.dist_frente < self.cfg.frente_caja_detect
            and not en_cooldown
            and (ahora - self.t_ultima_caja) > self.cfg.cooldown_caja
        )
        esquina_candidata = (
            (self.frente_es_esquina or self.front_class_sm == "CORNER")
            and self.dist_frente < self.cfg.frente_esquina_detect
            and not en_cooldown
        )

        if self.persist_check('to_box', caja_candidata):
            self.contador_cajas += 1
            self.t_ultima_caja = ahora
            self.reset_persistencia()
            self.get_logger().warn(
                f"Caja detectada por arco corto #{self.contador_cajas} | "
                f"ancho={math.degrees(self.front_ang_width):.1f}°"
            )
            self.iniciar_rodeo()
            return self.rodear_obstaculo()

        if self.persist_check('to_corner', esquina_candidata):
            self.reset_persistencia()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        # Protección general: si se aproxima una esquina y pierde la referencia
        # derecha, NO cambia a seguir la izquierda; avanza recto lento hasta
        # confirmar esquina o recuperar la referencia.
        if self.bloqueo_referencia_activo() or self.debe_proteger_cambio_referencia_pre_esquina():
            if self.referencia_bloqueada is None:
                self.bloquear_referencia_para_esquina()
            return self.control_con_referencia_bloqueada()

        # Si está demasiado cerca y no es pasadizo, decide por el clasificador.
        # BOX -> rodeo; CORNER/conectado -> esquina controlada.
        if self.dist_frente < self.cfg.frente_critico:
            if en_cooldown:
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            if self.front_class_sm == "BOX" and (ahora - self.t_ultima_caja) > self.cfg.cooldown_caja:
                self.contador_cajas += 1
                self.t_ultima_caja = ahora
                self.reset_persistencia()
                self.get_logger().warn(f"Caja crítica detectada #{self.contador_cajas}: iniciando rodeo.")
                self.iniciar_rodeo()
                return self.rodear_obstaculo()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        # MODO CARRIL CENTRAL: si el espacio es amplio, NO sigue la pared exterior.
        # Busca el hueco libre principal al frente y avanza por el centro.
        # Esto es lo que fuerza un recorrido tipo línea azul, en vez de pegarse
        # a la pared exterior como hacía la línea verde.
        if self.debe_usar_carril_central():
            self.ultimo_lado_pared = "carril_central"
            self.cambiar_estado("CARRIL_CENTRAL")
            return self.control_carril_central()

        # Caso ideal: ambas paredes laterales conectadas existen.
        if self.pared_der_valida and self.pared_izq_valida:
            self.ultimo_lado_pared = "ambas"
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.conducir_centrado()

        # Respaldo: solo pared derecha conectada.
        if self.pared_der_valida:
            self.ultimo_lado_pared = "derecha"
            self.cambiar_estado("SEGUIR_PARED_DERECHA_SUAVE")
            return self.seguir_derecha_suave()

        # Respaldo: solo pared izquierda conectada.
        # Cerca de una esquina, si la referencia estable es derecha, se ignora
        # la izquierda para evitar que el robot se dé la vuelta.
        if self.pared_izq_valida:
            if self.debe_proteger_cambio_referencia_pre_esquina():
                if self.referencia_bloqueada is None:
                    self.bloquear_referencia_para_esquina()
                return self.control_con_referencia_bloqueada()
            self.ultimo_lado_pared = "izquierda"
            self.cambiar_estado("SEGUIR_PARED_IZQUIERDA_SUAVE")
            return self.seguir_izquierda_suave()

        # Sin paredes laterales conectadas: no tomes la pared frontal como derecha.
        self.cambiar_estado("BUSCAR_DERECHA")
        return self.buscar_referencia_lateral()

    def debe_usar_carril_central(self):
        """Activa el modo de hueco libre cuando el espacio es amplio.

        Regla clave del reto: en zonas abiertas no se debe seguir una pared
        exterior. La pared solo sirve como límite de seguridad; la trayectoria
        se define por el centro del espacio libre frontal.
        """
        if not getattr(self.cfg, "usar_carril_central", True):
            return False

        # Nunca reemplaza maniobras de seguridad/esquina/rodeo.
        if self.estado_actual in self.estados_maniobra_esquina() or self.rodeo_caja.es_estado_rodeo(self.estado_actual):
            return False
        if self.dist_frente < self.cfg.frente_critico:
            return False

        # Pasillo estrecho real: ahí sí conviene centrar entre paredes.
        pasillo_estrecho = (
            self.pared_der_valida and self.pared_izq_valida and
            math.isfinite(self.ancho_pasillo) and
            self.ancho_pasillo <= self.cfg.ancho_pasillo_estrecho_max
        )
        if pasillo_estrecho:
            return False

        # Si el frente está razonablemente libre, prioriza carril central.
        if self.dist_frente >= self.cfg.carril_frente_min:
            return True

        # Si una sola pared aparece, no seguirla automáticamente; puede ser la
        # pared exterior que lleva a la ruta larga. Usa gap si hay hueco frontal.
        if (self.pared_der_valida != self.pared_izq_valida) and self.dist_frente > self.cfg.frente_critico:
            return True

        return False

    def _calcular_gap_frontal(self):
        """Devuelve (target_angle_rad, width_rad, clearance_m) del mejor hueco.

        Convención local del proyecto:
        - frente = +Y
        - derecha = +X
        - izquierda = -X
        Por eso angle = atan2(x, y): positivo apunta a derecha, negativo a izquierda.
        """
        if not self.datos_filtrados:
            return 0.0, 0.0, float('inf')

        x = np.asarray(self.datos_filtrados.get('x', []), dtype=float)
        y = np.asarray(self.datos_filtrados.get('y', []), dtype=float)
        if x.size == 0 or y.size == 0 or x.size != y.size:
            return 0.0, 0.0, float('inf')

        r = np.hypot(x, y)
        ang = np.arctan2(x, y)

        sector = math.radians(self.cfg.carril_sector_deg)
        n_bins = int(self.cfg.carril_bins)
        bins = np.linspace(-sector, sector, n_bins)
        medio = (bins[:-1] + bins[1:]) * 0.5
        dist_bins = np.full(n_bins - 1, self.cfg.carril_max_range, dtype=float)

        valid = (
            np.isfinite(x) & np.isfinite(y) & np.isfinite(r) &
            (r > 0.06) & (r < self.cfg.carril_max_range) &
            (np.abs(ang) <= sector) &
            (y > -0.05)
        )
        if np.any(valid):
            inds = np.digitize(ang[valid], bins) - 1
            rr = r[valid]
            for bi, rv in zip(inds, rr):
                if 0 <= bi < dist_bins.size and rv < dist_bins[bi]:
                    dist_bins[bi] = rv

        # Suavizado pequeño para que un punto aislado no cambie todo el rumbo.
        if dist_bins.size >= 5:
            suav = dist_bins.copy()
            for i in range(1, dist_bins.size - 1):
                suav[i] = min(dist_bins[i - 1], dist_bins[i], dist_bins[i + 1])
            dist_bins = suav

        libre = dist_bins >= self.cfg.carril_clearance_min

        grupos = []
        i = 0
        while i < libre.size:
            if not libre[i]:
                i += 1
                continue
            j = i
            while j + 1 < libre.size and libre[j + 1]:
                j += 1
            grupos.append((i, j))
            i = j + 1

        if not grupos:
            # Sin hueco claro: escoger el lado con más distancia diagonal, pero suave.
            if self.dist_diag_izq > self.dist_diag_der + 0.05:
                return math.radians(-22.0), 0.0, self.dist_diag_izq
            if self.dist_diag_der > self.dist_diag_izq + 0.05:
                return math.radians(22.0), 0.0, self.dist_diag_der
            return 0.0, 0.0, self.dist_frente

        mejor = None
        mejor_score = -1e9
        for i, j in grupos:
            width = bins[j + 1] - bins[i]
            if math.degrees(width) < self.cfg.carril_gap_min_deg:
                continue
            center = 0.5 * (bins[i] + bins[j + 1])
            clearance = float(np.percentile(dist_bins[i:j + 1], 35))
            # Puntaje Follow-the-Gap: el LiDAR decide el centro del hueco.
            # En curva se reduce casi a cero el sesgo a seguir recto, porque si no
            # el robot pelea contra el hueco lateral y termina dudando.
            modo_curva = (
                self.dist_frente < getattr(self.cfg, "gap_frente_curva", 0.90)
                or abs(self.dist_diag_izq - self.dist_diag_der) > getattr(self.cfg, "gap_diag_diferencia_curva", 0.14)
            )
            bias_frente = self.cfg.carril_bias_frente if not modo_curva else 0.03
            score = (
                3.1 * width +
                0.80 * min(clearance, self.cfg.carril_max_range) -
                bias_frente * abs(center)
            )

            # Si una diagonal está claramente más libre, premiamos el hueco hacia ese lado.
            # center < 0 = izquierda, center > 0 = derecha.
            dif_diag = self.dist_diag_izq - self.dist_diag_der
            if modo_curva and abs(dif_diag) > getattr(self.cfg, "gap_diag_diferencia_curva", 0.14):
                if dif_diag > 0:   # izquierda más libre
                    score += 0.45 * max(0.0, -center)
                else:              # derecha más libre
                    score += 0.45 * max(0.0, center)
            if score > mejor_score:
                mejor_score = score
                mejor = (center, width, clearance)

        if mejor is None:
            return 0.0, 0.0, self.dist_frente

        target, width, clearance = mejor

        # Límite del objetivo angular: deja tomar curvas, pero evita trompos.
        max_target = math.radians(getattr(self.cfg, "gap_target_max_deg", 58.0))
        target = max(-max_target, min(max_target, target))

        # Solo cuando el frente está MUY libre y no hay diferencia diagonal fuerte,
        # se permite suavizar hacia recto. En curvas manda el hueco del LiDAR.
        modo_curva = (
            self.dist_frente < getattr(self.cfg, "gap_frente_curva", 0.90)
            or abs(self.dist_diag_izq - self.dist_diag_der) > getattr(self.cfg, "gap_diag_diferencia_curva", 0.14)
        )
        if (not modo_curva) and self.dist_frente > self.cfg.carril_frente_recto:
            target *= self.cfg.carril_factor_target_con_frente_libre

        return float(target), float(width), float(clearance)

    def control_carril_central(self):
        """Seguidor de hueco libre central.

        No busca una pared. Calcula el centro del espacio libre frontal y gira
        suavemente hacia ese centro. Las paredes laterales solo corrigen si están
        demasiado cerca.
        """
        ahora = time.time()
        dt = max(0.01, ahora - self.t_gap_anterior)

        target_ang, gap_width, clearance = self._calcular_gap_frontal()

        # Suavizado del objetivo del hueco. No usa odometría: solo evita saltos
        # por lecturas puntuales del LiDAR.
        alpha_suav = float(getattr(self.cfg, "gap_suavizado_target", 0.55))
        prev_target = math.radians(self.gap_target_deg) if math.isfinite(self.gap_target_deg) else target_ang
        # Si el frente está cerrándose, bajamos suavizado para reaccionar más rápido.
        if self.dist_frente < getattr(self.cfg, "gap_frente_curva", 0.90):
            alpha_suav *= 0.45
        target_ang = alpha_suav * prev_target + (1.0 - alpha_suav) * target_ang

        # target_ang positivo = hueco hacia derecha; angular positivo gira a izquierda.
        error = -target_ang
        derivada = (error - self.gap_error_anterior) / dt

        w = self.cfg.kp_carril_gap * error + self.cfg.kd_carril_gap * derivada

        # Seguridad lateral suave: si se acerca demasiado a una pared, se aleja.
        if math.isfinite(self.dist_der) and self.dist_der < self.cfg.derecha_muy_cerca + 0.03:
            w += abs(self.cfg.w_lateral_seguridad)
        if math.isfinite(self.dist_izq) and self.dist_izq < self.cfg.izquierda_muy_cerca + 0.03:
            w -= abs(self.cfg.w_lateral_seguridad)

        w = self.saturar(w, self.cfg.max_w_carril_central)

        # Velocidad según frente y giro. En curva no se detiene por dudar:
        # avanza lento pero constante mientras anti-choque no vea peligro real.
        v = self.cfg.vel_carril_central
        if self.dist_frente < self.cfg.frente_alerta:
            v = min(v, max(self.cfg.gap_vel_min_curva, self.cfg.vel_lenta))
        if abs(w) > self.cfg.max_w_carril_central * 0.65:
            v = min(v, max(self.cfg.gap_vel_min_curva, self.cfg.vel_lenta * 1.05))

        self.gap_error_anterior = error
        self.t_gap_anterior = ahora
        self.gap_target_deg = math.degrees(target_ang)
        self.gap_width_deg = math.degrees(gap_width)
        self.gap_clearance = clearance
        self.error_centro_actual = target_ang
        self.error_ang_actual = 0.0

        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def conducir_centrado(self):
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        # Positivo = está más cerca de derecha, corregir a izquierda.
        error_centro = self.dist_izq_pared - self.dist_der_pared
        derivada = (error_centro - self.error_anterior) / dt

        # Orientación promedio de las líneas laterales. Si las paredes están inclinadas,
        # corrige suave sin pegarse a ninguna.
        # Heading real del segmento Split&Merge: mata zigzag y ayuda a quedar paralelo.
        error_ang = 0.5 * (self.alpha_pared_der + self.alpha_pared_izq)

        w = (
            self.cfg.kp_centrado * error_centro
            + self.cfg.kd_centrado * derivada
            - self.cfg.kp_angulo * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular)

        # Si se acerca al frente, bajar velocidad pero no activar rodeo agresivo.
        factor_giro = 1.0 - min(abs(w) / self.cfg.max_angular, 1.0) * 0.45
        v = max(0.035, self.cfg.vel_crucero * factor_giro)
        if self.dist_frente < self.cfg.frente_alerta:
            v = min(v, self.cfg.vel_lenta)

        self.error_anterior = error_centro
        self.t_anterior = ahora
        self.error_centro_actual = error_centro
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def seguir_derecha_suave(self):
        """Fallback: sigue derecha usando SOLO la pared derecha conectada/anclada."""
        target = self.cfg.distancia_derecha_objetivo
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        error = self.dist_der_pared - target
        derivada = (error - self.error_anterior) / dt
        error_ang = self.alpha_pared_der

        # Control tipo CapyGuardian: distancia + heading(alpha) + derivativo.
        # Lejos de derecha -> gira derecha; cerca de derecha -> gira izquierda.
        w = (
            -self.cfg.kp_distancia * error
            -self.cfg.kd_distancia * derivada
            -self.cfg.k_alpha_guardian * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular * 0.80)

        if self.dist_der_pared < self.cfg.derecha_muy_cerca:
            w = abs(self.cfg.max_angular * 0.65)  # abrir a izquierda

        v = self.cfg.vel_lenta if self.dist_frente < self.cfg.frente_alerta else self.cfg.vel_crucero * 0.85
        self.error_anterior = error
        self.t_anterior = ahora
        self.error_centro_actual = 0.0
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def seguir_izquierda_suave(self):
        """Fallback: sigue izquierda si no hay derecha conectada."""
        target = self.cfg.distancia_izquierda_objetivo
        ahora = time.time()
        dt = max(0.01, ahora - self.t_anterior)

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        error = self.dist_izq_pared - target
        derivada = (error - self.error_anterior) / dt
        error_ang = self.alpha_pared_izq

        # Control tipo CapyGuardian: distancia + heading(alpha) + derivativo.
        # Lejos de izquierda -> gira izquierda; cerca de izquierda -> gira derecha.
        w = (
            self.cfg.kp_distancia * error
            + self.cfg.kd_distancia * derivada
            - self.cfg.k_alpha_guardian * error_ang
        )
        w = self.saturar(w, self.cfg.max_angular * 0.80)

        if self.dist_izq_pared < self.cfg.izquierda_muy_cerca:
            w = -abs(self.cfg.max_angular * 0.65)  # abrir a derecha

        v = self.cfg.vel_lenta if self.dist_frente < self.cfg.frente_alerta else self.cfg.vel_crucero * 0.85
        self.error_anterior = error
        self.t_anterior = ahora
        self.error_centro_actual = 0.0
        self.error_ang_actual = error_ang
        return self.cmd_vel(v, self.cfg.signo_giro * w)

    def normalizar_angulo(self, angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    def yaw_girado_desde_inicio(self):
        if self.yaw_inicio_giro is None or not self.tengo_odom:
            return None
        return abs(self.normalizar_angulo(self.odom_yaw - self.yaw_inicio_giro))

    def pared_post_giro_paralela(self):
        """Verifica si ya hay una pared lateral útil y casi paralela.

        Si la referencia está bloqueada, no se acepta el lado contrario como
        condición de salida. Esto evita cortar el giro porque apareció la pared
        izquierda y luego usarla como guía falsa.
        """
        der_ok = self.pared_der_valida and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro
        izq_ok = self.pared_izq_valida and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro

        if self.referencia_bloqueada == "derecha":
            return der_ok or (self.pared_der_valida and self.pared_izq_valida)
        if self.referencia_bloqueada == "izquierda":
            return izq_ok or (self.pared_der_valida and self.pared_izq_valida)

        return der_ok or izq_ok or (self.pared_der_valida and self.pared_izq_valida)

    def distancia_odometrica_desde(self, x0, y0):
        if not self.tengo_odom:
            return None
        return math.hypot(self.odom_x - x0, self.odom_y - y0)

    def valor_por_esquina(self, valores, defecto):
        """Devuelve un ajuste según la esquina actual, usando contador circular."""
        try:
            if not valores:
                return defecto
            idx = int(self.indice_esquina_actual) % len(valores)
            return float(valores[idx])
        except Exception:
            return float(defecto)

    def deteccion_esquina_bloqueada(self):
        """Bloquea reentrada a esquina justo después de girar.

        Esto es lo que evita que en el tramo corto de la segunda esquina el robot
        vuelva a interpretar el frente como otra esquina y acumule giro hasta dar
        una vuelta completa.
        """
        dt = time.time() - self.t_ultima_esquina

        # IMPORTANTE: antes se desbloqueaba si el frente seguía crítico después
        # de 0.35 s. En la 4ta esquina eso provocaba reentradas a GIRO_EVITAR_FRENTE
        # y acumulaba giro hasta casi media vuelta. Ahora se bloquea por tiempo
        # y por distancia recorrida sin excepciones por frente crítico.
        if dt < self.cfg.cooldown_esquina:
            return True

        if dt < self.cfg.bloqueo_reentrada_esquina_seg:
            return True

        if self.tengo_odom and self.contador_esquinas > 0:
            dist = self.distancia_odometrica_desde(self.x_fin_ultima_esquina, self.y_fin_ultima_esquina)
            if dist is not None and dist < self.cfg.bloqueo_reentrada_esquina_m:
                return True

        return False

    def registrar_esquina_completada(self):
        if not self.esquina_en_proceso:
            return
        self.esquina_en_proceso = False
        self.contador_esquinas += 1
        self.t_ultima_esquina = time.time()
        self.x_fin_ultima_esquina = self.odom_x
        self.y_fin_ultima_esquina = self.odom_y
        self.get_logger().info(
            f"Esquina completada #{self.contador_esquinas} | próxima índice {(self.contador_esquinas % 4) + 1}"
        )

    def iniciar_avance_post_esquina(self):
        self.t_inicio_avance_post_esquina = time.time()
        self.x_inicio_avance_post_esquina = self.odom_x
        self.y_inicio_avance_post_esquina = self.odom_y
        self.yaw_salida_post_esquina = self.odom_yaw if self.tengo_odom else None
        self.sentido_salida_post_esquina = self.sentido_giro_frente
        self.indice_salida_post_esquina = self.indice_esquina_actual
        self.cambiar_estado("AVANZAR_POST_ESQUINA")

    def preparar_maniobra_esquina(self):
        """Prepara la esquina sin girar de golpe.

        El robot primero se acerca hasta una distancia fija y luego gira lentamente.
        En sentido antihorario siempre se toma la esquina hacia la izquierda.
        """
        if self.estado_actual in {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}:
            return

        self.indice_esquina_actual = self.contador_esquinas % 4
        self.esquina_en_proceso = True
        self.bloquear_referencia_para_esquina()
        self.get_logger().info(f"Preparando esquina #{self.indice_esquina_actual + 1}")

        if self.cfg.sentido_circuito_antihorario:
            self.sentido_giro_frente = self.cfg.sentido_giro_esquina  # +1 izquierda
        elif self.frente_conecta_der and not self.frente_conecta_izq:
            self.sentido_giro_frente = 1.0   # frontal conecta con derecha -> girar izquierda
        elif self.frente_conecta_izq and not self.frente_conecta_der:
            self.sentido_giro_frente = -1.0  # frontal conecta con izquierda -> girar derecha
        elif self.ultimo_lado_pared == "izquierda":
            self.sentido_giro_frente = 1.0
        elif self.ultimo_lado_pared == "derecha":
            self.sentido_giro_frente = -1.0
        else:
            self.sentido_giro_frente = self.cfg.sentido_giro_esquina

        self.yaw_inicio_giro = None
        self.error_anterior = 0.0
        self.t_anterior = time.time()
        self.cambiar_estado("ACERCAR_ESQUINA")

    # Compatibilidad con versiones anteriores del código.
    def preparar_giro_por_frente(self):
        self.preparar_maniobra_esquina()

    def acercar_esquina(self):
        """Se acerca lento a la esquina antes de girar.

        Si en realidad era pasadizo, vuelve al centrado. Si sí es esquina,
        se detiene a una distancia segura y recién ahí gira.
        """
        if self.frente_es_pasillo and self.dist_frente > self.cfg.frente_pasillo_libre:
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        if self.dist_frente <= self.cfg.distancia_detencion_esquina or self.dist_frente <= self.cfg.frente_critico:
            self.t_inicio_giro_frente = time.time()
            self.yaw_inicio_giro = self.odom_yaw if self.tengo_odom else None
            self.cambiar_estado("GIRO_EVITAR_FRENTE")
            return self.girar_por_frente()

        # Acercamiento lento respetando el bloqueo de referencia.
        # Si el bloqueo es derecha, no usamos izquierda sola como guía aunque aparezca.
        if self.bloqueo_referencia_activo():
            ref = self.referencia_bloqueada
            if self.pared_der_valida and self.pared_izq_valida:
                cmd = self.conducir_centrado()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            if ref == "derecha" and self.pared_der_valida:
                cmd = self.seguir_derecha_suave()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            if ref == "izquierda" and self.pared_izq_valida:
                cmd = self.seguir_izquierda_suave()
                cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
                cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.35)
                return cmd
            # Referencia perdida: avanza recto lento, sin girar hacia la pared contraria.
            return self.cmd_vel(self.cfg.vel_acercar_esquina, 0.0)

        # Acercamiento lento manteniendo el centro o la pared visible.
        if self.pared_der_valida and self.pared_izq_valida:
            cmd = self.conducir_centrado()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        if self.pared_der_valida:
            cmd = self.seguir_derecha_suave()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        if self.pared_izq_valida:
            cmd = self.seguir_izquierda_suave()
            cmd.linear.x = min(cmd.linear.x, self.cfg.vel_acercar_esquina * self.factor_velocidad)
            cmd.angular.z = self.saturar(cmd.angular.z, self.cfg.max_angular_segura * 0.45)
            return cmd

        return self.cmd_vel(self.cfg.vel_acercar_esquina, 0.0)

    def girar_por_frente(self):
        """Giro controlado de esquina.

        Ya no depende solo del tiempo ni sigue girando indefinidamente. Sale por:
        1) odometría: llegó al giro objetivo o al máximo permitido,
        2) LiDAR: volvió a ver pasadizo/pared lateral paralela,
        3) tiempo máximo de respaldo.
        """
        tiempo_girando = time.time() - self.t_inicio_giro_frente
        yaw_girado = self.yaw_girado_desde_inicio()
        yaw_obj = math.radians(self.valor_por_esquina(
            self.cfg.yaw_objetivo_esquinas_deg,
            self.cfg.yaw_objetivo_giro_esquina_deg
        ))
        yaw_max = math.radians(self.valor_por_esquina(
            self.cfg.yaw_max_esquinas_deg,
            self.cfg.yaw_max_giro_esquina_deg
        ))
        max_tiempo_giro = self.valor_por_esquina(
            self.cfg.max_tiempo_giro_esquinas,
            self.cfg.max_tiempo_giro_frente
        )
        factor_giro_actual = self.valor_por_esquina(self.cfg.factor_giro_esquinas, 1.0)

        hay_lateral_util = (
            self.pared_der_valida or self.pared_izq_valida or
            self.dist_der < self.cfg.lateral_reenganche_max or
            self.dist_izq < self.cfg.lateral_reenganche_max
        )

        pasadizo_recuperado = (
            self.frente_es_pasillo and
            self.dist_frente > self.cfg.frente_salida_libre and
            (self.pared_der_valida or self.pared_izq_valida)
        )

        paralelo_recuperado = (
            hay_lateral_util and
            self.dist_frente > self.cfg.frente_salida_con_lateral and
            self.pared_post_giro_paralela()
        )

        # Salida temprana: no espera a que el frente quede muy libre.
        # Si ya giró el objetivo y no está en distancia crítica, sale a alinear.
        frente_no_critico = self.dist_frente > self.cfg.frente_critico
        llego_objetivo = yaw_girado is not None and yaw_girado >= yaw_obj and frente_no_critico
        llego_maximo = yaw_girado is not None and yaw_girado >= yaw_max
        excedio_tiempo = yaw_girado is None and tiempo_girando > max_tiempo_giro

        # Si llegó al máximo o al tiempo máximo, NO se permite seguir corrigiendo
        # con giro grande. Sale directo a avance post-esquina para evitar media vuelta.
        if llego_maximo or excedio_tiempo:
            self.registrar_esquina_completada()
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        if pasadizo_recuperado or paralelo_recuperado or llego_objetivo:
            self.registrar_esquina_completada()
            self.t_inicio_alinear_post_giro = time.time()
            self.lateral_estable_post_giro = 0
            self.cambiar_estado("ALINEAR_POST_GIRO")
            return self.alinear_post_giro()

        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0

        # Gira lento. Avanza muy poco solo si no está demasiado pegado al frente.
        v = self.cfg.vel_avance_esquina if self.dist_frente > self.cfg.frente_min_avance_giro else 0.0
        w = self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * factor_giro_actual
        return self.cmd_vel(v, w)

    def alinear_post_giro(self):
        """Corrige la posición después del giro sin permitir otra vuelta completa."""
        tiempo_alineando = time.time() - self.t_inicio_alinear_post_giro

        estable = False
        if self.pared_der_valida and self.pared_izq_valida and self.dist_frente > self.cfg.frente_critico:
            estable = True
        elif self.referencia_bloqueada == "derecha":
            estable = (
                self.pared_der_valida
                and self.dist_frente > self.cfg.frente_critico
                and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro
            )
        elif self.referencia_bloqueada == "izquierda":
            estable = (
                self.pared_izq_valida
                and self.dist_frente > self.cfg.frente_critico
                and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro
            )
        elif self.pared_izq_valida and self.dist_frente > self.cfg.frente_critico and abs(self.pendiente_pared_izq) < self.cfg.tolerancia_paralelo_post_giro:
            estable = True
        elif self.pared_der_valida and self.dist_frente > self.cfg.frente_critico and abs(self.pendiente_pared_der) < self.cfg.tolerancia_paralelo_post_giro:
            estable = True

        if estable:
            self.lateral_estable_post_giro += 1
        else:
            self.lateral_estable_post_giro = 0

        # Debe ver lateral estable algunos ciclos antes de soltar el giro.
        if self.lateral_estable_post_giro >= 3:
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        # Corrección corta: no reinicia el giro grande.
        # En la 4ta esquina NO seguimos girando en el mismo sentido; salimos
        # recto y dejamos que el avance protegido estabilice el LiDAR.
        if self.indice_esquina_actual == 3:
            self.iniciar_avance_post_esquina()
            return self.avanzar_post_esquina()

        factor_alinear = min(1.0, self.valor_por_esquina(self.cfg.factor_giro_esquinas, 1.0))
        if tiempo_alineando < self.cfg.max_tiempo_alinear_post_giro:
            if self.dist_frente < self.cfg.frente_critico:
                # Si aún ve pared al frente, no aumentes el giro: contravolante muy suave.
                return self.cmd_vel(0.0, -self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * 0.10 * factor_alinear)
            return self.cmd_vel(self.cfg.vel_lenta, self.cfg.signo_giro * self.sentido_giro_frente * self.cfg.vel_giro_esquina * 0.08 * factor_alinear)

        # Si no logró alinear en poco tiempo, igual sale a avance post-esquina.
        # Así no se queda girando hasta completar una vuelta.
        self.iniciar_avance_post_esquina()
        return self.avanzar_post_esquina()

    def avanzar_post_esquina(self):
        """Avanza protegido después de cada esquina antes de detectar otra.

        Arreglo para la 4ta esquina:
        - no vuelve a lanzar GIRO_EVITAR_FRENTE mientras está saliendo,
        - no usa todavía seguir pared/centrar porque eso puede interpretar mal
          la pared frontal como referencia lateral,
        - mantiene el rumbo con odometría si existe,
        - si todavía ve frente crítico, hace contravolante suave en vez de seguir
          girando hacia la esquina.
        """
        dt = time.time() - self.t_inicio_avance_post_esquina
        dist = self.distancia_odometrica_desde(
            self.x_inicio_avance_post_esquina,
            self.y_inicio_avance_post_esquina
        )

        # La 4ta esquina necesita una salida protegida un poco más larga porque
        # la pared de la caja queda muy cerca y el LiDAR la vuelve a leer como frente.
        min_m = self.cfg.avance_post_esquina_min_m
        min_seg = self.cfg.avance_post_esquina_min_seg
        max_seg = self.cfg.avance_post_esquina_max_seg
        if self.indice_salida_post_esquina == 3:
            min_m = max(min_m, self.cfg.avance_post_esquina4_min_m)
            min_seg = max(min_seg, self.cfg.avance_post_esquina4_min_seg)
            max_seg = max(max_seg, self.cfg.avance_post_esquina4_max_seg)

        avance_ok = False
        if dist is not None and dist >= min_m:
            avance_ok = True
        if dt >= min_seg and dist is None:
            avance_ok = True
        if dt >= max_seg:
            avance_ok = True

        if avance_ok:
            self.liberar_bloqueo_referencia("avance post-esquina completado")
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        # Durante la salida obligatoria se ignora la detección de esquina.
        # Se avanza casi recto y solo se corrige rumbo.
        w_hold = 0.0
        if self.yaw_salida_post_esquina is not None and self.tengo_odom:
            yaw_err = self.normalizar_angulo(self.yaw_salida_post_esquina - self.odom_yaw)
            w_hold = self.saturar(self.cfg.kp_yaw_salida_post_esquina * yaw_err,
                                  self.cfg.max_w_salida_post_esquina)

        v = self.cfg.vel_avance_post_esquina

        # Si queda muy cerca del frente justo al salir, no volver a girar hacia
        # la izquierda. Hacer contravolante pequeño para no encarar la pared.
        if self.dist_frente < self.cfg.frente_critico:
            v = min(v, self.cfg.vel_lenta * 0.55)
            w_hold += -self.cfg.signo_giro * self.sentido_salida_post_esquina * self.cfg.contravolante_salida_esquina
        elif self.dist_frente < self.cfg.frente_alerta:
            v = min(v, self.cfg.vel_lenta * 0.80)

        w_hold = self.saturar(w_hold, self.cfg.max_w_salida_post_esquina)
        return self.cmd_vel(v, w_hold)

    def buscar_referencia_lateral(self):
        """
        Si no hay paredes laterales conectadas, gira suave a la derecha.
        Importante: no usa la pared frontal como derecha porque radar_utils exige
        cluster conectado al sector lateral ±90°.
        """
        if self.pared_der_valida or self.pared_izq_valida:
            self.cambiar_estado("CENTRAR_PASILLO")
            return self.navegar_por_centro()

        if self.dist_frente < self.cfg.frente_critico:
            if self.deteccion_esquina_bloqueada():
                self.cambiar_estado("ALINEAR_POST_GIRO")
                return self.alinear_post_giro()
            self.preparar_maniobra_esquina()
            return self.acercar_esquina()

        self.error_centro_actual = 0.0
        self.error_ang_actual = 0.0
        # En antihorario conviene buscar la referencia por la izquierda
        # (isla central). En horario se conserva la búsqueda a la derecha.
        sentido_busqueda = 1.0 if self.cfg.sentido_circuito_antihorario else -1.0
        return self.cmd_vel(
            self.cfg.vel_busqueda_derecha,
            self.cfg.signo_giro * sentido_busqueda * self.cfg.giro_busqueda_derecha
        )

    # ----------------------------------------------------------
    # Rodeo de caja: se conserva, pero ya no se dispara por cualquier pared frontal.
    # ----------------------------------------------------------
    def iniciar_rodeo(self):
        self.rodeo_caja.iniciar(self.cambiar_estado)

    def rodear_obstaculo(self):
        v, w = self.rodeo_caja.actualizar(
            self.estado_actual,
            self.dist_frente,
            self.cambiar_estado
        )
        return self.cmd_vel(v, w)

    # ==========================================================
    # UTILIDADES DE CONTROL
    # ==========================================================
    def cambiar_estado(self, nuevo_estado):
        if self.estado_actual != nuevo_estado:
            self.estado_actual = nuevo_estado
            self.t_estado = time.time()
            self.ciclos_estable = 0
            if nuevo_estado not in {"CENTRAR_PASILLO", "CARRIL_CENTRAL", "SEGUIR_PARED_DERECHA_SUAVE", "SEGUIR_PARED_IZQUIERDA_SUAVE", "REFERENCIA_BLOQUEADA", "ASPIRADORA_AVANZAR", "ASPIRADORA_SEGUIR_PARED", "ASPIRADORA_GIRAR", "ASPIRADORA_RETROCEDER", "LAB_AVANZAR", "LAB_SEGUIR_IZQUIERDA", "LAB_SEGUIR_DERECHA_RESPALDO", "LAB_GIRAR", "LAB_SALIDA_GIRO", "LAB_RETROCEDER", "LAB_ESCAPE_ATORADO", "PARAR_PARE"}:
                self.reset_persistencia()
            self.get_logger().info(f"Estado -> {self.estado_actual}")

    def _distancia_lateral_segura(self, lado):
        """Devuelve una distancia lateral conservadora para anti-choque.

        Usa la distancia simple del sector lateral y, si hay pared validada,
        también la distancia de pared. Se toma el mínimo finito para no ignorar
        una pared cercana aunque el clasificador todavía no la haya validado.
        """
        vals = []
        if lado == "derecha":
            vals = [self.dist_der]
            if self.pared_der_valida:
                vals.append(self.dist_der_pared)
        elif lado == "izquierda":
            vals = [self.dist_izq]
            if self.pared_izq_valida:
                vals.append(self.dist_izq_pared)
        finitos = [float(v) for v in vals if math.isfinite(float(v)) and float(v) > 0.0]
        return min(finitos) if finitos else float('inf')

    def _lado_mas_libre_para_escapar(self):
        """Elige hacia dónde girar cuando el frente se cierra.

        angular.z positivo = giro a la izquierda.
        Si hay más espacio a izquierda, retorna +1. Si hay más espacio a derecha, -1.
        """
        izq = self._distancia_lateral_segura("izquierda")
        der = self._distancia_lateral_segura("derecha")
        diag_izq = self.dist_diag_izq if math.isfinite(self.dist_diag_izq) else izq
        diag_der = self.dist_diag_der if math.isfinite(self.dist_diag_der) else der
        score_izq = min(izq, diag_izq)
        score_der = min(der, diag_der)
        return 1.0 if score_izq >= score_der else -1.0

    def aplicar_antichoque_global(self, v, w):
        """Capa de seguridad superior basada en LiDAR.

        Se aplica a TODOS los estados antes de publicar velocidad.
        Regla: ninguna lógica puede ordenar avance si frente/lados están en zona crítica.
        """
        if not getattr(self.cfg, "usar_antichoque_global", True):
            self.antichoque_activo = False
            self.antichoque_motivo = "OFF"
            return v, w

        v = float(v)
        w = float(w)
        motivos = []

        frente = self.dist_frente if math.isfinite(self.dist_frente) else float('inf')
        der = self._distancia_lateral_segura("derecha")
        izq = self._distancia_lateral_segura("izquierda")
        diag_der = self.dist_diag_der if math.isfinite(self.dist_diag_der) else float('inf')
        diag_izq = self.dist_diag_izq if math.isfinite(self.dist_diag_izq) else float('inf')

        # 1) Protección frontal: si el frente está muy cerca, jamás avanzar.
        if frente <= self.cfg.frente_emergencia:
            lado_escape = self._lado_mas_libre_para_escapar()
            v = min(v, 0.0)
            # Si ya estaba girando hacia el lado libre, respétalo; si no, fuerza escape.
            if abs(w) < self.cfg.w_escape_frontal * 0.60 or (w * lado_escape) < 0:
                w = self.cfg.signo_giro * lado_escape * self.cfg.w_escape_frontal
            motivos.append("FRENTE_EMERGENCIA")
        elif frente <= self.cfg.frente_stop_global:
            lado_escape = self._lado_mas_libre_para_escapar()
            v = min(v, 0.0)
            if abs(w) < self.cfg.w_escape_frontal * 0.45 or (w * lado_escape) < 0:
                w = self.cfg.signo_giro * lado_escape * self.cfg.w_escape_frontal * 0.75
            motivos.append("FRENTE_STOP")
        elif frente <= self.cfg.frente_freno_suave and v > self.cfg.vel_max_freno_suave:
            v = self.cfg.vel_max_freno_suave
            motivos.append("FRENTE_LENTO")

        # 2) Protección lateral y diagonal: alejarse del lado cercano.
        # Derecha cerca => giro izquierda positivo. Izquierda cerca => giro derecha negativo.
        if der <= self.cfg.lateral_stop_global or diag_der <= self.cfg.diagonal_stop_global:
            v = min(v, self.cfg.vel_max_lateral_cerca)
            w = max(w, self.cfg.signo_giro * abs(self.cfg.w_escape_lateral))
            motivos.append("DERECHA_CERCA")
        elif der <= self.cfg.lateral_freno_suave:
            v = min(v, max(self.cfg.vel_max_lateral_cerca, self.cfg.vel_lenta))
            w += self.cfg.signo_giro * abs(self.cfg.w_escape_lateral) * 0.45
            motivos.append("DERECHA_LENTA")

        if izq <= self.cfg.lateral_stop_global or diag_izq <= self.cfg.diagonal_stop_global:
            v = min(v, self.cfg.vel_max_lateral_cerca)
            w = min(w, -self.cfg.signo_giro * abs(self.cfg.w_escape_lateral))
            motivos.append("IZQUIERDA_CERCA")
        elif izq <= self.cfg.lateral_freno_suave:
            v = min(v, max(self.cfg.vel_max_lateral_cerca, self.cfg.vel_lenta))
            w -= self.cfg.signo_giro * abs(self.cfg.w_escape_lateral) * 0.45
            motivos.append("IZQUIERDA_LENTA")

        # 3) Si ambos lados están muy cerrados, no acelerar: seguir muy lento o detener.
        if der <= self.cfg.lateral_stop_global and izq <= self.cfg.lateral_stop_global:
            v = min(v, 0.0)
            w = 0.0
            motivos.append("PASO_MUY_ESTRECHO")

        self.antichoque_activo = bool(motivos)
        self.antichoque_motivo = "+".join(motivos) if motivos else "OK"
        return v, self.saturar(w, self.cfg.max_angular_segura)

    def cmd_vel(self, v, w):
        """
        Crea Twist aplicando:
        - anti-choque global antes de publicar
        - factor_velocidad solo a lineal
        - factor_angular solo a angular
        """
        cmd = Twist()

        v_base = float(v)
        w_base = float(w)

        # Memoria de recorrido: si todavía está en la primera vuelta y detecta que
        # está volviendo sobre sus propios pasos, aplica una corrección SUAVE.
        # Cuando el mapper detecta una vuelta completa, permite pasar otra vez.
        if getattr(self.cfg, "usar_antiretorno_mapeo", False):
            correccion = self.mapa_ruta.obtener_correccion_antiretorno(self.odom_x, self.odom_y, self.odom_yaw)
            self.evitar_retorno_activo = bool(correccion.get('activo', False))
            if self.evitar_retorno_activo and self.estado_actual not in {"ACERCAR_ESQUINA", "GIRO_EVITAR_FRENTE", "ALINEAR_POST_GIRO", "AVANZAR_POST_ESQUINA"}:
                w_base += self.cfg.signo_giro * correccion.get('bias_angular', 0.0)
                v_base *= self.cfg.factor_velocidad_antiretorno
        else:
            self.evitar_retorno_activo = False

        # Anti-choque global: última decisión antes de aplicar factores GUI.
        # Así ninguna lógica puede mandar avance cuando el LiDAR ve pared/caja cerca.
        v_base, w_base = self.aplicar_antichoque_global(v_base, w_base)

        v_final = v_base * self.factor_velocidad
        w_final = w_base * self.factor_angular

        v_final = self.saturar(v_final, self.cfg.max_lineal_segura)
        w_final = self.saturar(w_final, self.cfg.max_angular_segura)

        cmd.linear.x = v_final
        cmd.angular.z = w_final
        self.vel_lineal = cmd.linear.x
        self.vel_angular = cmd.angular.z
        return cmd

    def saturar(self, valor, limite):
        return max(-limite, min(limite, valor))

    def detener_robot(self):
        cmd = Twist()
        self.vel_lineal = 0.0
        self.vel_angular = 0.0
        self.publisher.publish(cmd)

    def timer_seguridad_callback(self):
        if self.solicitud_salir or not self.robot_habilitado or self.robot_pausado:
            self.detener_robot()
            return

        # Si la cámara confirmó PARE entre dos lecturas del LiDAR, este timer
        # fuerza el stop sin meter lógica extra ni ralentizar la navegación.
        if self.pare_en_detencion_activa():
            self.aplicar_parada_pare_si_corresponde()
            return
        if getattr(self, "meta_vision_activa", False) or getattr(self, "meta_vision_finalizada", False):
            self.aplicar_meta_si_corresponde()

    # ==========================================================
    # REPORTE / PLOT AL PAUSAR
    # ==========================================================
    def actualizar_trayectoria_plot(self):
        """Integra cmd_vel únicamente para dibujar la ruta al pausar.

        Esta función NO cambia estados, NO cambia velocidades y NO interviene
        en la lógica del robot. Solo guarda puntos para el reporte gráfico.
        """
        ahora = time.time()
        if self.plot_last_t is None:
            self.plot_last_t = ahora
            return

        dt = ahora - self.plot_last_t
        self.plot_last_t = ahora

        # Evita saltos si la Raspberry se congela o la ventana queda bloqueada.
        if dt <= 0.0 or dt > 0.60:
            return

        v = float(self.vel_lineal)
        w = float(self.vel_angular)

        if not math.isfinite(v) or not math.isfinite(w):
            return

        # Modelo diferencial simple: suficiente para graficar la forma del recorrido.
        yaw_mid = self.plot_yaw + 0.5 * w * dt
        dx = v * math.cos(yaw_mid) * dt
        dy = v * math.sin(yaw_mid) * dt
        self.plot_x += dx
        self.plot_y += dy
        self.plot_yaw = math.atan2(math.sin(self.plot_yaw + w * dt), math.cos(self.plot_yaw + w * dt))
        self.plot_total_distance += abs(v) * dt

        if not self.plot_path:
            self.plot_path.append((self.plot_x, self.plot_y))
            return

        ux, uy = self.plot_path[-1]
        if math.hypot(self.plot_x - ux, self.plot_y - uy) >= 0.015:
            self.plot_path.append((self.plot_x, self.plot_y))

    def distancia_de_path(self, path):
        total = 0.0
        if len(path) < 2:
            return 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        return total

    def registrar_historial_control(self):
        """Guarda muestras ligeras para graficar al presionar PAUSAR."""
        try:
            self.actualizar_trayectoria_plot()
            self.historial_control.append({
                "t": time.time(),
                "x": float(self.odom_x),
                "y": float(self.odom_y),
                "yaw": float(self.odom_yaw),
                "x_plot": float(self.plot_x),
                "y_plot": float(self.plot_y),
                "yaw_plot": float(self.plot_yaw),
                "v": float(self.vel_lineal),
                "w": float(self.vel_angular),
                "frente": float(self.dist_frente),
                "error_centro": float(self.error_centro_actual),
                "error_ang": float(self.error_ang_actual),
                "estado": str(self.estado_actual),
                "cajas": int(self.contador_cajas),
                "esquinas": int(self.contador_esquinas),
            })
        except Exception:
            # El historial nunca debe interrumpir el control del robot.
            pass

    def _promedio_historial(self, clave, absoluto=False):
        vals = []
        for item in self.historial_control:
            try:
                val = float(item.get(clave, float("nan")))
            except Exception:
                continue
            if math.isfinite(val):
                vals.append(abs(val) if absoluto else val)
        if not vals:
            return 0.0
        return sum(vals) / len(vals)

    def _max_historial(self, clave, absoluto=False):
        vals = []
        for item in self.historial_control:
            try:
                val = float(item.get(clave, float("nan")))
            except Exception:
                continue
            if math.isfinite(val):
                vals.append(abs(val) if absoluto else val)
        if not vals:
            return 0.0
        return max(vals)


    # ==========================================================
    # APRENDIZAJE PASIVO — EXPORTAR MAPA SIN CAMBIAR NAVEGACIÓN
    # ==========================================================
    def _json_limpio(self, obj):
        """Convierte numpy/tuplas/inf a tipos seguros para JSON."""
        try:
            if isinstance(obj, dict):
                return {str(k): self._json_limpio(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple, deque)):
                return [self._json_limpio(v) for v in obj]
            if isinstance(obj, (np.integer,)):
                return int(obj)
            if isinstance(obj, (np.floating,)):
                obj = float(obj)
            if isinstance(obj, float):
                return obj if math.isfinite(obj) else None
            if isinstance(obj, (int, str, bool)) or obj is None:
                return obj
        except Exception:
            return str(obj)
        return str(obj)

    def _carpeta_aprendizaje(self):
        carpeta = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "mapas_aprendidos"))
        os.makedirs(carpeta, exist_ok=True)
        return carpeta

    def _path_exportacion_aprendizaje(self):
        """Ruta principal del mapa aprendido.

        Si el mapeo interno todavía usa cmd_vel estimado, exportamos esa ruta
        porque es la misma que el reporte muestra. Esto no modifica el control.
        """
        mapa = self.mapa_ruta.obtener_datos_mapa()
        odom_path = list(mapa.get("path", []))
        cmd_path = list(self.plot_path)
        odom_dist = float(mapa.get("total_distance", 0.0) or 0.0)
        cmd_dist = float(self.plot_total_distance)
        odom_span = self.distancia_de_path(odom_path)
        usar_cmd = (len(odom_path) < 2 or odom_dist < 0.05 or odom_span < 0.05) and len(cmd_path) >= 2 and cmd_dist > 0.03
        if usar_cmd:
            return [(float(x), float(y)) for x, y in cmd_path], "cmd_vel estimado", cmd_dist
        return [(float(x), float(y)) for x, y in odom_path], str(mapa.get("map_source", "odom")), odom_dist

    def _crear_paquete_mapa_aprendido(self):
        mapa = self.mapa_ruta.obtener_datos_mapa()
        resumen = self.mapa_ruta.obtener_estado_resumen()
        path, fuente_path, distancia = self._path_exportacion_aprendizaje()
        occupied = list(mapa.get("occupied_centers", []))
        free = list(mapa.get("free_centers", []))
        visited = list(mapa.get("visited_centers", []))
        grid_res = float(mapa.get("resolution", 0.10) or 0.10)
        segmentos = self._segmentos_laberinto_desde_ocupacion(occupied, grid_res)
        intersecciones = []
        for item in list(getattr(self, "lab_intersecciones", [])):
            try:
                x, y, decision, t = item
                intersecciones.append({"x": float(x), "y": float(y), "decision": str(decision), "t": float(t)})
            except Exception:
                pass
        callejones = []
        for item in list(getattr(self, "lab_callejones", [])):
            try:
                x, y, t = item
                callejones.append({"x": float(x), "y": float(y), "t": float(t)})
            except Exception:
                pass

        return {
            "version": "capybot_w_aprendizaje_pasivo",
            "fecha": datetime.now().isoformat(timespec="seconds"),
            "nota": "Mapa guardado de forma pasiva; no modifica la navegación del robot.",
            "fuente_recorrido": fuente_path,
            "distancia_recorrida_m": float(distancia),
            "resumen": resumen,
            "pose_actual": {
                "x": float(self.plot_x),
                "y": float(self.plot_y),
                "yaw": float(self.plot_yaw),
                "odom_x": float(self.odom_x),
                "odom_y": float(self.odom_y),
                "odom_yaw": float(self.odom_yaw),
            },
            "meta_estimada": {
                "criterio": "META oficial = esquina superior derecha conocida del reto",
                "x": float(getattr(self, "meta_x", 3.60)),
                "y": float(getattr(self, "meta_y", 2.40)),
                "radio_m": float(getattr(self, "meta_radio", 0.48)),
                "distancia_actual_m": float(getattr(self, "meta_distancia", float('inf'))),
                "robot_cree_llegar": bool(getattr(self, "meta_cree_llegar", False)),
                "eventos": list(getattr(self, "meta_eventos", [])),
                "camara_verde": {
                    "detectadas": int(getattr(self, "meta_vision_detectadas", 0)),
                    "activa": bool(getattr(self, "meta_vision_activa", False)),
                    "finalizada": bool(getattr(self, "meta_vision_finalizada", False)),
                    "area_ratio": float(getattr(self, "meta_vision_area_ratio", 0.0)),
                    "error_x": float(getattr(self, "meta_vision_error_x", 0.0)),
                },
            },
            "grid": {
                "resolution_m": grid_res,
                "occupied_cells": mapa.get("occupied_cells", []),
                "free_cells": mapa.get("free_cells", []),
                "visited_cells": mapa.get("visited_cells", []),
                "occupied_centers": occupied,
                "free_centers": free,
                "visited_centers": visited,
            },
            "paredes_segmentos": [
                {"x1": float(a), "y1": float(b), "x2": float(c), "y2": float(d), "orientacion": str(o), "celdas": int(n)}
                for a, b, c, d, o, n in segmentos
            ],
            "ruta_recorrida": [{"i": i, "x": float(x), "y": float(y)} for i, (x, y) in enumerate(path)],
            "pares_detectados": list(getattr(self, "pare_eventos", [])),
            "intersecciones": intersecciones,
            "callejones": callejones,
            "metricas": {
                "pare_detenciones": int(self.pare_detenciones),
                "pare_falsos": int(self.pare_falsos),
                "cajas_detectadas": int(self.contador_cajas),
                "esquinas_completadas": int(self.contador_esquinas),
                "estado_actual": str(self.estado_actual),
                "referencia_estable": str(self.referencia_estable),
            },
        }

    def _guardar_csv_simple(self, ruta, filas, campos):
        with open(ruta, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=campos)
            writer.writeheader()
            for row in filas:
                writer.writerow({c: row.get(c, "") for c in campos})

    def _guardar_png_mapa_aprendido(self, paquete, ruta_png):
        try:
            fig, ax = plt.subplots(figsize=(9.0, 6.2), facecolor="#f6f8fa")
            ax.set_facecolor("#eef4f7")
            grid = paquete.get("grid", {})
            free = grid.get("free_centers", [])[-5500:]
            visited = grid.get("visited_centers", [])[-2600:]
            segmentos = paquete.get("paredes_segmentos", [])
            ruta = paquete.get("ruta_recorrida", [])
            pares = paquete.get("pares_detectados", [])
            inter = paquete.get("intersecciones", [])
            dead = paquete.get("callejones", [])

            if free:
                fx = [p[0] for p in free]; fy = [p[1] for p in free]
                ax.scatter(fx, fy, s=14, marker="s", c="#ffffff", alpha=0.55, linewidths=0, label="libre")
            if visited:
                vx = [p[0] for p in visited]; vy = [p[1] for p in visited]
                ax.scatter(vx, vy, s=18, marker="s", c="#d8f3dc", alpha=0.65, linewidths=0, label="visitado")
            first = True
            for seg in segmentos[:1100]:
                ax.plot([seg["x1"], seg["x2"]], [seg["y1"], seg["y2"]],
                        color="#9a4f12", linewidth=3.2, solid_capstyle="round",
                        label="pared" if first else None)
                first = False
            if ruta:
                rx = [p["x"] for p in ruta]; ry = [p["y"] for p in ruta]
                ax.plot(rx, ry, color="#00a884", linewidth=3.0, label="ruta aprendida", zorder=7)
                ax.scatter([rx[0]], [ry[0]], s=90, c="#1565c0", marker="o", label="inicio", zorder=8)
                ax.scatter([rx[-1]], [ry[-1]], s=95, c="#ff9800", marker="s", label="última pose", zorder=8)
            if pares:
                px = [float(p.get("x", 0.0)) for p in pares]
                py = [float(p.get("y", 0.0)) for p in pares]
                ax.scatter(px, py, s=120, c="#d32f2f", marker="8", label="PARE", zorder=9)
            try:
                meta = paquete.get("meta_estimada", {})
                mx = float(meta.get("x", 3.60)); my = float(meta.get("y", 2.40))
                ax.scatter([mx], [my], s=190, c="#43a047", marker="*", label="META estimada", zorder=10)
                ax.text(mx + 0.05, my + 0.05, "META", color="#2e7d32", fontsize=10, weight="bold")
            except Exception:
                pass
            if inter:
                ix = [float(p.get("x", 0.0)) for p in inter]
                iy = [float(p.get("y", 0.0)) for p in inter]
                ax.scatter(ix, iy, s=55, c="#26a69a", marker="D", label="intersección", zorder=8)
            if dead:
                dx = [float(p.get("x", 0.0)) for p in dead]
                dy = [float(p.get("y", 0.0)) for p in dead]
                ax.scatter(dx, dy, s=70, c="#fdd835", marker="x", label="callejón", zorder=8)

            ax.set_title("Mapa aprendido del laberinto — exportación pasiva", fontsize=13, weight="bold", color="#263238")
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            ax.grid(True, linestyle="--", alpha=0.28, color="#90a4ae")
            try:
                pts = []
                pts += [(p[0], p[1]) for p in free]
                pts += [(p[0], p[1]) for p in visited]
                pts += [(p["x"], p["y"]) for p in ruta]
                for seg in segmentos:
                    pts.append((seg["x1"], seg["y1"])); pts.append((seg["x2"], seg["y2"]))
                if pts:
                    xs = [float(a) for a, _ in pts]; ys = [float(b) for _, b in pts]
                    span = max(max(xs)-min(xs), max(ys)-min(ys), 1.0)
                    pad = max(0.45, span * 0.10)
                    ax.set_xlim(min(xs)-pad, max(xs)+pad); ax.set_ylim(min(ys)-pad, max(ys)+pad)
            except Exception:
                pass
            ax.set_aspect("equal", adjustable="box")
            ax.legend(loc="upper left", fontsize=8, framealpha=0.92)
            fig.tight_layout()
            fig.savefig(ruta_png, dpi=160, facecolor=fig.get_facecolor(), bbox_inches="tight")
            plt.close(fig)
        except Exception as e:
            try:
                self.get_logger().warn(f"No se pudo guardar PNG del mapa aprendido: {e}")
            except Exception:
                pass

    def guardar_mapa_aprendido(self, motivo="pausa"):
        """Guarda mapa/ruta/PARE/intersecciones sin cambiar la navegación.

        Se ejecuta al pausar o salir. Es una capa de aprendizaje pasiva para
        usar luego en una segunda corrida con A* o Dijkstra.
        """
        try:
            carpeta = self._carpeta_aprendizaje()
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            paquete = self._crear_paquete_mapa_aprendido()
            paquete["motivo_guardado"] = str(motivo)

            json_ts = os.path.join(carpeta, f"mapa_laberinto_{stamp}.json")
            json_latest = os.path.join(carpeta, "mapa_laberinto.json")
            with open(json_ts, "w", encoding="utf-8") as f:
                json.dump(self._json_limpio(paquete), f, ensure_ascii=False, indent=2)
            with open(json_latest, "w", encoding="utf-8") as f:
                json.dump(self._json_limpio(paquete), f, ensure_ascii=False, indent=2)

            ruta_csv = os.path.join(carpeta, "ruta_recorrida.csv")
            self._guardar_csv_simple(ruta_csv, paquete.get("ruta_recorrida", []), ["i", "x", "y"])

            pares_csv = os.path.join(carpeta, "pares_detectados.csv")
            self._guardar_csv_simple(pares_csv, paquete.get("pares_detectados", []), ["t", "x", "y", "yaw", "fuente", "area_ratio"])

            inter_csv = os.path.join(carpeta, "intersecciones.csv")
            self._guardar_csv_simple(inter_csv, paquete.get("intersecciones", []), ["t", "x", "y", "decision"])

            dead_csv = os.path.join(carpeta, "callejones.csv")
            self._guardar_csv_simple(dead_csv, paquete.get("callejones", []), ["t", "x", "y"])

            png_ts = os.path.join(carpeta, f"mapa_laberinto_{stamp}.png")
            png_latest = os.path.join(carpeta, "mapa_laberinto.png")
            self._guardar_png_mapa_aprendido(paquete, png_ts)
            try:
                import shutil
                shutil.copyfile(png_ts, png_latest)
            except Exception:
                pass

            self.get_logger().warn(f"Mapa aprendido guardado en: {carpeta}")
            return carpeta
        except Exception as e:
            try:
                self.get_logger().error(f"No se pudo guardar mapa aprendido: {e}")
            except Exception:
                pass
            return None


    def _segmentos_laberinto_desde_ocupacion(self, occupied_centers, grid_res=0.10):
        """Convierte celdas ocupadas en paredes rectas tipo plano/laberinto.

        El reporte anterior dibujaba puntos crudos del LiDAR. Esta función limpia
        puntos aislados, agrupa celdas vecinas y genera segmentos horizontales y
        verticales para que el mapa se vea como un laberinto, no como una nube.
        """
        try:
            if not occupied_centers:
                return []
            r = float(grid_res or 0.10)
            cells = set((int(round(x / r)), int(round(y / r))) for x, y in occupied_centers)
            if not cells:
                return []

            # Quitar ruido: una celda pared debe tener vecinos cercanos.
            clean = set()
            for ix, iy in cells:
                vecinos = 0
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        if (ix + dx, iy + dy) in cells:
                            vecinos += 1
                if vecinos >= 1:
                    clean.add((ix, iy))
            if not clean:
                clean = cells

            segmentos = []
            min_run = int(getattr(self.cfg, "mapa_min_run_pared", 3))

            # Runs horizontales por fila.
            rows = {}
            for ix, iy in clean:
                rows.setdefault(iy, []).append(ix)
            for iy, xs in rows.items():
                xs = sorted(set(xs))
                start = prev = xs[0]
                for ix in xs[1:] + [None]:
                    if ix is not None and ix == prev + 1:
                        prev = ix
                        continue
                    if (prev - start + 1) >= min_run:
                        y = iy * r
                        x1 = (start - 0.5) * r
                        x2 = (prev + 0.5) * r
                        segmentos.append((x1, y, x2, y, "H", prev - start + 1))
                    if ix is not None:
                        start = prev = ix

            # Runs verticales por columna.
            cols = {}
            for ix, iy in clean:
                cols.setdefault(ix, []).append(iy)
            for ix, ys in cols.items():
                ys = sorted(set(ys))
                start = prev = ys[0]
                for iy in ys[1:] + [None]:
                    if iy is not None and iy == prev + 1:
                        prev = iy
                        continue
                    if (prev - start + 1) >= min_run:
                        x = ix * r
                        y1 = (start - 0.5) * r
                        y2 = (prev + 0.5) * r
                        segmentos.append((x, y1, x, y2, "V", prev - start + 1))
                    if iy is not None:
                        start = prev = iy

            # Limitar segmentos muy pequeños y duplicados visuales.
            segmentos.sort(key=lambda a: a[-1], reverse=True)
            return segmentos[:900]
        except Exception:
            return []

    def tiempo_total_corrida(self):
        """Tiempo visible para métricas del reto. No controla movimiento."""
        try:
            if self.t_inicio_corrida is None:
                return 0.0
            fin = self.t_meta_alcanzada if self.t_meta_alcanzada is not None else time.time()
            return max(0.0, float(fin - self.t_inicio_corrida))
        except Exception:
            return 0.0

    def distancia_recorrida_metrica(self):
        """Distancia para mostrar en interfaz/reporte usando la fuente más útil."""
        try:
            resumen = self.mapa_ruta.obtener_estado_resumen()
            d_mapa = float(resumen.get('distance', 0.0) or 0.0)
        except Exception:
            d_mapa = 0.0
        try:
            d_cmd = float(self.plot_total_distance)
        except Exception:
            d_cmd = 0.0
        return max(d_mapa, d_cmd, 0.0)

    def llego_meta_metrica(self):
        try:
            return bool(getattr(self, "meta_vision_finalizada", False) or getattr(self, "meta_cree_llegar", False) or self.estado_actual == "META_ALCANZADA")
        except Exception:
            return False

    def registrar_colision_estimada(self):
        """Conteo pasivo y conservador: se activa solo ante distancias críticas.
        No cambia navegación ni frena motores; solo sirve para métricas.
        """
        try:
            ahora = time.time()
            if ahora < self._colision_cooldown_hasta:
                return
            frente_critico = math.isfinite(self.dist_frente) and self.dist_frente < 0.12
            der_critico = math.isfinite(self.dist_der) and self.dist_der < 0.055
            izq_critico = math.isfinite(self.dist_izq) and self.dist_izq < 0.055
            if frente_critico or der_critico or izq_critico:
                self.colisiones_estimadas += 1
                self._colision_cooldown_hasta = ahora + 2.0
        except Exception:
            pass

    def generar_reporte_pausa(self):
        """Abre y guarda un plot de diagnóstico cuando se presiona PAUSAR.

        Incluye:
        - recorrido del robot por odometría,
        - pose actual con flecha de orientación,
        - puntos de pared/obstáculo guardados por LiDAR,
        - métricas útiles para revisar pérdidas de orientación.
        """
        try:
            mapa = self.mapa_ruta.obtener_datos_mapa()
            resumen = self.mapa_ruta.obtener_estado_resumen()
            odom_path = list(mapa.get("path", []))
            walls = list(mapa.get("walls", []))[-1200:]
            obstacles = list(mapa.get("obstacles", []))[-800:]
            rectangle = list(mapa.get("rectangle", []))
            free_cells = list(mapa.get("free_centers", []))[-5000:]
            visited_cells = list(mapa.get("visited_centers", []))[-2200:]
            occupied_cells = list(mapa.get("occupied_centers", []))[-3500:]
            grid_res = float(mapa.get("resolution", 0.10) or 0.10)
            intersecciones_lab = [(a, b, c, d) for (a, b, c, d) in list(getattr(self, "lab_intersecciones", []))]
            callejones_lab = [(a, b, c) for (a, b, c) in list(getattr(self, "lab_callejones", []))]

            # Ruta principal del plot:
            # 1) usar /odom si realmente tiene movimiento;
            # 2) si /odom está en cero, usar la trayectoria estimada con cmd_vel.
            path_cmd = list(self.plot_path)
            odom_dist = float(resumen.get('distance', 0.0))
            cmd_dist = float(self.plot_total_distance)
            odom_span = self.distancia_de_path(odom_path)
            usar_cmd = (len(odom_path) < 2 or odom_dist < 0.05 or odom_span < 0.05) and len(path_cmd) >= 2 and cmd_dist > 0.03

            if usar_cmd:
                path = path_cmd
                pose_x = float(self.plot_x)
                pose_y = float(self.plot_y)
                pose_yaw = float(self.plot_yaw)
                distancia_reporte = cmd_dist
                fuente_recorrido = "cmd_vel estimado"
                # En esta versión el mapa también puede venir de cmd_vel estimado,
                # por eso conservamos paredes/obstáculos si ya fueron transformados.
            else:
                path = odom_path
                pose_x = float(self.odom_x)
                pose_y = float(self.odom_y)
                pose_yaw = float(self.odom_yaw)
                distancia_reporte = odom_dist
                fuente_recorrido = "/odom" if self.tengo_odom else str(mapa.get("map_source", self.plot_source or "cmd_vel estimado"))

            # Si todavía no hay suficientes puntos, al menos dibuja la pose actual.
            if not path:
                path = [(pose_x, pose_y)]

            fig = plt.figure(figsize=(12.8, 7.2), facecolor="#0b0b0e")
            fig.canvas.manager.set_window_title("Reporte de pausa — recorrido del robot")
            gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0], height_ratios=[1.0, 0.72])

            ax_map = fig.add_subplot(gs[:, 0], facecolor="#eef4f7")
            ax_info = fig.add_subplot(gs[0, 1], facecolor="#0b0b0e")
            ax_hist = fig.add_subplot(gs[1, 1], facecolor="#111116")

            fig.suptitle("Reporte al pausar: recorrido + diagnóstico", color="white", fontsize=14, weight="bold")

            # -------------------------
            # Mapa / recorrido
            # -------------------------
            # Mapa de ocupación procesado: dibujar PLANO de laberinto.
            # No se muestran todos los puntos crudos del LiDAR porque eso se ve
            # como nube. Primero se convierten las celdas ocupadas en paredes
            # rectas horizontales/verticales, parecido al plano de referencia.
            if free_cells:
                fx, fy = zip(*free_cells)
                ax_map.scatter(fx, fy, s=22, marker="s", c="#ffffff", alpha=0.36, linewidths=0, label="libre estimado", zorder=1)
            if visited_cells:
                vx, vy = zip(*visited_cells)
                ax_map.scatter(vx, vy, s=28, marker="s", c="#d9f7d9", alpha=0.55, linewidths=0, label="zona visitada", zorder=2)

            segmentos_pared = self._segmentos_laberinto_desde_ocupacion(occupied_cells, grid_res)
            if segmentos_pared:
                primero = True
                for x1, y1, x2, y2, _ori, _n in segmentos_pared:
                    ax_map.plot([x1, x2], [y1, y2], color="#9a4f12", linewidth=3.0,
                                solid_capstyle="round", alpha=0.88, zorder=4,
                                label="pared procesada" if primero else None)
                    primero = False
            elif occupied_cells:
                gx, gy = zip(*occupied_cells)
                ax_map.scatter(gx, gy, s=42, marker="s", c="#b85c1b", alpha=0.86, linewidths=0, label="pared ocupación")

            # Puntos crudos solo como referencia tenue, no como mapa principal.
            mostrar_puntos_crudos = bool(getattr(self.cfg, "mapa_mostrar_puntos_crudos", False))
            if mostrar_puntos_crudos and walls:
                wx, wy = zip(*walls)
                ax_map.scatter(wx, wy, s=2, c="#263238", alpha=0.12, label="puntos LiDAR")
            if mostrar_puntos_crudos and obstacles:
                ox, oy = zip(*obstacles)
                ax_map.scatter(ox, oy, s=8, c="#d32f2f", alpha=0.30, label="frente/obstáculos")
            if rectangle:
                rx, ry = zip(*rectangle)
                ax_map.plot(rx, ry, linestyle="--", linewidth=1.0, color="#009688", alpha=0.38, label="contorno estimado", zorder=3)
            if intersecciones_lab:
                ix = [p[0] for p in intersecciones_lab]
                iy = [p[1] for p in intersecciones_lab]
                ax_map.scatter(ix, iy, s=42, marker="D", alpha=0.95, label="decisiones/intersecciones", zorder=6)
            if callejones_lab:
                cx = [p[0] for p in callejones_lab]
                cy = [p[1] for p in callejones_lab]
                ax_map.scatter(cx, cy, s=52, marker="x", alpha=0.95, label="callejones", zorder=6)
            if path:
                px, py = zip(*path)
                ax_map.plot(px, py, linewidth=3.0, color="#00a884", label="recorrido robot", zorder=7)
                ax_map.scatter([px[0]], [py[0]], s=80, marker="o", color="#1565c0", label="inicio")
                ax_map.scatter([px[-1]], [py[-1]], s=90, marker="s", color="#ff9800", label="pausa")

            arrow_len = 0.24
            dx = arrow_len * math.cos(pose_yaw)
            dy = arrow_len * math.sin(pose_yaw)
            ax_map.arrow(
                pose_x, pose_y, dx, dy,
                width=0.012, head_width=0.075, head_length=0.09,
                length_includes_head=True, color="#00838f", label="orientación"
            )
            ax_map.text(pose_x, pose_y, "  robot", color="#263238", fontsize=9, weight="bold")

            ax_map.set_title(f"Plano tipo laberinto — paredes filtradas ({fuente_recorrido})", color="#263238", fontsize=11, weight="bold")
            ax_map.set_xlabel("x [m]", color="#263238")
            ax_map.set_ylabel("y [m]", color="#263238")
            ax_map.grid(True, linestyle="--", alpha=0.30, color="#90a4ae")
            try:
                ax_map.set_xticks(np.arange(math.floor(ax_map.get_xlim()[0]), math.ceil(ax_map.get_xlim()[1]) + 0.001, 0.60), minor=True)
                ax_map.set_yticks(np.arange(math.floor(ax_map.get_ylim()[0]), math.ceil(ax_map.get_ylim()[1]) + 0.001, 0.60), minor=True)
                ax_map.grid(True, which="minor", linestyle=":", alpha=0.18, color="#90a4ae")
            except Exception:
                pass
            ax_map.tick_params(colors="#263238")
            ax_map.set_aspect("equal", adjustable="box")
            ax_map.legend(loc="upper left", fontsize=7.2, facecolor="#ffffff", edgecolor="#78909c", labelcolor="#263238", framealpha=0.86)

            all_points = []
            all_points.extend(path)
            all_points.extend(walls)
            all_points.extend(obstacles)
            all_points.extend(free_cells)
            all_points.extend(visited_cells)
            all_points.extend(occupied_cells)
            all_points.extend([(p[0], p[1]) for p in intersecciones_lab])
            all_points.extend([(p[0], p[1]) for p in callejones_lab])
            if all_points:
                xs = [p[0] for p in all_points]
                ys = [p[1] for p in all_points]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                span = max(max_x - min_x, max_y - min_y, 0.8)
                pad = max(0.35, span * 0.12)
                ax_map.set_xlim(min_x - pad, max_x + pad)
                ax_map.set_ylim(min_y - pad, max_y + pad)

            # -------------------------
            # Panel de datos
            # -------------------------
            ax_info.axis("off")
            if len(self.tiempos_loop) > 0:
                dt_prom = sum(self.tiempos_loop) / len(self.tiempos_loop)
                hz = 1.0 / dt_prom if dt_prom > 0 else 0.0
            else:
                hz = 0.0
            t_prom = (sum(self.tiempos_proc) / len(self.tiempos_proc)) if self.tiempos_proc else 0.0
            t_max = max(self.tiempos_proc) if self.tiempos_proc else 0.0
            promedio_v = self._promedio_historial("v", absoluto=True)
            promedio_w = self._promedio_historial("w", absoluto=True)
            max_w = self._max_historial("w", absoluto=True)
            estado_previo = self.estado_post_pausa or self.estado_actual
            bateria_txt = "sin datos"
            if self.porcentaje_bateria is not None:
                bateria_txt = f"{self.porcentaje_bateria}%"
                if math.isfinite(self.voltaje_bateria):
                    bateria_txt += f" / {self.voltaje_bateria:.1f} V"

            tiempo_total = self.tiempo_total_corrida()
            llego_meta = self.llego_meta_metrica()
            pare_detectados = int(getattr(self, "pare_detenciones", 0))
            pare_respetados = int(getattr(self, "pare_respetados", 0))
            # Si ya está en META y hubo PARE activo que terminó, el contador de respetados
            # puede estar igualado al número de detenciones confirmadas.
            if llego_meta and pare_respetados < pare_detectados and not self.pare_en_detencion_activa():
                pare_respetados = pare_detectados

            info = [
                "MÉTRICAS DEL RETO",
                f"Tiempo total          : {tiempo_total:.1f} s",
                f"Distancia recorrida   : {distancia_reporte:.2f} m",
                f"PAREs detectados      : {pare_detectados}",
                f"PAREs respetados      : {pare_respetados}",
                f"Colisiones            : {int(getattr(self, 'colisiones_estimadas', 0))}",
                f"Llegó a meta          : {'SÍ' if llego_meta else 'NO'}",
                "",
                "DATOS DE APOYO",
                f"Estado final          : {estado_previo}",
                f"Fuente recorrido      : {fuente_recorrido}",
                f"PARE cámara           : {self.pare_fuente}",
                f"META cámara           : verdes={getattr(self, 'meta_vision_detectadas', 0)}",
                f"Frente actual         : {self.dist_frente:.2f} m",
                f"Ancho pasillo         : {self.ancho_pasillo:.2f} m",
                f"Decisión laberinto    : {getattr(self, 'lab_decision_actual', '---')}",
                f"Velocidad prom.       : {promedio_v:.3f} m/s",
                f"Giro prom. |w|        : {promedio_w:.3f} rad/s",
                f"Giro máximo |w|       : {max_w:.3f} rad/s",
                f"Loop aprox.           : {hz:.1f} Hz",
                f"Proc. LiDAR prom/max  : {t_prom:.1f}/{t_max:.1f} ms",
                f"Batería               : {bateria_txt}",
            ]
            ax_info.text(
                0.02, 0.98, "\n".join(info),
                va="top", ha="left", color="#eeeeee", fontsize=9.2,
                family="monospace",
                bbox=dict(facecolor="#111116", edgecolor="#555555", boxstyle="round,pad=0.55")
            )

            # -------------------------
            # Historial corto
            # -------------------------
            hist = list(self.historial_control)[-350:]
            if hist:
                t0 = hist[0]["t"]
                ts = [h["t"] - t0 for h in hist]
                frente = [h["frente"] if math.isfinite(h["frente"]) else float("nan") for h in hist]
                error_centro = [h["error_centro"] for h in hist]
                giro = [h["w"] for h in hist]

                # Capybot S: gráfico profesional sin líneas montadas.
                # Cada señal se normaliza en su propia banda vertical para que
                # frente, error y angular se puedan leer a la vez sin cruzarse.
                def _banda(vals, base, amp=0.28):
                    arr = np.array(vals, dtype=float)
                    valid = np.isfinite(arr)
                    if not np.any(valid):
                        return np.full_like(arr, base, dtype=float)
                    v = arr[valid]
                    lo, hi = float(np.nanpercentile(v, 5)), float(np.nanpercentile(v, 95))
                    if abs(hi - lo) < 1e-6:
                        out = np.zeros_like(arr, dtype=float)
                    else:
                        out = (arr - lo) / (hi - lo)
                        out = np.clip(out, 0.0, 1.0) * 2.0 - 1.0
                    out[~valid] = 0.0
                    return base + amp * out

                y_frente = _banda(frente, 2.0)
                y_error = _banda(error_centro, 1.0)
                y_giro = _banda(giro, 0.0)

                ax_hist.plot(ts, y_frente, linewidth=1.8, color="#80cbc4")
                ax_hist.plot(ts, y_error, linewidth=1.5, color="#fff176")
                ax_hist.plot(ts, y_giro, linewidth=1.5, color="#c5cae9")
                ax_hist.axhline(2.0, color="#80cbc4", alpha=0.20, linewidth=0.8)
                ax_hist.axhline(1.0, color="#fff176", alpha=0.20, linewidth=0.8)
                ax_hist.axhline(0.0, color="#c5cae9", alpha=0.20, linewidth=0.8)
                ax_hist.set_yticks([2.0, 1.0, 0.0])
                ax_hist.set_yticklabels(["frente", "error", "angular"], color="#cccccc")
                ax_hist.set_ylim(-0.55, 2.55)
                ax_hist.set_xlabel("últimos segundos", color="#cccccc")
                ax_hist.set_title("Historial antes de pausar — señales separadas", color="#e0e0e0", fontsize=10)
                ax_hist.grid(True, axis="x", linestyle="--", alpha=0.25)
                ax_hist.text(0.99, 0.92,
                             f"frente {frente[-1]:.2f} m",
                             transform=ax_hist.transAxes, ha="right", color="#80cbc4", fontsize=8)
                ax_hist.text(0.99, 0.60,
                             f"error {error_centro[-1]:+.2f} m",
                             transform=ax_hist.transAxes, ha="right", color="#fff176", fontsize=8)
                ax_hist.text(0.99, 0.27,
                             f"angular {giro[-1]:+.2f} rad/s",
                             transform=ax_hist.transAxes, ha="right", color="#c5cae9", fontsize=8)
            else:
                ax_hist.text(0.5, 0.5, "Sin historial todavía", ha="center", va="center", color="#eeeeee")
            ax_hist.tick_params(colors="#cccccc")

            fig.tight_layout(rect=[0, 0, 1, 0.95])

            carpeta = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "reportes_pausa"))
            os.makedirs(carpeta, exist_ok=True)
            nombre = f"reporte_pausa_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            ruta = os.path.join(carpeta, nombre)
            fig.savefig(ruta, dpi=150, facecolor=fig.get_facecolor(), bbox_inches="tight")
            self.ultimo_reporte_pausa = ruta
            self.get_logger().warn(f"Reporte de pausa guardado: {ruta}")

            # Mostrar ventana sin bloquear la interfaz principal.
            fig.canvas.draw_idle()
            try:
                fig.show()
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"No se pudo generar el reporte de pausa: {e}")

    # ==========================================================
    # INTERFAZ
    # ==========================================================
    def update_interface(self, frame):
        if self.solicitud_salir:
            return self.ui.scatter_frente, self.ui.scatter_izq, self.ui.scatter_der

        self.ui.actualizar_graficos(
            self.datos_filtrados,
            distancia_objetivo=0.0
        )
        mapa_gui = self.mapa_ruta.obtener_datos_mapa()
        try:
            mapa_gui["decisions"] = [(a, b) for (a, b, _, _) in list(getattr(self, "lab_intersecciones", []))]
            mapa_gui["deadends"] = [(a, b) for (a, b, _) in list(getattr(self, "lab_callejones", []))]
        except Exception:
            pass
        self.ui.actualizar_mapa_recorrido(mapa_gui)
        self.evaluar_meta_conocida()

        # Vista en vivo de cámara PARE dentro de la interfaz principal.
        # Solo muestra imagen/diagnóstico, no altera la lógica de recorrido.
        try:
            with self.camera_lock:
                frame_cam = None if self.camera_frame_rgb is None else self.camera_frame_rgb.copy()
                estado_cam = self.camera_preview_estado or self.pare_fuente
                if self.camera_enabled and frame_cam is not None and (time.time() - self.camera_frame_t) > 1.5:
                    estado_cam = "CAMARA SIN FRAMES"
            self.ui.actualizar_camara(frame_cam, estado_cam)
        except Exception:
            pass

        if len(self.tiempos_proc) > 0:
            t_prom = sum(self.tiempos_proc) / len(self.tiempos_proc)
            t_max = max(self.tiempos_proc)
        else:
            t_prom = 0.0
            t_max = 0.0

        if len(self.tiempos_loop) > 0:
            dt_prom = sum(self.tiempos_loop) / len(self.tiempos_loop)
            hz = 1.0 / dt_prom if dt_prom > 0 else 0.0
        else:
            hz = 0.0

        dist_der_mostrar = self.dist_der_pared if math.isfinite(self.dist_der_pared) else self.dist_der
        dist_izq_mostrar = self.dist_izq_pared if math.isfinite(self.dist_izq_pared) else self.dist_izq

        self.ui.renderizar_telemetria(
            self.estado_actual,
            [self.dist_frente, dist_izq_mostrar, dist_der_mostrar, self.dist_diag_der],
            [self.vel_lineal, self.vel_angular],
            [self.voltaje_bateria, self.porcentaje_bateria, self.bateria_fuente],
            [self.odom_x, self.odom_y, self.odom_yaw],
            [
                self.cfg.kp_centrado,
                self.cfg.kp_angulo,
                0.0,
                self.error_centro_actual,
                self.error_ang_actual,
                self.ancho_pasillo,
            ],
            [self.t_proc_actual, t_prom, t_max, hz],
            self.total_puntos,
            self.nombre_modo_velocidad,
            self.factor_angular,
            self.bateria_fuente,
            [self.pared_der_valida, self.pared_izq_valida, self.puntos_pared_der, self.puntos_pared_izq],
            {**self.mapa_ruta.obtener_estado_resumen(),
             "meta_cree_llegar": bool(getattr(self, "meta_cree_llegar", False)),
             "meta_distancia": float(getattr(self, "meta_distancia", float('inf'))),
             "meta_x": float(getattr(self, "meta_x", 3.60)),
             "meta_y": float(getattr(self, "meta_y", 2.40)),
             "meta_vision_finalizada": bool(getattr(self, "meta_vision_finalizada", False)),
             "meta_vision_activa": bool(getattr(self, "meta_vision_activa", False)),
             "meta_vision_detectadas": int(getattr(self, "meta_vision_detectadas", 0)),
             "metricas_reto": {
                 "tiempo_total": float(self.tiempo_total_corrida()),
                 "distancia_recorrida": float(self.distancia_recorrida_metrica()),
                 "pares_detectados": int(getattr(self, "pare_detenciones", 0)),
                 "pares_respetados": int(max(getattr(self, "pare_respetados", 0), getattr(self, "pare_detenciones", 0) if self.llego_meta_metrica() else getattr(self, "pare_respetados", 0))),
                 "colisiones": int(getattr(self, "colisiones_estimadas", 0)),
                 "llego_meta": bool(self.llego_meta_metrica()),
             }},
            [self.tipo_frente, self.front_class_sm, self.front_ang_width, self.contador_cajas],
        )
        return self.ui.scatter_frente, self.ui.scatter_izq, self.ui.scatter_der


def main(args=None):
    rclpy.init(args=args)
    sistema = SistemaControlBorde()

    ani = FuncAnimation(sistema.fig, sistema.update_interface, blit=False, interval=80)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(sistema), daemon=True)
    ros_thread.start()

    plt.show()

    print("Deteniendo de manera segura los actuadores del robot...")
    sistema.detener_robot()
    try:
        sistema.guardar_mapa_aprendido(motivo="cierre_ventana")
    except Exception:
        pass
    time.sleep(0.2)

    sistema.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
