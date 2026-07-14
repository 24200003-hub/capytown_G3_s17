#!/usr/bin/env python3
"""
lidar_utils.py
--------------
Funciones auxiliares para limpiar y segmentar el LaserScan del robot.
Estas funciones son usadas por box_detector y behavior_fsm.
"""

import math
import numpy as np


def _limpiar_ranges(ranges, valor_reemplazo=5.0):
    """Convierte el arreglo de rangos a NumPy y reemplaza lecturas inválidas."""
    arr = np.array(ranges, dtype=float)
    arr[(arr == 0.0) | np.isinf(arr) | np.isnan(arr)] = valor_reemplazo
    return arr


def segmentar_lidar(ranges, angle_min=None, angle_increment=None):
    """
    Divide el LiDAR en tres sectores principales: frente, izquierda y derecha.

    Retorna un diccionario con las distancias mínimas por sector:
        {'frente': x, 'izquierda': y, 'derecha': z}

    Se mantiene compatible con dos formas de uso:
    1) segmentar_lidar(msg.ranges, msg.angle_min, msg.angle_increment)
    2) segmentar_lidar(msg.ranges)
    """
    arr = _limpiar_ranges(ranges)
    n = len(arr)

    if n == 0:
        return {'frente': 5.0, 'izquierda': 5.0, 'derecha': 5.0, 'pasadizo': False, 'esquina': False}

    # Si no hay información angular, usar cortes por índice como respaldo.
    if angle_min is None or angle_increment is None:
        idx_frente = max(1, int(n * 0.08))
        frente = np.concatenate((arr[-idx_frente:], arr[:idx_frente]))
        derecha = arr[int(n * 0.12):int(n * 0.45)]
        izquierda = arr[int(n * 0.55):int(n * 0.88)]
    else:
        angles = angle_min + np.arange(n) * angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        # Frente: +/- 25 grados.
        # Laterales: 35 a 110 grados a cada lado.
        frente_mask = np.abs(angles) <= math.radians(25)
        izquierda_mask = (angles >= math.radians(35)) & (angles <= math.radians(110))
        derecha_mask = (angles <= -math.radians(35)) & (angles >= -math.radians(110))
        frente_der_mask = (angles <= math.radians(-8)) & (angles >= math.radians(-55))
        frente_izq_mask = (angles >= math.radians(8)) & (angles <= math.radians(55))

        frente = arr[frente_mask]
        izquierda = arr[izquierda_mask]
        derecha = arr[derecha_mask]
        frente_der = arr[frente_der_mask]
        frente_izq = arr[frente_izq_mask]

    def min_seg(segmento):
        return float(np.min(segmento)) if len(segmento) else 5.0

    frente_min = min_seg(frente)
    izq_min = min_seg(izquierda)
    der_min = min_seg(derecha)

    # Pasadizo: paredes laterales detectadas y frente abierto.
    pasadizo = (izq_min < 1.20 and der_min < 1.20 and frente_min > 0.45)

    # Esquina: algo al frente conectado visualmente con alguno de los costados.
    # Es una aproximación simple para el nodo behavior_fsm; el ejecutable usa
    # una clasificación por clusters más completa.
    try:
        f_der = min_seg(frente_der)
        f_izq = min_seg(frente_izq)
    except UnboundLocalError:
        f_der = der_min
        f_izq = izq_min
    esquina = (frente_min < 0.55 and (f_der < 0.80 or f_izq < 0.80 or der_min < 1.05 or izq_min < 1.05))

    return {
        'frente': frente_min,
        'izquierda': izq_min,
        'derecha': der_min,
        'pasadizo': pasadizo,
        'esquina': esquina,
    }


def detectar_escalon_caja(distancia_lateral, distancia_pared_ideal=0.60,
                           margen_min=0.10, margen_max=0.45):
    """
    Detecta posible caja cuando el LiDAR ve un objeto lateral más cercano que
    la pared esperada del pasillo.

    Ejemplo: si la pared está aprox. a 0.60 m, una caja puede aparecer entre
    0.10 m y 0.45 m.
    """
    if distancia_lateral is None or not math.isfinite(distancia_lateral):
        return False

    limite_superior = min(margen_max, distancia_pared_ideal - 0.08)
    return margen_min <= distancia_lateral <= limite_superior
