#!/usr/bin/env python3
"""
metrics_logger.py
-----------------
Nodo de METRICAS para el reto del Censo.

Compara las cajas detectadas (/cajas_avistadas) contra un ground-truth
declarado por parametro y calcula:
    VP  (verdaderos positivos)  : caja real detectada
    FP  (falsos positivos)      : deteccion sin caja real cerca
    FN  (falsos negativos)      : caja real no detectada
    tasa_deteccion = VP / (VP + FN)
    error_medio    : distancia media VP <-> caja real

Al apagar el nodo (Ctrl+C) escribe 'metricas_lidar.csv'.

ESAN - Robotica de Moviles 2026-I  |  Proyecto CapyTown
"""

import csv
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        # Ground-truth: lista plana [x1, y1, x2, y2, ...] en marco odom.
        self.declare_parameter('ground_truth', [
            1.0, 0.5,
            2.0, -0.5,
            3.0, 0.4,
        ])
        self.declare_parameter('tolerancia_match', 0.30)  # m para contar VP
        self.declare_parameter('archivo_csv', 'metricas_lidar.csv')

        gt_plano = list(self.get_parameter('ground_truth').value)
        self.gt = [(gt_plano[i], gt_plano[i + 1])
                   for i in range(0, len(gt_plano) - 1, 2)]
        self.tol = self.get_parameter('tolerancia_match').value
        self.archivo = self.get_parameter('archivo_csv').value

        self.ultimas_detecciones = []

        self.create_subscription(
            PoseArray, '/cajas_avistadas', self.cb_cajas, 10)
        self.get_logger().info(
            f'metrics_logger iniciado. Ground-truth: {len(self.gt)} cajas.')

    def cb_cajas(self, msg: PoseArray):
        self.ultimas_detecciones = [
            (p.position.x, p.position.y) for p in msg.poses]

    # ------------------------------------------------------------------
    def calcular(self):
        det = list(self.ultimas_detecciones)
        gt_libres = list(self.gt)
        vp = 0
        errores = []

        # Emparejamiento codicioso: cada deteccion busca la caja real mas cercana
        # dentro de la tolerancia y la "consume".
        for d in det:
            mejor_idx = -1
            mejor_dist = self.tol
            for i, g in enumerate(gt_libres):
                dist = math.hypot(d[0] - g[0], d[1] - g[1])
                if dist < mejor_dist:
                    mejor_dist = dist
                    mejor_idx = i
            if mejor_idx >= 0:
                vp += 1
                errores.append(mejor_dist)
                gt_libres.pop(mejor_idx)

        fp = len(det) - vp           # detecciones sin pareja
        fn = len(gt_libres)          # cajas reales sin detectar
        tasa = vp / (vp + fn) if (vp + fn) > 0 else 0.0
        err_medio = sum(errores) / len(errores) if errores else 0.0
        return vp, fp, fn, tasa, err_medio

    # ------------------------------------------------------------------
    def guardar_csv(self):
        vp, fp, fn, tasa, err = self.calcular()
        with open(self.archivo, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['metrica', 'valor'])
            w.writerow(['cajas_reales', len(self.gt)])
            w.writerow(['detecciones', len(self.ultimas_detecciones)])
            w.writerow(['VP', vp])
            w.writerow(['FP', fp])
            w.writerow(['FN', fn])
            w.writerow(['tasa_deteccion', round(tasa, 3)])
            w.writerow(['error_medio_m', round(err, 3)])
        self.get_logger().info(
            f'CSV guardado: {self.archivo}  |  VP={vp} FP={fp} FN={fn} '
            f'tasa={tasa:.2f} err={err:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    nodo = MetricsLogger()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_csv()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
