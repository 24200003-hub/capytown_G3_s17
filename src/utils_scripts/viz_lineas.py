#!/usr/bin/env python3
"""
viz_lineas.py  —  Detección y visualización de líneas en /scan.

Pipeline:
  /scan ──▶ filtrar (inf/nan/rango/angular)
       ──▶ pre-segmentar (rompe en saltos de índice y distancia)
       ──▶ Split-and-Merge por segmento
       ──▶ visualizar

Uso:
    python3 viz_lineas.py [opciones]

Filtrado:
    --remove-min DEG   ángulo inicio de zona eliminada
    --remove-max DEG   ángulo fin   de zona eliminada
    --rango-max  M     distancia máxima (default 4.0)

Pre-segmentación (controla cuándo empieza un objeto nuevo):
    --salto-dist   M   salto euclidiano entre puntos consecutivos para cortar (default 0.35)
    --salto-idx    N   lecturas inválidas consecutivas que justifican un corte (default 5)

Split-and-Merge (controla qué tan rectos deben ser los segmentos):
    --umbral-split M   distancia máx punto↔recta para que siga siendo la misma línea (default 0.06)
    --min-puntos   N   puntos mínimos por segmento (default 5)
    --min-longitud M   longitud mínima de segmento a dibujar (default 0.10)
"""

import argparse
import math
import threading
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ─── tipos ───────────────────────────────────────────────────────────────────
Point = Tuple[float, float]

@dataclass
class Segmento:
    puntos: List[Point]
    p1: Point
    p2: Point
    longitud: float


# ─── nodo ROS 2 ──────────────────────────────────────────────────────────────
class LineasViz(Node):
    def __init__(self, remove_min, remove_max, rango_max):
        super().__init__("viz_lineas")
        self.remove_min = remove_min
        self.remove_max = remove_max
        self.rango_max  = rango_max
        self._lock  = threading.Lock()
        self._datos = None
        self.create_subscription(LaserScan, "/scan", self._cb, 10)
        self.get_logger().info("viz_lineas: suscrito a /scan")

    def _cb(self, msg):
        with self._lock:
            self._datos = msg

    def get_datos(self):
        with self._lock:
            return self._datos


# ─── filtrado ────────────────────────────────────────────────────────────────
def _en_arco(theta, a_min, a_max):
    if a_min is None:
        return False
    if a_min <= a_max:
        return a_min <= theta <= a_max
    return theta >= a_min or theta <= a_max


def filtrar_scan(msg: LaserScan, remove_min, remove_max, rango_max):
    """Devuelve lista de (scan_idx, x, y) conservando el orden del barrido."""
    puntos = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r):
            continue
        if r < msg.range_min or r > min(msg.range_max, rango_max):
            continue
        theta   = msg.angle_min + i * msg.angle_increment
        theta_n = math.atan2(math.sin(theta), math.cos(theta))
        if _en_arco(theta_n, remove_min, remove_max):
            continue
        puntos.append((i, r * math.cos(theta), r * math.sin(theta)))
    return puntos


# ─── pre-segmentación ────────────────────────────────────────────────────────
def pre_segmentar(puntos_idx: List[Tuple],
                  salto_dist: float,
                  salto_idx: int) -> List[List[Point]]:
    """
    Rompe la nube en grupos contiguos cuando hay:
      - un hueco de índice > salto_idx  (lecturas inválidas seguidas)
      - un salto euclidiano > salto_dist (cambio de objeto)
    Devuelve lista de grupos, cada uno como lista de (x, y).
    """
    if not puntos_idx:
        return []

    grupos = []
    actual = [puntos_idx[0]]

    for k in range(1, len(puntos_idx)):
        i_prev, x_prev, y_prev = puntos_idx[k - 1]
        i_curr, x_curr, y_curr = puntos_idx[k]

        gap_idx  = i_curr - i_prev
        dist_euc = math.hypot(x_curr - x_prev, y_curr - y_prev)

        if gap_idx > salto_idx or dist_euc > salto_dist:
            if len(actual) >= 2:
                grupos.append([(x, y) for _, x, y in actual])
            actual = [puntos_idx[k]]
        else:
            actual.append(puntos_idx[k])

    if len(actual) >= 2:
        grupos.append([(x, y) for _, x, y in actual])

    return grupos


# ─── Split-and-Merge (IEPF) ──────────────────────────────────────────────────
def _dist_punto_recta(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / L


def _split(puntos: List[Point], umbral: float) -> List[List[Point]]:
    """Divide recursivamente un grupo hasta que todos los sub-grupos sean rectos."""
    if len(puntos) < 2:
        return [puntos]
    a, b   = puntos[0], puntos[-1]
    dists  = [_dist_punto_recta(p, a, b) for p in puntos]
    idx_mx = int(np.argmax(dists))
    d_mx   = dists[idx_mx]
    if d_mx > umbral and len(puntos) > 2:
        izq = _split(puntos[:idx_mx + 1], umbral)
        der = _split(puntos[idx_mx:],     umbral)
        return izq + der[1:]
    return [puntos]


def _merge(grupos: List[List[Point]], umbral: float) -> List[List[Point]]:
    """Une grupos adyacentes si al fusionarlos siguen siendo rectos."""
    if len(grupos) <= 1:
        return grupos
    merged = [grupos[0]]
    for g in grupos[1:]:
        candidato = merged[-1] + g
        if len(_split(candidato, umbral)) == 1:
            merged[-1] = candidato
        else:
            merged.append(g)
    return merged


def detectar_lineas(puntos_idx: List[Tuple],
                    salto_dist: float,
                    salto_idx:  int,
                    umbral_split: float,
                    min_puntos:   int,
                    min_longitud: float) -> List[Segmento]:
    """
    Pipeline completo:
      1. pre-segmentar (rompe en objetos distintos)
      2. Split-and-Merge por grupo
      3. filtrar por min_puntos y min_longitud
    """
    grupos = pre_segmentar(puntos_idx, salto_dist, salto_idx)

    segmentos = []
    for grupo in grupos:
        if len(grupo) < min_puntos:
            continue
        sub = _split(grupo, umbral_split)
        sub = _merge(sub, umbral_split)
        for g in sub:
            if len(g) < min_puntos:
                continue
            p1, p2 = g[0], g[-1]
            lon    = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if lon < min_longitud:
                continue
            segmentos.append(Segmento(puntos=g, p1=p1, p2=p2, longitud=lon))

    return segmentos


# ─── matplotlib ──────────────────────────────────────────────────────────────
PALETA = [
    "#ff7043", "#ab47bc", "#26c6da", "#d4e157",
    "#ffca28", "#ef5350", "#42a5f5", "#66bb6a",
    "#ff8a65", "#ce93d8", "#80deea", "#e6ee9c",
]


def construir_figura():
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#0f0e17")
    ax.tick_params(colors="white")
    ax.spines[:].set_color("#333")
    ax.set_aspect("equal")
    ax.grid(True, color="#1e1e2e", linewidth=0.6)
    ax.set_xlabel("x [m]", color="#aaa")
    ax.set_ylabel("y [m]", color="#aaa")
    ax.set_title("Detección de líneas — Split-and-Merge", color="white", fontsize=11)
    fig.tight_layout()
    return fig, ax


def actualizar_figura(ax, puntos_xy: List[Point], segmentos: List[Segmento],
                      n_grupos: int):
    ax.cla()
    ax.set_facecolor("#0f0e17")
    ax.tick_params(colors="white")
    ax.spines[:].set_color("#333")
    ax.set_aspect("equal")
    ax.grid(True, color="#1e1e2e", linewidth=0.6)
    ax.set_xlabel("x [m]", color="#aaa")
    ax.set_ylabel("y [m]", color="#aaa")
    ax.set_title(
        f"Split-and-Merge  —  {len(segmentos)} segmentos  /  "
        f"{n_grupos} grupos  /  {len(puntos_xy)} pts",
        color="white", fontsize=10,
    )

    # nube de puntos del scan (gris tenue)
    if puntos_xy:
        xs = [p[0] for p in puntos_xy]
        ys = [p[1] for p in puntos_xy]
        ax.scatter(xs, ys, s=5, color="#546e7a", alpha=0.5, zorder=2)

    # segmentos detectados
    for k, seg in enumerate(segmentos):
        color = PALETA[k % len(PALETA)]
        # puntos del segmento (encima de la nube)
        sx = [p[0] for p in seg.puntos]
        sy = [p[1] for p in seg.puntos]
        ax.scatter(sx, sy, s=12, color=color, alpha=0.8, zorder=3)
        # línea entre extremos
        ax.plot([seg.p1[0], seg.p2[0]], [seg.p1[1], seg.p2[1]],
                color=color, linewidth=2.2, zorder=4,
                label=f"L{k+1} {seg.longitud:.2f}m")
        # etiqueta en el centro
        cx = (seg.p1[0] + seg.p2[0]) / 2
        cy = (seg.p1[1] + seg.p2[1]) / 2
        ax.annotate(f"L{k+1}\n{seg.longitud:.2f}m", (cx, cy),
                    color=color, fontsize=7, ha="center",
                    bbox=dict(boxstyle="round,pad=0.2", fc="#0f0e17", alpha=0.8))

    # robot
    ax.plot(0, 0, "w^", markersize=12, zorder=6)
    ax.annotate("robot", (0.06, 0.06), color="white", fontsize=8)

    if segmentos:
        ax.legend(loc="upper right", facecolor="#1a1a2e", edgecolor="#444",
                  labelcolor="white", fontsize=7)

    todos = puntos_xy + [seg.p1 for seg in segmentos] + [seg.p2 for seg in segmentos]
    if todos:
        M = max(abs(v) for xy in todos for v in xy) + 0.3
        ax.set_xlim(-M, M)
        ax.set_ylim(-M, M)


# ─── main ────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Detección de líneas en /scan")
    # filtrado
    p.add_argument("--remove-min",    type=float, default=None)
    p.add_argument("--remove-max",    type=float, default=None)
    p.add_argument("--rango-max",     type=float, default=4.0)
    # pre-segmentación
    p.add_argument("--salto-dist",    type=float, default=0.35,
                   help="salto euclidiano (m) que separa dos objetos (default 0.35)")
    p.add_argument("--salto-idx",     type=int,   default=5,
                   help="huecos de índice consecutivos para cortar grupo (default 5)")
    # split-and-merge
    p.add_argument("--umbral-split",  type=float, default=0.06,
                   help="distancia punto↔recta para dividir (m, default 0.06)")
    p.add_argument("--min-puntos",    type=int,   default=5,
                   help="puntos mínimos por segmento (default 5)")
    p.add_argument("--min-longitud",  type=float, default=0.10,
                   help="longitud mínima de segmento (m, default 0.10)")
    return p.parse_args()


def main():
    args = parse_args()

    rm_min = math.radians(args.remove_min) if args.remove_min is not None else None
    rm_max = math.radians(args.remove_max) if args.remove_max is not None else None

    rclpy.init()
    nodo = LineasViz(rm_min, rm_max, args.rango_max)

    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()

    fig, ax = construir_figura()
    plt.ion()
    plt.show()

    try:
        while plt.fignum_exists(fig.number):
            msg = nodo.get_datos()
            if msg is not None:
                puntos_idx = filtrar_scan(msg, rm_min, rm_max, args.rango_max)
                puntos_xy  = [(x, y) for _, x, y in puntos_idx]

                grupos = pre_segmentar(puntos_idx, args.salto_dist, args.salto_idx)

                segmentos = detectar_lineas(
                    puntos_idx,
                    salto_dist   = args.salto_dist,
                    salto_idx    = args.salto_idx,
                    umbral_split = args.umbral_split,
                    min_puntos   = args.min_puntos,
                    min_longitud = args.min_longitud,
                )

                actualizar_figura(ax, puntos_xy, segmentos, len(grupos))
                fig.canvas.draw_idle()
            plt.pause(0.15)
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
