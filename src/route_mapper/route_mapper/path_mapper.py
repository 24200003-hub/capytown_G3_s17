#!/usr/bin/env python3
"""
path_mapper.py
---------------
Memoria simple del recorrido + mapa de ocupación local:
- Guarda la ruta del robot.
- Convierte puntos LiDAR del marco del robot al marco del mapa.
- Marca celdas libres, ocupadas y visitadas para dibujar un mapa tipo laberinto.
- Mantiene compatibilidad con el reporte anterior: path/walls/obstacles.

Convención de los puntos que llegan desde radar_utils:
- x_local positivo = derecha del robot
- y_local positivo = frente del robot
"""
import math
import time
from collections import deque, defaultdict
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np


Point = Tuple[float, float]
Cell = Tuple[int, int]


@dataclass
class RouteMemoryMapper:
    # Muestreo del recorrido
    path_min_step: float = 0.030
    max_path_points: int = 3500

    # Puntos LiDAR transformados a mapa
    max_wall_points: int = 7000
    max_obstacle_points: int = 2800
    max_map_range: float = 2.80
    lidar_stride: int = 5

    # Mapa de ocupación tipo laberinto
    grid_resolution: float = 0.10          # 10 cm por celda
    occ_hit_threshold: int = 2             # hits mínimos para pintar pared
    free_decay_weight: float = 0.65        # una celda muy libre no se vuelve pared por un ruido
    ray_stride_cells: int = 1
    max_occ_cells_to_draw: int = 3500
    max_free_cells_to_draw: int = 5000
    max_visited_cells_to_draw: int = 2200

    # Anti-retorno
    min_route_before_avoid_m: float = 1.20
    revisit_radius: float = 0.18
    recent_points_to_ignore: int = 45
    lap_radius: float = 0.28
    min_lap_distance_m: float = 2.20
    min_lap_time_s: float = 18.0
    anti_return_bias: float = 0.18

    path: Deque[Point] = field(default_factory=lambda: deque(maxlen=3500))
    wall_points: Deque[Point] = field(default_factory=lambda: deque(maxlen=7000))
    obstacle_points: Deque[Point] = field(default_factory=lambda: deque(maxlen=2800))

    # Diccionarios de celdas: (ix, iy) -> conteo
    occ_hits: Dict[Cell, int] = field(default_factory=lambda: defaultdict(int))
    free_hits: Dict[Cell, int] = field(default_factory=lambda: defaultdict(int))
    visited_hits: Dict[Cell, int] = field(default_factory=lambda: defaultdict(int))

    start_pose: Optional[Tuple[float, float, float]] = None
    last_pose: Optional[Tuple[float, float, float]] = None
    last_path_point: Optional[Point] = None
    start_time: float = field(default_factory=time.time)
    total_distance: float = 0.0
    lap_count: int = 0
    lap_locked: bool = False
    last_revisit_distance: float = float("inf")
    last_update_source: str = "sin datos"

    def __post_init__(self):
        self.path = deque(maxlen=self.max_path_points)
        self.wall_points = deque(maxlen=self.max_wall_points)
        self.obstacle_points = deque(maxlen=self.max_obstacle_points)

    # ------------------------------------------------------
    # ODOMETRÍA / POSE / RUTA
    # ------------------------------------------------------
    def actualizar_pose(self, x: float, y: float, yaw: float, source: str = "odom"):
        x = float(x); y = float(y); yaw = float(yaw)
        self.last_update_source = str(source)

        if self.start_pose is None:
            self.start_pose = (x, y, yaw)
            self.start_time = time.time()

        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            d = math.hypot(dx, dy)
            if d < 0.50:
                self.total_distance += d

        self.last_pose = (x, y, yaw)
        self.visited_hits[self._cell(x, y)] += 1

        if self.last_path_point is None:
            self.path.append((x, y))
            self.last_path_point = (x, y)
        else:
            dx = x - self.last_path_point[0]
            dy = y - self.last_path_point[1]
            if math.hypot(dx, dy) >= self.path_min_step:
                self.path.append((x, y))
                self.last_path_point = (x, y)

        self._actualizar_vueltas(x, y)

    def _actualizar_vueltas(self, x: float, y: float):
        if self.start_pose is None:
            return
        sx, sy, _ = self.start_pose
        dist_inicio = math.hypot(x - sx, y - sy)
        tiempo = time.time() - self.start_time
        if (self.total_distance > self.min_lap_distance_m and
                tiempo > self.min_lap_time_s and
                dist_inicio < self.lap_radius and
                not self.lap_locked):
            self.lap_count += 1
            self.lap_locked = True
        if dist_inicio > self.lap_radius * 1.8:
            self.lap_locked = False

    # ------------------------------------------------------
    # MAPA DE OCUPACIÓN
    # ------------------------------------------------------
    def _cell(self, x: float, y: float) -> Cell:
        r = self.grid_resolution
        return (int(round(float(x) / r)), int(round(float(y) / r)))

    def _cell_center(self, cell: Cell) -> Point:
        r = self.grid_resolution
        return (cell[0] * r, cell[1] * r)

    def _ray_cells(self, x0: float, y0: float, x1: float, y1: float):
        """Celdas entre robot y punto LiDAR. Sirve para marcar espacio libre."""
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist <= 1e-6:
            return []
        steps = max(1, int(dist / (self.grid_resolution * 0.65)))
        out = []
        last = None
        # No incluir el último punto: ese será pared/obstáculo.
        for k in range(0, max(1, steps - 1), self.ray_stride_cells):
            t = k / float(steps)
            c = self._cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if c != last:
                out.append(c)
                last = c
        return out

    def _occupied_cells(self):
        occ = []
        for c, hits in self.occ_hits.items():
            free = self.free_hits.get(c, 0)
            if hits >= self.occ_hit_threshold and hits >= free * self.free_decay_weight:
                occ.append(c)
        return occ

    def _free_cells(self):
        occ_set = set(self._occupied_cells())
        cells = [c for c, hits in self.free_hits.items() if hits > 0 and c not in occ_set]
        return cells

    def actualizar_lidar(self, datos_lidar: Dict, odom_x: float, odom_y: float, yaw: float,
                         pared_der_valida: bool = False, pared_izq_valida: bool = False):
        if datos_lidar is None:
            return
        x_local = datos_lidar.get('x')
        y_local = datos_lidar.get('y')
        if x_local is None or y_local is None or len(x_local) == 0:
            return

        x_local = np.asarray(x_local, dtype=float)
        y_local = np.asarray(y_local, dtype=float)
        ranges = np.hypot(x_local, y_local)
        valid = np.isfinite(x_local) & np.isfinite(y_local) & (ranges > 0.07) & (ranges < self.max_map_range)

        mask_frente = np.asarray(datos_lidar.get('mask_frente', np.zeros_like(valid)), dtype=bool)
        idx_wall = np.where(valid)[0][::max(1, self.lidar_stride)]
        idx_obs = np.where(valid & mask_frente)[0][::max(1, self.lidar_stride)]

        for idx in idx_wall:
            wx, wy = self._local_a_odom(x_local[idx], y_local[idx], odom_x, odom_y, yaw)
            self.wall_points.append((wx, wy))
            c_occ = self._cell(wx, wy)
            self.occ_hits[c_occ] += 1
            # Marcar celdas libres entre robot y pared.
            for c in self._ray_cells(odom_x, odom_y, wx, wy):
                self.free_hits[c] += 1

        for idx in idx_obs:
            self.obstacle_points.append(self._local_a_odom(x_local[idx], y_local[idx], odom_x, odom_y, yaw))

        # La celda actual del robot siempre es libre/visitada.
        c_robot = self._cell(odom_x, odom_y)
        self.free_hits[c_robot] += 2
        self.visited_hits[c_robot] += 1

    def _local_a_odom(self, x_der: float, y_frente: float, ox: float, oy: float, yaw: float) -> Point:
        wx = ox + y_frente * math.cos(yaw) + x_der * math.sin(yaw)
        wy = oy + y_frente * math.sin(yaw) - x_der * math.cos(yaw)
        return (float(wx), float(wy))

    def obtener_mapa_ocupacion(self) -> Dict:
        occ = self._occupied_cells()[-self.max_occ_cells_to_draw:]
        free = self._free_cells()[-self.max_free_cells_to_draw:]
        visited = list(self.visited_hits.keys())[-self.max_visited_cells_to_draw:]
        return {
            "resolution": self.grid_resolution,
            "occupied_cells": [(c[0], c[1]) for c in occ],
            "free_cells": [(c[0], c[1]) for c in free],
            "visited_cells": [(c[0], c[1]) for c in visited],
            "occupied_centers": [self._cell_center(c) for c in occ],
            "free_centers": [self._cell_center(c) for c in free],
            "visited_centers": [self._cell_center(c) for c in visited],
        }

    # ------------------------------------------------------
    # ANTI-RETORNO SUAVE
    # ------------------------------------------------------
    def obtener_correccion_antiretorno(self, x: float, y: float, yaw: float) -> Dict:
        if self.lap_count >= 1:
            self.last_revisit_distance = float("inf")
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}
        if self.total_distance < self.min_route_before_avoid_m:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}
        if len(self.path) <= self.recent_points_to_ignore + 5:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}
        antiguos = list(self.path)[:-self.recent_points_to_ignore]
        if not antiguos:
            return {"activo": False, "bias_angular": 0.0, "distancia": float("inf")}
        arr = np.asarray(antiguos, dtype=float)
        dx = arr[:, 0] - x
        dy = arr[:, 1] - y
        d2 = dx * dx + dy * dy
        i = int(np.argmin(d2))
        d = math.sqrt(float(d2[i]))
        self.last_revisit_distance = d
        if d > self.revisit_radius:
            return {"activo": False, "bias_angular": 0.0, "distancia": d}
        vx = float(dx[i]); vy = float(dy[i])
        lateral_izq = -math.sin(yaw) * vx + math.cos(yaw) * vy
        if abs(lateral_izq) < 0.04:
            bias = -self.anti_return_bias
        else:
            bias = -self.anti_return_bias if lateral_izq > 0 else self.anti_return_bias
        return {"activo": True, "bias_angular": bias, "distancia": d}

    # ------------------------------------------------------
    # DATOS PARA GUI / REPORTE
    # ------------------------------------------------------
    def obtener_rectangulo_estimado(self) -> List[Point]:
        if len(self.path) < 12:
            return []
        arr = np.asarray(self.path, dtype=float)
        min_x, min_y = np.min(arr[:, 0]), np.min(arr[:, 1])
        max_x, max_y = np.max(arr[:, 0]), np.max(arr[:, 1])
        if (max_x - min_x) < 0.15 or (max_y - min_y) < 0.15:
            return []
        return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)]

    def obtener_datos_mapa(self) -> Dict:
        occ = self.obtener_mapa_ocupacion()
        return {
            "path": list(self.path),
            "walls": list(self.wall_points),
            "obstacles": list(self.obstacle_points),
            "rectangle": self.obtener_rectangulo_estimado(),
            "laps": self.lap_count,
            "total_distance": self.total_distance,
            "last_revisit_distance": self.last_revisit_distance,
            "map_source": self.last_update_source,
            **occ,
        }

    def obtener_estado_resumen(self) -> Dict:
        occ = self.obtener_mapa_ocupacion()
        return {
            "laps": self.lap_count,
            "distance": self.total_distance,
            "path_points": len(self.path),
            "wall_points": len(self.wall_points),
            "obstacle_points": len(self.obstacle_points),
            "occupied_cells": len(occ.get("occupied_cells", [])),
            "free_cells": len(occ.get("free_cells", [])),
            "visited_cells": len(occ.get("visited_cells", [])),
            "revisit_distance": self.last_revisit_distance,
            "map_source": self.last_update_source,
        }
