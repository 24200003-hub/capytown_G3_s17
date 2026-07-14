#!/usr/bin/env python3
"""
radar_utils.py
--------------
Procesamiento del LaserScan para conducción por el centro del pasillo.
Convención usada para la interfaz:
- Frente del robot = eje +Y
- Derecha del robot = eje +X
- Izquierda del robot = eje -X

Mejora clave:
La pared derecha/izquierda NO se acepta solo por una lectura suelta. Primero se
agrupan puntos consecutivos del LiDAR. Para que una línea sea pared lateral debe
estar conectada con el sector lateral real del robot, cerca de ±90°. Así se evita
que la pared del frente sea confundida como "derecha" cuando el robot llega a
una curva o a un fondo.
"""
import math
import time
import numpy as np


# ----------------------------------------------------------
# Calibración del LiDAR
# ----------------------------------------------------------
# Tu LiDAR está montado con el cero angular apuntando hacia atrás.
# Por eso el código debe rotar el scan 180° para que:
#   Frente físico del robot  ->  Frente lógico del programa
#   Derecha física           ->  Derecha lógica
#   Izquierda física         ->  Izquierda lógica
#
# Si al probar ves que vuelve a quedar al revés, cambia este valor a 0.0.
LIDAR_OFFSET_DEG = 180.0

# Si derecha e izquierda salen intercambiadas después del offset, cambia a -1.0.
# Normalmente debe quedarse en +1.0.
LIDAR_Y_SIGN = 1.0


# ----------------------------------------------------------
# Utilidades básicas
# ----------------------------------------------------------
def _normalizar_angulos(angles):
    return np.arctan2(np.sin(angles), np.cos(angles))


def _limpiar_ranges(msg, valor_reemplazo=5.0):
    arr = np.array(msg.ranges, dtype=float)
    arr[(arr == 0.0) | np.isinf(arr) | np.isnan(arr)] = valor_reemplazo
    arr[arr < msg.range_min] = valor_reemplazo
    arr[arr > msg.range_max] = valor_reemplazo
    return arr


def _distancia_sector(ranges, angles, centro_deg, ancho_deg, modo="percentil"):
    centro = math.radians(centro_deg)
    ancho = math.radians(ancho_deg)
    dif = np.abs(np.arctan2(np.sin(angles - centro), np.cos(angles - centro)))
    vals = ranges[dif <= ancho]
    vals = vals[np.isfinite(vals)]
    vals = vals[(vals > 0.02) & (vals < 4.90)]

    if vals.size == 0:
        return float("inf")

    if modo == "min":
        return float(np.min(vals))

    # Percentil bajo: evita ruido extremo pero sigue reaccionando a objetos cercanos.
    return float(np.percentile(vals, 20))


# ----------------------------------------------------------
# Agrupación de líneas laterales
# ----------------------------------------------------------
def _clusters_por_continuidad(indices, x, y, ranges, gap_m=0.18, salto_rango_m=0.18):
    """Agrupa puntos consecutivos si están cerca en distancia euclidiana y rango."""
    if len(indices) == 0:
        return []

    clusters = []
    actual = [indices[0]]

    for prev, cur in zip(indices[:-1], indices[1:]):
        dx = x[cur] - x[prev]
        dy = y[cur] - y[prev]
        gap = math.hypot(dx, dy)
        salto = abs(ranges[cur] - ranges[prev])

        if gap > gap_m or salto > salto_rango_m:
            clusters.append(np.array(actual, dtype=int))
            actual = [cur]
        else:
            actual.append(cur)

    clusters.append(np.array(actual, dtype=int))
    return clusters


def _extraer_pared_lateral(x, y, angles, ranges, lado="derecha"):
    """
    Devuelve la línea lateral conectada al sector ±90°.

    lado="derecha": acepta solo clusters del sector -120° a -25° que además
    toquen la zona lateral -105° a -75°.

    lado="izquierda": acepta solo clusters del sector 25° a 120° que además
    toquen la zona lateral 75° a 105°.
    """
    if lado == "derecha":
        sector = (
            (angles >= math.radians(-120)) &
            (angles <= math.radians(-25)) &
            (x > 0.04)
        )
        anchor_min = math.radians(-105)
        anchor_max = math.radians(-75)
        dist_signo = 1.0
    else:
        sector = (
            (angles >= math.radians(25)) &
            (angles <= math.radians(120)) &
            (x < -0.04)
        )
        anchor_min = math.radians(75)
        anchor_max = math.radians(105)
        dist_signo = -1.0

    valid = (
        sector &
        np.isfinite(ranges) &
        (ranges > 0.08) &
        (ranges < 2.20) &
        (y > -0.25) &
        (y < 2.40)
    )

    indices = np.where(valid)[0]
    if indices.size < 6:
        return {
            "pendiente": 0.0,
            "valida": False,
            "puntos": 0,
            "distancia": float("inf"),
            "anclada": False,
            "y_extent": 0.0,
        }

    # Ordenar por ángulo para que los clusters sigan la geometría del LaserScan.
    indices = indices[np.argsort(angles[indices])]
    clusters = _clusters_por_continuidad(indices, x, y, ranges)

    mejor = None
    mejor_score = -1.0

    for c in clusters:
        if c.size < 6:
            continue

        ang_c = angles[c]
        y_c = y[c]
        x_c = x[c]

        # Debe tocar el sector lateral real. Esto evita confundir pared frontal.
        anclada = np.any((ang_c >= anchor_min) & (ang_c <= anchor_max))
        if not anclada:
            continue

        y_extent = float(np.max(y_c) - np.min(y_c)) if y_c.size > 0 else 0.0
        if y_extent < 0.14:
            # Cluster demasiado pequeño: probablemente caja/ruido, no pared continua.
            continue

        distancia = float(np.median(dist_signo * x_c))
        if not math.isfinite(distancia) or distancia < 0.06 or distancia > 1.70:
            continue

        score = y_extent + 0.015 * c.size
        if score > mejor_score:
            mejor = c
            mejor_score = score

    if mejor is None:
        return {
            "pendiente": 0.0,
            "valida": False,
            "puntos": 0,
            "distancia": float("inf"),
            "anclada": False,
            "y_extent": 0.0,
        }

    x_m = x[mejor]
    y_m = y[mejor]

    try:
        a, _b = np.polyfit(y_m, x_m, 1)  # x = a*y + b
    except Exception:
        a = 0.0

    if not np.isfinite(a):
        a = 0.0

    a = float(max(-1.0, min(1.0, a)))
    distancia = float(np.median(dist_signo * x_m))
    y_extent = float(np.max(y_m) - np.min(y_m))

    return {
        "pendiente": a,
        "valida": True,
        "puntos": int(mejor.size),
        "distancia": distancia,
        "anclada": True,
        "y_extent": y_extent,
    }


# ----------------------------------------------------------
# Análisis de frente: pasadizo vs esquina
# ----------------------------------------------------------
def _analizar_frente_pasillo_esquina(x, y, angles, ranges, pared_der, pared_izq, dist_frente):
    """Clasifica lo que hay delante del robot.

    - PASADIZO: hay paredes laterales a ambos lados y el frente está abierto.
    - ESQUINA: existe una pared/obstáculo al frente que pertenece al mismo
      cluster que una pared lateral izquierda o derecha. Esa conexión evita
      confundir una pared frontal suelta con una pared lateral.
    - BLOQUEO_FRONTAL: hay algo al frente, pero no se confirma conexión lateral.
    """
    ambos_lados = bool(pared_der.get("valida", False) and pared_izq.get("valida", False))

    # Frente libre + dos laterales = pasadizo claro.
    if ambos_lados and dist_frente > 0.46:
        return {
            "tipo": "PASADIZO",
            "es_pasillo": True,
            "es_esquina": False,
            "conecta_der": False,
            "conecta_izq": False,
            "conecta_ambos": False,
            "confianza": 1.0,
        }

    # Analiza clusters que incluyan frente y puedan continuar hacia algún lateral.
    valid = (
        np.isfinite(ranges) &
        (ranges > 0.08) &
        (ranges < 1.35) &
        (y > -0.10) &
        (np.abs(angles) <= math.radians(130))
    )
    indices = np.where(valid)[0]

    conecta_der = False
    conecta_izq = False
    puntos_frente = 0

    if indices.size >= 5:
        indices = indices[np.argsort(angles[indices])]
        clusters = _clusters_por_continuidad(indices, x, y, ranges, gap_m=0.16, salto_rango_m=0.16)

        for c in clusters:
            if c.size < 5:
                continue

            a = angles[c]
            x_c = x[c]
            y_c = y[c]
            r_c = ranges[c]

            # Frontal cercano: sector estrecho delante del robot.
            front_mask = (np.abs(a) <= math.radians(28)) & (y_c > 0.04) & (r_c < 1.05)
            has_front = bool(np.any(front_mask))
            if not has_front:
                continue

            puntos_frente += int(np.count_nonzero(front_mask))

            # Misma pieza geométrica se extiende hacia lateral: esquina/intersección.
            has_right = bool(np.any((a <= math.radians(-35)) & (a >= math.radians(-125)) & (x_c > 0.04)))
            has_left = bool(np.any((a >= math.radians(35)) & (a <= math.radians(125)) & (x_c < -0.04)))

            # También se acepta si el extremo frontal queda muy cerca de la línea lateral validada.
            if pared_der.get("valida", False):
                has_right = has_right or bool(np.any((x_c > 0.04) & (x_c < pared_der["distancia"] + 0.13)))
            if pared_izq.get("valida", False):
                has_left = has_left or bool(np.any((x_c < -0.04) & ((-x_c) < pared_izq["distancia"] + 0.13)))

            conecta_der = conecta_der or has_right
            conecta_izq = conecta_izq or has_left

    if dist_frente < 0.68 and (conecta_der or conecta_izq):
        return {
            "tipo": "ESQUINA",
            "es_pasillo": False,
            "es_esquina": True,
            "conecta_der": conecta_der,
            "conecta_izq": conecta_izq,
            "conecta_ambos": conecta_der and conecta_izq,
            "confianza": min(1.0, 0.50 + 0.04 * puntos_frente),
        }

    if dist_frente < 0.38:
        return {
            "tipo": "BLOQUEO_FRONTAL",
            "es_pasillo": False,
            "es_esquina": False,
            "conecta_der": conecta_der,
            "conecta_izq": conecta_izq,
            "conecta_ambos": conecta_der and conecta_izq,
            "confianza": min(0.8, 0.25 + 0.03 * puntos_frente),
        }

    return {
        "tipo": "INDEFINIDO",
        "es_pasillo": ambos_lados,
        "es_esquina": False,
        "conecta_der": conecta_der,
        "conecta_izq": conecta_izq,
        "conecta_ambos": conecta_der and conecta_izq,
        "confianza": 0.3 if ambos_lados else 0.0,
    }



# ----------------------------------------------------------
# Segmentación tipo CapyTown Guardian: Split & Merge ligero
# ----------------------------------------------------------
def _perp_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return math.hypot(px - ax, py - ay)
    return abs(dy * px - dx * py + bx * ay - by * ax) / L


def _split_segmento_indices(idx, x, y, split_thresh=0.06, min_pts=4):
    """Split recursivo sobre índices ya ordenados por ángulo."""
    if len(idx) < max(2, min_pts):
        return [idx]

    ax, ay = float(x[idx[0]]), float(y[idx[0]])
    bx, by = float(x[idx[-1]]), float(y[idx[-1]])
    dmax = -1.0
    imax = 0
    for k, ii in enumerate(idx):
        d = _perp_dist(float(x[ii]), float(y[ii]), ax, ay, bx, by)
        if d > dmax:
            dmax = d
            imax = k

    if dmax > split_thresh and 0 < imax < len(idx) - 1:
        izq = _split_segmento_indices(idx[:imax + 1], x, y, split_thresh, min_pts)
        der = _split_segmento_indices(idx[imax:], x, y, split_thresh, min_pts)
        return izq + der
    return [idx]


def _segmentos_split_merge(x, y, angles, ranges,
                           split_thresh=0.06,
                           pre_gap_m=0.35,
                           pre_gap_r=0.35,
                           min_pts=4,
                           min_len=0.08):
    """Extrae segmentos lineales del scan.

    Está inspirado en capytown_guardian.py, pero adaptado a este proyecto:
    frente = +Y, derecha = +X, izquierda = -X.
    """
    valid = (
        np.isfinite(ranges) &
        (ranges > 0.05) &
        (ranges < 3.50) &
        np.isfinite(x) & np.isfinite(y)
    )
    indices = np.where(valid)[0]
    if indices.size < min_pts:
        return []

    # Orden angular para conservar continuidad geométrica del LaserScan.
    indices = indices[np.argsort(angles[indices])]

    grupos = []
    actual = [int(indices[0])]
    for prev, cur in zip(indices[:-1], indices[1:]):
        prev = int(prev); cur = int(cur)
        gap = math.hypot(float(x[cur] - x[prev]), float(y[cur] - y[prev]))
        salto = abs(float(ranges[cur] - ranges[prev]))
        if gap > pre_gap_m or salto > pre_gap_r:
            if len(actual) >= min_pts:
                grupos.append(np.array(actual, dtype=int))
            actual = [cur]
        else:
            actual.append(cur)
    if len(actual) >= min_pts:
        grupos.append(np.array(actual, dtype=int))

    segmentos = []
    for g in grupos:
        for idx in _split_segmento_indices(g, x, y, split_thresh, min_pts):
            if len(idx) < min_pts:
                continue
            p1 = (float(x[idx[0]]), float(y[idx[0]]))
            p2 = (float(x[idx[-1]]), float(y[idx[-1]]))
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            lon = math.hypot(dx, dy)
            if lon < min_len:
                continue

            # alpha_lateral: ángulo de la línea respecto al eje frontal +Y.
            # 0 = pared paralela al robot. Signo ≈ pendiente x = a*y.
            alpha_lateral = math.atan2(dx, dy)
            if alpha_lateral > math.pi / 2:
                alpha_lateral -= math.pi
            if alpha_lateral < -math.pi / 2:
                alpha_lateral += math.pi

            segmentos.append({
                'idx': idx,
                'p1': p1,
                'p2': p2,
                'lon': float(lon),
                'mean_x': float(np.mean(x[idx])),
                'mean_y': float(np.mean(y[idx])),
                'alpha': float(max(-1.2, min(1.2, alpha_lateral))),
                'x_extent': float(np.max(x[idx]) - np.min(x[idx])),
                'y_extent': float(np.max(y[idx]) - np.min(y[idx])),
                'puntos': int(len(idx)),
            })
    return segmentos


def _segmento_lateral_guardian(segmentos, lado="derecha"):
    """Selecciona el segmento lateral más cercano usando Split & Merge.

    A diferencia de una lectura puntual, exige que la línea tenga longitud y que
    sea casi paralela al eje de avance. Esto da un alpha útil para el controlador.
    """
    mejor = None
    mejor_score = -1e9
    for s in segmentos:
        if s['lon'] < 0.12:
            continue
        # Debe ir principalmente en dirección frontal (+Y/-Y), no ser pared frontal.
        dy_abs = abs(s['p2'][1] - s['p1'][1])
        if dy_abs / max(s['lon'], 1e-6) < 0.55:
            continue

        if lado == "derecha":
            if s['mean_x'] < 0.06:
                continue
            dist = s['mean_x']
        else:
            if s['mean_x'] > -0.06:
                continue
            dist = -s['mean_x']

        if not math.isfinite(dist) or dist < 0.06 or dist > 1.80:
            continue

        # Score: cercano + largo. Evita tomar pedacitos de caja si hay pared real.
        score = (1.5 * s['lon']) - (0.35 * abs(dist)) + (0.01 * s['puntos'])
        if score > mejor_score:
            mejor = dict(s)
            mejor['distancia'] = float(dist)
            mejor_score = score
    return mejor


def _front_arc_guardian(angles, ranges, dist_izq, dist_der,
                        front_sector_deg=35.0,
                        front_alert_dist=0.62,
                        front_arc_radial=0.10,
                        front_wall_deg=45.0,
                        front_box_deg=30.0,
                        side_clear_dist=0.60):
    """Clasifica el obstáculo frontal por ancho angular.

    Regla tomada del capytown_guardian:
    - arco corto  => caja
    - arco ancho  => pared/esquina
    - zona ambigua se decide por hueco lateral.
    """
    sector = math.radians(front_sector_deg)
    valid = (
        np.isfinite(ranges) &
        (ranges > 0.05) &
        (ranges < 3.50) &
        (np.abs(angles) <= sector)
    )
    idx = np.where(valid)[0]
    if idx.size == 0:
        return float('inf'), 0.0, 'NONE'

    # Ordenar por ángulo y agrupar por continuidad radial.
    idx = idx[np.argsort(angles[idx])]
    grupos = []
    actual = [int(idx[0])]
    for prev, cur in zip(idx[:-1], idx[1:]):
        prev = int(prev); cur = int(cur)
        if abs(float(ranges[cur] - ranges[prev])) <= front_arc_radial:
            actual.append(cur)
        else:
            grupos.append(np.array(actual, dtype=int))
            actual = [cur]
    grupos.append(np.array(actual, dtype=int))

    # Escoger el grupo cuyo punto más cercano sea el más cercano global.
    mejor = None
    mejor_d = float('inf')
    for g in grupos:
        if g.size == 0:
            continue
        dm = float(np.min(ranges[g]))
        if dm < mejor_d:
            mejor = g
            mejor_d = dm

    if mejor is None:
        return float('inf'), 0.0, 'NONE'

    width = float(np.max(angles[mejor]) - np.min(angles[mejor]))
    width = abs(width)

    if mejor_d > front_alert_dist:
        return mejor_d, width, 'NONE'

    wall_ang = math.radians(front_wall_deg)
    box_ang = math.radians(front_box_deg)

    if width >= wall_ang:
        clase = 'CORNER'
    elif width <= box_ang:
        clase = 'BOX'
    else:
        # Si hay hueco lateral, se parece a una esquina/pasadizo abierto;
        # si no, se trata como caja/obstáculo corto para rodearlo.
        clase = 'CORNER' if (dist_izq > side_clear_dist or dist_der > side_clear_dist) else 'BOX'

    return mejor_d, width, clase

# ----------------------------------------------------------
# Función principal
# ----------------------------------------------------------
def procesar_escaneo_lidar(msg):
    """
    Limpia, convierte a cartesiano, calcula sectores y estima líneas laterales.
    Además entrega distancias laterales conectadas para conducir por el centro.
    """
    t0 = time.perf_counter()

    ranges = _limpiar_ranges(msg)
    n = len(ranges)

    if n == 0:
        return {
            'x': np.array([]), 'y': np.array([]),
            'mask_frente': np.array([], dtype=bool),
            'mask_izq': np.array([], dtype=bool),
            'mask_der': np.array([], dtype=bool),
            'dist_frente': float('inf'),
            'dist_izq': float('inf'),
            'dist_der': float('inf'),
            'dist_diag_der': float('inf'),
            'dist_diag_izq': float('inf'),
            'dist_der_pared': float('inf'),
            'dist_izq_pared': float('inf'),
            'pared_der_pendiente': 0.0,
            'pared_izq_pendiente': 0.0,
            'pared_der_valida': False,
            'pared_izq_valida': False,
            'pared_der_anclada': False,
            'pared_izq_anclada': False,
            'puntos_pared_der': 0,
            'puntos_pared_izq': 0,
            'ancho_pasillo': float('inf'),
            'error_centro': 0.0,
            'tipo_frente': 'SIN_DATOS',
            'frente_es_pasillo': False,
            'frente_es_esquina': False,
            'frente_conecta_der': False,
            'frente_conecta_izq': False,
            'frente_conecta_ambos': False,
            'frente_confianza': 0.0,
            'front_ang_width': 0.0,
            'front_class_sm': 'NONE',
            'front_dist_sm': float('inf'),
            'pared_der_alpha': 0.0,
            'pared_izq_alpha': 0.0,
            'pared_der_len': 0.0,
            'pared_izq_len': 0.0,
            'num_puntos': 0,
            'tiempo_proc_ms': 0.0,
        }

    angles_raw = msg.angle_min + np.arange(n) * msg.angle_increment

    # Corrige el marco angular del LiDAR al marco real del robot.
    # Antes el programa asumía que 0° del LiDAR miraba al frente,
    # pero en este robot el 0° está mirando hacia atrás.
    angles = LIDAR_Y_SIGN * angles_raw + math.radians(LIDAR_OFFSET_DEG)
    angles = _normalizar_angulos(angles)

    # Frente = +Y; derecha = +X.
    x_coords = -ranges * np.sin(angles)
    y_coords = ranges * np.cos(angles)

    mask_frente = np.abs(angles) <= math.radians(20)
    mask_izq = (angles >= math.radians(35)) & (angles <= math.radians(115))
    mask_der = (angles <= math.radians(-35)) & (angles >= math.radians(-115))

    dist_frente = _distancia_sector(ranges, angles, 0, 18, modo="min")
    dist_diag_der = _distancia_sector(ranges, angles, -45, 16, modo="percentil")
    dist_der = _distancia_sector(ranges, angles, -90, 20, modo="percentil")
    dist_diag_izq = _distancia_sector(ranges, angles, 45, 16, modo="percentil")
    dist_izq = _distancia_sector(ranges, angles, 90, 22, modo="percentil")

    pared_der = _extraer_pared_lateral(x_coords, y_coords, angles, ranges, lado="derecha")
    pared_izq = _extraer_pared_lateral(x_coords, y_coords, angles, ranges, lado="izquierda")

    # Adaptación de capytown_guardian: segmentos Split & Merge para obtener
    # alpha real de pared y ancho angular frontal caja/esquina.
    segmentos_sm = _segmentos_split_merge(x_coords, y_coords, angles, ranges)
    pared_der_sm = _segmento_lateral_guardian(segmentos_sm, lado="derecha")
    pared_izq_sm = _segmento_lateral_guardian(segmentos_sm, lado="izquierda")

    # Si Split&Merge encuentra una línea más fuerte, se usa para el controlador.
    if pared_der_sm is not None:
        pared_der["pendiente"] = pared_der_sm["alpha"]
        pared_der["distancia"] = pared_der_sm["distancia"]
        pared_der["valida"] = True
        pared_der["puntos"] = max(pared_der.get("puntos", 0), pared_der_sm["puntos"])
        pared_der["y_extent"] = max(pared_der.get("y_extent", 0.0), pared_der_sm["y_extent"])
    if pared_izq_sm is not None:
        pared_izq["pendiente"] = pared_izq_sm["alpha"]
        pared_izq["distancia"] = pared_izq_sm["distancia"]
        pared_izq["valida"] = True
        pared_izq["puntos"] = max(pared_izq.get("puntos", 0), pared_izq_sm["puntos"])
        pared_izq["y_extent"] = max(pared_izq.get("y_extent", 0.0), pared_izq_sm["y_extent"])

    dist_der_pared = pared_der["distancia"]
    dist_izq_pared = pared_izq["distancia"]

    if pared_der["valida"] and pared_izq["valida"]:
        ancho_pasillo = dist_der_pared + dist_izq_pared
        # Positivo = está más cerca de derecha y debe corregir a izquierda.
        error_centro = dist_izq_pared - dist_der_pared
    else:
        ancho_pasillo = float("inf")
        error_centro = 0.0

    frente_geo = _analizar_frente_pasillo_esquina(
        x_coords, y_coords, angles, ranges, pared_der, pared_izq, dist_frente
    )

    front_dist_sm, front_ang_width, front_class_sm = _front_arc_guardian(
        angles, ranges, dist_izq, dist_der
    )

    # Filtro anti-chasis / soporte frontal:
    # si aparece un obstáculo extremadamente cercano pero con arco angular muy
    # pequeño, normalmente es una pieza/cable del propio robot y no una pared.
    # Se ignora solo en ese caso estrecho para no empezar en falso modo choque.
    falso_chasis_frontal = (
        0.07 <= dist_frente <= 0.18 and
        front_class_sm == 'BOX' and
        front_ang_width <= math.radians(8.0)
    )
    if falso_chasis_frontal:
        ranges_sin_chasis = ranges.copy()
        mask_chasis = (
            (np.abs(angles) <= math.radians(18)) &
            (ranges_sin_chasis >= 0.05) &
            (ranges_sin_chasis <= 0.20)
        )
        ranges_sin_chasis[mask_chasis] = 5.0
        dist_frente = _distancia_sector(ranges_sin_chasis, angles, 0, 18, modo="min")
        front_dist_sm, front_ang_width, front_class_sm = _front_arc_guardian(
            angles, ranges_sin_chasis, dist_izq, dist_der
        )

    # Fusión de clasificadores: si el análisis geométrico ya confirmó esquina,
    # mantiene ESQUINA aunque el arco caiga en zona ambigua.
    if frente_geo['es_esquina'] and front_class_sm != 'BOX':
        front_class_sm = 'CORNER'
    elif frente_geo['tipo'] == 'BLOQUEO_FRONTAL' and front_class_sm == 'NONE':
        front_class_sm = 'BOX'

    t_ms = (time.perf_counter() - t0) * 1000.0

    return {
        'x': x_coords,
        'y': y_coords,
        'mask_frente': mask_frente,
        'mask_izq': mask_izq,
        'mask_der': mask_der,
        'dist_frente': dist_frente,
        'dist_izq': dist_izq,
        'dist_der': dist_der,
        'dist_diag_der': dist_diag_der,
        'dist_diag_izq': dist_diag_izq,
        'dist_der_pared': dist_der_pared,
        'dist_izq_pared': dist_izq_pared,
        'pared_der_pendiente': pared_der["pendiente"],
        'pared_izq_pendiente': pared_izq["pendiente"],
        'pared_der_valida': pared_der["valida"],
        'pared_izq_valida': pared_izq["valida"],
        'pared_der_anclada': pared_der["anclada"],
        'pared_izq_anclada': pared_izq["anclada"],
        'puntos_pared_der': pared_der["puntos"],
        'puntos_pared_izq': pared_izq["puntos"],
        'ancho_pasillo': ancho_pasillo,
        'error_centro': error_centro,
        'tipo_frente': frente_geo['tipo'],
        'frente_es_pasillo': frente_geo['es_pasillo'],
        'frente_es_esquina': frente_geo['es_esquina'],
        'frente_conecta_der': frente_geo['conecta_der'],
        'frente_conecta_izq': frente_geo['conecta_izq'],
        'frente_conecta_ambos': frente_geo['conecta_ambos'],
        'frente_confianza': frente_geo['confianza'],
        'front_ang_width': front_ang_width,
        'front_class_sm': front_class_sm,
        'front_dist_sm': front_dist_sm,
        'pared_der_alpha': pared_der["pendiente"],
        'pared_izq_alpha': pared_izq["pendiente"],
        'pared_der_len': pared_der_sm["lon"] if pared_der_sm is not None else pared_der.get("y_extent", 0.0),
        'pared_izq_len': pared_izq_sm["lon"] if pared_izq_sm is not None else pared_izq.get("y_extent", 0.0),
        'num_puntos': n,
        'tiempo_proc_ms': t_ms,
    }
