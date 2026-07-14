#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button


class RadarInterface:
    def __init__(
        self,
        callback_iniciar=None,
        callback_detener=None,
        callback_salir=None,
        callback_vel_lenta=None,
        callback_vel_media=None,
        callback_vel_rapida=None,
        callback_ang_menos=None,
        callback_ang_mas=None,
        callback_cargar_mapa=None,
        callback_usar_mapa=None,
    ):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 8.2), facecolor='#0b0b0e')
        self.fig.canvas.manager.set_window_title('CapyTown LiDAR — centrado + mapeo')

        gs = gridspec.GridSpec(1, 2, width_ratios=[1.08, 0.95], figure=self.fig)
        gs.update(left=0.06, right=0.96, top=0.91, bottom=0.23, wspace=0.22)

        # =====================================================
        # Vista local LiDAR
        # =====================================================
        self.ax_cartesian = self.fig.add_subplot(gs[0, 0], facecolor='#111116')
        self.ax_cartesian.set_title(
            "LiDAR local — centro entre pared izquierda y derecha",
            color='#e0e0e0', fontsize=12, pad=12, weight='bold'
        )

        self.scatter_frente = self.ax_cartesian.scatter([], [], c='#ff2a5f', s=10, label='FRENTE')
        self.scatter_izq = self.ax_cartesian.scatter([], [], c='#00f0ff', s=8, label='IZQ')
        self.scatter_der = self.ax_cartesian.scatter([], [], c='#00ff66', s=8, label='DER')
        self.ax_cartesian.plot(0, 0, marker='^', color='white', markersize=14,
                               markeredgecolor='#ffcc00', markeredgewidth=1.5, label='Robot')
        self.ax_cartesian.set_xlim(-1.6, 1.6)
        self.ax_cartesian.set_ylim(-0.4, 2.6)
        self.ax_cartesian.set_xlabel('← IZQ     DER →', color='#cfcfcf')
        self.ax_cartesian.set_ylabel('FRENTE ↑', color='#cfcfcf')
        self.ax_cartesian.grid(True, color='#252530', linestyle='--', linewidth=0.7)
        self.ax_cartesian.legend(loc='upper right', facecolor='#111116', edgecolor='#666666')
        self.linea_objetivo, = self.ax_cartesian.plot([], [], linestyle='--', linewidth=1.5, color='#ffcc00')

        # =====================================================
        # Panel de telemetría
        # =====================================================
        self.ax_text = self.fig.add_subplot(gs[0, 1], facecolor='#0b0b0e')
        self.ax_text.axis('off')
        # Panel de telemetría más compacto para dejar espacio visible a la cámara.
        self.ax_text.set_position([0.575, 0.465, 0.390, 0.430])

        # =====================================================
        # Botones: sin batería manual. La batería ahora solo se muestra desde /battery.
        # =====================================================
        self.ax_btn_lento = self.fig.add_axes([0.06, 0.150, 0.090, 0.048])
        self.ax_btn_medio = self.fig.add_axes([0.160, 0.150, 0.090, 0.048])
        self.ax_btn_rapido = self.fig.add_axes([0.260, 0.150, 0.090, 0.048])
        self.ax_btn_iniciar = self.fig.add_axes([0.370, 0.150, 0.150, 0.048])

        self.ax_btn_detener = self.fig.add_axes([0.06, 0.080, 0.190, 0.048])
        self.ax_btn_salir = self.fig.add_axes([0.270, 0.080, 0.105, 0.048])
        self.ax_btn_ang_menos = self.fig.add_axes([0.395, 0.080, 0.075, 0.048])
        self.ax_btn_ang_mas = self.fig.add_axes([0.485, 0.080, 0.075, 0.048])
        # Botones del modo experimental de mapa aprendido.
        # Antes estaban demasiado abajo (y=0.005) y en WayVNC podían quedar
        # ocultos fuera de la ventana. Se suben a la zona visible derecha.
        self.ax_btn_cargar_mapa = self.fig.add_axes([0.590, 0.152, 0.170, 0.046])
        self.ax_btn_usar_mapa = self.fig.add_axes([0.785, 0.152, 0.170, 0.046])

        self.btn_lento = Button(self.ax_btn_lento, 'LENTO', color='#37474f', hovercolor='#546e7a')
        self.btn_medio = Button(self.ax_btn_medio, 'MEDIO', color='#00695c', hovercolor='#00897b')
        self.btn_rapido = Button(self.ax_btn_rapido, 'RÁPIDO', color='#ef6c00', hovercolor='#fb8c00')
        self.btn_iniciar = Button(self.ax_btn_iniciar, 'INICIAR', color='#2e7d32', hovercolor='#4caf50')
        self.btn_detener = Button(self.ax_btn_detener, 'PAUSAR / REANUDAR', color='#d32f2f', hovercolor='#f44336')
        self.btn_salir = Button(self.ax_btn_salir, 'SALIR', color='#455a64', hovercolor='#607d8b')
        self.btn_ang_menos = Button(self.ax_btn_ang_menos, 'ANG -', color='#303f9f', hovercolor='#3f51b5')
        self.btn_ang_mas = Button(self.ax_btn_ang_mas, 'ANG +', color='#303f9f', hovercolor='#3f51b5')
        self.btn_cargar_mapa = Button(self.ax_btn_cargar_mapa, 'CARGAR MAPA', color='#1565c0', hovercolor='#1976d2')
        self.btn_usar_mapa = Button(self.ax_btn_usar_mapa, 'USAR MAPA', color='#6a1b9a', hovercolor='#8e24aa')

        for b in [
            self.btn_iniciar, self.btn_detener, self.btn_salir,
            self.btn_lento, self.btn_medio, self.btn_rapido,
            self.btn_ang_menos, self.btn_ang_mas,
            self.btn_cargar_mapa, self.btn_usar_mapa,
        ]:
            b.label.set_color('white')
            b.label.set_weight('bold')

        if callback_iniciar:
            self.btn_iniciar.on_clicked(callback_iniciar)
        if callback_detener:
            self.btn_detener.on_clicked(callback_detener)
        if callback_salir:
            self.btn_salir.on_clicked(callback_salir)
        if callback_vel_lenta:
            self.btn_lento.on_clicked(callback_vel_lenta)
        if callback_vel_media:
            self.btn_medio.on_clicked(callback_vel_media)
        if callback_vel_rapida:
            self.btn_rapido.on_clicked(callback_vel_rapida)
        if callback_ang_menos:
            self.btn_ang_menos.on_clicked(callback_ang_menos)
        if callback_ang_mas:
            self.btn_ang_mas.on_clicked(callback_ang_mas)
        if callback_cargar_mapa:
            self.btn_cargar_mapa.on_clicked(callback_cargar_mapa)
        if callback_usar_mapa:
            self.btn_usar_mapa.on_clicked(callback_usar_mapa)

        # =====================================================
        # Mapa global compacto: ruta, paredes y puntos detectados.
        # =====================================================
        # Vista de cámara en vivo. Se muestra en la misma interfaz para ver
        # exactamente lo que está viendo el robot al buscar señales PARE.
        # Cámara más grande y más arriba: permite validar qué ve el robot
        # antes de activar la detención PARE.
        self.ax_cam = self.fig.add_axes([0.590, 0.225, 0.365, 0.230], facecolor='#111116')
        self.ax_cam.set_title('Cámara PARE — vista del robot', color='#e0e0e0', fontsize=10, pad=4)
        self.ax_cam.axis('off')
        self._cam_placeholder = np.zeros((180, 320, 3), dtype=np.uint8)
        self._cam_placeholder[:, :] = (18, 18, 24)
        self.img_cam = self.ax_cam.imshow(self._cam_placeholder)
        self.txt_cam = self.ax_cam.text(
            0.5, 0.5, 'esperando cámara', transform=self.ax_cam.transAxes,
            ha='center', va='center', color='#eeeeee', fontsize=10, weight='bold'
        )

        # Mapa compacto más bajo para dejar libres los botones CARGAR/USAR MAPA.
        self.ax_mapa = self.fig.add_axes([0.590, 0.005, 0.365, 0.130], facecolor='#111116')
        self.ax_mapa.set_title('Mapa compacto', color='#e0e0e0', fontsize=9, pad=4)
        self.line_path, = self.ax_mapa.plot([], [], color='#00e676', linewidth=1.8, label='recorrido')
        self.scatter_free = self.ax_mapa.scatter([], [], c='#e8f5e9', s=18, marker='s', alpha=0.25, label='libre')
        self.scatter_visited = self.ax_mapa.scatter([], [], c='#66bb6a', s=18, marker='s', alpha=0.35, label='visitado')
        self.scatter_occ = self.ax_mapa.scatter([], [], c='#bf6f2e', s=20, marker='s', alpha=0.80, label='pared grid')
        self.scatter_walls = self.ax_mapa.scatter([], [], c='#9e9e9e', s=3, alpha=0.35, label='puntos')
        self.scatter_obs = self.ax_mapa.scatter([], [], c='#ff9800', s=5, alpha=0.75, label='puntos')
        self.scatter_decisions = self.ax_mapa.scatter([], [], c='#00e676', s=18, marker='D', alpha=0.90, label='decisiones')
        self.scatter_deadends = self.ax_mapa.scatter([], [], c='#ff1744', s=24, marker='x', alpha=0.95, label='sin salida')
        self.line_rect, = self.ax_mapa.plot([], [], color='#ffeb3b', linewidth=1.2, linestyle='--', label='contorno')
        self.ax_mapa.grid(True, color='#252530', linestyle='--', linewidth=0.5)
        self.ax_mapa.tick_params(axis='both', labelsize=7, colors='#cccccc')
        self.ax_mapa.legend(loc='upper right', fontsize=6, facecolor='#111116', edgecolor='#666666')

    def actualizar_graficos(self, datos_lidar, distancia_objetivo=0.25):
        if datos_lidar is None:
            return

        x, y = datos_lidar['x'], datos_lidar['y']
        self.scatter_frente.set_offsets(list(zip(x[datos_lidar['mask_frente']], y[datos_lidar['mask_frente']])))
        self.scatter_izq.set_offsets(list(zip(x[datos_lidar['mask_izq']], y[datos_lidar['mask_izq']])))
        self.scatter_der.set_offsets(list(zip(x[datos_lidar['mask_der']], y[datos_lidar['mask_der']])))
        self.linea_objetivo.set_data([0.0, 0.0], [-0.2, 2.2])

    def actualizar_camara(self, frame_rgb=None, estado='SIN_CAMARA'):
        """Actualiza la vista de cámara embebida en la interfaz.

        frame_rgb debe venir en RGB. Si no hay cámara o no llegó frame,
        deja un panel oscuro con el estado para diagnosticar rápido.
        """
        try:
            if frame_rgb is None:
                self.img_cam.set_data(self._cam_placeholder)
                self.txt_cam.set_text(str(estado))
                self.txt_cam.set_visible(True)
                return

            arr = np.asarray(frame_rgb)
            if arr.ndim != 3 or arr.shape[2] != 3 or arr.size == 0:
                self.img_cam.set_data(self._cam_placeholder)
                self.txt_cam.set_text(str(estado))
                self.txt_cam.set_visible(True)
                return

            self.img_cam.set_data(arr)
            self.txt_cam.set_visible(False)
        except Exception:
            # La cámara nunca debe tumbar la interfaz ni la navegación.
            try:
                self.img_cam.set_data(self._cam_placeholder)
                self.txt_cam.set_text('error cámara')
                self.txt_cam.set_visible(True)
            except Exception:
                pass

    def actualizar_mapa_recorrido(self, mapa):
        if not mapa:
            return

        path = mapa.get('path', [])
        walls = mapa.get('walls', [])
        obstacles = mapa.get('obstacles', [])
        rectangle = mapa.get('rectangle', [])
        decisions = mapa.get('decisions', [])
        deadends = mapa.get('deadends', [])
        free_centers = mapa.get('free_centers', [])
        visited_centers = mapa.get('visited_centers', [])
        occupied_centers = mapa.get('occupied_centers', [])

        if path:
            px, py = zip(*path)
            self.line_path.set_data(px, py)
        else:
            px, py = [], []
            self.line_path.set_data([], [])

        self.scatter_free.set_offsets(np.asarray(free_centers[-900:]) if free_centers else np.empty((0, 2)))
        self.scatter_visited.set_offsets(np.asarray(visited_centers[-600:]) if visited_centers else np.empty((0, 2)))
        self.scatter_occ.set_offsets(np.asarray(occupied_centers[-900:]) if occupied_centers else np.empty((0, 2)))
        self.scatter_walls.set_offsets(np.asarray(walls[-600:]) if walls else np.empty((0, 2)))
        self.scatter_obs.set_offsets(np.asarray(obstacles) if obstacles else np.empty((0, 2)))
        self.scatter_decisions.set_offsets(np.asarray(decisions) if decisions else np.empty((0, 2)))
        self.scatter_deadends.set_offsets(np.asarray(deadends) if deadends else np.empty((0, 2)))

        if rectangle:
            rx, ry = zip(*rectangle)
            self.line_rect.set_data(rx, ry)
        else:
            self.line_rect.set_data([], [])

        all_points = []
        if path:
            all_points.extend(path)
        if occupied_centers:
            all_points.extend(occupied_centers[-900:])
        if free_centers:
            all_points.extend(free_centers[-500:])
        if visited_centers:
            all_points.extend(visited_centers[-600:])
        if walls:
            all_points.extend(walls[-600:])
        if obstacles:
            all_points.extend(obstacles[-300:])
        if decisions:
            all_points.extend(decisions[-120:])
        if deadends:
            all_points.extend(deadends[-80:])

        if all_points:
            arr = np.asarray(all_points, dtype=float)
            min_x, min_y = np.min(arr[:, 0]), np.min(arr[:, 1])
            max_x, max_y = np.max(arr[:, 0]), np.max(arr[:, 1])
            pad = max(0.35, 0.12 * max(max_x - min_x, max_y - min_y, 1.0))
            self.ax_mapa.set_xlim(min_x - pad, max_x + pad)
            self.ax_mapa.set_ylim(min_y - pad, max_y + pad)
            self.ax_mapa.set_aspect('equal', adjustable='box')

    def renderizar_telemetria(
        self,
        estado,
        distancias,
        velocidades,
        bateria,
        odom,
        control,
        estadisticas,
        total_puntos,
        modo_velocidad="MEDIO",
        factor_angular=1.0,
        bateria_fuente="SIN DATOS",
        paredes=None,
        mapa_resumen=None,
        percepcion=None,
    ):
        self.ax_text.clear()
        self.ax_text.axis('off')

        estado_upper = estado.upper()
        if "ESPERANDO" in estado_upper:
            color_badge = '#607d8b'
        elif "PAUSA" in estado_upper or "CRITICO" in estado_upper or "DETENIDO" in estado_upper:
            color_badge = '#d32f2f'
        elif "RODEO" in estado_upper or "FRENTE" in estado_upper:
            color_badge = '#ff8f00'
        else:
            color_badge = '#00897b'

        def fmt(v):
            if v is None or not math.isfinite(v):
                return "---"
            return f"{v:.2f} m"

        def fmt_bat(bat):
            volt = bat[0] if len(bat) > 0 else float('nan')
            pct = bat[1] if len(bat) > 1 else None
            fuente = bat[2] if len(bat) > 2 else bateria_fuente
            if pct is None:
                return "🔋 Batería: sin datos [/battery]", '#aaaaaa'
            try:
                pct_i = int(pct)
            except Exception:
                return "🔋 Batería: sin datos [/battery]", '#aaaaaa'
            color = '#00ff00' if pct_i > 40 else '#ff3300'
            if volt is None or not math.isfinite(volt):
                return f"🔋 Batería: {pct_i}% [{fuente}]", color
            return f"🔋 Batería: {volt:.1f} V ({pct_i}%) [{fuente}]", color

        x, y, yaw = odom
        kp_cent, kp_ang, target, error_centro, error_ang, ancho_pasillo = control
        t_ms, t_prom, t_max, hz = estadisticas
        paredes = paredes or [False, False, 0, 0]
        pared_der_ok, pared_izq_ok, pts_der, pts_izq = paredes
        mapa_resumen = mapa_resumen or {}
        percepcion = percepcion or ["INDEFINIDO", "NONE", 0.0, 0]

        self.ax_text.text(
            0.03, 0.95, f"  ESTADO: {estado}  ", color='white', weight='bold', fontsize=11,
            bbox=dict(facecolor=color_badge, alpha=0.95, boxstyle='round,pad=0.45')
        )

        self.ax_text.text(0.03, 0.85, "📡 DISTANCIAS LiDAR", color='#9aa0aa', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.795, f"Frente      : {fmt(distancias[0])}", color='#ff4d4d', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.745, f"Derecha real: {fmt(distancias[2])}", color='#00ff66', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.695, f"Izquierda   : {fmt(distancias[1])}", color='#00e5ff', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.645, f"Ancho pas.  : {fmt(ancho_pasillo)}", color='#eeeeee', fontsize=10)
        self.ax_text.text(0.03, 0.595, f"Pared DER/IZQ: {pared_der_ok}/{pared_izq_ok}", color='#eeeeee', fontsize=9)
        self.ax_text.text(0.03, 0.550, f"Pts DER/IZQ  : {pts_der}/{pts_izq}", color='#eeeeee', fontsize=9)
        tipo_geo, clase_arco, ancho_arco, cajas = percepcion
        self.ax_text.text(0.03, 0.510, f"Frente geo  : {tipo_geo}", color='#eeeeee', fontsize=9)
        self.ax_text.text(0.03, 0.475, f"Arco front. : {clase_arco} / {math.degrees(ancho_arco):.0f}°", color='#eeeeee', fontsize=9)
        self.ax_text.text(0.55, 0.550, f"Cajas       : {cajas}", color='#ff9800', fontsize=10, weight='bold')

        self.ax_text.text(0.03, 0.420, "🕹️ CONTROL", color='#9aa0aa', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.375, f"Modo lineal : {modo_velocidad}", color='#ffcc00', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.330, f"Factor giro : {factor_angular:.2f}x", color='#ffcc00', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.285, f"Vel lineal  : {velocidades[0]:+.3f} m/s", color='#ffcc00', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.240, f"Vel angular : {velocidades[1]:+.3f} rad/s", color='#ffcc00', fontsize=10, weight='bold')
        self.ax_text.text(0.03, 0.195, f"Kp centro   : {kp_cent:.2f}", color='#eeeeee', fontsize=10)
        self.ax_text.text(0.03, 0.150, f"Kp ángulo   : {kp_ang:.2f}", color='#eeeeee', fontsize=10)
        self.ax_text.text(0.03, 0.105, f"Error centro: {error_centro:+.3f} m", color='#eeeeee', fontsize=10)
        self.ax_text.text(0.03, 0.080, f"Error pared : {error_ang:+.3f}", color='#eeeeee', fontsize=10)

        self.ax_text.text(0.55, 0.465, "📍 ODOMETRÍA", color='#9aa0aa', fontsize=10, weight='bold')
        self.ax_text.text(0.55, 0.420, f"x   : {x:+.3f} m", color='#dddddd', fontsize=10)
        self.ax_text.text(0.55, 0.375, f"y   : {y:+.3f} m", color='#dddddd', fontsize=10)
        self.ax_text.text(0.55, 0.330, f"yaw : {math.degrees(yaw):+.1f}°", color='#dddddd', fontsize=10)

        self.ax_text.text(0.55, 0.260, "📊 MÉTRICAS RETO", color='#9aa0aa', fontsize=10, weight='bold')
        metricas = mapa_resumen.get('metricas_reto', {}) or {}
        tiempo_total = float(metricas.get('tiempo_total', 0.0) or 0.0)
        dist_total = float(metricas.get('distancia_recorrida', mapa_resumen.get('distance', 0.0)) or 0.0)
        pares_det = int(metricas.get('pares_detectados', 0) or 0)
        pares_resp = int(metricas.get('pares_respetados', 0) or 0)
        colisiones = int(metricas.get('colisiones', 0) or 0)
        llego_meta = bool(metricas.get('llego_meta', False))
        self.ax_text.text(0.55, 0.215, f"Tiempo      : {tiempo_total:.1f} s", color='#dddddd', fontsize=10)
        self.ax_text.text(0.55, 0.170, f"Distancia   : {dist_total:.2f} m", color='#dddddd', fontsize=10)
        self.ax_text.text(0.55, 0.125, f"PARE det/resp: {pares_det}/{pares_resp}", color='#dddddd', fontsize=10)
        col_color = '#00e676' if colisiones == 0 else '#ff5252'
        self.ax_text.text(0.55, 0.080, f"Colisiones  : {colisiones}", color=col_color, fontsize=10, weight='bold')
        meta_color = '#00e676' if llego_meta else '#dddddd'
        self.ax_text.text(0.55, 0.045, f"Llegó meta  : {'SÍ' if llego_meta else 'NO'}", color=meta_color, fontsize=10, weight='bold')

        texto_bat, color_bat = fmt_bat(bateria)
        self.ax_text.text(0.03, 0.045, texto_bat, color=color_bat, fontsize=10, weight='bold')
