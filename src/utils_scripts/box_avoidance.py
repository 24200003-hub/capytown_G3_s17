#!/usr/bin/env python3
"""
box_avoidance.py
----------------
FSM modular para rodear una caja pegada a la pared.
La maniobra solicitada sigue esta secuencia de giros:
IZQUIERDA -> DERECHA -> DERECHA -> IZQUIERDA.

El módulo no publica en ROS directamente. Solo calcula velocidades base
(v, w) para que el nodo principal las publique y pueda aplicar el factor
de velocidad seleccionado en la interfaz.
"""
import math
import time


class BoxAvoidanceFSM:
    """Máquina de estados para rodear una caja con LiDAR frontal."""

    ESTADOS_RODEO = {
        "RODEO_GIRO_IZQ",
        "RODEO_AVANCE_1",
        "RODEO_GIRO_DER_1",
        "RODEO_AVANCE_2",
        "RODEO_GIRO_DER_2",
        "RODEO_AVANCE_3",
        "RODEO_GIRO_IZQ_FINAL",
    }

    def __init__(self, cfg, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.t_fin_estado = time.time()

    def es_estado_rodeo(self, estado):
        return estado in self.ESTADOS_RODEO

    def iniciar(self, cambiar_estado):
        """Inicia el rodeo por la izquierda."""
        cambiar_estado("RODEO_GIRO_IZQ")
        self.t_fin_estado = time.time() + self.cfg.rodeo_giro_izq_seg
        if self.logger:
            self.logger.warn("Caja/obstáculo detectado: iniciando rodeo L -> R -> R -> L.")

    def _distancia_segura(self, dist_frente):
        return (not math.isfinite(dist_frente)) or dist_frente > self.cfg.frente_critico

    def actualizar(self, estado_actual, dist_frente, cambiar_estado):
        """
        Devuelve (velocidad_lineal, velocidad_angular) según el estado actual.
        La estructura física de la maniobra es:
        1. Giro izquierda
        2. Avance corto
        3. Giro derecha
        4. Avance bordeando caja
        5. Giro derecha
        6. Avance de reincorporación
        7. Giro izquierda final
        """
        ahora = time.time()
        sg = self.cfg.signo_giro

        if estado_actual == "RODEO_GIRO_IZQ":
            if ahora < self.t_fin_estado or not self._distancia_segura(dist_frente):
                return 0.0, sg * self.cfg.vel_giro
            cambiar_estado("RODEO_AVANCE_1")
            self.t_fin_estado = ahora + self.cfg.rodeo_avance_1_seg
            return self.cfg.vel_rodeo, 0.0

        if estado_actual == "RODEO_AVANCE_1":
            if not self._distancia_segura(dist_frente):
                return 0.0, sg * self.cfg.vel_giro
            if ahora < self.t_fin_estado:
                return self.cfg.vel_rodeo, 0.0
            cambiar_estado("RODEO_GIRO_DER_1")
            self.t_fin_estado = ahora + self.cfg.rodeo_giro_der_1_seg
            return 0.0, -sg * self.cfg.vel_giro

        if estado_actual == "RODEO_GIRO_DER_1":
            if ahora < self.t_fin_estado:
                return 0.0, -sg * self.cfg.vel_giro
            cambiar_estado("RODEO_AVANCE_2")
            self.t_fin_estado = ahora + self.cfg.rodeo_avance_2_seg
            return self.cfg.vel_rodeo, 0.0

        if estado_actual == "RODEO_AVANCE_2":
            if not self._distancia_segura(dist_frente):
                return 0.0, sg * self.cfg.vel_giro
            if ahora < self.t_fin_estado:
                return self.cfg.vel_rodeo, 0.0
            cambiar_estado("RODEO_GIRO_DER_2")
            self.t_fin_estado = ahora + self.cfg.rodeo_giro_der_2_seg
            return 0.0, -sg * self.cfg.vel_giro

        if estado_actual == "RODEO_GIRO_DER_2":
            if ahora < self.t_fin_estado:
                return 0.0, -sg * self.cfg.vel_giro
            cambiar_estado("RODEO_AVANCE_3")
            self.t_fin_estado = ahora + self.cfg.rodeo_avance_3_seg
            return self.cfg.vel_rodeo, 0.0

        if estado_actual == "RODEO_AVANCE_3":
            if not self._distancia_segura(dist_frente):
                return 0.0, sg * self.cfg.vel_giro
            if ahora < self.t_fin_estado:
                return self.cfg.vel_rodeo, 0.0
            cambiar_estado("RODEO_GIRO_IZQ_FINAL")
            self.t_fin_estado = ahora + self.cfg.rodeo_giro_izq_final_seg
            return 0.0, sg * self.cfg.vel_giro

        if estado_actual == "RODEO_GIRO_IZQ_FINAL":
            if ahora < self.t_fin_estado:
                return 0.0, sg * self.cfg.vel_giro
            cambiar_estado("RECUPERAR_PARED")
            return self.cfg.vel_lenta, -sg * 0.25

        cambiar_estado("RECUPERAR_PARED")
        return 0.0, 0.0
