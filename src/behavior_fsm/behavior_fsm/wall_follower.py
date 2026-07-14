#!/usr/bin/env python3
import time

class WallFollowerController:
    def __init__(self):
        # 1. Parámetros del Controlador PD (Gains)
        # Estos valores se pueden tunear luego en el archivo params.yaml
        self.kp = 2.5   # Ganancia Proporcional: reacciona al error actual
        self.kd = 1.2   # Ganancia Derivativa: frena las oscilaciones (efecto amortiguador)

        # 2. Configuración de la geometría del circuito
        # Ancho pasillo = 60cm. Mitad exacta (línea central) = 30cm = 0.30m
        self.distancia_deseada = 0.30 
        
        # 3. Memoria del controlador
        self.ultimo_error = 0.0
        self.ultima_vez = time.time()

    def calcular_giro(self, distancia_actual_lateral):
        """
        Calcula la velocidad angular (en rad/s) usando las matemáticas del PD.
        Si seguimos la pared izquierda:
        - Si el robot se aleja (distancia > 30cm), el error cambia y gira a la izquierda.
        - Si el robot se pega (distancia < 30cm), el error cambia y gira a la derecha.
        """
        tiempo_actual = time.time()
        dt = tiempo_actual - self.ultima_vez
        
        # Evitar división por cero en la primera iteración o ciclos ultrarrápidos
        if dt <= 0.0:
            dt = 0.01

        # Calcular error actual: Objetivo - Real
        error = self.distancia_deseada - distancia_actual_lateral

        # Componente Proporcional
        P = self.kp * error

        # Componente Derivativa (tasa de cambio del error)
        derivada = (error - self.ultimo_error) / dt
        D = self.kd * derivada

        # Señal de control total (Velocidad Angular en Z)
        # Nota: El signo (+ o -) dependerá de si sigues la pared izquierda o derecha.
        # Para seguir pared IZQUIERDA: un error positivo (muy cerca) requiere girar a la derecha (-w)
        accion_angular = -(P + D)

        # Guardar estado para la próxima iteración
        self.ultimo_error = error
        self.ultima_vez = tiempo_actual

        # Limitador de seguridad (saturación) para evitar que el robot gire como loco (máx 1.5 rad/s)
        if accion_angular > 1.5: accion_angular = 1.5
        elif accion_angular < -1.5: accion_angular = -1.5

        return accion_angular

    def ajustar_velocidad_lineal(self, distancia_frente, velocidad_base=0.2):
        """
        Regula la velocidad hacia adelante. Si ve una pared o esquina al frente,
        frena automáticamente para darle espacio al control angular de rotar seguro.
        """
        if distancia_frente < 0.50:
            # Esquina o colisión inminente: frena progresivamente
            return velocidad_base * (distancia_frente / 0.50)
        return velocidad_base

    # =========================
    # MEJORAS: CRUCERO + ESQUINA SUAVE + RODEAR
    # =========================

    def detectar_crucero(self, lidar):
        # espacio libre frente + lados (pasillo limpio)
        return lidar.frente > 1.2 and abs(lidar.izquierda - lidar.derecha) < 0.15

    def detectar_esquina(self, lidar):
        # esquina si frente bajo y diferencia izquierda/derecha alta
        return lidar.frente < 0.8 and (lidar.derecha < 0.6 or lidar.izquierda < 0.6)

    def control_esquina_suave(self, error, alpha_theta, dt):
        # kp*error + kd*(alpha-theta)/dt
        return self.kp * error + self.kd * (alpha_theta) / max(dt, 0.01)

    def detectar_caja_L(self, lidar):
        # gap entre frente y lateral sin cierre completo
        return (lidar.derecha_gap and not lidar.cierre_derecha) or (lidar.izquierda_gap and not lidar.cierre_izquierda)

    def modo_crucero(self):
        self.velocidad = self.v_base
        self.angular = 0.0

    def modo_rodear(self, lado):
        if lado == "derecha":
            self.angular = -0.6
        else:
            self.angular = 0.6
