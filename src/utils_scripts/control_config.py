#!/usr/bin/env python3
"""
control_config.py
-----------------
Parámetros concentrados para tunear el robot sin tocar la lógica principal.
El robot ahora prioriza conducir por el centro del pasillo calculando la distancia
entre pared derecha e izquierda. La pared derecha/izquierda se valida por líneas
conectadas para no confundir la pared frontal con una pared lateral. Además se
integra mapeo simple del recorrido y anti-retorno suave.
"""
from dataclasses import dataclass


@dataclass
class ControlConfig:
    # Conducción por centro del pasillo
    distancia_derecha_objetivo: float = 0.25   # m, fallback cuando solo se ve derecha
    distancia_izquierda_objetivo: float = 0.25 # m, fallback cuando solo se ve izquierda
    tolerancia_distancia: float = 0.040        # m
    tolerancia_angulo: float = 0.11            # pendiente aprox. de pared para considerar paralelo
    ciclos_estable_necesarios: int = 6

    # Ganancias del controlador
    kp_centrado: float = 1.05                  # corrige hacia el medio entre izquierda y derecha
    kd_centrado: float = 0.18                  # reduce zigzag al centrar
    kp_distancia: float = 1.25                 # fallback lateral
    kd_distancia: float = 0.20                 # fallback lateral
    kp_angulo: float = 0.82                    # corrección suave por orientación de paredes
    k_alpha_guardian: float = 1.05              # término heading tipo CapyGuardian (paralelismo)
    persist_frames: int = 4                     # frames seguidos antes de cambiar a esquina/caja

    # Carril central / Follow-the-Gap.
    # En espacios amplios NO sigue la pared exterior; busca el hueco libre frontal
    # y avanza por el centro para tomar una ruta más directa tipo línea azul.
    usar_carril_central: bool = True
    ancho_pasillo_estrecho_max: float = 0.48    # m; debajo de esto sí centra entre paredes
    carril_frente_min: float = 0.28             # m; mínimo para activar hueco libre
    carril_frente_recto: float = 0.95           # m; si el frente está libre, sesga a ir recto
    carril_sector_deg: float = 105.0             # sector frontal evaluado por el gap
    carril_bins: int = 91                       # resolución angular del gap
    carril_clearance_min: float = 0.38          # m; bin considerado libre
    carril_gap_min_deg: float = 10.0            # descarta huecos muy finos
    carril_max_range: float = 2.20              # m; rango útil del gap
    carril_bias_frente: float = 0.25            # mayor = prefiere ir recto
    carril_factor_target_con_frente_libre: float = 1.0
    kp_carril_gap: float = 1.15                 # control hacia centro del hueco
    kd_carril_gap: float = 0.04
    max_w_carril_central: float = 0.38          # rad/s, evita trompos
    vel_carril_central: float = 0.105           # m/s, avance en zona abierta
    w_lateral_seguridad: float = 0.20           # empuje suave si roza pared lateral



    # Modo recomendado actual: LiDAR manda la navegación.
    # La odometría/IMU queda solo para plot/reporte y para estabilizaciones locales,
    # pero NO corrige la ruta con anti-retorno ni obliga a mantener un rumbo viejo.
    usar_odom_para_navegacion: bool = False
    usar_antiretorno_mapeo: bool = False

    # Follow-the-Gap más fuerte para curvas: si el frente se empieza a cerrar,
    # el robot sigue el hueco más libre del LiDAR y no pelea contra un yaw previo.
    gap_prioridad_lidar: bool = True
    gap_frente_curva: float = 0.92              # m; debajo de esto deja de sesgar a recto
    gap_diag_diferencia_curva: float = 0.14     # m; diferencia diagonal para entrar a curva
    gap_target_max_deg: float = 58.0            # límite del objetivo angular del hueco
    gap_suavizado_target: float = 0.55          # 0=no suaviza, 1=mantiene mucho target previo
    gap_vel_min_curva: float = 0.045            # velocidad mínima segura en curvas con gap




    # Modo laberinto / micromouse simple.
    # Diferencia clave vs DFS libre: el robot decide en intersecciones,
    # bloquea la decisión y sigue pared izquierda con distancia segura.
    usar_modo_laberinto: bool = True
    lab_preferir_pared: str = "izquierda"
    lab_dist_pared: float = 0.28
    lab_frente_giro: float = 0.40
    lab_frente_giro_rapido: float = 0.30  # acción inmediata: no quedarse pensando cerca de pared/PARE
    lab_frente_salida: float = 0.52
    lab_vel_avance: float = 0.160
    lab_vel_lenta: float = 0.085
    lab_vel_retroceso: float = -0.045
    lab_backoff_seg: float = 0.14
    lab_w_giro: float = 0.48
    lab_w_max: float = 0.70
    lab_kp_pared: float = 1.28
    lab_kd_pared: float = 0.08
    lab_kp_alpha: float = 0.52
    lab_kp_gap: float = 0.68
    lab_turn_min_seg: float = 0.20
    lab_turn_max_seg: float = 0.78
    lab_decision_lock_seg: float = 0.55
    lab_salida_giro_seg: float = 0.24
    lab_lateral_seguro: float = 0.16
    lab_izq_libre_para_girar: float = 0.38
    lab_der_libre_para_girar: float = 0.38
    lab_default_turn: float = 1.0  # +1 izquierda, -1 derecha
    lab_mapa_lidar_360: bool = True
    lab_usar_cmd_vel_mapa: bool = True
    lab_marcar_intersecciones: bool = True
    lab_bloquear_hasta_avanzar_m: float = 0.14

    # Capybot K: escape de atasco + exploración por fronteras locales.
    # Si gira o se queda casi sin avanzar, ejecuta una maniobra cerrada
    # (parar -> retroceder -> girar -> avanzar) antes de volver a decidir.
    lab_usar_escape_atorado: bool = True
    lab_stuck_timeout_seg: float = 1.05
    lab_giro_stuck_seg: float = 0.95
    lab_progress_min_m: float = 0.10
    lab_escape_stop_seg: float = 0.05
    lab_escape_backoff_seg: float = 0.20
    lab_escape_turn_seg: float = 0.48
    lab_escape_forward_seg: float = 0.30
    lab_escape_w: float = 0.50

    # Menos falsos eventos: no marcar intersección/callejón a cada frame.
    lab_evento_min_seg: float = 1.05
    lab_evento_min_dist: float = 0.22

    # Puntuación de caminos: espacio + zona desconocida/libre - zona visitada.
    lab_score_espacio: float = 0.85
    lab_score_prefer_frente: float = 0.34
    lab_score_prefer_izq: float = 0.20
    lab_score_prefer_der: float = -0.03
    lab_frontier_unknown_bonus: float = 0.20
    lab_frontier_free_bonus: float = 0.07
    lab_frontier_visit_penalty: float = 0.18
    lab_frontier_occ_penalty: float = 0.70

    # Modo aspiradora + mapa simple.
    # Idea: avanzar por zonas libres, seguir pared con distancia segura,
    # escapar si el frente se cierra, y mapear paredes/ruta con LiDAR 360°.
    usar_modo_aspiradora: bool = True
    aspiradora_preferir_pared: str = "izquierda"   # referencia principal luego de girar
    aspiradora_dist_pared: float = 0.28             # distancia prudente a pared seguida
    aspiradora_frente_giro: float = 0.43            # si frente baja de esto, decide giro
    aspiradora_frente_salida: float = 0.52          # frente libre para salir de giro
    aspiradora_vel_avance: float = 0.115
    aspiradora_vel_lenta: float = 0.055
    aspiradora_vel_escape: float = 0.000            # gira sin avanzar cuando está cerca
    aspiradora_w_giro: float = 0.34
    aspiradora_w_max: float = 0.48
    aspiradora_kp_pared: float = 1.05
    aspiradora_kd_pared: float = 0.08
    aspiradora_kp_alpha: float = 0.45
    aspiradora_kp_gap: float = 0.72
    aspiradora_turn_min_seg: float = 0.55           # mantiene giro elegido, evita dudar
    aspiradora_turn_max_seg: float = 1.75
    aspiradora_decision_lock_seg: float = 0.82
    aspiradora_lateral_seguro: float = 0.18
    aspiradora_usar_mapa_cmd_vel: bool = True
    aspiradora_mapa_lidar_360: bool = True
    aspiradora_backoff_seg: float = 0.22            # mini retroceso en atasco fuerte
    aspiradora_vel_retroceso: float = -0.025
    aspiradora_default_turn: float = 1.0            # +1 izquierda, -1 derecha

    # Velocidades base. La interfaz aplica factor lento/medio/rápido.
    vel_crucero: float = 0.120                 # m/s
    vel_acercamiento: float = 0.072            # m/s
    vel_lenta: float = 0.050                   # m/s
    vel_rodeo: float = 0.085                   # m/s
    vel_giro: float = 0.42                     # rad/s, giro general más suave

    # Esquinas del circuito en sentido ANTIHORARIO:
    # ROS usa angular.z positivo para girar a la izquierda.
    sentido_circuito_antihorario: bool = True
    sentido_giro_esquina: float = 1.0          # +1 izquierda, -1 derecha
    vel_giro_esquina: float = 0.20             # rad/s, giro lento/controlado para no pasarse
    vel_avance_esquina: float = 0.000          # m/s, avance mínimo mientras gira y sigue sensando
    frente_min_avance_giro: float = 0.34       # m, si hay menos se gira sin avanzar
    frente_salida_con_lateral: float = 0.30    # m, sale si reaparece pared lateral
    frente_salida_libre: float = 0.38          # m, frente suficiente para volver a avanzar
    max_tiempo_giro_frente: float = 4.20       # s, respaldo si no hay odometría
    lateral_reenganche_max: float = 1.15       # m, pared lateral útil para reincorporarse

    # Análisis frontal: diferencia entre PASADIZO y ESQUINA.
    frente_pasillo_libre: float = 0.46         # m, si está libre y hay paredes a ambos lados: pasadizo
    frente_esquina_detect: float = 0.68        # m, empieza a analizar pared frontal conectada
    frente_caja_detect: float = 0.43            # m, arco frontal corto = caja/obstáculo
    cooldown_caja: float = 2.00                 # s, evita contar/iniciar la misma caja varias veces
    distancia_detencion_esquina: float = 0.36  # m, se acerca hasta aquí antes de girar
    vel_acercar_esquina: float = 0.028         # m/s, acercamiento lento antes del giro
    yaw_objetivo_giro_esquina_deg: float = 64.0 # grados; sale temprano para alinear, no para completar 90° de golpe
    yaw_max_giro_esquina_deg: float = 78.0     # grados; nunca permite que una esquina se vuelva vuelta completa
    # Ajustes por esquina del circuito. La segunda suele estar más cerca, por eso
    # se gira menos y con más cuidado para no perder orientación.
    yaw_objetivo_esquinas_deg: tuple = (64.0, 56.0, 62.0, 38.0)
    yaw_max_esquinas_deg: tuple = (78.0, 70.0, 76.0, 48.0)
    max_tiempo_giro_esquinas: tuple = (3.60, 2.70, 3.30, 1.45)
    factor_giro_esquinas: tuple = (1.00, 0.82, 0.92, 0.42)
    tolerancia_paralelo_post_giro: float = 0.18 # pendiente aprox. para considerar paralelo
    max_tiempo_alinear_post_giro: float = 0.65 # s, corrección corta después del giro; bajo para no sobregirar en esquina 4
    cooldown_esquina: float = 1.10             # s, evita reentrar a giro y acumular vueltas
    avance_post_esquina_min_m: float = 0.16    # m, avance mínimo antes de volver a detectar esquina
    avance_post_esquina_min_seg: float = 0.85  # s, bloqueo corto post-giro para la segunda esquina
    avance_post_esquina_max_seg: float = 2.00  # s, no se queda avanzando si el tramo es corto
    vel_avance_post_esquina: float = 0.055     # m/s, avance lento de estabilización

    # Salida protegida especial para la 4ta esquina: reduce reentrada y contravolantea.
    avance_post_esquina4_min_m: float = 0.28   # m, mínimo de salida en esquina 4
    avance_post_esquina4_min_seg: float = 1.35 # s, mínimo de salida sin reanalizar esquina
    avance_post_esquina4_max_seg: float = 2.60 # s, respaldo si odometría no mide bien
    kp_yaw_salida_post_esquina: float = 1.15   # mantiene rumbo al salir de la esquina
    max_w_salida_post_esquina: float = 0.13    # rad/s, corrección muy suave
    contravolante_salida_esquina: float = 0.055 # rad/s, evita seguir girando contra la pared

    bloqueo_reentrada_esquina_m: float = 0.25  # m, ignora esquina hasta separarse un poco
    bloqueo_reentrada_esquina_seg: float = 1.65 # s, respaldo por tiempo si no hay odometría

    # Bloqueo de referencia lateral en todas las esquinas.
    # Evita que el robot pierda derecha y use izquierda como guía falsa antes
    # de una esquina, que era lo que causaba la vuelta en la 4ta esquina.
    bloqueo_referencia_esquinas: bool = True
    bloqueo_referencia_pre_esquina: bool = True
    referencia_preferida_esquina: str = "derecha"
    frente_zona_bloqueo_referencia: float = 0.72
    ref_memoria_lateral_seg: float = 1.40
    frames_cambio_referencia: int = 5
    avance_recto_ref_bloqueada: float = 0.032
    max_tiempo_bloqueo_referencia: float = 4.00

    max_angular: float = 0.56                  # rad/s antes del factor angular
    max_lineal_segura: float = 0.22            # límite final de seguridad
    max_angular_segura: float = 1.10           # límite final de seguridad

    # Selector de 3 velocidades desde GUI
    modo_velocidad_inicial: int = 2            # 1=lento, 2=medio, 3=rápido
    factor_lento: float = 0.72
    factor_medio: float = 1.20
    factor_rapido: float = 1.55

    # Control independiente de velocidad angular desde GUI
    factor_angular_inicial: float = 1.08
    factor_angular_min: float = 0.35
    factor_angular_max: float = 1.60
    factor_angular_paso: float = 0.10

    # Batería: indicador leído desde /battery. Sin botones manuales.
    bateria_voltaje_min: float = 10.5
    bateria_voltaje_max: float = 12.6

    # Umbrales de seguridad
    frente_alerta: float = 0.46                # m, reduce velocidad si algo aparece al frente
    frente_critico: float = 0.30               # m, gira para no chocar
    derecha_muy_cerca: float = 0.13            # m, evita rozar pared derecha
    izquierda_muy_cerca: float = 0.13          # m, evita rozar pared izquierda

    # Anti-choque global: capa superior que corrige cualquier comando antes
    # de publicarlo. No importa si el estado está en carril central, esquina,
    # rodeo o centrado: si el LiDAR ve peligro, se prohíbe avanzar.
    usar_antichoque_global: bool = True
    frente_freno_suave: float = 0.43           # m, empieza a limitar velocidad
    frente_stop_global: float = 0.28           # m, prohibido avanzar hacia adelante
    frente_emergencia: float = 0.20            # m, giro de escape sin avance
    lateral_freno_suave: float = 0.23          # m, empieza a alejarse del lado cercano
    lateral_stop_global: float = 0.14          # m, pared lateral demasiado cerca
    diagonal_stop_global: float = 0.22         # m, esquinas/diagonales demasiado cerca
    w_escape_frontal: float = 0.28             # rad/s, giro al lado más libre
    w_escape_lateral: float = 0.22             # rad/s, empuje lejos de pared lateral
    vel_max_freno_suave: float = 0.078         # m/s, máximo si el frente empieza a cerrarse
    vel_max_lateral_cerca: float = 0.060       # m/s, máximo si roza pared lateral
    lateral_perdida: float = 1.50              # m, pared lateral demasiado lejos/no confiable
    vel_busqueda_derecha: float = 0.00
    giro_busqueda_derecha: float = 0.24        # giro suave hacia derecha si no hay paredes

    # Tiempos de rodeo para secuencia solicitada: IZQ -> DER -> DER -> IZQ
    # Bajados para que no gire demasiado.
    rodeo_giro_izq_seg: float = 0.52
    rodeo_avance_1_seg: float = 0.45
    rodeo_giro_der_1_seg: float = 0.44
    rodeo_avance_2_seg: float = 0.82
    rodeo_giro_der_2_seg: float = 0.44
    rodeo_avance_3_seg: float = 0.48
    rodeo_giro_izq_final_seg: float = 0.44

    # Memoria de recorrido / anti-retorno
    factor_velocidad_antiretorno: float = 0.72  # reduce avance si detecta que vuelve sobre sus pasos

    # Si el robot gira al revés, cambia a -1.0
    signo_giro: float = 1.0
    # Reporte de mapa: convertir nube LiDAR en plano tipo laberinto.
    mapa_min_run_pared: int = 4
    mapa_mostrar_puntos_crudos: bool = False

    # Capybot N: mapeo limpio tipo laberinto.
    # El mapa ya NO guarda paredes durante giros/escape; solo mientras avanza.
    # Esto evita el dibujo circular y deja paredes rectas más parecidas a un plano.
    mapa_solo_lidar_en_avance: bool = True
    mapa_snap_yaw_cmd_vel: bool = True
    mapa_snap_yaw_deg: float = 90.0
    mapa_w_max_para_guardar: float = 0.16
    mapa_v_min_para_guardar: float = 0.018
    mapa_max_range_laberinto: float = 1.45
    mapa_grid_res_laberinto: float = 0.10
    mapa_lidar_stride: int = 4
    mapa_occ_hit_threshold: int = 2


# Parámetros agregados para capybot_o: detector de PARE por cámara.
# Se asignan como atributos de la clase para no alterar la lógica existente.
ControlConfig.usar_camara_pare = True
ControlConfig.camara_indice = 0
ControlConfig.camara_width = 480
ControlConfig.camara_height = 360
ControlConfig.camara_fps = 20

# HSV rojo: dos rangos porque el rojo cruza el 0° en HSV.
ControlConfig.pare_h1_min = 0
ControlConfig.pare_h1_max = 8
ControlConfig.pare_h2_min = 172
ControlConfig.pare_h2_max = 179
ControlConfig.pare_s_min = 145
ControlConfig.pare_v_min = 85

# Zona de atención de la cámara para reducir falsos positivos.
ControlConfig.pare_roi_x0 = 0.08
ControlConfig.pare_roi_x1 = 0.92
ControlConfig.pare_roi_y0 = 0.05
ControlConfig.pare_roi_y1 = 0.88

# Validación de contorno rojo.
ControlConfig.pare_area_min_px = 160
ControlConfig.pare_area_ratio_min = 0.0030
ControlConfig.pare_area_ratio_fuerte = 0.009
# Evita falsos positivos grandes como brazo/camiseta/objetos rojos muy extensos.
ControlConfig.pare_area_ratio_max = 0.045
ControlConfig.pare_ratio_min = 0.70
ControlConfig.pare_ratio_max = 1.45
ControlConfig.pare_frames_confirmacion = 2
ControlConfig.pare_frames_perdida = 3
ControlConfig.pare_procesar_cada_n_frames = 1

# Regla del reto: PARE detectado => detenerse ~3 s antes de continuar.
ControlConfig.pare_stop_seg = 3.0
ControlConfig.pare_cooldown_seg = 5.0


# capybot_u: PARE en marcha, sin prealerta ni pausas extra.
# La cámara detecta en hilo independiente cada frame y el stop se aplica apenas confirma.

# capybot_v: vista de cámara normal en interfaz.
# Forzar MJPG/convert RGB evita frames raros tipo máscara o colores corruptos.
ControlConfig.camara_fourcc = "MJPG"
ControlConfig.camara_swap_rb = False

# capybot_x: aviso pasivo de META conocida.
# No cambia la navegación. Solo avisa en la cámara/interfaz cuando la pose
# estimada se acerca a la esquina superior derecha oficial del reto.
ControlConfig.meta_avisar_en_frame = True
ControlConfig.meta_mostrar_distancia = True
ControlConfig.meta_x_m = 3.60
ControlConfig.meta_y_m = 2.40
ControlConfig.meta_radio_m = 0.48
ControlConfig.meta_min_recorrido_m = 1.20

# capybot_y: detector de META verde por cámara.
# La cámara confirma el cartel verde, alinea por el centro del frame,
# avanza un poco para entrar al área de meta y luego detiene el robot.
ControlConfig.usar_camara_meta_verde = True
ControlConfig.meta_h_min = 35
ControlConfig.meta_h_max = 92
ControlConfig.meta_s_min = 65
ControlConfig.meta_v_min = 65
ControlConfig.meta_roi_x0 = 0.04
ControlConfig.meta_roi_x1 = 0.96
ControlConfig.meta_roi_y0 = 0.02
ControlConfig.meta_roi_y1 = 0.78
ControlConfig.meta_area_min_px = 240
ControlConfig.meta_area_ratio_min = 0.0030
ControlConfig.meta_area_ratio_max = 0.22
ControlConfig.meta_ratio_min = 1.05
ControlConfig.meta_ratio_max = 5.50
ControlConfig.meta_frames_confirmacion = 2
ControlConfig.meta_frames_perdida = 5
ControlConfig.meta_avance_seg = 5.30
ControlConfig.meta_vel_avance = 0.085
ControlConfig.meta_kp_vision = 0.42
ControlConfig.meta_w_max = 0.28
ControlConfig.meta_frente_stop = 0.22

# capybot_ac: modo experimental para USAR MAPA APRENDIDO.
# Se activa por botón; el modo estable de exploración no se modifica.
ControlConfig.mapa_wp_min_dist = 0.30
ControlConfig.mapa_wp_radio = 0.22
ControlConfig.mapa_vel_avance = 0.105
ControlConfig.mapa_vel_lenta = 0.070
ControlConfig.mapa_vel_giro = 0.035
ControlConfig.mapa_vel_llegada = 0.045
ControlConfig.mapa_kp_yaw = 1.18
ControlConfig.mapa_w_max = 0.46
ControlConfig.mapa_ang_giro_lento = 0.70
ControlConfig.mapa_frente_stop = 0.28

# Ajustes AF: modo mapa aprendido seguro
try:
    ControlConfig.mapa_stuck_seg = 3.2
    ControlConfig.mapa_recovery_seg = 2.8
    ControlConfig.mapa_front_block_seg = 1.25
    ControlConfig.mapa_wp_radio = 0.24
    ControlConfig.mapa_wp_min_dist = 0.30
except NameError:
    pass

# capybot_ag: usar el mapa como memoria de decisiones, no como coordenadas exactas.
# CARGAR MAPA extrae la secuencia de giros/intersecciones que funcionó.
# USAR MAPA mantiene la navegación estable, pero intenta repetir esas decisiones si el LiDAR confirma que son seguras.
try:
    ControlConfig.mapa_decision_min_seg = 0.75
    ControlConfig.mapa_decision_lookahead = 4
    ControlConfig.mapa_interseccion_lateral_min = 0.58
except NameError:
    pass
