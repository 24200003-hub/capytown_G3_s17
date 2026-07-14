# Capybot: Sistema Autónomo Híbrido de Navegación y Resolución Topológica de Laberintos

Este repositorio contiene la arquitectura de software de **Capybot**, un agente robótico móvil diseñado para la resolución autónoma de laberintos estructurados complejos mediante un enfoque híbrido reactivo-deliberativo. El sistema combina el control dinámico de lazo cerrado con fusión sensorial multimodal (telemetría LiDAR y visión artificial por computadora) y un modelo basado en memoria topológica para la toma de decisiones.

---

## 1. Arquitectura de Control Cinemático (Capa Reactiva)

El movimiento del agente se gestiona a través de una máquina de estados finitos (FSM) sintonizada de manera determinista para priorizar la estabilidad cinemática y la resolución sistemática de entornos restringidos (paradigma *Micromouse*).

### 1.1. Máquina de Estados de Navegación (`LAB_*`)
* **`LAB_AVANZAR`**: Vector de empuje longitudinal enfocado en la maximización del espacio libre frontal detectado por telemetría láser.
* **`LAB_SEGUIR_IZQUIERDA`**: Algoritmo de seguimiento de contornos. El sistema mantiene un *setpoint* de seguridad crítico de **0.28 metros** respecto a la pared lateral izquierda para garantizar una cobertura geométrica exhaustiva y prevenir ciclos de navegación infinitos.
* **`LAB_GIRAR` y `LAB_SALIDA_GIRO`**: Ante oclusiones frontales, el controlador interrumpe el avance longitudinal, evalúa el gradiente de densidad de obstáculos y ejecuta una rotación controlada hacia el hemisferio con mayor espacio libre, aplicando un temporizador de salida para suprimir oscilaciones cíclicas derecha/izquierda.
* **`LAB_ESCAPE_ATORADO`**: Mecanismo *watchdog* de seguridad. Si los actuadores experimentan un estancamiento prolongado debido a un falso frente o un callejón sin salida, el sistema ejecuta una retracción lineal inversa, reorienta el vector angular hacia la zona despejada y restablece el bucle principal de exploración.

### 1.2. Sintonización del Controlador Proporcional (Gain Tuning)
Los parámetros dinámicos del controlador proporcional (P) fueron optimizados con el fin de maximizar el rendimiento de tránsito y mitigar la latencia transitoria:
* **Velocidad Lineal de Avance (`lab_vel_avance`)**: Establecida en **0.160 m/s** para tramos rectos.
* **Velocidad Angular Máxima (`lab_w_max`)**: Acotada a **0.62 rad/s** (con picos de prueba de hasta **0.70 rad/s**) para asegurar estabilidad estructural en curvas cerradas.
* **Ganancia de Seguimiento (`lab_kp_pared`)**: Incrementada a **1.28** para suprimir el error de retraso en la corrección de orientación respecto a los límites físicos.
* **Ventana Computacional (`lab_decision_lock_seg`)**: Reducida a **0.82 segundos**, minimizando el tiempo muerto de procesamiento de datos antes de la ejecución de comandos de velocidad (`cmd_vel`).

---

## 2. Percepción Semántica y Visión Artificial

El robot incorpora una cámara RGB procesada matricialmente a través de la librería OpenCV, operando en paralelo al procesamiento asíncrono del escáner láser.

### 2.1. Detección de Señales Restrictivas (Señal PARE)
El flujo de visión realiza una transformación del espacio de color BGR a HSV para aislar la longitud de onda del color rojo bajo condiciones variables de iluminación.
* **Filtros Morfológicos y Geométricos**: Se configuran umbrales rigurosos de saturación (`pare_s_min = 145`), restricciones de relación de aspecto (*aspect ratio*) compatibles con señalizaciones octagonales y un límite de área máxima (`pare_area_ratio_max`) enfocado en descartar ruido de fondo o falsos positivos inducidos por el entorno de prueba.
* **Lógica de Ejecución**: La validación requiere persistencia temporal mínima (2 a 3 frames). Al confirmarse la señal, se activa el estado `PARAR_PARE`, aplicando un freno completo a los motores (`0.0 m/s`) durante **3.0 segundos**. Un temporizador de *cooldown* de **6.0 segundos** bloquea re-detecciones consecutivas del mismo objeto geométrico.

### 2.2. Reconocimiento y Alineación visomotora a Meta (Objetivo Verde)
La llegada al objetivo final se codifica semánticamente mediante un patrón cromático verde. Al ser capturado por la Región de Interés (ROI) de la cámara:
1.  **Lazo Cerrado de Orientación**: El robot abandona el seguimiento reactivo de paredes y calcula el error horizontal del centroide del objetivo respecto al centro óptico de la cámara, aplicando un control de dirección guiado por la ganancia `meta_kp_vision = 0.42`.
2.  **Aproximación Cinemática**: Una vez alineado el eje longitudinal del chasis, el sistema ejecuta un avance ciego temporizado parametrizado en función del entorno físico:
    * *Configuración Corta*: 0.085 m/s durante 2.95 s (Aproximadamente **25 cm**).
    * *Configuración Larga*: 0.085 m/s durante 5.30 s (Aproximadamente **45 cm**).
3.  **Localización Estocástica**: Como control pasivo complementario, el software computa la distancia euclidiana continua del agente hacia un vector de coordenadas absoluto predefinido para el objetivo en `(x: 3.60, y: 2.40)`.

---

## 3. Cartografía y Capa Deliberativa (Memoria de Decisiones)

La acumulación de error por deriva odométrica representa un desafío crítico al depender de la estimación matemática de velocidades en ausencia de codificadores (*encoders*) absolutos en las ruedas. El sistema resuelve esta limitación mediante abstracción topológica.

### 3.1. Cuantización Angular y Mapa de Ocupación
Para la construcción del mapa de ocupación bidimensional procesado mediante lecturas LiDAR de 360°, el software implementa un filtro de **cuantización del *yaw* (guiñada)**. La orientación estimada del robot se aproxima estrictamente a los cuatro ejes ortogonales puros del plano (**0°, 90°, 180°, 270°**). Esta discretización espacial permite limpiar y agrupar las nubes de puntos crudas del láser en proyecciones lineales perfectas (paredes rectas), eliminando las distorsiones circulares artificiales generadas durante las maniobras de giro. El rango de mapeo está estrictamente limitado al contorno inmediato del circuito, omitiendo ruidos externos.

### 3.2. Aprendizaje Pasivo y Modelo de Memoria Topológica
Durante la fase de exploración autónoma, el sistema ejecuta un hilo secundario asíncrono que exporta la estructura lógica del laberinto en un formato persistente estructurado dentro del directorio `/mapas_aprendidos/` (`mapa_laberinto.json`, `ruta_recorrida.csv`, `intersecciones.csv`).

Frente a la inestabilidad de algoritmos globales tradicionales como $A^*$ ejecutados sobre grillas cartesianas con errores de desplazamiento físico, la arquitectura adopta un enfoque basado en **Memoria de Decisiones**:
* Al activar el modo de mapa aprendido, el robot carga la secuencia lógica de acciones que resultaron exitosas en corridas previas (ej. *[Intersección 1: Girar Izquierda, Intersección 2: Avanzar Recto]*).
* **Prioridad de Seguridad Robusta**: Al arribar a un nodo topológico (intersección), el agente extrae la decisión memorizada, pero condiciona su ejecución a una auditoría en tiempo real realizada por el sensor LiDAR. Si el canal telemétrico indica que la dirección aprendida se encuentra bloqueada u obstruida accidentalmente, la memoria se degrada de forma segura y devuelve el control operativo a la lógica reactiva estable.

---

## 4. Telemetría e Interfaz Humano-Máquina (HMI)

El sistema integra un panel de control desarrollado bajo Matplotlib y expuesto remotamente mediante protocolos WayVNC, diseñado para la visualización del estado interno del sistema en tiempo real.

* **Subsistema de Video**: Muestra el flujo normalizado de la cámara (forzando codificación MJPG/RGB según hardware), superponiendo las cajas delimitadoras (*bounding boxes*) de los detectores de color HSV y alertas dinámicas de estado (`PARE: DETENIDO`, `META ALCANZADA`).
* **Separación de Canales Telemétricos**: Las variables críticas de control (errores de trayectoria, velocidad angular aplicada, lecturas de distancia frontal crítica) están desacopladas en bandas horizontales independientes para evitar solapamientos y facilitar la interpretación visual de la dinámica del lazo cerrado.
* **Compilación de Reporte de Desempeño**: Al pausar o finalizar el programa, el sistema exporta un informe estructurado que prioriza métricas analíticas de rendimiento del reto:
    1. Tiempo total de ejecución (Time of Flight).
    2. Distancia euclidiana e integrada recorrida.
    3. Tasa de efectividad de detención ante señalizaciones (PAREs detectados vs. respetados).
    4. Estimador pasivo de colisiones críticas calculado a partir de zonas de proximidad límite del LiDAR de 360°.
    5. Estado booleano de éxito en el arribo a meta.
