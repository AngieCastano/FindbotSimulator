# CLAUDE.md — Robot autónomo de búsqueda de víctimas en escombros

## Contexto del proyecto
Proyecto de grado de Ingeniería Electrónica (UNAD), metodología CDIO. Se construye un
**entorno virtual de prueba** y un **sistema robótico autónomo** que explora un escenario de
escombros post-sísmico generado proceduralmente, construye su propio mapa y busca víctimas.
Hay dos frentes que deben mantenerse **coherentes entre sí**:
- **Simulación** (`usar_ws/`): ROS 2 + Gazebo. Objetivo inmediato: MVP de punta a punta.
- **Hardware** (`hardware/`): circuito (KiCad) y firmware del microcontrolador (Wokwi/ESP32).
  Tiene su propio `hardware/CLAUDE.md` con el detalle eléctrico y la tabla de pines.

## Sensores y actuadores del robot (FUENTE ÚNICA — no agregar ni quitar sin preguntar)
Esta tabla manda sobre todo lo demás. El modelo de Gazebo (xacro), el circuito de KiCad y el
firmware deben tener exactamente estos elementos, con los mismos nombres de tópico.

| # | Elemento | Modelo real (referencia) | Conexión física | En Gazebo | Tópico ROS 2 | Para qué sirve | Estado |
|---|---|---|---|---|---|---|---|
| 1 | 2 motorreductores | TT 1:48 con encoder | TB6612FNG ← ESP32 | DiffDrive | `/cmd_vel` (entrada) | Mover el robot | MVP |
| 2 | 2 encoders de rueda | Hall de cuadratura del motor | ESP32 (GPIO con interrupción) | DiffDrive (odom) | `/odom`, `/joint_states` | Odometría para SLAM y Nav2 | MVP |
| 3 | IMU 6 ejes | MPU6050 | ESP32 por I2C | sensor `imu` | `/imu/data_raw` | Corregir el giro (yaw) de la odometría | MVP |
| 4 | LiDAR 2D 360° | RPLiDAR A1 | Computador por USB | `gpu_lidar` | `/scan` | Mapa (SLAM) y obstáculos | MVP |
| 5 | Cámara RGB-D | Intel RealSense D435 u Orbbec Astra | Computador por USB | `rgbd_camera` | `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | Detectar víctimas (forma humana) y obstáculos bajos | MVP |
| 6 | Monitor de batería | Divisor resistivo | ESP32 (ADC) | — (no aplica) | `/battery_state` | Seguridad: avisar batería baja | MVP |
| 7 | Cámara térmica | MLX90640 (32×24 px) | ESP32 por I2C | sensor `thermal` (víctimas a 34 °C de superficie) + `usar_sim_sensors/thermal_camera_sim` (ruido) | `/thermal/image` (32FC1, °C) | 2.º signo de vida: calor corporal | Fase 2 |
| 8 | Sensor de CO₂ | Figaro TGS4161 (350–10 000 ppm) | ESP32 (ADC, seguidor MCP6004; calefactor por MOSFET) | `usar_sim_sensors/co2_sim` (pluma de CO₂ de cada víctima + respuesta lenta del sensor) | `/co2/concentration` (`usar_msgs/GasConcentration`) | 3.er signo de vida: respiración (CO₂ exhalado) | Fase 2 |
| 9 | Arreglo de micrófonos | ReSpeaker USB Mic Array v2.0 (4 micrófonos) | Computador por USB | `usar_sim_sensors` (se define en la Fase 11) | se define en la Fase 11 | 4.º signo de vida: voz, golpes | Fase 2 |
| 10 | Radar Doppler | HB100 (10,525 GHz) | ESP32 (ADC, 2 etapas + pasabanda) | `usar_sim_sensors` (se define en la Fase 12) | se define en la Fase 12 | 5.º signo de vida: movimiento respiratorio (con el robot quieto) | Fase 2 |

Reglas:
- **Estado "MVP"** = obligatorio en simulación, circuito y firmware. **"Fase 2"** = el circuito
  deja el conector listo y el xacro lo incluye desactivado por parámetro (`use_thermal:=false`,
  `use_co2:=false`, ...).
- Gazebo no modela gases, sonido ni radar: en simulación esos sensores los generan nodos de
  `usar_sim_sensors` (Capa 2), con la excepción a la regla del ground truth descrita abajo.
- Las alturas y posiciones de los sensores en el xacro deben ser las del robot físico
  (tomarlas de `hardware/CLAUDE.md`, sección "Geometría").

## Arquitectura de procesamiento (robot físico)
- **ESP32** a bordo: controla motores (PID de velocidad por rueda), lee encoders, IMU, batería
  y (fase 2) cámara térmica. Se comunica con ROS 2 mediante **micro-ROS** por USB serial
  (WiFi como opción): suscribe `/cmd_vel`, publica `/odom`, `/joint_states`, `/imu/data_raw`,
  `/battery_state`.
- **Computador** (Ubuntu 24.04 + ROS 2 Jazzy): LiDAR y cámara RGB-D por USB, SLAM, Nav2,
  exploración, detección y métricas. Los mismos nodos de `usar_ws/` sirven para simulación y
  robot real; solo cambia el launch (`sim:=true/false`).
- Fusión: `robot_localization` (EKF) combina `/odom` + `/imu/data_raw` → `/odometry/filtered`,
  tanto en simulación como en el robot real.

## Stack de software (no cambiar sin preguntar)
- Ubuntu 24.04 + **ROS 2 Jazzy** + **Gazebo Harmonic** (gz-sim 8) + `ros_gz` (bridge y spawn).
- SLAM: `slam_toolbox` (online async). Navegación: **Nav2**. Fusión: `robot_localization`.
- Nodos propios en **Python 3** (rclpy), `ament_python`. Firmware del ESP32 en C++ (Arduino/PlatformIO + micro-ROS).
- Visualización: RViz2.

## Arquitectura por capas (del informe, Fase 2)
1. **Capa 1 – Generación procedural (PCG):** `E = G(θ, s)`. Entrada: semilla `s` y parámetros `θ`
   (tamaño del área, nº de bloques, dimensiones, densidad, nº de víctimas). Salida: mundo `.sdf`
   reproducible + **ground truth** (`ground_truth.yaml`: obstáculos, víctimas, área explorable).
   Restricciones: suelo plano, área delimitada por muros, escombros con colisión rígida, sin
   solapamientos, nada fuera de límites, zona libre alrededor del spawn, todas las víctimas
   alcanzables (flood-fill sobre rejilla inflada con el radio del robot). Las víctimas tienen
   color distintivo y temperatura de 37 °C (componente térmico de Gazebo) para la fase 2.
2. **Capa 2 – Simulación (Gazebo):** robot diferencial planar `χ = [x, y, θ]ᵀ` con exactamente
   los sensores de la tabla (ruido gaussiano en LiDAR, IMU y odometría).
3. **Capa 3 – Navegación y percepción (ROS 2):** EKF → `slam_toolbox` (`/map`, TF map→odom) →
   Nav2 (costmaps; capa de obstáculos con `/scan` y capa voxel con la profundidad de la cámara
   para obstáculos bajos) → acción `NavigateToPose`.
4. **Capa 4 – Misión de búsqueda:**
   - **Exploración por fronteras** (Yamauchi 1997): fronteras libre/desconocido en `/map`,
     clusters, meta = argmin `J(g) = α·C(g) − β·I(g)`; α, β en YAML; lista negra de metas fallidas.
   - **Detector de víctimas** con interfaz común por modalidad:
     - *Forma humana (MVP)*: segmentación de color en RGB + profundidad → punto 3D en `map`.
     - *Calor (fase 2)*: umbral de temperatura en `/thermal/image` → dirección de la víctima.
     Confirmación tras N observaciones, fusión de detecciones a menos de D m; publica
     `/victims/markers` y `/victims/detections` (incluye qué modalidad detectó).
5. **Evaluación:** `usar_metrics` (el ÚNICO que lee el ground truth) calcula cobertura
   `C% = A_explorada / A_explorable × 100`, tiempo, distancia, víctimas detectadas, tasa de
   detección, falsos positivos y error de localización, **por modalidad sensorial**. Exporta
   `results/<seed>_<fecha>.json` y `.csv`.

## Regla crítica de diseño (no romper nunca)
**El ground truth NO puede llegar al robot.** Ni el explorador, ni el detector, ni SLAM, ni Nav2
pueden leer `ground_truth.yaml` ni posiciones de Gazebo de víctimas u obstáculos. El robot decide
solo con sus sensores y su propio `/map`.

**Única excepción: los simuladores de sensores de la Capa 2 (`usar_sim_sensors`).** Hacen el
papel de la física que Gazebo no modela (gases, sonido, radar): para calcular qué mediría el
sensor pueden leer `ground_truth.yaml` y la pose real del robot (tópicos internos `/sim/*`).
Condiciones (las comprueba `usar_metrics/test/test_ground_truth_isolation.py`):
- solo publican **mediciones de sensor** realistas (concentración, imagen, espectro...), con
  ruido, alcance y atenuación; **nunca posiciones, distancias a víctimas ni identificadores**;
- ningún otro nodo del robot se suscribe a `/sim/*`;
- en el robot real no existen: su lugar lo ocupa el sensor físico, con el mismo tópico y
  mensaje.

## Estructura del repositorio
```
CLAUDE.md                 # este archivo (general + sensores)
docs/                     # informe, PLAN_MVP.md
usar_ws/src/
  usar_description/       # xacro del robot y sensores, config del bridge
  usar_worldgen/          # generador procedural
  usar_bringup/           # launch (sim:=true/false), params de Nav2, slam_toolbox, EKF, RViz
  usar_exploration/       # frontier_explorer
  usar_detection/         # victim_detector (modalidades: forma humana, térmica)
  usar_metrics/           # evaluator + análisis
  usar_msgs/              # mensajes propios (detecciones de víctimas, concentración de gas)
  usar_sim_sensors/       # simuladores de sensores de la Capa 2 (térmica, CO₂, sonido, radar)
hardware/
  CLAUDE.md               # detalle eléctrico, tabla de pines, geometría
  kicad/                  # esquemático
  firmware/               # ESP32 (PlatformIO + micro-ROS)
  wokwi/                  # simulación del circuito y firmware
scripts/  results/
```

## Comandos
- Compilar: `cd usar_ws && colcon build --symlink-install && source install/setup.bash`
- Generar mundo: `ros2 run usar_worldgen generate --seed 42 --out worlds/seed42`
- Misión simulada: `ros2 launch usar_bringup mission.launch.py seed:=42`
- Tests: `colcon test && colcon test-result --verbose`

## Forma de trabajar
- Trabaja **por fases** (ver `docs/PLAN_MVP.md`). No avances sin que la fase anterior compile
  y cumpla sus criterios de aceptación. Antes de cada fase, presenta un plan corto y espera confirmación.
- Si cambias un sensor, su posición o un tópico, actualiza en el mismo cambio: la tabla de este
  archivo, el xacro, `hardware/CLAUDE.md` y el firmware. Nunca dejes uno desactualizado.
- Después de cada cambio: compila, corre los tests y dime qué comando ejecutar para verificarlo.
- Parámetros en YAML, nada "quemado" en el código. Todo reproducible por semilla.
- Comentarios y docstrings en español. Nombres de código en inglés.
- Si algo del informe es ambiguo o técnicamente inviable, pregúntame antes de decidir.

## Sesiones paralelas y git
- Puede haber dos sesiones de Claude abiertas a la vez: **simulación** (solo `usar_ws/`,
  `scripts/`, `results/`) y **hardware** (solo `hardware/`). No edites fuera de tu zona.
- Solo la sesión de simulación lanza Gazebo y ejecuta `colcon build`.
- Haz commit **solo de tu carpeta** (`git add usar_ws/` o `git add hardware/`), nunca `git add -A`.
  Commit al cerrar cada fase con un mensaje descriptivo.
