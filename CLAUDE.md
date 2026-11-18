# CLAUDE.md — Robot autónomo de búsqueda de víctimas en escombros (simulación)

## Contexto del proyecto
Proyecto de grado de Ingeniería Electrónica (UNAD), metodología CDIO. Se construye un
**entorno virtual de prueba** y un **sistema robótico autónomo simulado** que explora un
escenario de escombros post-sísmico generado proceduralmente, construye su propio mapa y
busca víctimas simuladas. Objetivo inmediato: **Producto Mínimo Viable (MVP)** que funcione
de punta a punta con un solo comando.

## Stack (no cambiar sin preguntar)
- Ubuntu 24.04 + **ROS 2 Jazzy** + **Gazebo Harmonic** (gz-sim 8) + `ros_gz` (bridge y spawn).
- SLAM: `slam_toolbox` (modo online async). Navegación: **Nav2**.
- Lenguaje: **Python 3** (rclpy) para todos los nodos propios. Build con `colcon`, paquetes `ament_python`
  (y `ament_cmake` solo si hace falta para instalar URDF/mundos/launch).
- Visualización: RViz2.

## Arquitectura por capas (del informe, Fase 2)
1. **Capa 1 – Generación procedural (PCG):** `E = G(θ, s)`. Entrada: semilla `s` y parámetros `θ`
   (tamaño del área, nº de bloques, dimensiones, densidad, nº de víctimas). Salida: mundo `.sdf`
   reproducible + archivo **ground truth** (`ground_truth.yaml`: obstáculos, víctimas, área explorable).
   Restricciones: suelo plano, área delimitada por muros, escombros como cajas/cilindros con colisión
   rígida, sin solapamientos, sin obstáculos fuera de los límites, zona libre alrededor del spawn del
   robot y comprobación de que todas las víctimas sean alcanzables (flood-fill sobre rejilla inflada
   con el radio del robot).
2. **Capa 2 – Simulación (Gazebo):** robot diferencial planar, pose `χ = [x, y, θ]ᵀ`. Plugins:
   diff-drive (`/cmd_vel` → ruedas, publica `/odom` y TF odom→base_link), LiDAR 2D (`/scan`, con ruido
   gaussiano), cámara RGB-D (`/camera/image`, `/camera/depth_image`, `/camera/camera_info`).
3. **Capa 3 – Navegación y percepción (ROS 2):** `slam_toolbox` publica `/map` y TF map→odom;
   Nav2 (costmaps global/local, planificador, controlador) recibe metas por la acción `NavigateToPose`.
4. **Capa 4 – Misión de búsqueda:**
   - **Exploración por fronteras** (Yamauchi 1997): detecta fronteras libre/desconocido en `/map`,
     agrupa en clusters, y elige la meta minimizando `J(g) = α·C(g) − β·I(g)`
     (C = costo/distancia de trayectoria, I = celdas desconocidas alrededor de la frontera).
     α y β son parámetros ROS. Lista negra de metas fallidas. Termina cuando no quedan fronteras
     válidas o se agota el tiempo.
   - **Detector de víctimas:** signo de vida "forma humana" en el MVP. Las víctimas son modelos de
     color distintivo; el detector segmenta color en la imagen RGB, usa la profundidad para obtener
     el punto 3D y lo transforma al frame `map`. Agrupa detecciones cercanas (una víctima = una
     detección confirmada tras N observaciones). Publica `MarkerArray` en `/victims/markers` y
     guarda log.
5. **Evaluación (métricas):** nodo aparte que SÍ puede leer el ground truth y calcula:
   cobertura `C% = A_explorada / A_explorable × 100`, tiempo de exploración, distancia recorrida,
   víctimas detectadas, tasa de detección, falsos positivos y error de localización de víctimas
   (distancia estimada vs real, umbral de asociación configurable). Exporta `results/<seed>_<fecha>.json` y `.csv`.

## Regla crítica de diseño (no romper nunca)
**El ground truth NO puede llegar al robot.** Ni el explorador, ni el detector, ni SLAM, ni Nav2
pueden leer `ground_truth.yaml` ni posiciones de Gazebo de víctimas/obstáculos. Solo el paquete
`usar_metrics` lo lee. El robot decide únicamente con `/scan`, `/odom`, cámara y su propio `/map`.

## Estructura del workspace
```
usar_ws/src/
  usar_description/   # URDF/xacro del robot, sensores, config del bridge
  usar_worldgen/      # generador procedural (CLI: --seed, --size, --n-obstacles, --n-victims, --out)
  usar_bringup/       # launch files, params de Nav2, slam_toolbox, RViz
  usar_exploration/   # nodo frontier_explorer
  usar_detection/     # nodo victim_detector
  usar_metrics/       # nodo evaluator + scripts de análisis
scripts/              # run_experiment.sh, batch de varias semillas
results/              # salidas de experimentos (no versionar datos pesados)
```

## Comandos
- Compilar: `cd usar_ws && colcon build --symlink-install && source install/setup.bash`
- Generar mundo: `ros2 run usar_worldgen generate --seed 42 --out worlds/seed42`
- Misión completa: `ros2 launch usar_bringup mission.launch.py seed:=42`
- Tests: `colcon test && colcon test-result --verbose`

## Forma de trabajar
- Trabaja **por fases** (ver `docs/PLAN_MVP.md`). No avances de fase sin que la anterior compile
  y cumpla sus criterios de aceptación.
- Antes de cada fase, presenta un plan corto y espera mi confirmación.
- Después de cada cambio: compila, corre los tests y dime exactamente qué comando ejecutar para
  verlo funcionar en Gazebo/RViz (yo lo verifico visualmente).
- Prefiere paquetes estándar (slam_toolbox, Nav2, ros_gz) antes que reimplementar.
- Parámetros en archivos YAML, nada "quemado" en el código. Todo reproducible por semilla.
- Comentarios y docstrings en español. Nombres de código en inglés.
- Haz un commit de git al cerrar cada fase con un mensaje descriptivo.
- Si algo del informe es ambiguo o técnicamente inviable, pregúntame antes de decidir.
