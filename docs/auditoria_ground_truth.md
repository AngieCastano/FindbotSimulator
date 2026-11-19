# Auditoría: el ground truth no llega al robot

Regla crítica del proyecto (CLAUDE.md): ni el explorador, ni el detector, ni SLAM, ni Nav2 pueden
leer `ground_truth.yaml` ni posiciones de Gazebo de víctimas u obstáculos. Solo `usar_metrics` lo
lee. El robot decide únicamente con `/scan`, `/odom`, la cámara y su propio `/map`.

La regla se verificó de tres formas: búsqueda en el código, grafo de ROS durante una misión real y
pruebas automáticas que fallan si se rompe.

## 1. Búsqueda en el código

`grep -rn "ground_truth" usar_ws/src --include=*.py --include=*.yaml --include=*.xml`
(sin las carpetas `test/`):

| Archivo | Uso | Veredicto |
|---|---|---|
| `usar_worldgen/cli.py`, `ground_truth.py` | Generan y escriben `ground_truth.yaml` | Herramienta offline (Capa 1), no corre en el robot |
| `usar_worldgen/plot.py` | Lee el ground truth para dibujar la vista superior | Herramienta offline |
| `usar_worldgen/spawn.py` | Comentario: `spawn.yaml` es público y NO contiene el escenario | Solo comentario |
| `usar_metrics/evaluator.py` | Lee el ground truth para calcular las métricas | Permitido: es el evaluador |
| `usar_bringup/launch/mission.launch.py` | Genera el mundo si falta y pasa la ruta del ground truth al nodo `evaluator` | Permitido: solo lo recibe el evaluador |
| `usar_bringup/launch/sim.launch.py` | Comentario: "este launch NO lee ground_truth.yaml" (usa `spawn.yaml`) | Solo comentario |
| `usar_exploration/frontier_explorer.py` | Docstring: "este nodo NUNCA lee ground_truth.yaml ni poses de Gazebo" | Solo comentario |
| `usar_detection/victim_detector.py` | Docstring: ídem | Solo comentario |

**Poses de Gazebo.** El servidor de Gazebo publica las poses de todos los modelos
(`scene-broadcaster`), pero en la red interna de Gazebo, invisible para ROS. Solo llega a ROS lo que
el puente `ros_gz_bridge` traduce, y su configuración (`usar_description/config/bridge.yaml`) se
limita a:

| Tópico | Dirección | Origen |
|---|---|---|
| `/clock` | Gazebo → ROS | Reloj de simulación |
| `/cmd_vel` | ROS → Gazebo | Órdenes de velocidad |
| `/odom`, `/tf` (odom → base_footprint) | Gazebo → ROS | Plugin `DiffDrive`: odometría **integrada desde las ruedas**, con deriva, no la pose real |
| `/joint_states` | Gazebo → ROS | Ángulos de las ruedas |
| `/scan` | Gazebo → ROS | LiDAR con ruido gaussiano |
| `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | Gazebo → ROS | Cámara RGB-D |

Ningún tópico `/world/...` ni `/model/...`, y el robot no usa los plugins
`OdometryPublisher` ni `PosePublisher` (que publicarían la pose real).

## 2. Grafo de ROS durante una misión

Misión `mission.launch.py seed:=42` sin ventanas; a los 40 s se consultaron con la API de grafo de
rclpy las suscripciones de cada nodo (se omiten `/parameter_events`, `/rosout` y `/clock`):

| Nodo | Se suscribe a | Publica |
|---|---|---|
| `frontier_explorer` | `/map`, `/global_costmap/costmap`, `/tf`, `/tf_static`, acciones `navigate_to_pose` y `compute_path_to_pose` | `/exploration/status`, `/exploration/frontiers`, `/cmd_vel_teleop` |
| `victim_detector` | `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/tf`, `/tf_static` | `/victims/markers`, `/victims/detections`, `/victims/debug_image` |
| `slam_toolbox` | `/scan`, `/map` | `/map`, `/pose`, `/tf` (map → odom) |
| `global_costmap` / `local_costmap` | `/scan`, `/camera/depth/points`, `/map` (global) | costmaps |
| `controller_server`, `bt_navigator` | `/odom`, `/tf` | `/cmd_vel_nav`, acciones |
| `collision_monitor` (×2) | `/scan`, `/camera/depth/points`, `/local_costmap/obstacle_points`, órdenes de velocidad | `/cmd_vel` |
| `depth_decimate` / `depth_to_cloud` | `/camera/depth_image`, `/camera/camera_info` | `/camera/depth/points` |
| `costmap_obstacle_cloud` | `/local_costmap/costmap` | `/local_costmap/obstacle_points` |
| **`evaluator`** | `/map`, `/odom`, `/victims/detections`, `/exploration/status` | **nada** |

Todas las entradas de los nodos del robot vienen, directa o indirectamente, de `/scan`, `/odom`,
la cámara y el `/map` que construye SLAM. El evaluador no publica ningún tópico: lo que lee del
ground truth no puede volver al robot.

## 3. Pruebas automáticas

`usar_metrics/test/test_ground_truth_isolation.py` (se ejecuta con `colcon test`) falla si:

- algún archivo Python de `usar_description`, `usar_bringup`, `usar_exploration` o
  `usar_detection` importa `usar_worldgen.ground_truth` o usa `ground_truth` en código (se analiza
  el árbol sintáctico: los comentarios y docstrings no cuentan); única excepción autorizada,
  `mission.launch.py`;
- `mission.launch.py` entrega la ruta del ground truth a un nodo distinto de `evaluator`;
- `usar_metrics` crea algún publicador de ROS;
- el puente expone un tópico de Gazebo no auditado (en particular `/world/...` o `/model/...`);
- el robot deja de usar `DiffDrive` para la odometría o usa `OdometryPublisher` o
  `PosePublisher`.

Una prueba adicional verifica que el análisis distingue un uso real (import, ruta en una cadena)
de una mención en un comentario.
