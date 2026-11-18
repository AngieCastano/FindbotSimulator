# Plan del MVP — prompts por fase para Claude

Copia cada prompt en el chat de Claude dentro de VS Code, **uno a la vez**. Antes de pasar al
siguiente, verifica tú mismo el criterio de aceptación (✅). Si algo falla, pega el error
completo de la terminal y pide que lo corrija antes de seguir.

---

## Fase 0 — Verificar el entorno
```
Lee CLAUDE.md y docs/PLAN_MVP.md. No escribas código todavía.
Verifica mi entorno: versión de Ubuntu, que ROS 2 Jazzy esté instalado y se pueda hacer source,
que Gazebo Harmonic abra (gz sim --version), y que estén instalados ros-jazzy-ros-gz,
ros-jazzy-navigation2, ros-jazzy-nav2-bringup, ros-jazzy-slam-toolbox, ros-jazzy-xacro y
ros-jazzy-robot-state-publisher. Dame la lista de lo que falta y los comandos exactos para
instalarlo. Luego crea el workspace usar_ws con los 6 paquetes vacíos de CLAUDE.md, inicializa git
con un .gitignore adecuado para ROS 2 y confirma que `colcon build` termina sin errores.
```
✅ `colcon build` compila los 6 paquetes vacíos.

---

## Fase 1 — Capa 1: Generador procedural E = G(θ, s)
```
Implementa el paquete usar_worldgen según CLAUDE.md (Capa 1).
- CLI `generate` con argumentos: --seed, --size (ancho y largo en m), --n-obstacles,
  --obstacle-size-range, --n-victims, --robot-spawn, --out. También acepta un YAML de parámetros θ.
- Genera world.sdf para Gazebo Harmonic (suelo plano, 4 muros perimetrales, escombros como cajas
  y cilindros con rotación aleatoria en yaw, víctimas como modelos simples de forma humana de
  color naranja intenso) y ground_truth.yaml (obstáculos, víctimas, semilla, parámetros, y la
  rejilla del área explorable calculada con flood-fill desde el spawn inflando obstáculos con el
  radio del robot).
- Validaciones: sin solapamientos, nada fuera de límites, zona libre de 1 m alrededor del spawn,
  todas las víctimas alcanzables; si no se cumple, reintenta con sub-semilla y regístralo.
- Misma semilla = mismo mundo exacto. Escribe tests pytest de reproducibilidad y de restricciones.
- Agrega un script que dibuje en PNG la vista superior del mundo generado (para mi informe).
```
✅ Dos ejecuciones con `--seed 42` producen archivos idénticos; `gz sim world.sdf` abre el
escenario con escombros y víctimas; tests en verde.

---

## Fase 2 — Capa 2: Robot en Gazebo
```
Implementa usar_description y un launch sim.launch.py en usar_bringup:
- Robot diferencial en xacro (chasis ~0.4 m, 2 ruedas + rueda loca), con plugins de Gazebo
  Harmonic: DiffDrive, JointStatePublisher, LiDAR 2D 360° (rango 0.12–12 m, ruido gaussiano) y
  cámara RGB-D frontal.
- Configura ros_gz_bridge (YAML) para /cmd_vel, /odom, /tf, /scan, /clock, /camera/image,
  /camera/depth_image, /camera/camera_info. use_sim_time en todo.
- El launch recibe seed:=N, genera el mundo si no existe, lanza Gazebo, hace spawn del robot en
  la pose de spawn y abre RViz con robot, scan y cámara.
```
✅ El robot aparece en Gazebo; en RViz se ve el LiDAR y la imagen; puedo moverlo con
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

---

## Fase 3 — Capa 3: SLAM + Nav2
```
Integra slam_toolbox (online async) y Nav2 en un launch nav.launch.py, con parámetros propios en
usar_bringup/config (costmaps inflados al tamaño del robot, controlador apropiado para robot
diferencial en espacios estrechos). Verifica el árbol TF map→odom→base_link→sensores.
Explícame qué parámetros elegiste y por qué, en lenguaje que pueda poner en mi informe.
```
✅ Moviendo el robot con teleop se construye el mapa en RViz; enviando una "2D Goal Pose"
desde RViz el robot llega solo esquivando escombros.

---

## Fase 4 — Capa 4a: Exploración por fronteras con J(g)
```
Implementa el nodo frontier_explorer en usar_exploration según CLAUDE.md:
- Suscribe a /map, obtiene la pose del robot por TF, detecta celdas frontera, las agrupa (BFS),
  descarta clusters pequeños, y para cada candidato calcula C(g) (longitud de ruta usando el
  servicio de planificación de Nav2 o, si falla, distancia euclidiana) e I(g) (celdas
  desconocidas en un radio). Elige argmin J(g) = α·C(g) − β·I(g), normalizando C e I.
- Envía la meta con la acción NavigateToPose, replanifica al terminar, al fallar o cada T segundos;
  lista negra para metas fallidas.
- Criterio de fin: sin fronteras válidas o tiempo máximo. Publica /exploration/status y las
  fronteras como MarkerArray.
- α, β, tamaño mínimo de cluster, radio de información y tiempo máximo en YAML.
Recuerda: prohibido leer ground_truth.yaml.
```
✅ Con un solo launch el robot explora todo el escenario sin teleop y se detiene al final.

---

## Fase 5 — Capa 4b: Detector de víctimas
```
Implementa victim_detector en usar_detection (signo de vida: forma humana por color, MVP):
- Segmenta el color de las víctimas en /camera/image (HSV con umbrales en YAML), toma el
  centroide del blob, lee la profundidad, calcula el punto 3D con camera_info y lo transforma a map.
- Confirma una víctima tras N detecciones consistentes; fusiona detecciones a menos de D metros.
- Publica /victims/markers y /victims/detections; guarda log con tiempo, pose estimada y confianza.
- Deja el diseño preparado (interfaz/clase base) para agregar luego otros signos de vida
  (calor, sonido, CO₂) sin reescribir el nodo.
```
✅ Durante la exploración aparecen marcadores sobre las víctimas en RViz.

---

## Fase 6 — Métricas y experimento completo
```
Implementa usar_metrics (evaluator) y mission.launch.py que lanza todo con seed:=N:
- Métricas del informe: cobertura C% (celdas libres del /map dentro del área explorable del ground
  truth / área explorable), tiempo de exploración, distancia recorrida (integrando odometría),
  víctimas detectadas, tasa de detección, falsos positivos y error de localización (asociación
  por vecino más cercano con umbral configurable).
- Al terminar la misión guarda results/<seed>_<timestamp>.json y .csv, el mapa final (pgm/yaml)
  y una imagen comparando mapa construido vs ground truth con víctimas reales y estimadas.
- Script scripts/run_batch.sh que corre varias semillas en modo headless y un script Python que
  consolide los resultados en una tabla con media y desviación estándar.
```
✅ `ros2 launch usar_bringup mission.launch.py seed:=42` termina solo y deja los resultados;
el batch de 5 semillas genera la tabla consolidada.

---

## Fase 7 — Cierre del MVP
```
Revisa todo el proyecto: ejecuta los tests, corrige advertencias, verifica que ningún nodo del
robot accede al ground truth (búscalo en el código y muéstrame la evidencia), y escribe un
README.md con: requisitos, instalación, cómo ejecutar, arquitectura por capas (diagrama mermaid),
tópicos y parámetros principales, cómo reproducir un experimento por semilla, y limitaciones
conocidas del MVP. Haz el commit final y crea el tag v0.1-mvp.
```
✅ Alguien que clone el repo puede reproducir un experimento siguiendo solo el README.
