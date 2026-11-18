# Capa 3 — Configuración de SLAM y navegación

Este documento justifica los parámetros de mapeo (slam_toolbox) y navegación (Nav2) del robot
diferencial *findbot*. Los valores están en `usar_ws/src/usar_bringup/config/`.

## 1. Arquitectura de la capa

```
/scan ──► slam_toolbox ──► /map  +  TF map→odom
/odom, TF odom→base_footprint (Gazebo DiffDrive)

/map, /scan, /camera/depth/points ──► costmaps ──► planificador (SmacPlanner2D)
                                                └► controlador (Rotation Shim + RPP)
NavigateToPose (acción) ──► bt_navigator ──► /cmd_vel_nav ──► velocity_smoother
        ──► /cmd_vel_smoothed ──► collision_monitor ──────────► /cmd_vel
teleop (scripts/teleop.sh) ──► /cmd_vel_teleop ──► collision_monitor_teleop ──► /cmd_vel
```

El árbol de transformaciones es `map → odom → base_footprint → base_link → {lidar_link,
camera_link → camera_optical_frame, ruedas}`. `map→odom` lo publica slam_toolbox (corrige la
deriva de la odometría), `odom→base_footprint` lo publica el plugin de tracción diferencial y el
resto son transformaciones fijas del modelo URDF.

## 2. SLAM — slam_toolbox (modo *online asíncrono*)

Se eligió slam_toolbox porque es el SLAM 2D de referencia en ROS 2, basado en grafos de poses con
cierre de lazos (Karto + optimizador Ceres). El modo asíncrono procesa el escaneo más reciente
sin acumular retraso, lo que es adecuado para exploración en tiempo real.

| Parámetro | Valor | Justificación |
|---|---|---|
| `resolution` | 0.05 m | Igual a la rejilla del *ground truth* generado por la Capa 1, de modo que la cobertura se calcula comparando celda a celda sin re-muestreo. 5 cm es suficiente para representar escombros de 0.3 m o más. |
| `min_laser_range` / `max_laser_range` | 0.12 / 12 m | Coinciden con las especificaciones del LiDAR simulado. |
| `minimum_travel_distance` / `minimum_travel_heading` | 0.2 m / 0.2 rad | Se integra un escaneo nuevo cada 20 cm u 11°. Valores menores que los por defecto (0.5) porque el entorno es denso y el robot gira mucho entre obstáculos; un mapa más detallado mejora la detección de fronteras en la Capa 4. |
| `map_update_interval` | 1 s | El mapa se republica cada segundo para que el explorador de fronteras trabaje con información reciente. |
| `do_loop_closing` | `true` | Al volver a zonas ya vistas, el cierre de lazos corrige la deriva acumulada de la odometría. |
| `loop_search_maximum_distance` | 4 m | Radio de búsqueda de cierres de lazo, acorde al tamaño del escenario (20 × 20 m). |
| `transform_publish_period` | 0.02 s | TF `map→odom` a 50 Hz, suficiente para el controlador (20 Hz). |

## 3. Representación del entorno — costmaps

Nav2 usa dos mapas de costo: uno **global** (todo el mapa, para planificar rutas) y uno **local**
(ventana móvil de 5 × 5 m alrededor del robot, para el control reactivo).

| Parámetro | Valor | Justificación |
|---|---|---|
| `robot_radius` | 0.30 m | Radio envolvente físico del robot (0.25 m: chasis 0.40 × 0.30 m con ruedas) más un margen de seguridad de 5 cm. Es el mismo radio con el que la Capa 1 calcula el área explorable, lo que hace coherentes navegación y métricas. Con 0.28 m el planificador aceptaba huecos solo 4 cm más anchos que el robot, en los que este quedaba bloqueado (ver sección 7). |
| `inflation_radius` | 0.70 m | Zona de costo decreciente alrededor de los obstáculos. El robot tiende a circular por el centro de los pasillos y el planificador prefiere pasos amplios, pero aún puede atravesar huecos de unos 0.65 m cuando no hay alternativa. |
| `cost_scaling_factor` | 2.0 | Caída exponencial suave del costo: el costo sigue siendo alto a varios decímetros del obstáculo, lo que aleja las rutas de los huecos justos. |
| `track_unknown_space` (global) | `true` | Distingue espacio desconocido de espacio libre; imprescindible para la exploración por fronteras. |
| Ventana local | 5 × 5 m | Cubre la distancia de frenado y el horizonte del controlador con margen, con bajo costo de cómputo. |

**Fuentes de obstáculos.** Además del LiDAR (plano de escaneo a 0.18 m), los costmaps usan la
**nube de puntos de la cámara de profundidad**, filtrada entre 0.05 y 0.5 m de altura. Esto
permite detectar obstáculos que quedan por debajo del plano del LiDAR, en particular las
extremidades de las víctimas acostadas (0.10–0.14 m). El umbral inferior de 0.05 m descarta el
suelo. La imagen de profundidad se diezma ×4 (640×480 → 160×120 ≈ 19 000 puntos) antes de
convertirla en nube, para reducir el costo de cómputo sin perder obstáculos relevantes a menos
de 3 m. El mapa de SLAM se construye solo con el LiDAR; la cámara solo afecta a la navegación.

Cada sensor escribe en **su propia capa** del costmap: el LiDAR en una `ObstacleLayer` 2D y la
cámara en una `VoxelLayer` 3D (vóxeles de 5 cm hasta 0.5 m de altura). Si compartieran capa, cada
rayo del LiDAR que pasa *por encima* de una extremidad (a 0.18 m) borraría la marca que dejó la
cámara, porque en una capa 2D despejar una celda la despeja a todas las alturas. En la capa 3D un
rayo de la cámara solo despeja los vóxeles que atraviesa, de modo que las extremidades persisten
en el costmap aunque dejen de estar en el campo de visión.

## 4. Planificador global — SmacPlanner2D

Planificador A* sobre la rejilla del costmap (vecindad de 8 celdas) con suavizado posterior.

| Parámetro | Valor | Justificación |
|---|---|---|
| `allow_unknown` | `true` | Permite trazar rutas a través de zonas aún no mapeadas. Sin esto, el robot no podría dirigirse a fronteras de exploración. |
| `cost_travel_multiplier` | 2.0 | Las rutas evitan las zonas de costo alto (cerca de escombros) y prefieren el centro de los pasillos. |
| `tolerance` | 0.25 m | Si la meta exacta es inalcanzable, acepta un punto válido a menos de 25 cm. |
| `max_planning_time` | 2 s | Límite de tiempo por plan; en un escenario de 20 × 20 m los planes tardan milisegundos. |

## 5. Controlador local — Rotation Shim + Regulated Pure Pursuit (RPP)

**Regulated Pure Pursuit** sigue la ruta global persiguiendo un punto objetivo (*lookahead*)
situado a cierta distancia sobre la ruta, y **regula** la velocidad lineal: frena en curvas
cerradas, cerca de obstáculos y al aproximarse a la meta. Es un algoritmo geométrico simple,
determinista y fácil de justificar, adecuado para un robot diferencial a baja velocidad.
Se combina con **Rotation Shim**, que hace girar al robot sobre su propio eje cuando la ruta
arranca en una dirección muy distinta a la orientación actual (aprovechando que un robot
diferencial puede girar en el sitio) antes de ceder el control a RPP.

| Parámetro | Valor | Justificación |
|---|---|---|
| `desired_linear_vel` | 0.35 m/s | Velocidad de crucero prudente para un entorno con obstáculos cercanos y SLAM en línea. |
| `lookahead_dist` (min–max) | 0.6 m (0.3–0.9) | Un *lookahead* corto sigue la ruta con precisión en pasillos estrechos; escala con la velocidad (`lookahead_time` = 1.5 s). |
| `regulated_linear_scaling_min_radius` | 0.9 m | En curvas de radio menor a 0.9 m reduce la velocidad para no salirse de la ruta. |
| `use_cost_regulated_linear_velocity_scaling` | `true` | Reduce la velocidad cuando el robot pasa cerca de obstáculos (a menos de 0.5 m). |
| `use_collision_detection` | `true` | Proyecta la trayectoria 1 s hacia adelante y se detiene si colisionaría. |
| `angular_dist_threshold` (Shim) | 0.785 rad (45°) | Si el error de orientación supera 45°, primero gira en el sitio a 1 rad/s. |
| `allow_reversing` | `false` | El robot solo tiene sensores de profundidad al frente, por lo que no debe circular hacia atrás. |
| Tolerancia de meta | 0.25 m / 0.5 rad | Suficiente para exploración e inspección de víctimas; no se requiere precisión de atraque. |

## 6. Suavizado de velocidad y comportamientos de recuperación

- **velocity_smoother**: limita velocidad (0.4 m/s, 1.0 rad/s) y aceleración (1.0 m/s²,
  2.0 rad/s²) por debajo de los límites del plugin de tracción (0.5 m/s, 1.5 rad/s), evitando
  arranques bruscos que degradan la odometría y el SLAM.
- **Comportamientos de recuperación** (`behavior_trees/navigate_to_pose_no_backup.xml`, basado
  en el árbol por defecto de Nav2 con replanificación a 1 Hz): si el robot queda bloqueado o sin
  progreso (menos de 0.3 m en 15 s), limpia los costmaps, gira en el sitio o espera, y vuelve a
  planificar. Se hicieron dos cambios respecto al árbol por defecto:
  - **Sin retroceso (BackUp).** El robot solo tiene sensores de profundidad al frente; al
    retroceder a ciegas podía empujar extremidades de víctimas que el LiDAR no ve.
  - **Limpieza parcial** (`ClearCostmapExceptRegion`, 1.5 m): se borran los obstáculos lejanos
    pero se conservan los cercanos al robot, de modo que no se olvidan las extremidades
    detectadas por la cámara justo cuando el robot está maniobrando junto a ellas.

## 6b. Filtro de seguridad — Collision Monitor

Toda orden de velocidad, de Nav2 o de la teleoperación, pasa por un `collision_monitor`
(`nav2_collision_monitor`) antes de llegar a las ruedas. A diferencia de los costmaps, que guían
la planificación, el monitor actúa directamente sobre la velocidad con los datos más recientes
del LiDAR y de la nube de profundidad (0.05–0.5 m de altura). Es una última barrera que también
protege al robot cuando se conduce manualmente.

Se usa un polígono de parada que **depende del sentido del movimiento** (`velocity_polygon`):

| Movimiento | Zona vigilada (desde el centro de giro) | Justificación |
|---|---|---|
| Avance (v > 0.02 m/s) | 0.30 m por delante del frontal, 0.54 m de ancho | Distancia de frenado ≈ 0.18 m a 0.5 m/s (aceleración 1 m/s² + latencia de sensores); al detenerse, el obstáculo sigue dentro del campo de la cámara (recorte cercano 0.05 m). |
| Retroceso (v < −0.02 m/s) | 0.15 m por detrás de la parte trasera | Solo el LiDAR vigila esta zona. |
| Giro en el sitio | Círculo de 0.31 m | Envuelve el círculo que barre la carcasa (0.29 m). |

Con una única huella circular, un obstáculo que quedara dentro del círculo bloquearía cualquier
movimiento, incluido alejarse. Con zonas por sentido, si algo queda pegado delante el robot aún
puede retroceder o girar.

**Memoria de obstáculos.** Junto al frontal hay un punto ciego: el LiDAR pasa por encima de las
extremidades y la cámara no ve a menos de 5 cm de su lente. Si el monitor usara solo datos
instantáneos, al frenar con una extremidad en ese punto ciego dejaría de verla y volvería a
permitir el avance. Por eso tiene una tercera fuente: el nodo `costmap_obstacle_cloud`
(`usar_bringup`) publica las celdas letales del costmap local, que conserva las extremidades
vistas por la cámara, como nube de puntos (`/local_costmap/obstacle_points`, 5 Hz).

## 7. Verificación y ajustes derivados de las pruebas

`scripts/smoke_test_nav.py` comprueba de forma automática el árbol TF completo, la publicación de
`/map` a 0.05 m, la orientación correcta de la nube de profundidad (todos los puntos delante del
robot y el suelo a z ≈ 0) y que `NavigateToPose` lleva al robot a una meta a ~7 m rodeando
escombros.

Las pruebas comparando la odometría y la pose estimada por SLAM con la pose real del simulador
(usada solo como referencia de evaluación) llevaron a dos correcciones:

1. **Geometría de contacto de las ruedas.** Con ruedas de colisión cilíndrica, el motor físico
   generaba el contacto en el borde interior de la rueda; la separación efectiva entre ruedas
   resultaba ~13 % menor que la nominal y la odometría subestimaba los giros (≈ 45° de error por
   vuelta). El error angular acumulado superaba la ventana de búsqueda del *scan matcher* y el
   mapa se deformaba (obstáculos duplicados, paredes giradas). Usando una **esfera** como
   geometría de colisión de las ruedas (la forma visual sigue siendo un cilindro) el contacto
   queda en el plano medio y la odometría coincide con el giro real.
2. **Margen de seguridad.** Con `robot_radius` = 0.28 m el robot entraba en huecos de 0.60 m y
   quedaba bloqueado: su huella tocaba la zona letal del costmap y el controlador y los
   comportamientos de recuperación rechazaban cualquier movimiento. Se aumentó a 0.30 m y se
   amplió el inflado (0.70 m, factor 2.0).
3. **Centro de giro.** El eje de las ruedas estaba 5 cm por delante de `base_footprint`. Un robot
   diferencial gira sobre el punto medio de su eje, pero la odometría y Nav2 suponen que lo hace
   sobre `base_footprint`. Al girar en el sitio, la huella real se desplazaba hacia los
   obstáculos y la odometría acumulaba error de posición (≈ 0.1 m). Se colocó `base_link` sobre
   el eje de ruedas, como es la convención en ROS; el chasis queda desplazado 5 cm hacia atrás.
4. **Obstáculos bajos (extremidades de víctimas).** Con el LiDAR y la cámara en la misma capa,
   el LiDAR borraba del costmap global las piernas y brazos detectados por la cámara; el
   planificador trazaba rutas sobre ellas y el robot se subía a la víctima. Se separaron los
   sensores en capas distintas (sección 3), y se eliminó el retroceso de las recuperaciones
   (sección 6).

Resultado en la semilla 42 tras los ajustes (2 vueltas en el sitio y 5 metas encadenadas, ~45 m
de recorrido): 5/5 metas alcanzadas, error de la odometría ≤ 0.02 m y ≤ 0.1°, error de
localización de SLAM ≤ 0.09 m y ≤ 0.6°, y ninguna detección de colisión inminente.

**Limitación conocida.** La odometría de ruedas no detecta deslizamientos: si el robot choca o
queda trabado, las ruedas siguen girando y la odometría acumula un desplazamiento falso que
degrada el mapa. El filtro de colisión evita los choques tanto en modo autónomo como al
teleoperar con `scripts/teleop.sh`, pero no si se publica directamente en `/cmd_vel`.

**Metas en huecos muy estrechos.** Si se pide una meta a menos de ~0.3 m de una extremidad (por
ejemplo, entre dos víctimas muy juntas), Nav2 la aborta sin tocar a la víctima, pero el robot puede
quedar en una zona de costo inscrito desde la que no puede girar. Por eso el explorador de la
Capa 4 debe elegir metas con holgura respecto a los obstáculos y poner en lista negra las que fallen.
