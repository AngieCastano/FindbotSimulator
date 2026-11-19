# Capa 4a — Exploración autónoma por fronteras

Este documento describe el nodo `frontier_explorer` (paquete `usar_exploration`) y justifica sus
parámetros (`usar_exploration/config/explorer.yaml`).

## 1. Principio

Una **frontera** es el límite entre el espacio conocido libre y el espacio desconocido
(Yamauchi, 1997). Dirigirse a una frontera garantiza observar zona nueva; repetir el proceso hasta
que no queden fronteras alcanzables explora todo el espacio accesible. El explorador usa
únicamente el mapa que construye el propio robot (`/map`, de slam_toolbox), su costmap global y su
pose estimada (TF `map → base_footprint`); **no tiene acceso al ground truth**.

```
/map ──► detección de fronteras ──► agrupación (clusters) ──► meta segura por cluster
                                                                     │
/global_costmap/costmap ─────────────────────────────────────────────┤ (holgura)
                                                                     ▼
            ComputePathToPose (Nav2) ──► C(g)        I(g) ◄── celdas desconocidas
                                   └──────► J(g) = α·C(g) − β·I(g) ──► argmin ──► NavigateToPose
```

## 2. Algoritmo

1. **Clasificación de celdas** de `/map`: desconocida (−1), libre (≤ `free_threshold`) u ocupada
   (≥ `occupied_threshold`).
2. **Celdas frontera:** celdas libres con al menos un vecino desconocido en vecindad 4.
3. **Agrupación:** componentes conexas en vecindad 8 (equivalente a un recorrido BFS). Cada
   cluster se divide además en tramos con una rejilla de `segment_size`: una frontera larga o en
   anillo tiene un centroide que no representa a ninguna de sus partes. Es el caso del borde del
   primer escaneo, porque slam_toolbox solo marca como libre una celda atravesada por al menos dos
   rayos y el primer mapa es un disco de ≈ 1 m alrededor del robot. Se descartan los tramos con
   menos de `min_cluster_size` celdas, que suelen ser ruido del LiDAR o huecos sin interés.
4. **Meta segura:** se parte de la celda del cluster más cercana a su centroide (el centroide de
   una frontera curva puede caer fuera de ella) y se busca, a menos de `goal_search_radius`, la
   celda libre más cercana que esté a más de `min_goal_clearance` de cualquier obstáculo del mapa
   y cuyo costo en el costmap global no supere `max_goal_cost`. El costmap incluye las
   extremidades de las víctimas detectadas por la cámara, que el mapa de SLAM (solo LiDAR) no
   contiene. Si no existe esa celda, la frontera se descarta: así se evita enviar al robot a huecos
   estrechos en los que quedaría sin poder maniobrar.
5. **Ganancia de información I(g):** número de celdas desconocidas a menos de `info_radius` de la
   meta.
6. **Costo de trayectoria C(g):** longitud de la ruta que devuelve el planificador de Nav2
   (acción `ComputePathToPose`). Solo se calcula para los `max_candidates` candidatos más cercanos
   en línea recta, para acotar el tiempo de decisión. Si no hay ruta, se usa la distancia
   euclidiana multiplicada por `plan_fail_penalty`.
7. **Selección:** se normalizan C e I dividiendo por su máximo entre los candidatos, de modo que
   ambos términos quedan en [0, 1] y α y β son pesos comparables, y se elige
   `g* = argmin J(g) = α·C(g) − β·I(g)`.
8. **Ejecución:** la meta se envía con `NavigateToPose`. Se abandona y se elige otra si:
   - Nav2 la aborta;
   - el robot no avanza `progress_distance` en `progress_timeout`;
   - se supera `goal_timeout`;
   - al revisarla cada `replan_period`, la zona ya quedó observada (I(g) < `min_info_gain`).
9. **Lista negra:** toda meta terminada, alcanzada o fallida, se añade a una lista negra; ninguna
   meta nueva puede quedar a menos de `blacklist_radius` de ellas. Esto evita bucles sobre
   fronteras inalcanzables o inobservables (por ejemplo, el interior de un escombro).
10. **Rechazos y fallos inmediatos:** si Nav2 rechaza la meta (p. ej. aún se está activando) o la
    aborta en menos de `quick_failure_time`, el problema es la situación del robot y no la meta:
    se reintenta tras `rejection_retry_delay` sin añadirla a la lista negra (salvo tras
    `max_rejections` intentos seguidos).
11. **Escape de la zona inscrita:** antes de elegir meta se comprueba el costo del costmap global
    en la posición del robot. Si está en la zona inscrita (a menos de `robot_radius` de un
    obstáculo, por ejemplo porque una extremidad se detectó tarde), el planificador de Nav2
    rechazaría cualquier ruta ("Start occupied"). El explorador elige entonces, entre 12 rumbos,
    el de menor costo del costmap a 0.15, 0.30 y 0.45 m. Prefiere avanzar o retroceder en línea
    recta, porque girar cerca de un obstáculo barre las esquinas de la carcasa. Después se desplaza
    hasta `escape_distance` o hasta salir de la zona. La orden se publica en `/cmd_vel_teleop`, de
    modo que pasa por el collision_monitor y no puede provocar un choque.

    *Añadido en la Fase 6:* el escape también se activa tras `stuck_failures` metas seguidas
    fallidas por falta de progreso sin que el robot se mueva del sitio. En la prueba de la Fase 6
    el robot quedó 140 s inmóvil fuera de la zona inscrita: el controlador RPP veía una colisión
    en su trayectoria ("Controller patience exceeded") y abortaba cada meta. En ese caso el
    escape recorre `escape_distance` completo.
12. **Fin:** cuando no hay fronteras válidas durante `no_frontier_timeout` seguidos, o al agotar
    `max_time`. El nodo cancela la meta en curso y publica `finished:no_frontiers` o
    `finished:timeout` en `/exploration/status`.

## 3. Parámetros

| Parámetro | Valor | Justificación |
|---|---|---|
| `alpha`, `beta` | 1.0, 1.0 | Peso igual a distancia e información (con C e I normalizados). β mayor favorece zonas grandes sin explorar aunque estén lejos; α mayor, fronteras cercanas (menos recorrido, más metas). |
| `free_threshold` / `occupied_threshold` | 20 / 65 | Umbrales de ocupación habituales en ROS (map_server usa 0.25 / 0.65). |
| `min_cluster_size` | 8 celdas | 0.4 m de frontera: menor que el ancho del robot no aporta un paso útil. |
| `segment_size` | 1.5 m | Tramos de frontera de tamaño comparable a la zona que se observa al llegar (`info_radius`). |
| `info_radius` | 1.0 m | Del orden del alcance útil para descubrir zona nueva al llegar a la meta en un escenario denso. |
| `min_info_gain` | 10 celdas | 0.025 m² sin explorar: por debajo, la frontera se considera cerrada. |
| `goal_search_radius` | 1.0 m | Margen para desplazar la meta desde la frontera hacia una zona con holgura. |
| `min_goal_clearance` | 0.45 m | `robot_radius` (0.30 m) + 0.15 m: el robot puede girar en la meta sin entrar en la zona inscrita del costmap. |
| `max_goal_cost` | 70 | Costo del costmap global (escala 0–100) a ≈ 0.47 m de un obstáculo con el inflado configurado (0.70 m, factor 2.0). |
| `max_candidates` | 6 | Las peticiones al planificador se hacen una a una (una nueva cancela la anterior); 6 mantiene la decisión por debajo de un segundo. |
| `plan_fail_penalty` | 3.0 | Una frontera sin ruta conocida no se descarta (la ruta puede abrirse al explorar), pero se desfavorece. |
| `replan_period` | 5 s | Revisión periódica de la meta en curso sin interrumpir la navegación. |
| `blacklist_radius` | 0.6 m | Algo mayor que el diámetro del robot. |
| `progress_distance` / `progress_timeout` | 0.3 m / 20 s | Detecta bloqueos que Nav2 no reporta (p. ej. recuperaciones repetidas). |
| `goal_timeout` | 120 s | Cota por meta: en un escenario de 20 × 20 m ninguna ruta razonable dura más. |
| `no_frontier_timeout` | 10 s | Espera a que el mapa se actualice antes de declarar el fin. |
| `max_time` | 900 s | Límite de la misión (15 min de tiempo simulado). |
| `quick_failure_time` | 3 s | Un aborto tan rápido indica que Nav2 no pudo ni empezar (p. ej. "Start occupied"). |
| `escape_cost_threshold` | 99 | Costo "inscrito" en la escala 0–100 del costmap publicado (100 = letal). |
| `escape_distance` / `escape_speed` | 0.25 m / 0.12 m/s | Basta para salir de la zona inscrita (0.30 m) a baja velocidad. |
| `stuck_failures` | 2 | Una meta sin progreso puede deberse a la meta; dos seguidas en el mismo sitio indican que el robot está bloqueado. |

## 4. Interfaces

| Interfaz | Tipo | Descripción |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` (entrada) | Mapa de slam_toolbox. |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` (entrada) | Holgura de las metas. |
| `compute_path_to_pose` | acción Nav2 (cliente) | Longitud de ruta, C(g). |
| `navigate_to_pose` | acción Nav2 (cliente) | Ejecución de la meta. |
| `/exploration/status` | `std_msgs/String` | `waiting`, `exploring`, `finished:no_frontiers`, `finished:timeout`. |
| `/exploration/frontiers` | `visualization_msgs/MarkerArray` | Celdas frontera (cian), candidatos (amarillo), meta elegida (verde), lista negra (rojo). |

## 5. Verificación

- **Pruebas unitarias** (`test/test_frontier_core.py`), con rejillas sintéticas: clasificación,
  vecindad 4 de las fronteras, agrupación en vecindad 8 y descarte de clusters pequeños, celda
  representativa, ganancia de información, meta segura con holgura y con veto del costmap, lista
  negra, normalización de J(g) y efecto de α y β, longitud de ruta y alineación de rejillas.
- **Prueba de extremo a extremo** en la semilla 42 (sección 6).

## 6. Resultados

Prueba de extremo a extremo en la semilla 42 (`exploration.launch.py`, sin interfaz gráfica). La
cobertura se calculó **fuera del robot**, con un script de evaluación que compara el `/map` con el
área explorable del ground truth (251.3 m²). Es la misma métrica que implementará la Fase 6.

| Métrica | Resultado |
|---|---|
| Cobertura del área explorable | **96.2 %** (92.5 % a los 480 s) |
| Tiempo | 900 s (fin por `max_time`) |
| Distancia recorrida | 99 m |
| Metas enviadas / completadas | 32 / 14 (las demás se abandonaron por falta de progreso o aborto de Nav2) |
| Escapes de la zona inscrita | 2, ambos con éxito |
| Inclinación máxima del robot | 0.0° (no se subió a ningún objeto) |
| Distancia mínima del centro del robot a un objeto | 0.28 m (carcasa: 0.25 m), sin contacto |

**Limitación observada.** La mayor parte del tiempo perdido se debe al controlador RPP cerca de los
escombros: su comprobación de colisión detiene al robot cuando el arco hacia el punto *lookahead*
entra en la zona inscrita, y la meta se abandona tras 20 s sin progreso. Con el límite de 15 min
la exploración termina por tiempo con más del 95 % del área cubierta; quedan fronteras pequeñas en
zonas estrechas entre escombros.
