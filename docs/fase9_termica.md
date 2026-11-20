# Fase 9 — Signo de vida "calor corporal" con la cámara térmica (MLX90640)

El robot detecta víctimas por su **temperatura** con una cámara térmica de 32×24 píxeles, el
sensor 7 de la tabla de CLAUDE.md (fase 2: se activa con `use_thermal:=true`). La cámara no mide
distancia: cada mancha caliente da una dirección, y la distancia sale del mapa que construye el
propio robot.

**Uso.** `ros2 launch usar_bringup mission.launch.py seed:=42 world_params:=dusty use_thermal:=true`.
Esta fase añade el paquete `usar_sim_sensors`: tras compilar, hay que volver a ejecutar
`source install/setup.bash` en las terminales ya abiertas (si no, el launch falla con
"package 'usar_sim_sensors' not found").

## 1. Temperaturas en el escenario (`usar_worldgen`)

| Parámetro | Valor | Efecto |
|---|---|---|
| `ambient_temperature_c` | 20 °C | Suelo, muros y escombros (`<atmosphere>` del mundo). |
| `victim_temperature_c` | **34 °C** | Superficie de una persona vestida (la temperatura interna de 37 °C no se ve desde fuera). Componente térmico de Gazebo en cada parte del cuerpo. |
| `n_warm_debris` | 0 | Escombros tibios (sol, tuberías): falsos positivos térmicos para la Fase 13. |
| `warm_debris_temperature_range` | 28–45 °C | Temperatura de los escombros tibios. |

- Los escombros tibios se eligen entre los ya colocados, con un flujo aleatorio propio: no
  cambian la disposición del escenario.
- Ground truth formato 3: `temperature_c` en víctimas y escombros tibios, `ambient_temperature_c`.
- **Mundos ya generados:** `ensure_world` regenera la carpeta del mundo si se generó con otros
  parámetros o con un formato anterior (la misma semilla da el mismo escenario). Los mundos de
  fases anteriores ganan las temperaturas sin borrar nada a mano.
- `ros2 run usar_worldgen generate --seed 42 --n-warm-debris 6` y `--victim-temperature 37`.

## 2. Sensor simulado

| Elemento | Valor | Fuente |
|---|---|---|
| Resolución | 32×24 px | MLX90640 |
| Campo de visión | 55° horizontal (≈ 42° vertical: Gazebo usa píxeles cuadrados; el real tiene 35°) | versión BAA |
| Frecuencia | 4 Hz | |
| Posición | 6 cm a la derecha de la cámara RGB-D y 3 mm más abajo (0.117 m del suelo) | `hardware/CLAUDE.md`, Geometría |
| Ruido temporal (NETD) | 0.2 K por imagen | |
| Ruido de patrón fijo | 0.3 K por píxel, constante | |

**Cadena de la imagen:**

```
Gazebo (sensor "thermal", mono16, 0.01 K/unidad)
  → /sim/thermal/image_raw  (bridge_thermal.yaml)
  → usar_sim_sensors/thermal_camera_sim  (ruido del MLX90640, conversión a °C)
  → /thermal/image  (sensor_msgs/Image 32FC1, °C por píxel, frame thermal_optical_frame)
```

- El sensor térmico de Gazebo Harmonic **hace fallar al servidor si lleva `<noise>`**
  (`ThermalCameraSensor::SetScene`, verificado). Por eso el ruido lo añade el nodo
  `thermal_camera_sim` del paquete nuevo `usar_sim_sensors` (Capa 2), que solo transforma la
  imagen: no lee la pose ni el ground truth. El paquete estaba previsto para la Fase 10; se
  adelantó para este nodo.
- **Formato de `/thermal/image` para el robot real:** `32FC1` en °C, 32×24, a 4 Hz, frame
  `thermal_optical_frame`, sin `camera_info` (los intrínsecos salen del campo de visión en el
  YAML del detector). Es lo que debe publicar el ESP32 con el MLX90640: así el detector no
  distingue la simulación del robot.
- Medido en Gazebo: el ambiente sale exacto (293.15 K) y un objeto a 307.15 K se lee a
  308.40 K (+1.25 K), dentro de la exactitud del MLX90640 (±1.5 K).

## 3. Detector `thermal` (`usar_detection/thermal_detector.py`)

1. **Píxeles calientes:** al menos 28 °C y 5 °C por encima del fondo (mediana de la imagen).
2. **Manchas:** píxeles calientes conectados; se descartan las de menos de 2 píxeles y las que
   tienen algún píxel por encima de 40 °C. El filtro va por mancha: los píxeles del borde de un
   objeto a 60 °C, cubiertos solo en parte, promedian ≈ 34 °C y parecerían una persona
   (encontrado en las pruebas).
3. **Rayos:** una dirección por columna de la mancha.
4. **Cota por el punto de apoyo:** la víctima apoya en el suelo; el píxel caliente más bajo
   acota la distancia (d ≤ h / tan(depresión de su borde superior)). Con 1.9° por píxel solo es
   útil hasta ≈ 2 m.
5. **Triangulación** (`bearing_evidence.py`): cada rayo se recorta con el `/map` del robot
   (`map_filter.ray_extents`: hasta el primer obstáculo + 0.4 m, la primera celda desconocida o
   su distancia máxima) y marca todas las celdas que cruza, anotando el sector de su dirección
   (12 sectores de 30°). Solo es evidencia una celda cruzada desde **al menos 3 sectores
   distintos**: la víctima queda en la intersección de los rayos tomados al moverse el robot;
   un punto mal ubicado cambia de sitio con cada punto de vista. Girar en el sitio mirando la
   misma mancha no basta (siempre el mismo sector).

**Por qué triangular.** La primera versión tomaba como distancia la primera celda ocupada del
`/map` en la dirección del rayo (el LiDAR, con su plano a 0.18 m, ve el torso y la cabeza). En
la misión encontró las 5 víctimas pero con **14 falsos positivos**, todos a 0.9–3.9 m de una
víctima real: rayos que pasan por encima de las piernas (más bajas que el LiDAR) o por el borde
del cuerpo y terminan en el escombro de detrás. Con 4 imágenes por segundo, 5 observaciones del
mismo punto erróneo se juntan en poco más de un segundo.

Cada detector tiene su propio mapa de evidencia (sin filtro de forma para el térmico) y el
registro común une las confirmaciones: la etiqueta en RViz es, por ejemplo, "Víctima 3 (100 %,
forma+calor)". En RViz, el panel **Termica (deteccion)** muestra la imagen térmica en falso color
con el contorno de las manchas aceptadas y su temperatura máxima.

## 4. Métricas por modalidad

El detector publica también las confirmaciones de **cada detector por separado** en
`/victims/detections/<detector>` (PoseArray). El evaluador calcula con ellas, además del
resultado fusionado, lo que habría encontrado cada modalidad sola: bloque `by_modality` del JSON
(víctimas detectadas, falsos positivos, error de localización por detector).

## 5. Resultados

Semilla 42 `dusty` (víctimas grises, 10 escombros bajos, sin escombros tibios), 15 min,
`use_thermal:=true` (detectores color, forma y calor a la vez). "Solo calor" es lo que confirmó
el detector térmico por sí solo (`by_modality`).

| Versión del detector térmico | Solo calor: víctimas | Solo calor: falsos positivos | Solo forma | Fusionado (víctimas / FP) |
|---|---|---|---|---|
| Primera celda ocupada | 5 / 5 | **14** | 5 / 5, 0 FP | 5 / 5, 11 FP |
| **Triangulación** (actual) | 3 / 5 | **3** | 4 / 5, 0 FP | 4 / 5, 1 FP |

- La triangulación baja los falsos positivos térmicos de 14 a 3, a cambio de confirmar menos
  víctimas (hacen falta vistas desde 3 direcciones). El error de localización del calor es
  0.33 m, como el de la forma.
- El falso positivo que queda en el resultado fusionado (3.48, −5.69) está entre las víctimas 1
  y 3, separadas 1.3 m: los rayos de dos fuentes cercanas se cruzan también donde no hay
  ninguna ("intersecciones fantasma", el problema clásico de la triangulación con varias
  fuentes).
- En esta misión la forma encontró 4 / 5 (la víctima 2, en el oeste, no llegó a confirmarse):
  la simulación no es determinista y el recorrido cambia entre misiones.
- Por ahora el registro común acepta como víctima lo que confirme **cualquier** detector, así
  que un falso positivo del calor pasa al resultado. La regla de fusión (p. ej. exigir dos
  modalidades, o pesar la confianza de cada una) es la Fase 13; con `by_modality` cada misión ya
  da los datos de cada sensor por separado.

## 6. Limitaciones

- El calor solo no ubica bien varias víctimas cercanas (intersecciones fantasma) y necesita
  verlas desde varias direcciones: con una sola pasada del robot puede no confirmarlas.
- Los rayos se cortan en el primer obstáculo del `/map`: una víctima detrás de un escombro bajo
  que el LiDAR sí ve (más alto que 0.18 m) puede quedar fuera del alcance de los rayos.
- En la simulación las víctimas tienen una temperatura uniforme y el resto del mundo está a
  temperatura ambiente; no hay reflejos, sol ni gradientes. Los escombros tibios
  (`n_warm_debris`) son la única fuente de falsos positivos térmicos.
- El campo de visión vertical simulado (≈ 42°) es mayor que el real (35°).
