# Fase 10 — Mensajes propios, excepción del ground truth y signo de vida "CO₂"

Esta fase añade el tercer signo de vida: el **CO₂ exhalado** por las víctimas, medido con un
sensor puntual Figaro TGS4161 (sensor 8 de la tabla de CLAUDE.md, fase 2: `use_co2:=true`).
Gazebo no simula gases, así que la medida la genera un simulador de la Capa 2; para eso se
formaliza la excepción a la regla del ground truth. También se crean los mensajes propios del
proyecto.

**Uso.**
```
ros2 launch usar_bringup mission.launch.py seed:=42 world_params:=dusty use_co2:=true
```
Se puede combinar con `use_thermal:=true`. Esta fase añade el paquete `usar_msgs`: tras compilar,
ejecutar `source install/setup.bash` en las terminales ya abiertas.

## 1. Paquete `usar_msgs`

| Mensaje | Uso |
|---|---|
| `VictimDetection` | Una víctima: número, posición, confianza, observaciones y **detectores que la confirmaron** (`sources`). |
| `VictimDetectionArray` | `/victims/detections` (fusionadas) y `/victims/detections/<detector>` (cada detector por separado). Antes era `geometry_msgs/PoseArray`, que no decía qué modalidad detectó. |
| `GasConcentration` | `/co2/concentration`: gas, concentración (ppm) y varianza. |

## 2. Excepción del ground truth (CLAUDE.md y `docs/auditoria_ground_truth.md`, sección 4)

Los simuladores de sensores de `usar_sim_sensors` pueden leer `ground_truth.yaml` y la pose real
del robot (`/sim/ground_truth/odom`, plugin `OdometryPublisher` de Gazebo, solo con
`sim_pose:=true`), pero **solo publican medidas** (ppm, °C). Las pruebas automáticas comprueban
que solo crean publicadores de mensajes de medida, que la ruta del ground truth solo llega a
ellos y que ningún otro paquete del robot nombra un tópico `/sim/*`. Se verificó que el plugin
publica la pose en el frame del mundo (un modelo en (3, 2) con yaw 0.5 se publica así).

## 3. Simulador `co2_sim` (`usar_sim_sensors/co2_model.py`)

| Parte del modelo | Valor | Motivo |
|---|---|---|
| Ambiente | 400 ppm | Aire exterior; línea base con la que el firmware calibra el sensor (`CO2_REF_PPM`). |
| Pluma de cada víctima | exceso = 2000 ppm · exp(−r / 1 m) | Solución estacionaria de difusión con pérdidas; zona alta junto a la cabeza en un hueco semicerrado. |
| Turbulencia | exceso × (1 + 0.3·n(t)), n de Ornstein-Uhlenbeck, τ = 5 s | Las corrientes de aire hacen fluctuar la pluma. |
| Respuesta del TGS4161 | primer orden en log(ppm), τ = 20 s | Electrolito sólido: FEM ∝ log(ppm), respuesta lenta. |
| Ruido | 2 % de la lectura (en log) | |
| Rango | 350–10 000 ppm, 2 Hz | |

La posición del sensor es la pose real del robot compuesta con `co2_link` (posición de
`hardware/CLAUDE.md`). **Formato para el ESP32:** `usar_msgs/GasConcentration` en
`/co2/concentration`, `gas: "CO2"`, ppm, frame `co2_link`.

## 4. Detector `co2` (`usar_detection/co2_detector.py`)

El sensor solo da un número en el sitio del robot: la fuente se deduce del recorrido.

1. **Deshacer el retardo.** Con una ventana de 4 s se ajusta una recta a log(lectura) y se estima
   la concentración real como exp(log c + τ · pendiente) (τ = 20 s, dato del fabricante). Una
   lectura va ≈ 10 s por detrás de la realidad; la estimación no, a cambio de amplificar el ruido
   (≈ ±12 % por estimación).
2. **Mapa de concentración** en celdas de 0.25 m; línea base = mediana de las celdas.
3. **Zonas con exceso** de más de 250 ppm; las que están a menos de 2 m son la misma fuente.
4. **Ajuste de la fuente** con el modelo exceso = A · exp(−|p − q| / 1 m) por mínimos cuadrados,
   buscando q a menos de 2 m de la celda máxima: la fuente puede estar a un lado del camino.
5. **Mapa de evidencia propio** (celdas de 0.25 m, une estimaciones a menos de 1 m): la fuente
   estimada deriva ±1 m entre actualizaciones y su posición final es el promedio.

El detector usa el mismo modelo de dispersión que el simulador (forma exponencial, 1 m), como si
se hubiera calibrado; con una pluma real esa suposición sería la principal fuente de error.

## 5. Cambio en la exploración: retroceso por la propia huella

En la segunda misión con CO₂ el robot quedó **encerrado 12 de los 15 minutos** en un hueco junto
a la víctima 2 (seed 42): una caja alta a 0.48 m al norte, la cabeza de la víctima a 0.56 m al
suroeste y otra caja alta al este. Entró, pero desde dentro el costmap veía cerradas todas las
salidas: el planificador no encontraba ruta a ninguna meta ("no valid path found"), el
controlador veía "collision ahead" (503 veces) y los 54 escapes (giros y avances de 15–25 cm,
5 de último recurso) no lo sacaban. Recorrió 34 m (cobertura 44 %).

**Solución** (`frontier_explorer`, `frontier_core.Breadcrumbs`):
- El explorador guarda la **huella** del robot (un punto cada 10 cm, posición de SLAM).
- Si en **60 s** no se aleja **1 m** del mismo sitio, el robot está encerrado: cancela la meta y
  **vuelve por su huella** 2 m (y hasta salir de ese radio), con persecución pura, marcha atrás
  si el camino queda detrás (no necesita girar en el hueco). El camino por el que entró ya cupo
  físicamente. Va por el `collision_monitor` (sigue protegido); si este lo frena 8 s, abandona.
- Desde entonces descarta las metas a menos de 1.2 m de ese sitio (para no volver a entrar).

Verificación (seed 42 `dusty` + CO₂): el robot quedó encerrado una vez, volvió 2.1 m por su
huella, salió del hueco y siguió: **cobertura 100 %, 134 m** (antes 44 %, 34 m).

## 6. Cambio en el registro común de víctimas

En esa misma misión la forma confirmó las víctimas 1 y 3 (a 1.5 m una de otra) por separado,
pero una estimación de CO₂ entre ambas hizo que el registro las uniera en una sola: 4 / 5
fusionadas con 5 / 5 de la forma. Además, la posición fusionada se ponderaba por el número de
observaciones, y el CO₂ (2 lecturas por segundo) acumulaba miles. Ahora:
- la posición fusionada se pondera por la **precisión** de cada detector (1/σ²; σ = 0.3 m
  forma y color, 0.4 m calor, 1.0 m CO₂);
- dos confirmaciones del **mismo detector** nunca se unen: la estimación de CO₂ se asocia solo
  a la víctima más cercana.

## 7. Resultados

Semilla 42 `dusty` (víctimas grises, 10 escombros bajos), 15 min, `use_co2:=true` (detectores
color, forma y CO₂). "Solo X" es lo que confirmó cada detector por sí solo (`by_modality`).

| Misión | Solo CO₂ | Solo forma | Fusionado | Error medio | Cobertura | Recorrido |
|---|---|---|---|---|---|---|
| 1. Primera versión del detector de CO₂ | 4/5, **3 FP** | 5/5, 0 FP | 5/5, 2 FP | 0.27 m | 99.9 % | 118 m |
| 2. CO₂ agrupando la deriva (robot encerrado 12 min) | 1/5, 0 FP | 3/5, 0 FP | 3/5, 0 FP | 0.38 m | **44 %** | **34 m** |
| 3. + retroceso por la huella | 4/5, 3 FP | 5/5, 0 FP | 4/5, 3 FP | 0.45 m | 100 % | 134 m |
| 4. + registro ponderado por precisión | 3/5, **0 FP** | 4/5, 0 FP | 4/5, **0 FP** | **0.17 m** | 99 % | 120 m |

- **Encierros:** con el retroceso por la huella la cobertura volvió a 99–100 % (misiones 3 y 4).
  En la misión 4 el robot quedó encerrado 4 veces y las 4 salió del hueco.
- **CO₂ solo:** encuentra 3–4 de 5 víctimas. En la misión 1 los 3 falsos positivos eran la misma
  víctima repetida (la fuente estimada derivaba ±1 m); en la 3, estimaciones a ≈ 2.5 m de una
  víctima real. Ubica con error del orden de 0.5 m cuando acierta.
- **Fusión:** en la misión 3 una estimación de CO₂ entre las víctimas 1 y 3 (a 1.5 m) unió en el
  registro dos víctimas que la forma veía por separado; con el registro ponderado por precisión
  (misión 4) se mantienen separadas y el error medio baja de 0.45 a 0.17 m.
- En la misión 4 ningún detector confirmó la víctima 2 (junto al hueco donde el robot quedó
  encerrado; después evita las metas a menos de 1.2 m de ese sitio). La simulación no es
  determinista: en la misión 3 la forma sí la encontró.

## 8. Limitaciones

- La pluma simulada es isótropa y no la bloquean los escombros; el viento real la desplaza y la
  fuente estimada quedaría corrida en la dirección del viento.
- Dos víctimas a menos de 2 m se ven como una sola fuente.
- El sensor es lento: el robot tiene que pasar cerca y despacio, o volver a pasar.
- Tras un retroceso el robot evita las metas cerca del hueco: si una víctima está justo ahí,
  solo la verá de lejos.
