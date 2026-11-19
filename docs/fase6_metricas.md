# Fase 6 — Métricas y experimento completo

Este documento describe el evaluador (`usar_metrics`), la misión completa (`mission.launch.py`)
y el lote de semillas (`scripts/run_batch.sh`).

## 1. Regla del ground truth

El nodo `evaluator` es el **único** proceso que lee `ground_truth.yaml`. No publica nada: solo
escucha los tópicos que el robot ya produce y escribe archivos al terminar. El robot (SLAM, Nav2,
explorador, detector) decide sin conocer el ground truth, y la evaluación no lo modifica.

```
/map ─────────────────┐
/odom ────────────────┤                     ground_truth.yaml
/victims/detections ──┼─► evaluator ◄───────── (área explorable, víctimas, escombros, spawn)
/exploration/status ──┘       │
                              └─► results/seed<N>_<fecha>.json / .csv / _map.pgm / _map.yaml
                                  / _comparison.png
```

## 2. Métricas

| Métrica | Cálculo |
|---|---|
| Cobertura C% | `A_explorada / A_explorable × 100`. `A_explorable` es el área del ground truth a la que puede llegar el centro del robot (flood-fill sobre los escombros inflados con el radio del robot). `A_explorada` son las celdas de esa área cuya celda del `/map` es libre (valor 0–25). |
| Tiempo de exploración | Tiempo simulado desde el primer estado activo del explorador hasta `finished:<motivo>`. |
| Distancia recorrida | Suma de los desplazamientos sucesivos de `/odom`. |
| Víctimas detectadas | Víctimas reales asociadas a una víctima confirmada por el detector. |
| Tasa de detección | `detectadas / víctimas reales × 100`. |
| Falsos positivos | Víctimas confirmadas sin víctima real asociada. |
| Precisión | `detectadas / confirmadas × 100`. |
| Error de localización | Distancia entre la posición estimada y la real de cada víctima detectada (media y máxima). |

**Frames.** slam_toolbox fija el frame `map` en la pose inicial del robot. El evaluador pasa el
mapa y las víctimas estimadas al frame del mundo con la pose del spawn del ground truth.

**Cobertura.** Solo se cuentan las celdas del área explorable. Las celdas libres que SLAM marque
fuera de ella (detrás de los muros, dentro de escombros o en los bordes inaccesibles junto a los
obstáculos) no cuentan, de modo que la cobertura nunca supera el 100 %. Las celdas pegadas a los
obstáculos, que el centro del robot no puede alcanzar, tampoco están en el denominador.

**Asociación de víctimas.** Uno a uno, minimizando la suma de distancias (algoritmo húngaro,
`scipy.optimize.linear_sum_assignment`). Una pareja solo cuenta si su distancia es menor o igual
que `association_threshold`. Una asociación voraz (la pareja más cercana primero) puede dejar sin
pareja una estimación válida cuando dos víctimas están cerca; la asociación óptima no.

## 3. Parámetros (`usar_metrics/config/evaluator.yaml`)

| Parámetro | Valor | Justificación |
|---|---|---|
| `association_threshold` | 1.0 m | Una víctima mide 1.7 × 0.6 m: 1 m acepta una estimación en cualquier punto del cuerpo, pero no la confunde con otra víctima. |
| `sample_period` | 10 s | Resolución suficiente para las curvas de cobertura de una misión de 15 min (90 muestras). |
| `free_threshold` / `occupied_threshold` | 25 / 65 | Los umbrales estándar de nav2_map_server (0.25 / 0.65). |
| `finish_delay` | 3 s | Espera las últimas detecciones (se publican a 5 Hz) tras el fin del explorador. |
| `max_mission_time` | 1200 s | Seguridad: cierra la misión si el explorador no termina (su límite es 900 s). |

## 4. Archivos de resultados

| Archivo | Contenido |
|---|---|
| `seed<N>_<fecha>.json` | Resumen: métricas, detalle por víctima (real, estimada, error), posiciones de los falsos positivos, parámetros del mundo y del evaluador. |
| `seed<N>_<fecha>.csv` | Evolución cada 10 s: tiempo, cobertura, distancia, víctimas confirmadas, detectadas y falsos positivos, estado del explorador. |
| `seed<N>_<fecha>_map.pgm` / `.yaml` | Mapa final en el formato de nav2_map_server (se puede cargar con `map_server`). |
| `seed<N>_<fecha>_comparison.png` | Mapa construido sobre los escombros reales, con las víctimas reales (X), estimadas (●) y la trayectoria. |
| `victims_seed<N>_<fecha>.csv` | Log del detector (Fase 5). |

## 5. Uso

Misión completa con ventanas (termina sola y cierra Gazebo y RViz):

```bash
ros2 launch usar_bringup mission.launch.py seed:=42
```

Sin ventanas, o prueba corta de 5 min:

```bash
ros2 launch usar_bringup mission.launch.py seed:=42 headless:=true
ros2 launch usar_bringup mission.launch.py seed:=42 quick:=true
```

Lote de semillas (sin ventanas, una tras otra) y tabla consolidada:

```bash
scripts/run_batch.sh 1 2 3 4 5
scripts/consolidate_results.py results/batch_<fecha>      # rehacer la tabla
```

`run_batch.sh` guarda todo en `results/batch_<fecha>/` y al final escribe `summary.csv` y
`summary.md`, con una fila por semilla y la media ± desviación estándar muestral de cada métrica.

## 6. Resultados

Lote de las semillas 1 a 5 (`scripts/run_batch.sh 1 2 3 4 5`, sin ventanas, 15 min de
exploración por misión como máximo, unos 15.5 min reales cada una):

| Semilla | Fin | Tiempo [s] | Cobertura [%] | Recorrido [m] | Detectadas | Falsos positivos | Error medio [m] |
|---|---|---|---|---|---|---|---|
| 1 | no_frontiers | 873.0 | 100.0 | 140.5 | 5 / 5 | 1 | 0.30 |
| 2 | timeout | 899.8 | 85.3 | 82.1 | 5 / 5 | 0 | 0.22 |
| 3 | timeout | 900.5 | 100.0 | 153.2 | 5 / 5 | 0 | 0.31 |
| 4 | timeout | 899.8 | 99.9 | 111.1 | 5 / 5 | 0 | 0.22 |
| 5 | timeout | 900.5 | 98.4 | 126.8 | 5 / 5 | 0 | 0.23 |
| **media ± σ** | | 894.7 ± 12.1 | 96.7 ± 6.4 | 122.7 ± 27.6 | 5.0 ± 0.0 | 0.2 ± 0.4 | 0.25 ± 0.04 |

- **Víctimas:** 25 de 25 detectadas (tasa del 100 %), un falso positivo en 5 misiones y error de
  localización medio de 0.25 m (máximo individual 0.62 m).
- **Cobertura:** cuatro misiones superan el 98 %. En la semilla 2 se agotó el tiempo con la zona
  sur sin recorrer (85.3 %). Las semillas 3 y 4 terminan por tiempo aunque cubren casi el 100 % del
  área explorable: quedan fronteras en zonas que el robot ve pero a las que no puede llegar (por
  ejemplo, huecos estrechos entre escombros), y el explorador las sigue intentando.
- **Escape por bloqueo** (añadido en esta fase, ver `fase4_exploracion.md`): se activó entre 3 y 8
  veces por misión, además de 2 a 10 escapes de la zona inscrita.

**Falso positivo de la semilla 1: víctima partida en dos.** La estimación falsa está sobre un
extremo del cuerpo de `victim_00`, y la estimación asociada a esa víctima está en el otro extremo
(error 0.62 m). El robot vio los dos extremos del cuerpo, pero no la parte central (tapada por
escombros o fuera de la vista), y el mapa de evidencia no los une porque los separan más de
`join_distance` (0.2 m). Pasó lo mismo en una prueba de 5 min de la semilla 42 con `victim_01`.
Es una limitación del detector de la Fase 5 que las métricas ponen de manifiesto.

La simulación no es determinista: repetir una semilla da resultados parecidos pero no idénticos.
