# Fase 12 — Signo de vida "respiración" con el radar Doppler HB100

El HB100 (10,525 GHz, onda continua) detecta movimiento dentro de su haz. El pecho de una
persona que respira (4–8 mm, 12–20 respiraciones/min) modula la fase del eco y la señal IF
oscila al ritmo de la respiración. No mide distancia ni dirección: es un detector **guía**,
como el sonido (Fase 11). Solo funciona con el robot quieto.

Decisiones aprobadas:
1. El ESP32 publica la señal muestreada y el computador hace el análisis espectral (un solo
   algoritmo en Python para simulación y robot real).
2. El radar es guía (`cue_only`): sus detecciones son candidatos y la cámara confirma.
3. Paradas más largas con el radar (se pidieron 10 s; quedaron en 11 s, ver sección 6.2).

## 1. Mensaje y tópico

`/radar/signal` (`usar_msgs/RadarSignal`, frame `radar_link`):

| Campo | Contenido |
|---|---|
| `header.stamp` | instante de la **última** muestra del bloque |
| `sample_rate` | 20 Hz |
| `samples` | tensión en el ADC respecto a VREF (V), de la más antigua a la más reciente |
| `saturated` | alguna muestra del bloque llegó al límite del ADC |

### Especificación para la sesión de hardware (firmware del ESP32)

El circuito y el muestreo ya existen (`hardware/CLAUDE.md`: GPIO34, 200 Hz, VREF 1,30 V).
Falta publicarlo por micro-ROS:

1. Cada muestra del ADC del radar (200 Hz): `v = radar_adc_volts() − RADAR_VREF`.
2. Promediar bloques de **10** muestras (filtro anti-alias y diezmado 200 → 20 Hz).
3. Acumular **20** valores promediados y publicar un `usar_msgs/RadarSignal` por segundo:
   `sample_rate = 20.0`, `frame_id = "radar_link"`, `stamp` = hora de la última muestra,
   `saturated = true` si alguna de las 200 muestras crudas quedó a menos de 2 cuentas de 0 o de
   4095.
4. QoS fiable (los bloques deben llegar todos para que la ventana de análisis sea continua).
   Carga: ≈ 100 B/s.
5. Actualizar `hardware/CLAUDE.md` (tópico del radar) y la tabla de tópicos del firmware.

`usar_msgs` debe compilarse en el agente micro-ROS (mensaje propio, igual que
`GasConcentration`).

## 2. Simulador `radar_sim` (`usar_sim_sensors/radar_model.py`)

Excepción de la Capa 2: lee `ground_truth.yaml` (víctimas, escombros) y la pose real; publica
solo la tensión medida.

- **Respiración** de cada víctima (`breathing_fraction`): d(t) = D·sin(2πft + ψ), D 4–8 mm,
  f 12–20/min, sorteados por semilla. Señal IF: A·cos(φ0 + 4π·d(t)/λ), λ = 2,85 cm (modulación
  de fase de ≈ 2 rad: la respiración y sus armónicos).
- **Amplitud** A = `ref_amplitude` (60 µV a 1 m en el eje) · (1/r)² · patrón de la antena
  (−3 dB por vía a ±40°, nada detrás) · 8 dB por escombro y por vía.
- **Robot en marcha**: ecos del escenario con Doppler 2·v/λ (v = avance + giro·0,5 m) y
  amplitud 150 µV; la velocidad varía linealmente dentro de cada bloque, así un arranque o un
  tirón barre todas las frecuencias intermedias. Al parar, la fase se congela: un escalón que
  los pasa-altos tardan segundos en descargar.
- **Circuito** (H7): ruido 3 µV referido a la entrada, 2 etapas de banda 0,11–43 Hz y 66,8 dB,
  saturación del ADC (−1,25/+1,10 V respecto a VREF), 0,8 mV por cuenta, promedio de 10
  muestras.

Alcance resultante con el robot quieto: ≈ 4 m con línea de vista, ≈ 2 m tras un escombro.

## 3. Detector `radar` (`usar_detection/radar_detector.py`)

1. **Quieto y asentado**: el radar debe llevar `settle_time + window + guard_time`
   (2,5 + 5 + 1 s) sin moverse más de 1 cm ni girar más de 0,01 rad (pose del propio robot).
2. **Ventana** de 5 s que termina 1 s antes de la última muestra (guarda).
3. Se resta un polinomio de grado 2 (cola del asentamiento) y se calcula el espectro (Hann).
4. **Respiración** si la densidad en 0,15–1 Hz supera en 10 dB a la de 2–8 Hz y su valor
   eficaz supera 3 mV.
5. Candidato en el eje de la antena a 1,5 m (sin distancia medida). El tracker exige 2
   ventanas.

## 4. Explorador y robot

- `explorer_radar.yaml` (con `use_radar:=true`): paradas de escucha activadas y de 11 s.
- `radar_link` en el xacro (posición de `hardware/CLAUDE.md`), solo con `use_radar:=true`.
- `use_radar` en todos los launch y `USE_RADAR=true` en `scripts/run_batch.sh`.

## 5. Pruebas

- `usar_sim_sensors/test/test_radar_model.py`: respiración reproducible, 1/r², haz, escombros,
  robot en marcha.
- `usar_detection/test/test_radar.py`: análisis espectral (respiración, ruido, cola del
  asentamiento), espera de asentamiento, tirón, arranque en el último bloque, config.
- `test_ground_truth_isolation.py`: `RadarSignal` es una medida permitida.

## 6. Resultados

### 6.1 Primera evaluación (semillas 1–3, escenario `dusty`, 900 s)

| Semilla | Víctimas | Falsos positivos | Cobertura | Candidatos del radar (distancia a la víctima más cercana) |
|---|---|---|---|---|
| 1 | 5/5 | 0 | 75.2 % | 0.68, 1.43, 1.46, 1.60 m |
| 2 | 4/5 | 0 | 97.5 % | 0.87, 0.78, 1.02, **7.25** m |
| 3 | 5/5 | 0 | 99.7 % | **4.71** m |

14/15 víctimas y ningún falso positivo confirmado. La víctima 5 de la semilla 1 la
confirmaron juntas la forma y el radar. Dos candidatos no tenían ninguna víctima en el haz:

- **Semilla 2:** un tirón de 0,02 rad (≈ 1°) 7 s antes, por debajo de la tolerancia de 0,05 rad,
  dejó un transitorio en los pasa-altos.
- **Semilla 3:** el robot empezó a girar en la última fracción de segundo de la ventana; el
  arranque barre el Doppler de los ecos desde 0 Hz y cruza la banda de la respiración.

### 6.2 Corrección

Quietud más estricta (1 cm, 0,01 rad) y 1 s de guarda al final de la ventana. Para conservar
2–3 ventanas por parada, el asentamiento bajó a 2,5 s (la cola restante queda 6 veces por
debajo del umbral) y las paradas subieron de 10 a 11 s. En el simulador, la velocidad ahora
varía linealmente dentro de cada bloque (antes se tomaba constante).

Escenarios sin víctima, 20 semillas cada uno (detecciones falsas):

| Escenario | Antes | Después |
|---|---|---|
| Parada normal | 0 | 0 |
| Tirón de 0,02 rad a mitad de parada | 3–5 por parada | 0 |
| Arranque de giro al final de la ventana | 1 por parada | 0 |
| Arrastre lento (0,001 rad/s) | 0 | 0 |

Con una víctima a 2 m siguen saliendo 2 ventanas positivas por parada.

### 6.3 Evaluación con la corrección (`results/radar_eval2/`)

| Semilla | Víctimas | Falsos positivos | Cobertura | Ventanas quieto / con respiración | Candidatos (distancia a la víctima más cercana) |
|---|---|---|---|---|---|
| 1 | 5/5 | 0 | 87.1 % | 70 / 4 | 2.23, 1.94 m |
| 2 | 3/5 | 0 | 96.8 % | 58 / 4 | 2.04, 2.19 m |
| 3 | 2/5 | 0 | 61.7 % | 43 / 4 | 2.18 m |

- **Ya no hay candidatos falsos**: los 5 están a ≈ 2 m de una víctima real (error esperable:
  el candidato va en el eje a 1,5 m y la víctima puede estar a un lado del haz o más lejos).
- **Ningún falso positivo** confirmado en las 6 misiones con radar.
- **Víctimas**: 10/15, igual que las misiones sin sensores guía de `deriva_slam.md` (10/15);
  la primera evaluación con radar dio 14/15. Con 3 semillas la variación entre misiones es
  mayor que el efecto del radar: no se puede afirmar que mejore la detección.
- **Cobertura** media 81.9 % (90.8 % en la primera evaluación, ≈ 99 % sin radar). Las paradas
  cuestan ≈ 165 s de 900 (15 × 11 s) y la semilla 3 se atascó en una zona con muchos escombros
  (13 metas fallidas). Se comprobó que las paradas no causan los fallos "sin progreso": las
  metas con parada fallan así en el 25–39 % de los casos y las metas sin sensores guía en el
  27 % (las paradas solo ocurren en metas largas, que son las difíciles).

Conclusión: el radar cumple como guía (candidatos cerca de víctimas reales, cero falsos
positivos), pero en este escenario (20 × 20 m, 15 min) la cámara ya encuentra casi todo y las
paradas largas restan cobertura. Su valor y la política de paradas (duración, cada cuánto, solo
donde hay escombros sin mirar) se medirán en la Fase 13 con más semillas y ablación.

## 7. Limitaciones

- La posición del candidato es el eje de la antena a 1,5 m: con víctimas a 3–4 m el error es
  de varios metros. Basta para que el explorador vaya a mirar, no para ubicar.
- El modelo de amplitud es simplificado (pecho como reflector puntual, patrón gaussiano, misma
  atenuación para todos los escombros). Los valores (`ref_amplitude`, `obstacle_attenuation_db`)
  deben contrastarse con el radar real cuando el firmware publique `/radar/signal`.
- No se modelan otras fuentes de movimiento (escombros que se asientan, otras personas,
  vibración del propio robot con los motores parados).
