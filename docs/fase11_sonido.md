# Fase 11 — Signo de vida "voz / golpes" con el arreglo de micrófonos (ReSpeaker)

El robot oye a las víctimas que piden ayuda o golpean los escombros con un **ReSpeaker USB Mic
Array v2.0** (4 micrófonos, sensor 9 de la tabla de CLAUDE.md, fase 2: `use_sound:=true`). Como la
cámara térmica, da una **dirección** pero no una distancia; a diferencia de ella, el sonido
**atraviesa los escombros** y solo se oye bien **con el robot quieto**.

**Uso.**
```
ros2 launch usar_bringup mission.launch.py seed:=42 world_params:=dusty use_sound:=true
```
Se puede combinar con `use_thermal:=true` y `use_co2:=true`. Tras compilar, ejecutar
`source install/setup.bash` en las terminales ya abiertas (mensaje nuevo en `usar_msgs`).

## 1. Qué entrega el sensor real y qué se simula

El procesador del ReSpeaker no solo entrega audio: estima la **dirección de llegada** (DOA) del
sonido más fuerte y si es **voz** (VAD). Eso es lo que usa el robot, y lo que se simula; simular
el audio crudo (16 kHz × 4 canales) sería muy costoso y no cambiaría las decisiones del robot.

**Mensaje** `usar_msgs/SoundDirection` en `/sound/doa` (10 Hz, frame `mic_array_link`, en la
posición de `hardware/CLAUDE.md`): azimut (rad, en el frame del arreglo), nivel total (dB SPL),
`voice_activity` y confianza (0–1, según la relación señal/ruido). El driver del ReSpeaker real
debe publicar este mismo mensaje.

## 2. Simulador `sound_sim` (`usar_sim_sensors/sound_model.py`)

Excepción del ground truth (CLAUDE.md): conoce las víctimas y los escombros y la pose real.

| Parte | Valor | Motivo |
|---|---|---|
| Fuentes | Ráfagas de 2 s cada 10–20 s (sorteado), 65 dB SPL a 1 m | Pedir ayuda, golpear. Reproducible por semilla. |
| Víctimas que hacen ruido | `vocal_fraction: 1.0` (todas) | Para evaluar el sensor; la Fase 13 probará fracciones realistas (inconscientes). |
| Propagación | −20·log10 r y −8 dB por escombro en la línea recta | Divergencia esférica y transmisión a través de bloques. |
| Ruido | Ambiente 35 dB; motores 55 dB con el robot en marcha | En marcha los sonidos débiles quedan tapados. |
| Detección de voz | SNR ≥ 6 dB | |
| Error de dirección | σ = 10° con SNR 10 dB (baja con más SNR), entre 3° y 40° | |
| Ecos | Probabilidad 5 % + 10 % por escombro en medio (máx. 60 %): dirección falsa | Reflexiones en las ruinas. |
| Varias fuentes | Una sola dirección: la del sonido más fuerte | Como el procesador del ReSpeaker. |

## 3. Detector `sound` (`usar_detection/sound_detector.py`)

1. **Solo con el robot quieto:** usa una lectura si el arreglo no se ha movido más de 3 cm ni
   girado más de 0.05 rad en el último segundo (con la pose del propio robot, por TF).
2. **Rayo** en el frame del mapa desde la posición del arreglo, si hay voz con confianza ≥ 0.1.
3. **Triangulación** con la misma rejilla de intersección que la cámara térmica
   (`bearing_evidence`), pero los rayos **no se cortan en los obstáculos ni en lo desconocido**
   del `/map` (el sonido atraviesa los escombros, y el interior de un escombro nunca se
   cartografía): llegan al borde del mapa o a 10 m. Celdas de 0.3 m (la dirección es menos
   precisa que la térmica) y al menos 3 sectores de 30° distintos.
4. Mapa de evidencia propio con celdas de 0.3 m; σ = 0.7 m en el registro común.
5. **Sin "triangulación" desde un solo sitio:** cerca del origen de un rayo una celda abarca
   más de un sector, y rayos en direcciones distintas desde el mismo punto la cruzan todos. En
   la primera misión eso produjo 2 falsos positivos exactamente donde el robot pasó mucho tiempo
   quieto (el spawn y un hueco). Ahora `bearing_evidence` solo marca celdas a más de
   2 · celda · sectores / 2π del origen (≈ 1.1 m para el sonido, ≈ 0.6 m para la térmica).

## 4. Paradas de escucha (`frontier_explorer`)

Con `use_sound:=true`, cada **45 s** de navegación el robot cancela la meta, se queda quieto
**5 s** y la reenvía (`listen_enabled`, `listen_period`, `listen_duration` en `explorer.yaml`).
Son las ocasiones en que el detector puede escuchar; la Fase 12 (radar Doppler, que solo ve la
respiración con el robot quieto) las reutiliza. En el log aparecen junto a errores de Nav2
("Failed to get result ... in node halt!") que solo indican la cancelación de la meta.

## 5. Cambio en la exploración: encierros repetidos

En la primera misión el robot quedó encerrado 3 veces en el mismo hueco (≈ (1, −2.4)): evitar
las metas a menos de 1.2 m no impedía que la ruta hacia otra meta lo llevara de nuevo por allí.
Ahora, si queda encerrado cerca de un encierro anterior, el retroceso y la zona evitada crecen
(×2, ×3, ...).

## 6. Resultados

Semilla 42 `dusty`, 15 min, `use_sound:=true` (detectores color, forma y sonido).

| Misión | Solo sonido | Solo forma | Fusionado | Error medio | Cobertura | Retrocesos | Paradas |
|---|---|---|---|---|---|---|---|
| 1. Primera versión | 2/5, 2 FP (en sitios donde el robot estuvo quieto) | 4/5, 0 FP | 4/5, 2 FP | 0.26 m | 70 % | 7 (3 en el mismo hueco) | 14 |
| 2. + sin triangulación desde un solo sitio + encierros escalonados | 0/5, 1 FP | 2/5, 0 FP | 2/5, 1 FP | 0.91 m | 74 % | 4 (distintos) | 17 |

- **La misión 2 no sirve para juzgar el sonido:** el mapa de SLAM quedó girado y desplazado
  0.5–0.8 m respecto al escenario real (se ve en la imagen comparativa), así que todas las
  posiciones salieron corridas: la forma, que suele ubicar con 0.25 m, falló por 0.9 m, y los
  rayos de sonido no coinciden entre paradas.
- En la misión 1 el sonido encontró 2 víctimas (error 0.36 m) y sus 2 falsos positivos eran el
  defecto de la triangulación desde un solo sitio, ya corregido.
- La cobertura con sonido (70–74 %) es menor que en las misiones de la Fase 10 (99–100 %):
  las paradas cuestan ≈ 10 % del tiempo, y en estas dos misiones hubo más encierros.
- Con una misión por versión no se puede separar el efecto de los cambios de la variabilidad
  de la simulación (SLAM, recorrido).

### 6.1 Evaluación con varias semillas (con y sin sonido)

Semillas 1, 2 y 3 `dusty`, 15 min, headless. Sin sonido: detectores color y forma. Con sonido:
además micrófonos, detector `sound` y paradas de escucha. Resultados en
`results/fase11_eval/{sin_sonido,con_sonido}/summary.md`.

| Semilla | Sin sonido: cobertura / víctimas / FP / error | Con sonido: cobertura / víctimas / FP / error | Solo sonido (con sonido) |
|---|---|---|---|
| 1 | 99.5 % / 5/5 / 0 / 0.41 m | 98.7 % / 3/5 / 1 / 0.32 m | 1/5, 1 FP |
| 2 | 97.9 % / 5/5 / 0 / 0.35 m | 85.7 % / 5/5 / 3 / 0.19 m | 2/5, 3 FP |
| 3 | 99.7 % / 5/5 / 0 / 0.24 m | 100 % / 5/5 / 2 / 0.25 m | 2/5, 2 FP |
| **media** | **99.0 % / 5.0 / 0 / 0.33 m** | **94.8 % / 4.3 / 2.0 / 0.25 m** | **1.7/5, 2.0 FP** |

- **El sonido no aportó ninguna víctima que la forma no encontrara**: todas sus detecciones
  correctas coincidieron con una de la forma (se fusionaron, y bajaron algo el error medio).
- **Todos los falsos positivos son del sonido** (6 en 3 misiones), a 1.3–5.6 m de la víctima
  más cercana y casi todos con pocas observaciones (5–29). Son "fantasmas" de triangulación:
  el arreglo da una sola dirección, la del sonido más fuerte, así que en paradas distintas los
  rayos apuntan a víctimas distintas (o a ecos) y se cruzan en sitios donde no hay nadie.
- Las 2 víctimas perdidas en la semilla 1 no son efecto del sonido: el robot no entró a la zona
  inferior izquierda (el LiDAR la cartografió desde lejos y el explorador la dio por explorada;
  la cámara necesita acercarse). El sonido tampoco las oyó: lejos y detrás de escombros.
- La semilla 2 perdió cobertura (85.7 %) por 5 encierros, uno repetido.

**Conclusión:** como signo de vida independiente que confirma víctimas, el sonido simulado no
mejora la búsqueda: no agrega detecciones y suma ≈ 2 falsos positivos por misión.

### 6.2 Rediseño: el sonido guía, la cámara confirma

Como en una búsqueda real (se escucha, y luego se va a mirar), el sonido pasó a ser un detector
**guía** (`sound.cue_only: true` en `detector_sound.yaml`):

1. **Registro de víctimas** (`VictimRegistry`): un grupo formado solo por detectores guía no es
   una víctima sino un **candidato**, publicado en `/victims/candidates`
   (`usar_msgs/VictimDetectionArray`) y en RViz como cilindro naranja "Candidato (sonido)".
   Se vuelve víctima (fuentes `human_shape+sound`) cuando un detector confirmador (forma, color,
   calor, CO₂) la ve a menos de `merge_distance`. Los candidatos no consumen números de víctima.
2. **Explorador** (`frontier_explorer`, parámetros `candidate_*` en `explorer.yaml`): antes de
   elegir otra frontera va a mirar el candidato pendiente más cercano. La meta es un punto libre
   a 0.8–2 m del candidato, con holgura, con línea de vista en el `/map` y lo más cerca posible
   del robot (`frontier_core.find_view_goal`), orientado hacia el candidato; al llegar se queda
   4 s quieto mirándolo. Cada candidato se visita una vez (radio 1 m), con un tope de 10 por
   misión; si se resuelve mientras el robot va, la meta se cancela.
3. **Métricas:** `/victims/detections` (fusionado) ya no incluye los candidatos. La modalidad
   `sound` del evaluador sigue midiendo las estimaciones del sonido solo: ahí un "falso
   positivo" es ahora una visita en vano, no una víctima reportada.

Sin detectores guía (misiones sin `use_sound`) no llegan candidatos y nada cambia.

### 6.3 Comparación de las tres versiones

Mismas semillas (1, 2, 3 `dusty`, 15 min). Resultados de "sonido guía" en
`results/fase11_eval/con_sonido_guia/`.

| Versión | Víctimas | Falsos positivos | Error medio | Cobertura |
|---|---|---|---|---|
| Sin sonido | 15/15 | 0 | 0.33 m | 99.0 % |
| Sonido confirma | 13/15 | 6 | 0.25 m | 94.8 % |
| **Sonido guía** | **12/15** | **0** | **0.20 m** | 93.6 % |

| Semilla (sonido guía) | Víctimas | FP | Cobertura | Candidatos visitados |
|---|---|---|---|---|
| 1 | 2/5 | 0 | 82.7 % | 5 |
| 2 | 5/5 | 0 | 98.3 % | 9 (1 resuelto en camino) |
| 3 | 5/5 | 0 | 99.8 % | 0 (el sonido no llegó a triangular) |

- **Los falsos positivos del sonido desaparecieron** (6 → 0): sus estimaciones erradas solo
  cuestan una visita en vano.
- **La semilla 1 no sirve para juzgar:** el mapa de SLAM se degradó (paredes duplicadas y puntos
  sueltos en la mitad superior, obstáculos corridos respecto al escenario real) y la cobertura
  cayó a 82.7 %. Es la segunda misión con sonido con deriva de SLAM (la otra es la misión 2 de
  la sección 6); en las misiones sin sonido de esta fase y en las de la Fase 10 no ocurrió.
  Con tan pocas misiones no se puede afirmar si las paradas influyen; queda como punto a
  investigar.
- En las semillas 2 y 3 la forma encontró las 5 víctimas igual que sin sonido: en este
  escenario (20 × 20 m, 15 min, cobertura ≈ 99 %) la cámara llega a todas, así que el sonido no
  tiene víctimas que añadir. Su utilidad esperable es en escenarios donde la cobertura no
  alcanza (más grandes, menos tiempo) o con víctimas que la cámara no ve.

## 7. Limitaciones

- La dirección es la del sonido más fuerte: una víctima cercana tapa a las lejanas mientras
  emite.
- La propagación es en línea recta con atenuación por bloque; no hay difracción ni reverberación
  más allá de la probabilidad de eco.
- Las paradas cuestan tiempo de exploración (5 s de cada 50).
