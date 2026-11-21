# Guion del video explicativo de findbot (máximo 3:00)

**Objetivo:** que se vea la simulación funcionando, no diapositivas.
**Semilla para todas las tomas:** `seed:=1`.
**Narración:** 328 palabras, unos 2:31 de voz a 130 palabras por minuto, dentro de 3:00 de video.

## Antes de grabar: qué funciona hoy y qué no

Revisión del repositorio hecha el 21 de noviembre de 2026. Todo lo marcado "Sí" tiene evidencia en
el repositorio; nada de lo marcado [PENDIENTE] aparece en el video como si funcionara.

| Elemento | ¿Se puede grabar hoy? | Evidencia |
|---|---|---|
| Generación procedural (misma semilla = mismo mundo) | Sí | Prueba `usar_worldgen/test/test_reproducibility.py::test_same_seed_produces_identical_files`; mundos en `usar_ws/worlds/` |
| Gazebo con el robot explorando | Sí, con la precaución 1 | Misiones completas de hoy (`results/fase11_eval/sin_sonido/`): semillas 1, 2 y 3 con 5/5 víctimas y 98–99,7 % de cobertura |
| Mapa en RViz, fronteras y víctimas con esfera roja | Sí | `usar_bringup/rviz/exploration.rviz` (displays `/map`, `/exploration/frontiers`, `/victims/markers`) |
| Métricas en `results/` | Sí | Lote de semillas 1–5: `results/batch_20261119_205642/summary.md` (25/25 víctimas) |
| Diagrama de bloques de la capa física | Sí | `hardware/docs/figuras/r4_bloques.png` |
| Esquemático de KiCad | Sí | `hardware/kicad/findbot.kicad_pro` (ERC sin errores) |
| Gráficas de ngspice | Sí | `hardware/docs/figuras/` (72 criterios, 0 fallas) |
| Wokwi: firmware real, señal de motores y conteo de encoders | Sí, con la precaución 2 | Escenario de 111 pasos superado (`hardware/docs/evidencia/`) |

**Precauciones**

1. La sesión de simulación tiene cambios de la Fase 11 (sonido) sin commit. Las misiones de hoy
   salieron bien, pero conviene grabar después de su commit, o hacer antes una prueba corta con
   `quick:=true` para confirmar que se abren Gazebo y RViz.
2. La cuota gratuita de pruebas automáticas de Wokwi (`wokwi-cli`) está agotada este mes. No sé si
   la extensión de VS Code usa la misma cuota: probarla antes del día de grabación.

---

## Guion por escenas

### Escena 1 — Gancho (0:00–0:15, 15 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:00–0:15 | Gazebo a ras de suelo, cámara lenta entre escombros grises; termina sobre una víctima naranja acostada entre dos bloques. Sin RViz. | "Después de un sismo, entrar a los escombros es peligroso y nadie sabe cómo quedó el lugar. Cada minuto cuenta para encontrar a las personas atrapadas." | Buscar sobrevivientes entre escombros | `ros2 launch usar_bringup sim.launch.py seed:=1` (en el contenedor). En Gazebo: clic derecho sobre una víctima → *Move to* y mover la cámara con el ratón (probar antes los controles de órbita). Grabar 30 s de movimiento lento y usar 15 s. |

*Duración estimada:* 26 palabras, unos 12 s de voz.

### Escena 2 — La solución (0:15–0:35, 20 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:15–0:35 | Pantalla dividida: Gazebo a la izquierda, vista superior del mundo; RViz a la derecha, casi todo gris al arrancar la misión. Los cuatro rótulos aparecen uno por uno sobre la toma. | "Proponemos un robot que entra solo, arma su propio mapa y busca víctimas. Antes de construirlo, lo probamos en un entorno virtual de cuatro capas: escenario, física, navegación y misión de búsqueda." | Escenario · Física · Navegación · Búsqueda | Primeros 30 s de la grabación de la escena 4 (`mission.launch.py seed:=1`). Los rótulos se agregan en la edición. |

*Duración estimada:* 32 palabras, unos 15 s de voz.

### Escena 3 — Generación procedural (0:35–1:00, 25 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:35–0:45 | Terminal a pantalla completa con fuente grande: se genera la semilla 1 dos veces y `md5sum` muestra la misma huella para los dos `world.sdf`. | "El escenario no se dibuja a mano: un programa lo genera a partir de un número, la semilla. Con la misma semilla sale exactamente el mismo mundo," | Misma semilla = mismo mundo | En el contenedor: `ros2 run usar_worldgen generate --seed 1 --out /tmp/demo_a`, luego `ros2 run usar_worldgen generate --seed 1 --out /tmp/demo_b` y `md5sum /tmp/demo_a/world.sdf /tmp/demo_b/world.sdf`. Resaltar en la edición las dos huellas iguales. |
| 0:45–1:00 | Vista superior de Gazebo (semilla 1); corte rápido a la vista superior de la semilla 2, distinta. | "con cuarenta escombros y cinco víctimas, todas alcanzables. Así cada experimento se puede repetir." | 40 escombros · 5 víctimas | Gazebo de la escena 1 con cámara cenital (complemento *View Angle* → vista superior). Para la semilla 2: `ros2 run usar_worldgen plot --world worlds/seed2` y abrir `worlds/seed2/top_view.png`. |

*Duración estimada:* 41 palabras, unos 19 s de voz.

### Escena 4 — El robot en acción (1:00–1:50, 50 s, parte central)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 1:00–1:15 | Pantalla dividida, Gazebo a la izquierda y RViz a la derecha. El robot sale del centro; en RViz el mapa crece alrededor de los puntos del láser. | "Ahora el robot está solo. A la izquierda, el mundo simulado; a la derecha, lo que el robot sabe. Con su láser dibuja el mapa: blanco es libre, negro es obstáculo y gris, lo que aún no conoce." | Exploración autónoma (×20) | `ros2 launch usar_bringup mission.launch.py seed:=1`. Grabar la misión completa (hasta que se cierre sola, unos 15 min de tiempo simulado) y acelerar unas ×20. Calcular el factor en la edición: minutos grabados × 60 / 45 s. |
| 1:15–1:30 | Acercamiento a RViz: los marcadores de frontera en el borde gris/blanco y la línea del plan hacia una de ellas. | "El borde entre lo conocido y lo desconocido se llama frontera, y el robot elige la que promete más información por menos camino." | Fronteras = borde de lo desconocido | Misma grabación, recortada a RViz con un acercamiento suave en la edición. |
| 1:30–1:50 | Momento en que aparece la primera esfera roja con su etiqueta "Víctima 1"; cuadro pequeño con la vista de la cámara del robot. | "Mientras avanza, su cámara busca formas humanas. Cuando confirma una víctima, la marca con una esfera roja. Y una regla clave: el robot nunca sabe dónde están las víctimas; solo usa sus sensores." | Víctima confirmada | Misma grabación. El minuto exacto de cada confirmación está en `results/victims_seed1_<fecha>.csv`: usarlo para ubicar el corte. En este tramo, reducir la aceleración a ×4. |

*Duración estimada:* 94 palabras, unos 43 s de voz.

### Escena 5 — Resultados (1:50–2:10, 20 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 1:50–2:00 | Imagen de comparación de la misión grabada: mapa construido sobre los escombros reales, víctimas reales y estimadas, trayectoria. | "Al final, un evaluador independiente compara lo encontrado con la realidad." | Evaluación contra la realidad | Archivo que la misión de la escena 4 genera sola: `results/seed1_<fecha>_comparison.png`. |
| 2:00–2:10 | Tabla del lote de cinco semillas con la fila "media ± σ" resaltada. | "En cinco escenarios distintos, el robot encontró las veinticinco víctimas, exploró en promedio el noventa y siete por ciento del área y las ubicó con un error de veinticinco centímetros." | 25/25 víctimas · 97 % · 25 cm | Abrir `results/batch_20261119_205642/summary.md` en VS Code con vista previa de Markdown (Ctrl+Shift+V). |

*Duración estimada:* 41 palabras, unos 19 s de voz. Las cifras de la narración son las del lote
de semillas 1–5 (96,7 ± 6,4 % de cobertura, 0,25 ± 0,04 m de error), no las de la misión grabada.

### Escena 6 — Del simulador al robot real (2:10–2:45, 35 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 2:10–2:18 | Diagrama de bloques de la capa física; recorrido lento de izquierda (batería) a derecha (portátil). | "El siguiente paso es el robot físico. Este es su diagrama de conexiones" | Capa física del robot | `xdg-open hardware/docs/figuras/r4_bloques.png`; recorrido animado en la edición. |
| 2:18–2:24 | KiCad con el esquemático; acercamiento al ESP32 y al módulo TB6612. | "y este, el circuito de la placa." | Esquemático en KiCad | `kicad hardware/kicad/findbot.kicad_pro` → abrir el esquemático, ocultar los paneles laterales y hacer zoom con la rueda del ratón. |
| 2:24–2:32 | Diagrama de Bode del amplificador del radar (la meseta de 67 dB entre 0,1 y 45 Hz). | "Cada etapa se simuló antes de fabricarla, como el amplificador del radar que detecta la respiración." | Simulación del circuito (ngspice) | `xdg-open hardware/docs/figuras/h2_radar_amp_bode.png`. Para regenerarla: `hardware/spice/run_spice.sh`. |
| 2:32–2:45 | VS Code con Wokwi: el ESP32 simulado. Se escribe `v 0.2 0` en el monitor serial y aparece `# motores ON pwm=20000 Hz`; luego se gira la perilla del encoder izquierdo, se escribe `?` y aparece `enc_l=…`. | "Y el mismo programa del microcontrolador corre en un simulador: ordenamos avanzar, se activa la señal de los motores y los encoders cuentan cada giro de rueda." | Firmware real en Wokwi | En VS Code: abrir `hardware/wokwi/diagram.json`, F1 → *Wokwi: Select Config File* → `hardware/wokwi/wokwi.toml`, y F1 → *Wokwi: Start Simulator*. Compilar antes: `cd hardware/firmware && pio run -e esp32_debug`. Ver [PENDIENTE] 1 y 2. |

*Duración estimada:* 63 palabras, unos 29 s de voz.

### Escena 7 — Cierre (2:45–3:00, 15 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 2:45–3:00 | Último cuadro de la misión: RViz con el mapa completo y las esferas rojas; se aleja lentamente y aparece el nombre del proyecto. | "En resumen: un entorno de prueba reproducible y un robot que explora y encuentra víctimas por sí solo. Lo que sigue es construirlo y llevarlo del simulador a los escombros reales." | findbot · UNAD · Ingeniería Electrónica | Los últimos segundos de la grabación de la escena 4, antes de que la misión se cierre sola. Alejamiento y título en la edición. |

*Duración estimada:* 31 palabras, unos 14 s de voz.

### Conteo

| Escena | Duración en pantalla | Palabras | Voz estimada (130 ppm) |
|---|---:|---:|---:|
| 1. Gancho | 15 s | 26 | 12 s |
| 2. La solución | 20 s | 32 | 15 s |
| 3. Generación procedural | 25 s | 41 | 19 s |
| 4. El robot en acción | 50 s | 94 | 43 s |
| 5. Resultados | 20 s | 41 | 19 s |
| 6. Del simulador al robot real | 35 s | 63 | 29 s |
| 7. Cierre | 15 s | 31 | 14 s |
| **Total** | **3:00** | **328** | **2:31** |

---

## [PENDIENTE]: lo que no está en el video como si funcionara

1. **Motores visibles en Wokwi.** Wokwi no tiene el TB6612 ni motores: el circuito muestra la
   señal de los motores solo como texto en el monitor serial y en el analizador lógico (que se ve
   al detener la simulación, con un programa externo). Por eso la narración dice "se activa la
   señal de los motores" y no "giran los motores". Si se quiere ver algo moverse, falta agregar
   LEDs en los pines de PWM del circuito de Wokwi.
2. **Cuota de Wokwi.** Ver la precaución 2.
3. **Robot físico.** No existe todavía: el video solo muestra simulación, y el cierre lo presenta
   como siguiente paso.
4. **Enlace micro-ROS de punta a punta.** La prueba de que un `/cmd_vel` de ROS 2 mueve los
   motores del ESP32 simulado quedó pendiente (falso positivo corregido en H7). No aparece en el
   video.
5. **Semilla 42.** No hay una misión completa validada con la semilla 42 en el escenario normal
   (solo una prueba de 5 min con 1 de 5 víctimas). Por eso el video usa la semilla 1.
6. **Signos de vida de la fase 2** (térmica, CO₂, sonido): existen en la simulación, pero su
   evaluación sigue en curso en la sesión de simulación. Se dejaron fuera del video.
7. **OBS Studio** no está instalado: `sudo apt install obs-studio`.

---

## Lista de tomas en el orden más práctico para grabarlas

1. **Wokwi en VS Code** (escena 6, 13 s). Primero, porque es lo único que puede fallar por la
   cuota; si falla, queda tiempo para decidir qué hacer.
2. **KiCad, diagrama de bloques y gráfica de Bode** (escena 6). Capturas estáticas, 2 min.
3. **Terminal con `generate` y `md5sum`** (escena 3), dentro del contenedor.
4. **Vistas superiores de las semillas 1 y 2** (escena 3).
5. **Gazebo a ras de suelo** (escena 1) con `sim.launch.py seed:=1`. Cerrar antes de la misión.
6. **Misión completa** con `mission.launch.py seed:=1`, grabando Gazebo y RViz en pantalla
   dividida, de principio a fin (escenas 2, 4 y 7). Es la toma más larga, y se cierra sola.
7. **Resultados** (escena 5): la imagen de comparación de la toma 6 y la tabla del lote.

## Recomendaciones de grabación

- **OBS Studio** a 1920×1080, 30 fps, en formato MKV (no se corrompe si se corta la grabación;
  luego se convierte a MP4 desde OBS).
- **Una escena de OBS por toma:** "Gazebo + RViz" (dos capturas de ventana lado a lado, 960 px
  cada una), "Terminal", "VS Code" y "KiCad".
- **Gazebo:** ocultar los paneles laterales; cámara cenital para la exploración y a ras de suelo
  para el gancho. Si el video va lento, grabar igual: se acelera en la edición.
- **RViz:** fondo oscuro (ya viene con 48;48;48). Dejar visibles solo *Mapa (SLAM)*, *Fronteras*,
  *Plan global*, *Víctimas detectadas*, *RobotModel* y *LaserScan*. Apagar *Costmap global*,
  *Costmap local*, *TF*, *Odometry* y *Nube profundidad*, que ensucian la imagen. Ocultar el
  panel izquierdo (*Displays*) antes de grabar. Si las esferas de víctimas se ven pequeñas,
  aumentar la escala del display *Víctimas detectadas*.
- **Terminal y VS Code:** fuente de 18–20 pt y tema oscuro.
- **Misma semilla en todo:** `seed:=1`. No mezclar con la variante `dusty`: sus víctimas son
  grises y se ven peor en video.
- **Antes de la misión:** cerrar otras simulaciones (comparten los tópicos de ROS) y probar la
  pantalla dividida con `quick:=true`.

## Sugerencias de edición

- **Acelerar:** la exploración (×16 a ×24, para que 15 min quepan en unos 45 s); reducir a ×4
  justo cuando aparece cada esfera roja, para que se alcance a ver.
- **Pantalla dividida:** Gazebo + RViz en las escenas 2, 4 y 7; en la escena 4, un recuadro
  pequeño con la cámara del robot cuando confirma la víctima.
- **Acercamientos:** sobre las fronteras (escena 4) y sobre las dos huellas `md5sum` iguales
  (escena 3).
- **Música:** instrumental suave a −25 dB bajo la voz; subirla solo en los 2 s finales.
- **Subtítulos:** quemados en la imagen (el video puede verse sin sonido), con el texto exacto de
  la narración, máximo dos líneas.
- **Rótulos:** los de la columna "Texto en pantalla", abajo a la izquierda, 2–3 s cada uno.

---

## Narración completa (para leer en voz alta)

Después de un sismo, entrar a los escombros es peligroso y nadie sabe cómo quedó el lugar. Cada
minuto cuenta para encontrar a las personas atrapadas.

Proponemos un robot que entra solo, arma su propio mapa y busca víctimas. Antes de construirlo, lo
probamos en un entorno virtual de cuatro capas: escenario, física, navegación y misión de
búsqueda.

El escenario no se dibuja a mano: un programa lo genera a partir de un número, la semilla. Con la
misma semilla sale exactamente el mismo mundo, con cuarenta escombros y cinco víctimas, todas
alcanzables. Así cada experimento se puede repetir.

Ahora el robot está solo. A la izquierda, el mundo simulado; a la derecha, lo que el robot sabe.
Con su láser dibuja el mapa: blanco es libre, negro es obstáculo y gris, lo que aún no conoce. El
borde entre lo conocido y lo desconocido se llama frontera, y el robot elige la que promete más
información por menos camino. Mientras avanza, su cámara busca formas humanas. Cuando confirma
una víctima, la marca con una esfera roja. Y una regla clave: el robot nunca sabe dónde están las
víctimas; solo usa sus sensores.

Al final, un evaluador independiente compara lo encontrado con la realidad. En cinco escenarios
distintos, el robot encontró las veinticinco víctimas, exploró en promedio el noventa y siete por
ciento del área y las ubicó con un error de veinticinco centímetros.

El siguiente paso es el robot físico. Este es su diagrama de conexiones y este, el circuito de la
placa. Cada etapa se simuló antes de fabricarla, como el amplificador del radar que detecta la
respiración. Y el mismo programa del microcontrolador corre en un simulador: ordenamos avanzar,
se activa la señal de los motores y los encoders cuentan cada giro de rueda.

En resumen: un entorno de prueba reproducible y un robot que explora y encuentra víctimas por sí
solo. Lo que sigue es construirlo y llevarlo del simulador a los escombros reales.
