# Guion — video de la simulación en Gazebo y RViz (máx. 3:00)

**Experimento:** demo, semilla 13, modo autónomo (`results/victims_seed13_demo_20261122_024907.csv`).

- **Escenario** (`usar_ws/worlds/seed13_demo/ground_truth.yaml`): 20 × 20 m, 40 escombros,
  3 víctimas grises ("cubiertas de polvo") a 2.1, 4.4 y 5.8 m del robot, 286.8 m² explorables.
- **Sensores:** cámara térmica, CO₂, micrófonos y radar. Los detectores de la cámara están
  apagados (`camera_detection:=false`): solo confirman los signos de vida.
- **Duración:** 133.6 s de tiempo simulado. La misión se cerró antes de terminar, así que no
  hay mapa final, gráficas ni JSON: los resultados del video salen del CSV de víctimas
  comparado con el ground truth.
- **Resultado:**

  | Víctima confirmada | Sensores | Tiempo | Distancia a la víctima real |
  |---|---|---|---|
  | 3 | radar + calor | 115.3 s | 0.89 m (acierto, umbral 1 m) |
  | 2 | CO₂ + calor | 60.0 s | 2.06 m (fuera del umbral) |

  La tercera víctima (a 5.8 m) no se alcanzó.

**Quedan fuera** porque no existen en `results/` para esta simulación: modo teleoperado y
estación remota, alcance de comunicación, latencias, comparación teleoperado vs. autónomo,
boxplots por dificultad y matriz de validación.

**Comando de todas las tomas** (abre Gazebo y RViz con la misma semilla):
```
docker exec -it findbot_sim bash -c 'source /opt/ros/jazzy/setup.bash && source /home/findbot/FindbotSimulator/usar_ws/install/setup.bash && cd /home/findbot/FindbotSimulator/usar_ws && ros2 launch usar_bringup mission.launch.py seed:=13 world_params:=demo use_thermal:=true use_co2:=true use_sound:=true use_radar:=true camera_detection:=false'
```

---

## Escena 1 — Gancho (0:00–0:15 · 30 palabras · ~14 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:00–0:15 | Gazebo en plano general, cámara bajando lentamente hacia el robot entre los escombros. | Después de un sismo, encontrar a las personas atrapadas es urgente y peligroso. Este robot autónomo entra a los escombros, construye su propio mapa y busca víctimas con varios sensores. | Búsqueda de víctimas tras un sismo | Comando de arriba. Grabar Gazebo en los primeros 15 s (el robot aún casi no se mueve) moviendo la cámara con la rueda del ratón desde arriba hacia el robot. |

## Escena 2 — El escenario (0:15–0:35 · 44 palabras · ~20 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:15–0:25 | Gazebo en vista superior del área completa. | El escenario se genera por computador a partir de una semilla: con la semilla trece siempre sale el mismo mundo. | Generación procedural · semilla 13 | Misma ejecución. Gazebo: vista superior (botón "Top" del menú de vista o rueda del ratón hasta ver los 20 × 20 m). |
| 0:25–0:35 | Zoom en Gazebo sobre una víctima gris junto a un escombro. | Veinte por veinte metros, cuarenta escombros y tres víctimas grises, del mismo color que los escombros, así que el color no ayuda a encontrarlas. | 20 × 20 m · 3 víctimas | Gazebo: acercar a la víctima en (0.3, 2.1), a 2 m por delante y a la izquierda del robot. |

## Escena 3 — El robot en acción (0:35–1:50 · 145 palabras · ~67 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:35–0:50 | Pantalla dividida: Gazebo (vista superior) a la izquierda, RViz a la derecha con el mapa casi vacío y el barrido del LiDAR. | A la izquierda, Gazebo: el mundo simulado. A la derecha, RViz: lo que el robot sabe. Al empezar, su mapa está vacío. El LiDAR, un láser que gira y mide distancias, barre el entorno. | Mapa inicial: vacío | Misma ejecución, grabando los dos desde el primer segundo. RViz: dejar activos "Mapa (SLAM)", "LaserScan", "RobotModel", "Fronteras (exploración)" y "Victimas detectadas"; desactivar "Costmap global", "Costmap local", "TF" y "Punto lookahead (RPP)". |
| 0:50–1:10 | RViz a pantalla completa: el mapa crece (acelerado ×4), fronteras de colores. | Con esas medidas, SLAM construye el mapa y ubica al robot dentro de él. Blanco es libre, negro son obstáculos y gris, lo desconocido. Las fronteras son el borde entre lo conocido y lo desconocido: el robot elige la más conveniente y va hacia ella. | SLAM: mapa + ubicación · Fronteras | Grabar RViz durante los primeros ~90 s simulados y acelerar ×4 en edición. |
| 1:10–1:25 | Panel "Termica (deteccion)" ampliado: mancha caliente de una víctima. Luego el robot detenido. | La cámara térmica ve el calor del cuerpo. Cada cierto tiempo el robot se detiene once segundos: así el radar puede notar la respiración y los micrófonos, la voz. | Calor corporal · Parada de 11 s | RViz: agrandar el panel "Termica (deteccion)" y desactivar "Camara (deteccion)" (queda vacío en este modo). Grabar cuando aparezca la mancha caliente y una parada completa (la terminal muestra "Parada de escucha"). |
| 1:25–1:50 | RViz: cilindro naranja "Candidato" y, después, el marcador numerado de la víctima confirmada. | Un solo sensor no basta: crea un candidato, este cilindro, y el robot va a revisarlo. Cuando dos sensores coinciden en el mismo lugar, por ejemplo calor y radar, la víctima se confirma y aparece su marcador. | Candidato → víctima confirmada | Grabar hasta ~120 s simulados: hacia los 60 s aparece la confirmación por CO₂ + calor y hacia los 115 s la de radar + calor (la terminal muestra "Víctima N confirmada … por radar+thermal"). Hacer zoom en RViz sobre el marcador. |

## Escena 4 — Resultados (1:50–2:45 · 89 palabras · ~41 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 1:50–2:05 | Pantalla dividida: RViz con los marcadores finales (izquierda) y Gazebo en vista superior con las víctimas reales (derecha). | ¿Qué tan bien lo hizo? Comparamos con el ground truth: la posición real de cada víctima, que el robot nunca conoce. | Ground truth: posición real | RViz y Gazebo a los ~134 s simulados (fin de la misión del experimento), en el mismo encuadre de vista superior. |
| 2:05–2:20 | Zoom sobre el marcador de radar + calor y la víctima real junto a él. | En dos minutos y catorce segundos, el robot confirmó dos detecciones. La primera, por radar y calor, quedó a 0,89 metros de una víctima real: un acierto. | Radar + calor: 0,89 m | Datos: `results/victims_seed13_demo_20261122_024907.csv` (fila `final`, víctima 3) frente a `usar_ws/worlds/seed13_demo/ground_truth.yaml` (`victim_01`). |
| 2:20–2:45 | Zoom sobre el marcador de CO₂ + calor; un círculo de 1 m alrededor de la víctima real muestra que queda fuera. Luego la víctima lejana sin marcador. | La segunda, por CO₂ y calor, quedó a 2,06 metros de la víctima más cercana. El límite para contar un acierto es un metro: es un error de ubicación. La tercera víctima, a 5,8 metros, no se alcanzó en ese tiempo. | CO₂ + calor: 2,06 m · Límite: 1 m | Datos: mismo CSV (fila `final`, víctima 2) frente a `victim_02`; `victim_00` a 5,8 m. El círculo de 1 m se dibuja en edición. |

## Escena 5 — Cierre (2:45–3:00 · 35 palabras · ~16 s)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 2:45–3:00 | Gazebo en plano general alejándose del robot; fundido a negro con el nombre del proyecto. | El robot exploró solo, sin mapa previo, y encontró víctimas que no se distinguen por su color, combinando sensores de signos de vida. Es la base del sistema que se probará en el robot real. | Autónomo · Sin mapa previo · Multisensor | Cualquier momento de la misma ejecución: alejar la cámara de Gazebo con la rueda del ratón. |

**Total de narración: 343 palabras (~2:38 a 130 palabras/min; el resto son pausas y la música del cierre).**

---

## Tomas que debes grabar (orden práctico)

Todas salen de **una sola ejecución** del comando de arriba. Graba Gazebo y RViz a la vez, en
pantalla dividida desde OBS, durante unos 3–4 min reales, y saca las tomas en edición.

| # | Ventana | Momento (tiempo simulado) | Duración | Qué debe verse |
|---|---|---|---|---|
| 1 | Gazebo | 0–15 s | 15 s | Plano general bajando hacia el robot (escena 1). |
| 2 | Gazebo | 0–20 s | 20 s | Vista superior completa y zoom a la víctima en (0.3, 2.1) (escena 2). |
| 3 | Gazebo + RViz | 0–90 s | 90 s | Pantalla dividida: mapa vacío → mapa creciendo, LiDAR y fronteras (escena 3). |
| 4 | RViz | cuando aparezca | 15 s | Panel térmico ampliado con la mancha caliente y una parada de 11 s. |
| 5 | RViz | ~55–120 s | 30 s | Cilindro de candidato y marcadores de las víctimas confirmadas (60 s y 115 s). |
| 6 | Gazebo + RViz | ~134 s | 20 s | Mismo encuadre en vista superior: marcadores frente a víctimas reales (escena 4). |
| 7 | Gazebo | cualquiera | 10 s | Plano alejándose para el cierre. |

Para no perder el momento de la confirmación, deja visible la terminal: imprime
"Candidato (…)" y "Víctima N confirmada … por …". Como cada ejecución puede variar un poco,
si los tiempos no coinciden con los del experimento, ajusta la narración de la escena 4 a lo
que muestre la terminal.

## Recomendaciones de grabación

- **OBS Studio** a 1920 × 1080 y 30 fps; dos fuentes de "Captura de ventana" (Gazebo y RViz)
  en una escena dividida 50/50, más una escena de pantalla completa para cada ventana.
- **RViz:**
  - fondo oscuro (el de la configuración: gris 48);
  - ocultar el panel "Tool Properties" y, si sobra espacio, "Views";
  - dejar solo los displays de la escena 3;
  - vista superior ("TopDownOrtho" en "Views");
  - ampliar el panel térmico arrastrando su borde;
  - si los marcadores se ven pequeños, acercar el zoom en vez de cambiar su tamaño.
- **Gazebo:** vista superior para las escenas 2–4, ocultar paneles laterales (menú de los
  tres puntos) y desactivar la cuadrícula si distrae.

## Sugerencias de edición

- **Pantalla dividida** en la escena 3 (primeros 15 s) y en la escena 4 (comparación
  marcadores vs. víctimas reales).
- **Acelerar ×4** el crecimiento del mapa (escena 3) y volver a velocidad normal en la
  parada de 11 s y en la confirmación.
- **Zoom** sobre cada marcador mientras se dice su distancia, y círculo de 1 m dibujado
  alrededor de la víctima real en la escena 4.
- Música suave de fondo a −20 dB bajo la voz; **subtítulos** con la narración exacta.

## Narración completa (para leer en voz alta)

Después de un sismo, encontrar a las personas atrapadas es urgente y peligroso. Este robot
autónomo entra a los escombros, construye su propio mapa y busca víctimas con varios sensores.

El escenario se genera por computador a partir de una semilla: con la semilla trece siempre
sale el mismo mundo. Veinte por veinte metros, cuarenta escombros y tres víctimas grises, del
mismo color que los escombros, así que el color no ayuda a encontrarlas.

A la izquierda, Gazebo: el mundo simulado. A la derecha, RViz: lo que el robot sabe. Al
empezar, su mapa está vacío. El LiDAR, un láser que gira y mide distancias, barre el entorno.
Con esas medidas, SLAM construye el mapa y ubica al robot dentro de él. Blanco es libre, negro
son obstáculos y gris, lo desconocido. Las fronteras son el borde entre lo conocido y lo
desconocido: el robot elige la más conveniente y va hacia ella. La cámara térmica ve el calor
del cuerpo. Cada cierto tiempo el robot se detiene once segundos: así el radar puede notar la
respiración y los micrófonos, la voz. Un solo sensor no basta: crea un candidato, este
cilindro, y el robot va a revisarlo. Cuando dos sensores coinciden en el mismo lugar, por
ejemplo calor y radar, la víctima se confirma y aparece su marcador.

¿Qué tan bien lo hizo? Comparamos con el ground truth: la posición real de cada víctima, que
el robot nunca conoce. En dos minutos y catorce segundos, el robot confirmó dos detecciones.
La primera, por radar y calor, quedó a 0,89 metros de una víctima real: un acierto. La
segunda, por CO₂ y calor, quedó a 2,06 metros de la víctima más cercana. El límite para contar
un acierto es un metro: es un error de ubicación. La tercera víctima, a 5,8 metros, no se
alcanzó en ese tiempo.

El robot exploró solo, sin mapa previo, y encontró víctimas que no se distinguen por su color,
combinando sensores de signos de vida. Es la base del sistema que se probará en el robot real.
