# Guion: video de la simulación del hardware de findbot

**Duración máxima:** 3:00 · **Idioma:** español · **Alcance:** solo la circuitería (capa física,
KiCad, ngspice, firmware en Wokwi y pruebas de estrés). No incluye Gazebo ni ROS 2.

**Regla del guion:** todo lo que aparece funcionando se comprobó en el repositorio el
2026-11-21. Lo que no existe o no se pudo comprobar está marcado **[PENDIENTE]** y no se narra
como si funcionara.

Rutas relativas a la raíz del repositorio (`FindbotSimulator/`).

---

## 0. Revisión previa: qué existe y qué se puede grabar hoy

| Elemento | ¿Existe? | Evidencia comprobada | ¿Se graba en vivo? |
|---|---|---|---|
| Diagrama de bloques de la capa física (R4) | Sí | `hardware/docs/figuras/r4_bloques.png` | Imagen con recorrido de zoom |
| Esquemático de KiCad | Sí | `hardware/kicad/findbot.kicad_sch` (KiCad 10 instalado) | Sí, en KiCad |
| ERC del esquemático | Sí | `hardware/kicad/out/erc.rpt`: 0 errores, 0 advertencias | Sí, en KiCad o con `kicad-cli` |
| Simulaciones de ngspice (R1) | Sí | 11 netlists en `hardware/spice/`; corrida completa de hoy: **72 criterios, 0 fallas** en 1 min 31 s | Sí, en la terminal (`run_spice.sh`) |
| Gráficas de R1 | Sí | `hardware/docs/figuras/r1_*.png`, `h2_radar_amp_bode.png` | Como animaciones de osciloscopio (sección 9.2) |
| Firmware y máquinas de estados (R2) | Sí | `hardware/firmware/src/main.cpp` (estados Reposo, En marcha, Sin mando, Batería crítica); diagrama `r2_estados_traccion.png`; 43 pruebas unitarias | Sí |
| Emulación en Wokwi con log serial | Sí | Escenario H5 superado el 2026-11-20 16:44 (`hardware/docs/evidencia/wokwi_h5_serial_2026-11-20_1644.log`) | Sí, en VS Code, si la extensión tiene minutos (ver [PENDIENTE] 2) |
| Analizador lógico en Wokwi | Parcial | Graba 8 canales (PWM, dirección y STBY de los motores, calefactor). Las capturas guardadas solo muestran cambios de STBY y de dirección; **no se ve el tren de PWM** | Ver [PENDIENTE] 3 y 4 |
| Motores girando en Wokwi | **No** | El circuito de Wokwi no tiene motores: el TB6612 y los motores no existen en Wokwi | [PENDIENTE] 1 |
| Encoders contando en Wokwi | Sí | Dos perillas KY-040 hacen de encoders; el log muestra `enc_l=20 enc_r=-20` | Sí |
| Estado de modo seguro (FAILSAFE) | Sí, con otro nombre | Estado `kStale`, que el firmware imprime como `# vigilante: sin mando, motores a 0` | Sí |
| Estado de batería baja (LOW_BATTERY) | Sí, con otro nombre | Nivel *bajo* (6,6 V) visible como `nivel=1` en la línea de estado; nivel *crítico* (6,2 V) imprime `# motores OFF: bateria critica` y el LED de la placa parpadea | Sí; el nivel bajo no imprime aviso propio ([PENDIENTE] 5) |
| Pruebas de estrés (R3) | Sí | `stress_mc.cir` (300 corridas), `stress_temp.cir`, `stress_noise.cir`, motor bloqueado en `motor_drive.cir`; tablas 13 y 14 del reporte | Sí, en la terminal + animación de Monte Carlo |
| `REPORTE_SIMULACION.md` | Sí | `hardware/docs/REPORTE_SIMULACION.md` (y `.docx`) | Para tablas de apoyo |

Los nombres FAILSAFE y LOW_BATTERY no existen literalmente en el código. En pantalla se usan los
mensajes reales del monitor serial, y la narración dice “modo seguro” y “batería baja”.

---

## 1. Introducción (0:00–0:15)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:00–0:15 | Encadenado rápido de 3 s cada uno: el diagrama de bloques, el esquemático de KiCad, la animación del osciloscopio y Wokwi con el ESP32. Termina en Wokwi | Este es findbot, un robot que buscará víctimas entre escombros. Antes de comprar un solo componente, probamos todo su circuito en el computador. Así un error cuesta un clic, no una placa quemada. | findbot · validación virtual del circuito | Recortes de 3 s de las tomas T1, T3, T6 y T8 (sección 9.1) |

**Palabras: 33 · Duración de voz: ≈ 15 s**

## 2. Diagrama de bloques de la capa física, R4 (0:15–0:40)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:15–0:25 | Zoom al recuadro “Energía”. El cursor sigue la flecha Batería 2S → fusible + IRF4905 → MP1584 → 5V_USB | La energía sale de una batería de dos celdas, pasa por un fusible y dos reguladores que entregan cinco voltios. | Energía: 7,4 V → 5 V | Abrir `hardware/docs/figuras/r4_bloques.png` en el visor de imágenes, a pantalla completa. Zoom con la rueda del ratón sobre la izquierda |
| 0:25–0:33 | Paneo hacia “Placa base”: el ESP32 al centro, con las flechas que llegan de la IMU, el MCP6004 y los encoders, y la que sale al TB6612 → motores | El cerebro es un ESP32. Lee los sensores y mueve los motores con un puente H, un circuito que invierte el giro. | Control: ESP32 + puente H | Misma imagen; arrastrar hacia la derecha y detenerse en el ESP32 |
| 0:33–0:40 | Paneo hacia arriba: LiDAR, cámara y micrófonos → hub USB → portátil | La cámara y el LiDAR van por USB al portátil, que hace el mapa. | Comunicación: USB al portátil | Misma imagen; arrastrar hacia la esquina superior derecha |

**Palabras: 56 · Duración de voz: ≈ 26 s**

## 3. Esquemático en KiCad (0:40–1:00)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 0:40–0:52 | Esquemático completo en tema oscuro. Zoom sucesivo de 2 s por bloque: alimentación (MP1584, IRF4905), acondicionamiento (MCP6004), ESP32, potencia (TB6612) y conectores de sensores | Con ese mapa dibujamos el esquemático en KiCad: alimentación, acondicionamiento de señales, el ESP32, la potencia y los conectores de los sensores. | Esquemático: 5 bloques | `kicad hardware/kicad/findbot.kicad_pro` → doble clic en `findbot.kicad_sch`. Encuadrar solo el lienzo (ocultar paneles con *Ver*). Zoom con la rueda sobre cada bloque |
| 0:52–1:00 | Ventana del revisor eléctrico: clic en “Ejecutar ERC” → “0 errores, 0 advertencias” | El revisor eléctrico busca pines sueltos o salidas en corto. Resultado: cero errores. | ERC: 0 errores ✔ | En el editor: *Inspeccionar → Comprobador de reglas eléctricas → Ejecutar ERC*. Alternativa en la terminal: `cd hardware/kicad && kicad-cli sch erc findbot.kicad_sch -o /tmp/erc.rpt && grep "ERC messages" /tmp/erc.rpt` |

**Palabras: 35 · Duración de voz: ≈ 16 s**

## 4. Circuitos y potencia en ngspice, R1 (1:00–1:45)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 1:00–1:06 | Terminal: `./run_spice.sh` en marcha, con las líneas `OK …` pasando (acelerado ×10) | Luego simulamos cada circuito en ngspice, que funciona como un osciloscopio virtual. | ngspice: 11 circuitos | `cd hardware/spice && ./run_spice.sh` (tarda 1 min 31 s; se acelera en edición). Terminal a pantalla completa, fuente de 18 pt |
| 1:06–1:18 | Animación: la curva amarilla de la batería cae cuando arrancan los motores; la verde de 5 V casi no se mueve. Luego el zoom progresivo hasta ver el rizado | Cuando arrancan los motores, la batería cae, pero la salida de cinco voltios se mantiene. El rizado, la pequeña ondulación que deja el regulador, no pasa de treinta y cinco milivoltios. | 5 V estable ✔ · Rizado < 50 mV ✔ | `hardware/docs/video_assets/a1_buck_arranque.gif` y `a2_rizado_zoom.gif`, a pantalla completa en el navegador. Se importan directo en el editor de video |
| 1:18–1:32 | Animación: el barrido de frecuencia dibuja la campana del filtro del radar; la línea roja de −3 dB corta en 0,1 y 43 Hz | El diagrama de Bode muestra cuánto amplifica el filtro del radar cada frecuencia. Deja pasar de una décima a cuarenta y tres hercios: ahí está la respiración. | Bode: pasa 0,1–43 Hz | `hardware/docs/video_assets/a4_radar_bode.gif` |
| 1:32–1:45 | Animación: la corriente del motor sube a 1,08 A y baja a 0,2 A; zoom a 1 ms con los 20 dientes del PWM | El PWM prende y apaga el motor veinte mil veces por segundo para regular su fuerza. La corriente nunca pasa el límite del driver. | PWM 20 kHz · I < 1,2 A ✔ | `hardware/docs/video_assets/a3_motor_pwm.gif` |

**Palabras: 94 · Duración de voz: ≈ 43 s**

## 5. Firmware en Wokwi, R2 (1:45–2:25)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 1:45–1:53 | VS Code: clic en *Start Simulator*. El ESP32 arranca y el monitor serial muestra `# findbot firmware \| IMU ok \| bateria 8.25 V` | Este es el firmware real del robot, corriendo en Wokwi, un emulador del ESP32. | Firmware real en Wokwi | Ver la preparación de Wokwi en la sección 9.3. Pantalla dividida: circuito a la izquierda y monitor serial a la derecha |
| 1:53–2:00 | Se escribe `v 0.2 0` en el monitor y aparece `# motores ON pwm=20000 Hz` | Le ordenamos avanzar y se activa la señal de los motores. | motores ON · PWM 20 kHz | Escribir en el monitor serial de Wokwi: `v 0.2 0` + Enter |
| 2:00–2:08 | Clic en las flechas de la perilla `enc_l`; se escribe `?` y aparece `enc_l=20 enc_r=-20` | Estas perillas hacen de encoders, los sensores que cuentan las vueltas de cada rueda. | Encoders contando ✔ | Clic varias veces en las flechas de las perillas KY-040 y luego `?` + Enter |
| 2:08–2:16 | Diagrama de estados (`r2_estados_traccion.png`) 2 s, y vuelta al monitor: aparece `# vigilante: sin mando, motores a 0` | El código es una máquina de estados: modos con reglas para pasar de uno a otro. Sin órdenes por medio segundo, entra en modo seguro y frena. | Modo seguro: frena solo | El aviso sale solo, 0,5 s después de `v 0.2 0`. La imagen está en `hardware/docs/figuras/r2_estados_traccion.png` |
| 2:16–2:25 | Se arrastra hacia abajo la perilla `pot_bat`; `?` muestra `nivel=1`; luego `# motores OFF: bateria critica` y el LED de la placa parpadea | Si baja la batería, primero avisa nivel bajo y luego apaga los motores. | Batería baja → motores OFF | Arrastrar despacio `pot_bat` hacia la izquierda; escribir `?` cerca del 46 % del recorrido (nivel bajo, 6,6 V) y seguir hasta cerca del 43 % (crítico, 6,2 V). Valores estimados a partir del escenario H5, donde el 60,6 % da 8,25 V y el 42,6 % da 6,15 V. Zoom al LED azul de la placa |

**Palabras: 79 · Duración de voz: ≈ 36 s**

## 6. Pruebas de estrés, R3 (2:25–2:50)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 2:25–2:35 | Animación del histograma de Monte Carlo llenándose; termina con “300 de 300 bajo el límite” | Después vinieron las pruebas de estrés. Monte Carlo sortea trescientos lotes de componentes; ninguno se sale del límite. | Monte Carlo: 300/300 ✔ | `hardware/docs/video_assets/a5_montecarlo.gif` |
| 2:35–2:43 | Imagen de la rueda bloqueada (`r1_motor_bloqueado.png`) 3 s, y luego el final de la terminal con `72 criterios, 0 fallas` | También probamos rueda bloqueada, ruido, frío y calor. Setenta y dos pruebas, todas pasan. | 72 pruebas · 0 fallas ✔ | `hardware/docs/figuras/r1_motor_bloqueado.png`; luego la toma T5 (final de `run_spice.sh`) |
| 2:43–2:50 | Tabla 14 del reporte (correcciones C1–C4), resaltando las filas C1 y C2 | Y corregimos cuatro cosas, como limitar el voltaje del motor. | 4 correcciones al diseño | Abrir `hardware/docs/REPORTE_SIMULACION.md` en VS Code → vista previa (Ctrl+Shift+V) → sección 7.5 |

**Palabras: 42 · Duración de voz: ≈ 19 s**

## 7. Cierre (2:50–3:00)

| Tiempo | Qué se ve en pantalla | Narración (texto exacto) | Texto en pantalla | Cómo obtener la toma |
|---|---|---|---|---|
| 2:50–3:00 | El esquemático completo se aleja despacio; aparece el título | El circuito quedó validado en simulación. Ahora sí: a comprar componentes y armar el robot. | Circuito validado ✔ · findbot · UNAD | Toma T3 (KiCad): zoom hacia afuera hasta ver todo el esquemático |

**Palabras: 15 · Duración de voz: ≈ 7 s**

---

## 8. Resumen de tiempos

| Escena | Tiempo | Palabras | Voz estimada (130 ppm) |
|---|---|---|---|
| 1. Introducción | 0:00–0:15 | 33 | 15 s |
| 2. Diagrama de bloques | 0:15–0:40 | 56 | 26 s |
| 3. KiCad + ERC | 0:40–1:00 | 35 | 16 s |
| 4. ngspice (R1) | 1:00–1:45 | 94 | 43 s |
| 5. Wokwi (R2) | 1:45–2:25 | 79 | 36 s |
| 6. Estrés (R3) | 2:25–2:50 | 42 | 19 s |
| 7. Cierre | 2:50–3:00 | 15 | 7 s |
| **Total** | **3:00** | **354** | **≈ 2:43** |

Cifras narradas y su fuente:
- Rizado: `power_supply` da como máximo 33,1 mV (batería de 6,2 V), así que se narra “no pasa de treinta y cinco”.
- Filtro del radar: 0,11–43,5 Hz y 66,8 dB, según `radar_amp`.
- Motor: pico de 1,085 A y rueda bloqueada de 1,093 A, frente a 1,2 A de corriente media del TB6612.
- Monte Carlo: 300 corridas, con máximo de 2,293 V frente al límite de 2,45 V.
- Todas las cifras salen de `hardware/spice/results/summary.txt`, corrida del 2026-11-21.

---

## 9. Extras

### 9.1 Tomas en el orden más práctico para grabarlas

| # | Toma | Herramienta | Duración bruta | Se usa en |
|---|---|---|---|---|
| T1 | Wokwi completo: arranque → `v 0.2 0` → aviso del vigilante → perillas de los encoders + `?` → `pot_bat` hasta crítico. Grabarlo de corrido 2–3 veces | VS Code + Wokwi | 3 min | Escenas 1 y 5 |
| T2 | Lo mismo con más zoom solo al monitor serial (respaldo para leer los mensajes) | VS Code + Wokwi | 2 min | Escena 5 |
| T3 | KiCad: esquemático completo, zoom por bloque y alejamiento final | KiCad | 2 min | Escenas 1, 3 y 7 |
| T4 | KiCad: ejecutar el ERC | KiCad | 30 s | Escena 3 |
| T5 | Terminal: `./run_spice.sh` completo hasta `72 criterios, 0 fallas` | Terminal | 1 min 31 s | Escenas 4 y 6 |
| T6 | Las 5 animaciones de `video_assets/` (se importan directo; no hay que grabarlas) | Editor de video | — | Escenas 1, 4 y 6 |
| T7 | Diagrama de bloques con recorrido de zoom | Visor de imágenes | 1 min | Escena 2 |
| T8 | Imágenes fijas: `r2_estados_traccion.png`, `r1_motor_bloqueado.png` y la tabla 14 del reporte | Visor / VS Code | 1 min | Escenas 5 y 6 |
| T9 | Narración en voz alta (sección 9.6), en una sola toma, en un cuarto silencioso | Grabadora o OBS | 3 min | Todo |

Wokwi va primero porque depende de los minutos de la extensión ([PENDIENTE] 2). Si falla, queda
tiempo para el plan B: el log de evidencia y la toma de pantalla del escenario H5.

### 9.2 Animaciones creadas (`hardware/docs/video_assets/`)

Las gráficas estáticas se ven poco dinámicas, así que se generaron animaciones tipo osciloscopio
(fondo negro, cuadrícula verde y un haz que dibuja la señal) **con los datos reales de ngspice**,
sin dibujar nada a mano.

| Archivo | Qué muestra | Duración |
|---|---|---|
| `a1_buck_arranque.gif` | 5V_USB y VBAT apareciendo; los motores arrancan en 1 ms y la carga USB máxima entra en 2 ms (batería de 7,4 V) | 5,6 s |
| `a2_rizado_zoom.gif` | Zoom continuo desde la vista completa hasta una ventana de 40 µs donde se ve el rizado del regulador | 6,0 s |
| `a3_motor_pwm.gif` | Corriente de arranque del motor y luego zoom a 1 ms con los 20 pulsos del PWM (rizado de 99 mA) | 6,8 s |
| `a4_radar_bode.gif` | Barrido de frecuencia del filtro del radar (se vuelve a correr el análisis AC de `radar_amp.cir`) | 5,6 s |
| `a5_montecarlo.gif` | Histograma que se llena de 5 en 5 hasta 300 corridas, con el límite de 2,45 V | 5,4 s |

Para regenerarlas, por ejemplo si cambia el diseño: `python3 hardware/docs/video_assets/make_animations.py`
(unos 50 s; requiere Pillow y ngspice).

Nota: el GIF del rizado marca 4,9 mV porque mide solo esa ventana de 40 µs con batería de 7,4 V.
El criterio de la simulación toma todo el régimen: 19,3 mV con 7,4 V y 33,1 mV en el peor caso,
con 6,2 V. Por eso la narración dice “no pasa de treinta y cinco”.

Otras formas de darles movimiento a las imágenes fijas:
- En el diagrama de bloques y en el esquemático, recorrer las flechas con el cursor en lugar de cortar.
- Aplicar a las PNG fijas (`r1_motor_bloqueado.png`, tabla 14) un zoom lento del 100 al 110 % en el editor (efecto Ken Burns).
- Conversión a MP4, si el editor no acepta GIF: `ffmpeg -i a1_buck_arranque.gif -pix_fmt yuv420p a1_buck_arranque.mp4` (ffmpeg no está instalado: `sudo apt install ffmpeg`).

### 9.3 Preparación de Wokwi en VS Code

1. Compilar el firmware de texto: `cd hardware/firmware && pio run -e esp32_debug`.
2. En VS Code, abrir `hardware/wokwi/diagram.json`. Con F1 elegir *Wokwi: Select Config File* → `hardware/wokwi/wokwi.toml`, y luego F1 → *Wokwi: Start Simulator*.
3. La perilla `pot_bat` arranca en 8,25 V (valor 620 de 1023 en `diagram.json`).
4. Ensayar una vez completa sin grabar, para saber dónde están `pot_bat`, `enc_l`, `enc_r` y el LED de la placa.

### 9.4 Recomendaciones de grabación

- **OBS Studio** (no está instalado: `sudo apt install obs-studio`), en 1920×1080 a 30 fps, formato MKV (se convierte a MP4 al final con *Archivo → Remux*).
- **Tema oscuro:**
  - VS Code: *Dark Modern*.
  - KiCad: *Preferencias → Colores del esquemático*; si no trae uno oscuro, se instala desde el *Gestor de complementos y contenido → Color Themes*.
- **Fuente grande:**
  - Monitor serial de Wokwi y terminal: 18–20 pt. En VS Code, `"terminal.integrated.fontSize": 20` y zoom con Ctrl + `+`.
- **Ocultar lo que distrae:**
  - VS Code: barra lateral (Ctrl+B), minimapa y panel de problemas.
  - KiCad: paneles de propiedades y jerarquía.
  - El escritorio: notificaciones desactivadas.
- **Ventanas:** maximizadas, con el mismo tamaño en todas las tomas para que los cortes no salten.
- **Cursor:** visible y moviéndose despacio. Hacer una pausa de 1 s antes y después de cada clic.

### 9.5 Sugerencias de edición

- **Pantalla dividida:**
  - Escena 5: circuito de Wokwi a la izquierda (60 %) y monitor serial a la derecha (40 %).
  - Escena 4: en el paso del rizado, opcionalmente la gráfica completa al lado del zoom.
- **Acelerar:**
  - `run_spice.sh` ×10.
  - En Wokwi, los tiempos muertos entre comandos (×2–×4).
  - Nunca durante la aparición de un mensaje que se narra.
- **Resaltar:** recuadro o círculo sobre cada mensaje del monitor serial en el instante en que se nombra.
- **Música:** instrumental suave, a −25 dB bajo la voz, sin letra. Debe ser de licencia libre (por ejemplo, la biblioteca de audio de YouTube).
- **Subtítulos:** con el texto de la sección 9.6 (sirve como base del .srt). Fuente sin serifa, blanca con borde negro, en la parte inferior.
- **Rótulos:** los de la columna “Texto en pantalla”, siempre en la misma esquina, durante 2–3 s.
- **Transiciones:** cortes secos o fundidos de 0,3 s. Nada de efectos llamativos.

### 9.6 Narración completa (para leer en voz alta)

Este es findbot, un robot que buscará víctimas entre escombros. Antes de comprar un solo
componente, probamos todo su circuito en el computador. Así un error cuesta un clic, no una placa
quemada.

La energía sale de una batería de dos celdas, pasa por un fusible y dos reguladores que
entregan cinco voltios. El cerebro es un ESP32. Lee los sensores y mueve los motores con un
puente H, un circuito que invierte el giro. La cámara y el LiDAR van por USB al portátil, que
hace el mapa.

Con ese mapa dibujamos el esquemático en KiCad: alimentación, acondicionamiento de señales, el
ESP32, la potencia y los conectores de los sensores. El revisor eléctrico busca pines sueltos o
salidas en corto. Resultado: cero errores.

Luego simulamos cada circuito en ngspice, que funciona como un osciloscopio virtual. Cuando
arrancan los motores, la batería cae, pero la salida de cinco voltios se mantiene. El rizado, la
pequeña ondulación que deja el regulador, no pasa de treinta y cinco milivoltios. El diagrama de
Bode muestra cuánto amplifica el filtro del radar cada frecuencia. Deja pasar de una décima a
cuarenta y tres hercios: ahí está la respiración. El PWM prende y apaga el motor veinte mil veces
por segundo para regular su fuerza. La corriente nunca pasa el límite del driver.

Este es el firmware real del robot, corriendo en Wokwi, un emulador del ESP32. Le ordenamos
avanzar y se activa la señal de los motores. Estas perillas hacen de encoders, los sensores que
cuentan las vueltas de cada rueda. El código es una máquina de estados: modos con reglas para
pasar de uno a otro. Sin órdenes por medio segundo, entra en modo seguro y frena. Si baja la
batería, primero avisa nivel bajo y luego apaga los motores.

Después vinieron las pruebas de estrés. Monte Carlo sortea trescientos lotes de componentes;
ninguno se sale del límite. También probamos rueda bloqueada, ruido, frío y calor. Setenta y dos
pruebas, todas pasan. Y corregimos cuatro cosas, como limitar el voltaje del motor.

El circuito quedó validado en simulación. Ahora sí: a comprar componentes y armar el robot.

---

## 10. [PENDIENTE]

1. **Motores girando en Wokwi.** El circuito de Wokwi no tiene motores ni TB6612, porque Wokwi no
   los modela. Solo se ven los avisos del monitor serial y, en el analizador, los pines del
   puente H. Por eso la narración dice “se activa la señal de los motores”. *Falta:* agregar dos
   LED con resistencia en los pines de PWM (GPIO 26 y 25) del circuito de Wokwi, para que su
   brillo muestre la velocidad.
2. **Minutos de Wokwi.** La cuota de las pruebas automáticas de la línea de comandos está agotada
   este mes. No está comprobado que la extensión de VS Code siga funcionando. *Falta:* abrir el
   simulador en VS Code antes del día de grabación. *Plan B:* mostrar el log de evidencia
   `hardware/docs/evidencia/wokwi_h5_serial_2026-11-20_1644.log`, rotulado como “registro del 20/11/2026”.
3. **PWM en el analizador lógico.** Las dos capturas guardadas (`hardware/wokwi/wokwi.vcd` y la de
   evidencia de micro-ROS) no muestran el tren de pulsos de 20 kHz: solo cambian STBY y los pines
   de dirección. No se ha probado si una orden de varios segundos lo captura. Por eso el PWM se
   muestra con la simulación de ngspice (`a3_motor_pwm.gif`) y no con el analizador. *Falta:* en
   VS Code, mantener `v 0.2 0` repetido varios segundos, detener la simulación y revisar el
   archivo VCD que descarga Wokwi.
4. **Encoders en el analizador y visor de VCD.**
   - El analizador lógico no tiene conectados los encoders (GPIO 18, 19, 16 y 17): sus 8 canales
     están ocupados por los motores y el calefactor. Los encoders se muestran con el contador
     del monitor serial.
   - Para ver un VCD hace falta PulseView o GTKWave, y no hay ninguno instalado
     (`sudo apt install pulseview`).
5. **Aviso de batería baja.** El nivel bajo (6,6 V) no imprime un mensaje propio. Solo se ve como
   `nivel=1` al escribir `?`, o en la columna `bat_level` de la telemetría. El nivel crítico sí
   imprime `# motores OFF: bateria critica`.
6. **Herramientas sin instalar:** OBS Studio, ffmpeg y PulseView
   (`sudo apt install obs-studio ffmpeg pulseview`).
7. **Prueba de micro-ROS con `/cmd_vel`.** La prueba de punta a punta de que una orden de ROS 2
   enciende los motores sigue pendiente (por la cuota de Wokwi), así que no aparece en el video.
