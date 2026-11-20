# Hardware del robot findbot: diseño y verificación (fases H0–H7)

> **Nota (H7):** varios valores de este documento fueron corregidos por la validación virtual
> (pull-ups I2C 2,2 kΩ, referencia del radar 1,30 V, límite de 5,0 V en el motor, paro de 1,5 A).
> La versión vigente y completa está en [`REPORTE_SIMULACION.md`](REPORTE_SIMULACION.md).

**Proyecto:** robot autónomo de búsqueda de víctimas en escombros — Ingeniería Electrónica, UNAD.
**Metodología:** CDIO (Concebir, Diseñar, Implementar, Operar).
**Alcance:** circuito de la placa base, firmware del ESP32 y su integración con ROS 2. El
software de navegación y percepción (`usar_ws/`) se documenta aparte.

Todo lo descrito es reproducible desde el repositorio: los valores numéricos viven en un solo
archivo, [`hardware/config/robot_hw.yaml`](../config/robot_hw.yaml), y cada artefacto (circuitos
SPICE, esquemático, firmware, simulación Wokwi) se genera o verifica a partir de él. Un único
comando, `hardware/check_all.sh`, repite todas las verificaciones.

---

## 1. Concebir

### 1.1 Requisitos
La tabla de sensores del `CLAUDE.md` raíz es la especificación: el modelo de Gazebo, el circuito
y el firmware deben tener los mismos elementos y los mismos tópicos de ROS 2.

| Elemento | Conexión | Tópico | Estado |
|---|---|---|---|
| 2 motorreductores TT 1:48 + encoders Hall | ESP32 (TB6612FNG, contador PCNT) | `/cmd_vel`, `/odom`, `/joint_states` | MVP |
| IMU MPU6050 | ESP32 (I2C) | `/imu/data_raw` | MVP |
| Monitor de batería (divisor resistivo) | ESP32 (ADC) | `/battery_state` | MVP |
| RPLiDAR A1, RealSense D435 | computador (USB) | `/scan`, `/camera/*` | MVP |
| Cámara térmica MLX90640 | ESP32 (I2C, conector listo) | `/thermal/image` | Fase 2 |
| CO₂ Figaro TGS4161 | ESP32 (ADC + acondicionamiento) | `/co2/concentration` | Fase 2 |
| Arreglo de micrófonos ReSpeaker | computador (USB) | por definir | Fase 2 |
| Radar Doppler HB100 | ESP32 (ADC + acondicionamiento) | por definir | Fase 2 |

### 1.2 Decisiones de diseño
| Decisión | Alternativa descartada | Razón |
|---|---|---|
| Computador (portátil) fuera del robot, un solo cable USB 3.0 activo a un hub a bordo | Raspberry Pi 5 a bordo | Con motores TT el robot debe ser liviano (~1,1 kg); la Pi sumaba ~20 W y ~350 g |
| Chasis de 22 × 16 cm y ruedas de 65 mm | Chasis de 40 × 30 cm y 5 kg (primer modelo de la simulación) | Un motor TT 1:48 no mueve 5 kg con margen |
| Placa base con módulos (TB6612, GY-521, reguladores) sobre zócalos | Chips soldados (SSOP, QFN) | Se suelda a mano y se repara |
| Micrófonos por USB al computador | 4 micrófonos I2S en el ESP32 | Los dos buses I2S del ESP32 no están sincronizados (la triangulación necesita muestreo simultáneo) y gastaban 6 pines |
| Mantener la IMU | Odometría solo con encoders | En escombros las ruedas patinan; el giróscopo corrige el ángulo |
| Cámara RealSense D435 | Orbbec Astra | 72 g frente a 300 g y controlador oficial en ROS 2 Jazzy |

### 1.3 Arquitectura
```
 2S Li-ion 18650 + BMS ─ interruptor ─ fusible 5 A ─ IRF4905 (inversión) ── VBAT (6,0–8,4 V)
   ├─ divisor 27k/10k + RC ─────────────────────────► ESP32 GPIO36
   ├─ TB6612FNG ──► 2 motores TT con encoder
   ├─ MP1584 5 V 3 A (5V_USB) ─► hub USB 3.0 ─┬─ RPLiDAR A1    ─┐
   │                                          ├─ RealSense D435 ├─ cable USB activo ─► portátil
   │                                          ├─ ReSpeaker      ┤   (ROS 2 Jazzy)
   │                                          └─ ESP32 (USB)    ┘
   └─ MP1584 5 V + filtro LC (5V_AUX) ─► calefactor TGS4161, HB100, encoders
 ESP32 3V3 ─► MPU6050, MLX90640 (fase 2), MCP6004 (CO₂, radar, VREF)
```

---

## 2. Diseñar

### 2.1 Asignación de pines (ESP32-WROOM-32)
19 señales; ninguna en pines de arranque prohibidos, en la UART0 (reservada para micro-ROS) ni en
la memoria flash. Las entradas analógicas usan solo el ADC1, porque el ADC2 deja de funcionar
con el WiFi activo. Los encoders usan el contador por hardware (PCNT) en cuadratura ×4. La tabla
completa está en `hardware/CLAUDE.md`; `scripts/check_pins.py` verifica estas reglas.

### 2.2 Presupuesto de potencia
| Consumidor | Típico (W) | Pico (W) |
|---|---:|---:|
| Motores | 3,0 | 12,0 |
| Dispositivos USB (LiDAR, D435, ReSpeaker, ESP32, hub) | 7,4 | 10,5 |
| Auxiliares (calefactor del CO₂, HB100, encoders) | 0,7 | 0,9 |
| **Total** | **11,1** | **23,4** |

Con una batería 2S de 3 Ah (22 Wh) la autonomía estimada es de ~1,5 h. El firmware limita el PWM
para que el motor TT nunca reciba más de sus 6 V nominales, aunque la batería esté en 8,4 V.

### 2.3 Acondicionamiento analógico — verificación con ngspice
Cinco circuitos simulados con modelos de comportamiento del MCP6004 y del AO3400; 23 criterios
de aceptación, **todos cumplidos** (`spice/run_spice.sh`).

| Circuito | Resultado | Criterio |
|---|---|---|
| Divisor de batería 27k/10k + 100 nF | 2,30 V en el ADC en el peor caso (8,4 V, ±1 %); 3,0 mV por paso del ADC; asentamiento en 3,4 ms | ≤ 2,45 V; ≤ 10 mV; < 5 ms |
| Encoders, salida de colector abierto + 10 kΩ a 3V3 | 0,007 V / 3,3 V; subida en 2,3 µs a 1,76 kHz | ≤ 0,8 V / ≥ 2,5 V; < 5 µs |
| Encoders, módulo con pull-up a 5 V + 15 kΩ a GND | 0,010 V / 3,0 V; subida en 1,4 µs | igual |
| Amplificador del TGS4161 (ganancia 3) | ganancia 3,00; salida de 0,58 a 1,51 V con el offset en contra; error de carga de 1 mV | 3 ± 1 %; 0,15–1,75 V |
| Amplificador del HB100 (2 etapas) | 66,8 dB; banda de −3 dB entre 0,11 y 43 Hz; reposo en 1,66 V; sin saturar con 0,5 mV pico | 63–69 dB; ~0,1–50 Hz |
| Calefactor del TGS4161 con AO3400 | 4,998 V en el calefactor; 0,1 mW en el MOSFET | 5 ± 0,2 V; < 50 mW |

Figuras: [respuesta en frecuencia del radar](../spice/results/radar_amp_bode.svg),
[transitorio del radar](../spice/results/radar_amp_tran.svg),
[divisor de batería](../spice/results/battery_divider_dc.svg),
[nivel lógico de los encoders](../spice/results/encoder_level.svg),
[amplificador del CO₂](../spice/results/co2_buffer_dc.svg).

### 2.4 Esquemático (KiCad 10)
61 componentes y 47 redes en un plano A2 con tres bloques (alimentación; ESP32, motores, encoders
e IMU; analógico). Lo genera `kicad/gen_schematic.py` desde el YAML, así que la tabla de pines y
el esquemático no pueden quedar distintos. **ERC: 0 errores y 0 advertencias.**
`kicad/check_netlist.py` compara la netlist exportada con la tabla de pines y con los valores
simulados en ngspice. [Esquemático en PDF](../kicad/out/findbot.pdf).

### 2.5 Geometría
| Elemento | x (m) | z sobre el suelo (m) |
|---|---:|---:|
| Eje de ruedas (`base_link`), r = 0,0325 m, separación 0,196 m | 0 | 0,0325 |
| Plano de escaneo del LiDAR | −0,03 | 0,145 |
| Cámara D435 | 0,055 | 0,1235 |
| IMU | 0 | 0,058 |

Regla de diseño: nada sobre la placa superior supera el plano del LiDAR. Límites del robot real:
0,30 m/s y 1,5 rad/s.

---

## 3. Implementar

### 3.1 Firmware
- **Lógica pura** (`lib/robot_core/`, sin Arduino): PID de velocidad por rueda con protección
  contra saturación del integrador, cinemática y odometría, límite del PWM a 6 V con
  compensación de zona muerta, batería con histéresis, vigilante de comandos y conversiones para
  ROS. Se prueba en el computador: **43 pruebas, todas aprobadas.**
- **Controladores** (`src/`): PWM a 20 kHz (inaudible para los micrófonos), contador PCNT para
  los encoders, MPU6050 por registros, ADC1 con la calibración de fábrica.
- **Configuración generada:** `tools/gen_config.py` convierte el YAML en `config_gen.h` antes de
  cada compilación; el código no tiene números fijos.
- **Dos núcleos:** el control corre en el núcleo 1 y micro-ROS en una tarea propia en el
  núcleo 0, de modo que la comunicación nunca bloquea el control.

### 3.2 Interfaz ROS 2 (micro-ROS por USB serial)
| Tópico | Tipo | Frecuencia |
|---|---|---|
| `/cmd_vel` (entrada) | `geometry_msgs/Twist` | — |
| `/odom` y `/tf` (odom → base_footprint) | `nav_msgs/Odometry`, `tf2_msgs/TFMessage` | 20 Hz |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` (best effort, sin orientación) | 50 Hz |
| `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz |

Uso del enlace serial a 921 600 baudios: ~40 % (prueba nativa). Memoria: RAM 21,4 %, flash 33,6 %.

### 3.3 Seguridad
- Sin `/cmd_vel` durante 0,5 s → motores a cero.
- Batería bajo 6,2 V → motores deshabilitados (STBY en bajo) hasta que se recupere.
- STBY con pull-down: los motores quedan apagados mientras el ESP32 arranca.
- Fusible de 5 A y protección contra batería invertida.

### 3.4 Costo estimado
MVP ≈ **540 USD** (≈ 2,16 millones de COP); sensores de fase 2 ≈ **182 USD**. La cámara D435 es
el 43 % del total. Detalle en [`bom/resumen.md`](../bom/resumen.md). Los precios son
**estimados** y deben verificarse con proveedores.

---

## 4. Operar: verificación sin el robot físico

| Nivel | Herramienta | Resultado |
|---|---|---|
| Circuitos analógicos | ngspice | 23/23 criterios |
| Esquemático | KiCad ERC + netlist frente a la tabla de pines | 0 errores; 19 GPIO coinciden |
| Lógica del firmware | Unity, 43 pruebas nativas | 43/43 |
| Firmware en el ESP32 simulado | Wokwi, escenario de 111 pasos | Superado: PWM a 20 kHz y sentido de giro, parada sin mandos, IMU, encoders (±20 cuentas exactas), batería crítica |
| Integración ROS 2 | Wokwi ⇄ agente micro-ROS ⇄ `ros2` CLI | 7/7: tópicos, tipos, frames, datos, hora sincronizada y `/cmd_vel` que habilita los motores |

### 4.1 Errores que encontró la verificación
1. **Interrupción de desbordes de los encoders sin registrar** (encontrado en Wokwi): el
   servicio de interrupciones se instalaba antes que las unidades del contador. En el robot, la
   odometría se habría corrompido tras ~14 vueltas de rueda.
2. **El ping al agente bloqueaba el control** (prueba de punta a punta): si se perdía la
   conexión con el robot en marcha, los motores habrían seguido con el último PWM durante los
   reintentos. Se resolvió con la tarea separada en el núcleo 0.
3. **Aserciones de pruebas desactivadas:** en Unity las comparaciones con `double` vienen
   desactivadas y no verificaban nada; se activaron.
4. **Dominio DDS:** el ESP32 no lee `ROS_DOMAIN_ID`; ahora el dominio está en el YAML y las
   pruebas usan un dominio aislado.

### 4.2 Limitaciones de la verificación simulada
- El ADC de Wokwi ignora la atenuación: la batería se lee ~11 % alta en la simulación. En el ESP32
  real la calibración de fábrica es la correcta.
- El serial de Wokwi pasa por la nube (0,5–1 s de ida y vuelta): las frecuencias de publicación
  solo se pueden medir con el ESP32 por USB. Además, la cola del canal crece mientras se crean
  las entidades de micro-ROS; la prueba de punta a punta usa esperas más largas
  (`firmware.ros.e2e`) que el firmware del robot.
- Los modelos SPICE del MCP6004 y del AO3400 son de comportamiento, ajustados a la hoja de datos.

---

## 5. Lista de verificación con el hardware real
| # | Prueba | Cómo | Dónde se ajusta |
|---|---|---|---|
| 1 | Pulsos por vuelta del encoder | Girar la rueda 10 vueltas a mano y leer las cuentas | `motors.encoder_ppr_motor` |
| 2 | Pull-up interno del módulo del encoder | Multímetro en la salida A con la rueda quieta | Caso A o B (R8–R11) |
| 3 | Orden de pines del módulo TB6612 | Comparar con la serigrafía | `gen_schematic.py` |
| 4 | Sentido de motores y encoders | `v 0.1 0` en `esp32_debug`: ambas ruedas avanzan y las cuentas crecen | `firmware.*_invert_*` |
| 5 | Lectura de batería | Comparar `vbat` con un multímetro en 3 tensiones | divisor / calibración |
| 6 | Zona muerta y PID | Rampas de velocidad, registrar `wl`/`wr` | `firmware.motor_deadband`, `firmware.pid` |
| 7 | Frecuencias reales en ROS 2 | `ros2 topic hz` con el ESP32 por USB | `firmware.ros.publish_hz` |
| 8 | Plano del LiDAR y distancia mínima de la D435 | Medición y `realsense-viewer` | `geometry.sensors` |
| 9 | TGS4161 | 2 h de precalentamiento; calibrar en aire limpio; aliento como prueba | `firmware.co2.slope_v_per_decade` |
| 10 | Radar | Persona a 1 m con el robot quieto; RMS frente a sala vacía | ganancia en `analog.radar` |

---

## 6. Coherencia con la simulación y trabajo pendiente
`scripts/check_consistency.py` compara el hardware con los archivos de la simulación (solo
lectura). Al cierre de la H7 no hay errores en la zona de hardware; las diferencias pendientes
corresponden a la simulación: geometría del chasis TT, IMU en el xacro y en el bridge, y límites
de velocidad. En la fase 2, el firmware debe publicar `/co2/concentration`
(`usar_msgs/GasConcentration`, que ya definió la sesión de simulación) y luego `/thermal/image`
y el radar.

## 7. Reproducir
```bash
bash -ic 'hardware/check_all.sh --e2e'   # todas las verificaciones (Wokwi requiere WOKWI_CLI_TOKEN)
hardware/spice/run_spice.sh              # solo SPICE
hardware/kicad/build.sh                  # esquemático, ERC, PDF y lista de materiales
cd hardware/firmware && pio test -e native && pio run -e esp32
```
