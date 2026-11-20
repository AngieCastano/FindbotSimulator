# hardware/CLAUDE.md — Circuito y firmware del robot "findbot"

Complementa al `CLAUDE.md` raíz (que manda: tabla de sensores, reglas de sesiones y de git).
Aquí va solo el detalle de hardware. **Los datos numéricos viven en `config/robot_hw.yaml`**;
este archivo los explica. Si cambias uno, cambia el otro en el mismo commit y corre
`python3 hardware/scripts/check_pins.py`.

## Decisiones de diseño (H0–H1)
- **Computador fuera del robot**: portátil Ubuntu 24.04 + Jazzy, unido por **un solo cable**
  USB 3.0 activo (~5 m) a un **hub USB alimentado a bordo**. Por ese hub van el RPLiDAR A1, la
  RealSense D435, el arreglo de micrófonos ReSpeaker y el ESP32 (micro-ROS por serial).
- **Chasis pequeño acorde a motores TT 1:48** (ruedas de 65 mm), 2 placas, ~1,1 kg total.
- **Sensores de signos de vida**: además de la tabla raíz, se incluyen CO₂ (Figaro TGS4161),
  sonido (ReSpeaker USB Mic Array v2.0, 4 micrófonos, **al computador por USB**) y radar
  Doppler (HB100 10,525 GHz). *Pendiente de aprobar el texto nuevo de la tabla raíz.*
- **IMU MPU6050 se mantiene** (MVP). La simulación debe agregarla (bloque PENDIENTE).
- Cámara elegida: **RealSense D435** (72 g, `realsense2_camera` en Jazzy) sobre la Astra (300 g).

## Diagrama de bloques
```
 2S Li-ion 18650 + BMS ─ interruptor ─ fusible 5 A ─ ideal-diode ── VBAT (6,0–8,4 V)
   │                                                                 │
   ├─ divisor 27k/10k + RC ─────────────────────────► ESP32 GPIO36 (ADC_VBAT)
   ├─ TB6612FNG VM ──► motor TT izq. / der. (con encoder Hall)
   ├─ buck 5 V 3 A (5V_USB) ─► hub USB 3.0 ─┬─ RPLiDAR A1     ─┐
   │                                        ├─ RealSense D435  ├─ cable USB 3.0 activo ─► portátil
   │                                        ├─ ReSpeaker 4-mic ┤
   │                                        └─ ESP32 (USB)     ─┘
   └─ buck 5 V 1 A + LC (5V_AUX) ─► calefactor TGS4161 (MOSFET), HB100, encoders
 ESP32 3V3 ─► MPU6050, MLX90640 (fase 2, conector DNP), TB6612 VCC, MCP6004, pull-ups
 MCP6004 (rail-to-rail, entrada CMOS 1 pA): seguidor TGS4161 → GPIO39 ; 2 etapas + pasabanda del HB100 → GPIO34
```

## Tabla de pines (ESP32-WROOM-32, DevKit V1)
| GPIO | Red (KiCad / config.h) | Conecta a | Nota |
|---|---|---|---|
| 26 | `MOT_L_PWM` | TB6612 PWMA | LEDC 20 kHz (inaudible para los micrófonos) |
| 27 | `MOT_L_IN1` | TB6612 AIN1 | |
| 14 | `MOT_L_IN2` | TB6612 AIN2 | emite PWM al arrancar → inocuo porque STBY está en bajo |
| 25 | `MOT_R_PWM` | TB6612 PWMB | LEDC 20 kHz |
| 33 | `MOT_R_IN1` | TB6612 BIN1 | |
| 32 | `MOT_R_IN2` | TB6612 BIN2 | |
| 13 | `MOT_STBY` | TB6612 STBY | pull-down 10 kΩ: motores apagados hasta que el firmware arranca |
| 18 / 19 | `ENC_L_A` / `ENC_L_B` | encoder izquierdo | contador por hardware **PCNT** en cuadratura ×4 |
| 16 / 17 | `ENC_R_A` / `ENC_R_B` | encoder derecho | PCNT; pull-up 10 kΩ a 3V3 |
| 21 / 22 | `I2C_SDA` / `I2C_SCL` | MPU6050 (0x68), MLX90640 (0x33) | 400 kHz, pull-up 4,7 kΩ |
| 23 | `IMU_INT` | MPU6050 INT | dato listo |
| 36 | `ADC_VBAT` | divisor de batería | ADC1, 11 dB |
| 39 | `ADC_CO2` | seguidor del TGS4161 | ADC1 |
| 34 | `ADC_RADAR` | IF del HB100 amplificada | ADC1, muestreo 200 Hz |
| 4 | `CO2_HEATER_EN` | MOSFET AO3400 del calefactor | |
| 2 | `LED_STATUS` | LED de la placa | estado de la conexión micro-ROS |

Prohibidos: GPIO1/3 (UART0 = micro-ROS), 6–11 (flash), 12 (strapping de tensión de flash).
Analógicos solo en ADC1 (ADC2 falla con WiFi). Libres: GPIO35 (ADC1), GPIO5, GPIO15.

## Alimentación y presupuesto de potencia
- **Batería**: 2S1P Li-ion 18650 3000 mAh (7,4 V nominales, 22 Wh) + BMS 2S 10 A.
  Umbrales en `/battery_state`: aviso 6,6 V (3,3 V/celda), **parada de motores 6,2 V**.
- **Motores**: TB6612 VM directo de VBAT; el firmware limita el duty a `6 V / Vbat` para no
  pasar la tensión nominal del TT.
- **Divisor**: 27 kΩ / 10 kΩ → 8,4 V da 2,27 V en el ADC (< 2,45 V lineal), 100 nF de filtro.
- Presupuesto en batería: **típico ≈ 11 W, pico ≈ 23 W** → autonomía estimada ~1,5 h.
  Detalle por consumidor en `robot_hw.yaml → power_budget_w`.

## Acondicionamiento analógico (verificado en H2 con ngspice — `hardware/spice/`)
Valores en `robot_hw.yaml → analog`. MCP6004 (4 amplificadores): CO₂, radar etapa 1, radar etapa 2 y VREF.
- **TGS4161**: FEM de 0,2–0,5 V con impedancia altísima → no inversor MCP6004, **ganancia 3**
  (Rf 20 kΩ / Rg 10 kΩ), RC 1 kΩ/100 nF, ADC a 6 dB. Salida 0,58–1,51 V en el peor caso de offset;
  error de carga 1 mV (con Rs supuesta de 1 GΩ). El offset del amplificador (±4,5 mV) se elimina
  con la calibración de línea base en aire limpio. Calefactor 5 V/50 mA conmutado por AO3400 (GPIO4):
  4,998 V en el calefactor, 0,1 mW en el MOSFET. Precalentamiento ~2 h la primera vez.
- **HB100**: 2 etapas inversoras iguales referidas a VREF 1,65 V (Rin 100 kΩ + Cin 22 µF,
  Rf 4,7 MΩ ∥ Cf 470 pF) → **66,8 dB**, banda de −3 dB **0,11–43 Hz**, RC 1 kΩ/1 µF hacia el ADC.
  Entrada máxima sin saturar ≈ 0,7 mV pico. Solo es útil con el robot quieto.
- **Batería**: 2,30 V en el ADC en el peor caso (8,4 V, ±1 %), resolución 3 mV de batería por LSB,
  asentamiento al 99 % en 3,4 ms.
- **Encoders**: caso A (colector abierto + 10 kΩ a 3V3): 0,007/3,3 V, subida 2,3 µs a 1,76 kHz.
  Caso B (módulo con pull-up a 5 V): poblar 15 kΩ a GND → 3,0 V alto. **Medir el módulo real
  antes de soldar** (si su pull-up no es de 10 kΩ, recalcular en `robot_hw.yaml` y volver a simular).

## Geometría (la copia el xacro — fuente: `robot_hw.yaml → geometry`)
`base_link` = punto medio del eje de ruedas (0,0325 m sobre el suelo). x adelante, z arriba.

| Elemento | Frame | x | y | z (base_link) | z suelo | Nota |
|---|---|---|---|---|---|---|
| Chasis (caja) | `base_link` | −0,05 (centro) | 0 | 0,0455 | 0,078 | 0,22 × 0,16 × 0,066 m, 0,86 kg |
| Ruedas TT | `*_wheel_link` | 0 | ±0,098 | 0 | 0,0325 | r 0,0325, ancho 0,026, separación 0,196 |
| Rueda loca | `caster_link` | −0,14 | 0 | — | — | bola r 0,0125 |
| LiDAR A1 | `lidar_link` | −0,03 | 0 | 0,1125 | 0,145 | plano de escaneo; 5,5 Hz, 0,15–12 m |
| D435 | `camera_link` | 0,055 | 0 | 0,091 | 0,1235 | RGB 69°, profundidad 87°, distancia mínima ~0,20 m |
| MPU6050 | `imu_link` | 0 | 0 | 0,0255 | 0,058 | en la PCB, alineada con base_link |
| MLX90640 (fase 2) | `thermal_link` | 0,055 | −0,06 | 0,088 | 0,1205 | versión BAA 55° × 35° |
| HB100 | `radar_link` | 0,06 | 0 | 0,037 | 0,0695 | antena hacia +x |
| TGS4161 | `co2_link` | 0,03 | 0,05 | 0,0275 | 0,06 | |
| ReSpeaker | `mic_array_link` | −0,12 | 0 | 0,0835 | 0,116 | 4 micrófonos, radio 0,032 m |

Límites reales: v ≤ 0,30 m/s, ω ≤ 1,5 rad/s, a ≤ 0,5 m/s², α ≤ 2,0 rad/s².
**Regla**: nada sobre la placa alta (0,111 m) puede superar el plano del LiDAR (0,145 m).
Los nombres de joint del firmware en `/joint_states` son `left_wheel_joint` y `right_wheel_joint`,
y `/odom` va de `odom` a `base_footprint`, igual que en el xacro.

## Herramientas y comandos
- Revisar pines y geometría: `python3 hardware/scripts/check_pins.py`
- Simulaciones SPICE + criterios: `hardware/spice/run_spice.sh` → `hardware/spice/results/summary.txt`
  y gráficos `.svg`. Los netlists leen `results/params.inc`, generado desde `robot_hw.yaml`;
  los modelos de comportamiento (MCP6004, AO3400) están en `spice/models/`.
- PlatformIO Core 6.2 (`~/.local/bin/pio`), KiCad 10 (`kicad-cli`), ngspice 42, Wokwi 4.1 (licencia en VS Code).

## Fases de hardware
H1 arquitectura · H2 SPICE · H3 KiCad (ERC) · H4 firmware base + tests nativos · H5 Wokwi ·
H6 micro-ROS · H7 coherencia y cierre. Commit de `hardware/` al cerrar cada fase.
