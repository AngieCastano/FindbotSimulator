# Evidencia de las simulaciones de Wokwi (copias fijas)

Los guiones de Wokwi sobrescriben `hardware/wokwi/out/` en cada corrida. Estas copias conservan la
evidencia usada en `docs/REPORTE_SIMULACION.md`.

| Archivo | Origen | Corrida |
|---|---|---|
| `wokwi_h5_serial_2026-11-20_1644.log` | `wokwi/out/serial.log` (escenario H5, 111 pasos) | 2026-11-20 16:44, **superada** (`Scenario completed successfully`) |
| `wokwi_microros_e2e_2026-11-20_1648.vcd` | `wokwi/out/microros/e2e.vcd` (analizador lógico, prueba micro-ROS) | 2026-11-20 16:48 |

Nota sobre la traza VCD: el canal D6 (MOT_STBY) solo vale 1 en t = 0 (estado de encendido de
Wokwi) y no vuelve a subir. Por lo tanto **no** demuestra que un `/cmd_vel` de ROS 2 haya
habilitado los motores; la verificación original tenía un falso positivo (corregida en H7). Los
registros de texto de esa prueba (`ros2_checks.log`, `agent.log`) fueron sobrescritos por una
corrida posterior que falló por la cuota de CI de Wokwi.
