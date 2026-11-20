// Firmware del ESP32 de findbot.
// Tareas periódicas (frecuencias en robot_hw.yaml → firmware.rates_hz):
//   control   PID de ruedas + odometría + vigilante de comandos + parada por batería crítica
//   imu       lectura del MPU6050
//   radar     muestreo del ADC del radar y RMS en ventana
//   slow      batería y CO2
// Interfaz según el entorno de PlatformIO (FINDBOT_MICROROS):
//   esp32        micro-ROS (H6): /cmd_vel → control; publica /odom, /tf, /joint_states,
//                /imu/data_raw, /battery_state (ros_bridge.cpp). Sin texto por el serial.
//   esp32_debug  texto (H4/H5): telemetría CSV "T,..." y mandos de una línea:
//     v <lineal m/s> <angular rad/s>   velocidad deseada (alimenta el vigilante)
//     s                                detener
//     h <0|1>                          calefactor del TGS4161
//     c                                calibrar línea base de CO2 (aire limpio)
//     r                                reiniciar odometría
//     ?                                línea de estado (IMU, batería, motores, odometría, encoders)
//   Avisos: "# motores ON pwm=… Hz", "# vigilante: sin mando, motores a 0",
//           "# motores OFF: bateria critica", "# bateria recuperada".
#include <Arduino.h>

#include "analog_inputs.h"
#include "battery.h"
#include "config_gen.h"
#include "encoders.h"
#include "imu_mpu6050.h"
#include "kinematics.h"
#include "log.h"
#include "motor_limit.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "watchdog.h"

#if FINDBOT_MICROROS
#include "ros_bridge.h"
#endif

namespace {

// Planificador simple: true cuando venció el periodo (sin acumular atraso si el lazo se demora).
struct Periodic {
  uint32_t period_us;
  uint32_t last_us = 0;
  bool due(uint32_t now_us) {
    if (now_us - last_us < period_us) return false;
    last_us += period_us;
    if (now_us - last_us >= period_us) last_us = now_us;
    return true;
  }
};

const core::DiffDriveParams kDrive{cfg::WHEEL_RADIUS, cfg::WHEEL_SEPARATION, cfg::MAX_LIN_VEL,
                                   cfg::MAX_ANG_VEL,  cfg::MAX_LIN_ACC,      cfg::MAX_ANG_ACC};
const core::PidGains kPid{cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT};
const core::BatteryParams kBattery{cfg::BAT_DIVIDER_RATIO, cfg::BAT_V_FULL,   cfg::BAT_V_LOW,
                                   cfg::BAT_V_CRITICAL,    cfg::BAT_V_CUTOFF, cfg::BAT_FILTER_ALPHA,
                                   cfg::BAT_HYSTERESIS};

Motors motors;
Encoders encoders;
Mpu6050 imu;
AnalogInputs analog;

core::Odometry odom(kDrive, cfg::COUNTS_PER_WHEEL_REV);
core::VelocityLimiter limiter(kDrive);
core::Pid pid_left(kPid), pid_right(kPid);
core::BatteryMonitor battery(kBattery);
core::CommandWatchdog watchdog(cfg::CMD_TIMEOUT_MS);
core::Co2Estimator co2(cfg::CO2_GAIN, cfg::CO2_SLOPE_V_PER_DECADE, cfg::CO2_REF_PPM);
core::RunningRms radar_rms(cfg::RADAR_RMS_WINDOW);

Periodic t_control{cfg::PERIOD_CONTROL_US};
Periodic t_imu{cfg::PERIOD_IMU_US};
Periodic t_radar{cfg::PERIOD_RADAR_US};
Periodic t_slow{cfg::PERIOD_SLOW_US};
Periodic t_telemetry{cfg::PERIOD_TELEMETRY_US};

// Estado de la tracción: cada cambio se avisa por serial en esp32_debug (pruebas de Wokwi, H5).
enum class DriveState { kIdle, kDriving, kStale, kCritical };
DriveState drive_state = DriveState::kIdle;
bool had_command = false;

void set_drive_state(DriveState next) {
  if (next == drive_state) return;
  if (drive_state == DriveState::kCritical) LOG_PRINTF("# bateria recuperada\n");
  switch (next) {
    case DriveState::kCritical: LOG_PRINTF("# motores OFF: bateria critica\n"); break;
    case DriveState::kStale: LOG_PRINTF("# vigilante: sin mando, motores a 0\n"); break;
    case DriveState::kDriving: LOG_PRINTF("# motores ON pwm=%lu Hz\n", motors.pwm_frequency()); break;
    case DriveState::kIdle: break;
  }
  drive_state = next;
}

core::Twist2D cmd{0.0f, 0.0f};
ImuSample imu_sample{};
float co2_adc = 0.0f;
int32_t ticks_left = 0, ticks_right = 0;  // cuentas acumuladas (diagnóstico)

// Protege lo que comparten el lazo de control (núcleo 1) y la tarea de micro-ROS (núcleo 0):
// el mando de velocidad, el vigilante y la copia de datos a publicar. Secciones muy cortas.
portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

// Punto único de entrada de mandos de velocidad (texto o /cmd_vel)
void on_velocity_command(float linear, float angular) {
  portENTER_CRITICAL(&g_lock);
  cmd = {linear, angular};
  watchdog.feed(millis());
  had_command = true;
  portEXIT_CRITICAL(&g_lock);
}

void control_step(float dt) {
  int32_t dl = 0, dr = 0;
  encoders.read_deltas(dl, dr);
  ticks_left += dl;
  ticks_right += dr;
  odom.update(dl, dr, dt);

  const bool critical = battery.level() == core::BatteryLevel::kCritical;
  portENTER_CRITICAL(&g_lock);
  const bool stale = watchdog.expired(millis());
  const core::Twist2D last_cmd = cmd;
  portEXIT_CRITICAL(&g_lock);
  const core::Twist2D target = (critical || stale) ? core::Twist2D{0.0f, 0.0f} : last_cmd;
  const core::Twist2D limited = limiter.update(target, dt);

  if (critical) {
    // Batería crítica: motores apagados (STBY en bajo) hasta que se recupere
    set_drive_state(DriveState::kCritical);
    limiter.reset();
    pid_left.reset();
    pid_right.reset();
    motors.enable(false);
    return;
  }
  if (stale) {
    set_drive_state(had_command ? DriveState::kStale : DriveState::kIdle);
  } else if (target.v == 0.0f && target.w == 0.0f) {
    set_drive_state(DriveState::kIdle);
  } else {
    set_drive_state(DriveState::kDriving);
  }
  if (limited.v == 0.0f && limited.w == 0.0f) {
    pid_left.reset();
    pid_right.reset();
    motors.set(0.0f, 0.0f);
    return;
  }
  const core::WheelSpeeds want = core::inverse_kinematics(kDrive, limited);
  const core::WheelSpeeds meas = odom.wheel_speeds();
  // Prealimentación (velocidad / velocidad máxima a tensión nominal) + corrección PID
  const float u_left = want.left / cfg::WHEEL_MAX_RAD_S + pid_left.update(want.left, meas.left, dt);
  const float u_right = want.right / cfg::WHEEL_MAX_RAD_S + pid_right.update(want.right, meas.right, dt);
  const float v_bat = battery.voltage();
  if (!motors.enabled()) motors.enable(true);
  motors.set(core::duty_from_command(u_left, v_bat, cfg::MOTOR_V_RATED, cfg::MOTOR_DEADBAND),
             core::duty_from_command(u_right, v_bat, cfg::MOTOR_V_RATED, cfg::MOTOR_DEADBAND));
}

void slow_step() {
  battery.update(analog.battery_adc_volts());
  co2_adc = analog.co2_adc_volts();
}

// LED: fijo si todo bien; parpadea si hay falla (IMU, batería crítica o sin agente micro-ROS)
void status_led(bool fault) {
  digitalWrite(cfg::PIN_LED_STATUS, fault ? !digitalRead(cfg::PIN_LED_STATUS) : HIGH);
}

#if FINDBOT_MICROROS
// ------------------------------- Interfaz micro-ROS -------------------------------
RosBridge ros;
RosSnapshot g_snapshot{};

// Tarea de micro-ROS (núcleo cfg::ROS_TASK_CORE): copia los datos y atiende al agente.
void ros_task(void*) {
  for (;;) {
    portENTER_CRITICAL(&g_lock);
    const RosSnapshot local = g_snapshot;
    portEXIT_CRITICAL(&g_lock);
    ros.spin(local);
    vTaskDelay(1);
  }
}

void interface_begin() {
  ros.begin(&on_velocity_command);
  xTaskCreatePinnedToCore(ros_task, "microros", cfg::ROS_TASK_STACK, nullptr, 1, nullptr,
                          cfg::ROS_TASK_CORE);
}

// Lazo de control: publica su estado más reciente para la tarea de micro-ROS
void interface_poll() {
  const core::WheelSpeeds pos = odom.wheel_positions();
  const core::WheelSpeeds vel = odom.wheel_speeds();
  const RosSnapshot snap{odom.x(),    odom.y(),    odom.theta(), odom.v(),
                         odom.w(),    pos.left,    pos.right,    vel.left,
                         vel.right,   imu_sample,  imu.ok(),     battery.voltage(),
                         battery.percentage(),     battery.level()};
  portENTER_CRITICAL(&g_lock);
  g_snapshot = snap;
  portEXIT_CRITICAL(&g_lock);
}

void interface_periodic() {
  status_led(!imu.ok() || battery.level() == core::BatteryLevel::kCritical || !ros.connected());
}

#else
// ------------------------------- Interfaz de texto --------------------------------
char line[64];
size_t line_len = 0;

void handle_command(const char* text) {
  float a = 0.0f, b = 0.0f;
  int flag = 0;
  switch (text[0]) {
    case 'v':
      if (sscanf(text + 1, "%f %f", &a, &b) == 2) on_velocity_command(a, b);
      break;
    case '?':
      LOG_PRINTF("# estado imu_gz=%.2f bat=%.2f nivel=%d motores=%d x=%.3f theta=%.3f "
                 "enc_l=%ld enc_r=%ld\n",
                 imu_sample.gz, battery.voltage(), static_cast<int>(battery.level()),
                 motors.enabled() ? 1 : 0, odom.x(), odom.theta(),
                 static_cast<long>(ticks_left), static_cast<long>(ticks_right));
      break;
    case 's':
      cmd = {0.0f, 0.0f};
      break;
    case 'h':
      if (sscanf(text + 1, "%d", &flag) == 1) analog.set_heater(flag != 0);
      break;
    case 'c':
      co2.calibrate(co2_adc);
      break;
    case 'r':
      odom.reset();
      break;
    default:
      LOG_PRINTF("# mando desconocido: %s\n", text);
  }
}

void interface_begin() {
  LOG_PRINTF("# T,t_ms,x,y,theta,v,w,wl,wr,ax,ay,az,gx,gy,gz,vbat,bat_level,bat_pct,"
             "co2_emf,co2_ppm,radar_rms,heater,cmd_active\n");
}

void interface_poll() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (line_len > 0) {
        line[line_len] = '\0';
        handle_command(line);
        line_len = 0;
      }
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = ch;
    }
  }
}

void interface_periodic() {
  const core::WheelSpeeds ws = odom.wheel_speeds();
  LOG_PRINTF("T,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,"
             "%.2f,%d,%.0f,%.3f,%.0f,%.4f,%d,%d\n",
             static_cast<unsigned long>(millis()), odom.x(), odom.y(), odom.theta(), odom.v(),
             odom.w(), ws.left, ws.right, imu_sample.ax, imu_sample.ay, imu_sample.az,
             imu_sample.gx, imu_sample.gy, imu_sample.gz, battery.voltage(),
             static_cast<int>(battery.level()), battery.percentage() * 100.0f,
             co2.emf(co2_adc), co2.ppm(co2_adc), radar_rms.rms(), analog.heater() ? 1 : 0,
             watchdog.expired(millis()) ? 0 : 1);
  status_led(!imu.ok() || battery.level() == core::BatteryLevel::kCritical);
}
#endif

}  // namespace

void setup() {
  Serial.begin(cfg::USB_BAUD);
  pinMode(cfg::PIN_LED_STATUS, OUTPUT);
  motors.begin();
  encoders.begin();
  analog.begin();
  for (int i = 0; i < 5; ++i) slow_step();  // batería válida antes de mover motores
  const bool imu_ok = imu.begin();
  if (imu_ok) imu.calibrate_gyro(cfg::IMU_CALIB_SAMPLES);
  LOG_PRINTF("# findbot firmware | IMU %s | bateria %.2f V\n", imu_ok ? "ok" : "NO RESPONDE",
             battery.voltage());
  interface_begin();
  const uint32_t now = micros();
  for (Periodic* p : {&t_control, &t_imu, &t_radar, &t_slow, &t_telemetry}) p->last_us = now;
}

void loop() {
  const uint32_t now = micros();
  interface_poll();
  if (t_control.due(now)) control_step(cfg::PERIOD_CONTROL_US * 1e-6f);
  if (t_imu.due(now)) imu.read(imu_sample);
  if (t_radar.due(now)) radar_rms.add(analog.radar_adc_volts() - cfg::RADAR_VREF);
  if (t_slow.due(now)) slow_step();
  if (t_telemetry.due(now)) interface_periodic();
}
