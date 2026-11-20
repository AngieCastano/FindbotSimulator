#pragma once
// Puente micro-ROS (solo en el entorno esp32): suscribe /cmd_vel y publica /odom, /tf,
// /joint_states, /imu/data_raw y /battery_state con los nombres y frecuencias de robot_hw.yaml.
// Corre en su propia tarea FreeRTOS (núcleo cfg::ROS_TASK_CORE): las esperas de comunicación
// (ping al agente, publicación confiable) nunca bloquean el lazo de control del núcleo 1.
// Máquina de estados del agente (ejemplo oficial de reconexión de micro-ROS):
//   esperando agente → agente disponible (crear entidades) → conectado ⇄ desconectado.
#include <stdint.h>

#include "battery.h"
#include "imu_mpu6050.h"

// Copia de los datos del robot (por valor): la arma el lazo de control dentro de una sección
// crítica y la tarea de ROS la publica sin tocar los objetos del control.
struct RosSnapshot {
  float x, y, theta, v, w;
  float wheel_pos_left, wheel_pos_right;  // rad
  float wheel_vel_left, wheel_vel_right;  // rad/s
  ImuSample imu;
  bool imu_ok;
  float battery_voltage;
  float battery_percentage;  // 0..1
  core::BatteryLevel battery_level;
};

class RosBridge {
 public:
  using CmdCallback = void (*)(float linear, float angular);

  void begin(CmdCallback on_cmd);
  // Una iteración de la máquina de estados. Devuelve true si hay sesión con el agente.
  bool spin(const RosSnapshot& s);
  bool connected() const;
};
