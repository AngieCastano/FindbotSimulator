#pragma once
// Conversiones puras para armar los mensajes de ROS 2 (sin dependencias de micro-ROS, así se
// prueban en native). El puente src/ros_bridge.cpp las usa para llenar los mensajes.
#include <stdint.h>

#include "battery.h"

namespace core {

struct Quaternion {
  double x, y, z, w;
};

// Rotación pura en z (robot planar): q = (0, 0, sin(θ/2), cos(θ/2)).
Quaternion yaw_to_quaternion(float yaw);

// Matriz de covarianza 6×6 (fila mayor, como en ROS) con la diagonal dada y ceros fuera de ella.
void covariance6_from_diagonal(const float diag[6], double out[36]);

// Matriz 3×3 diagonal con la misma varianza en los 3 ejes.
void covariance3_isotropic(float variance, double out[9]);

// Tiempo en nanosegundos → builtin_interfaces/Time (sec, nanosec). Negativo → 0.
void split_nanos(int64_t nanos, int32_t& sec, uint32_t& nanosec);

// Valores de sensor_msgs/BatteryState (deben coincidir con las constantes del mensaje).
constexpr uint8_t kPowerSupplyStatusDischarging = 2;  // POWER_SUPPLY_STATUS_DISCHARGING
constexpr uint8_t kPowerSupplyHealthGood = 1;          // POWER_SUPPLY_HEALTH_GOOD
constexpr uint8_t kPowerSupplyHealthDead = 3;          // POWER_SUPPLY_HEALTH_DEAD
constexpr uint8_t kPowerSupplyTechnologyLion = 2;      // POWER_SUPPLY_TECHNOLOGY_LION

// OK y LOW → GOOD (el porcentaje indica cuánto queda); CRITICAL → DEAD (motores detenidos).
uint8_t battery_health(BatteryLevel level);

}  // namespace core
