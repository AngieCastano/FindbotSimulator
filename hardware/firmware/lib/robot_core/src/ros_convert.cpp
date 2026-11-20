#include "ros_convert.h"

#include <math.h>

namespace core {

Quaternion yaw_to_quaternion(float yaw) {
  const double half = 0.5 * static_cast<double>(yaw);
  return {0.0, 0.0, sin(half), cos(half)};
}

void covariance6_from_diagonal(const float diag[6], double out[36]) {
  for (int i = 0; i < 36; ++i) out[i] = 0.0;
  for (int i = 0; i < 6; ++i) out[i * 6 + i] = static_cast<double>(diag[i]);
}

void covariance3_isotropic(float variance, double out[9]) {
  for (int i = 0; i < 9; ++i) out[i] = 0.0;
  for (int i = 0; i < 3; ++i) out[i * 3 + i] = static_cast<double>(variance);
}

void split_nanos(int64_t nanos, int32_t& sec, uint32_t& nanosec) {
  if (nanos < 0) nanos = 0;
  sec = static_cast<int32_t>(nanos / 1000000000LL);
  nanosec = static_cast<uint32_t>(nanos % 1000000000LL);
}

uint8_t battery_health(BatteryLevel level) {
  return level == BatteryLevel::kCritical ? kPowerSupplyHealthDead : kPowerSupplyHealthGood;
}

}  // namespace core
