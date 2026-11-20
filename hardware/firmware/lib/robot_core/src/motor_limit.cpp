#include "motor_limit.h"

#include "util.h"

namespace core {

float duty_from_command(float u, float v_bat, float v_rated, float deadband) {
  constexpr float kEpsilon = 1e-3f;
  if (v_bat <= 0.0f || fabsf(u) < kEpsilon) {
    return 0.0f;
  }
  const float duty_max = clampf(v_rated / v_bat, 0.0f, 1.0f);
  if (duty_max <= deadband) {
    return 0.0f;  // batería tan alta/baja que no queda rango útil: no mover
  }
  const float mag = clampf(fabsf(u), 0.0f, 1.0f);
  return signf(u) * (deadband + (duty_max - deadband) * mag);
}

}  // namespace core
