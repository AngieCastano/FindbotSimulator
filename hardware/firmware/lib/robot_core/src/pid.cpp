#include "pid.h"

#include "util.h"

namespace core {

float Pid::update(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }
  const float error = setpoint - measured;
  const float p = g_.kp * error;
  const float d = has_prev_ ? -g_.kd * (measured - prev_measured_) / dt : 0.0f;
  prev_measured_ = measured;
  has_prev_ = true;

  // Integración condicional: no integrar si la salida ya está saturada en el mismo sentido.
  const float candidate = clampf(integral_ + g_.ki * error * dt, -g_.i_limit, g_.i_limit);
  const float unsat = p + candidate + d;
  const bool saturating = (unsat > g_.out_limit && error > 0.0f) ||
                          (unsat < -g_.out_limit && error < 0.0f);
  if (!saturating) {
    integral_ = candidate;
  }
  return clampf(p + integral_ + d, -g_.out_limit, g_.out_limit);
}

}  // namespace core
