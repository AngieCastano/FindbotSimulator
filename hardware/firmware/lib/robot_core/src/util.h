#pragma once
// Utilidades numéricas comunes.
#include <math.h>

namespace core {

constexpr float kPi = 3.14159265358979f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float signf(float v) { return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f); }

// Normaliza un ángulo a (−π, π].
inline float wrap_angle(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a <= -kPi) a += 2.0f * kPi;
  return a;
}

}  // namespace core
