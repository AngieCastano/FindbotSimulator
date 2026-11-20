#pragma once
// Cinemática del robot diferencial: limitador de velocidad/aceleración, cinemática inversa
// (v, ω → ruedas) y odometría por integración del arco (x, y, θ en el marco odom).
#include <stdint.h>

namespace core {

struct DiffDriveParams {
  float wheel_radius;      // m
  float wheel_separation;  // m
  float max_lin_vel;       // m/s
  float max_ang_vel;       // rad/s
  float max_lin_acc;       // m/s²
  float max_ang_acc;       // rad/s²
};

struct WheelSpeeds {
  float left;   // rad/s
  float right;  // rad/s
};

struct Twist2D {
  float v;  // m/s
  float w;  // rad/s
};

// Recorta (v, ω) a los máximos y limita la variación por periodo a la aceleración máxima.
class VelocityLimiter {
 public:
  explicit VelocityLimiter(const DiffDriveParams& p) : p_(p) {}
  Twist2D update(Twist2D target, float dt);
  void reset() { last_ = {0.0f, 0.0f}; }

 private:
  DiffDriveParams p_;
  Twist2D last_{0.0f, 0.0f};
};

WheelSpeeds inverse_kinematics(const DiffDriveParams& p, Twist2D cmd);
Twist2D forward_kinematics(const DiffDriveParams& p, WheelSpeeds w);

// Convierte una variación de cuentas del encoder en ángulo de rueda (rad).
inline float ticks_to_rad(int32_t ticks, int32_t counts_per_rev) {
  return 6.28318530718f * static_cast<float>(ticks) / static_cast<float>(counts_per_rev);
}

class Odometry {
 public:
  Odometry(const DiffDriveParams& p, int32_t counts_per_rev) : p_(p), cpr_(counts_per_rev) {}

  // Integra un periodo con las cuentas acumuladas de cada rueda desde la llamada anterior.
  void update(int32_t d_ticks_left, int32_t d_ticks_right, float dt);
  void reset();

  float x() const { return x_; }
  float y() const { return y_; }
  float theta() const { return theta_; }
  float v() const { return v_; }
  float w() const { return w_; }
  WheelSpeeds wheel_speeds() const { return speeds_; }       // rad/s
  WheelSpeeds wheel_positions() const { return positions_; } // rad acumulados (joint_states)

 private:
  DiffDriveParams p_;
  int32_t cpr_;
  float x_ = 0.0f, y_ = 0.0f, theta_ = 0.0f, v_ = 0.0f, w_ = 0.0f;
  WheelSpeeds speeds_{0.0f, 0.0f};
  WheelSpeeds positions_{0.0f, 0.0f};
};

}  // namespace core
