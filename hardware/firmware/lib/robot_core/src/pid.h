#pragma once
// PID de velocidad por rueda con anti-windup (integración condicional + límite del término I)
// y derivada sobre la medición (sin "patada" al cambiar la consigna).

namespace core {

struct PidGains {
  float kp;
  float ki;
  float kd;
  float i_limit;    // |ki·∫e| máximo
  float out_limit;  // |salida| máxima
};

class Pid {
 public:
  explicit Pid(const PidGains& gains) : g_(gains) {}

  // Devuelve la corrección para este periodo; dt en segundos (> 0).
  float update(float setpoint, float measured, float dt);

  void reset() {
    integral_ = 0.0f;
    has_prev_ = false;
  }

  float integral_term() const { return integral_; }

 private:
  PidGains g_;
  float integral_ = 0.0f;   // ya multiplicado por ki
  float prev_measured_ = 0.0f;
  bool has_prev_ = false;
};

}  // namespace core
