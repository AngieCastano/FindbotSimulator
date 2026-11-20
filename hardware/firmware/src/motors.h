#pragma once
// Controlador de los 2 motores TT por el TB6612FNG: PWM LEDC a 20 kHz + pines de dirección.
// STBY queda en bajo (pull-down en la placa) hasta que se llama enable(true).

class Motors {
 public:
  void begin();
  void enable(bool on);
  // duty ∈ [−1, 1]; el signo da el sentido (ya corregido por la inversión del YAML).
  void set(float duty_left, float duty_right);
  bool enabled() const { return enabled_; }

 private:
  void apply(int ch, int in1, int in2, float duty, bool invert);
  bool enabled_ = false;
};
