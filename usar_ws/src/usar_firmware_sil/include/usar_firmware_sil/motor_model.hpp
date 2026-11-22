// Modelo del motorreductor TT 1:48 y del encoder para el firmware en el lazo.
//
// El ESP32 manda un duty de PWM al TB6612; el motor convierte la tensión en par. Modelo de
// motor de corriente continua en régimen permanente (la inductancia, 1 mH, da constantes de
// tiempo de < 1 ms, despreciables frente al periodo de control de 20 ms):
//   τ = τ_paro · (V / V_ensayo) − (τ_paro / ω_vacío) · ω      (par en la rueda)
// con los datos de hardware/config/robot_hw.yaml (Adafruit 3777 a 6 V: 250 rpm sin carga,
// 0.8 kg·cm de par de paro). Con duty 0 el TB6612 cortocircuita el motor (freno): queda solo el
// término de fuerza contraelectromotriz. La fricción de la caja se modela como par de Coulomb.
#pragma once

#include <cmath>
#include <cstdint>

namespace sil {

struct MotorParams {
  double v_test;          // V con los que se midieron los datos
  double no_load_speed;   // rad/s de la rueda a v_test
  double stall_torque;    // N·m en la rueda a v_test
  double friction_torque; // N·m de fricción de Coulomb de la caja
};

// Par en la rueda (N·m) con el duty dado (−1..1), la batería v_bat y la velocidad ω (rad/s).
inline double wheel_torque(const MotorParams& p, double duty, double v_bat, double omega) {
  const double v = duty * v_bat;
  double tau = p.stall_torque * (v / p.v_test) - (p.stall_torque / p.no_load_speed) * omega;
  // Fricción de Coulomb: se opone al giro; parado, absorbe hasta friction_torque de par.
  if (std::fabs(omega) > 1e-3) {
    tau -= std::copysign(p.friction_torque, omega);
  } else if (std::fabs(tau) <= p.friction_torque) {
    tau = 0.0;
  } else {
    tau -= std::copysign(p.friction_torque, tau);
  }
  return tau;
}

// Cuentas acumuladas del encoder de cuadratura para un ángulo de rueda (rad): cuantización.
inline int32_t encoder_ticks(double angle, int32_t counts_per_rev) {
  return static_cast<int32_t>(std::floor(angle * counts_per_rev / (2.0 * M_PI)));
}

}  // namespace sil
