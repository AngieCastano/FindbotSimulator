// Emulación del lazo de control de velocidad con el CÓDIGO REAL del firmware (lib/robot_core) y un
// modelo electromecánico del motor TT (el mismo de spice/motor_drive.cir, promediado en el PWM).
// Reproduce control_step() de src/main.cpp: limitador → cinemática inversa → prealimentación + PID
// → duty_from_command(motor_v_limit) → planta → encoder cuantizado (PCNT ×4) → odometría.
// Uso: sim_control <escenario> <vbat> <factor_carga> <ruido_cuentas> <usar_pid 0|1> <semilla>
//      <R> <L> <Ke> <Kt> <b> <J> <T_carga> <R_on>
// Salida CSV por stdout: t, v_ref (limitada), v_odom, v_real, duty
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "config_gen.h"
#include "kinematics.h"
#include "motor_limit.h"
#include "pid.h"

int main(int argc, char** argv) {
  if (argc != 15) {
    fprintf(stderr, "uso: %s escenario vbat carga ruido pid semilla R L Ke Kt b J Tc Ron\n", argv[0]);
    return 2;
  }
  const double vbat = atof(argv[2]), load_k = atof(argv[3]);
  const int noise = atoi(argv[4]);
  const bool use_pid = atoi(argv[5]) != 0;
  srand(static_cast<unsigned>(atoi(argv[6])));
  const double R = atof(argv[7]), L = atof(argv[8]), Ke = atof(argv[9]), Kt = atof(argv[10]);
  const double b = atof(argv[11]), J = atof(argv[12]), Tc = atof(argv[13]) * load_k, Ron = atof(argv[14]);

  const core::DiffDriveParams p{cfg::WHEEL_RADIUS, cfg::WHEEL_SEPARATION, cfg::MAX_LIN_VEL,
                                cfg::MAX_ANG_VEL,  cfg::MAX_LIN_ACC,      cfg::MAX_ANG_ACC};
  const core::PidGains g{cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT};
  core::VelocityLimiter limiter(p);
  core::Pid pid(g);
  core::Odometry odom(p, cfg::COUNTS_PER_WHEEL_REV);

  const double dt = 1e-5;                            // paso de la planta
  const double t_ctrl = cfg::PERIOD_CONTROL_US * 1e-6;
  const double t_end = 4.0, t_on = 0.5, t_off = 3.0;  // mando de 0.30 m/s entre t_on y t_off
  double i = 0, w = 0, theta = 0, next_ctrl = 0, duty = 0;
  long ticks_prev = 0;
  printf("t,v_ref,v_odom,v_real,duty\n");
  for (double t = 0; t < t_end; t += dt) {
    if (t >= next_ctrl) {
      next_ctrl += t_ctrl;
      const long ticks = static_cast<long>(floor(theta / (2 * M_PI) * cfg::COUNTS_PER_WHEEL_REV));
      long d = ticks - ticks_prev;
      ticks_prev = ticks;
      if (noise > 0) d += (rand() % (2 * noise + 1)) - noise;  // cuentas espurias/perdidas
      odom.update(static_cast<int32_t>(d), static_cast<int32_t>(d), static_cast<float>(t_ctrl));
      const core::Twist2D target{(t >= t_on && t < t_off) ? 0.30f : 0.0f, 0.0f};
      const core::Twist2D lim = limiter.update(target, static_cast<float>(t_ctrl));
      if (lim.v == 0.0f && lim.w == 0.0f) {
        pid.reset();
        duty = 0;
      } else {
        const core::WheelSpeeds want = core::inverse_kinematics(p, lim);
        const float meas = odom.wheel_speeds().left;
        float u = want.left / cfg::WHEEL_MAX_RAD_S;
        if (use_pid) u += pid.update(want.left, meas, static_cast<float>(t_ctrl));
        duty = core::duty_from_command(u, static_cast<float>(vbat), cfg::MOTOR_V_LIMIT, cfg::MOTOR_DEADBAND);
      }
      printf("%.3f,%.4f,%.4f,%.4f,%.4f\n", t, lim.v, odom.v(), w * cfg::WHEEL_RADIUS, duty);
    }
    // Planta: PWM promediado, puente con R_on, motor R-L con fuerza contraelectromotriz
    const double vm = duty * vbat;
    const double di = (vm - (R + Ron) * i - Ke * w) / L;
    const double torque = Kt * i - b * w - (w > 1e-3 ? Tc : (w < -1e-3 ? -Tc : 0.0));
    double dw = torque / J;
    if (fabs(w) <= 1e-3 && fabs(Kt * i) <= Tc) dw = 0;  // fricción estática: no arranca sin par
    i += di * dt;
    w += dw * dt;
    theta += w * dt;
  }
  return 0;
}
