// Pruebas del firmware en el lazo: modelo del motor, encoder y la cadena de control del ESP32
// (robot_core compilado para el PC) cerrada sobre una rueda simulada.
#include <gtest/gtest.h>

#include <cmath>

#include "config_gen.h"
#include "kinematics.h"
#include "motor_limit.h"
#include "pid.h"
#include "usar_firmware_sil/motor_model.hpp"
#include "watchdog.h"

namespace {
const sil::MotorParams kMotor{6.0, 26.18, 0.0785, 0.008};
}

TEST(MotorModel, StallAndNoLoadMatchDatasheet)
{
  // A 6 V: par de paro con la rueda quieta, ≈ 0 en vacío (menos la fricción).
  EXPECT_NEAR(sil::wheel_torque(kMotor, 1.0, 6.0, 0.0), 0.0785 - 0.008, 1e-9);
  EXPECT_NEAR(sil::wheel_torque(kMotor, 1.0, 6.0, 26.18), -0.008, 1e-9);
  // Duty 0 (freno): solo la fuerza contraelectromotriz se opone al giro.
  EXPECT_LT(sil::wheel_torque(kMotor, 0.0, 7.8, 10.0), 0.0);
  // Parado y con poco par: la fricción lo retiene.
  EXPECT_EQ(sil::wheel_torque(kMotor, 0.05, 7.8, 0.0), 0.0);
}

TEST(Encoder, QuantizesToCountsPerRevolution)
{
  EXPECT_EQ(sil::encoder_ticks(2.0 * M_PI, cfg::COUNTS_PER_WHEEL_REV), cfg::COUNTS_PER_WHEEL_REV);
  EXPECT_EQ(sil::encoder_ticks(0.001, 2112), 0);   // menos de una cuenta (2.97 mrad)
}

TEST(Watchdog, StopsAfterTimeout)
{
  core::CommandWatchdog wd(cfg::CMD_TIMEOUT_MS);
  EXPECT_TRUE(wd.expired(0));
  wd.feed(1000);
  EXPECT_FALSE(wd.expired(1000 + cfg::CMD_TIMEOUT_MS));
  EXPECT_TRUE(wd.expired(1000 + cfg::CMD_TIMEOUT_MS + 1));
}

TEST(ControlLoop, PidDrivesSimulatedWheelToSetpoint)
{
  // Una rueda con la inercia equivalente de robot_hw.yaml (j_eq) y el modelo del motor: la
  // cadena del firmware (prealimentación + PID + duty_from_command) llega a la consigna.
  const core::DiffDriveParams drive{cfg::WHEEL_RADIUS, cfg::WHEEL_SEPARATION, cfg::MAX_LIN_VEL,
    cfg::MAX_ANG_VEL, cfg::MAX_LIN_ACC, cfg::MAX_ANG_ACC};
  core::Pid pid({cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT});
  const double j_eq = 2.0e-5, dt = cfg::PERIOD_CONTROL_US * 1e-6;
  const float want = core::inverse_kinematics(drive, {0.2f, 0.0f}).left;   // 0.2 m/s
  double omega = 0.0;
  for (int k = 0; k < 150; ++k) {    // 3 s
    const float u = want / cfg::WHEEL_MAX_RAD_S + pid.update(want, static_cast<float>(omega),
        static_cast<float>(dt));
    const double duty = core::duty_from_command(u, 7.8f, cfg::MOTOR_V_LIMIT, cfg::MOTOR_DEADBAND);
    // Integración fina (el motor es mucho más rápido que el periodo de control)
    for (int s = 0; s < 20; ++s) {
      omega += sil::wheel_torque(kMotor, duty, 7.8, omega) / j_eq * dt / 20.0;
    }
  }
  EXPECT_NEAR(omega, want, 0.05 * want);
}
