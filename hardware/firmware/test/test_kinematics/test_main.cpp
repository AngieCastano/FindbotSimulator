// Pruebas nativas: limitador de velocidad, cinemática inversa/directa y odometría.
#include <math.h>
#include <unity.h>

#include "config_gen.h"
#include "kinematics.h"

using namespace core;

void setUp() {}
void tearDown() {}

static const DiffDriveParams kP{cfg::WHEEL_RADIUS, cfg::WHEEL_SEPARATION, cfg::MAX_LIN_VEL,
                                cfg::MAX_ANG_VEL,  cfg::MAX_LIN_ACC,      cfg::MAX_ANG_ACC};
static const int32_t kCpr = cfg::COUNTS_PER_WHEEL_REV;

// Cuentas que corresponden a recorrer "dist" metros con una rueda.
static int32_t ticks_for(float dist) {
  return static_cast<int32_t>(lroundf(dist / (2.0f * 3.14159265f * kP.wheel_radius) * kCpr));
}

void test_config_counts_per_rev() {
  TEST_ASSERT_EQUAL_INT32(11 * 48 * 4, cfg::COUNTS_PER_WHEEL_REV);
}

void test_inverse_straight() {
  const WheelSpeeds w = inverse_kinematics(kP, {0.2f, 0.0f});
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f / kP.wheel_radius, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, w.left, w.right);
}

void test_inverse_rotate_in_place() {
  const WheelSpeeds w = inverse_kinematics(kP, {0.0f, 1.0f});
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -w.right, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f * kP.wheel_separation / kP.wheel_radius, w.right);
}

void test_forward_is_inverse_of_inverse() {
  const Twist2D cmd{0.17f, -0.8f};
  const Twist2D back = forward_kinematics(kP, inverse_kinematics(kP, cmd));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, cmd.v, back.v);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, cmd.w, back.w);
}

void test_limiter_clamps_to_max() {
  VelocityLimiter lim(kP);
  Twist2D out{0, 0};
  for (int i = 0; i < 500; ++i) out = lim.update({5.0f, -9.0f}, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kP.max_lin_vel, out.v);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -kP.max_ang_vel, out.w);
}

void test_limiter_respects_acceleration() {
  VelocityLimiter lim(kP);
  const Twist2D out = lim.update({kP.max_lin_vel, kP.max_ang_vel}, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kP.max_lin_acc * 0.02f, out.v);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kP.max_ang_acc * 0.02f, out.w);
}

void test_odometry_straight_one_meter() {
  Odometry odo(kP, kCpr);
  const int32_t step = ticks_for(0.01f);  // 1 cm por periodo
  for (int i = 0; i < 100; ++i) odo.update(step, step, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, odo.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, odo.y());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, odo.theta());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, odo.v());  // 1 cm / 20 ms
}

void test_odometry_rotate_half_turn() {
  // Giro en el lugar de π rad: cada rueda recorre π·L/2 en sentidos opuestos.
  Odometry odo(kP, kCpr);
  const float arc = 3.14159265f * kP.wheel_separation / 2.0f;
  const int32_t total = ticks_for(arc);
  const int n = 50;
  int32_t done = 0;
  for (int i = 1; i <= n; ++i) {
    const int32_t target = total * i / n;
    odo.update(-(target - done), target - done, 0.02f);
    done = target;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.14159265f, fabsf(odo.theta()));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, odo.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, odo.y());
}

void test_odometry_quarter_circle_arc() {
  // Arco de radio R = 0.5 m y 90°: debe terminar en (R, R) con θ = π/2.
  Odometry odo(kP, kCpr);
  const float R = 0.5f, half = kP.wheel_separation / 2.0f, angle = 3.14159265f / 2.0f;
  const int n = 200;
  const float dl = (R - half) * angle / n, dr = (R + half) * angle / n;
  float acc_l = 0.0f, acc_r = 0.0f;
  int32_t sent_l = 0, sent_r = 0;
  for (int i = 0; i < n; ++i) {
    acc_l += dl;
    acc_r += dr;
    const int32_t tl = ticks_for(acc_l), tr = ticks_for(acc_r);
    odo.update(tl - sent_l, tr - sent_r, 0.02f);
    sent_l = tl;
    sent_r = tr;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, R, odo.x());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, R, odo.y());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, angle, odo.theta());
}

void test_odometry_wheel_positions_for_joint_states() {
  Odometry odo(kP, kCpr);
  odo.update(kCpr, -kCpr / 2, 0.02f);  // izquierda 1 vuelta, derecha −½ vuelta
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * 3.14159265f, odo.wheel_positions().left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3.14159265f, odo.wheel_positions().right);
}

void test_theta_stays_wrapped() {
  Odometry odo(kP, kCpr);
  const int32_t t = ticks_for(3.14159265f * kP.wheel_separation / 2.0f);  // π por llamada
  for (int i = 0; i < 7; ++i) odo.update(-t, t, 0.02f);
  TEST_ASSERT_TRUE(odo.theta() <= 3.1416f && odo.theta() > -3.1416f);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_config_counts_per_rev);
  RUN_TEST(test_inverse_straight);
  RUN_TEST(test_inverse_rotate_in_place);
  RUN_TEST(test_forward_is_inverse_of_inverse);
  RUN_TEST(test_limiter_clamps_to_max);
  RUN_TEST(test_limiter_respects_acceleration);
  RUN_TEST(test_odometry_straight_one_meter);
  RUN_TEST(test_odometry_rotate_half_turn);
  RUN_TEST(test_odometry_quarter_circle_arc);
  RUN_TEST(test_odometry_wheel_positions_for_joint_states);
  RUN_TEST(test_theta_stays_wrapped);
  return UNITY_END();
}
