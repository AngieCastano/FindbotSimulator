// Pruebas nativas: PID, límite del PWM y vigilante de comandos.
#include <unity.h>

#include "config_gen.h"
#include "motor_limit.h"
#include "pid.h"
#include "watchdog.h"

using namespace core;

void setUp() {}
void tearDown() {}

static const PidGains kGains{0.5f, 2.0f, 0.0f, 0.4f, 1.0f};

void test_pid_proportional_only() {
  Pid pid({2.0f, 0.0f, 0.0f, 1.0f, 10.0f});
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, pid.update(1.0f, 0.0f, 0.02f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, pid.update(0.0f, 0.5f, 0.02f));
}

void test_pid_integral_accumulates() {
  Pid pid({0.0f, 1.0f, 0.0f, 10.0f, 10.0f});
  for (int i = 0; i < 50; ++i) pid.update(1.0f, 0.0f, 0.02f);  // 1 s con error 1
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, pid.integral_term());
}

void test_pid_output_is_clamped() {
  Pid pid(kGains);
  const float out = pid.update(100.0f, 0.0f, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kGains.out_limit, out);
}

void test_pid_anti_windup() {
  // Con la salida saturada mucho tiempo, el integrador no debe crecer sin límite:
  // al invertir el error la salida debe responder de inmediato.
  Pid pid(kGains);
  for (int i = 0; i < 1000; ++i) pid.update(100.0f, 0.0f, 0.02f);
  TEST_ASSERT_TRUE(pid.integral_term() <= kGains.i_limit + 1e-6f);
  const float out = pid.update(0.0f, 1.0f, 0.02f);
  TEST_ASSERT_TRUE(out < 0.0f);
}

void test_pid_closed_loop_converges() {
  // Planta de primer orden y' = (u·K − y)/τ: el PID debe llevar y a la consigna sin error.
  Pid pid({0.3f, 3.0f, 0.0f, 1.0f, 1.0f});
  const float K = 20.0f, tau = 0.1f, dt = 0.02f, target = 8.0f;
  float y = 0.0f;
  for (int i = 0; i < 250; ++i) {
    const float u = pid.update(target, y, dt);
    y += dt * (u * K - y) / tau;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, target, y);
}

void test_pid_zero_dt_is_safe() {
  Pid pid(kGains);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(1.0f, 0.0f, 0.0f));
}

void test_duty_zero_command() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, duty_from_command(0.0f, 8.4f, 6.0f, 0.12f));
}

void test_duty_never_exceeds_rated_voltage() {
  // Con batería llena (8.4 V) el duty máximo es 6/8.4: el motor ve como máximo 6 V.
  const float d = duty_from_command(1.0f, 8.4f, 6.0f, 0.12f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 6.0f / 8.4f, d);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.0f, d * 8.4f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -6.0f / 8.4f, duty_from_command(-5.0f, 8.4f, 6.0f, 0.12f));
}

void test_duty_low_battery_saturates_at_one() {
  // Con batería por debajo de la tensión nominal el duty se limita a 1.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, duty_from_command(1.0f, 5.5f, 6.0f, 0.12f));
}

void test_duty_deadband_and_monotonic() {
  const float small = duty_from_command(0.01f, 7.4f, 6.0f, 0.12f);
  TEST_ASSERT_TRUE(small >= 0.12f);
  float prev = 0.0f;
  for (int i = 1; i <= 100; ++i) {
    const float d = duty_from_command(i / 100.0f, 7.4f, 6.0f, 0.12f);
    TEST_ASSERT_TRUE(d > prev);
    prev = d;
  }
}

void test_duty_invalid_battery() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, duty_from_command(1.0f, 0.0f, 6.0f, 0.12f));
}

void test_watchdog_expired_until_fed() {
  CommandWatchdog wd(500);
  TEST_ASSERT_TRUE(wd.expired(0));
  wd.feed(1000);
  TEST_ASSERT_FALSE(wd.expired(1400));
  TEST_ASSERT_FALSE(wd.expired(1500));
  TEST_ASSERT_TRUE(wd.expired(1501));
}

void test_watchdog_millis_wraparound() {
  CommandWatchdog wd(500);
  wd.feed(0xFFFFFF00u);
  TEST_ASSERT_FALSE(wd.expired(0x00000010u));  // 272 ms después, con desborde
  TEST_ASSERT_TRUE(wd.expired(0x00000200u));   // 768 ms después
}

void test_config_timeout_and_pid_from_yaml() {
  TEST_ASSERT_TRUE(cfg::CMD_TIMEOUT_MS > 0);
  TEST_ASSERT_TRUE(cfg::PID_OUT_LIMIT > 0.0f);
  TEST_ASSERT_TRUE(cfg::MOTOR_DEADBAND >= 0.0f && cfg::MOTOR_DEADBAND < 0.5f);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_pid_proportional_only);
  RUN_TEST(test_pid_integral_accumulates);
  RUN_TEST(test_pid_output_is_clamped);
  RUN_TEST(test_pid_anti_windup);
  RUN_TEST(test_pid_closed_loop_converges);
  RUN_TEST(test_pid_zero_dt_is_safe);
  RUN_TEST(test_duty_zero_command);
  RUN_TEST(test_duty_never_exceeds_rated_voltage);
  RUN_TEST(test_duty_low_battery_saturates_at_one);
  RUN_TEST(test_duty_deadband_and_monotonic);
  RUN_TEST(test_duty_invalid_battery);
  RUN_TEST(test_watchdog_expired_until_fed);
  RUN_TEST(test_watchdog_millis_wraparound);
  RUN_TEST(test_config_timeout_and_pid_from_yaml);
  return UNITY_END();
}
