// Pruebas nativas: conversiones para los mensajes de micro-ROS y nombres de la configuración.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "config_gen.h"
#include "ros_convert.h"

using namespace core;

void setUp() {}
void tearDown() {}

void test_quaternion_identity() {
  const Quaternion q = yaw_to_quaternion(0.0f);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, q.z);
  TEST_ASSERT_EQUAL_DOUBLE(1.0, q.w);
}

void test_quaternion_quarter_turn_and_unit_norm() {
  const Quaternion q = yaw_to_quaternion(static_cast<float>(M_PI / 2));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, sqrt(0.5), q.z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, sqrt(0.5), q.w);
  for (float yaw = -3.1f; yaw < 3.1f; yaw += 0.37f) {
    const Quaternion r = yaw_to_quaternion(yaw);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, yaw, 2.0 * atan2(r.z, r.w));  // ida y vuelta
  }
}

void test_covariance6_diagonal_layout() {
  double m[36];
  covariance6_from_diagonal(cfg::COV_ODOM_POSE, m);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      const double expected = (r == c) ? static_cast<double>(cfg::COV_ODOM_POSE[r]) : 0.0;
      TEST_ASSERT_EQUAL_DOUBLE(expected, m[r * 6 + c]);
    }
  }
  TEST_ASSERT_EQUAL_DOUBLE(static_cast<double>(cfg::COV_ODOM_POSE[5]), m[35]);  // yaw
}

void test_covariance3_isotropic() {
  double m[9];
  covariance3_isotropic(0.5f, m);
  TEST_ASSERT_EQUAL_DOUBLE(0.5, m[0]);
  TEST_ASSERT_EQUAL_DOUBLE(0.5, m[4]);
  TEST_ASSERT_EQUAL_DOUBLE(0.5, m[8]);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, m[1]);
}

void test_split_nanos() {
  int32_t sec = -1;
  uint32_t ns = 1;
  split_nanos(1700000000123456789LL, sec, ns);
  TEST_ASSERT_EQUAL_INT32(1700000000, sec);
  TEST_ASSERT_EQUAL_UINT32(123456789u, ns);
  split_nanos(-5, sec, ns);
  TEST_ASSERT_EQUAL_INT32(0, sec);
  TEST_ASSERT_EQUAL_UINT32(0u, ns);
}

void test_battery_health_mapping() {
  TEST_ASSERT_EQUAL_UINT8(kPowerSupplyHealthGood, battery_health(BatteryLevel::kOk));
  TEST_ASSERT_EQUAL_UINT8(kPowerSupplyHealthGood, battery_health(BatteryLevel::kLow));
  TEST_ASSERT_EQUAL_UINT8(kPowerSupplyHealthDead, battery_health(BatteryLevel::kCritical));
}

void test_names_match_simulation() {
  // Deben coincidir con usar_description (xacro y bridge.yaml) de la sesión de simulación
  TEST_ASSERT_EQUAL_STRING("/cmd_vel", cfg::TOPIC_CMD_VEL);
  TEST_ASSERT_EQUAL_STRING("/odom", cfg::TOPIC_ODOM);
  TEST_ASSERT_EQUAL_STRING("/joint_states", cfg::TOPIC_JOINT_STATES);
  TEST_ASSERT_EQUAL_STRING("/imu/data_raw", cfg::TOPIC_IMU);
  TEST_ASSERT_EQUAL_STRING("/battery_state", cfg::TOPIC_BATTERY);
  TEST_ASSERT_EQUAL_STRING("odom", cfg::FRAME_ODOM);
  TEST_ASSERT_EQUAL_STRING("base_footprint", cfg::FRAME_BASE);
  TEST_ASSERT_EQUAL_STRING("left_wheel_joint", cfg::JOINT_LEFT);
  TEST_ASSERT_EQUAL_STRING("right_wheel_joint", cfg::JOINT_RIGHT);
  // El robot comparte el dominio DDS de la simulación; solo esp32_e2e usa el dominio aislado
  TEST_ASSERT_EQUAL_UINT32(0u, cfg::ROS_DOMAIN_ID);
}

void test_publish_rates_fit_serial_link() {
  // Tamaños aproximados serializados (bytes) × frecuencia ≤ 60 % de 921600 baudios (8N1)
  const double bytes_per_s = 725.0 * 1000.0 / cfg::PUB_PERIOD_ODOM_MS +
                             (cfg::PUBLISH_ODOM_TF ? 120.0 * 1000.0 / cfg::PUB_PERIOD_ODOM_MS : 0.0) +
                             330.0 * 1000.0 / cfg::PUB_PERIOD_IMU_MS +
                             120.0 * 1000.0 / cfg::PUB_PERIOD_JOINT_STATES_MS +
                             100.0 * 1000.0 / cfg::PUB_PERIOD_BATTERY_MS;
  const double link = cfg::USB_BAUD / 10.0;
  TEST_ASSERT_TRUE(bytes_per_s < 0.6 * link);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_quaternion_identity);
  RUN_TEST(test_quaternion_quarter_turn_and_unit_norm);
  RUN_TEST(test_covariance6_diagonal_layout);
  RUN_TEST(test_covariance3_isotropic);
  RUN_TEST(test_split_nanos);
  RUN_TEST(test_battery_health_mapping);
  RUN_TEST(test_names_match_simulation);
  RUN_TEST(test_publish_rates_fit_serial_link);
  return UNITY_END();
}
