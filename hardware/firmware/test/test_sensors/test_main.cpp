// Pruebas nativas: batería, conversiones de la IMU, CO2 y RMS del radar.
#include <math.h>
#include <unity.h>

#include "battery.h"
#include "config_gen.h"
#include "sensors.h"

using namespace core;

void setUp() {}
void tearDown() {}

static const BatteryParams kBat{cfg::BAT_DIVIDER_RATIO, cfg::BAT_V_FULL,   cfg::BAT_V_LOW,
                                cfg::BAT_V_CRITICAL,    cfg::BAT_V_CUTOFF, 1.0f,
                                cfg::BAT_HYSTERESIS};

static float adc_for(float v_bat) { return v_bat * cfg::BAT_DIVIDER_RATIO; }

void test_battery_divider_matches_h2() {
  // 27k/10k: 8.4 V → 2.27 V en el ADC (verificado con ngspice en la H2)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.27f, adc_for(8.4f));
}

void test_battery_voltage_and_percentage() {
  BatteryMonitor bat(kBat);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 7.4f, bat.update(adc_for(7.4f)));
  TEST_ASSERT_EQUAL(BatteryLevel::kOk, bat.level());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, (7.4f - 6.0f) / (8.4f - 6.0f), bat.percentage());
}

void test_battery_levels_with_hysteresis() {
  BatteryMonitor bat(kBat);
  bat.update(adc_for(7.0f));
  TEST_ASSERT_EQUAL(BatteryLevel::kOk, bat.level());
  bat.update(adc_for(cfg::BAT_V_LOW - 0.05f));
  TEST_ASSERT_EQUAL(BatteryLevel::kLow, bat.level());
  bat.update(adc_for(cfg::BAT_V_LOW + 0.05f));  // dentro de la histéresis: sigue LOW
  TEST_ASSERT_EQUAL(BatteryLevel::kLow, bat.level());
  bat.update(adc_for(cfg::BAT_V_LOW + 0.15f));
  TEST_ASSERT_EQUAL(BatteryLevel::kOk, bat.level());
  bat.update(adc_for(cfg::BAT_V_CRITICAL - 0.05f));
  TEST_ASSERT_EQUAL(BatteryLevel::kCritical, bat.level());
  bat.update(adc_for(cfg::BAT_V_CRITICAL + 0.05f));  // no sale de crítico sin histéresis
  TEST_ASSERT_EQUAL(BatteryLevel::kCritical, bat.level());
}

void test_battery_filter_first_sample_unfiltered() {
  BatteryParams p = kBat;
  p.alpha = 0.2f;
  BatteryMonitor bat(p);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 8.0f, bat.update(adc_for(8.0f)));
  TEST_ASSERT_EQUAL(BatteryLevel::kOk, bat.level());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 7.8f, bat.update(adc_for(7.0f)));  // 8 + 0.2·(7 − 8)
}

void test_imu_scale_factors() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 16384.0f, mpu_accel_lsb_per_g(2));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 131.072f, mpu_gyro_lsb_per_dps(250));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kGravity, mpu_accel_to_ms2(16384, 2));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.14159f / 2.0f, mpu_gyro_to_rads(11796, 250));  // ≈ 90 °/s
}

void test_imu_fs_sel_codes() {
  TEST_ASSERT_EQUAL_INT(0, mpu_accel_fs_sel(cfg::IMU_ACCEL_RANGE_G));
  TEST_ASSERT_EQUAL_INT(0, mpu_gyro_fs_sel(cfg::IMU_GYRO_RANGE_DPS));
  TEST_ASSERT_EQUAL_INT(3, mpu_gyro_fs_sel(2000));
  TEST_ASSERT_EQUAL_INT(-1, mpu_accel_fs_sel(3));
}

void test_co2_baseline_and_decade() {
  Co2Estimator co2(cfg::CO2_GAIN, cfg::CO2_SLOPE_V_PER_DECADE, cfg::CO2_REF_PPM);
  TEST_ASSERT_TRUE(isnan(co2.ppm(1.0f)));  // sin calibrar no hay estimación
  const float base_adc = 0.35f * cfg::CO2_GAIN;  // FEM 0.35 V en aire limpio
  co2.calibrate(base_adc);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, cfg::CO2_REF_PPM, co2.ppm(base_adc));
  // Una década más de CO2 baja la FEM en "slope" voltios
  const float adc_10x = (0.35f - cfg::CO2_SLOPE_V_PER_DECADE) * cfg::CO2_GAIN;
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f * cfg::CO2_REF_PPM, co2.ppm(adc_10x));
}

void test_radar_rms_of_sine() {
  RunningRms rms(200);
  for (int i = 0; i < 400; ++i) rms.add(0.5f * sinf(2.0f * 3.14159265f * 10.0f * i / 200.0f));
  TEST_ASSERT_TRUE(rms.full());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f / sqrtf(2.0f), rms.rms());
}

void test_radar_rms_window_forgets() {
  RunningRms rms(10);
  for (int i = 0; i < 10; ++i) rms.add(1.0f);
  for (int i = 0; i < 10; ++i) rms.add(0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, rms.rms());
}

void test_radar_vref_matches_h2() {
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.65f, cfg::RADAR_VREF);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_battery_divider_matches_h2);
  RUN_TEST(test_battery_voltage_and_percentage);
  RUN_TEST(test_battery_levels_with_hysteresis);
  RUN_TEST(test_battery_filter_first_sample_unfiltered);
  RUN_TEST(test_imu_scale_factors);
  RUN_TEST(test_imu_fs_sel_codes);
  RUN_TEST(test_co2_baseline_and_decade);
  RUN_TEST(test_radar_rms_of_sine);
  RUN_TEST(test_radar_rms_window_forgets);
  RUN_TEST(test_radar_vref_matches_h2);
  return UNITY_END();
}
