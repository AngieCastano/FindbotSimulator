#include "imu_mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

#include "config_gen.h"
#include "sensors.h"

namespace {
// Registros del MPU6050 (hoja de registros, rev. 4.2)
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegIntEnable = 0x38;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kWhoAmIValue = 0x68;
constexpr uint8_t kClockPllGyroX = 0x01;
constexpr uint8_t kIntDataReady = 0x01;
constexpr uint32_t kInternalRateHz = 1000;  // con el DLPF activo

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
}  // namespace

bool Mpu6050::write_reg(unsigned char reg, unsigned char value) {
  Wire.beginTransmission(cfg::IMU_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Mpu6050::begin() {
  Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL, cfg::IMU_I2C_HZ);
  Wire.beginTransmission(cfg::IMU_I2C_ADDRESS);
  Wire.write(kRegWhoAmI);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(cfg::IMU_I2C_ADDRESS, uint8_t{1}) != 1 ||
      Wire.read() != kWhoAmIValue) {
    ok_ = false;
    return false;
  }
  const int accel_sel = core::mpu_accel_fs_sel(cfg::IMU_ACCEL_RANGE_G);
  const int gyro_sel = core::mpu_gyro_fs_sel(cfg::IMU_GYRO_RANGE_DPS);
  const uint32_t rate_hz = 1000000u / cfg::PERIOD_IMU_US;
  const uint8_t div = static_cast<uint8_t>(kInternalRateHz / rate_hz - 1u);
  ok_ = accel_sel >= 0 && gyro_sel >= 0 &&
        write_reg(kRegPwrMgmt1, kClockPllGyroX) &&
        write_reg(kRegSmplrtDiv, div) &&
        write_reg(kRegConfig, cfg::IMU_DLPF_CFG) &&
        write_reg(kRegGyroConfig, static_cast<uint8_t>(gyro_sel << 3)) &&
        write_reg(kRegAccelConfig, static_cast<uint8_t>(accel_sel << 3)) &&
        write_reg(kRegIntEnable, kIntDataReady);
  return ok_;
}

bool Mpu6050::read(ImuSample& out) {
  if (!ok_) return false;
  uint8_t buf[14];
  Wire.beginTransmission(cfg::IMU_I2C_ADDRESS);
  Wire.write(kRegAccelXoutH);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(cfg::IMU_I2C_ADDRESS, uint8_t{14}) != 14) {
    return false;
  }
  for (uint8_t& b : buf) b = static_cast<uint8_t>(Wire.read());
  // buf[6..7] es la temperatura (no se usa)
  out.ax = core::mpu_accel_to_ms2(be16(&buf[0]), cfg::IMU_ACCEL_RANGE_G);
  out.ay = core::mpu_accel_to_ms2(be16(&buf[2]), cfg::IMU_ACCEL_RANGE_G);
  out.az = core::mpu_accel_to_ms2(be16(&buf[4]), cfg::IMU_ACCEL_RANGE_G);
  out.gx = core::mpu_gyro_to_rads(be16(&buf[8]), cfg::IMU_GYRO_RANGE_DPS) - bias_[0];
  out.gy = core::mpu_gyro_to_rads(be16(&buf[10]), cfg::IMU_GYRO_RANGE_DPS) - bias_[1];
  out.gz = core::mpu_gyro_to_rads(be16(&buf[12]), cfg::IMU_GYRO_RANGE_DPS) - bias_[2];
  return true;
}

void Mpu6050::calibrate_gyro(int samples) {
  if (!ok_ || samples <= 0) return;
  float sum[3] = {0.0f, 0.0f, 0.0f};
  bias_[0] = bias_[1] = bias_[2] = 0.0f;
  int n = 0;
  ImuSample s{};
  for (int i = 0; i < samples; ++i) {
    if (read(s)) {
      sum[0] += s.gx;
      sum[1] += s.gy;
      sum[2] += s.gz;
      ++n;
    }
    delayMicroseconds(cfg::PERIOD_IMU_US);
  }
  if (n > 0) {
    for (int k = 0; k < 3; ++k) bias_[k] = sum[k] / static_cast<float>(n);
  }
}
