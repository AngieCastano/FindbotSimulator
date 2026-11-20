#pragma once
// Conversiones de sensores, sin dependencias de hardware:
// - IMU MPU6050: cuentas crudas → m/s² y rad/s según el rango configurado.
// - CO2 TGS4161: tensión del ADC → FEM del sensor → ppm relativo a una línea base calibrada.
// - Radar HB100: muestra del ADC → señal centrada en VREF y su valor RMS en una ventana.
#include <stdint.h>

namespace core {

constexpr float kGravity = 9.80665f;

// Factores de escala del MPU6050 (datasheet, tabla de sensibilidad).
inline float mpu_accel_lsb_per_g(int range_g) { return 32768.0f / static_cast<float>(range_g); }
inline float mpu_gyro_lsb_per_dps(int range_dps) { return 32768.0f / static_cast<float>(range_dps); }
inline float mpu_accel_to_ms2(int16_t raw, int range_g) {
  return static_cast<float>(raw) / mpu_accel_lsb_per_g(range_g) * kGravity;
}
inline float mpu_gyro_to_rads(int16_t raw, int range_dps) {
  return static_cast<float>(raw) / mpu_gyro_lsb_per_dps(range_dps) * 0.0174532925f;
}
// Código de registro GYRO_CONFIG/ACCEL_CONFIG (bits FS_SEL) para un rango; −1 si no es válido.
int mpu_accel_fs_sel(int range_g);
int mpu_gyro_fs_sel(int range_dps);

class Co2Estimator {
 public:
  Co2Estimator(float gain, float slope_v_per_decade, float ref_ppm)
      : gain_(gain), slope_(slope_v_per_decade), ref_ppm_(ref_ppm) {}

  float emf(float adc_volts) const { return adc_volts / gain_; }
  // Fija la línea base: la FEM actual corresponde a ref_ppm (aire limpio).
  void calibrate(float adc_volts) {
    base_emf_ = emf(adc_volts);
    calibrated_ = true;
  }
  bool calibrated() const { return calibrated_; }
  // La FEM del TGS4161 BAJA cuando sube el CO2 (≈ slope V por década).
  float ppm(float adc_volts) const;

 private:
  float gain_, slope_, ref_ppm_;
  float base_emf_ = 0.0f;
  bool calibrated_ = false;
};

// RMS en ventana deslizante de N muestras (N ≤ kMaxWindow), con suma acumulada O(1).
class RunningRms {
 public:
  static constexpr int kMaxWindow = 512;
  explicit RunningRms(int window);
  float add(float sample);
  float rms() const;
  bool full() const { return count_ >= window_; }

 private:
  int window_;
  int index_ = 0;
  int count_ = 0;
  double sum_sq_ = 0.0;
  float buffer_[kMaxWindow] = {};
};

}  // namespace core
