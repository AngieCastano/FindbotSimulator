#pragma once
// MPU6050 por I2C, leído directamente por registros (sin librerías externas).
// Unidades de salida: m/s² y rad/s (las que espera sensor_msgs/Imu en /imu/data_raw).

struct ImuSample {
  float ax, ay, az;  // m/s²
  float gx, gy, gz;  // rad/s, sin el sesgo calibrado
};

class Mpu6050 {
 public:
  bool begin();                  // configura rangos, filtro y frecuencia; false si no responde
  bool read(ImuSample& out);     // lectura en ráfaga de 14 bytes
  void calibrate_gyro(int samples);  // promedio con el robot quieto
  bool ok() const { return ok_; }

 private:
  bool write_reg(unsigned char reg, unsigned char value);
  bool ok_ = false;
  float bias_[3] = {0.0f, 0.0f, 0.0f};
};
