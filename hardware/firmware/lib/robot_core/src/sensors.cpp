#include "sensors.h"

#include <math.h>

namespace core {

int mpu_accel_fs_sel(int range_g) {
  switch (range_g) {
    case 2: return 0;
    case 4: return 1;
    case 8: return 2;
    case 16: return 3;
    default: return -1;
  }
}

int mpu_gyro_fs_sel(int range_dps) {
  switch (range_dps) {
    case 250: return 0;
    case 500: return 1;
    case 1000: return 2;
    case 2000: return 3;
    default: return -1;
  }
}

float Co2Estimator::ppm(float adc_volts) const {
  if (!calibrated_ || slope_ <= 0.0f) {
    return NAN;
  }
  const float decades = (base_emf_ - emf(adc_volts)) / slope_;
  return ref_ppm_ * powf(10.0f, decades);
}

RunningRms::RunningRms(int window)
    : window_(window < 1 ? 1 : (window > kMaxWindow ? kMaxWindow : window)) {}

float RunningRms::add(float sample) {
  const float sq = sample * sample;
  sum_sq_ += sq - buffer_[index_];
  buffer_[index_] = sq;
  index_ = (index_ + 1) % window_;
  if (count_ < window_) ++count_;
  if (sum_sq_ < 0.0) sum_sq_ = 0.0;  // errores de redondeo
  return rms();
}

float RunningRms::rms() const {
  return count_ == 0 ? 0.0f : static_cast<float>(sqrt(sum_sq_ / count_));
}

}  // namespace core
