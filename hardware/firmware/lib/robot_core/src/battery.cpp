#include "battery.h"

#include "util.h"

namespace core {

float BatteryMonitor::update(float adc_volts) {
  const float v_bat = adc_volts / p_.divider_ratio;
  if (!initialized_) {
    v_ = v_bat;  // primera muestra: sin filtro, para no arrancar en "crítico"
    initialized_ = true;
  } else {
    v_ += p_.alpha * (v_bat - v_);
  }

  // Bajar de nivel es inmediato; subir exige superar el umbral + histéresis.
  switch (level_) {
    case BatteryLevel::kOk:
      if (v_ < p_.v_critical) level_ = BatteryLevel::kCritical;
      else if (v_ < p_.v_low) level_ = BatteryLevel::kLow;
      break;
    case BatteryLevel::kLow:
      if (v_ < p_.v_critical) level_ = BatteryLevel::kCritical;
      else if (v_ > p_.v_low + p_.hysteresis) level_ = BatteryLevel::kOk;
      break;
    case BatteryLevel::kCritical:
      if (v_ > p_.v_low + p_.hysteresis) level_ = BatteryLevel::kOk;
      else if (v_ > p_.v_critical + p_.hysteresis) level_ = BatteryLevel::kLow;
      break;
  }
  return v_;
}

float BatteryMonitor::percentage() const {
  return clampf((v_ - p_.v_cutoff) / (p_.v_full - p_.v_cutoff), 0.0f, 1.0f);
}

}  // namespace core
