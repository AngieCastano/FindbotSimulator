#pragma once
// Monitor de batería: tensión en el ADC → tensión de batería filtrada (EMA), nivel con
// histéresis (OK / LOW / CRITICAL) y porcentaje lineal entre corte del BMS y carga completa.

namespace core {

enum class BatteryLevel { kOk, kLow, kCritical };

struct BatteryParams {
  float divider_ratio;  // V_adc / V_bat
  float v_full;
  float v_low;
  float v_critical;
  float v_cutoff;
  float alpha;       // filtro EMA (1 = sin filtro)
  float hysteresis;  // V para salir de un nivel
};

class BatteryMonitor {
 public:
  explicit BatteryMonitor(const BatteryParams& p) : p_(p) {}

  // adc_volts: tensión medida en el pin del ADC. Devuelve la tensión de batería filtrada.
  float update(float adc_volts);

  float voltage() const { return v_; }
  BatteryLevel level() const { return level_; }
  float percentage() const;  // 0..1
  bool valid() const { return initialized_; }

 private:
  BatteryParams p_;
  float v_ = 0.0f;
  bool initialized_ = false;
  BatteryLevel level_ = BatteryLevel::kOk;
};

}  // namespace core
