#pragma once
// Entradas analógicas (ADC1 con calibración de fábrica: analogReadMilliVolts) y calefactor:
// batería (GPIO36), CO2 TGS4161 (GPIO39), radar HB100 (GPIO34), calefactor (GPIO4).

class AnalogInputs {
 public:
  void begin();
  float battery_adc_volts() const;
  float co2_adc_volts() const;
  float radar_adc_volts() const;
  void set_heater(bool on);
  bool heater() const { return heater_; }

 private:
  bool heater_ = false;
};
