#include "analog_inputs.h"

#include <Arduino.h>

#include "config_gen.h"

namespace {
adc_attenuation_t attenuation(int db) {
  switch (db) {
    case 0: return ADC_0db;
    case 2: return ADC_2_5db;  // 2.5 dB en el YAML se escribe como 2
    case 6: return ADC_6db;
    default: return ADC_11db;
  }
}

float read_volts(int pin) { return static_cast<float>(analogReadMilliVolts(pin)) / 1000.0f; }
}  // namespace

void AnalogInputs::begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(cfg::PIN_ADC_VBAT, attenuation(cfg::BAT_ADC_ATTEN_DB));
  analogSetPinAttenuation(cfg::PIN_ADC_CO2, attenuation(cfg::CO2_ADC_ATTEN_DB));
  analogSetPinAttenuation(cfg::PIN_ADC_RADAR, attenuation(cfg::RADAR_ADC_ATTEN_DB));
  pinMode(cfg::PIN_CO2_HEATER_EN, OUTPUT);
  set_heater(false);
}

float AnalogInputs::battery_adc_volts() const { return read_volts(cfg::PIN_ADC_VBAT); }
float AnalogInputs::co2_adc_volts() const { return read_volts(cfg::PIN_ADC_CO2); }
float AnalogInputs::radar_adc_volts() const { return read_volts(cfg::PIN_ADC_RADAR); }

void AnalogInputs::set_heater(bool on) {
  digitalWrite(cfg::PIN_CO2_HEATER_EN, on ? HIGH : LOW);
  heater_ = on;
}
