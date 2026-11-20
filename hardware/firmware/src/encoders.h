#pragma once
// Encoders de cuadratura de las 2 ruedas (×4).
// Backend PCNT (contador por hardware, por defecto) o ISR (interrupciones, respaldo para
// simuladores sin PCNT); se elige en robot_hw.yaml → firmware.encoder_backend.
#include <stdint.h>

class Encoders {
 public:
  void begin();
  // Cuentas acumuladas desde la llamada anterior (signo ya corregido por la inversión del YAML).
  void read_deltas(int32_t& left, int32_t& right);
};
