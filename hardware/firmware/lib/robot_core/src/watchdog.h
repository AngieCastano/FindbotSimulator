#pragma once
// Vigilante de comandos: si no llega /cmd_vel en timeout_ms, el robot debe detenerse.
// Usa resta sin signo, así que funciona aunque millis() dé la vuelta (cada ~49 días).
#include <stdint.h>

namespace core {

class CommandWatchdog {
 public:
  explicit CommandWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void feed(uint32_t now_ms) {
    last_ms_ = now_ms;
    fed_ = true;
  }

  // Expirado también si nunca llegó un comando.
  bool expired(uint32_t now_ms) const { return !fed_ || (now_ms - last_ms_) > timeout_ms_; }

 private:
  uint32_t timeout_ms_;
  uint32_t last_ms_ = 0;
  bool fed_ = false;
};

}  // namespace core
