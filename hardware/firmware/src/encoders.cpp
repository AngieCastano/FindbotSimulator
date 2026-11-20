#include "encoders.h"

#include <Arduino.h>
#include <driver/pcnt.h>

#include "config_gen.h"

namespace {

// ---------------- Backend PCNT ----------------
// El contador es de 16 bits: al llegar a ±kLimit vuelve a 0 y una interrupción suma ±kLimit
// al acumulador. Si se lee justo después del desborde y antes de la interrupción, el salto
// de ±kLimit se corrige en read() (las cuentas reales por periodo son ≪ kLimit/2).
constexpr int16_t kLimit = 30000;

struct PcntWheel {
  pcnt_unit_t unit;
  volatile int32_t overflow = 0;
  int32_t prev_total = 0;
};

PcntWheel g_pcnt[2] = {{PCNT_UNIT_0}, {PCNT_UNIT_1}};
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pcnt_overflow_isr(void* arg) {
  auto* w = static_cast<PcntWheel*>(arg);
  uint32_t status = 0;
  pcnt_get_event_status(w->unit, &status);
  portENTER_CRITICAL_ISR(&g_mux);
  if (status & PCNT_EVT_H_LIM) w->overflow += kLimit;
  if (status & PCNT_EVT_L_LIM) w->overflow -= kLimit;
  portEXIT_CRITICAL_ISR(&g_mux);
}

void pcnt_setup(PcntWheel& w, int pin_a, int pin_b) {
  // Configuración ×4 estándar (ejemplo "rotary encoder" de ESP-IDF)
  pcnt_config_t c = {};
  c.pulse_gpio_num = pin_a;
  c.ctrl_gpio_num = pin_b;
  c.channel = PCNT_CHANNEL_0;
  c.unit = w.unit;
  c.pos_mode = PCNT_COUNT_DEC;
  c.neg_mode = PCNT_COUNT_INC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = kLimit;
  c.counter_l_lim = -kLimit;
  pcnt_unit_config(&c);
  c.pulse_gpio_num = pin_b;
  c.ctrl_gpio_num = pin_a;
  c.channel = PCNT_CHANNEL_1;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&c);

  pcnt_set_filter_value(w.unit, cfg::ENCODER_GLITCH_FILTER);
  pcnt_filter_enable(w.unit);
  pcnt_event_enable(w.unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(w.unit, PCNT_EVT_L_LIM);
  pcnt_counter_pause(w.unit);
  pcnt_counter_clear(w.unit);
  pcnt_isr_handler_add(w.unit, pcnt_overflow_isr, &w);
  pcnt_counter_resume(w.unit);
}

int32_t pcnt_delta(PcntWheel& w) {
  int16_t count = 0;
  portENTER_CRITICAL(&g_mux);
  pcnt_get_counter_value(w.unit, &count);
  const int32_t total = w.overflow + count;
  portEXIT_CRITICAL(&g_mux);
  int32_t delta = total - w.prev_total;
  if (delta > kLimit / 2) delta -= kLimit;   // desborde aún no sumado por la ISR
  if (delta < -kLimit / 2) delta += kLimit;
  w.prev_total += delta;
  return delta;
}

// ---------------- Backend ISR ----------------
// Tabla de transición de cuadratura: índice = (estado_anterior << 2) | estado_nuevo.
constexpr int8_t kQuad[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

struct IsrWheel {
  int pin_a, pin_b;
  volatile int32_t count = 0;
  volatile uint8_t state = 0;
  int32_t prev = 0;
};

IsrWheel g_isr[2] = {{cfg::PIN_ENC_L_A, cfg::PIN_ENC_L_B}, {cfg::PIN_ENC_R_A, cfg::PIN_ENC_R_B}};

void IRAM_ATTR isr_edge(void* arg) {
  auto* w = static_cast<IsrWheel*>(arg);
  const uint8_t s = static_cast<uint8_t>((digitalRead(w->pin_a) << 1) | digitalRead(w->pin_b));
  w->count += kQuad[(w->state << 2) | s];
  w->state = s;
}

void isr_setup(IsrWheel& w) {
  pinMode(w.pin_a, INPUT);
  pinMode(w.pin_b, INPUT);
  w.state = static_cast<uint8_t>((digitalRead(w.pin_a) << 1) | digitalRead(w.pin_b));
  attachInterruptArg(digitalPinToInterrupt(w.pin_a), isr_edge, &w, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(w.pin_b), isr_edge, &w, CHANGE);
}

int32_t isr_delta(IsrWheel& w) {
  noInterrupts();
  const int32_t now = w.count;
  interrupts();
  const int32_t delta = now - w.prev;
  w.prev = now;
  return delta;
}

}  // namespace

void Encoders::begin() {
  if (cfg::ENCODER_USE_PCNT) {
    pcnt_isr_service_install(0);
    pcnt_setup(g_pcnt[0], cfg::PIN_ENC_L_A, cfg::PIN_ENC_L_B);
    pcnt_setup(g_pcnt[1], cfg::PIN_ENC_R_A, cfg::PIN_ENC_R_B);
  } else {
    isr_setup(g_isr[0]);
    isr_setup(g_isr[1]);
  }
}

void Encoders::read_deltas(int32_t& left, int32_t& right) {
  if (cfg::ENCODER_USE_PCNT) {
    left = pcnt_delta(g_pcnt[0]);
    right = pcnt_delta(g_pcnt[1]);
  } else {
    left = isr_delta(g_isr[0]);
    right = isr_delta(g_isr[1]);
  }
  if (cfg::ENCODER_INVERT_LEFT) left = -left;
  if (cfg::ENCODER_INVERT_RIGHT) right = -right;
}
