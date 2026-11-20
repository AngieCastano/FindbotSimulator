#include "motors.h"

#include <Arduino.h>

#include "config_gen.h"

namespace {
constexpr int kChLeft = 0;   // canales LEDC
constexpr int kChRight = 1;
constexpr uint32_t kDutyMax = (1u << cfg::MOTOR_PWM_BITS) - 1u;
}  // namespace

void Motors::begin() {
  pinMode(cfg::PIN_MOT_STBY, OUTPUT);
  digitalWrite(cfg::PIN_MOT_STBY, LOW);
  for (int pin : {cfg::PIN_MOT_L_IN1, cfg::PIN_MOT_L_IN2, cfg::PIN_MOT_R_IN1, cfg::PIN_MOT_R_IN2}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  ledcSetup(kChLeft, cfg::MOTOR_PWM_FREQ, cfg::MOTOR_PWM_BITS);
  ledcSetup(kChRight, cfg::MOTOR_PWM_FREQ, cfg::MOTOR_PWM_BITS);
  ledcAttachPin(cfg::PIN_MOT_L_PWM, kChLeft);
  ledcAttachPin(cfg::PIN_MOT_R_PWM, kChRight);
  ledcWrite(kChLeft, 0);
  ledcWrite(kChRight, 0);
}

void Motors::enable(bool on) {
  if (!on) {
    set(0.0f, 0.0f);
  }
  digitalWrite(cfg::PIN_MOT_STBY, on ? HIGH : LOW);
  enabled_ = on;
}

void Motors::set(float duty_left, float duty_right) {
  apply(kChLeft, cfg::PIN_MOT_L_IN1, cfg::PIN_MOT_L_IN2, duty_left, cfg::MOTOR_INVERT_LEFT);
  apply(kChRight, cfg::PIN_MOT_R_IN1, cfg::PIN_MOT_R_IN2, duty_right, cfg::MOTOR_INVERT_RIGHT);
}

void Motors::apply(int ch, int in1, int in2, float duty, bool invert) {
  if (invert) {
    duty = -duty;
  }
  const float mag = fminf(fabsf(duty), 1.0f);
  if (mag == 0.0f) {
    // Rueda libre (IN1 = IN2 = 0): sin frenado brusco al soltar el mando
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }
  digitalWrite(in1, duty > 0.0f ? HIGH : LOW);
  digitalWrite(in2, duty > 0.0f ? LOW : HIGH);
  ledcWrite(ch, static_cast<uint32_t>(mag * kDutyMax + 0.5f));
}
