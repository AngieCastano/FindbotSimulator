#pragma once
// Conversión del mando del controlador a duty del PWM.
// u ∈ [−1, 1] es la fracción de la tensión nominal del motor (6 V del TT). Con batería de 8.4 V
// el duty máximo queda en 6/8.4 para no sobrepasar la tensión nominal. La zona muerta se
// compensa: cualquier |u| > 0 arranca en "deadband" y crece linealmente hasta el duty máximo.

namespace core {

float duty_from_command(float u, float v_bat, float v_rated, float deadband);

}  // namespace core
