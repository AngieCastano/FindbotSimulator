# Señal IF del radar Doppler HB100 tras el acondicionamiento (2 etapas, 66,8 dB, 0,11–43 Hz).
# La publica el ESP32 (ADC a 200 Hz, promedio de bloques de 10 muestras → sample_rate) o, en
# simulación, usar_sim_sensors/radar_sim. Solo la medida: sin posiciones ni distancias.
std_msgs/Header header          # frame_id: radar_link; stamp: instante de la ÚLTIMA muestra
float32 sample_rate             # Hz (20)
float32[] samples               # V en el ADC respecto a VREF, de la más antigua a la más reciente
bool saturated                  # alguna muestra del bloque llegó al límite del ADC
