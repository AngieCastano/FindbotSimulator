# Dirección de llegada del sonido estimada por un arreglo de micrófonos (p. ej. ReSpeaker USB
# Mic Array v2.0, que la calcula en su propio procesador junto con la detección de voz).
std_msgs/Header header          # frame_id: frame del arreglo (mic_array_link)
float32 azimuth                 # rad, en el plano del arreglo, desde +x, antihorario
float32 level_db                # nivel sonoro total medido (dB SPL)
bool voice_activity             # el arreglo detecta voz o golpes (no solo ruido de fondo)
float32 confidence              # 0..1: confianza de la dirección (relación señal/ruido)
