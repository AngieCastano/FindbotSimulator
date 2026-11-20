# Concentración de un gas medida por un sensor (p. ej. CO₂ con el Figaro TGS4161).
std_msgs/Header header          # frame_id: frame del sensor (co2_link)
string gas                      # fórmula: "CO2"
float32 concentration           # ppm (partes por millón en volumen)
float32 variance                # ppm²; 0 si se desconoce
