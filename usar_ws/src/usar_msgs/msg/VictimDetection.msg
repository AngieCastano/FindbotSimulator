# Víctima confirmada por el detector (frame del encabezado de VictimDetectionArray).
uint32 id                       # número estable de la víctima (se conserva entre mensajes)
geometry_msgs/Point position    # posición estimada (m)
float32 confidence              # 0..1
uint32 observations             # capturas que la sustentan
string[] sources                # detectores (modalidades) que la confirmaron: color_shape,
                                # human_shape, thermal, co2, ...
