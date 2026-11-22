# Calidad del enlace WiFi robot ↔ estación remota, como la reporta la radio del robot.
# En simulación la calcula usar_sim_sensors/comm_model (pérdida por trayecto y escombros); en el
# robot real, el driver WiFi. Solo la medida: sin posiciones ni distancias.
std_msgs/Header header
float32 rssi_dbm                # potencia recibida (dBm)
