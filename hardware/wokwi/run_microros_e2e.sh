#!/usr/bin/env bash
# Fase H6: prueba de punta a punta del firmware micro-ROS SIN el ESP32 físico.
#   ESP32 simulado (Wokwi, firmware esp32) ⇄ socat (puerto serial virtual) ⇄ micro-ROS agent
#   (Docker) ⇄ DDS ⇄ ros2 CLI (Docker). Usa un ROS_DOMAIN_ID aislado para no mezclarse con la
#   simulación de Gazebo (dominio 0): un /cmd_vel de prueba nunca mueve al robot simulado.
# Requiere: wokwi-cli + WOKWI_CLI_TOKEN, socat, docker con la imagen microros/micro-ros-agent:jazzy.
set -euo pipefail
cd "$(dirname "$0")"
DOMAIN="${E2E_ROS_DOMAIN_ID:-77}"
IMAGE="microros/micro-ros-agent:jazzy"
AGENT="findbot_uros_agent_e2e"
OUT="out/microros"
PIO="${PIO:-$HOME/.local/bin/pio}"
BAUD=$(python3 -c "import yaml;print(yaml.safe_load(open('../config/robot_hw.yaml'))['mcu']['usb_serial_baud'])")
mkdir -p "$OUT"
TMP=$(mktemp -d)

cleanup() {
  docker logs "$AGENT" >"$OUT/agent.log" 2>&1 || true
  docker rm -f "$AGENT" >/dev/null 2>&1 || true
  [ -n "${SOCAT_PID:-}" ] && kill "$SOCAT_PID" 2>/dev/null || true
  rm -rf "$TMP"
}
trap cleanup EXIT

command -v wokwi-cli >/dev/null && command -v socat >/dev/null || { echo "Faltan wokwi-cli o socat"; exit 2; }
[ -n "${WOKWI_CLI_TOKEN:-}" ] || { echo "Falta WOKWI_CLI_TOKEN"; exit 2; }

(cd ../firmware && "$PIO" run -e esp32_e2e | tail -2)
python3 gen_wokwi.py >/dev/null

# 1) ESP32 simulado: su serial queda en un pseudo-terminal ($TMP/tty). La traza del analizador
#    lógico (STBY, IN1/IN2...) se guarda al terminar la simulación.
socat PTY,link="$TMP/tty",raw,echo=0 \
  EXEC:"wokwi-cli -q --interactive --timeout 180000 --timeout-exit-code 0 --vcd-file $PWD/$OUT/e2e.vcd microros" \
  2>"$OUT/socat.log" &
SOCAT_PID=$!
for _ in $(seq 50); do [ -e "$TMP/tty" ] && break; sleep 0.2; done
PTY=$(readlink -f "$TMP/tty")
echo "ESP32 simulado en $PTY"
sleep 10
if ! kill -0 "$SOCAT_PID" 2>/dev/null; then
  # wokwi-cli terminó antes de tiempo (p. ej. cuota mensual de CI del plan gratuito agotada)
  echo "La simulación de Wokwi no arrancó:"; grep -v '^$' "$OUT/socat.log" | head -3
  exit 3
fi

# 2) Agente micro-ROS sobre ese puerto. Un pseudo-terminal del host no existe dentro del espacio
#    devpts del contenedor (--device da EIO): se monta /dev/pts del host y se permiten solo los
#    dispositivos de pseudo-terminal (major 136), sin --privileged.
docker run -d --rm --name "$AGENT" --net=host --ipc=host -e ROS_DOMAIN_ID="$DOMAIN" \
  -v /dev/pts:/dev/pts --device-cgroup-rule="c 136:* rmw" \
  "$IMAGE" serial --dev "$PTY" -b "$BAUD" >/dev/null

# 3) Verificación con la CLI de ROS 2 (otro contenedor, mismo dominio aislado)
docker run --rm --net=host --ipc=host -e ROS_DOMAIN_ID="$DOMAIN" --entrypoint bash "$IMAGE" -c '
  source /opt/ros/jazzy/setup.bash
  echo "== esperando al nodo findbot_esp32"
  timeout 300 bash -c "until [ \$(ros2 topic list 2>/dev/null | grep -cE \"^/(odom|tf|joint_states|imu/data_raw|battery_state|cmd_vel)\$\") -eq 6 ]; do sleep 1; done" || { echo "SIN_TOPICOS"; exit 1; }
  echo "== ros2 topic list -t"; ros2 topic list -t
  echo "== /odom";          timeout 20 ros2 topic echo --once /odom --field header
  timeout 20 ros2 topic echo --once /odom --field child_frame_id
  echo "== /joint_states";  timeout 20 ros2 topic echo --once /joint_states --field name
  echo "== /imu/data_raw";  timeout 20 ros2 topic echo --once /imu/data_raw --qos-reliability best_effort --field header.frame_id
  timeout 20 ros2 topic echo --once /imu/data_raw --qos-reliability best_effort --field linear_acceleration.z
  echo "== /battery_state"; timeout 20 ros2 topic echo --once /battery_state --field voltage
  timeout 20 ros2 topic echo --once /battery_state --field power_supply_health
  echo "== frecuencia /imu/data_raw"; PYTHONUNBUFFERED=1 timeout 15 ros2 topic hz /imu/data_raw 2>&1 | grep "average rate" | tail -1
  echo "== /cmd_vel 2 s a 10 Hz"; timeout 10 ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" >/dev/null
  echo "== FIN_ROS"
' | tee "$OUT/ros2_checks.log"

# 4) Fin de la simulación (timeout de wokwi-cli) para que escriba la traza VCD
echo "Esperando el fin de la simulación para leer la traza del analizador lógico..."
wait "$SOCAT_PID" || true
SOCAT_PID=""
python3 - "$OUT" <<'PY'
import re, sys
out = sys.argv[1]
log = open(f'{out}/ros2_checks.log', encoding='utf-8').read()
checks = {
    'tópicos con sus tipos': all(t in log for t in (
        '/odom [nav_msgs/msg/Odometry]', '/joint_states [sensor_msgs/msg/JointState]',
        '/imu/data_raw [sensor_msgs/msg/Imu]', '/battery_state [sensor_msgs/msg/BatteryState]',
        '/cmd_vel [geometry_msgs/msg/Twist]', '/tf [tf2_msgs/msg/TFMessage]')),
    'frames de /odom (odom → base_footprint)': "frame_id: odom" in log and 'base_footprint' in log,
    'nombres de /joint_states': 'left_wheel_joint' in log and 'right_wheel_joint' in log,
    'frame de /imu/data_raw (imu_link)': 'imu_link' in log,
    'aceleración z ≈ 9.81 m/s²': bool(re.search(r'^9\.[6-9]\d*|^10\.0', log, re.M)),
    # En Wokwi el serial pasa por la nube (~0.5–1 s de ida y vuelta): los tópicos confiables esperan
    # confirmación y la tasa total cae a ~1 Hz. Aquí solo se exige que la IMU llegue; los 50 Hz se
    # verifican con el ESP32 real por USB (ida y vuelta de milisegundos).
    '/imu/data_raw llega (tasa limitada por la latencia de Wokwi)': any(
        float(r) > 0 for r in re.findall(r'average rate: ([0-9.]+)', log)),
}
# Traza VCD: el canal D6 del analizador lógico es MOT_STBY → debe subir a 1 con /cmd_vel
vcd = open(f'{out}/e2e.vcd', encoding='utf-8', errors='replace').read()
ids = dict(re.findall(r'\$var wire 1 (\S+) (D\d) \$end', vcd))
inv = {v: k for k, v in ids.items()}
stby_id = inv.get('D6')
checks['/cmd_vel habilita los motores (STBY = 1 en la traza)'] = bool(
    stby_id and re.search(rf'^1{re.escape(stby_id)}$', vcd, re.M))
fails = 0
for name, ok in checks.items():
    print(f"{'OK   ' if ok else 'FALLA'} {name}")
    fails += not ok
print(f'{len(checks)} verificaciones, {fails} fallas')
sys.exit(1 if fails else 0)
PY
