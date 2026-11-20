#!/usr/bin/env bash
# Fase H5: compila el firmware, regenera el circuito Wokwi y corre la prueba automática.
# Requiere wokwi-cli y la variable WOKWI_CLI_TOKEN (https://wokwi.com/dashboard/ci).
set -euo pipefail
cd "$(dirname "$0")"
command -v wokwi-cli >/dev/null || { echo "Falta wokwi-cli (ver hardware/CLAUDE.md)"; exit 2; }
[ -n "${WOKWI_CLI_TOKEN:-}" ] || { echo "Falta la variable WOKWI_CLI_TOKEN"; exit 2; }
PIO="${PIO:-$HOME/.local/bin/pio}"
(cd ../firmware && "$PIO" run -e esp32_debug | tail -3)
python3 gen_wokwi.py
mkdir -p out
wokwi-cli lint .
wokwi-cli . --diagram-file diagram_ci.json --scenario scenario_h5.yaml --timeout 60000 --serial-log-file out/serial.log
echo "Prueba H5 superada (registro serial en hardware/wokwi/out/serial.log)"
