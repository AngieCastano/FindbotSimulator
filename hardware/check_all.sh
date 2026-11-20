#!/usr/bin/env bash
# Verificación completa del hardware (Fase H7): corre las comprobaciones de H1–H6 y resume.
# Uso: hardware/check_all.sh [--e2e]
#   --e2e  agrega la prueba de punta a punta micro-ROS (Wokwi + agente en Docker, ~3 min)
# Las pruebas de Wokwi se omiten si falta WOKWI_CLI_TOKEN (correr con: bash -ic 'hardware/check_all.sh').
set -uo pipefail
cd "$(dirname "$0")"
PIO="${PIO:-$HOME/.local/bin/pio}"
E2E=0
[ "${1:-}" = "--e2e" ] && E2E=1
LOG_DIR=$(mktemp -d)
declare -a NAMES RESULTS

run() {  # run <fase> <descripción> <comando...>
  local phase="$1" desc="$2"; shift 2
  local log="$LOG_DIR/$(echo "$phase-$desc" | tr -c 'a-zA-Z0-9' '_').log"
  printf '%-4s %-45s ' "$phase" "$desc"
  if "$@" >"$log" 2>&1; then
    echo "OK"; RESULTS+=("OK")
  else
    echo "FALLA (ver $log)"; RESULTS+=("FALLA")
  fi
  NAMES+=("$phase $desc")
}

skip() { printf '%-4s %-45s OMITIDO (%s)\n' "$1" "$2" "$3"; NAMES+=("$1 $2"); RESULTS+=("OMITIDO"); }

echo "== Verificación del hardware de findbot =="
run H1 "pines y geometría (check_pins)"            python3 scripts/check_pins.py
run H2 "SPICE: 23 criterios (ngspice)"             spice/run_spice.sh
run H3 "esquemático: ERC + netlist + PDF/BOM"      kicad/build.sh
run H4 "config_gen.h al día con el YAML"           bash -c "cd firmware && python3 tools/gen_config.py --check"
run H4 "pruebas nativas (Unity)"                   bash -c "cd firmware && '$PIO' test -e native"
run H4 "compila esp32_debug (texto)"               bash -c "cd firmware && '$PIO' run -e esp32_debug"
run H6 "compila esp32 (micro-ROS)"                 bash -c "cd firmware && '$PIO' run -e esp32"
run H6 "compila esp32_e2e (micro-ROS, dominio 77)" bash -c "cd firmware && '$PIO' run -e esp32_e2e"
if [ -n "${WOKWI_CLI_TOKEN:-}" ]; then
  run H5 "Wokwi: escenario automático (111 pasos)"  wokwi/run_wokwi.sh
  if [ "$E2E" = 1 ]; then
    run H6 "punta a punta micro-ROS (7 verificaciones)" wokwi/run_microros_e2e.sh
  else
    skip H6 "punta a punta micro-ROS" "usar --e2e"
  fi
else
  skip H5 "Wokwi: escenario automático" "falta WOKWI_CLI_TOKEN"
  skip H6 "punta a punta micro-ROS" "falta WOKWI_CLI_TOKEN"
fi
run H7 "coherencia con simulación y CLAUDE.md"     python3 scripts/check_consistency.py
run H7 "lista de materiales con costos"            python3 bom/gen_bom.py

fails=0
for r in "${RESULTS[@]}"; do [ "$r" = "FALLA" ] && fails=$((fails + 1)); done
echo "== ${#RESULTS[@]} verificaciones, $fails fallas (registros en $LOG_DIR) =="
echo "Advertencias de coherencia (cambios pendientes en la simulación / fase 2):"
python3 scripts/check_consistency.py | grep ADVERTENCIA | sed 's/^/  /' || true
exit $((fails > 0))
