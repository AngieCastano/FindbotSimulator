#!/usr/bin/env bash
# Corre todas las simulaciones SPICE de la Fase H2 y verifica los criterios (criteria.yaml).
# Resultados: hardware/spice/results/ (summary.txt, gráficos .svg, logs de ngspice).
set -euo pipefail
exec python3 "$(dirname "$0")/check_spice.py" "$@"
