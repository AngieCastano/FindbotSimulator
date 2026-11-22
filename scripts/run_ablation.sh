#!/usr/bin/env bash
# Experimentos de ablación de la Fase 13: corre cada configuración de sensores con las mismas
# semillas (comparación pareada) y deja los resultados en results/ablation/<escenario>/<config>/.
#
# Uso (dentro del contenedor, con el workspace compilado):
#   scripts/run_ablation.sh hard "A B C D E F G H" "1 2 3 4 5"
#   scripts/run_ablation.sh dusty "A F" "1 2 3 4 5"
#   scripts/run_ablation.sh hard "F" "6"          # una sola misión
# Después: python3 scripts/ablation.py results/ablation/hard
#
# Configuraciones:
#   A  solo cámara (forma humana y color)              línea base
#   B  A + cámara térmica
#   C  A + sensor de CO₂
#   D  A + micrófonos (guía) y paradas de 5 s
#   E  A + radar Doppler (guía) y paradas de 11 s
#   F  todos los sensores, fusión por puntaje
#   G  F + término γ·S(g) en el explorador (explorer_gamma.yaml)
#   H  todos los signos de vida, sin detectores de la cámara
#
# Una configuración ya terminada para una semilla (existe su JSON) no se repite: se puede cortar
# y reanudar. Variables: MISSION_TIMEOUT (s reales por misión, por defecto 1500).
set -u

WORLD="${1:?escenario (p. ej. hard o dusty)}"
CONFIGS="${2:-A B C D E F G H}"
SEEDS="${3:-1 2 3 4 5}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="$REPO/results/ablation/$WORLD"
export MISSION_TIMEOUT="${MISSION_TIMEOUT:-1500}"
GAMMA_YAML="$REPO/usar_ws/src/usar_exploration/config/explorer_gamma.yaml"

config_env() {
  # Variables de run_batch.sh de cada configuración.
  case "$1" in
    A) echo "" ;;
    B) echo "USE_THERMAL=true" ;;
    C) echo "USE_CO2=true" ;;
    D) echo "USE_SOUND=true" ;;
    E) echo "USE_RADAR=true" ;;
    F) echo "USE_THERMAL=true USE_CO2=true USE_SOUND=true USE_RADAR=true" ;;
    G) echo "USE_THERMAL=true USE_CO2=true USE_SOUND=true USE_RADAR=true EXPLORER_OVERLAY=$GAMMA_YAML" ;;
    H) echo "USE_THERMAL=true USE_CO2=true USE_SOUND=true USE_RADAR=true CAMERA_DETECTION=false" ;;
    *) echo "configuración desconocida: $1" >&2; return 1 ;;
  esac
}

echo "[ablation] $(date +%H:%M:%S) escenario=$WORLD configuraciones=[$CONFIGS] semillas=[$SEEDS]"
for seed in $SEEDS; do               # semilla por fuera: con un corte, todas las configs avanzan
  for cfg in $CONFIGS; do
    env_vars="$(config_env "$cfg")" || exit 1
    out="$ROOT/$cfg"
    if ls "$out"/seed"${seed}"_"${WORLD}"_*[0-9].json >/dev/null 2>&1; then
      echo "[ablation] $cfg semilla $seed: ya hecha, se omite"
      continue
    fi
    echo "[ablation] $(date +%H:%M:%S) $cfg semilla $seed ($env_vars)"
    # shellcheck disable=SC2086
    env $env_vars WORLD_PARAMS="$WORLD" RESULTS_DIR="$out" "$REPO/scripts/run_batch.sh" "$seed" \
      | grep -E "^\[batch\] semilla|Semilla [0-9]+ \(" || true
  done
done
echo "[ablation] $(date +%H:%M:%S) terminado. Análisis: python3 scripts/ablation.py $ROOT"
