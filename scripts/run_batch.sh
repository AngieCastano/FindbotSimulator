#!/usr/bin/env bash
# Corre la misión completa (mission.launch.py) en modo headless para varias semillas, una tras
# otra, y consolida los resultados en una tabla con media y desviación estándar.
#
# Uso (dentro del contenedor, con el workspace compilado):
#   scripts/run_batch.sh                 # semillas 1 2 3 4 5
#   scripts/run_batch.sh 1 2 3 42
#   QUICK=true scripts/run_batch.sh 1 2  # 5 min de exploración por semilla (prueba rápida)
#
# Variables opcionales:
#   RESULTS_DIR      carpeta de salida (por defecto results/batch_<fecha>)
#   MISSION_TIMEOUT  tiempo real máximo por misión en s (por defecto 1800); si se agota, la
#                    misión se interrumpe con Ctrl+C y el evaluador guarda resultados parciales.
#
# Salida: en RESULTS_DIR, por semilla seed<N>_<fecha>.{json,csv,_map.pgm,_map.yaml,
# _comparison.png}, victims_seed<N>_<fecha>.csv y seed<N>_launch.log; al final summary.csv y
# summary.md.
#
# Cierra cualquier otra simulación antes de lanzarlo: comparten los tópicos de ROS.

set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEEDS=("$@")
[ ${#SEEDS[@]} -eq 0 ] && SEEDS=(1 2 3 4 5)
OUT="${RESULTS_DIR:-$REPO/results/batch_$(date +%Y%m%d_%H%M%S)}"
TIMEOUT="${MISSION_TIMEOUT:-1800}"
QUICK="${QUICK:-false}"

if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg prefix usar_metrics >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$REPO/usar_ws/install/setup.bash"
fi
mkdir -p "$OUT"
cd "$REPO/usar_ws" || exit 1   # los mundos se generan en usar_ws/worlds/

echo "[batch] Semillas: ${SEEDS[*]} | quick=$QUICK | resultados: $OUT"
failed=()
for seed in "${SEEDS[@]}"; do
  log="$OUT/seed${seed}_launch.log"
  echo "[batch] $(date +%H:%M:%S) semilla $seed: misión en curso (log: $log)"
  start=$(date +%s)
  # setsid: el launch y sus nodos quedan en su propio grupo de procesos, para cerrarlo entero.
  setsid ros2 launch usar_bringup mission.launch.py "seed:=$seed" headless:=true \
    "quick:=$QUICK" "results_dir:=$OUT" >"$log" 2>&1 &
  pid=$!
  while kill -0 "$pid" 2>/dev/null; do
    if [ $(( $(date +%s) - start )) -ge "$TIMEOUT" ]; then
      echo "[batch] semilla $seed: tiempo real agotado ($TIMEOUT s), interrumpiendo"
      kill -INT -- "-$pid" 2>/dev/null
      sleep 30
      kill -KILL -- "-$pid" 2>/dev/null
      break
    fi
    sleep 5
  done
  wait "$pid" 2>/dev/null
  # Procesos que hayan sobrevivido al cierre (p. ej. el servidor de Gazebo).
  kill -KILL -- "-$pid" 2>/dev/null
  summary=$(ls -t "$OUT"/seed"${seed}"_*.json 2>/dev/null | head -1)
  if [ -n "$summary" ]; then
    echo "[batch] semilla $seed: $(grep -m1 '\[evaluator\] Semilla' "$log" | sed 's/.*\[evaluator\] //')"
  else
    echo "[batch] semilla $seed: SIN RESULTADOS (revisa $log)"
    failed+=("$seed")
  fi
  sleep 5
done

ros2 run usar_metrics consolidate "$OUT"
if [ ${#failed[@]} -gt 0 ]; then
  echo "[batch] Semillas sin resultados: ${failed[*]}"
  exit 1
fi
