#!/usr/bin/env bash
# Fase H3: genera el esquemático desde robot_hw.yaml, corre el ERC, verifica la netlist
# contra la tabla de pines y exporta PDF + BOM a hardware/kicad/out/.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p out
python3 gen_schematic.py
kicad-cli sch erc --severity-all --exit-code-violations -o out/erc.rpt findbot.kicad_sch >/dev/null \
  || { echo "ERC con violaciones: ver hardware/kicad/out/erc.rpt"; exit 1; }
grep "ERC messages" out/erc.rpt
python3 check_netlist.py
kicad-cli sch export pdf -o out/findbot.pdf findbot.kicad_sch >/dev/null
kicad-cli sch export bom --fields 'Reference,Value,Footprint,${QUANTITY},${DNP}' \
  --labels 'Ref,Valor,Huella,Cant,No montar' --group-by 'Value,Footprint,${DNP}' \
  -o out/findbot_bom.csv findbot.kicad_sch >/dev/null
echo "PDF y BOM en hardware/kicad/out/"
