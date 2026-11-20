#!/usr/bin/env python3
"""Corre las simulaciones SPICE de la Fase H2 y verifica sus criterios de aceptación.

1. Genera results/params.inc con los valores de hardware/config/robot_hw.yaml
   (secciones "analog" y "battery"), para que ningún valor quede fijo en los netlists.
2. Ejecuta ngspice en modo batch para cada netlist (.cir) y guarda el log en results/.
3. Lee las líneas "RESULT <nombre> <valor>" y las compara con criteria.yaml.
Uso: python3 hardware/spice/check_spice.py   (código de salida 1 si algún criterio falla)
"""
import re
import subprocess
import sys
from pathlib import Path

import yaml

SPICE_DIR = Path(__file__).resolve().parent
HW_YAML = SPICE_DIR.parent / 'config' / 'robot_hw.yaml'
RESULTS = SPICE_DIR / 'results'
RESULT_RE = re.compile(r'^RESULT\s+(\w+)\s+(\S+)', re.MULTILINE)


def flatten(prefix, node, out):
    """Aplana un diccionario anidado en parámetros prefijo_clave con valores numéricos."""
    for key, value in node.items():
        name = f'{prefix}_{key}'
        if isinstance(value, dict):
            flatten(name, value, out)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = value
        elif isinstance(value, str):
            # PyYAML (YAML 1.1) lee "1.0e9" como texto: se convierte si es un número
            try:
                out[name] = float(value)
            except ValueError:
                pass
    return out


def write_params(cfg):
    """Escribe results/params.inc con un .param por cada valor numérico."""
    params = flatten('analog', cfg['analog'], {})
    flatten('battery', cfg['battery'], params)
    lines = ['* Generado por check_spice.py desde config/robot_hw.yaml — no editar']
    lines += [f'.param {name}={value!r}' for name, value in sorted(params.items())]
    (RESULTS / 'params.inc').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def run_netlist(name):
    """Ejecuta un netlist y devuelve {resultado: valor} (vacío si ngspice falló)."""
    proc = subprocess.run(['ngspice', '-b', f'{name}.cir'], cwd=SPICE_DIR,
                          capture_output=True, text=True, timeout=300)
    log = proc.stdout + proc.stderr
    (RESULTS / f'{name}.log').write_text(log, encoding='utf-8')
    values = {}
    for key, raw in RESULT_RE.findall(log):
        try:
            values[key] = float(raw)
        except ValueError:
            values[key] = None   # p. ej. una medición .meas que no encontró el cruce
    return values


def main():
    """Punto de entrada: simula todo, imprime la tabla y devuelve el código de salida."""
    RESULTS.mkdir(exist_ok=True)
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    criteria = yaml.safe_load((SPICE_DIR / 'criteria.yaml').read_text(encoding='utf-8'))
    write_params(cfg)

    failures = 0
    rows = []
    for netlist, limits in criteria.items():
        values = run_netlist(netlist)
        for key, (lo, hi) in limits.items():
            value = values.get(key)
            ok = value is not None and (lo is None or value >= lo) and (hi is None or value <= hi)
            failures += not ok
            shown = 'sin dato' if value is None else f'{value:.4g}'
            rows.append(f'{"OK " if ok else "FALLA"} {netlist:16s} {key:12s} {shown:>10s}'
                        f'   [{lo if lo is not None else "-"}, {hi if hi is not None else "-"}]')
    report = '\n'.join(rows) + f'\n\n{len(rows)} criterios, {failures} fallas\n'
    (RESULTS / 'summary.txt').write_text(report, encoding='utf-8')
    print(report, end='')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
