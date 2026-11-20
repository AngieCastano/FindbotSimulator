#!/usr/bin/env python3
"""H7 R2/R3 — Emulación del lazo de control con el código real del firmware.

Compila tools/sim_control.cpp con lib/robot_core y include/config_gen.h (g++), lo ejecuta en varios
escenarios con el modelo del motor TT de robot_hw.yaml (sim_models.motor) y evalúa la respuesta de
la velocidad del robot medida por la odometría: error en régimen, sobrepaso y tiempo de
establecimiento. Salidas en firmware/results/control/: <escenario>.csv y summary.txt.
Uso: python3 hardware/firmware/tools/run_control_sim.py   (código de salida 1 si algo falla)
"""
import csv
import math
import subprocess
import sys
from pathlib import Path

import yaml

FW = Path(__file__).resolve().parents[1]
HW_YAML = FW.parent / 'config' / 'robot_hw.yaml'
OUT = FW / 'results' / 'control'
V_TARGET = 0.30

# escenario: (vbat, factor de carga, ruido en cuentas, PID activo, descripción)
SCENARIOS = {
    'nominal': (7.4, 1.0, 0, 1, 'batería nominal, piso duro'),
    'bateria_baja': (6.3, 1.0, 0, 1, 'batería casi en el umbral crítico'),
    'carga_x3': (7.4, 3.0, 0, 1, 'terreno irregular: triple resistencia a la rodadura'),
    'ruido_encoder': (7.4, 1.0, 2, 1, 'encoder con ±2 cuentas espurias por periodo'),
    'sin_pid': (7.4, 1.0, 0, 0, 'solo prealimentación (referencia)'),
}
CRITERIA = {'err_pct': 3.0, 'overshoot_pct': 10.0, 'settle_s': 1.2}


def plant_params(cfg):
    """Mismas fórmulas que spice/motor_drive.cir."""
    m, g = cfg['sim_models']['motor'], 9.80665
    r = m['v_test'] / m['i_stall']
    wnl = m['rpm_noload'] * 2 * math.pi / 60
    ke = (m['v_test'] - m['i_noload'] * r) / wnl
    kt = m['stall_torque_kgcm'] * 0.0980665 / m['i_stall']
    b = kt * m['i_noload'] / wnl
    tload = 0.5 * cfg['geometry']['mass_total'] * g * m['crr'] * cfg['geometry']['wheels']['radius']
    ron = cfg['sim_models']['tb6612']['r_on']
    return [r, float(m['l_arm']), ke, kt, b, float(m['j_eq']), tload, ron]


def metrics(rows):
    on = [r for r in rows if 0.5 <= r['t'] < 3.0]
    steady = [r['v_odom'] for r in on if r['t'] >= 2.0]
    mean = sum(steady) / len(steady)
    peak = max(r['v_odom'] for r in on)
    settle = None
    for k, r in enumerate(on):
        if all(abs(x['v_odom'] - V_TARGET) <= 0.05 * V_TARGET for x in on[k:]):
            settle = r['t'] - 0.5
            break
    return {'v_steady': mean, 'err_pct': 100 * abs(mean - V_TARGET) / V_TARGET,
            'overshoot_pct': max(0.0, 100 * (peak - V_TARGET) / V_TARGET),
            'settle_s': settle if settle is not None else float('inf')}


def main():
    cfg = yaml.safe_load(HW_YAML.read_text(encoding='utf-8'))
    subprocess.run(['python3', str(FW / 'tools' / 'gen_config.py')], check=True, capture_output=True)
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / 'sim_control'
    src = FW / 'lib' / 'robot_core' / 'src'
    subprocess.run(['g++', '-std=gnu++17', '-O2', '-Wall', '-Wextra', '-Werror', f'-I{src}', f'-I{FW / "include"}',
                    str(FW / 'tools' / 'sim_control.cpp'), str(src / 'pid.cpp'), str(src / 'kinematics.cpp'),
                    str(src / 'motor_limit.cpp'), '-o', str(exe), '-lm'], check=True)
    params = [f'{x:.6g}' for x in plant_params(cfg)]
    lines, fails = [], 0
    for name, (vbat, load, noise, pid, desc) in SCENARIOS.items():
        res = subprocess.run([str(exe), name, str(vbat), str(load), str(noise), str(pid), '42', *params],
                             check=True, capture_output=True, text=True)
        (OUT / f'{name}.csv').write_text(res.stdout, encoding='utf-8')
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(res.stdout.splitlines())]
        m = metrics(rows)
        ok = name == 'sin_pid' or all(m[k] <= lim for k, lim in CRITERIA.items())
        fails += not ok
        tag = 'REFERENCIA' if name == 'sin_pid' else ('OK' if ok else 'FALLA')
        lines.append(f'{tag:10s} {name:14s} v={m["v_steady"]:.3f} m/s  error={m["err_pct"]:.1f} %  '
                     f'sobrepaso={m["overshoot_pct"]:.1f} %  establecimiento={m["settle_s"]:.2f} s  ({desc})')
    lines.append(f'criterios (con PID): error ≤ {CRITERIA["err_pct"]} %, sobrepaso ≤ {CRITERIA["overshoot_pct"]} %, '
                 f'establecimiento (banda 5 %) ≤ {CRITERIA["settle_s"]} s; {fails} fallas')
    (OUT / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
