#!/usr/bin/env python3
"""
Análisis de los experimentos de ablación de la Fase 13 (results/ablation/<escenario>/<config>/).

Lee el JSON y el CSV de cada misión (los escribe usar_metrics/evaluator) y produce, en
<escenario>/report/:
  ablation.md       tabla por configuración (media ± σ), diferencia pareada por semilla frente
                    a la línea base A, comparación de la fusión por puntaje con la fusión "O" y
                    víctimas que encontró cada detector por separado
  ablation.csv      una fila por misión
  *.png             barras con desviación estándar (detección, falsos positivos, cobertura,
                    tiempo hasta la primera víctima)

Uso: python3 scripts/ablation.py results/ablation/hard [results/ablation/dusty ...]
"""

import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

CONFIG_LABELS = {
    'A': 'A: solo cámara', 'B': 'B: A + térmica', 'C': 'C: A + CO₂', 'D': 'D: A + sonido',
    'E': 'E: A + radar', 'F': 'F: todos (puntaje)', 'G': 'F + γ·S(g)',
    'H': 'H: sin cámara',
}
METRICS = [  # (clave, título, unidad, formato)
    ('detection_rate', 'Tasa de detección', '%', '{:.1f}'),
    ('false_positives', 'Falsos positivos', '', '{:.2f}'),
    ('coverage', 'Cobertura', '%', '{:.1f}'),
    ('first_victim_s', 'Tiempo hasta la 1.ª víctima', 's', '{:.0f}'),
    ('distance', 'Recorrido', 'm', '{:.1f}'),
    ('or_false_positives', 'Falsos positivos con fusión "O"', '', '{:.2f}'),
    ('or_detection_rate', 'Tasa de detección con fusión "O"', '%', '{:.1f}'),
    ('slam_error', 'Error medio de SLAM', 'm', '{:.2f}'),
    ('ekf_error', 'Error medio del EKF', 'm', '{:.2f}'),
]


def first_victim_time(csv_path):
    """Tiempo de exploración (s) en que aparece la primera víctima detectada (None: ninguna)."""
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            try:
                if int(float(row.get('victims_detected') or 0)) > 0:
                    return float(row['exploration_time'])
            except (KeyError, ValueError):
                continue
    return None


def load_mission(path):
    """Métricas de una misión a partir de su JSON (y el CSV con el mismo nombre)."""
    with open(path, encoding='utf-8') as stream:
        d = json.load(stream)
    v, fo = d['victims'], d.get('fusion_or') or {}
    loc = d.get('robot_localization') or {}
    return {
        'seed': d['seed'], 'detected': v['detected'], 'n_victims': v['n_victims'],
        'detection_rate': v['detection_rate_pct'], 'false_positives': v['false_positives'],
        'coverage': d['coverage_pct'], 'distance': d['distance_m'],
        'first_victim_s': first_victim_time(path[:-len('.json')] + '.csv'),
        'or_detection_rate': fo.get('detection_rate_pct'),
        'or_false_positives': fo.get('false_positives'),
        'slam_error': loc.get('slam_error_m_mean'), 'ekf_error': loc.get('ekf_error_m_mean'),
        'by_modality': {k: m['detected'] for k, m in (d.get('by_modality') or {}).items()},
        'by_modality_fp': {k: m['false_positives']
                           for k, m in (d.get('by_modality') or {}).items()},
    }


def load(world_dir):
    """{config: {seed: métricas}} de todas las misiones del escenario."""
    data = {}
    for cfg_dir in sorted(glob.glob(os.path.join(world_dir, '*'))):
        cfg = os.path.basename(cfg_dir)
        if cfg not in CONFIG_LABELS:
            continue
        for path in sorted(glob.glob(os.path.join(cfg_dir, 'seed*_[0-9]*[0-9].json'))):
            m = load_mission(path)
            data.setdefault(cfg, {})[m['seed']] = m     # la última misión de la semilla manda
    return data


def stats(values):
    """(media, σ muestral, n) sin los None."""
    vals = np.array([x for x in values if x is not None], dtype=float)
    if len(vals) == 0:
        return None, None, 0
    return float(vals.mean()), float(vals.std(ddof=1)) if len(vals) > 1 else 0.0, len(vals)


def fmt(mean, std, spec):
    return '–' if mean is None else f'{spec.format(mean)} ± {spec.replace("+", "").format(std)}'


def write_report(world_dir, data):
    out = os.path.join(world_dir, 'report')
    os.makedirs(out, exist_ok=True)
    configs = [c for c in CONFIG_LABELS if c in data]
    world = os.path.basename(os.path.normpath(world_dir))
    lines = [f'# Ablación — escenario `{world}`', '',
             'Media ± desviación estándar muestral sobre las semillas. Diferencia pareada: '
             'misma semilla (mismo mundo) frente a la configuración A.', '']
    # Tabla principal
    head = ['Configuración', 'n'] + [t + (f' [{u}]' if u else '') for _, t, u, _ in METRICS[:5]]
    lines += ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for cfg in configs:
        missions = list(data[cfg].values())
        row = [CONFIG_LABELS[cfg], str(len(missions))]
        for key, _, _, spec in METRICS[:5]:
            mean, std, _ = stats(m[key] for m in missions)
            row.append(fmt(mean, std, spec))
        lines.append('| ' + ' | '.join(row) + ' |')
    # Diferencia pareada frente a A
    if 'A' in data:
        lines += ['', '## Diferencia pareada frente a A (misma semilla)', '',
                  '| Configuración | Δ víctimas detectadas | Δ falsos positivos | Δ cobertura [%] | '
                  'semillas |', '|---|---|---|---|---|']
        for cfg in configs:
            if cfg == 'A':
                continue
            seeds = sorted(set(data[cfg]) & set(data['A']))
            if not seeds:
                continue
            d_det = [data[cfg][s]['detected'] - data['A'][s]['detected'] for s in seeds]
            d_fp = [data[cfg][s]['false_positives'] - data['A'][s]['false_positives']
                    for s in seeds]
            d_cov = [data[cfg][s]['coverage'] - data['A'][s]['coverage'] for s in seeds]
            lines.append(f'| {CONFIG_LABELS[cfg]} | {fmt(*stats(d_det)[:2], "{:+.2f}")} | '
                         f'{fmt(*stats(d_fp)[:2], "{:+.2f}")} | '
                         f'{fmt(*stats(d_cov)[:2], "{:+.1f}")} | {len(seeds)} |')
    # Fusión por puntaje frente a fusión "O"
    lines += ['', '## Fusión por puntaje frente a fusión "O"', '',
              'La fusión "O" (cualquier detector confirmador basta) se calcula con las '
              'confirmaciones de cada detector por separado en la misma misión.', '',
              '| Configuración | Detección puntaje [%] | Detección "O" [%] | FP puntaje | FP "O" |',
              '|---|---|---|---|---|']
    for cfg in configs:
        missions = list(data[cfg].values())
        cells = [fmt(*stats(m[k] for m in missions)[:2], spec) for k, spec in (
            ('detection_rate', '{:.1f}'), ('or_detection_rate', '{:.1f}'),
            ('false_positives', '{:.2f}'), ('or_false_positives', '{:.2f}'))]
        lines.append(f'| {CONFIG_LABELS[cfg]} | ' + ' | '.join(cells) + ' |')
    # Cada detector por separado
    modalities = sorted({k for c in configs for m in data[c].values() for k in m['by_modality']})
    if modalities:
        lines += ['', '## Víctimas que confirmó cada detector por separado (media por misión; '
                  'entre paréntesis, falsos positivos)', '',
                  'Sonido y radar son guías: sus "víctimas" son candidatos y sus "falsos '
                  'positivos", candidatos a más de 1 m de una víctima (el radar ubica en el eje '
                  'de la antena a 1,5 m).', '',
                  '| Configuración | ' + ' | '.join(modalities) + ' |',
                  '|---|' + '---|' * len(modalities)]
        for cfg in configs:
            missions = list(data[cfg].values())
            cells = []
            for mod in modalities:
                det = stats(m['by_modality'].get(mod) for m in missions)[0]
                fp = stats(m['by_modality_fp'].get(mod) for m in missions)[0]
                cells.append('–' if det is None else f'{det:.1f} ({fp:.1f})')
            lines.append(f'| {CONFIG_LABELS[cfg]} | ' + ' | '.join(cells) + ' |')
    # Localización
    lines += ['', '## Localización del robot', '', '| Configuración | ' +
              ' | '.join(t + f' [{u}]' for _, t, u, _ in METRICS[7:]) + ' |', '|---|---|---|']
    for cfg in configs:
        missions = list(data[cfg].values())
        lines.append(f'| {CONFIG_LABELS[cfg]} | ' + ' | '.join(
            fmt(*stats(m[k] for m in missions)[:2], spec) for k, _, _, spec in METRICS[7:]) + ' |')
    with open(os.path.join(out, 'ablation.md'), 'w', encoding='utf-8') as stream:
        stream.write('\n'.join(lines) + '\n')
    # CSV por misión
    keys = ['config', 'seed', 'detected', 'n_victims'] + [k for k, *_ in METRICS]
    with open(os.path.join(out, 'ablation.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        for cfg in configs:
            for seed in sorted(data[cfg]):
                writer.writerow({'config': cfg, **data[cfg][seed]})
    # Gráficas
    for key, title, unit, _ in METRICS[:4]:
        means, stds = zip(*[stats(m[key] for m in data[c].values())[:2] for c in configs])
        fig, ax = plt.subplots(figsize=(8, 4))
        x = np.arange(len(configs))
        ax.bar(x, [m or 0 for m in means], yerr=[s or 0 for s in stds], capsize=4,
               color=['tab:gray' if c == 'A' else 'tab:blue' for c in configs])
        ax.set_xticks(x, [CONFIG_LABELS[c] for c in configs], rotation=30, ha='right')
        ax.set_ylabel(f'{title}' + (f' [{unit}]' if unit else ''))
        ax.set_title(f'{title} — escenario {world}')
        ax.grid(axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'{key}.png'), dpi=150)
        plt.close(fig)
    return os.path.join(out, 'ablation.md')


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    for world_dir in argv[1:]:
        data = load(world_dir)
        if not data:
            print(f'{world_dir}: no hay misiones')
            continue
        print(f'{world_dir}: ' + ', '.join(f'{c} ({len(v)})' for c, v in data.items()))
        print('  informe:', write_report(world_dir, data))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
