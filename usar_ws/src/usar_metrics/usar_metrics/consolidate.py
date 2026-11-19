"""
CLI `consolidate`: junta los resultados de varias misiones en una tabla con media y desviación.

Lee los JSON de resumen del evaluador (results/seed<N>_<fecha>.json) y escribe
`summary.csv` y `summary.md` con una fila por misión y las filas de media y desviación estándar
muestral.

Ejemplos:
    ros2 run usar_metrics consolidate results/batch_20261119_120000
    ros2 run usar_metrics consolidate results/seed1_*.json results/seed2_*.json --out results/tabla
"""

import argparse
import csv
import glob
import json
import os
import sys
from typing import Dict, List, Optional, Sequence

from .metrics_core import mean_std

#: (clave en la tabla, encabezado, función que extrae el valor del JSON, decimales)
COLUMNS = [
    ('seed', 'Semilla', lambda d: d['seed'], 0),
    ('end_reason', 'Fin', lambda d: d['end_reason'], None),
    ('exploration_time_s', 'Tiempo [s]', lambda d: d['exploration_time_s'], 1),
    ('coverage_pct', 'Cobertura [%]', lambda d: d['coverage_pct'], 1),
    ('distance_m', 'Recorrido [m]', lambda d: d['distance_m'], 1),
    ('n_victims', 'Víctimas', lambda d: d['victims']['n_victims'], 0),
    ('detected', 'Detectadas', lambda d: d['victims']['detected'], 0),
    ('detection_rate_pct', 'Tasa detección [%]', lambda d: d['victims']['detection_rate_pct'],
     1),
    ('false_positives', 'Falsos positivos', lambda d: d['victims']['false_positives'], 0),
    ('localization_error_mean_m', 'Error medio [m]',
     lambda d: d['victims']['localization_error_mean_m'], 2),
]
NUMERIC = [c for c in COLUMNS if c[3] is not None and c[0] != 'seed']


def collect(paths: Sequence[str]) -> List[str]:
    """Lista de JSON de resumen a partir de carpetas y patrones."""
    files = []
    for path in paths:
        if os.path.isdir(path):
            files += glob.glob(os.path.join(path, 'seed*.json'))
        else:
            files += glob.glob(path)
    return sorted(set(files))


def load(files: Sequence[str]) -> List[Dict]:
    """Lee los resúmenes, ordenados por semilla."""
    runs = []
    for path in files:
        with open(path, 'r', encoding='utf-8') as stream:
            data = json.load(stream)
        data['_file'] = os.path.basename(path)
        runs.append(data)
    return sorted(runs, key=lambda d: (d['seed'], d['_file']))


def table(runs: Sequence[Dict]) -> Dict[str, List]:
    """Filas por misión y estadísticos (media, desviación) de las columnas numéricas."""
    rows = [{key: get(run) for key, _, get, _ in COLUMNS} for run in runs]
    stats = {}
    for key, _, _, _ in NUMERIC:
        stats[key] = mean_std([row[key] for row in rows])
    return {'rows': rows, 'stats': stats}


def _fmt(value, decimals: Optional[int]) -> str:
    if value is None:
        return '–'
    if decimals is None:
        return str(value)
    return f'{float(value):.{decimals}f}'


def write_csv(path: str, result: Dict) -> None:
    """Tabla en CSV: una fila por misión, más 'media' y 'desviacion'."""
    keys = [c[0] for c in COLUMNS]
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(keys)
        for row in result['rows']:
            writer.writerow(['' if row[k] is None else row[k] for k in keys])
        for label, idx in (('media', 0), ('desviacion', 1)):
            line = [label, '']
            for key in keys[2:]:
                value = result['stats'][key][idx]
                line.append('' if value is None else round(value, 4))
            writer.writerow(line)


def markdown(result: Dict) -> str:
    """Tabla en Markdown con la fila 'media ± desviación'."""
    header = [c[1] for c in COLUMNS]
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    for row in result['rows']:
        lines.append('| ' + ' | '.join(_fmt(row[k], d) for k, _, _, d in COLUMNS) + ' |')
    cells = ['**media ± σ**', '']
    for key, _, _, decimals in COLUMNS[2:]:
        mean, std, n = result['stats'][key]
        # Las columnas enteras (víctimas, falsos positivos) llevan un decimal en la media: una
        # media de 0.2 falsos positivos no debe redondearse a 0.
        dec = max(decimals, 1)
        cells.append('–' if mean is None else f'{mean:.{dec}f} ± {std:.{dec}f}')
    lines.append('| ' + ' | '.join(cells) + ' |')
    n_runs = len(result['rows'])
    return (f'Resultados consolidados de {n_runs} misiones (σ: desviación estándar muestral).'
            '\n\n' + '\n'.join(lines) + '\n')


def main(argv: Optional[List[str]] = None) -> int:
    """Punto de entrada de `ros2 run usar_metrics consolidate`."""
    parser = argparse.ArgumentParser(prog='consolidate', description=__doc__.split('\n')[1])
    parser.add_argument('inputs', nargs='+', help='carpetas o JSON de resumen (admite *)')
    parser.add_argument('--out', help='prefijo de salida (por defecto <carpeta>/summary)')
    args = parser.parse_args(argv)

    files = collect(args.inputs)
    if not files:
        print('[consolidate] ERROR: no se encontraron resúmenes seed*.json', file=sys.stderr)
        return 1
    result = table(load(files))
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(files[0])), 'summary')
    write_csv(out + '.csv', result)
    text = markdown(result)
    with open(out + '.md', 'w', encoding='utf-8') as stream:
        stream.write(text)
    print(text)
    print(f'[consolidate] {len(files)} misiones → {out}.csv, {out}.md')
    return 0


if __name__ == '__main__':
    sys.exit(main())
