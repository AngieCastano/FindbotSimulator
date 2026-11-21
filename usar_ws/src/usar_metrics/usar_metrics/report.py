"""
Escritura de los resultados de una misión: JSON, CSV, mapa (pgm/yaml) e imagen comparativa.

Herramienta del evaluador: usa el ground truth, nunca la ejecuta el robot.
"""

import csv
import json
import os
from typing import Dict, List, Sequence, Tuple

import matplotlib
matplotlib.use('Agg')
from matplotlib import patches  # noqa: E402,I100
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402
import numpy as np  # noqa: E402
from usar_worldgen.geometry import Box  # noqa: E402
import yaml  # noqa: E402

from .metrics_core import (GridGeometry, occupancy_to_pgm, Pose2D,  # noqa: E402
                           VictimEvaluation)

TIMESERIES_FIELDS = ['sim_time', 'exploration_time', 'coverage_pct', 'distance_m',
                     'victims_estimated', 'victims_detected', 'false_positives', 'status']


def write_json(path: str, data: Dict) -> None:
    """Guarda el resumen de la misión."""
    with open(path, 'w', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def write_timeseries(path: str, rows: Sequence[Dict]) -> None:
    """Guarda la evolución temporal de las métricas (una fila por muestra)."""
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=TIMESERIES_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, '') for k in TIMESERIES_FIELDS})


POSE_FIELDS = ['sim_time', 'true_x', 'true_y', 'true_yaw', 'slam_x', 'slam_y', 'slam_yaw',
               'odom_x', 'odom_y', 'odom_yaw', 'slam_error_m', 'slam_yaw_error_deg',
               'odom_error_m', 'odom_yaw_error_deg']


def write_pose_errors(path: str, rows: Sequence[Dict]) -> None:
    """Guarda la serie del error de localización (pose real, SLAM y odometría, en el mundo)."""
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=POSE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, '') for k in POSE_FIELDS})


def render_pose_errors(path: str, rows: Sequence[Dict], title: str, dpi: int = 150) -> None:
    """Gráfica del error de posición y de orientación de SLAM y de la odometría en el tiempo."""
    t = np.array([r['sim_time'] for r in rows], dtype=float)
    t = t - t[0]
    fig, (ax_p, ax_y) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for key, label, color in (('slam', 'SLAM (map)', 'tab:red'),
                              ('odom', 'Odometría de ruedas (odom)', 'tab:blue')):
        ax_p.plot(t, [r[f'{key}_error_m'] for r in rows], color=color, label=label)
        ax_y.plot(t, [r[f'{key}_yaw_error_deg'] for r in rows], color=color, label=label)
    ax_p.set_ylabel('Error de posición [m]')
    ax_y.set_ylabel('Error de orientación [°]')
    ax_y.set_xlabel('Tiempo [s]')
    for ax in (ax_p, ax_y):
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
    ax_p.legend(loc='upper left')
    ax_p.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def write_map(base_path: str, map_data: np.ndarray, geometry: GridGeometry,
              free_threshold: int, occupied_threshold: int) -> Tuple[str, str]:
    """Guarda el mapa en el formato de nav2_map_server (<base>.pgm y <base>.yaml)."""
    pgm_path, yaml_path = base_path + '.pgm', base_path + '.yaml'
    img = occupancy_to_pgm(map_data, geometry.rows, geometry.cols, free_threshold,
                           occupied_threshold)
    with open(pgm_path, 'wb') as stream:
        stream.write(f'P5\n# usar_metrics\n{geometry.cols} {geometry.rows}\n255\n'.encode())
        stream.write(img.tobytes())
    meta = {
        'image': os.path.basename(pgm_path),
        'mode': 'trinary',
        'resolution': geometry.resolution,
        'origin': [geometry.origin_x, geometry.origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': occupied_threshold / 100.0,
        'free_thresh': free_threshold / 100.0,
    }
    with open(yaml_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(meta, stream, sort_keys=False)
    return pgm_path, yaml_path


def render_comparison(path: str, gt: Dict, map_data: np.ndarray, geometry: GridGeometry,
                      spawn: Pose2D, victims: VictimEvaluation,
                      trajectory_world: np.ndarray, summary: Dict,
                      free_threshold: int, occupied_threshold: int, dpi: int = 150) -> None:
    """
    Mapa construido (alineado con el mundo) sobre el ground truth.

    Escombros reales en contorno, víctimas reales (X) y estimadas (●), trayectoria del robot.
    """
    bounds = gt['bounds']
    x_min, x_max, y_min, y_max = (bounds['x_min'], bounds['x_max'], bounds['y_min'],
                                  bounds['y_max'])
    fig, ax = plt.subplots(figsize=(8, 8.8))

    # Mapa de SLAM: 0 desconocido, 1 libre, 2 ocupado; se coloca en el mundo con la pose del
    # spawn (origen del frame map).
    grid = np.asarray(map_data, dtype=np.int16).reshape(geometry.rows, geometry.cols)
    classes = np.zeros(grid.shape, dtype=np.uint8)
    classes[(grid >= 0) & (grid <= free_threshold)] = 1
    classes[grid >= occupied_threshold] = 2
    extent = (geometry.origin_x, geometry.origin_x + geometry.cols * geometry.resolution,
              geometry.origin_y, geometry.origin_y + geometry.rows * geometry.resolution)
    image = ax.imshow(classes, origin='lower', extent=extent, interpolation='nearest',
                      cmap=ListedColormap(['#d9d9d9', '#ffffff', '#202020']), vmin=0, vmax=2,
                      zorder=0)
    image.set_transform(Affine2D().rotate(spawn.yaw).translate(spawn.x, spawn.y)
                        + ax.transData)

    ax.add_patch(patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False,
                                   lw=1.5, ls='--', edgecolor='#555555', zorder=3))
    for obs in gt['obstacles']:
        if obs['type'] == 'box':
            corners = Box(obs['x'], obs['y'], obs['size_x'], obs['size_y'], obs['yaw']).corners()
            patch = patches.Polygon(corners, closed=True)
        else:
            patch = patches.Circle((obs['x'], obs['y']), obs['radius'])
        if obs.get('category', 'rubble') == 'rubble':
            patch.set(fill=False, edgecolor='#c0392b', lw=1.0, zorder=3)
        else:   # escombro bajo (tablón, losa, bloque): se confunde con una víctima
            patch.set(fill=False, edgecolor='#9a7b2f', lw=1.2, ls='--', zorder=3)
        ax.add_patch(patch)

    if len(trajectory_world) > 1:
        ax.plot(trajectory_world[:, 0], trajectory_world[:, 1], color='#1f77b4', lw=0.8,
                alpha=0.7, zorder=4)

    # Contorno del cuerpo de cada víctima real (1.7 × 0.6 m): una estimación puede caer en un
    # extremo del cuerpo y no en su centro.
    for vic in gt.get('victims', []):
        if {'length', 'width', 'yaw'} <= set(vic):
            corners = Box(vic['x'], vic['y'], vic['length'], vic['width'], vic['yaw']).corners()
            ax.add_patch(patches.Polygon(corners, closed=True, facecolor='#ff7f0e', alpha=0.35,
                                         edgecolor='#7a2e00', lw=0.8, zorder=5))
    for match in victims.matches:
        tx, ty = match.true_xy
        ax.plot(tx, ty, marker='X', ms=11, color='#ff7f0e', mec='#7a2e00', zorder=6)
        label = f'V{match.victim_id}'
        if match.detected:
            ex, ey = match.estimated_xy
            ax.plot([tx, ex], [ty, ey], color='#2ca02c', lw=1.2, zorder=5)
            ax.plot(ex, ey, 'o', ms=7, color='#2ca02c', mec='black', zorder=7)
            label += f' ({match.error:.2f} m)'
        ax.annotate(label, (tx, ty), xytext=(7, 7), textcoords='offset points', fontsize=8,
                    fontweight='bold', color='#7a2e00', zorder=8)
    for fx, fy in victims.false_positives:
        ax.plot(fx, fy, 'o', ms=7, color='#d62728', mec='black', zorder=7)

    sx, sy = spawn.x, spawn.y
    ax.plot(sx, sy, marker='s', ms=8, color='#1f77b4', zorder=8)

    legend = [
        patches.Patch(facecolor='#ffffff', edgecolor='#999999', label='Libre (mapa SLAM)'),
        patches.Patch(facecolor='#202020', label='Ocupado (mapa SLAM)'),
        patches.Patch(facecolor='#d9d9d9', label='Desconocido'),
        patches.Patch(fill=False, edgecolor='#c0392b', label='Escombros reales'),
        patches.Patch(fill=False, edgecolor='#9a7b2f', ls='--', label='Escombros bajos'),
        Line2D([], [], color='#1f77b4', lw=1, label='Trayectoria (odometría)'),
        Line2D([], [], marker='X', ls='', color='#ff7f0e', mec='#7a2e00', ms=9,
               label='Víctima real (centro y cuerpo)'),
        Line2D([], [], marker='o', ls='', color='#2ca02c', mec='black', label='Estimada'),
        Line2D([], [], marker='o', ls='', color='#d62728', mec='black', label='Falso positivo'),
    ]
    ax.legend(handles=legend, loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=4,
              frameon=False, fontsize=8)
    vs = summary['victims']
    ax.set_title(f"Semilla {gt['seed']} — cobertura {summary['coverage_pct']:.1f} %, "
                 f"víctimas {vs['detected']}/{vs['n_victims']}, "
                 f"falsos positivos {vs['false_positives']}\n"
                 f"tiempo {summary['exploration_time_s']:.0f} s, "
                 f"recorrido {summary['distance_m']:.1f} m ({summary['end_reason']})",
                 fontsize=10)
    # Los límites incluyen las estimaciones fuera del escenario (falsos positivos tras un muro).
    margin = 0.5
    extra = np.array([*victims.false_positives,
                      *(m.estimated_xy for m in victims.matches if m.detected)]).reshape(-1, 2)
    ax.set_xlim(min(x_min, *extra[:, 0]) - margin, max(x_max, *extra[:, 0]) + margin)
    ax.set_ylim(min(y_min, *extra[:, 1]) - margin, max(y_max, *extra[:, 1]) + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, ls=':', lw=0.5, alpha=0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def victim_rows(victims: VictimEvaluation) -> List[Dict]:
    """Detalle por víctima (para el JSON)."""
    rows = []
    for m in victims.matches:
        rows.append({
            'id': m.victim_id, 'name': m.name,
            'true_xy': [round(v, 4) for v in m.true_xy],
            'estimated_xy': [round(v, 4) for v in m.estimated_xy] if m.detected else None,
            'error_m': round(m.error, 4) if m.error is not None else None,
        })
    return rows
