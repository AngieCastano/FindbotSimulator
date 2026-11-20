"""
CLI `plot`: dibuja en PNG la vista superior de un escenario generado (para el informe).

Herramienta offline: lee ground_truth.yaml, nunca se ejecuta en el robot.

Ejemplo:
    ros2 run usar_worldgen plot --world worlds/seed42
"""

import argparse
import math
import os
import sys
from typing import List, Optional

import matplotlib
matplotlib.use('Agg')
from matplotlib import patches  # noqa: E402,I100
from matplotlib.colors import ListedColormap  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from .geometry import Box  # noqa: E402
from .ground_truth import explorable_from_ground_truth, load_ground_truth  # noqa: E402
from .sdf_writer import VICTIM_MATERIALS  # noqa: E402

LOW_DEBRIS_RGB = '#c9b38a'   # escombros bajos (tablones, losas, bloques)


def render_top_view(gt: dict, out_path: str, dpi: int = 150) -> None:
    """Dibuja muros, área explorable, escombros, víctimas y spawn; guarda el PNG."""
    bounds = gt['bounds']
    x_min, x_max = bounds['x_min'], bounds['x_max']
    y_min, y_max = bounds['y_min'], bounds['y_max']
    spec, explorable = explorable_from_ground_truth(gt)
    params = gt['params']

    fig, ax = plt.subplots(figsize=(8, 8 * (y_max - y_min) / (x_max - x_min) + 0.6))
    extent = (spec.origin_x, spec.origin_x + spec.cols * spec.resolution,
              spec.origin_y, spec.origin_y + spec.rows * spec.resolution)
    ax.imshow(explorable, origin='lower', extent=extent, interpolation='nearest',
              cmap=ListedColormap(['#ffffff', '#d8f0d8']), vmin=0, vmax=1, zorder=0)

    ax.add_patch(patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False,
                                   lw=3, edgecolor='black', zorder=4))
    for obs in gt['obstacles']:
        if obs['type'] == 'box':
            corners = Box(obs['x'], obs['y'], obs['size_x'], obs['size_y'],
                          obs['yaw']).corners()
            patch = patches.Polygon(corners, closed=True)
        else:
            patch = patches.Circle((obs['x'], obs['y']), obs['radius'])
        if obs.get('category', 'rubble') == 'rubble':
            patch.set(facecolor='#7a7670', edgecolor='#3a3836', lw=0.8, zorder=2)
        else:
            patch.set(facecolor=LOW_DEBRIS_RGB, edgecolor='#3a3836', lw=0.8, hatch='///',
                      zorder=2)
        if 'temperature_c' in obs:   # escombro tibio: borde rojo y su temperatura
            patch.set(edgecolor='#d62728', lw=2.0)
            ax.annotate(f"{obs['temperature_c']:.0f} °C", (obs['x'], obs['y']), fontsize=7,
                        ha='center', va='center', color='#d62728', fontweight='bold', zorder=5)
        ax.add_patch(patch)

    victim_rgb = VICTIM_MATERIALS[params.get('victim_appearance', 'orange')][0]
    for vic in gt['victims']:
        corners = Box(vic['x'], vic['y'], vic['length'], vic['width'], vic['yaw']).corners()
        ax.add_patch(patches.Polygon(corners, closed=True, facecolor=victim_rgb,
                                     edgecolor='#7a2e00', lw=1.0, zorder=3))
        ax.annotate(f"V{vic['id']}", (vic['x'], vic['y']), xytext=(6, 6),
                    textcoords='offset points', fontsize=9, fontweight='bold',
                    color='#7a2e00', zorder=5)

    spawn = gt['robot_spawn']
    radius = params['robot_radius']
    ax.add_patch(patches.Circle((spawn['x'], spawn['y']), params['spawn_clearance'],
                                fill=False, ls='--', edgecolor='#1f5fbf', zorder=3))
    ax.add_patch(patches.Circle((spawn['x'], spawn['y']), radius, facecolor='#1f5fbf',
                                zorder=4))
    ax.arrow(spawn['x'], spawn['y'], 2 * radius * math.cos(spawn['yaw']),
             2 * radius * math.sin(spawn['yaw']), width=0.04, color='#1f5fbf', zorder=4)

    legend = [
        patches.Patch(facecolor='#d8f0d8', edgecolor='#999999', label='Área explorable'),
        patches.Patch(facecolor='#7a7670', label='Escombros'),
        patches.Patch(facecolor=LOW_DEBRIS_RGB, hatch='///', label='Escombros bajos'),
        patches.Patch(facecolor=victim_rgb, edgecolor='#7a2e00',
                      label=f"Víctimas ({params.get('victim_appearance', 'orange')})"),
        patches.Patch(facecolor='#1f5fbf', label='Spawn del robot'),
    ]
    ax.legend(handles=legend, loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=3,
              frameon=False, fontsize=9)
    area = gt['explorable_area']['area_m2']
    ax.set_title(f"Escenario semilla {gt['seed']} — {len(gt['obstacles'])} escombros, "
                 f"{len(gt['victims'])} víctimas, área explorable {area:.1f} m²")
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    margin = 0.5
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.set_aspect('equal')
    ax.grid(True, ls=':', lw=0.5, alpha=0.6)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def main(argv: Optional[List[str]] = None) -> int:
    """Punto de entrada de `ros2 run usar_worldgen plot`."""
    parser = argparse.ArgumentParser(prog='plot',
                                     description='Vista superior en PNG de un escenario.')
    parser.add_argument('--world', required=True,
                        help='directorio generado (con ground_truth.yaml)')
    parser.add_argument('--out', help='ruta del PNG (por defecto <world>/top_view.png)')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args(argv)

    gt_path = os.path.join(args.world, 'ground_truth.yaml')
    if not os.path.isfile(gt_path):
        print(f'[usar_worldgen] ERROR: no existe {gt_path}', file=sys.stderr)
        return 1
    out_path = args.out or os.path.join(args.world, 'top_view.png')
    render_top_view(load_ground_truth(gt_path), out_path, args.dpi)
    print(f'[usar_worldgen] Vista superior: {os.path.abspath(out_path)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
