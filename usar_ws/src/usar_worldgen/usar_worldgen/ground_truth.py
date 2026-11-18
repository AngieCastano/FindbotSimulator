"""
Exportación y lectura del ground truth del escenario (ground_truth.yaml).

IMPORTANTE: este archivo solo puede leerlo el paquete usar_metrics (y herramientas offline
como plot). Ningún nodo del robot debe importarlo ni abrir ground_truth.yaml.
"""

from typing import Any, Dict, Tuple

import numpy as np
import yaml

from .generator import GeneratedWorld
from .geometry import Box
from .grid import GridSpec, rle_decode, rle_encode

FORMAT_VERSION = 1


def build_ground_truth(world: GeneratedWorld) -> Dict[str, Any]:
    """Diccionario serializable con toda la verdad del terreno del escenario."""
    params = world.params
    x_min, x_max, y_min, y_max = params.bounds
    spawn_x, spawn_y, spawn_yaw = params.robot_spawn

    obstacles = []
    for obs in world.obstacles:
        entry: Dict[str, Any] = {'id': obs.idx, 'type': obs.kind, 'x': obs.shape.x,
                                 'y': obs.shape.y, 'yaw': obs.yaw, 'height': obs.height}
        if isinstance(obs.shape, Box):
            entry.update(size_x=obs.shape.size_x, size_y=obs.shape.size_y)
        else:
            entry.update(radius=obs.shape.radius)
        obstacles.append(entry)

    victims = [{'id': v.idx, 'name': f'victim_{v.idx:02d}', 'x': v.shape.x, 'y': v.shape.y,
                'yaw': v.shape.yaw, 'length': v.shape.size_x, 'width': v.shape.size_y,
                'reach_distance': v.reach_distance} for v in world.victims]

    grid = world.grid
    return {
        'format_version': FORMAT_VERSION,
        'seed': params.seed,
        'generation': {
            'attempt': world.attempt,
            'sub_seed': world.sub_seed,
            'retries': world.retries,
        },
        'params': params.to_dict(),
        'bounds': {'x_min': x_min, 'x_max': x_max, 'y_min': y_min, 'y_max': y_max},
        'robot_spawn': {'x': spawn_x, 'y': spawn_y, 'yaw': spawn_yaw},
        'obstacles': obstacles,
        'victims': victims,
        'explorable_area': {
            'description': ('Celdas alcanzables por el centro del robot (flood-fill desde el '
                            'spawn con obstáculos inflados por robot_radius)'),
            'resolution': grid.resolution,
            'origin': [grid.origin_x, grid.origin_y],
            'cols': grid.cols,
            'rows': grid.rows,
            'free_cells': int(world.explorable.sum()),
            'area_m2': round(world.explorable_area_m2, 4),
            'encoding': 'rle_row_major_starting_with_false',
            'runs': rle_encode(world.explorable),
        },
    }


def write_ground_truth(world: GeneratedWorld, path: str) -> None:
    """Escribe ground_truth.yaml."""
    data = build_ground_truth(world)
    with open(path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write('# Ground truth generado por usar_worldgen. '
                     'SOLO lo puede leer usar_metrics.\n')
        yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=None,
                       allow_unicode=True, width=100)


def load_ground_truth(path: str) -> Dict[str, Any]:
    """Lee ground_truth.yaml."""
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def explorable_from_ground_truth(data: Dict[str, Any]) -> Tuple[GridSpec, np.ndarray]:
    """Reconstruye la rejilla y la máscara del área explorable desde el ground truth."""
    area = data['explorable_area']
    spec = GridSpec(float(area['origin'][0]), float(area['origin'][1]),
                    float(area['resolution']), int(area['cols']), int(area['rows']))
    return spec, rle_decode(area['runs'], spec.rows, spec.cols)
