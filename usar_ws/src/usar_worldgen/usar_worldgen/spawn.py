"""
Archivo público spawn.yaml: pose inicial del robot en el escenario generado.

A diferencia de ground_truth.yaml, este archivo NO contiene información del escenario
(obstáculos ni víctimas); solo la pose de aparición. Lo usa el launch de simulación para
hacer spawn del robot sin tocar el ground truth.
"""

from typing import Dict

import yaml

from .params import WorldParams

SPAWN_FILE = 'spawn.yaml'


def write_spawn(params: WorldParams, path: str) -> None:
    """Escribe spawn.yaml con la semilla y la pose de aparición (x, y, yaw)."""
    x, y, yaw = params.robot_spawn
    data = {'seed': params.seed, 'robot_spawn': {'x': x, 'y': y, 'yaw': yaw}}
    with open(path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write('# Pose inicial del robot (pública). Generado por usar_worldgen.\n')
        yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=None)


def load_spawn(path: str) -> Dict[str, float]:
    """Lee spawn.yaml y devuelve {'x': .., 'y': .., 'yaw': ..}."""
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    pose = data['robot_spawn']
    return {key: float(pose[key]) for key in ('x', 'y', 'yaw')}
