"""
CLI `generate`: crea world.sdf y ground_truth.yaml a partir de θ y la semilla s.

Prioridad de parámetros: valores por defecto < YAML (--params) < argumentos de la CLI.

Ejemplo:
    ros2 run usar_worldgen generate --seed 42 --out worlds/seed42
"""

import argparse
import logging
import os
import sys
from typing import List, Optional

from .generator import generate_world, GenerationError
from .ground_truth import write_ground_truth
from .params import load_params_yaml, WorldParams
from .sdf_writer import write_world_sdf
from .spawn import SPAWN_FILE, write_spawn

WORLD_FILE = 'world.sdf'
GROUND_TRUTH_FILE = 'ground_truth.yaml'


def build_parser() -> argparse.ArgumentParser:
    """Parser de argumentos de la CLI."""
    parser = argparse.ArgumentParser(
        prog='generate',
        description='Generador procedural de escenarios de escombros E = G(θ, s).')
    parser.add_argument('--params', metavar='YAML',
                        help='YAML con los parámetros θ (la CLI tiene prioridad)')
    parser.add_argument('--seed', type=int, help='semilla s (entero >= 0)')
    parser.add_argument('--size', type=float, nargs=2, metavar=('ANCHO', 'LARGO'),
                        help='dimensiones interiores del área en metros')
    parser.add_argument('--n-obstacles', type=int, help='número de escombros')
    parser.add_argument('--obstacle-size-range', type=float, nargs=2, metavar=('MIN', 'MAX'),
                        help='rango del lado de cajas / diámetro de cilindros en metros')
    parser.add_argument('--n-victims', type=int, help='número de víctimas')
    parser.add_argument('--victim-appearance', choices=('orange', 'dusty'),
                        help="aspecto de las víctimas ('dusty': grises, sin color distintivo)")
    parser.add_argument('--n-low-debris', type=int,
                        help='número de escombros bajos (tablones, losas, bloques)')
    parser.add_argument('--robot-spawn', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='pose de aparición del robot (m, m, rad)')
    parser.add_argument('--out', help='directorio de salida (por defecto worlds/seed<SEMILLA>)')
    parser.add_argument('--quiet', action='store_true', help='solo muestra advertencias')
    return parser


def params_from_args(args: argparse.Namespace) -> WorldParams:
    """Combina valores por defecto, YAML y CLI en un WorldParams validado."""
    params = WorldParams()
    if args.params:
        params = params.merged(load_params_yaml(args.params))
    params = params.merged({
        'seed': args.seed,
        'size': args.size,
        'n_obstacles': args.n_obstacles,
        'obstacle_size_range': args.obstacle_size_range,
        'n_victims': args.n_victims,
        'robot_spawn': args.robot_spawn,
        'victim_appearance': args.victim_appearance,
        'n_low_debris': args.n_low_debris,
    })
    params.validate()
    return params


def main(argv: Optional[List[str]] = None) -> int:
    """Punto de entrada de `ros2 run usar_worldgen generate`."""
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING if args.quiet else logging.INFO,
                        format='[usar_worldgen] %(levelname)s: %(message)s')
    logger = logging.getLogger('usar_worldgen')
    try:
        params = params_from_args(args)
        world = generate_world(params)
    except (ValueError, GenerationError) as error:
        logger.error('%s', error)
        return 1

    out_dir = args.out or os.path.join('worlds', f'seed{params.seed}')
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, WORLD_FILE)
    gt_path = os.path.join(out_dir, GROUND_TRUTH_FILE)
    write_world_sdf(world, world_path)
    write_ground_truth(world, gt_path)
    write_spawn(params, os.path.join(out_dir, SPAWN_FILE))

    logger.info('Semilla %d → intento %d (sub-semilla %d, %d reintentos)', params.seed,
                world.attempt, world.sub_seed, len(world.retries))
    logger.info('%d escombros, %d víctimas, área explorable %.2f m²', len(world.obstacles),
                len(world.victims), world.explorable_area_m2)
    logger.info('Mundo: %s', os.path.abspath(world_path))
    logger.info('Ground truth: %s', os.path.abspath(gt_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
