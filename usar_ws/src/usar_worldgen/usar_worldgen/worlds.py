"""
Ubicación y generación bajo demanda de los mundos por semilla (usada por los launch files).

Cada variante de parámetros θ tiene su propia carpeta para no mezclar mundos:
  worlds/seed<N>/           parámetros por defecto
  worlds/seed<N>_<nombre>/  parámetros de <nombre>_params.yaml (p. ej. seed42_dusty)

world_params puede ser una ruta a un YAML o el nombre corto de un YAML incluido en el paquete
(`dusty` → share/usar_worldgen/config/dusty_params.yaml).
"""

import os
from typing import Dict, Optional, Tuple

PARAMS_SUFFIX = '_params.yaml'
REQUIRED_FILES = ('world.sdf', 'spawn.yaml', 'ground_truth.yaml')


def _package_config_dir() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('usar_worldgen'), 'config')
    except (ImportError, LookupError):
        return None


def resolve_world_params(world_params: str, config_dir: Optional[str] = None) -> Tuple[str, str]:
    """
    Ruta absoluta del YAML de parámetros y etiqueta de la variante.

    '' o 'default' → ('', ''). Una ruta existente → (ruta, nombre sin '_params.yaml').
    Un nombre corto → el YAML del paquete con ese nombre.
    """
    if not world_params or world_params == 'default':
        return '', ''
    path = os.path.abspath(os.path.expanduser(world_params))
    if not os.path.isfile(path):
        config_dir = config_dir or _package_config_dir()
        candidate = os.path.join(config_dir or '', world_params + PARAMS_SUFFIX)
        if not os.path.isfile(candidate):
            raise FileNotFoundError(
                f'world_params {world_params!r}: no es un archivo ni un YAML del paquete '
                f'({candidate})')
        path = candidate
    name = os.path.basename(path)
    for suffix in (PARAMS_SUFFIX, '.yaml', '.yml'):
        if name.endswith(suffix):
            name = name[:-len(suffix)]
            break
    tag = '' if name == 'default' else name
    return path, tag


def world_dir_for(seed: int, worlds_dir: str, tag: str) -> str:
    """Carpeta del mundo de una semilla y variante."""
    return os.path.join(worlds_dir, f'seed{seed}' + (f'_{tag}' if tag else ''))


def _is_current(world_dir: str, expected: Dict) -> bool:
    """Cierto si el mundo guardado se generó con estos parámetros y el formato actual."""
    import yaml

    from .ground_truth import FORMAT_VERSION
    if not all(os.path.isfile(os.path.join(world_dir, f)) for f in REQUIRED_FILES):
        return False
    try:
        with open(os.path.join(world_dir, 'ground_truth.yaml'), encoding='utf-8') as stream:
            gt = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError):
        return False
    return (isinstance(gt, dict) and gt.get('format_version') == FORMAT_VERSION
            and gt.get('params') == expected)


def ensure_world(seed: int, worlds_dir: str, world_params: str = '',
                 config_dir: Optional[str] = None) -> str:
    """
    Carpeta del mundo (seed, variante); lo genera si falta.

    También lo regenera si se generó con otros parámetros (p. ej. tras cambiar el YAML de la
    variante) o con un formato de ground truth anterior: la semilla da el mismo escenario.
    """
    from .cli import build_parser, main as generate_world_cli, params_from_args

    params_path, tag = resolve_world_params(world_params, config_dir)
    world_dir = world_dir_for(seed, os.path.abspath(os.path.expanduser(worlds_dir)), tag)
    args = ['--seed', str(seed), '--out', world_dir]
    if params_path:
        args += ['--params', params_path]
    expected = params_from_args(build_parser().parse_args(args)).to_dict()
    if not _is_current(world_dir, expected):
        if generate_world_cli(args + ['--quiet'], configure_logging=False) != 0:
            variant = tag or 'default'
            raise RuntimeError(f'No se pudo generar el mundo de la semilla {seed} ({variant})')
    return world_dir
