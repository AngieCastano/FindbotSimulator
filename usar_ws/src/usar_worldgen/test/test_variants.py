"""Variantes del escenario (Fase 8): víctimas 'dusty' y escombros bajos."""

from itertools import combinations
import os

import pytest
from usar_worldgen.generator import LOW_DEBRIS_TYPES
from usar_worldgen.geometry import Box, Circle, shapes_intersect
from usar_worldgen.worlds import resolve_world_params, world_dir_for

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
ORANGE = '1.0000 0.4000 0.0000'


def _footprints(gt):
    shapes = []
    for obs in gt['obstacles']:
        if obs['type'] == 'box':
            shapes.append(Box(obs['x'], obs['y'], obs['size_x'], obs['size_y'], obs['yaw']))
        else:
            shapes.append(Circle(obs['x'], obs['y'], obs['radius']))
    return shapes + [Box(v['x'], v['y'], v['length'], v['width'], v['yaw'])
                     for v in gt['victims']]


@pytest.mark.parametrize('seed', [1, 42])
def test_default_variant_is_unchanged(generate, seed):
    """Los parámetros nuevos con su valor por defecto no alteran el mundo del MVP."""
    dir_a, _ = generate('--seed', str(seed), name='a')
    dir_b, _ = generate('--seed', str(seed), '--victim-appearance', 'orange',
                        '--n-low-debris', '0', name='b')
    assert (dir_a / 'world.sdf').read_bytes() == (dir_b / 'world.sdf').read_bytes()


def test_dusty_victims_have_no_orange(generate):
    dir_d, gt = generate('--seed', '42', '--victim-appearance', 'dusty', name='d')
    dir_o, _ = generate('--seed', '42', name='o')
    assert ORANGE not in (dir_d / 'world.sdf').read_text()
    assert ORANGE in (dir_o / 'world.sdf').read_text()
    assert all(v['appearance'] == 'dusty' for v in gt['victims'])
    # Solo cambia el material: la disposición del escenario es la misma.
    _, gt_o = generate('--seed', '42', name='o2')
    assert gt['obstacles'] == gt_o['obstacles']
    assert [(v['x'], v['y']) for v in gt['victims']] == \
        [(v['x'], v['y']) for v in gt_o['victims']]


@pytest.mark.parametrize('seed', [3, 42])
def test_low_debris(generate, seed):
    _, gt = generate('--seed', str(seed), '--n-low-debris', '12', name='low')
    params = gt['params']
    low = [o for o in gt['obstacles'] if o['category'] != 'rubble']
    assert len(low) == 12
    assert len(gt['obstacles']) == params['n_obstacles'] + 12
    kinds = {name for name, _, _ in LOW_DEBRIS_TYPES}
    lo, hi = params['low_debris_height_range']
    for obs in low:
        assert obs['category'] in kinds
        assert lo <= obs['height'] <= hi
    gap = params['min_gap']
    for a, b in combinations(_footprints(gt), 2):
        assert not shapes_intersect(a, b, gap)
    for victim in gt['victims']:
        assert victim['reach_distance'] <= params['victim_reach_distance']


@pytest.mark.parametrize('seed', [1, 2, 3, 42])
def test_low_debris_keep_main_layout(generate, seed):
    """
    Con escombros bajos el resto del escenario es idéntico al de la variante por defecto.

    Así la misma semilla en las variantes orange y dusty se compara sobre el mismo escenario.
    """
    _, base = generate('--seed', str(seed), name='base')
    _, low = generate('--seed', str(seed), '--n-low-debris', '10', name='low')
    assert low['generation']['attempt'] == base['generation']['attempt']
    n = base['params']['n_obstacles']
    assert low['obstacles'][:n] == base['obstacles']
    assert [(v['x'], v['y'], v['yaw']) for v in low['victims']] == \
        [(v['x'], v['y'], v['yaw']) for v in base['victims']]


def test_resolve_world_params(tmp_path):
    assert resolve_world_params('') == ('', '')
    assert resolve_world_params('default') == ('', '')
    path, tag = resolve_world_params('dusty', config_dir=CONFIG_DIR)
    assert tag == 'dusty' and path.endswith('dusty_params.yaml')
    custom = tmp_path / 'mi_escenario.yaml'
    custom.write_text('n_victims: 3\n')
    assert resolve_world_params(str(custom)) == (str(custom), 'mi_escenario')
    with pytest.raises(FileNotFoundError):
        resolve_world_params('no_existe', config_dir=CONFIG_DIR)
    assert world_dir_for(42, 'worlds', '') == os.path.join('worlds', 'seed42')
    assert world_dir_for(42, 'worlds', 'dusty') == os.path.join('worlds', 'seed42_dusty')


def test_dusty_params_file(generate):
    path, _ = resolve_world_params('dusty', config_dir=CONFIG_DIR)
    _, gt = generate('--params', path, '--seed', '42', name='dp')
    assert gt['params']['victim_appearance'] == 'dusty'
    assert gt['params']['n_low_debris'] > 0
