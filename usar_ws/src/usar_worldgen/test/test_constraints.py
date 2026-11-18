"""Restricciones del escenario, verificadas a partir del ground truth exportado."""

from itertools import combinations
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from usar_worldgen.generator import generate_world, GenerationError
from usar_worldgen.geometry import (Box, Circle, contains, shape_in_bounds, shapes_intersect)
from usar_worldgen.grid import clearance_map, rle_decode, rle_encode
from usar_worldgen.ground_truth import explorable_from_ground_truth
from usar_worldgen.params import WorldParams

SEEDS = [0, 1, 2, 3, 42, 99, 2024]


def _shapes(gt):
    """Huellas en planta de escombros y víctimas reconstruidas desde el ground truth."""
    shapes = []
    for obs in gt['obstacles']:
        if obs['type'] == 'box':
            shapes.append(Box(obs['x'], obs['y'], obs['size_x'], obs['size_y'], obs['yaw']))
        else:
            shapes.append(Circle(obs['x'], obs['y'], obs['radius']))
    victims = [Box(v['x'], v['y'], v['length'], v['width'], v['yaw']) for v in gt['victims']]
    return shapes, victims


def _bounds(gt):
    b = gt['bounds']
    return (b['x_min'], b['x_max'], b['y_min'], b['y_max'])


@pytest.fixture(params=SEEDS)
def world(request, generate):
    _, gt = generate('--seed', str(request.param))
    return gt


def test_counts(world):
    assert len(world['obstacles']) == world['params']['n_obstacles']
    assert len(world['victims']) == world['params']['n_victims']


def test_no_overlaps(world):
    obstacles, victims = _shapes(world)
    gap = world['params']['min_gap']
    for a, b in combinations(obstacles + victims, 2):
        assert not shapes_intersect(a, b, gap), (a, b)


def test_everything_inside_bounds(world):
    obstacles, victims = _shapes(world)
    for shape in obstacles + victims:
        assert shape_in_bounds(shape, _bounds(world)), shape


def test_spawn_clearance(world):
    spawn = world['robot_spawn']
    zone = Circle(spawn['x'], spawn['y'], world['params']['spawn_clearance'])
    obstacles, victims = _shapes(world)
    for shape in obstacles + victims:
        assert not shapes_intersect(shape, zone), shape


def test_explorable_area_is_consistent(world):
    """El área explorable contiene el spawn y respeta el radio del robot."""
    spec, mask = explorable_from_ground_truth(world)
    area = world['explorable_area']
    assert mask.sum() == area['free_cells'] > 0
    assert math.isclose(area['area_m2'], mask.sum() * spec.resolution ** 2, rel_tol=1e-6)
    spawn = world['robot_spawn']
    assert mask[spec.world_to_cell(spawn['x'], spawn['y'])]
    obstacles, victims = _shapes(world)
    clearance = clearance_map(spec, _bounds(world), obstacles + victims)
    assert (clearance[mask] > world['params']['robot_radius']).all()


def test_all_victims_reachable(world):
    spec, mask = explorable_from_ground_truth(world)
    xg, yg = spec.cell_centers()
    reach = world['params']['victim_reach_distance']
    for victim in world['victims']:
        footprint = Box(victim['x'], victim['y'], victim['length'], victim['width'],
                        victim['yaw'])
        dist = np.hypot(xg[mask] - footprint.x, yg[mask] - footprint.y)
        # Cota laxa con el centro; la exacta la da reach_distance del ground truth.
        assert dist.min() <= reach + math.hypot(footprint.size_x, footprint.size_y) / 2
        assert victim['reach_distance'] <= reach


def test_explorable_grid_is_connected(world):
    from scipy import ndimage
    _, mask = explorable_from_ground_truth(world)
    _, n_components = ndimage.label(mask)
    assert n_components == 1


def test_sdf_contains_all_models(generate):
    out, gt = generate('--seed', '5')
    root = ET.parse(out / 'world.sdf').getroot()
    names = [m.get('name') for m in root.iter('model')]
    assert sum(n.startswith('obstacle_') for n in names) == len(gt['obstacles'])
    assert sum(n.startswith('victim_') for n in names) == len(gt['victims'])
    assert sum(n.startswith('wall_') for n in names) == 4
    assert 'ground_plane' in names


def test_rle_roundtrip():
    rng = np.random.default_rng(0)
    for shape in [(1, 1), (3, 7), (40, 25)]:
        mask = rng.random(shape) > 0.5
        assert (rle_decode(rle_encode(mask), *shape) == mask).all()
    full = np.ones((4, 4), dtype=bool)
    assert rle_encode(full)[0] == 0
    assert (rle_decode(rle_encode(full), 4, 4) == full).all()


def test_geometry_primitives():
    assert shapes_intersect(Box(0, 0, 2, 2), Box(1.5, 0, 2, 2))
    assert not shapes_intersect(Box(0, 0, 2, 2), Box(3, 0, 2, 2))
    assert shapes_intersect(Box(0, 0, 2, 2), Box(2.05, 0, 2, 2), gap=0.1)
    # Cajas rotadas 45°: sus esquinas se acercan pero no se tocan.
    assert not shapes_intersect(Box(0, 0, 1, 1, math.pi / 4), Box(1.5, 0, 1, 1, math.pi / 4))
    assert shapes_intersect(Circle(0, 0, 1), Circle(1.9, 0, 1))
    assert not shapes_intersect(Circle(0, 0, 1), Circle(2.1, 0, 1))
    assert shapes_intersect(Box(0, 0, 2, 2), Circle(1.5, 0, 0.6))
    assert not shapes_intersect(Box(0, 0, 2, 2), Circle(1.5, 1.5, 0.6))
    assert contains(Box(0, 0, 2, 1, math.pi / 2), np.array(0.0), np.array(0.9))
    assert not shape_in_bounds(Circle(9.8, 0, 0.5), (-10, 10, -10, 10))


def test_impossible_scenario_raises():
    params = WorldParams(seed=0, size=(4.0, 4.0), n_obstacles=60, max_attempts=2,
                         max_placement_tries=20)
    with pytest.raises(GenerationError):
        generate_world(params)
