"""Reproducibilidad: misma semilla y mismos θ ⇒ mismos archivos byte a byte."""

import pytest
from usar_worldgen.cli import main as generate_main
from usar_worldgen.generator import derive_sub_seed, generate_world
from usar_worldgen.params import WorldParams
from usar_worldgen.spawn import load_spawn
import yaml

FILES = ('world.sdf', 'ground_truth.yaml', 'spawn.yaml')


@pytest.mark.parametrize('seed', [0, 42, 1234])
def test_same_seed_produces_identical_files(generate, seed):
    dir_a, _ = generate('--seed', str(seed), name='a')
    dir_b, _ = generate('--seed', str(seed), name='b')
    for name in FILES:
        assert (dir_a / name).read_bytes() == (dir_b / name).read_bytes(), name


def test_different_seeds_produce_different_worlds(generate):
    dir_a, gt_a = generate('--seed', '1', name='a')
    dir_b, gt_b = generate('--seed', '2', name='b')
    assert (dir_a / 'world.sdf').read_bytes() != (dir_b / 'world.sdf').read_bytes()
    assert gt_a['obstacles'] != gt_b['obstacles']


def test_yaml_params_equivalent_to_cli(generate, tmp_path):
    theta = {'seed': 7, 'size': [12.0, 15.0], 'n_obstacles': 15, 'n_victims': 3,
             'obstacle_size_range': [0.4, 1.2], 'robot_spawn': [1.0, -2.0, 0.5]}
    yaml_path = tmp_path / 'theta.yaml'
    yaml_path.write_text(yaml.safe_dump(theta))
    dir_yaml, _ = generate('--params', str(yaml_path), name='yaml')
    dir_cli, _ = generate('--seed', '7', '--size', '12', '15', '--n-obstacles', '15',
                          '--n-victims', '3', '--obstacle-size-range', '0.4', '1.2',
                          '--robot-spawn', '1', '-2', '0.5', name='cli')
    for name in FILES:
        assert (dir_yaml / name).read_bytes() == (dir_cli / name).read_bytes(), name


def test_cli_overrides_yaml(generate, tmp_path):
    yaml_path = tmp_path / 'theta.yaml'
    yaml_path.write_text(yaml.safe_dump({'seed': 3, 'n_victims': 2}))
    _, gt = generate('--params', str(yaml_path), '--n-victims', '4', name='o')
    assert gt['seed'] == 3
    assert len(gt['victims']) == 4


def test_ground_truth_records_seed_and_params(generate):
    _, gt = generate('--seed', '42', '--n-obstacles', '10')
    assert gt['seed'] == 42
    assert gt['params']['n_obstacles'] == 10
    assert gt['generation']['sub_seed'] == derive_sub_seed(42, gt['generation']['attempt'])


def test_retries_are_deterministic_and_logged():
    """Escenario saturado: algunos intentos fallan y se reintenta con sub-semillas."""
    base = {'size': (8.0, 8.0), 'n_obstacles': 14, 'obstacle_size_range': (0.8, 1.8),
            'n_victims': 3, 'max_attempts': 50, 'max_placement_tries': 200}
    with_retries = []
    for seed in range(15):
        world = generate_world(WorldParams(seed=seed, **base))
        assert world.attempt == len(world.retries)
        assert world.sub_seed == derive_sub_seed(seed, world.attempt)
        for i, retry in enumerate(world.retries):
            assert retry['attempt'] == i
            assert retry['sub_seed'] == derive_sub_seed(seed, i)
            assert retry['reason']
        if world.retries:
            with_retries.append(seed)
            again = generate_world(WorldParams(seed=seed, **base))
            assert again.retries == world.retries
            assert again.sub_seed == world.sub_seed
    assert with_retries, 'el escenario de prueba debería provocar al menos un reintento'


def test_spawn_file_is_public_and_consistent(generate):
    """spawn.yaml solo expone la pose inicial, nunca información del escenario."""
    out, gt = generate('--seed', '8', '--robot-spawn', '2', '-3', '1.5')
    data = yaml.safe_load((out / 'spawn.yaml').read_text())
    assert set(data) == {'seed', 'robot_spawn'}
    assert data['robot_spawn'] == gt['robot_spawn'] == {'x': 2.0, 'y': -3.0, 'yaw': 1.5}
    assert load_spawn(str(out / 'spawn.yaml')) == {'x': 2.0, 'y': -3.0, 'yaw': 1.5}


def test_invalid_params_fail_cleanly():
    assert generate_main(['--seed', '-1', '--out', '/nonexistent', '--quiet']) == 1
    assert generate_main(['--size', '1', '1', '--out', '/nonexistent', '--quiet']) == 1
