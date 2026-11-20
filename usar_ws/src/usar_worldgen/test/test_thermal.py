"""Temperaturas del escenario (Fase 9): víctimas, ambiente y escombros tibios."""

import os
import re

from usar_worldgen.worlds import ensure_world
import yaml

THERMAL_RE = re.compile(r'<temperature>([0-9.]+)</temperature></plugin>')


def _model_temperatures(sdf_text):
    """{nombre del modelo: [temperaturas en K de sus visuales]}."""
    temps = {}
    for name, body in re.findall(r'<model name="([^"]+)">(.*?)</model>', sdf_text, re.S):
        found = [float(t) for t in THERMAL_RE.findall(body)]
        if found:
            temps[name] = found
    return temps


def test_victims_are_warm_and_rubble_at_ambient(generate):
    out, gt = generate('--seed', '42', name='t')
    text = (out / 'world.sdf').read_text()
    temps = _model_temperatures(text)
    victims = {n: t for n, t in temps.items() if n.startswith('victim_')}
    assert len(victims) == len(gt['victims']) == 5
    for parts in victims.values():
        assert len(parts) == 6                                  # cabeza, torso, piernas, brazos
        assert all(abs(t - (34.0 + 273.15)) < 1e-3 for t in parts)
    assert not any(n.startswith('obstacle_') for n in temps)   # sin escombros tibios
    assert re.search(r'<atmosphere type="adiabatic">\s*<temperature>293.1500', text)
    assert gt['ambient_temperature_c'] == 20.0
    assert all(v['temperature_c'] == 34.0 for v in gt['victims'])


def test_victim_temperature_option(generate):
    out, gt = generate('--seed', '3', '--victim-temperature', '37', name='t37')
    temps = _model_temperatures((out / 'world.sdf').read_text())
    assert all(abs(t - 310.15) < 1e-3 for n, ts in temps.items() for t in ts)
    assert gt['params']['victim_temperature_c'] == 37.0


def test_warm_debris_keep_layout(generate):
    _, base = generate('--seed', '42', name='base')
    out, warm = generate('--seed', '42', '--n-warm-debris', '6', name='warm')
    strip = [{k: v for k, v in o.items() if k != 'temperature_c'} for o in warm['obstacles']]
    assert strip == base['obstacles']
    assert warm['victims'] == base['victims']
    hot = [o for o in warm['obstacles'] if 'temperature_c' in o]
    assert len(hot) == 6
    assert all(28.0 <= o['temperature_c'] <= 45.0 for o in hot)
    temps = _model_temperatures((out / 'world.sdf').read_text())
    assert sorted(f'obstacle_{o["id"]:03d}' for o in hot) == \
        sorted(n for n in temps if n.startswith('obstacle_'))


def test_warm_debris_reproducible(generate):
    _, a = generate('--seed', '7', '--n-warm-debris', '4', name='a')
    _, b = generate('--seed', '7', '--n-warm-debris', '4', name='b')
    assert a['obstacles'] == b['obstacles']


def test_ensure_world_regenerates_stale(tmp_path):
    world_dir = ensure_world(5, str(tmp_path))
    gt_path = os.path.join(world_dir, 'ground_truth.yaml')
    with open(gt_path, encoding='utf-8') as stream:
        gt = yaml.safe_load(stream)
    first = os.path.getmtime(gt_path)
    assert ensure_world(5, str(tmp_path)) == world_dir
    assert os.path.getmtime(gt_path) == first                  # al día: no se regenera
    gt['format_version'] = 2                                   # mundo de una versión anterior
    with open(gt_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(gt, stream)
    ensure_world(5, str(tmp_path))
    with open(gt_path, encoding='utf-8') as stream:
        assert yaml.safe_load(stream)['format_version'] == 3
