"""Pruebas de las métricas, la escritura de resultados y la consolidación (sin ROS)."""

import json
import math
import os

import numpy as np
import pytest
from usar_metrics import consolidate, report
from usar_metrics.metrics_core import (associate_victims, coverage, explorable_centers,
                                       GridGeometry, map_to_world, mean_std,
                                       occupancy_to_pgm, PathIntegrator, Pose2D, world_to_map)

ORIGIN = Pose2D(0.0, 0.0, 0.0)


def _square_area(size=4.0, res=0.1):
    """Área explorable cuadrada [-size/2, size/2]² completa."""
    n = int(round(size / res))
    geometry = GridGeometry(-size / 2, -size / 2, res, n, n)
    return geometry, explorable_centers(geometry, np.ones((n, n), dtype=bool))


def _map(value, size=6.0, res=0.05):
    """/map de size × size centrado en el origen, con todas las celdas a `value`."""
    n = int(round(size / res))
    return np.full(n * n, value, dtype=np.int8), GridGeometry(-size / 2, -size / 2, res, n, n)


# ---------------------------------------------------------------------- frames
def test_map_world_roundtrip():
    spawn = Pose2D(1.5, -2.0, 0.7)
    pts = np.array([[0.0, 0.0], [1.0, 2.0], [-3.0, 0.5]])
    world = map_to_world(pts, spawn)
    assert world[0] == pytest.approx([1.5, -2.0])
    assert world_to_map(world, spawn) == pytest.approx(pts)


def test_map_to_world_rotation():
    # Robot que arranca mirando a +y: el eje x del mapa apunta a +y del mundo.
    world = map_to_world(np.array([[1.0, 0.0]]), Pose2D(0.0, 0.0, math.pi / 2))
    assert world[0] == pytest.approx([0.0, 1.0], abs=1e-9)


# ---------------------------------------------------------------------- cobertura
def test_coverage_full_and_empty():
    _, centers = _square_area()
    free, geometry = _map(0)
    assert coverage(free, geometry, centers, ORIGIN)[0] == pytest.approx(100.0)
    unknown, geometry = _map(-1)
    assert coverage(unknown, geometry, centers, ORIGIN)[0] == 0.0
    occupied, geometry = _map(100)
    assert coverage(occupied, geometry, centers, ORIGIN)[0] == 0.0


def test_coverage_half_map_known():
    _, centers = _square_area()
    data, geometry = _map(-1)
    grid = data.reshape(geometry.rows, geometry.cols)
    grid[:, : geometry.cols // 2] = 0         # mitad x < 0 libre
    pct, covered = coverage(grid.ravel(), geometry, centers, ORIGIN)
    assert pct == pytest.approx(50.0)
    assert covered == len(centers) // 2


def test_coverage_ignores_free_cells_outside_explorable_area():
    # Mapa libre mucho mayor que el área explorable: la cobertura no supera el 100 %.
    _, centers = _square_area(size=2.0)
    data, geometry = _map(0, size=10.0)
    assert coverage(data, geometry, centers, ORIGIN)[0] == pytest.approx(100.0)


def test_coverage_uses_spawn_offset():
    # El frame map está desplazado: el robot arrancó en (2, 0) del mundo. Un mapa libre solo
    # en x_map < 0 cubre la parte del mundo con x < 2, es decir, toda el área [-2, 2]².
    _, centers = _square_area()
    data, geometry = _map(-1, size=12.0)
    grid = data.reshape(geometry.rows, geometry.cols)
    grid[:, : geometry.cols // 2] = 0
    assert coverage(grid.ravel(), geometry, centers, Pose2D(2.0, 0.0, 0.0))[0] == \
        pytest.approx(100.0)
    assert coverage(grid.ravel(), geometry, centers, Pose2D(-2.0, 0.0, 0.0))[0] == \
        pytest.approx(0.0)


def test_coverage_free_threshold():
    _, centers = _square_area()
    data, geometry = _map(30)
    assert coverage(data, geometry, centers, ORIGIN, free_threshold=25)[0] == 0.0
    assert coverage(data, geometry, centers, ORIGIN, free_threshold=50)[0] == 100.0


# ---------------------------------------------------------------------- distancia
def test_path_integrator_square():
    path = PathIntegrator()
    for x, y in [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]:
        path.add(x, y)
    assert path.distance == pytest.approx(4.0)


def test_path_integrator_filters_jitter():
    path = PathIntegrator(min_step=0.01)
    rng = np.random.default_rng(0)
    for _ in range(1000):          # robot quieto con ruido de ±1 mm
        path.add(*rng.uniform(-0.001, 0.001, 2))
    assert path.distance < 0.01
    path.add(1.0, 0.0)
    assert path.distance == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------- víctimas
VICTIMS = [{'id': 0, 'name': 'victim_00', 'x': 0.0, 'y': 0.0},
           {'id': 1, 'name': 'victim_01', 'x': 5.0, 'y': 0.0},
           {'id': 2, 'name': 'victim_02', 'x': 0.0, 'y': 5.0}]


def test_associate_all_detected():
    ev = associate_victims([(0.2, 0.0), (5.0, 0.3), (0.0, 4.6)], VICTIMS, 1.0)
    assert ev.detected == 3 and not ev.false_positives
    assert ev.detection_rate == pytest.approx(100.0)
    assert sorted(ev.errors) == pytest.approx([0.2, 0.3, 0.4])
    s = ev.summary()
    assert s['localization_error_mean_m'] == pytest.approx(0.3)
    assert s['localization_error_max_m'] == pytest.approx(0.4)
    assert s['precision_pct'] == pytest.approx(100.0)


def test_associate_threshold_and_false_positives():
    ev = associate_victims([(0.3, 0.0), (5.0, 1.5), (9.0, 9.0)], VICTIMS, 1.0)
    assert ev.detected == 1
    assert len(ev.false_positives) == 2          # la de (5, 1.5) está a 1.5 m > umbral
    assert not ev.matches[1].detected and not ev.matches[2].detected
    assert ev.summary()['detection_rate_pct'] == pytest.approx(33.33)
    assert ev.summary()['precision_pct'] == pytest.approx(33.33)


def test_associate_one_to_one():
    # Dos estimaciones de la misma víctima: una cuenta, la otra es falso positivo.
    ev = associate_victims([(0.1, 0.0), (-0.3, 0.0)], VICTIMS, 1.0)
    assert ev.detected == 1
    assert ev.matches[0].error == pytest.approx(0.1)
    assert ev.false_positives == [(-0.3, 0.0)]


def test_associate_global_optimum():
    # Una asociación voraz uniría A (0.6) con V0 por ser la pareja más cercana y dejaría a B
    # (a 2.05 m de V1) sin pareja. La óptima asigna A → V1 (0.8 m) y B → V0 (0.65 m).
    victims = [{'id': 0, 'x': 0.0, 'y': 0.0}, {'id': 1, 'x': 1.4, 'y': 0.0}]
    ev = associate_victims([(0.6, 0.0), (-0.65, 0.0)], victims, 1.0)
    assert ev.detected == 2 and not ev.false_positives


def test_associate_no_estimates():
    ev = associate_victims([], VICTIMS, 1.0)
    assert ev.detected == 0 and ev.detection_rate == 0.0
    s = ev.summary()
    assert s['precision_pct'] is None and s['localization_error_mean_m'] is None


# ---------------------------------------------------------------------- archivos
def test_occupancy_to_pgm_values_and_orientation():
    data = np.array([0, 100, -1, 50], dtype=np.int8)     # fila 0 (y bajo): libre, ocupado
    img = occupancy_to_pgm(data, rows=2, cols=2)
    # La fila 0 del mapa (y mínimo) es la última de la imagen.
    assert img.tolist() == [[205, 205], [254, 0]]


def test_write_map_files(tmp_path):
    data, geometry = _map(0, size=1.0)
    pgm, yml = report.write_map(str(tmp_path / 'm'), data, geometry, 25, 65)
    raw = open(pgm, 'rb').read()
    assert raw.startswith(b'P5') and raw.endswith(bytes([254]) * 10)
    assert 'mode: trinary' in open(yml).read()


def test_render_comparison(tmp_path):
    gt = {'seed': 7, 'bounds': {'x_min': -3.0, 'x_max': 3.0, 'y_min': -3.0, 'y_max': 3.0},
          'obstacles': [{'type': 'box', 'x': 1.0, 'y': 1.0, 'yaw': 0.3, 'size_x': 0.5,
                         'size_y': 0.8},
                        {'type': 'cylinder', 'x': -1.0, 'y': 1.0, 'radius': 0.4}]}
    ev = associate_victims([(0.1, 0.0), (2.5, 2.5)], VICTIMS[:1], 1.0)
    data, geometry = _map(0)
    summary = {'coverage_pct': 50.0, 'exploration_time_s': 10.0, 'distance_m': 1.0,
               'end_reason': 'timeout', 'victims': ev.summary()}
    out = tmp_path / 'c.png'
    report.render_comparison(str(out), gt, data, geometry, Pose2D(0.5, 0.0, 0.4), ev,
                             np.array([[0.0, 0.0], [1.0, 0.0]]), summary, 25, 65, dpi=40)
    assert out.read_bytes()[:4] == b'\x89PNG'


def test_mean_std():
    mean, std, n = mean_std([1.0, 2.0, 3.0, None])
    assert (mean, n) == (2.0, 3) and std == pytest.approx(1.0)
    assert mean_std([None]) == (None, None, 0)
    assert mean_std([5.0]) == (5.0, 0.0, 1)


def _summary(seed, coverage_pct, detected, fp, err):
    return {'seed': seed, 'end_reason': 'timeout', 'exploration_time_s': 900.0,
            'coverage_pct': coverage_pct, 'distance_m': 100.0,
            'victims': {'n_victims': 5, 'detected': detected,
                        'detection_rate_pct': 20.0 * detected, 'false_positives': fp,
                        'localization_error_mean_m': err}}


def test_consolidate(tmp_path):
    for s in (_summary(1, 90.0, 5, 0, 0.2), _summary(2, 80.0, 3, 1, 0.4),
              _summary(3, 70.0, 0, 0, None)):
        with open(tmp_path / f"seed{s['seed']}_20261119_000000.json", 'w') as stream:
            json.dump(s, stream)
    (tmp_path / 'victims_seed1_x.csv').write_text('ignorar')
    assert consolidate.main([str(tmp_path)]) == 0
    text = (tmp_path / 'summary.md').read_text()
    assert '80.0 ± 10.0' in text                 # cobertura media ± σ
    assert '0.30 ± 0.14' in text                 # error medio: solo misiones con detecciones
    assert '0.3 ± 0.6' in text                   # falsos positivos: media con un decimal
    lines = (tmp_path / 'summary.csv').read_text().splitlines()
    assert len(lines) == 1 + 3 + 2
    assert lines[-2].startswith('media')


def test_consolidate_without_inputs(tmp_path):
    assert consolidate.main([str(tmp_path)]) == 1
    assert not os.path.exists(tmp_path / 'summary.csv')
