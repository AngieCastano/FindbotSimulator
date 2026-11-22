"""Pruebas del modelo de la señal WiFi estación–robot."""

import math

import numpy as np
from usar_sim_sensors.comm_model import free_space_loss_db, model_params, WifiLinkModel

BOX = {'type': 'box', 'x': 5.0, 'y': 0.0, 'size_x': 0.5, 'size_y': 2.0, 'yaw': 0.0}
BOUNDS = {'x_min': -10.0, 'x_max': 10.0, 'y_min': -10.0, 'y_max': 10.0}


def _model(obstacles=(), bounds=None, **over):
    return WifiLinkModel(list(obstacles), bounds, model_params({'shadowing_sigma_db': 0.0,
                                                                **over}),
                         np.random.default_rng(0))


def test_friis_at_one_metre():
    assert math.isclose(free_space_loss_db(1.0, 2.4e9), 40.05, abs_tol=0.05)


def test_path_loss_exponent():
    m = _model()
    r10, _, _ = m.mean_rssi((0.0, 0.0), (10.0, 0.0))
    r20, _, _ = m.mean_rssi((0.0, 0.0), (20.0, 0.0))
    assert math.isclose(r10 - r20, 10 * 3.0 * math.log10(2.0), abs_tol=1e-9)


def test_each_obstacle_and_wall_attenuates():
    free, _, _ = _model().mean_rssi((0.0, 0.0), (8.0, 0.0))
    blocked, n_obs, _ = _model([BOX]).mean_rssi((0.0, 0.0), (8.0, 0.0))
    assert n_obs == 1 and math.isclose(free - blocked, 10.0, abs_tol=1e-9)
    inside, _, walls = _model(bounds=BOUNDS).mean_rssi((-12.0, 0.0), (-5.0, 0.0))
    assert walls == 1
    assert math.isclose(_model().mean_rssi((-12.0, 0.0), (-5.0, 0.0))[0] - inside, 12.0)


def test_shadowing_is_bounded_and_reproducible():
    a = WifiLinkModel([], None, model_params(), np.random.default_rng(3))
    b = WifiLinkModel([], None, model_params(), np.random.default_rng(3))
    xs = [a.measure((0, 0), (5, 0), 0.1) for _ in range(200)]
    assert xs == [b.measure((0, 0), (5, 0), 0.1) for _ in range(200)]
    mean, _, _ = a.mean_rssi((0, 0), (5, 0))
    assert abs(np.mean(xs) - mean) < 3.0 and np.std(xs) < 4.0
