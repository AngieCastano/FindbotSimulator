"""Pruebas del modelo de ruido de la odometría de ruedas."""

import math

import numpy as np
from usar_sim_sensors.odom_model import OdomNoiseModel


def _straight(model, length, steps):
    """Avanza en línea recta `length` m en `steps` incrementos; devuelve la pose ruidosa final."""
    out = None
    for i in range(steps + 1):
        out = model.step(length * i / steps, 0.0, 0.0, 0.2, 0.0)
    return out


def test_zero_noise_reproduces_ideal_odometry():
    model = OdomNoiseModel(0, 0, 0, 0, 0, 0)
    out = None
    for i in range(101):
        t = i / 100.0
        out = model.step(math.cos(t) - 1.0, math.sin(t), t, 0.3, 0.5)
    x, y, yaw, v, w = out
    assert math.isclose(x, math.cos(1.0) - 1.0, abs_tol=1e-9)
    assert math.isclose(y, math.sin(1.0), abs_tol=1e-9)
    assert math.isclose(yaw, 1.0, abs_tol=1e-9)
    assert (v, w) == (0.3, 0.5)


def test_first_sample_sets_common_origin():
    model = OdomNoiseModel(0.01, 0.01, 0.01, 0.01, 0.1, 0.1, seed=3)
    assert model.step(1.0, 2.0, 0.5, 0.0, 0.0) == (1.0, 2.0, 0.5, 0.0, 0.0)


def test_noise_is_reproducible_by_seed():
    a = _straight(OdomNoiseModel(0.001, 0, 0.0001, 0, 0.01, 0.01, seed=7), 10.0, 100)
    b = _straight(OdomNoiseModel(0.001, 0, 0.0001, 0, 0.01, 0.01, seed=7), 10.0, 100)
    c = _straight(OdomNoiseModel(0.001, 0, 0.0001, 0, 0.01, 0.01, seed=8), 10.0, 100)
    assert a == b
    assert a != c


def test_no_motion_no_drift():
    model = OdomNoiseModel(0.01, 0.01, 0.01, 0.01, 0.0, 0.0, seed=1)
    for _ in range(100):
        x, y, yaw, _, _ = model.step(0.0, 0.0, 0.0, 0.0, 0.0)
    assert (x, y, yaw) == (0.0, 0.0, 0.0)


def test_accumulated_error_matches_k_and_not_rate():
    """σ del avance tras L m ≈ sqrt(k_ss·L), igual con pasos grandes o pequeños."""
    k_ss, length = 0.001, 20.0
    expected = math.sqrt(k_ss * length)
    for steps in (20, 400):
        errors = [_straight(OdomNoiseModel(k_ss, 0, 0, 0, 0, 0, seed=s), length, steps)[0]
                  - length for s in range(300)]
        assert abs(np.std(errors) - expected) < 0.2 * expected, steps
        assert abs(np.mean(errors)) < 0.3 * expected


def test_heading_error_grows_with_turning():
    k_tt, turns = 0.0001, 4
    finals = []
    for s in range(300):
        model = OdomNoiseModel(0, 0, 0, k_tt, 0, 0, seed=s)
        total = 0.0
        for i in range(turns * 100 + 1):
            total = 2 * math.pi * turns * i / (turns * 100)
            out = model.step(0.0, 0.0, math.atan2(math.sin(total), math.cos(total)), 0.0, 1.0)
        finals.append(math.atan2(math.sin(out[2] - total), math.cos(out[2] - total)))
    expected = math.sqrt(k_tt * 2 * math.pi * turns)
    assert abs(np.std(finals) - expected) < 0.2 * expected
    assert math.isclose(model.var_yaw, k_tt * 2 * math.pi * turns, rel_tol=1e-6)
