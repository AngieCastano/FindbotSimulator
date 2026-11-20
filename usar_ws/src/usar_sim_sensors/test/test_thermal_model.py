"""Modelo de ruido del MLX90640 (usar_sim_sensors)."""

import numpy as np
import pytest
from usar_sim_sensors.thermal_model import raw_to_celsius, ThermalNoiseModel


def test_raw_to_celsius():
    raw = np.array([[29315, 30715]], dtype=np.uint16)
    assert raw_to_celsius(raw, 0.01) == pytest.approx(np.array([[20.0, 34.0]]), abs=1e-3)


def test_noise_statistics_and_fixed_pattern():
    model = ThermalNoiseModel(32, 24, netd=0.2, fixed_pattern=0.3, min_temp=-40, max_temp=300,
                              seed=1)
    flat = np.full((24, 32), 20.0, np.float32)
    frames = np.stack([model.apply(flat) for _ in range(400)])
    temporal = frames.std(axis=0).mean()
    assert temporal == pytest.approx(0.2, rel=0.1)
    assert frames.mean(axis=0) - 20.0 == pytest.approx(model.pattern, abs=0.05)
    assert model.pattern.std() == pytest.approx(0.3, rel=0.15)


def test_reproducible_and_clipped():
    a = ThermalNoiseModel(4, 3, 0.2, 0.3, -40, 300, seed=7)
    b = ThermalNoiseModel(4, 3, 0.2, 0.3, -40, 300, seed=7)
    img = np.array([[20, 400, -60, 34]] * 3, np.float32)
    out_a, out_b = a.apply(img), b.apply(img)
    assert np.array_equal(out_a, out_b)
    assert out_a.max() <= 300 and out_a.min() >= -40


def test_wrong_size_rejected():
    with pytest.raises(ValueError):
        ThermalNoiseModel(32, 24, 0.2, 0.3, -40, 300).apply(np.zeros((10, 10), np.float32))
