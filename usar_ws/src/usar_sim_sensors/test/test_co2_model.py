"""Modelo de CO₂ y del sensor TGS4161 (usar_sim_sensors)."""

import math

import numpy as np
import pytest
from usar_sim_sensors.co2_model import Co2Plume, Tgs4161


def _plume(sigma=0.0, sources=((0.0, 0.0),)):
    return Co2Plume(sources, 400.0, 2000.0, 1.0, sigma, 5.0, np.random.default_rng(0))


def test_plume_decays_with_distance():
    plume = _plume()
    assert plume.sample(0.0, 0.0, 0.5) == pytest.approx(2400.0)
    assert plume.sample(1.0, 0.0, 0.5) == pytest.approx(400 + 2000 / math.e)
    assert plume.sample(10.0, 0.0, 0.5) == pytest.approx(400.0, abs=1.0)


def test_plume_sums_sources_and_no_sources_is_ambient():
    two = _plume(sources=((0, 0), (2, 0)))
    assert two.mean_excess(1.0, 0.0) == pytest.approx(2 * 2000 / math.e)
    assert _plume(sources=()).sample(0, 0, 0.5) == 400.0


def test_turbulence_fluctuates_but_never_below_ambient():
    plume = _plume(sigma=0.3)
    values = np.array([plume.sample(0.5, 0.0, 0.5) for _ in range(4000)])
    assert values.min() >= 400.0
    assert values.std() > 50.0
    assert values.mean() == pytest.approx(400 + 2000 * math.exp(-0.5), rel=0.1)


def test_sensor_first_order_response():
    sensor = Tgs4161(20.0, 0.0, 350, 10000, np.random.default_rng(0), initial_ppm=400.0)
    readings = [sensor.update(4000.0, 1.0) for _ in range(20)]      # 20 s: una τ
    expected = math.exp(math.log(400) + (1 - math.exp(-1)) * (math.log(4000) - math.log(400)))
    assert readings[-1] == pytest.approx(expected, rel=1e-6)
    for _ in range(200):
        last = sensor.update(4000.0, 1.0)
    assert last == pytest.approx(4000.0, rel=1e-3)


def test_sensor_range_and_noise():
    sensor = Tgs4161(0.0, 0.02, 350, 10000, np.random.default_rng(1), initial_ppm=400.0)
    assert sensor.update(50000.0, 1.0) == 10000.0
    assert sensor.update(100.0, 1.0) == 350.0
    readings = np.array([sensor.update(1000.0, 1.0) for _ in range(2000)])
    assert np.std(np.log(readings)) == pytest.approx(0.02, rel=0.1)
