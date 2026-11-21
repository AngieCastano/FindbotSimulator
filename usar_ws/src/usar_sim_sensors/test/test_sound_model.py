"""Modelo de sonido y del arreglo de micrófonos (usar_sim_sensors)."""

import math

import numpy as np
import pytest
from usar_sim_sensors.sound_model import (count_obstacles_between, db_sum, MicArrayModel,
                                          model_params, segment_hits_box, segment_hits_circle,
                                          VictimSounds)

BOX = {'type': 'box', 'x': 2.0, 'y': 0.0, 'size_x': 0.5, 'size_y': 1.0, 'yaw': 0.0}
CYL = {'type': 'cylinder', 'x': 4.0, 'y': 0.0, 'radius': 0.3}


def test_segment_obstacle_intersections():
    assert segment_hits_box((0, 0), (5, 0), 2, 0, 0.5, 1.0, 0.0)
    assert not segment_hits_box((0, 1), (5, 1), 2, 0, 0.5, 1.0, 0.0)
    # Segmento vertical en x = 2.4: no corta la caja (x ∈ [1.75, 2.25]); girada 90° sí
    # (x ∈ [1.5, 2.5]).
    assert not segment_hits_box((2.4, -1), (2.4, 1), 2, 0, 0.5, 1.0, 0.0)
    assert segment_hits_box((2.4, -1), (2.4, 1), 2, 0, 0.5, 1.0, math.pi / 2)
    assert segment_hits_circle((0, 0), (5, 0), 4, 0, 0.3)
    assert not segment_hits_circle((0, 0), (3, 0), 4, 0, 0.3)                  # no llega
    assert count_obstacles_between((0, 0), (5, 0), [BOX, CYL]) == 2
    assert count_obstacles_between((0, 2), (5, 2), [BOX, CYL]) == 0


def test_db_sum():
    assert db_sum([60.0, 60.0]) == pytest.approx(63.01, abs=0.01)
    assert db_sum([60.0, 30.0]) == pytest.approx(60.0, abs=0.01)


def test_bursts_schedule_and_fraction():
    rng = np.random.default_rng(0)
    sounds = VictimSounds(5, 0.6, 2.0, (10.0, 20.0), rng)
    assert len(sounds.vocal) == 3
    active = np.array([[i in sounds.active(t) for i in range(5)]
                       for t in np.arange(0, 600, 0.1)])
    duty = active.mean(axis=0)
    for i in range(5):
        if i in sounds.vocal:
            assert 0.08 < duty[i] < 0.2            # ≈ 2 s de cada 15 s
        else:
            assert duty[i] == 0.0


def _model(**over):
    return MicArrayModel([(5.0, 0.0)], [], model_params({'echo_prob': 0.0, **over}),
                         np.random.default_rng(1))


def test_direction_of_a_clear_source():
    model = _model()
    az = [model.measure((0.0, 0.0), 0.0, False, [0]) for _ in range(300)]
    assert all(a[2] for a in az)                   # voz detectada
    angles = np.array([a[0] for a in az])
    assert np.mean(angles) == pytest.approx(0.0, abs=0.02)
    assert np.std(angles) < math.radians(8)
    # En el frame del arreglo: el robot mirando a +y oye la fuente a −90°.
    rotated = _model()
    az_rot = np.mean([rotated.measure((0, 0), math.pi / 2, False, [0])[0] for _ in range(200)])
    assert az_rot == pytest.approx(-math.pi / 2, abs=0.05)


def test_motor_noise_and_silence_hide_the_source():
    model = _model(source_db=50.0)                  # 50 dB a 1 m → 36 dB a 5 m
    assert model.measure((0, 0), 0.0, False, [0])[2] is False
    assert _model().measure((0, 0), 0.0, True, [0])[2] is False     # motores en marcha
    assert _model().measure((0, 0), 0.0, False, [])[2] is False     # nadie emite


def test_rubble_attenuates_and_creates_echoes():
    wall = [{'type': 'box', 'x': 2.5, 'y': 0.0, 'size_x': 0.4, 'size_y': 2.0, 'yaw': 0.0}]
    model = MicArrayModel([(5.0, 0.0)], wall, model_params({'echo_prob': 0.0}),
                          np.random.default_rng(2))
    (_, clear_level, _), = _model().received((0, 0), [0])
    (_, level, n), = model.received((0, 0), [0])
    assert n == 1 and level == pytest.approx(clear_level - 8.0)
    echo = MicArrayModel([(5.0, 0.0)], wall * 3, model_params({'source_db': 90.0}),
                         np.random.default_rng(3))   # fuerte: se oye a través de 3 escombros
    angles = np.array([echo.measure((0, 0), 0.0, False, [0])[0] for _ in range(500)])
    far = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) > math.radians(45)
    assert 0.15 < far.mean() < 0.6                 # parte de las direcciones son ecos
