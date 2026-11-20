"""Detector de CO₂ (Fase 10): compensación del retardo del sensor y ubicación de la fuente."""

import math

import numpy as np
import pytest
from usar_detection.co2_detector import Co2Detector  # noqa: F401  (registra el detector)
from usar_detection.sign_of_life import Co2Observation, create_detector

TAU = 20.0


def drive(path, sources, speed=0.2, rate=2.0, noise=0.02, seed=0, wait=0.0):
    """
    Lecturas de un TGS4161 simulado (primer orden en log ppm) a lo largo de un recorrido.

    path: lista de puntos (x, y) que el robot une en línea recta a `speed` m/s.
    """
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate
    samples = []
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(int(np.linalg.norm(b - a) / (speed * dt)), 1)
        samples += [a + (b - a) * k / n for k in range(n)]
    samples += [np.asarray(path[-1], float)] * int(wait * rate)
    log_c, t, obs = math.log(400.0), 0.0, []
    for p in samples:
        true = 400.0 + sum(2000.0 * math.exp(-math.hypot(p[0] - sx, p[1] - sy))
                           for sx, sy in sources)
        log_c += (1 - math.exp(-dt / TAU)) * (math.log(true) - log_c)
        reading = math.exp(log_c + rng.normal(0, noise))
        obs.append(Co2Observation(concentration=reading, position=(p[0], p[1]), stamp_sec=t))
        t += dt
    return obs


def run(observations, **params):
    det = create_detector('co2', params)
    found = []
    for obs in observations:
        found = det.detect(obs)
    return det, found


def test_registered_as_co2_sensor():
    det = create_detector('co2', {})
    assert det.sensor == 'co2' and det.tracker_params()['cell_size'] > 0.1


def test_clean_air_gives_nothing():
    path = [(-6, 0), (6, 0), (6, 3), (-6, 3)]
    det, found = run(drive(path, sources=[]))
    assert found == []
    assert det.baseline() == pytest.approx(400.0, rel=0.05)


def test_lag_compensation_tracks_true_concentration():
    # Paso de 400 a ~2400 ppm: la lectura tarda decenas de segundos; la estimación, no.
    det = create_detector('co2', {})
    log_c, est = math.log(400.0), []
    for k in range(80):                       # 40 s a 2 Hz, quieto en una pluma de 2400 ppm
        log_c += (1 - math.exp(-0.5 / TAU)) * (math.log(2400.0) - log_c)
        det.detect(Co2Observation(math.exp(log_c), (0.0, 0.0), stamp_sec=k * 0.5))
        est.append(det.last_estimate)
    reading_10s = math.exp(math.log(400) + (1 - math.exp(-10 / TAU)) *
                           (math.log(2400) - math.log(400)))
    assert reading_10s < 1000                            # la lectura a los 10 s va muy atrás
    assert est[20] == pytest.approx(2400.0, rel=0.05)   # la estimación ya da la real


def test_source_beside_two_passes_is_located():
    # Fuente a 0.8 m del primer recorrido; un segundo recorrido perpendicular la cruza.
    source = (0.5, 0.8)
    path = [(-6, 0), (6, 0), (6, -4), (0.0, -4), (0.0, 5), (-6, 5)]
    _, found = run(drive(path, [source]))
    assert len(found) == 1
    est = found[0].points[0]
    assert math.hypot(est[0] - source[0], est[1] - source[1]) < 1.0


def test_two_separate_sources():
    sources = [(-4.0, 0.6), (4.0, -0.6)]
    path = [(-8, 0), (8, 0), (8, 3), (4, 3), (4, -3), (-4, -3), (-4, 3)]
    _, found = run(drive(path, sources))
    assert len(found) == 2
    for sx, sy in sources:
        assert min(math.hypot(d.points[0][0] - sx, d.points[0][1] - sy) for d in found) < 1.2


def test_drifting_estimates_confirm_one_victim():
    # Las estimaciones de la fuente derivan ±1 m entre actualizaciones: un solo grupo confirmado.
    from usar_detection.tracker import VictimTracker
    det = create_detector('co2', {})
    tracker = VictimTracker(5, min_cell_frames=2, **det.tracker_params())
    rng = np.random.default_rng(3)
    for k in range(200):
        drift = rng.normal(0.0, 0.5, 2)
        tracker.update(np.array([[2.0 + drift[0], 1.0 + drift[1], 0.1]]), k * 0.5)
    confirmed = tracker.confirmed()
    assert len(confirmed) == 1
    assert math.hypot(confirmed[0].position[0] - 2.0, confirmed[0].position[1] - 1.0) < 0.3
