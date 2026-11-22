"""Pruebas del detector de respiración con el radar Doppler."""

import math
import os

import numpy as np
from usar_detection.radar_detector import breathing_analysis, DEFAULTS, RadarDetector
from usar_detection.sign_of_life import create_detector, RadarObservation
import yaml

RATE = 20.0


def _signal(seconds, freq=0.25, amp=0.05, noise=0.002, seed=0, t0=0.0):
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(int(seconds * RATE)) / RATE
    return amp * np.sin(2 * math.pi * freq * t) + rng.normal(0.0, noise, len(t))


def _feed(det, seconds, position=(0.0, 0.0), yaw=0.0, amp=0.05, move=0.0, saturated=False,
          turns=None):
    """Bloques de 1 s; con move > 0 el robot avanza move m por bloque; turns: giro por bloque."""
    detections = []
    x = position[0]
    heading = yaw
    for k in range(seconds):
        block = _signal(1.0, amp=amp, seed=k, t0=float(k))
        x += move
        heading += turns[k] if turns else 0.0
        obs = RadarObservation(samples=list(block), sample_rate=RATE, saturated=saturated,
                               position=(x, position[1]), yaw=heading, stamp_sec=float(k + 1))
        detections += det.detect(obs)
    return detections


#: Bloques de 1 s quieto antes de la primera ventana (la quietud se cuenta desde el primero).
NEEDED = math.ceil(DEFAULTS['settle_time'] + DEFAULTS['window'] + DEFAULTS['guard_time']) + 1


def test_analysis_finds_breathing():
    rms, snr, peak = breathing_analysis(_signal(5.0), RATE, DEFAULTS)
    assert snr > 20.0
    assert rms > 0.02
    assert abs(peak - 0.25) < 0.1


def test_analysis_rejects_noise_and_slow_trend():
    noise = _signal(5.0, amp=0.0)
    rms, snr, _ = breathing_analysis(noise, RATE, DEFAULTS)
    assert snr < DEFAULTS['min_snr_db']
    t = np.arange(100) / RATE
    tail = 0.3 * np.exp(-(t + 3.0) / 1.45) * (1 - (t + 3.0) / 1.45) + noise
    rms, _, _ = breathing_analysis(tail, RATE, DEFAULTS)
    assert rms < DEFAULTS['min_band_rms']


def test_detects_only_after_settling_window_and_guard():
    assert _feed(RadarDetector({}), NEEDED - 1) == []
    found = _feed(RadarDetector({}), NEEDED + 1, position=(1.0, 2.0), yaw=math.pi / 2)
    assert len(found) == 2
    x, y, _ = found[0].point
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 2.0 + DEFAULTS['nominal_range'], abs_tol=1e-9)
    assert found[0].kind == 'radar' and found[0].points.shape == (1, 3)


def test_small_jerk_restarts_the_wait():
    """Un tirón de 0,02 rad (≈ 1°) deja un transitorio: se vuelve a esperar desde ahí."""
    turns = [0.0] * 20
    turns[5] = 0.02
    found = _feed(RadarDetector({}), 20, turns=turns)
    # Quieto desde t = 6 s: ventanas desde t = 6 + 9 = 15 s hasta t = 20 s.
    assert len(found) == 20 - (6 + NEEDED - 1) + 1


def test_motion_in_last_block_is_not_analysed():
    """Si el robot arranca en el último bloque, la pose lo delata y no hay ventana nueva."""
    turns = [0.0] * NEEDED + [0.015]
    found = _feed(RadarDetector({}), NEEDED + 1, turns=turns)
    assert len(found) == 1                       # solo la ventana anterior al arranque


def test_no_detection_while_moving_saturated_or_without_breathing():
    assert _feed(RadarDetector({}), 15, move=0.1) == []
    assert _feed(RadarDetector({}), 15, saturated=True) == []
    assert _feed(RadarDetector({}), 15, amp=0.0) == []


def test_radar_is_registered_and_cue_only_in_config():
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'detector_radar.yaml')
    with open(path, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['victim_detector']['ros__parameters']['radar']
    det = create_detector('radar', params)
    assert det.cue_only and det.sensor == 'radar'
    assert det.tracker_params()['min_observations'] == 2
