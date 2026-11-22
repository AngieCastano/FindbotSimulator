"""Pruebas del modelo de la señal del radar Doppler HB100."""

import math

import numpy as np
from usar_sim_sensors.radar_model import BreathingVictims, HB100Model, model_params

BOX = {'type': 'box', 'x': 1.0, 'y': 0.0, 'size_x': 0.3, 'size_y': 1.0, 'yaw': 0.0}


def _model(victims, obstacles=(), fraction=1.0, seed=0, **overrides):
    rng = np.random.default_rng(seed)
    p = model_params(overrides)
    breathing = BreathingVictims(len(victims), fraction, p['rate_bpm'], p['displacement'], rng)
    return HB100Model(victims, list(obstacles), breathing, p, rng)


def _rms(model, seconds, xy=(0.0, 0.0), yaw=0.0, speed=0.0):
    out = [model.block(t, 20, xy, yaw, speed)[0] for t in range(seconds)]
    return float(np.std(np.concatenate(out[-5:])))


def test_breathing_is_reproducible_and_in_range():
    a = BreathingVictims(10, 0.5, (12.0, 20.0), (0.004, 0.008), np.random.default_rng(3))
    b = BreathingVictims(10, 0.5, (12.0, 20.0), (0.004, 0.008), np.random.default_rng(3))
    assert a.breathing == b.breathing and a.freq == b.freq
    assert len(a.breathing) == 5
    assert all(0.2 <= f <= 20.0 / 60.0 for f in a.freq.values())
    assert all(0.004 <= d <= 0.008 for d in a.amp.values())


def test_amplitude_range_beam_and_obstacles():
    m = _model([(1.0, 0.0), (2.0, 0.0), (-1.0, 0.0), (math.cos(math.radians(40)),
                                                      math.sin(math.radians(40)))])
    a1, a2 = m.amplitude(0, (0.0, 0.0), 0.0), m.amplitude(1, (0.0, 0.0), 0.0)
    assert math.isclose(a1, m.p['ref_amplitude'], rel_tol=1e-9)
    assert math.isclose(a2, a1 / 4.0, rel_tol=1e-9)                  # 1/r²
    assert m.amplitude(2, (0.0, 0.0), 0.0) == 0.0                    # detrás de la antena
    edge = m.amplitude(3, (0.0, 0.0), 0.0)
    assert math.isclose(20 * math.log10(edge / a1), -6.0, abs_tol=1e-6)  # −3 dB por vía
    blocked = _model([(2.0, 0.0)], [BOX])
    loss = 20 * math.log10(blocked.amplitude(0, (0.0, 0.0), 0.0) / a2)
    assert math.isclose(loss, -2 * m.p['obstacle_attenuation_db'], abs_tol=1e-6)


def test_block_shape_rate_and_quantization():
    m = _model([(1.5, 0.0)])
    samples, saturated = m.block(0.0, 20, (0.0, 0.0), 0.0, 0.0)
    assert samples.shape == (20,)
    assert not saturated
    assert np.all(samples >= m.p['adc_min']) and np.all(samples <= m.p['adc_max'])


def test_breathing_victim_raises_signal_when_still():
    quiet = _rms(_model([(50.0, 50.0)], fraction=0.0), 15)
    victim = _rms(_model([(1.5, 0.0)]), 15)
    assert victim > 10 * quiet


def test_motion_masks_breathing():
    """En marcha los ecos del escenario (Doppler 2·v/λ) superan a la respiración."""
    still = _rms(_model([(1.5, 0.0)]), 15)
    moving = _rms(_model([(1.5, 0.0)]), 15, speed=0.2)
    assert moving > 2 * still
    slow = _rms(_model([(1.5, 0.0)]), 15, speed=0.05)   # Doppler en banda: aún mayor
    assert slow > moving
