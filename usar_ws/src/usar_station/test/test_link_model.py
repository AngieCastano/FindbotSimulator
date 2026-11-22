"""Pruebas del modelo del enlace de datos robot ↔ estación."""

import numpy as np
import pytest
from usar_station.link_model import DEFAULTS, LinkChannel, packet_error_rate, phy_rate


def _channel(seed=0, **over):
    return LinkChannel({**DEFAULTS, **over}, np.random.default_rng(seed))


def test_per_is_half_at_sensitivity_and_monotonic():
    assert packet_error_rate(0.0, 1.5) == pytest.approx(0.5)
    assert packet_error_rate(10.0, 1.5) < 0.01 < 0.99 < packet_error_rate(-10.0, 1.5)


def test_phy_rate_follows_margin():
    rates = DEFAULTS['rates']
    assert phy_rate(30.0, rates) == 65.0 and phy_rate(7.0, rates) == 6.5
    assert phy_rate(-3.0, rates) == 1.0


def test_ideal_link_without_rssi():
    assert _channel().send(10.0, 1000, None) == pytest.approx(10.0 + DEFAULTS['base_latency'])


def test_strong_signal_is_fast_and_weak_signal_loses_messages():
    strong = _channel()
    t = strong.send(0.0, 1000, -50.0)
    assert t is not None and t < 0.01
    weak = _channel()
    lost = sum(weak.send(float(i), 1000, -95.0) is None for i in range(500))
    assert lost > 50                          # margen −5 dB: la mayoría de tramas fallan
    assert _channel().send(0.0, 1000, -100.0) is None     # bajo min_margin: sin enlace


def test_queue_delays_and_drops_when_saturated():
    ch = _channel()
    arrivals = [ch.send(0.0, 400_000, -84.0) for _ in range(5)]   # 0.4 MB a 6.5 Mb/s ≈ 0.5 s
    assert arrivals[0] is not None and 0.4 < arrivals[0] < 1.0
    assert any(a is None for a in arrivals)  # la cola supera max_latency


def test_compression_shortens_images():
    a = _channel().send(0.0, 900_000, -70.0, compressible=False)
    b = _channel().send(0.0, 900_000, -70.0, compressible=True)
    assert b < a


def test_control_priority_does_not_wait_behind_video():
    ch = _channel()
    for _ in range(3):
        ch.send(0.0, 2_000_000, -75.0, compressible=False)   # cola llena de tráfico normal
    t = ch.send(0.0, 100, -75.0, high_priority=True)
    assert t is not None and t < 0.05
