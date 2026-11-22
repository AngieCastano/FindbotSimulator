"""
Modelo del enlace de datos WiFi robot ↔ estación, sin dependencias de ROS.

A partir del RSSI que reporta la radio (usar_msgs/LinkQuality) decide qué le pasa a cada
mensaje, como la capa MAC de 802.11:

1. **Margen** = RSSI − sensibilidad del receptor (dB).
2. **Velocidad física** según el margen (adaptación de tasa): a más margen, más Mb/s.
3. **Tasa de error por trama** PER = 1 / (1 + exp(margen / pendiente)) (50 % en el umbral).
4. **Reintentos**: cada intento falla con probabilidad PER; tras `retry_limit` reintentos la
   trama se pierde (en 802.11 el límite habitual es 7).
5. **Canal compartido** (semidúplex): los mensajes se transmiten de uno en uno; si la cola
   acumula más de `max_latency` s, el mensaje se descarta (búfer lleno).
6. **Prioridad** (WMM de 802.11e): los mensajes de control (mandos, metas, latido) tienen su
   propia cola y no esperan detrás del video; sí ocupan el canal (retrasan al tráfico normal).

Las imágenes se transmitirían comprimidas: su tiempo en el aire usa el tamaño entre
`compression_ratio` (JPEG ≈ 20:1 para 640 × 480).
"""

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

#: (margen mínimo en dB, velocidad física en Mb/s), de mayor a menor.
DEFAULT_RATES = [(25.0, 65.0), (18.0, 39.0), (12.0, 19.5), (6.0, 6.5), (0.0, 1.0)]


def packet_error_rate(margin: float, slope: float) -> float:
    """Probabilidad de que falle un intento de transmisión (0..1)."""
    return 1.0 / (1.0 + math.exp(max(min(margin / slope, 50.0), -50.0)))


def phy_rate(margin: float, rates: Sequence[Tuple[float, float]]) -> float:
    """Velocidad física (Mb/s) para el margen dado; la mínima si no alcanza ninguna."""
    for min_margin, rate in rates:
        if margin >= min_margin:
            return rate
    return rates[-1][1]


class LinkChannel:
    """Canal compartido: calcula cuándo llega cada mensaje, o si se pierde."""

    def __init__(self, params: Dict, rng: np.random.Generator):
        self.p = params
        self.rng = rng
        self.free_at = 0.0             # cola normal
        self.free_at_high = 0.0        # cola de control (prioridad)

    def send(self, now: float, size_bytes: int, rssi: Optional[float],
             compressible: bool = False, high_priority: bool = False) -> Optional[float]:
        """Instante de llegada (s) del mensaje enviado en `now`, o None si se pierde."""
        p = self.p
        if rssi is None:                      # sin modelo de señal: enlace ideal
            return now + p['base_latency']
        margin = rssi - p['sensitivity_dbm']
        if margin < p['min_margin']:
            return None                       # sin enlace (asociación perdida)
        size = size_bytes / (p['compression_ratio'] if compressible else 1.0)
        airtime = (size * 8.0) / (phy_rate(margin, p['rates']) * 1e6) + p['frame_overhead']
        per = packet_error_rate(margin, p['per_slope_db'])
        attempts = 1
        lost = False
        while self.rng.random() < per:
            attempts += 1
            if attempts > p['retry_limit'] + 1:
                lost = True                   # agotó los reintentos
                break
        busy = attempts * airtime
        if high_priority:
            start = max(now, self.free_at_high)
            self.free_at_high = start + busy
            self.free_at = max(self.free_at, now) + busy     # el control también ocupa el aire
        else:
            start = max(now, self.free_at)
            if start + busy + p['base_latency'] - now > p['max_latency']:
                return None                   # cola llena: se descarta sin transmitir
            self.free_at = start + busy
        return None if lost else start + busy + p['base_latency']


DEFAULTS = {
    'sensitivity_dbm': -90.0,
    'per_slope_db': 1.5,
    'min_margin': -6.0,
    'retry_limit': 7,
    'frame_overhead': 0.0003,
    'base_latency': 0.004,
    'max_latency': 1.0,
    'compression_ratio': 20.0,
    'rates': DEFAULT_RATES,
}
