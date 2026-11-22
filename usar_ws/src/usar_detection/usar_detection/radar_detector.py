"""
Signo de vida "movimiento respiratorio" con el radar Doppler HB100 (fase 2).

El HB100 es un radar de onda continua: no mide distancia ni dirección, solo si algo se mueve
dentro de su haz (unos ±40°) y con qué ritmo. El pecho de una persona que respira (4–8 mm a
12–20 respiraciones/min) modula la fase del eco, y la señal IF acondicionada oscila al ritmo de
la respiración (y sus armónicos).

Análisis de cada ventana (robot quieto):

1. **Robot quieto y circuito asentado.** Con el robot en marcha los ecos del escenario tapan
   todo; al parar, el escalón que dejan tarda segundos en descargarse por los pasa-altos del
   circuito. Incluso un giro de pocos mrad/s pone esos ecos en la banda de la respiración, y
   un tirón de 1° deja un transitorio. Solo se analiza si el radar lleva
   `settle_time + window + guard_time` s sin moverse más de `stationary_motion` m ni girar más
   de `stationary_turn` rad (pose del propio robot, TF). La ventana termina `guard_time` s antes
   de la última muestra: si el robot arranca justo al final, la pose ya lo delata y la ventana
   no incluye el arranque. Las paradas de escucha del explorador dan esas ocasiones.
2. **Quitar la cola del asentamiento**: se resta un polinomio de grado `detrend_order`.
3. **Espectro** (ventana de Hann, con ceros hasta `nfft` puntos): valor eficaz en la banda de
   respiración (`breath_band`, incluye los armónicos de la modulación de fase) y densidad de
   ruido en `noise_band` (sin señales fisiológicas).
4. **Decisión**: respiración si la densidad en la banda supera a la del ruido en `min_snr_db`
   y su valor eficaz supera `min_band_rms` V (rechaza restos del asentamiento).

Como no hay distancia, la detección se ubica en el eje de la antena a `nominal_range` m: es una
guía (cue_only) para que el explorador vaya a mirar y la cámara confirme.
"""

from collections import deque
import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .sign_of_life import Detection, RadarObservation, register_detector, SignOfLifeDetector

DEFAULTS = {
    'window': 5.0,                 # s analizados
    'settle_time': 2.5,            # s quieto antes de la ventana (asentamiento del circuito)
    'guard_time': 1.0,             # s entre el final de la ventana y la última muestra
    'stationary_motion': 0.01,     # m
    'stationary_turn': 0.01,       # rad
    'detrend_order': 2,
    'nfft': 512,
    'breath_band': [0.15, 1.0],    # Hz
    'noise_band': [2.0, 8.0],      # Hz
    'min_snr_db': 10.0,
    'min_band_rms': 0.003,         # V
    'nominal_range': 1.5,          # m sobre el eje de la antena
    'evidence_height': 0.1,        # m (solo visualización)
}


def breathing_analysis(samples: Sequence[float], rate: float, params: Dict
                       ) -> Tuple[float, float, float]:
    """
    Analiza una ventana de la señal IF.

    Devuelve (valor eficaz en la banda de respiración [V], SNR de densidades [dB],
    frecuencia del pico en la banda [Hz]).
    """
    x = np.asarray(samples, dtype=float)
    t = np.arange(len(x)) / rate
    if len(x) > params['detrend_order'] + 1:
        x = x - np.polyval(np.polyfit(t, x, int(params['detrend_order'])), t)
    w = np.hanning(len(x))
    nfft = max(int(params['nfft']), len(x))
    spec = np.abs(np.fft.rfft(x * w, nfft)) ** 2
    freqs = np.fft.rfftfreq(nfft, 1.0 / rate)
    # Potencia (V²) por bin, normalizada para que la suma de todos los bins sea la media de x².
    power = spec * 2.0 / (np.sum(w ** 2) * nfft / len(x)) / len(x)
    df = freqs[1] - freqs[0]
    lo, hi = params['breath_band']
    nlo, nhi = params['noise_band']
    band = (freqs >= lo) & (freqs <= hi)
    noise = (freqs >= nlo) & (freqs <= min(nhi, rate / 2.0 - df))
    band_power = float(power[band].sum())
    band_density = band_power / max(band.sum() * df, 1e-12)
    noise_density = float(power[noise].sum()) / max(noise.sum() * df, 1e-12)
    snr_db = 10.0 * math.log10((band_density + 1e-18) / (noise_density + 1e-18))
    peak = float(freqs[band][np.argmax(power[band])]) if band.any() else 0.0
    return math.sqrt(band_power), snr_db, peak


@register_detector
class RadarDetector(SignOfLifeDetector):
    """Ventanas de la señal del HB100 con el robot quieto → punto en el eje de la antena."""

    name = 'radar'
    sensor = 'radar'
    position_sigma = 1.5          # m: sin distancia ni dirección precisa

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **params})
        self._samples = deque()                    # (t, v)
        self._poses = deque()                      # (t, x, y, yaw)
        self.windows = 0                           # ventanas analizadas
        self.last = None                           # (rms, snr, pico) de la última ventana

    def tracker_params(self) -> Dict[str, float]:
        """Evidencia en celdas grandes: la ubicación es solo orientativa."""
        return {'cell_size': 0.5, 'join_distance': 1.0, 'min_cell_frames': 1,
                'min_observations': 2}

    def _still_since(self, obs: RadarObservation) -> float:
        """Segundos que lleva el radar quieto (según la pose del robot)."""
        p = self.params
        self._poses.append((obs.stamp_sec, obs.position[0], obs.position[1], obs.yaw))
        horizon = p['settle_time'] + p['window'] + p['guard_time'] + 1.0
        while self._poses and obs.stamp_sec - self._poses[0][0] > horizon:
            self._poses.popleft()
        still = 0.0
        for t, x, y, yaw in reversed(self._poses):
            turn = abs(math.atan2(math.sin(yaw - obs.yaw), math.cos(yaw - obs.yaw)))
            if math.hypot(x - obs.position[0], y - obs.position[1]) > p['stationary_motion'] \
                    or turn > p['stationary_turn']:
                break
            still = obs.stamp_sec - t
        return still

    def detect(self, observation: RadarObservation) -> List[Detection]:
        """Un punto (frame global) si la última ventana con el robot quieto tiene respiración."""
        p = self.params
        rate = float(observation.sample_rate)
        n = len(observation.samples)
        for k, v in enumerate(observation.samples):
            self._samples.append((observation.stamp_sec - (n - 1 - k) / rate, float(v)))
        end = observation.stamp_sec - p['guard_time']
        while self._samples and end - self._samples[0][0] > p['window']:
            self._samples.popleft()
        if observation.saturated:
            self._samples.clear()               # el bloque saturado no sirve para analizar
        still = self._still_since(observation)
        window = [v for t, v in self._samples if t <= end + 1e-6]
        needed = int(round(p['window'] * rate)) - 1
        if still < p['settle_time'] + p['window'] + p['guard_time'] or len(window) < needed:
            return []
        rms, snr, peak = breathing_analysis(window, rate, p)
        self.windows += 1
        self.last = (rms, snr, peak)
        if snr < p['min_snr_db'] or rms < p['min_band_rms']:
            return []
        x = observation.position[0] + p['nominal_range'] * math.cos(observation.yaw)
        y = observation.position[1] + p['nominal_range'] * math.sin(observation.yaw)
        point = np.array([[x, y, p['evidence_height']]])
        return [Detection(kind=self.name, point=tuple(point[0]), pixel=(0.0, 0.0),
                          confidence=float(min(1.0, (snr - p['min_snr_db']) / 20.0 + 0.5)),
                          extra={'band_rms': rms, 'snr_db': snr, 'breath_hz': peak},
                          points=point)]
