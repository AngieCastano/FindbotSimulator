"""
Modelo de la señal IF del radar Doppler HB100 (10,525 GHz) con su acondicionamiento.

Gazebo no simula radar. Este modelo (Capa 2, excepción del ground truth: conoce dónde están las
víctimas y los escombros) calcula la tensión que vería el ADC del ESP32:

1. **Respiración.** Cada víctima que respira (`breathing_fraction`) mueve el pecho
   d(t) = D·sin(2π f t + ψ), con D de 4–8 mm y f de 12–20 respiraciones/min (sorteados,
   reproducible por semilla). El mezclador del HB100 entrega A·cos(φ0 + 4π d(t) / λ): una
   modulación de fase (λ = 2,85 cm, unos 2 rad de desviación), no un tono puro.
2. **Amplitud.** A = `ref_amplitude` (tensión IF de un pecho a 1 m en el eje) · (1 m / r)² ·
   patrón de la antena (−3 dB a ±`beam_half_width_deg`, nada detrás) · `obstacle_attenuation_db`
   por escombro en la línea, dos veces (ida y vuelta).
3. **Robot en marcha.** El escenario estático se mueve respecto a la antena: ecos con
   desplazamiento Doppler f = 2·v/λ (≈ 14 Hz a 0,2 m/s) y amplitud `clutter_amplitude`, mucho
   mayor que la respiración. Al parar, el último valor queda como un escalón que los filtros
   pasa-altos tardan segundos en descargar: por eso el radar solo sirve con el robot quieto y
   tras un tiempo de asentamiento.
4. **Circuito (hardware/CLAUDE.md).** Ruido del mezclador y del amplificador (`noise_rms`,
   referido a la entrada), 2 etapas de `gain_db` en total con banda 0,11–43 Hz, saturación del
   ADC, cuantización, y promedio de bloques de `decimation` muestras (200 → 20 Hz) como hará el
   firmware antes de publicar.

Solo produce la tensión medida: nunca posiciones, distancias ni identificadores.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from scipy import signal

from usar_sim_sensors.sound_model import count_obstacles_between


class BreathingVictims:
    """Parámetros de respiración de cada víctima (reproducibles por semilla)."""

    def __init__(self, n_victims: int, breathing_fraction: float,
                 rate_bpm: Tuple[float, float], displacement: Tuple[float, float],
                 rng: np.random.Generator):
        n_breathing = int(round(breathing_fraction * n_victims))
        chosen = rng.choice(n_victims, size=n_breathing, replace=False) if n_breathing else []
        self.breathing = sorted(int(i) for i in chosen)
        self.freq = {i: float(rng.uniform(*rate_bpm)) / 60.0 for i in self.breathing}
        self.amp = {i: float(rng.uniform(*displacement)) for i in self.breathing}
        self.psi = {i: float(rng.uniform(0.0, 2 * math.pi)) for i in self.breathing}
        self.phi0 = {i: float(rng.uniform(0.0, 2 * math.pi)) for i in self.breathing}


class HB100Model:
    """Genera bloques de la señal IF acondicionada, muestreada y promediada."""

    def __init__(self, victims: Sequence[Tuple[float, float]], obstacles: Sequence[Dict],
                 breathing: BreathingVictims, params: Dict, rng: np.random.Generator):
        self.victims = [tuple(v) for v in victims]
        self.obstacles = list(obstacles)
        self.breathing = breathing
        self.p = params
        self.rng = rng
        fs = float(params['adc_rate'])
        # Cada etapa: pasa-altos (Cin, Rin) y pasa-bajos (Cf, Rf) de primer orden = un pasabanda
        # de orden 1. Las dos etapas en cascada.
        stage = signal.butter(1, [params['band_low'], params['band_high']], btype='bandpass',
                              fs=fs, output='sos')
        self.sos = np.vstack([stage, stage])
        self.zi = np.zeros((self.sos.shape[0], 2))
        self.gain = 10.0 ** (params['gain_db'] / 20.0)
        self.clutter_phase = float(rng.uniform(0.0, 2 * math.pi))

    def amplitude(self, i: int, radar_xy, radar_yaw: float) -> float:
        """Amplitud IF (V, antes del amplificador) del eco del pecho de la víctima i."""
        p = self.p
        vx, vy = self.victims[i]
        r = max(math.hypot(vx - radar_xy[0], vy - radar_xy[1]), p['min_distance'])
        bearing = math.atan2(vy - radar_xy[1], vx - radar_xy[0]) - radar_yaw
        bearing = abs(math.atan2(math.sin(bearing), math.cos(bearing)))
        if bearing >= math.pi / 2:
            return 0.0                                     # detrás de la antena de parche
        # Ganancia de potencia de la antena (una vía): −3 dB en el borde del haz. La amplitud
        # recibida es proporcional a G (ida y vuelta: G² en potencia), igual que la atenuación.
        pattern_db = -3.0 * (bearing / math.radians(p['beam_half_width_deg'])) ** 2
        n = count_obstacles_between((vx, vy), radar_xy, self.obstacles)
        gain_db = 2.0 * pattern_db - 2.0 * n * p['obstacle_attenuation_db']
        return p['ref_amplitude'] * (1.0 / r) ** 2 * 10.0 ** (gain_db / 20.0)

    def block(self, t0: float, n_out: int, radar_xy, radar_yaw: float, speed: float,
              speed_start: Optional[float] = None) -> Tuple[np.ndarray, bool]:
        """
        n_out muestras promediadas desde el instante t0 (s).

        `speed` es la velocidad efectiva del escenario respecto a la antena (avance más giro) al
        final del bloque, en m/s; si se da `speed_start` (al inicio), varía linealmente entre
        ambas (un arranque o un tirón pasa por todas las frecuencias Doppler intermedias).
        Devuelve (V en el ADC respecto a VREF, saturado).
        """
        p = self.p
        dec = int(p['decimation'])
        fs = float(p['adc_rate'])
        n = n_out * dec
        t = t0 + np.arange(n) / fs
        x = self.rng.normal(0.0, p['noise_rms'], n)
        b = self.breathing
        k = 4.0 * math.pi / p['wavelength']
        for i in b.breathing:
            a = self.amplitude(i, radar_xy, radar_yaw)
            if a <= 0.0:
                continue
            chest = b.amp[i] * np.sin(2 * math.pi * b.freq[i] * t + b.psi[i])
            x += a * np.cos(b.phi0[i] + k * chest)
        # Ecos del escenario: su fase gira con la velocidad (Doppler) y se congela al parar.
        start = speed if speed_start is None else speed_start
        doppler = 2.0 * np.linspace(start, speed, n) / p['wavelength']
        phases = self.clutter_phase + 2 * math.pi * np.cumsum(doppler) / fs
        self.clutter_phase = float(phases[-1] % (2 * math.pi))
        x += p['clutter_amplitude'] * np.cos(phases)
        y, self.zi = signal.sosfilt(self.sos, self.gain * x, zi=self.zi)
        lo, hi = p['adc_min'], p['adc_max']
        saturated = bool(np.any(y <= lo) or np.any(y >= hi))
        y = np.clip(y, lo, hi)
        y = np.round(y / p['adc_lsb']) * p['adc_lsb']
        return y.reshape(n_out, dec).mean(axis=1), saturated


DEFAULTS = {
    'breathing_fraction': 1.0,
    'rate_bpm': [12.0, 20.0],
    'displacement': [0.004, 0.008],
    'wavelength': 0.0285,
    'ref_amplitude': 60.0e-6,
    'beam_half_width_deg': 40.0,
    'obstacle_attenuation_db': 8.0,
    'min_distance': 0.3,
    'clutter_amplitude': 150.0e-6,
    'noise_rms': 3.0e-6,
    'gain_db': 66.8,
    'band_low': 0.11,
    'band_high': 43.0,
    'adc_rate': 200.0,
    'decimation': 10,
    'adc_min': -1.25,
    'adc_max': 1.10,
    'adc_lsb': 0.0008,
}


def model_params(overrides: Optional[Dict] = None) -> Dict:
    """Parámetros del modelo (valores por defecto + los dados)."""
    return {**DEFAULTS, **(overrides or {})}
