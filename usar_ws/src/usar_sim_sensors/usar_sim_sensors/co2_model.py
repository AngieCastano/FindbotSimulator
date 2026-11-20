"""
Modelo del CO₂ exhalado por las víctimas y de la respuesta del sensor Figaro TGS4161.

Gazebo no simula gases. Este modelo (Capa 2, excepción del ground truth: conoce dónde están las
víctimas) calcula qué mediría el sensor en cada instante:

1. **Pluma de cada víctima.** Una persona en reposo exhala ≈ 0.3 L/min de CO₂; en un espacio
   semicerrado entre escombros se forma una zona de concentración alta alrededor de la cabeza que
   decae con la distancia. Se modela el exceso sobre el ambiente como
   `peak_excess · exp(−r / decay_length)` (suma de todas las víctimas): la solución estacionaria
   de difusión con pérdidas en 2D tiene esa forma lejos de la fuente.
2. **Turbulencia.** Las corrientes de aire hacen que la concentración fluctúe: el exceso se
   multiplica por `1 + σ·n(t)`, con n(t) un proceso de Ornstein-Uhlenbeck de varianza unidad y
   tiempo de correlación `turbulence_tau` (truncado para no ser negativo).
3. **Sensor TGS4161.** Electrolito sólido: su FEM es proporcional al logaritmo de la
   concentración y responde lento (constante de tiempo `response_tau`, del orden de decenas de
   segundos). Se filtra log(ppm) con un sistema de primer orden y se añade ruido gaussiano en el
   dominio logarítmico (≈ % de la lectura). Rango 350–10 000 ppm.

Solo produce una concentración (ppm): nunca posiciones ni distancias a las víctimas.
"""

import math
from typing import Optional, Sequence, Tuple

import numpy as np


class Co2Plume:
    """Concentración real de CO₂ (ppm) en un punto del plano, con turbulencia."""

    def __init__(self, sources: Sequence[Tuple[float, float]], ambient_ppm: float,
                 peak_excess_ppm: float, decay_length: float, turbulence_sigma: float,
                 turbulence_tau: float, rng: np.random.Generator):
        self.sources = np.asarray(sources, dtype=float).reshape(-1, 2)
        self.ambient = float(ambient_ppm)
        self.peak = float(peak_excess_ppm)
        self.decay = float(decay_length)
        self.sigma = float(turbulence_sigma)
        self.tau = float(turbulence_tau)
        self.rng = rng
        self._n = 0.0                      # estado del proceso de Ornstein-Uhlenbeck

    def mean_excess(self, x: float, y: float) -> float:
        """Exceso medio (sin turbulencia) sobre el ambiente en (x, y)."""
        if len(self.sources) == 0:
            return 0.0
        r = np.hypot(self.sources[:, 0] - x, self.sources[:, 1] - y)
        return float(self.peak * np.exp(-r / self.decay).sum())

    def sample(self, x: float, y: float, dt: float) -> float:
        """Concentración instantánea en (x, y) tras avanzar dt segundos la turbulencia."""
        if self.tau > 0 and dt > 0:
            a = math.exp(-dt / self.tau)
            self._n = a * self._n + math.sqrt(1.0 - a * a) * self.rng.normal()
        factor = max(0.0, 1.0 + self.sigma * self._n)
        return self.ambient + self.mean_excess(x, y) * factor


class Tgs4161:
    """Respuesta lenta (primer orden en log(ppm)), ruido y saturación del sensor."""

    def __init__(self, response_tau: float, noise_log: float, min_ppm: float, max_ppm: float,
                 rng: np.random.Generator, initial_ppm: Optional[float] = None):
        self.tau = float(response_tau)
        self.noise = float(noise_log)
        self.min_ppm, self.max_ppm = float(min_ppm), float(max_ppm)
        self.rng = rng
        self._log = None if initial_ppm is None else math.log(initial_ppm)

    def update(self, true_ppm: float, dt: float) -> float:
        """Lectura del sensor (ppm) tras dt segundos expuesto a true_ppm."""
        target = math.log(max(true_ppm, 1e-3))
        if self._log is None:
            self._log = target
        elif self.tau > 0:
            self._log += (1.0 - math.exp(-max(dt, 0.0) / self.tau)) * (target - self._log)
        else:
            self._log = target
        reading = math.exp(self._log + self.rng.normal(0.0, self.noise))
        return float(min(max(reading, self.min_ppm), self.max_ppm))
