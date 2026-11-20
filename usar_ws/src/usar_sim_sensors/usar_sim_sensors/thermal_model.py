"""
Modelo del MLX90640 sobre la imagen ideal de la cámara térmica de Gazebo.

Gazebo entrega la temperatura exacta de cada píxel (mono16, `resolution` K por unidad), sin
ruido: su sensor térmico no admite <noise>. Aquí se añaden las imperfecciones del sensor real:
  - ruido temporal gaussiano (NETD) en cada imagen,
  - ruido de patrón fijo: un desvío propio de cada píxel, constante en el tiempo,
  - saturación al rango de medida del sensor.
Solo transforma la imagen: no usa la pose del robot ni el ground truth.
"""

from typing import Optional

import numpy as np

KELVIN = 273.15


def raw_to_celsius(raw: np.ndarray, resolution: float) -> np.ndarray:
    """Imagen mono16 de Gazebo (resolution K por unidad) → °C en float32."""
    return (raw.astype(np.float32) * np.float32(resolution) - np.float32(KELVIN))


class ThermalNoiseModel:
    """Ruido del sensor térmico, reproducible por semilla."""

    def __init__(self, width: int, height: int, netd: float, fixed_pattern: float,
                 min_temp: float, max_temp: float, seed: int = 0):
        self.rng = np.random.default_rng(seed)
        self.netd = float(netd)
        self.min_temp, self.max_temp = float(min_temp), float(max_temp)
        self.pattern = (self.rng.normal(0.0, fixed_pattern, (height, width)).astype(np.float32)
                        if fixed_pattern > 0 else np.zeros((height, width), np.float32))

    def apply(self, celsius: np.ndarray, rng: Optional[np.random.Generator] = None
              ) -> np.ndarray:
        """Imagen en °C con ruido temporal y de patrón fijo, saturada al rango del sensor."""
        rng = rng or self.rng
        if celsius.shape != self.pattern.shape:
            raise ValueError(f'imagen {celsius.shape} distinta de la configurada '
                             f'{self.pattern.shape}')
        noisy = celsius + self.pattern
        if self.netd > 0:
            noisy = noisy + rng.normal(0.0, self.netd, celsius.shape).astype(np.float32)
        return np.clip(noisy, self.min_temp, self.max_temp).astype(np.float32)
