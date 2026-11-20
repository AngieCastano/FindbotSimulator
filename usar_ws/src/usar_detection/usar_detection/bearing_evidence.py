"""
Triangulación de un sensor que da direcciones pero no distancias (cámara térmica).

Cada rayo caliente marca todas las celdas que atraviesa en el /map del robot (hasta donde puede
estar la fuente: ver map_filter.ray_extents), anotando el sector de su dirección (n_bins
sectores de 360°/n_bins). Una fuente de calor real queda en la intersección de rayos tomados
desde direcciones distintas al moverse el robot; un punto mal ubicado (p. ej. el escombro que hay
detrás de una víctima cuando el rayo pasa por encima de sus piernas) cambia de sitio con cada
punto de vista. Solo las celdas cruzadas desde al menos `min_bearings` sectores distintos son
evidencia de víctima.
"""

from typing import Dict, Tuple

import numpy as np


class BearingEvidence:
    """Rejilla de intersección de rayos en el frame del mapa."""

    def __init__(self, cell_size: float = 0.15, n_bins: int = 12, min_bearings: int = 3):
        self.cell_size = float(cell_size)
        self.n_bins = int(n_bins)
        self.min_bearings = int(min_bearings)
        self._bits: Dict[Tuple[int, int], int] = {}

    def add_rays(self, start: np.ndarray, directions: np.ndarray, lengths: np.ndarray,
                 min_range: float = 0.0) -> np.ndarray:
        """
        Incorpora los rayos de una captura y devuelve la evidencia triangulada.

        Devuelve los centros (M×2) de las celdas tocadas en esta captura que ya se han cruzado
        desde min_bearings sectores distintos.

        start: (x, y); directions: N×2; lengths: N (m, longitud de cada rayo).
        """
        dirs = np.asarray(directions, dtype=float).reshape(-1, 2)
        lengths = np.asarray(lengths, dtype=float).reshape(-1)
        keep = lengths > min_range
        dirs, lengths = dirs[keep], lengths[keep]
        if len(dirs) == 0:
            return np.zeros((0, 2))
        dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        angles = np.arctan2(dirs[:, 1], dirs[:, 0]) % (2 * np.pi)
        bins = np.minimum((angles / (2 * np.pi) * self.n_bins).astype(int), self.n_bins - 1)
        step = self.cell_size / 2.0
        touched: Dict[Tuple[int, int], int] = {}
        for d, length, b in zip(dirs, lengths, bins):
            ts = np.arange(min_range, length + 1e-9, step)
            cells = np.floor((start[None, :2] + ts[:, None] * d[None, :]) / self.cell_size)
            for key in set(map(tuple, cells.astype(int))):
                touched[key] = touched.get(key, 0) | (1 << int(b))
        result = []
        for key, bits in touched.items():
            total = self._bits.get(key, 0) | bits
            self._bits[key] = total
            if bin(total).count('1') >= self.min_bearings:
                result.append(key)
        if not result:
            return np.zeros((0, 2))
        return (np.array(result, dtype=float) + 0.5) * self.cell_size

    def bearings(self, x: float, y: float) -> int:
        """Número de sectores distintos desde los que se ha cruzado la celda de (x, y)."""
        key = (int(np.floor(x / self.cell_size)), int(np.floor(y / self.cell_size)))
        return bin(self._bits.get(key, 0)).count('1')
