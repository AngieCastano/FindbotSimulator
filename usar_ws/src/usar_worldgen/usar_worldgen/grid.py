"""
Rejilla de ocupación del escenario y cálculo del área explorable.

Convención (igual a nav_msgs/OccupancyGrid): la celda (fila j, columna i) tiene su centro en
(origin_x + (i + 0.5)·res, origin_y + (j + 0.5)·res); el orden de serialización es por filas.
"""

from dataclasses import dataclass
from typing import Iterable, List, Tuple

import numpy as np
from scipy import ndimage

from .geometry import point_distance, Shape


@dataclass
class GridSpec:
    """Geometría de la rejilla."""

    origin_x: float
    origin_y: float
    resolution: float
    cols: int
    rows: int

    @classmethod
    def from_bounds(cls, bounds: Tuple[float, float, float, float],
                    resolution: float) -> 'GridSpec':
        """Rejilla que cubre exactamente el área interior."""
        x_min, x_max, y_min, y_max = bounds
        cols = int(np.ceil(round((x_max - x_min) / resolution, 6)))
        rows = int(np.ceil(round((y_max - y_min) / resolution, 6)))
        return cls(x_min, y_min, resolution, cols, rows)

    def cell_centers(self) -> Tuple[np.ndarray, np.ndarray]:
        """Mallas X, Y de centros de celda, forma (rows, cols)."""
        xs = self.origin_x + (np.arange(self.cols) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.rows) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Índices (fila, columna) de la celda que contiene el punto (x, y)."""
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        return row, col


def clearance_map(spec: GridSpec, bounds: Tuple[float, float, float, float],
                  shapes: Iterable[Shape]) -> np.ndarray:
    """
    Distancia exacta de cada centro de celda al obstáculo o muro más cercano.

    Los muros se modelan como los bordes del área interior.
    """
    xg, yg = spec.cell_centers()
    x_min, x_max, y_min, y_max = bounds
    dist = np.minimum.reduce([xg - x_min, x_max - xg, yg - y_min, y_max - yg])
    for shape in shapes:
        dist = np.minimum(dist, point_distance(shape, xg, yg))
    return dist


def explorable_mask(spec: GridSpec, clearance: np.ndarray, robot_radius: float,
                    spawn_xy: Tuple[float, float]) -> np.ndarray:
    """
    Flood-fill (4-conexo) desde el spawn sobre la rejilla inflada con el radio del robot.

    Devuelve la máscara booleana de las celdas que el centro del robot puede alcanzar.
    """
    free = clearance > robot_radius
    row, col = spec.world_to_cell(*spawn_xy)
    if not free[row, col]:
        return np.zeros_like(free)
    labels, _ = ndimage.label(free)
    return labels == labels[row, col]


def rle_encode(mask: np.ndarray) -> List[int]:
    """Codifica una máscara booleana por filas como longitudes de racha, empezando por 0."""
    flat = mask.ravel(order='C').astype(np.int8)
    if flat.size == 0:
        return []
    change = np.flatnonzero(np.diff(flat)) + 1
    edges = np.concatenate(([0], change, [flat.size]))
    runs = np.diff(edges).tolist()
    if flat[0] == 1:
        runs = [0] + runs
    return [int(r) for r in runs]


def rle_decode(runs: List[int], rows: int, cols: int) -> np.ndarray:
    """Inversa de rle_encode."""
    values = np.zeros(len(runs), dtype=bool)
    values[1::2] = True
    flat = np.repeat(values, runs)
    if flat.size != rows * cols:
        raise ValueError(f'RLE con {flat.size} celdas, se esperaban {rows * cols}')
    return flat.reshape(rows, cols)
