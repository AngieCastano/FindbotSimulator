"""Ruta de barrido del operador simulado (sin dependencias de ROS)."""

import math
from typing import List, Optional, Tuple

import numpy as np


def sweep_waypoints(size: Tuple[float, float], margin: float, spacing: float,
                    start: Tuple[float, float] = (0.0, 0.0)) -> List[Tuple[float, float]]:
    """
    Recorrido en zigzag (boustrophedon) por carriles paralelos a x cada `spacing` m.

    Empieza por el carril más cercano a `start` y recorre primero la mitad del área donde está,
    como haría un operador que sale del punto de entrada.
    """
    half_w, half_h = size[0] / 2.0 - margin, size[1] / 2.0 - margin
    n = max(1, int(math.ceil(2 * half_h / spacing - 1e-9)) + 1)   # separación ≤ spacing
    lanes = [-half_h + i * (2 * half_h / max(n - 1, 1)) for i in range(n)]
    first = int(np.argmin([abs(y - start[1]) for y in lanes]))
    order = list(range(first, n)) + list(range(first - 1, -1, -1))
    points, left_to_right = [], start[0] <= 0.0
    for idx in order:
        xs = (-half_w, half_w) if left_to_right else (half_w, -half_w)
        points += [(xs[0], lanes[idx]), (xs[1], lanes[idx])]
        left_to_right = not left_to_right
    return points


def free_near(grid: np.ndarray, origin: Tuple[float, float], resolution: float,
              point: Tuple[float, float], radius: float,
              occupied: int = 65) -> Optional[Tuple[float, float]]:
    """La celda no ocupada del mapa más cercana a `point` (a menos de `radius` m), o None."""
    h, w = grid.shape
    col = int((point[0] - origin[0]) / resolution)
    row = int((point[1] - origin[1]) / resolution)
    r = int(radius / resolution)
    best, best_d = None, None
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            rr, cc = row + dr, col + dc
            if not (0 <= rr < h and 0 <= cc < w):
                continue
            if 0 <= grid[rr, cc] < occupied or grid[rr, cc] < 0:
                d = dr * dr + dc * dc
                if d <= r * r and (best_d is None or d < best_d):
                    best, best_d = (rr, cc), d
    if best is None:
        return None
    return (origin[0] + (best[1] + 0.5) * resolution, origin[1] + (best[0] + 0.5) * resolution)
