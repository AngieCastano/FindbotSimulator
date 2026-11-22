"""Pruebas de la ruta de barrido del operador simulado."""

import numpy as np
import pytest
from usar_station.sweep import free_near, sweep_waypoints


def test_sweep_covers_area_in_lanes():
    pts = sweep_waypoints((20.0, 20.0), 1.5, 3.0, (0.0, 0.0))
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    assert min(xs) == pytest.approx(-8.5) and max(xs) == pytest.approx(8.5)
    assert min(ys) == pytest.approx(-8.5) and max(ys) == pytest.approx(8.5)
    lanes = sorted({round(y, 6) for y in ys})
    assert len(lanes) >= 6 and np.diff(lanes).max() <= 3.0 + 1e-9
    assert abs(pts[0][1]) <= 1.5             # empieza por el carril más cercano al robot


def test_free_near_moves_goal_out_of_obstacle():
    grid = np.zeros((40, 40), dtype=np.int16)
    grid[18:23, 18:23] = 100                 # obstáculo de 0.25 m en el centro
    p = free_near(grid, (0.0, 0.0), 0.05, (1.0, 1.0), 0.5)
    assert p is not None
    row, col = int(p[1] / 0.05), int(p[0] / 0.05)
    assert grid[row, col] < 65
    full = np.full((10, 10), 100, dtype=np.int16)
    assert free_near(full, (0.0, 0.0), 0.05, (0.25, 0.25), 0.1) is None
