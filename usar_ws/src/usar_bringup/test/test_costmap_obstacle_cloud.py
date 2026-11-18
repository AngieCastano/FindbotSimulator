"""Pruebas de la conversión de celdas letales del costmap a puntos."""

import numpy as np
from usar_bringup.costmap_obstacle_cloud import lethal_cells_to_points


def test_only_lethal_cells_become_points():
    """Solo las celdas letales (100) generan puntos, en el centro de la celda."""
    data = [0, 99, 100,
            -1, 100, 50]
    pts = lethal_cells_to_points(data, width=3, height=2, resolution=0.1,
                                 origin_xy=(1.0, -2.0), lethal_value=100, z=0.1)
    assert pts.shape == (2, 3)
    expected = np.array([[1.25, -1.95, 0.1], [1.15, -1.85, 0.1]], dtype=np.float32)
    assert np.allclose(pts, expected)


def test_empty_costmap_gives_no_points():
    """Un costmap sin obstáculos produce una nube vacía."""
    pts = lethal_cells_to_points([0] * 16, 4, 4, 0.05, (0.0, 0.0), 100, 0.1)
    assert pts.shape == (0, 3)
