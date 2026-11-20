"""
Filtro de evidencia con el mapa del propio robot (/map de SLAM).

Una detección cuyo punto cae en una celda desconocida o fuera del mapa no puede ser una víctima
en el espacio explorado: suele deberse a un error momentáneo de la pose (SLAM) que proyecta la
detección detrás de un muro. El LiDAR (alcance 12 m) cubre de sobra el alcance útil de la cámara
(5–6 m), así que las víctimas reales caen en celdas conocidas.
"""

import numpy as np


def known_cells_mask(points: np.ndarray, data: np.ndarray, origin_x: float, origin_y: float,
                     resolution: float, width: int, height: int) -> np.ndarray:
    """Máscara (N,) de los puntos N×≥2 que caen en celdas conocidas (≥ 0) del OccupancyGrid."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return np.zeros(0, dtype=bool)
    pts = pts.reshape(len(pts), -1)
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    cols = np.floor((pts[:, 0] - origin_x) / resolution).astype(int)
    rows = np.floor((pts[:, 1] - origin_y) / resolution).astype(int)
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    known = np.zeros(len(pts), dtype=bool)
    known[inside] = grid[rows[inside], cols[inside]] >= 0
    return known
