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


def ray_extents(data: np.ndarray, origin_x: float, origin_y: float, resolution: float,
                width: int, height: int, start: np.ndarray, directions: np.ndarray,
                max_ranges: np.ndarray, min_range: float = 0.0, beyond: float = 0.0,
                occupied_threshold: int = 65) -> np.ndarray:
    """
    Hasta dónde puede estar, a lo largo de cada rayo, lo que ve un sensor sin distancia.

    El rayo recorre el /map del robot desde min_range y termina en lo que llegue antes:
      - la primera celda ocupada, más `beyond` m (el objeto visto tiene cierto fondo),
      - la primera celda desconocida o el borde del mapa (detrás no se sabe qué hay),
      - su distancia máxima (max_ranges).
    Devuelve la longitud (m) de cada rayo; ≤ min_range si no recorre nada.

    start: (x, y) del sensor; directions: N×2 (se normalizan); max_ranges: N (m).
    """
    dirs = np.asarray(directions, dtype=float).reshape(-1, 2)
    if len(dirs) == 0:
        return np.zeros(0)
    norms = np.linalg.norm(dirs, axis=1)
    dirs = dirs / np.where(norms > 0, norms, 1.0)[:, None]
    max_ranges = np.broadcast_to(np.asarray(max_ranges, dtype=float), (len(dirs),))
    limit = float(np.max(np.where(np.isfinite(max_ranges), max_ranges, 0.0), initial=0.0))
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    step = resolution / 2.0
    ts = np.arange(min_range, limit + step, step)                  # (S,)
    if len(ts) == 0:
        return np.full(len(dirs), min_range)
    xs = start[0] + dirs[:, :1] * ts[None, :]                       # (N, S)
    ys = start[1] + dirs[:, 1:2] * ts[None, :]
    cols = np.floor((xs - origin_x) / resolution).astype(int)
    rows = np.floor((ys - origin_y) / resolution).astype(int)
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    values = np.full(cols.shape, -1, dtype=np.int16)               # fuera del mapa = desconocido
    values[inside] = grid[rows[inside], cols[inside]]
    big = np.inf
    occ = values >= occupied_threshold
    unknown = values < 0
    t_occ = np.where(occ.any(axis=1), ts[occ.argmax(axis=1)] + beyond, big)
    t_unk = np.where(unknown.any(axis=1), ts[unknown.argmax(axis=1)], big)
    return np.minimum(np.minimum(t_occ, t_unk), max_ranges)
