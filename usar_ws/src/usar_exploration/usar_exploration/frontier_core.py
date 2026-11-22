"""
Lógica de exploración por fronteras (Yamauchi, 1997) sin dependencias de ROS.

Trabaja sobre rejillas de ocupación en la convención de nav_msgs/OccupancyGrid: arreglo
(filas = y, columnas = x), valores -1 = desconocido, 0..100 = probabilidad de ocupación.
Solo recibe el mapa construido por el propio robot (/map) y su costmap: nunca el ground truth.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

UNKNOWN = -1
EIGHT_CONNECTIVITY = np.ones((3, 3), dtype=bool)


@dataclass
class GridInfo:
    """Geometría de una rejilla: resolución (m/celda) y origen (x, y) de la celda (0, 0)."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def cell_to_world(self, row: float, col: float) -> Tuple[float, float]:
        """Centro de la celda (row, col) en metros."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Celda (row, col) que contiene el punto, o None si queda fuera de la rejilla."""
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None


@dataclass
class Frontier:
    """Cluster de celdas frontera y la meta segura asociada."""

    cells: np.ndarray                     # N×2 (row, col)
    centroid: Tuple[float, float]         # en metros
    goal: Optional[Tuple[float, float]] = None
    info_gain: float = 0.0                # I(g): celdas desconocidas alrededor de la meta
    path_cost: float = float('nan')       # C(g): longitud de la ruta (m)
    score: float = float('nan')           # J(g)
    extra: dict = field(default_factory=dict)

    @property
    def size(self) -> int:
        """Número de celdas del cluster."""
        return len(self.cells)


def classify(grid: np.ndarray, free_threshold: int, occupied_threshold: int):
    """Devuelve las máscaras (libre, ocupado, desconocido) de la rejilla."""
    unknown = grid == UNKNOWN
    free = (grid >= 0) & (grid <= free_threshold)
    occupied = grid >= occupied_threshold
    return free, occupied, unknown


def frontier_mask(free: np.ndarray, unknown: np.ndarray) -> np.ndarray:
    """Celdas libres con al menos un vecino (4-conectividad) desconocido."""
    padded = np.pad(unknown, 1, constant_values=False)
    neighbour_unknown = (padded[:-2, 1:-1] | padded[2:, 1:-1]
                         | padded[1:-1, :-2] | padded[1:-1, 2:])
    return free & neighbour_unknown


def cluster_frontiers(mask: np.ndarray, info: GridInfo, min_size: int,
                      segment_size: Optional[float] = None) -> List[Frontier]:
    """
    Agrupa las celdas frontera en clusters 8-conexos y descarta los menores que min_size.

    El etiquetado de componentes conexas equivale a un recorrido BFS desde cada celda no visitada.
    Si se da segment_size (m), cada cluster se divide además en tramos según una rejilla de ese
    lado: una frontera larga o en anillo (p. ej. el borde del primer escaneo alrededor del robot)
    tiene un centroide que no representa a ninguna de sus partes; sus tramos sí.
    """
    labels, count = ndimage.label(mask, structure=EIGHT_CONNECTIVITY)
    frontiers = []
    for label in range(1, count + 1):
        cells = np.argwhere(labels == label)
        if segment_size:
            tile = max(int(round(segment_size / info.resolution)), 1)
            keys = cells // tile
            _, group = np.unique(keys, axis=0, return_inverse=True)
            parts = [cells[group.ravel() == k] for k in range(group.max() + 1)]
        else:
            parts = [cells]
        for part in parts:
            if len(part) < min_size:
                continue
            mean_row, mean_col = part.mean(axis=0)
            frontiers.append(Frontier(cells=part, centroid=info.cell_to_world(mean_row, mean_col)))
    return frontiers


def representative_cell(frontier: Frontier, info: GridInfo) -> Tuple[int, int]:
    """Celda del cluster más cercana a su centroide (el centroide puede caer fuera del cluster)."""
    target = info.world_to_cell(*frontier.centroid)
    if target is None:
        target = tuple(frontier.cells[0])
    d2 = ((frontier.cells - np.array(target)) ** 2).sum(axis=1)
    row, col = frontier.cells[int(np.argmin(d2))]
    return int(row), int(col)


def disk_kernel(radius_cells: int) -> np.ndarray:
    """Máscara circular de radio radius_cells (en celdas)."""
    r = max(int(radius_cells), 0)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def information_map(unknown: np.ndarray, radius_cells: int) -> np.ndarray:
    """Para cada celda, número de celdas desconocidas a menos de radius_cells (I(g))."""
    kernel = disk_kernel(radius_cells).astype(np.float32)
    return ndimage.convolve(unknown.astype(np.float32), kernel, mode='constant', cval=0.0)


def clearance_map(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Distancia (m) desde cada celda al obstáculo más cercano del mapa."""
    if not occupied.any():
        return np.full(occupied.shape, np.inf)
    return ndimage.distance_transform_edt(~occupied) * resolution


def find_safe_goal(cell: Tuple[int, int], free: np.ndarray, clearance: np.ndarray,
                   info: GridInfo, search_radius: float, min_clearance: float,
                   cost_ok: Optional[np.ndarray] = None) -> Optional[Tuple[int, int]]:
    """
    Celda libre más cercana a `cell` con holgura suficiente respecto a los obstáculos.

    Busca dentro de search_radius (m). Una celda es válida si es libre en el mapa, está a más de
    min_clearance (m) de cualquier obstáculo del mapa y, si se proporciona cost_ok (máscara
    alineada con el mapa), si el costmap la considera de costo aceptable.
    """
    r = int(np.ceil(search_radius / info.resolution))
    row0, col0 = cell
    r_lo, r_hi = max(row0 - r, 0), min(row0 + r + 1, info.height)
    c_lo, c_hi = max(col0 - r, 0), min(col0 + r + 1, info.width)
    valid = free[r_lo:r_hi, c_lo:c_hi] & (clearance[r_lo:r_hi, c_lo:c_hi] >= min_clearance)
    if cost_ok is not None:
        valid &= cost_ok[r_lo:r_hi, c_lo:c_hi]
    rows, cols = np.nonzero(valid)
    if len(rows) == 0:
        return None
    d2 = (rows + r_lo - row0) ** 2 + (cols + c_lo - col0) ** 2
    inside = d2 <= r * r
    if not inside.any():
        return None
    k = int(np.argmin(np.where(inside, d2, np.iinfo(np.int64).max)))
    return int(rows[k] + r_lo), int(cols[k] + c_lo)


def line_of_sight(occupied: np.ndarray, info: GridInfo, start: Tuple[float, float],
                  end: Tuple[float, float], ignore_end: float = 0.0) -> bool:
    """
    Cierto si el segmento start–end (m) no cruza celdas ocupadas del mapa.

    Los últimos ignore_end m antes de `end` no se comprueban: el objetivo puede estar junto a
    (o contra) un escombro que el mapa marca como ocupado.
    """
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    usable = length - ignore_end
    if usable <= 0.0:
        return True
    n = max(2, int(np.ceil(usable / (0.5 * info.resolution))) + 1)
    t = np.linspace(0.0, usable / max(length, 1e-9), n)
    xs = start[0] + t * (end[0] - start[0])
    ys = start[1] + t * (end[1] - start[1])
    cols = np.floor((xs - info.origin_x) / info.resolution).astype(int)
    rows = np.floor((ys - info.origin_y) / info.resolution).astype(int)
    inside = (rows >= 0) & (rows < info.height) & (cols >= 0) & (cols < info.width)
    return not bool(occupied[rows[inside], cols[inside]].any())


def find_view_goal(target: Tuple[float, float], robot: Tuple[float, float], free: np.ndarray,
                   occupied: np.ndarray, clearance: np.ndarray, info: GridInfo,
                   min_distance: float, max_distance: float, min_clearance: float,
                   cost_ok: Optional[np.ndarray] = None, ignore_end: float = 0.3,
                   max_checks: int = 400) -> Optional[Tuple[float, float, float]]:
    """
    Punto desde el que mirar un objetivo (p. ej. un candidato a víctima oído por el sonido).

    Busca celdas libres, con holgura, a entre min_distance y max_distance m del objetivo; elige
    la más cercana al robot que tenga línea de vista hasta el objetivo (si ninguna la tiene, la
    más cercana al robot). Devuelve (x, y, yaw) con yaw mirando al objetivo, o None.
    """
    tcell = info.world_to_cell(*target)
    r = int(np.ceil(max_distance / info.resolution))
    if tcell is None:
        return None
    row0, col0 = tcell
    r_lo, r_hi = max(row0 - r, 0), min(row0 + r + 1, info.height)
    c_lo, c_hi = max(col0 - r, 0), min(col0 + r + 1, info.width)
    valid = free[r_lo:r_hi, c_lo:c_hi] & (clearance[r_lo:r_hi, c_lo:c_hi] >= min_clearance)
    if cost_ok is not None:
        valid &= cost_ok[r_lo:r_hi, c_lo:c_hi]
    rows, cols = np.nonzero(valid)
    if len(rows) == 0:
        return None
    rows, cols = rows + r_lo, cols + c_lo
    xs = info.origin_x + (cols + 0.5) * info.resolution
    ys = info.origin_y + (rows + 0.5) * info.resolution
    d_target = np.hypot(xs - target[0], ys - target[1])
    ring = (d_target >= min_distance) & (d_target <= max_distance)
    if not ring.any():
        return None
    xs, ys = xs[ring], ys[ring]
    order = np.argsort(np.hypot(xs - robot[0], ys - robot[1]))
    best = int(order[0])
    for k in order[:max_checks]:
        if line_of_sight(occupied, info, (xs[k], ys[k]), target, ignore_end):
            best = int(k)
            break
    x, y = float(xs[best]), float(ys[best])
    return x, y, math.atan2(target[1] - y, target[0] - x)


def is_blacklisted(point: Tuple[float, float], blacklist: Sequence[Tuple[float, float]],
                   radius: float) -> bool:
    """Cierto si el punto está a menos de radius (m) de alguna meta de la lista negra."""
    return any(np.hypot(point[0] - bx, point[1] - by) < radius for bx, by in blacklist)


def sign_evidence(point: Tuple[float, float], signs: Sequence[Tuple[float, float]],
                  sigma: float) -> float:
    """S(g): suma de gaussianas (σ m) centradas en los candidatos a víctima (signos de vida)."""
    return float(sum(math.exp(-((point[0] - sx) ** 2 + (point[1] - sy) ** 2)
                              / (2.0 * sigma * sigma)) for sx, sy in signs))


def score_frontiers(frontiers: Sequence[Frontier], alpha: float, beta: float,
                    gamma: float = 0.0, signs: Sequence[Tuple[float, float]] = (),
                    sign_sigma: float = 2.0) -> None:
    """
    Calcula J(g) = α·C(g) − β·I(g) − γ·S(g), con C, I y S normalizados a [0, 1] entre candidatos.

    S(g) es la evidencia de signos de vida cerca de la meta (candidatos a víctima de los
    detectores guía, `signs`): con γ > 0 el robot prefiere las fronteras junto a ellos. Con
    γ = 0 (por defecto) J es la de Yamauchi. Modifica `score` (y `extra['sign']`) de cada
    frontera; las de C(g) no finito reciben J = +inf.
    """
    costs = np.array([f.path_cost for f in frontiers], dtype=float)
    gains = np.array([f.info_gain for f in frontiers], dtype=float)
    evidence = np.array([sign_evidence(f.goal or f.centroid, signs, sign_sigma)
                         if gamma > 0 and signs
                         else 0.0 for f in frontiers], dtype=float)
    finite = np.isfinite(costs)
    c_max = costs[finite].max() if finite.any() else 1.0
    g_max = gains.max() if len(gains) and gains.max() > 0 else 1.0
    s_max = evidence.max() if len(evidence) and evidence.max() > 0 else 1.0
    for f, c, g, s, ok in zip(frontiers, costs, gains, evidence, finite):
        f.extra['sign'] = float(s / s_max)
        f.score = (alpha * c / max(c_max, 1e-9) - beta * g / g_max - gamma * s / s_max) \
            if ok else float('inf')


def best_frontier(frontiers: Sequence[Frontier]) -> Optional[Frontier]:
    """Frontera con menor J(g) finito, o None."""
    finite = [f for f in frontiers if np.isfinite(f.score)]
    return min(finite, key=lambda f: f.score) if finite else None


def path_length(points: Sequence[Tuple[float, float]]) -> float:
    """Longitud (m) de una polilínea."""
    if len(points) < 2:
        return 0.0
    p = np.asarray(points, dtype=float)
    return float(np.hypot(*np.diff(p, axis=0).T).sum())


def align_to(grid_info: GridInfo, other: np.ndarray, other_info: GridInfo,
             fill: int) -> np.ndarray:
    """
    Remuestrea `other` (p. ej. el costmap global) sobre la rejilla `grid_info` (el /map).

    Ambas rejillas deben tener la misma resolución; las celdas sin correspondencia reciben fill.
    """
    out = np.full((grid_info.height, grid_info.width), fill, dtype=other.dtype)
    if abs(grid_info.resolution - other_info.resolution) > 1e-6:
        return out
    d_col = int(round((other_info.origin_x - grid_info.origin_x) / grid_info.resolution))
    d_row = int(round((other_info.origin_y - grid_info.origin_y) / grid_info.resolution))
    r0, c0 = max(d_row, 0), max(d_col, 0)
    r1 = min(d_row + other_info.height, grid_info.height)
    c1 = min(d_col + other_info.width, grid_info.width)
    if r1 > r0 and c1 > c0:
        out[r0:r1, c0:c1] = other[r0 - d_row:r1 - d_row, c0 - d_col:c1 - d_col]
    return out


def cost_at(cost: np.ndarray, info: GridInfo, x: float, y: float, outside: int = 100) -> int:
    """Costo del costmap en el punto (x, y); `outside` si queda fuera de la rejilla."""
    cell = info.world_to_cell(x, y)
    return int(cost[cell]) if cell is not None else outside


def escape_direction(cost: np.ndarray, info: GridInfo, x: float, y: float, yaw: float,
                     probe_distances: Sequence[float], n_directions: int = 12,
                     unknown_cost: int = 50, blocked_cost: int = 99) -> Tuple[float, bool]:
    """
    Dirección para salir de la zona inscrita del costmap (centro del robot cerca de un obstáculo).

    Evalúa n_directions rumbos alrededor del robot; el costo de cada uno es el máximo del costmap
    en los puntos situados a probe_distances (m) en esa dirección (desconocido = unknown_cost).
    Devuelve (rumbo absoluto, avanzar): avanzar es True si la dirección queda en el semiplano
    delantero (el robot avanza hacia ella) y False si queda detrás (retrocede, sin girar 180°).

    Preferencias: primero las direcciones libres (costo < blocked_cost) que no exigen girar (de
    frente o de espaldas), porque girar cerca de un obstáculo barre las esquinas de la carcasa;
    luego el menor costo y, a igualdad, la dirección más alineada con el frente (cámara).
    """
    best = None
    for k in range(n_directions):
        rel = -math.pi + 2 * math.pi * k / n_directions
        heading = yaw + rel
        samples = [cost_at(cost, info, x + d * math.cos(heading), y + d * math.sin(heading))
                   for d in probe_distances]
        score = max(unknown_cost if c < 0 else c for c in samples)
        straight = min(abs(rel), math.pi - abs(rel)) < 1e-6
        key = (score >= blocked_cost, not straight, score, abs(rel))
        if best is None or key < best[0]:
            best = (key, heading, abs(rel) <= math.pi / 2)
    return math.atan2(math.sin(best[1]), math.cos(best[1])), best[2]


def straight_escape_forward(cost: np.ndarray, info: GridInfo, x: float, y: float, yaw: float,
                            probe_distances: Sequence[float], unknown_cost: int = 50) -> bool:
    """
    Sentido (True = adelante) de un escape en línea recta, sin girar.

    Compara el costo máximo del costmap en los puntos a probe_distances por delante y por detrás
    del robot y elige el menor (a igualdad, adelante: ahí mira la cámara).
    """
    def score(sign):
        samples = [cost_at(cost, info, x + sign * d * math.cos(yaw), y + sign * d * math.sin(yaw))
                   for d in probe_distances]
        return max(unknown_cost if c < 0 else c for c in samples)
    return score(1.0) <= score(-1.0)


# ---------------------------------------------------------------------- retroceso por la huella
class Breadcrumbs:
    """
    Huella del recorrido del robot: puntos (x, y) separados al menos `spacing` m.

    Si el robot queda encerrado en un hueco en el que el costmap ya no deja salida (al acercarse
    se marcaron obstáculos que cierran la entrada por la que llegó), su propio recorrido es un
    camino que ya cupo físicamente: deshacerlo es la forma más segura de salir.
    """

    def __init__(self, spacing: float = 0.1, max_points: int = 400):
        self.spacing = float(spacing)
        self.max_points = int(max_points)
        self.points: List[Tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        """Añade la posición si se alejó `spacing` del último punto."""
        if not self.points or math.hypot(x - self.points[-1][0],
                                         y - self.points[-1][1]) >= self.spacing:
            self.points.append((float(x), float(y)))
            if len(self.points) > self.max_points:
                del self.points[0]

    def path_back(self, x: float, y: float, distance: float, min_exit: float = 0.0
                  ) -> List[Tuple[float, float]]:
        """
        Camino de vuelta desde (x, y) por la huella, de al menos `distance` m de recorrido.

        Empieza en el punto de la huella más cercano al robot y retrocede en el tiempo; termina
        cuando ha recorrido `distance` y el punto está a más de `min_exit` m de (x, y) en línea
        recta (fuera del hueco). Vacío si no hay huella.
        """
        if not self.points:
            return []
        pts = np.asarray(self.points)
        start = int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y)))
        path, travelled = [], 0.0
        prev = (x, y)
        for i in range(start, -1, -1):
            p = self.points[i]
            travelled += math.hypot(p[0] - prev[0], p[1] - prev[1])
            path.append(p)
            prev = p
            if travelled >= distance and math.hypot(p[0] - x, p[1] - y) >= min_exit:
                break
        return path


def follow_path_command(pose: Tuple[float, float, float], path: Sequence[Tuple[float, float]],
                        lookahead: float, speed: float, turn_speed: float,
                        turn_in_place: float = 0.7) -> Tuple[float, float, int]:
    """
    Orden (v, ω) para seguir un camino con persecución pura, hacia delante o marcha atrás.

    Avanza hacia el primer punto del camino a más de `lookahead`; si ese punto queda detrás del
    robot (a más de 90°), va marcha atrás (no hace falta girar en un hueco estrecho). Si el error
    de rumbo supera `turn_in_place`, solo gira. Devuelve también el índice del punto del camino
    más cercano al robot (para descartar lo ya recorrido).
    """
    x, y, yaw = pose
    pts = np.asarray(path, dtype=float)
    dists = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
    nearest = int(np.argmin(dists))
    ahead = np.nonzero(dists[nearest:] >= lookahead)[0]
    target = pts[nearest + ahead[0]] if len(ahead) else pts[-1]
    bearing = math.atan2(target[1] - y, target[0] - x)
    err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    direction = 1.0
    if abs(err) > math.pi / 2:                       # objetivo detrás: marcha atrás
        direction = -1.0
        err = math.atan2(math.sin(err - math.pi), math.cos(err - math.pi))
    omega = max(-turn_speed, min(turn_speed, 2.0 * err))
    if abs(err) > turn_in_place:
        return 0.0, omega, nearest
    return direction * speed * max(0.3, math.cos(err)), omega, nearest
