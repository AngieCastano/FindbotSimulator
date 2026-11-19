"""
Cálculo de las métricas de la misión (funciones puras, sin ROS).

Este módulo SÍ usa el ground truth: solo lo importa el evaluador, nunca un nodo del robot.

Convenciones:
- Frame del mundo: el de Gazebo y del ground truth.
- Frame `map`: el de SLAM. slam_toolbox lo fija en la pose inicial del robot, de modo que un
  punto del mapa pasa al mundo rotándolo el yaw del spawn y sumándole su posición.
- OccupancyGrid: 0 = libre, 100 = ocupado, −1 = desconocido; fila = y, columna = x.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class Pose2D:
    """Pose plana (x, y, yaw)."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridGeometry:
    """Geometría de una rejilla de ocupación (origen sin rotación, como la del /map)."""

    origin_x: float
    origin_y: float
    resolution: float
    cols: int
    rows: int


def map_to_world(points: np.ndarray, spawn: Pose2D) -> np.ndarray:
    """Lleva puntos N×2 del frame `map` al frame del mundo."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(spawn.yaw), math.sin(spawn.yaw)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([spawn.x, spawn.y])


def world_to_map(points: np.ndarray, spawn: Pose2D) -> np.ndarray:
    """Lleva puntos N×2 del frame del mundo al frame `map` (inversa de map_to_world)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2) - np.array([spawn.x, spawn.y])
    c, s = math.cos(spawn.yaw), math.sin(spawn.yaw)
    rot = np.array([[c, s], [-s, c]])
    return pts @ rot.T


def explorable_centers(geometry: GridGeometry, mask: np.ndarray) -> np.ndarray:
    """Centros (N×2, frame del mundo) de las celdas del área explorable."""
    rows, cols = np.nonzero(mask)
    xs = geometry.origin_x + (cols + 0.5) * geometry.resolution
    ys = geometry.origin_y + (rows + 0.5) * geometry.resolution
    return np.column_stack([xs, ys])


def coverage(map_data: np.ndarray, map_geometry: GridGeometry, centers_world: np.ndarray,
             spawn: Pose2D, free_threshold: int = 25) -> Tuple[float, int]:
    """
    Cobertura C% = A_explorada / A_explorable × 100.

    Se cuentan las celdas del área explorable (sus centros en `centers_world`) cuya celda del
    /map es libre (0 ≤ valor ≤ free_threshold). Así el numerador nunca supera al denominador y
    las celdas libres detrás de los muros o dentro de escombros no inflan la cobertura.
    Devuelve (porcentaje, celdas explorables cubiertas).
    """
    total = len(centers_world)
    if total == 0:
        return 0.0, 0
    grid = np.asarray(map_data, dtype=np.int16).reshape(map_geometry.rows, map_geometry.cols)
    pts = world_to_map(centers_world, spawn)
    cols = np.floor((pts[:, 0] - map_geometry.origin_x) / map_geometry.resolution).astype(int)
    rows = np.floor((pts[:, 1] - map_geometry.origin_y) / map_geometry.resolution).astype(int)
    inside = (cols >= 0) & (cols < map_geometry.cols) & (rows >= 0) & (rows < map_geometry.rows)
    values = grid[rows[inside], cols[inside]]
    covered = int(np.count_nonzero((values >= 0) & (values <= free_threshold)))
    return 100.0 * covered / total, covered


class PathIntegrator:
    """Distancia recorrida integrando las posiciones sucesivas de la odometría."""

    def __init__(self, min_step: float = 0.0):
        #: Pasos menores que min_step se acumulan hasta superarlo (filtra el ruido en reposo).
        self.min_step = float(min_step)
        self.distance = 0.0
        self._last: Optional[Tuple[float, float]] = None

    def add(self, x: float, y: float) -> float:
        """Incorpora una posición; devuelve la distancia acumulada."""
        if self._last is None:
            self._last = (x, y)
            return self.distance
        step = math.hypot(x - self._last[0], y - self._last[1])
        if step >= self.min_step:
            self.distance += step
            self._last = (x, y)
        return self.distance


@dataclass
class VictimMatch:
    """Resultado de la asociación de una víctima real."""

    victim_id: int
    name: str
    true_xy: Tuple[float, float]
    estimated_xy: Optional[Tuple[float, float]] = None
    error: Optional[float] = None

    @property
    def detected(self) -> bool:
        """Cierto si alguna estimación quedó asociada a esta víctima."""
        return self.estimated_xy is not None


@dataclass
class VictimEvaluation:
    """Asociación de víctimas estimadas con las reales."""

    matches: List[VictimMatch]
    false_positives: List[Tuple[float, float]] = field(default_factory=list)
    threshold: float = 1.0

    @property
    def n_victims(self) -> int:
        """Número de víctimas reales."""
        return len(self.matches)

    @property
    def detected(self) -> int:
        """Víctimas reales asociadas a una estimación."""
        return sum(m.detected for m in self.matches)

    @property
    def detection_rate(self) -> float:
        """Tasa de detección (recall) en %."""
        return 100.0 * self.detected / self.n_victims if self.n_victims else 0.0

    @property
    def errors(self) -> List[float]:
        """Errores de localización de las víctimas detectadas (m)."""
        return [m.error for m in self.matches if m.error is not None]

    def summary(self) -> Dict:
        """Diccionario con las métricas de víctimas (para JSON)."""
        errors = self.errors
        estimated = self.detected + len(self.false_positives)
        return {
            'n_victims': self.n_victims,
            'n_estimated': estimated,
            'detected': self.detected,
            'detection_rate_pct': round(self.detection_rate, 2),
            'false_positives': len(self.false_positives),
            'precision_pct': round(100.0 * self.detected / estimated, 2) if estimated else None,
            'localization_error_mean_m': round(float(np.mean(errors)), 4) if errors else None,
            'localization_error_max_m': round(float(np.max(errors)), 4) if errors else None,
            'association_threshold_m': self.threshold,
        }


def associate_victims(estimates: Sequence[Tuple[float, float]], victims: Sequence[Dict],
                      threshold: float) -> VictimEvaluation:
    """
    Asocia víctimas estimadas (frame del mundo) con las reales del ground truth.

    Asociación uno a uno que minimiza la suma de distancias (algoritmo húngaro). Una pareja
    solo cuenta si su distancia es ≤ threshold; las estimaciones sin pareja son falsos positivos
    y las víctimas sin pareja, no detectadas.
    """
    matches = [VictimMatch(int(v['id']), str(v.get('name', f"victim_{v['id']}")),
                           (float(v['x']), float(v['y']))) for v in victims]
    est = np.asarray(estimates, dtype=float).reshape(-1, 2)
    used = set()
    if len(est) and matches:
        truth = np.array([m.true_xy for m in matches])
        dist = np.linalg.norm(est[:, None, :] - truth[None, :, :], axis=2)
        # Las parejas por encima del umbral se penalizan para que no desplacen a las válidas.
        cost = np.where(dist <= threshold, dist, 1e6)
        for i, j in zip(*linear_sum_assignment(cost)):
            if dist[i, j] <= threshold:
                matches[j].estimated_xy = (float(est[i, 0]), float(est[i, 1]))
                matches[j].error = float(dist[i, j])
                used.add(int(i))
    false_pos = [(float(p[0]), float(p[1])) for i, p in enumerate(est) if i not in used]
    return VictimEvaluation(matches, false_pos, float(threshold))


def occupancy_to_pgm(map_data: np.ndarray, rows: int, cols: int,
                     free_threshold: int = 25, occupied_threshold: int = 65) -> np.ndarray:
    """
    Imagen PGM (uint8, fila 0 arriba) en el formato trinario de nav2_map_server.

    Libre = 254, ocupado = 0, desconocido = 205.
    """
    grid = np.asarray(map_data, dtype=np.int16).reshape(rows, cols)
    img = np.full((rows, cols), 205, dtype=np.uint8)
    img[(grid >= 0) & (grid <= free_threshold)] = 254
    img[grid >= occupied_threshold] = 0
    return np.flipud(img)


def mean_std(values: Sequence[Optional[float]]) -> Tuple[Optional[float], Optional[float], int]:
    """Media, desviación estándar muestral (ddof=1) y n de los valores no nulos."""
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return None, None, 0
    std = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
    return float(np.mean(vals)), std, len(vals)
