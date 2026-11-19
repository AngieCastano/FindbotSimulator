"""
Mapa de evidencia de víctimas en el frame del mapa: acumulación, agrupación y confirmación.

Cada punto 3D detectado (p. ej. cada píxel naranja con profundidad válida), llevado al frame `map`
y proyectado al suelo, suma evidencia en una rejilla de `cell_size`. Para cada celda se cuenta en
cuántas capturas distintas se observó. Periódicamente:

  1. se toman las celdas observadas en al menos `min_cell_frames` capturas (filtra ruido),
  2. se agrupan las que están a menos de `join_distance` (partes de una misma víctima: cabeza,
     torso, extremidades, vistas desde distintos puntos),
  3. un grupo observado en al menos `min_observations` capturas distintas es una víctima
     confirmada; su posición es el centroide del grupo ponderado por la evidencia.

A diferencia de asociar un único punto por detección, el centroide de la evidencia tiende al
centro del cuerpo a medida que se observa desde varios ángulos, y las partes de una víctima vistas
por separado no generan víctimas duplicadas. Los números de víctima se mantienen entre
actualizaciones asociando cada grupo al número confirmado más cercano (a menos de
`match_distance`).
"""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np
from scipy import ndimage


@dataclass
class Track:
    """Grupo de evidencia (víctima confirmada o tentativa)."""

    track_id: int
    position: Tuple[float, float, float]
    observations: int                    # capturas distintas que aportaron evidencia
    first_stamp: float
    last_stamp: float
    cells: int = 0
    victim_id: Optional[int] = None

    @property
    def confirmed(self) -> bool:
        """Cierto si el grupo es una víctima confirmada."""
        return self.victim_id is not None

    def confidence(self, min_observations: int) -> float:
        """Confianza 0..1: 0.5 al confirmarse y 1.0 al duplicar las observaciones necesarias."""
        return min(1.0, self.observations / (2.0 * min_observations))


@dataclass
class _Cell:
    hits: int
    frames: Set[int]
    sum_z: float
    first_stamp: float
    last_stamp: float


class VictimTracker:
    """Acumula evidencia en una rejilla del mapa y confirma víctimas."""

    def __init__(self, min_observations: int, cell_size: float = 0.1,
                 join_distance: float = 0.2, min_cell_frames: int = 2,
                 match_distance: float = 1.0):
        self.min_observations = int(min_observations)
        self.cell_size = float(cell_size)
        self.join_distance = float(join_distance)
        self.min_cell_frames = int(min_cell_frames)
        self.match_distance = float(match_distance)
        self._cells: Dict[Tuple[int, int], _Cell] = {}
        self._frame = 0
        self._next_victim = 1
        self._victims: Dict[int, Tuple[float, float]] = {}   # número → última posición
        self.tracks: List[Track] = []

    def update(self, points: Iterable[Tuple[float, float, float]], stamp: float) -> List[Track]:
        """Incorpora los puntos de una captura; devuelve las víctimas recién confirmadas."""
        self._frame += 1
        pts = np.asarray(list(points), dtype=float).reshape(-1, 3)
        if len(pts):
            keys = np.floor(pts[:, :2] / self.cell_size).astype(int)
            uniq, inverse, counts = np.unique(keys, axis=0, return_inverse=True,
                                              return_counts=True)
            sum_z = np.bincount(inverse.ravel(), weights=pts[:, 2])
            for (i, j), n, zs in zip(map(tuple, uniq), counts, sum_z):
                cell = self._cells.get((i, j))
                if cell is None:
                    self._cells[(i, j)] = _Cell(int(n), {self._frame}, float(zs), stamp, stamp)
                else:
                    cell.hits += int(n)
                    cell.frames.add(self._frame)
                    cell.sum_z += float(zs)
                    cell.last_stamp = stamp
        return self._rebuild()

    def _rebuild(self) -> List[Track]:
        """Agrupa las celdas con evidencia y actualiza las víctimas confirmadas."""
        good = [(k, c) for k, c in self._cells.items() if len(c.frames) >= self.min_cell_frames]
        self.tracks = []
        if not good:
            return []
        keys = np.array([k for k, _ in good])
        origin = keys.min(axis=0)
        idx = keys - origin
        grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
        grid[idx[:, 0], idx[:, 1]] = True
        # Cada grupo se dilata la mitad de join_distance: dos celdas quedan unidas si las separa
        # menos de join_distance.
        radius = max(int(round(self.join_distance / (2 * self.cell_size))), 1)
        yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
        joined = ndimage.binary_dilation(grid, structure=(xx * xx + yy * yy) <= radius * radius)
        labels, count = ndimage.label(joined)
        cell_labels = labels[idx[:, 0], idx[:, 1]]

        groups = []
        for label in range(1, count + 1):
            members = [good[n][1] for n in np.nonzero(cell_labels == label)[0]]
            member_keys = keys[cell_labels == label]
            weights = np.array([c.hits for c in members], dtype=float)
            centers = (member_keys + 0.5) * self.cell_size
            x, y = (centers * weights[:, None]).sum(axis=0) / weights.sum()
            z = sum(c.sum_z for c in members) / weights.sum()
            frames = set().union(*(c.frames for c in members))
            groups.append(Track(
                track_id=label, position=(float(x), float(y), float(z)),
                observations=len(frames),
                first_stamp=min(c.first_stamp for c in members),
                last_stamp=max(c.last_stamp for c in members), cells=len(members)))

        newly = self._assign_ids(groups)
        self.tracks = groups
        return newly

    def _assign_ids(self, groups: List[Track]) -> List[Track]:
        """Conserva los números de víctima ya asignados; numera las nuevas confirmadas."""
        newly = []
        free = dict(self._victims)
        # Primero los grupos con más evidencia, para que conserven su número en caso de duda.
        for group in sorted(groups, key=lambda g: -g.observations):
            if group.observations < self.min_observations:
                continue
            best, best_d = None, self.match_distance
            for vid, (vx, vy) in free.items():
                d = math.hypot(group.position[0] - vx, group.position[1] - vy)
                if d < best_d:
                    best, best_d = vid, d
            if best is not None:
                group.victim_id = best
                del free[best]
            else:
                group.victim_id = self._next_victim
                self._next_victim += 1
                newly.append(group)
            self._victims[group.victim_id] = group.position[:2]
        return newly

    def confirmed(self) -> List[Track]:
        """Víctimas confirmadas, ordenadas por número."""
        return sorted((t for t in self.tracks if t.confirmed), key=lambda t: t.victim_id)

    def tentative(self) -> List[Track]:
        """Grupos aún sin confirmar."""
        return [t for t in self.tracks if not t.confirmed]
