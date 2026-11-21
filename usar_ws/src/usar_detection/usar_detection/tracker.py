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

Opcionalmente se exige una geometría (filtro de forma, Fase 8): largo, ancho, altura máxima y
perfil de alturas a lo largo del grupo (relación entre el tercio más alto y el más bajo). Un
grupo que no la cumple sigue como tentativo aunque tenga observaciones suficientes.

Una víctima ya confirmada no se retira solo porque su grupo crezca por encima del largo o ancho
máximos: ocurre cuando la evidencia de dos víctimas cercanas (o de una víctima y un escombro bajo
vecino) acaba uniéndose. En ese caso se mantiene en su última posición. Sí se retira si el grupo
deja de cumplir la altura o el perfil (la evidencia contradice la forma humana).

A diferencia de asociar un único punto por detección, el centroide de la evidencia tiende al
centro del cuerpo a medida que se observa desde varios ángulos, y las partes de una víctima vistas
por separado no generan víctimas duplicadas. Los números de víctima se mantienen entre
actualizaciones asociando cada grupo al número confirmado más cercano (a menos de
`match_distance`).
"""

from dataclasses import dataclass, field
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
    length: float = 0.0                  # m, a lo largo del eje principal del grupo
    width: float = 0.0                   # m, perpendicular al eje principal
    height: float = 0.0                  # m, altura máxima sobre el suelo
    profile_ratio: float = 1.0           # altura del tercio más alto / del más bajo
    shape_ok: bool = True
    #: Solo falla el largo o el ancho máximos (p. ej. dos víctimas cercanas unidas).
    oversize: bool = False
    #: Centros de las celdas del grupo (uso interno del tracker).
    cell_centers: Optional[np.ndarray] = field(default=None, repr=False, compare=False)

    @property
    def confirmed(self) -> bool:
        """Cierto si el grupo es una víctima confirmada."""
        return self.victim_id is not None

    def confidence(self, min_observations: int) -> float:
        """Confianza 0..1: 0.5 al confirmarse y 1.0 al duplicar las observaciones necesarias."""
        return min(1.0, self.observations / (2.0 * min_observations))


#: Claves del filtro de forma (todas opcionales).
SHAPE_KEYS = ('min_length', 'max_length', 'min_width', 'max_width', 'min_height', 'max_height',
              'min_profile_ratio')


def shape_matches(track: 'Track', shape: Optional[Dict[str, float]]) -> bool:
    """Cierto si la geometría del grupo cumple el filtro (sin filtro, siempre)."""
    if not shape:
        return True
    checks = (
        ('min_length', track.length, 1), ('max_length', track.length, -1),
        ('min_width', track.width, 1), ('max_width', track.width, -1),
        ('min_height', track.height, 1), ('max_height', track.height, -1),
        ('min_profile_ratio', track.profile_ratio, 1),
    )
    for key, value, sign in checks:
        if key in shape and sign * (value - float(shape[key])) < 0:
            return False
    return True


@dataclass
class _Cell:
    hits: int
    frames: Set[int]
    sum_z: float
    max_z: float
    first_stamp: float
    last_stamp: float


class VictimTracker:
    """Acumula evidencia en una rejilla del mapa y confirma víctimas."""

    def __init__(self, min_observations: int, cell_size: float = 0.1,
                 join_distance: float = 0.2, min_cell_frames: int = 2,
                 match_distance: float = 1.0, shape: Optional[Dict[str, float]] = None):
        self.min_observations = int(min_observations)
        self.cell_size = float(cell_size)
        self.join_distance = float(join_distance)
        self.min_cell_frames = int(min_cell_frames)
        self.match_distance = float(match_distance)
        #: Filtro de forma (claves de SHAPE_KEYS) o None para no exigir geometría.
        self.shape = dict(shape) if shape else None
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
            inverse = inverse.ravel()
            sum_z = np.bincount(inverse, weights=pts[:, 2])
            max_z = np.full(len(uniq), -np.inf)
            np.maximum.at(max_z, inverse, pts[:, 2])
            for (i, j), n, zs, zmax in zip(map(tuple, uniq), counts, sum_z, max_z):
                cell = self._cells.get((i, j))
                if cell is None:
                    self._cells[(i, j)] = _Cell(int(n), {self._frame}, float(zs), float(zmax),
                                                stamp, stamp)
                else:
                    cell.hits += int(n)
                    cell.frames.add(self._frame)
                    cell.sum_z += float(zs)
                    cell.max_z = max(cell.max_z, float(zmax))
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
            track = Track(
                track_id=label, position=(float(x), float(y), float(z)),
                observations=len(frames),
                first_stamp=min(c.first_stamp for c in members),
                last_stamp=max(c.last_stamp for c in members), cells=len(members))
            self._measure(track, centers, weights, np.array([c.max_z for c in members]))
            groups.append(track)

        newly = self._assign_ids(groups)
        self.tracks = groups
        return newly

    def _assign_ids(self, groups: List[Track]) -> List[Track]:
        """Conserva los números de víctima ya asignados; numera las nuevas confirmadas."""
        newly = []
        free = dict(self._victims)
        held = []
        # Primero los grupos con más evidencia, para que conserven su número en caso de duda.
        for group in sorted(groups, key=lambda g: -g.observations):
            if group.observations < self.min_observations or not group.shape_ok:
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
        # Víctimas confirmadas cuyo grupo solo ha crecido demasiado: se mantienen.
        for group in groups:
            if not group.oversize or group.observations < self.min_observations:
                continue
            for vid, (vx, vy) in list(free.items()):
                d = np.hypot(group.cell_centers[:, 0] - vx, group.cell_centers[:, 1] - vy)
                if d.min() <= 2 * self.cell_size:
                    kept = Track(track_id=group.track_id, position=(vx, vy, group.position[2]),
                                 observations=group.observations,
                                 first_stamp=group.first_stamp, last_stamp=group.last_stamp,
                                 cells=group.cells, victim_id=vid, length=group.length,
                                 width=group.width, height=group.height,
                                 profile_ratio=group.profile_ratio, shape_ok=False,
                                 oversize=True)
                    held.append(kept)
                    del free[vid]
        groups.extend(held)
        return newly

    def _measure(self, track: Track, centers: np.ndarray, weights: np.ndarray,
                 heights: np.ndarray) -> None:
        """Largo, ancho, altura y perfil de alturas del grupo; aplica el filtro de forma."""
        track.height = float(heights.max())
        if len(centers) >= 2:
            mean = np.average(centers, axis=0, weights=weights)
            cov = np.cov((centers - mean).T, aweights=weights) if weights.sum() > 0 else None
            if cov is not None and np.all(np.isfinite(cov)):
                _, vecs = np.linalg.eigh(cov)
                axis = vecs[:, -1]                       # eje principal (mayor varianza)
            else:
                axis = np.array([1.0, 0.0])
            along = (centers - mean) @ axis
            across = (centers - mean) @ np.array([-axis[1], axis[0]])
            track.length = float(np.ptp(along) + self.cell_size)
            track.width = float(np.ptp(across) + self.cell_size)
            # Perfil: altura máxima en cada tercio del largo (silueta lateral del cuerpo).
            edges = np.linspace(along.min(), along.max() + 1e-9, 4)
            thirds = [heights[(along >= a) & (along < b)] for a, b in zip(edges[:-1], edges[1:])]
            tops = [t.max() for t in thirds if len(t)]
            if len(tops) == 3 and min(tops) > 0:
                track.profile_ratio = float(max(tops) / min(tops))
        else:
            track.length = track.width = self.cell_size
        track.cell_centers = centers
        track.shape_ok = shape_matches(track, self.shape)
        if not track.shape_ok and self.shape:
            relaxed = {k: v for k, v in self.shape.items() if k not in ('max_length', 'max_width')}
            track.oversize = shape_matches(track, relaxed)

    def confirmed(self) -> List[Track]:
        """Víctimas confirmadas, ordenadas por número."""
        return sorted((t for t in self.tracks if t.confirmed), key=lambda t: t.victim_id)

    def tentative(self) -> List[Track]:
        """Grupos aún sin confirmar."""
        return [t for t in self.tracks if not t.confirmed]


@dataclass
class FusedVictim:
    """Víctima publicada: une las confirmaciones de varios detectores en el mismo lugar."""

    victim_id: int
    position: Tuple[float, float, float]
    sources: List[str]                   # detectores que la confirmaron
    observations: int                    # máximo de capturas entre los detectores
    confidence: float                    # máximo entre los detectores


class VictimRegistry:
    """
    Une las víctimas confirmadas por cada detector y mantiene una numeración común.

    Dos confirmaciones de detectores distintos a menos de merge_distance son la misma víctima.
    La posición es la media ponderada por la precisión de cada detector (1/σ², `sigmas`): el
    CO₂ ubica con un error de ≈ 1 m y la forma con ≈ 0.3 m; el número de observaciones no
    cuenta (cada sensor tiene su propia frecuencia). Dos confirmaciones del mismo detector nunca
    se unen: si la forma ve dos víctimas cercanas, una estimación de CO₂ entre ambas se asocia a
    la más cercana en vez de juntarlas. La fusión completa de evidencias de signos de vida es la
    Fase 13; aquí solo se evita publicar dos veces la misma víctima.

    Detectores **guía** (`cue_only`, p. ej. el sonido): solos no confirman una víctima. Un grupo
    formado solo por detectores guía queda como **candidato** (`self.candidates`: el explorador
    va a mirarlo); se vuelve víctima cuando un detector confirmador (forma, color, calor, CO₂) lo
    ve en el mismo sitio. Los candidatos no consumen números de víctima.
    """

    def __init__(self, merge_distance: float = 1.0, match_distance: float = 1.0):
        self.merge_distance = float(merge_distance)
        self.match_distance = float(match_distance)
        self._next = 1
        self._last: Dict[int, Tuple[float, float]] = {}   # número → última posición conocida
        self._confirmed_ids: Set[int] = set()
        self.victims: List[FusedVictim] = []
        self.candidates: List[FusedVictim] = []

    def update(self, confirmed: Dict[str, List[Track]], min_observations: int,
               sigmas: Optional[Dict[str, float]] = None,
               cue_only: Optional[Set[str]] = None
               ) -> Tuple[List[FusedVictim], List[int]]:
        """
        Recalcula las víctimas; devuelve (nuevas, números que dejaron de estar confirmados).

        sigmas: error de localización típico (m) de cada detector; por defecto 1 para todos.
        cue_only: detectores guía (sus grupos solos son candidatos, no víctimas).
        """
        sigmas = sigmas or {}
        cue_only = set(cue_only or ())
        # Primero los detectores más precisos (y, a igualdad, las víctimas con más evidencia).
        entries = sorted(((name, t) for name, tracks in confirmed.items() for t in tracks),
                         key=lambda e: (sigmas.get(e[0], 1.0), -e[1].observations))
        clusters: List[Dict] = []
        for name, track in entries:
            x, y, z = track.position
            options = [c for c in clusters if name not in c['sources']
                       and math.hypot(x - c['xy'][0], y - c['xy'][1]) < self.merge_distance]
            if options:
                cluster = min(options, key=lambda c: math.hypot(x - c['xy'][0],
                                                                y - c['xy'][1]))
            else:
                cluster = {'xy': (x, y), 'sum': np.zeros(3), 'w': 0.0, 'sources': set(),
                           'obs': 0, 'conf': 0.0}
                clusters.append(cluster)
            w = 1.0 / max(sigmas.get(name, 1.0), 1e-3) ** 2
            cluster['sum'] += w * np.array([x, y, z])
            cluster['w'] += w
            cluster['xy'] = tuple(cluster['sum'][:2] / cluster['w'])
            cluster['sources'].add(name)
            cluster['obs'] = max(cluster['obs'], track.observations)
            cluster['conf'] = max(cluster['conf'], track.confidence(min_observations))

        cues = [c for c in clusters if c['sources'] <= cue_only]
        self.candidates = [
            FusedVictim(i + 1, tuple(float(v) for v in c['sum'] / c['w']), sorted(c['sources']),
                        c['obs'], c['conf'])
            for i, c in enumerate(sorted(cues, key=lambda c: -c['obs']))]
        clusters = [c for c in clusters if not c['sources'] <= cue_only]

        free = dict(self._last)
        victims, newly = [], []
        for cluster in sorted(clusters, key=lambda c: -c['obs']):
            px, py = cluster['xy']
            best, best_d = None, self.match_distance
            for vid, (vx, vy) in free.items():
                d = math.hypot(px - vx, py - vy)
                if d < best_d:
                    best, best_d = vid, d
            if best is None:
                best = self._next
                self._next += 1
            else:
                del free[best]
            # Nueva: número nuevo, o un número que no estaba confirmado (víctima retirada que
            # vuelve a confirmarse).
            is_new = best not in self._confirmed_ids
            pos = cluster['sum'] / cluster['w']
            victim = FusedVictim(best, (float(pos[0]), float(pos[1]), float(pos[2])),
                                 sorted(cluster['sources']), cluster['obs'], cluster['conf'])
            victims.append(victim)
            if is_new:
                newly.append(victim)
        current = {v.victim_id for v in victims}
        self._last.update({v.victim_id: v.position[:2] for v in victims})
        self.victims = sorted(victims, key=lambda v: v.victim_id)
        retracted = sorted(self._confirmed_ids - current)
        self._confirmed_ids = current
        return newly, retracted
