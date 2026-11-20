"""
Detector de forma humana por geometría 3D (Fase 8), independiente del color.

Una víctima acostada es un objeto BAJO (≈ 0.2 m de alto) y alargado (≈ 1.7 × 0.6 m) cuya altura
cambia a lo largo del cuerpo (cabeza y torso más altos que las piernas). Por cada imagen de
profundidad el detector:
  1. calcula el punto 3D de los píxeles (submuestreados) y los lleva al frame de la base del
     robot (z = altura sobre el suelo),
  2. descarta el suelo (z ≤ floor_height),
  3. marca las columnas del suelo donde hay puntos por encima de max_victim_height (escombros
     altos) y descarta los puntos bajos de esas columnas: la base de un escombro alto no es una
     víctima,
  4. descarta los puntos cuya altura máxima visible no alcanza min_verifiable_height: muy cerca
     de la cámara (a 12 cm del suelo) no se ve si el objeto sigue hacia arriba,
  5. agrupa los puntos bajos restantes en el plano del suelo (rejilla de cluster_cell); cada grupo
     con suficientes puntos y extensión plausible es una detección.

La forma completa (largo, ancho, altura y perfil de alturas) se comprueba en el mapa de evidencia,
que acumula las vistas desde distintos ángulos (ver group_shape y tracker.SHAPE_KEYS).
"""

import math
from typing import Dict, List, Optional

import cv2
import numpy as np
from scipy import ndimage

from usar_detection.sign_of_life import (
    Detection, Observation, register_detector, SignOfLifeDetector)

DEFAULTS = {
    'stride': 2,                    # px: se procesa 1 de cada stride² píxeles
    'min_depth': 0.3,               # m
    'max_depth': 5.0,               # m: más lejos la profundidad y la altura son poco precisas
    'floor_height': 0.04,           # m: por debajo, suelo (ruido de profundidad incluido)
    'max_victim_height': 0.32,      # m: una víctima acostada no supera esta altura
    'tall_cell': 0.1,               # m: rejilla de columnas altas
    'tall_dilation': 1,             # celdas: margen alrededor de las columnas altas
    'min_verifiable_height': 0.45,  # m: altura hasta la que debe verse por encima del objeto
    'cluster_cell': 0.15,           # m: rejilla de agrupación en el plano del suelo
    'min_cluster_points': 40,       # puntos (tras el submuestreo)
    'max_cluster_extent': 2.4,      # m: un grupo mayor no es una víctima
    # Filtro de forma del grupo en el mapa de evidencia (tracker.SHAPE_KEYS).
    'shape_min_length': 1.2,
    'shape_max_length': 2.4,
    'shape_max_width': 1.1,
    'shape_min_height': 0.15,
    'shape_max_height': 0.30,
    'shape_min_profile_ratio': 1.25,
}


@register_detector
class HumanShapeDetector(SignOfLifeDetector):
    """Objetos bajos y alargados (silueta de un cuerpo acostado) en la nube de profundidad."""

    name = 'human_shape'

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **(params or {})})
        self._mask: Optional[np.ndarray] = None

    def group_shape(self) -> Optional[Dict[str, float]]:
        """Geometría exigida a un grupo de evidencia para confirmarlo como víctima."""
        p = self.params
        return {key: float(p['shape_' + key]) for key in (
            'min_length', 'max_length', 'max_width', 'min_height', 'max_height',
            'min_profile_ratio')}

    def debug_mask(self) -> Optional[np.ndarray]:
        """Píxeles que forman parte de alguna detección en la última imagen."""
        return self._mask

    def detect(self, observation: Observation) -> List[Detection]:
        """Detecciones de objetos bajos con forma plausible de víctima."""
        p = self.params
        depth = observation.depth
        h, w = depth.shape
        self._mask = np.zeros((h, w), dtype=np.uint8)
        if observation.sensor_to_base is None:
            return []
        stride = max(int(p['stride']), 1)
        vs, us = np.mgrid[0:h:stride, 0:w:stride]
        z = depth[vs, us].astype(np.float32)
        valid = np.isfinite(z) & (z >= p['min_depth']) & (z <= p['max_depth'])
        us, vs, z = us[valid], vs[valid], z[valid]
        if len(z) == 0:
            return []
        k = observation.k
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        cam = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])
        m = np.asarray(observation.sensor_to_base, dtype=float)
        base = cam @ m[:3, :3].T + m[:3, 3]
        height = base[:, 2]

        above_floor = height > p['floor_height']
        tall = above_floor & (height > p['max_victim_height'])
        low = above_floor & ~tall
        if not low.any():
            return []

        # Columnas altas: celdas del suelo con algún punto por encima de max_victim_height.
        cell = float(p['tall_cell'])
        keys = np.floor(base[:, :2] / cell).astype(int)
        if tall.any():
            origin = keys.min(axis=0)
            grid = np.zeros(keys.max(axis=0) - origin + 1, dtype=bool)
            tk = keys[tall] - origin
            grid[tk[:, 0], tk[:, 1]] = True
            if p['tall_dilation'] > 0:
                grid = ndimage.binary_dilation(grid, iterations=int(p['tall_dilation']))
            lk = keys - origin
            low &= ~grid[lk[:, 0], lk[:, 1]]

        # Altura máxima visible a la distancia de cada punto (rayo de la fila superior).
        cam_origin = m[:3, 3]
        top_ray = m[:3, :3] @ np.array([0.0, -cy / fy, 1.0])
        top_elev = math.atan2(top_ray[2], math.hypot(top_ray[0], top_ray[1]))
        dist = np.hypot(base[:, 0] - cam_origin[0], base[:, 1] - cam_origin[1])
        visible_top = cam_origin[2] + dist * math.tan(top_elev)
        low &= visible_top >= p['min_verifiable_height']
        if not low.any():
            return []

        return self._clusters(base[low], cam[low], us[low], vs[low], stride)

    def _clusters(self, base, cam, us, vs, stride) -> List[Detection]:
        """Agrupa los puntos bajos en el plano del suelo; una detección por grupo válido."""
        p = self.params
        cell = float(p['cluster_cell'])
        keys = np.floor(base[:, :2] / cell).astype(int)
        origin = keys.min(axis=0)
        idx = keys - origin
        grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
        grid[idx[:, 0], idx[:, 1]] = True
        labels, count = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
        point_labels = labels[idx[:, 0], idx[:, 1]]
        detections = []
        for label in range(1, count + 1):
            sel = point_labels == label
            if sel.sum() < p['min_cluster_points']:
                continue
            xy = base[sel, :2]
            extent = float(np.hypot(*np.ptp(xy, axis=0)))
            if extent > p['max_cluster_extent']:
                continue
            pts = cam[sel]
            u, v = float(np.median(us[sel])), float(np.median(vs[sel]))
            self._mask[vs[sel], us[sel]] = 255
            detections.append(Detection(
                kind=self.name, point=tuple(float(c) for c in np.median(pts, axis=0)),
                pixel=(u, v), confidence=0.5,
                extra={'height': float(base[sel, 2].max()), 'extent': extent,
                       'points': float(sel.sum())},
                points=pts))
        if stride > 1 and detections:
            self._mask = cv2.dilate(self._mask, np.ones((stride, stride), np.uint8))
        return detections
