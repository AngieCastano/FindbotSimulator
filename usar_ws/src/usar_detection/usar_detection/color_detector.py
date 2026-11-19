"""
Detector de "forma humana" por color (signo de vida del MVP).

Las víctimas simuladas tienen un color naranja distintivo. El detector:
  1. convierte la imagen a HSV y umbraliza el tono, la saturación y el brillo,
  2. limpia la máscara con apertura y cierre morfológicos,
  3. separa los blobs (componentes conexas) y descarta los de área pequeña,
  4. calcula el punto 3D de cada píxel del blob con profundidad válida y los agrupa en el plano
     del suelo (lateral × profundidad) con una rejilla de cluster_cell: dos víctimas que en la
     imagen forman un único blob (una detrás de otra, o casi pegadas) dan grupos separados,
  5. cada grupo con suficientes puntos es una detección, cuyo punto 3D es la mediana de sus
     puntos (más robusta que el píxel del centroide, que puede caer en el suelo entre las
     extremidades).
"""

from typing import Dict, List, Optional

import cv2
import numpy as np
from scipy import ndimage

from usar_detection.sign_of_life import (
    Detection, Observation, register_detector, SignOfLifeDetector)

DEFAULTS = {
    'hsv_lower': [10, 150, 60],          # OpenCV: H ∈ [0, 180], S y V ∈ [0, 255]
    'hsv_upper': [30, 255, 255],
    'morph_kernel': 5,                   # px: lado del elemento estructurante
    'min_blob_area': 150,                # px (en la imagen de 640×480)
    'min_depth': 0.1,                    # m
    'max_depth': 6.0,                    # m: más lejos la profundidad es poco fiable
    'min_valid_depth_fraction': 0.3,     # fracción de píxeles del blob con profundidad válida
    'max_points_per_blob': 2000,         # submuestreo de los puntos 3D de cada blob
    'cluster_cell': 0.15,                # m: rejilla (lateral × profundidad) para separar objetos
    'min_cluster_points': 30,            # puntos (tras el submuestreo) para aceptar un grupo
}


@register_detector
class ColorShapeDetector(SignOfLifeDetector):
    """Segmentación de color HSV + profundidad → posición 3D de cada blob."""

    name = 'color_shape'

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **(params or {})})
        p = self.params
        self.lower = np.array(p['hsv_lower'], dtype=np.uint8)
        self.upper = np.array(p['hsv_upper'], dtype=np.uint8)
        size = max(int(p['morph_kernel']), 1)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        self._mask: Optional[np.ndarray] = None

    def segment(self, rgb: np.ndarray) -> np.ndarray:
        """Máscara binaria (uint8 0/255) de los píxeles con el color de las víctimas."""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    def detect(self, observation: Observation) -> List[Detection]:
        """Detecta los blobs de color y estima su posición 3D en el frame de la cámara."""
        p = self.params
        mask = self.segment(observation.rgb)
        self._mask = mask
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        k = observation.k
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        detections = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < p['min_blob_area']:
                continue
            rows, cols = np.nonzero(labels == label)
            z = observation.depth[rows, cols].astype(np.float64)
            valid = np.isfinite(z) & (z >= p['min_depth']) & (z <= p['max_depth'])
            if valid.sum() < max(p['min_valid_depth_fraction'] * area, 1):
                continue
            rows, cols, z = rows[valid], cols[valid], z[valid]
            if len(z) > p['max_points_per_blob']:
                step = int(np.ceil(len(z) / p['max_points_per_blob']))
                rows, cols, z = rows[::step], cols[::step], z[::step]
            xs = (cols - cx) * z / fx
            ys = (rows - cy) * z / fy
            for members in self._split(xs, z):
                point = (float(np.median(xs[members])), float(np.median(ys[members])),
                         float(np.median(z[members])))
                pixel = (float(np.mean(cols[members])), float(np.mean(rows[members])))
                detections.append(Detection(
                    kind=self.name, point=point, pixel=pixel,
                    confidence=float(valid.mean()),
                    extra={'area_px': float(area), 'depth_m': point[2]},
                    points=np.column_stack([xs[members], ys[members], z[members]])))
        return detections

    def _split(self, xs: np.ndarray, zs: np.ndarray) -> List[np.ndarray]:
        """Grupos de puntos conexos en una rejilla (lateral x, profundidad z) de cluster_cell."""
        cell = self.params['cluster_cell']
        ix = np.floor(xs / cell).astype(int)
        iz = np.floor(zs / cell).astype(int)
        ix -= ix.min()
        iz -= iz.min()
        grid = np.zeros((iz.max() + 1, ix.max() + 1), dtype=bool)
        grid[iz, ix] = True
        labels, count = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
        point_labels = labels[iz, ix]
        groups = [point_labels == k for k in range(1, count + 1)]
        return [g for g in groups if g.sum() >= self.params['min_cluster_points']]

    def debug_mask(self) -> Optional[np.ndarray]:
        """Máscara de la última imagen procesada."""
        return self._mask
