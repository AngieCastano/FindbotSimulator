"""
Signo de vida "calor corporal" con la cámara térmica (MLX90640, 32×24 px).

La cámara térmica no mide distancia: cada píxel caliente da solo una dirección. Por imagen:

1. **Píxeles calientes.** Temperatura de al menos `min_temperature` y `min_contrast` por encima
   del fondo (mediana de la imagen).
2. **Manchas.** Píxeles calientes conectados (8 vecinos); se descartan las de menos de
   `min_pixels` píxeles (ruido) y las que tienen algún píxel por encima de `max_temperature`
   (una persona vestida se ve a 30–36 °C; más caliente es una fuente artificial). El filtro va
   por mancha y no por píxel: los píxeles del borde de un objeto muy caliente, cubiertos solo en
   parte, promedian una temperatura "humana".
3. **Rayos.** Una dirección por columna de la mancha (centro de la columna, altura media de sus
   píxeles calientes). El nodo prolonga cada rayo sobre el /map del propio robot hasta la primera
   celda ocupada: el LiDAR ve el torso y la cabeza de las víctimas (más altos que su plano).
4. **Cota de distancia por el suelo.** La víctima apoya en el suelo: su punto de apoyo está en el
   píxel caliente más bajo de la columna o en el de debajo (cubierto solo en parte), o más arriba
   si algo lo tapa. Con la altura de la cámara eso acota la distancia: d ≤ h / tan(depresión del
   borde superior de ese píxel). Así un rayo que pasa por encima de las piernas (que el LiDAR no
   ve) no se prolonga hasta el escombro de detrás. Con 1.9° por píxel la cota solo es útil cerca
   (≲ 2 m); más lejos el píxel más bajo toca el horizonte y queda `max_range`.
"""

from typing import Dict, List

import numpy as np
from scipy import ndimage

from .sign_of_life import Detection, register_detector, SignOfLifeDetector, ThermalObservation

DEFAULTS = {
    'min_temperature': 28.0,    # °C
    'max_temperature': 40.0,    # °C: más caliente no es una persona (motor, tubería, fuego)
    'min_contrast': 5.0,        # °C sobre el fondo (mediana de la imagen)
    'min_pixels': 2,
    'max_range': 6.0,           # m: más lejos, una víctima ocupa menos de un píxel de alto
    'range_margin': 0.5,        # m añadidos a la cota de distancia por el suelo
}


@register_detector
class ThermalDetector(SignOfLifeDetector):
    """Manchas de temperatura corporal → rayos en el frame de la cámara térmica."""

    name = 'thermal'
    sensor = 'thermal'

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **params})
        self._mask = None

    def detect(self, observation: ThermalObservation) -> List[Detection]:
        """Manchas calientes de la imagen, con un rayo por columna."""
        p = self.params
        temp = np.asarray(observation.temperature, dtype=np.float32)
        finite = np.isfinite(temp)
        self._mask = np.zeros(temp.shape, dtype=np.uint8)
        if not finite.any():
            return []
        background = float(np.median(temp[finite]))
        hot = finite & (temp >= p['min_temperature']) & \
            (temp - background >= p['min_contrast'])
        labels, count = ndimage.label(hot, structure=np.ones((3, 3), dtype=bool))
        fx, cx, fy, cy = (observation.k[0], observation.k[2], observation.k[4],
                          observation.k[5])
        base = observation.sensor_to_base
        detections = []
        for label in range(1, count + 1):
            rows, cols = np.nonzero(labels == label)
            peak = float(temp[rows, cols].max())
            if len(rows) < int(p['min_pixels']) or peak > p['max_temperature']:
                continue
            self._mask[rows, cols] = 255
            columns = np.unique(cols)
            mid_v = np.array([rows[cols == u].mean() for u in columns]) + 0.5
            low_v = np.array([rows[cols == u].max() for u in columns], dtype=float)
            u_center = columns + 0.5
            rays = np.column_stack([(u_center - cx) / fx, (mid_v - cy) / fy,
                                    np.ones(len(columns))])
            rays /= np.linalg.norm(rays, axis=1, keepdims=True)
            bound = np.full(len(columns), float(p['max_range']))
            if base is not None:
                # Borde superior del píxel caliente más bajo, en el frame de la base.
                low = np.column_stack([(u_center - cx) / fx, (low_v - cy) / fy,
                                       np.ones(len(columns))]) @ base[:3, :3].T
                depression = -np.arctan2(low[:, 2], np.hypot(low[:, 0], low[:, 1]))
                height = float(base[2, 3])
                with np.errstate(divide='ignore'):
                    floor = np.where(depression > 1e-3,
                                     height / np.tan(np.maximum(depression, 1e-3))
                                     + p['range_margin'], np.inf)
                bound = np.minimum(bound, floor)
            direction = rays.mean(axis=0)
            detections.append(Detection(
                kind=self.name, point=tuple(direction / np.linalg.norm(direction)),
                pixel=(float(cols.mean() + 0.5), float(rows.mean() + 0.5)),
                confidence=float(min(1.0, (peak - background) / (2.0 * p['min_contrast']))),
                extra={'temperature': peak, 'background': background,
                       'pixels': float(len(rows))},
                rays=rays, ray_max_range=bound))
        return detections

    def debug_mask(self):
        """Píxeles de las manchas aceptadas en la última imagen."""
        return self._mask
