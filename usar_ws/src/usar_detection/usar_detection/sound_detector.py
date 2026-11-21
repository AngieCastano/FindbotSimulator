"""
Signo de vida "voz / golpes" con un arreglo de micrófonos (ReSpeaker USB Mic Array v2.0).

El arreglo entrega, varias veces por segundo, la dirección de llegada del sonido más fuerte y si
es voz (no solo ruido). Como la cámara térmica, da una dirección pero no una distancia: la
víctima se ubica por **triangulación** de las direcciones tomadas desde puntos distintos
(bearing_evidence), con dos diferencias:

- **El sonido atraviesa los escombros**: los rayos no se cortan en el primer obstáculo del /map
  ni en lo desconocido (el interior de un escombro nunca se cartografía); solo en el borde del
  mapa o a `max_range` m.
- **Solo se usan lecturas con el robot quieto**: en marcha, el ruido de los motores tapa los
  sonidos débiles y la dirección es poco fiable. El detector lo decide con la pose del propio
  robot (TF): si en `stationary_window` s el arreglo no se ha movido más de
  `stationary_motion` m ni girado más de `stationary_turn` rad. Las paradas de escucha del
  explorador (listen_*) dan esas ocasiones.

La dirección es menos precisa que la de la cámara térmica (decenas de grados con señal débil, y
ecos): celdas de triangulación mayores.
"""

from collections import deque
import math
from typing import Dict, List

import numpy as np

from .sign_of_life import Detection, register_detector, SignOfLifeDetector, SoundObservation

DEFAULTS = {
    'min_confidence': 0.1,
    'stationary_window': 1.0,      # s
    'stationary_motion': 0.03,     # m
    'stationary_turn': 0.05,       # rad
    'max_range': 10.0,             # m
    'bearing_cell': 0.3,           # m
    'bearing_bins': 12,            # sectores de 30°
    'min_bearings': 3,
    'min_range': 0.3,              # m
}


@register_detector
class SoundDetector(SignOfLifeDetector):
    """Direcciones de llegada de voz con el robot quieto → rayos en el frame global."""

    name = 'sound'
    sensor = 'sound'
    position_sigma = 0.7          # m: triangulación con direcciones de ±10–40°

    def __init__(self, params: Dict):
        super().__init__({**DEFAULTS, **params})
        self._poses = deque()                      # (t, x, y, yaw)
        self.listening = False

    def tracker_params(self) -> Dict[str, float]:
        """Evidencia en celdas mayores: la triangulación del sonido es poco precisa."""
        return {'cell_size': 0.3, 'join_distance': 0.6}

    def bearing_params(self) -> Dict[str, float]:
        """Rayos que atraviesan escombros y lo desconocido, hasta max_range."""
        p = self.params
        return {'cell': p['bearing_cell'], 'bins': p['bearing_bins'],
                'min_bearings': p['min_bearings'], 'min_range': p['min_range'],
                'beyond': 1.0e3, 'stop_unknown': False}

    def _stationary(self, obs: SoundObservation) -> bool:
        p = self.params
        self._poses.append((obs.stamp_sec, obs.position[0], obs.position[1], obs.yaw))
        while self._poses and obs.stamp_sec - self._poses[0][0] > p['stationary_window']:
            self._poses.popleft()
        if obs.stamp_sec - self._poses[0][0] < 0.8 * p['stationary_window']:
            return False
        poses = np.array(self._poses)
        moved = np.hypot(poses[:, 1] - obs.position[0], poses[:, 2] - obs.position[1]).max()
        turned = np.abs(np.arctan2(np.sin(poses[:, 3] - obs.yaw),
                                   np.cos(poses[:, 3] - obs.yaw))).max()
        return moved <= p['stationary_motion'] and turned <= p['stationary_turn']

    def detect(self, observation: SoundObservation) -> List[Detection]:
        """Un rayo (frame global) si hay voz con confianza suficiente y el robot está quieto."""
        p = self.params
        self.listening = self._stationary(observation)
        if not (self.listening and observation.voice_activity
                and observation.confidence >= p['min_confidence']):
            return []
        heading = observation.yaw + observation.azimuth
        ray = np.array([[math.cos(heading), math.sin(heading), 0.0]])
        return [Detection(kind=self.name, point=tuple(ray[0]), pixel=(0.0, 0.0),
                          confidence=float(observation.confidence),
                          extra={'level_db': float(observation.level_db),
                                 'azimuth': float(observation.azimuth)},
                          rays=ray, ray_max_range=np.array([p['max_range']]))]
