"""
Interfaz común para detectores de signos de vida.

Un detector recibe una observación de un sensor del robot (cámara RGB-D o cámara térmica, según
su atributo `sensor`) y devuelve detecciones en el frame del sensor: puntos 3D si el sensor mide
distancia, o solo direcciones (rayos) si no la mide, como la cámara térmica. El nodo
victim_detector no conoce el algoritmo concreto: para añadir otro signo de vida (sonido, CO₂)
basta con escribir una subclase de SignOfLifeDetector y registrarla con @register_detector,
sin modificar el nodo.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Observation:
    """Datos de una captura de la cámara RGB-D (todo en el frame óptico de la cámara)."""

    rgb: np.ndarray                      # H×W×3, uint8, orden RGB
    depth: np.ndarray                    # H×W, float32, metros (inf/nan = sin dato)
    k: Sequence[float]                   # matriz intrínseca 3×3 en fila (camera_info.k)
    stamp_sec: float = 0.0
    frame_id: str = ''
    #: Transformación 4×4 base del robot ← sensor (base_footprint: z hacia arriba, suelo en z=0).
    #: La necesitan los detectores que razonan con la altura sobre el suelo (forma humana).
    sensor_to_base: Optional[np.ndarray] = None


@dataclass
class ThermalObservation:
    """Imagen de la cámara térmica (°C por píxel, frame óptico de la cámara térmica)."""

    temperature: np.ndarray              # H×W, float32, °C
    k: Sequence[float]                   # matriz intrínseca 3×3 en fila
    stamp_sec: float = 0.0
    frame_id: str = ''
    #: Transformación 4×4 base del robot ← sensor (base_footprint: z hacia arriba, suelo en z=0).
    sensor_to_base: Optional[np.ndarray] = None


@dataclass
class Co2Observation:
    """Lectura de un sensor de gas puntual (CO₂), con la posición del sensor en el mapa."""

    concentration: float                 # ppm
    position: Tuple[float, float]        # (x, y) del sensor en el frame global (map)
    stamp_sec: float = 0.0
    frame_id: str = ''


@dataclass
class SoundObservation:
    """Dirección de llegada del sonido (arreglo de micrófonos), con la pose del arreglo."""

    azimuth: float                       # rad en el frame del arreglo
    level_db: float
    voice_activity: bool
    confidence: float
    position: Tuple[float, float]        # (x, y) del arreglo en el frame global
    yaw: float                           # orientación del arreglo en el frame global (rad)
    stamp_sec: float = 0.0
    frame_id: str = ''


@dataclass
class Detection:
    """Una detección de un signo de vida."""

    kind: str                            # nombre del signo de vida (p. ej. 'color_shape')
    point: Tuple[float, float, float]    # posición 3D en el frame del sensor (m)
    pixel: Tuple[float, float]           # (u, v) en la imagen
    confidence: float                    # 0..1, según el detector
    extra: Dict[str, float] = field(default_factory=dict)
    #: Puntos 3D (N×3, frame del sensor) que sustentan la detección; alimentan el mapa de
    #: evidencia. Si el detector no los da, se usa solo `point`.
    points: Optional[np.ndarray] = None
    #: Sensores sin distancia (térmica): direcciones unitarias (N×3, frame del sensor). El nodo
    #: las prolonga sobre el /map del robot hasta la primera celda ocupada.
    rays: Optional[np.ndarray] = None
    #: Distancia máxima (m) plausible de cada rayo según la geometría de la imagen (N); inf si no
    #: se puede acotar.
    ray_max_range: Optional[np.ndarray] = None


class SignOfLifeDetector(ABC):
    """Clase base de los detectores de signos de vida."""

    #: Nombre con el que se selecciona el detector en los parámetros (lista `detectors`).
    name = 'base'
    #: Error de localización típico (m): pesa la posición al unir detectores distintos.
    position_sigma = 0.3
    #: Sensor que consume: 'rgbd' (Observation), 'thermal' (ThermalObservation), 'co2'
    #: (Co2Observation) o 'sound' (SoundObservation). Las detecciones de 'co2' y 'sound' ya
    #: vienen en el frame global (puntos y rayos, respectivamente).
    sensor = 'rgbd'

    def __init__(self, params: Dict):
        self.params = params
        #: Detector guía (`<nombre>.cue_only: true`): sus confirmaciones solas no son víctimas,
        #: sino candidatos que el explorador va a mirar (ver VictimRegistry).
        self.cue_only = bool(params.get('cue_only', False))

    @abstractmethod
    def detect(self, observation: Observation) -> List[Detection]:
        """Devuelve las detecciones presentes en la observación."""

    def debug_mask(self) -> Optional[np.ndarray]:
        """Máscara de la última detección, para visualización (opcional)."""
        return None

    def tracker_params(self) -> Dict[str, float]:
        """Parámetros del mapa de evidencia propios de este detector (p. ej. celdas mayores)."""
        return {}

    def bearing_params(self) -> Dict[str, float]:
        """
        Triangulación de los rayos de un sensor sin distancia (ver bearing_evidence).

        Claves: cell, bins, min_bearings, min_range, beyond (m tras el primer obstáculo del
        /map; muy grande = el rayo atraviesa escombros), stop_unknown (cortar en lo
        desconocido).
        """
        return {}

    def group_shape(self) -> Optional[Dict[str, float]]:
        """
        Filtro de forma que el mapa de evidencia exige a un grupo para confirmarlo (opcional).

        Claves: ver usar_detection.tracker.SHAPE_KEYS. None = sin filtro.
        """
        return None


_REGISTRY: Dict[str, Callable[[Dict], SignOfLifeDetector]] = {}


def register_detector(cls):
    """Decorador: registra una subclase de SignOfLifeDetector por su atributo `name`."""
    _REGISTRY[cls.name] = cls
    return cls


def create_detector(name: str, params: Dict) -> SignOfLifeDetector:
    """Instancia el detector registrado con ese nombre."""
    if name not in _REGISTRY:
        raise ValueError(f'Detector desconocido: {name!r}; disponibles: {sorted(_REGISTRY)}')
    return _REGISTRY[name](params)


def available_detectors() -> List[str]:
    """Nombres de los detectores registrados."""
    return sorted(_REGISTRY)


def intrinsics_from_fov(width: int, height: int, horizontal_fov: float) -> List[float]:
    """Matriz intrínseca (fila) de una cámara ideal de píxeles cuadrados, centro en la imagen."""
    fx = (width / 2.0) / np.tan(horizontal_fov / 2.0)
    return [fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0]


def backproject(u: float, v: float, z: float, k: Sequence[float]) -> Tuple[float, float, float]:
    """Punto 3D (frame óptico: x derecha, y abajo, z adelante) del píxel (u, v) a profundidad z."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)
