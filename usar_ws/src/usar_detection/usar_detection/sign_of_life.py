"""
Interfaz común para detectores de signos de vida.

Un detector recibe una observación de los sensores del robot (por ahora la cámara RGB-D) y
devuelve detecciones con su posición 3D en el frame del sensor. El nodo victim_detector no conoce
el algoritmo concreto: para añadir otro signo de vida (calor, sonido, CO₂) basta con escribir una
subclase de SignOfLifeDetector y registrarla con @register_detector, sin modificar el nodo.
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


class SignOfLifeDetector(ABC):
    """Clase base de los detectores de signos de vida."""

    #: Nombre con el que se selecciona el detector en los parámetros (lista `detectors`).
    name = 'base'

    def __init__(self, params: Dict):
        self.params = params

    @abstractmethod
    def detect(self, observation: Observation) -> List[Detection]:
        """Devuelve las detecciones presentes en la observación."""

    def debug_mask(self) -> Optional[np.ndarray]:
        """Máscara de la última detección, para visualización (opcional)."""
        return None

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


def backproject(u: float, v: float, z: float, k: Sequence[float]) -> Tuple[float, float, float]:
    """Punto 3D (frame óptico: x derecha, y abajo, z adelante) del píxel (u, v) a profundidad z."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)
