"""Primitivas geométricas 2D (huellas en planta) y pruebas de intersección."""

from dataclasses import dataclass
import math
from typing import Tuple, Union

import numpy as np


@dataclass(frozen=True)
class Box:
    """Rectángulo orientado: centro (x, y), lados (size_x, size_y) y giro yaw."""

    x: float
    y: float
    size_x: float
    size_y: float
    yaw: float = 0.0

    def corners(self) -> np.ndarray:
        """Esquinas en coordenadas del mundo, forma (4, 2), en sentido antihorario."""
        hx, hy = self.size_x / 2, self.size_y / 2
        local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, -s], [s, c]])
        return local @ rot.T + np.array([self.x, self.y])

    def axes(self) -> Tuple[np.ndarray, np.ndarray]:
        """Ejes unitarios locales (u, v) expresados en el mundo."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c, s]), np.array([-s, c])


@dataclass(frozen=True)
class Circle:
    """Círculo: centro (x, y) y radio."""

    x: float
    y: float
    radius: float


Shape = Union[Box, Circle]


def point_distance(shape: Shape, px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """
    Distancia euclidiana de los puntos (px, py) a la figura (0 en el interior).

    Acepta arreglos de cualquier forma (vectorizado).
    """
    if isinstance(shape, Circle):
        return np.maximum(np.hypot(px - shape.x, py - shape.y) - shape.radius, 0.0)
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    dx, dy = px - shape.x, py - shape.y
    u = c * dx + s * dy
    v = -s * dx + c * dy
    ex = np.maximum(np.abs(u) - shape.size_x / 2, 0.0)
    ey = np.maximum(np.abs(v) - shape.size_y / 2, 0.0)
    return np.hypot(ex, ey)


def contains(shape: Shape, px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """Máscara booleana de los puntos (px, py) que caen dentro de la figura."""
    if isinstance(shape, Circle):
        return np.hypot(px - shape.x, py - shape.y) <= shape.radius
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    dx, dy = px - shape.x, py - shape.y
    u = c * dx + s * dy
    v = -s * dx + c * dy
    return (np.abs(u) <= shape.size_x / 2) & (np.abs(v) <= shape.size_y / 2)


def _project(box: Box, axis: np.ndarray) -> Tuple[float, float]:
    """Intervalo [min, max] de la proyección de la caja sobre un eje unitario."""
    proj = box.corners() @ axis
    return float(proj.min()), float(proj.max())


def _boxes_intersect(a: Box, b: Box, gap: float) -> bool:
    """Teorema del eje separador; conservador cuando gap > 0."""
    for axis in (*a.axes(), *b.axes()):
        a_min, a_max = _project(a, axis)
        b_min, b_max = _project(b, axis)
        if a_max + gap <= b_min or b_max + gap <= a_min:
            return False
    return True


def shapes_intersect(a: Shape, b: Shape, gap: float = 0.0) -> bool:
    """Comprueba si las figuras se solapan o quedan a menos de gap metros."""
    if isinstance(a, Circle) and isinstance(b, Circle):
        return math.hypot(a.x - b.x, a.y - b.y) < a.radius + b.radius + gap
    if isinstance(a, Circle):
        a, b = b, a
    if isinstance(b, Circle):
        dist = float(point_distance(a, np.array(b.x), np.array(b.y)))
        return dist < b.radius + gap
    return _boxes_intersect(a, b, gap)


def shape_in_bounds(shape: Shape, bounds: Tuple[float, float, float, float]) -> bool:
    """Comprueba si la figura está completamente dentro de (x_min, x_max, y_min, y_max)."""
    x_min, x_max, y_min, y_max = bounds
    if isinstance(shape, Circle):
        return (x_min <= shape.x - shape.radius and shape.x + shape.radius <= x_max
                and y_min <= shape.y - shape.radius and shape.y + shape.radius <= y_max)
    pts = shape.corners()
    return bool((pts[:, 0] >= x_min).all() and (pts[:, 0] <= x_max).all()
                and (pts[:, 1] >= y_min).all() and (pts[:, 1] <= y_max).all())
