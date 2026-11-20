"""Parámetros θ del generador procedural E = G(θ, s)."""

from dataclasses import asdict, dataclass, fields
from typing import Any, Dict, Optional, Tuple

import yaml


VICTIM_APPEARANCES = ('orange', 'dusty')


@dataclass
class WorldParams:
    """
    Conjunto de parámetros θ (más la semilla s) que definen un escenario.

    Todas las longitudes están en metros y los ángulos en radianes.
    """

    seed: int = 42
    # Dimensiones interiores del área (ancho en x, largo en y), centrada en el origen.
    size: Tuple[float, float] = (20.0, 20.0)
    n_obstacles: int = 40
    # Rango del lado de las cajas / diámetro de los cilindros.
    obstacle_size_range: Tuple[float, float] = (0.3, 2.0)
    # Rango de alturas; el mínimo debe superar la altura del LiDAR para que sea visible.
    obstacle_height_range: Tuple[float, float] = (0.4, 1.5)
    # Fracción de escombros que son cajas (el resto son cilindros).
    box_ratio: float = 0.6
    # Separación mínima entre objetos (evita contactos exactos).
    min_gap: float = 0.05
    n_victims: int = 5
    # Pose de aparición del robot: x, y, yaw.
    robot_spawn: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Radio de la zona libre alrededor del spawn.
    spawn_clearance: float = 1.0
    # Radio del robot usado para inflar los obstáculos en la rejilla.
    robot_radius: float = 0.30
    # Resolución de la rejilla del área explorable (igual a la del /map de SLAM).
    grid_resolution: float = 0.05
    # Distancia máxima desde una celda alcanzable hasta la víctima para considerarla alcanzable.
    victim_reach_distance: float = 1.0
    # Aspecto de las víctimas: 'orange' (naranja distintivo, MVP) o 'dusty' (gris como los
    # escombros, "cubiertas de polvo"): con 'dusty' el color no permite detectarlas.
    victim_appearance: str = 'orange'
    # Escombros bajos (tablones, losas, bloques) que se confunden geométricamente con una víctima
    # acostada. Se generan con un flujo aleatorio aparte: con 0 el mundo no cambia.
    n_low_debris: int = 0
    low_debris_height_range: Tuple[float, float] = (0.08, 0.28)
    # Temperaturas (Fase 9, cámara térmica). Ambiente: todo lo que no tiene temperatura propia
    # (suelo, muros, escombros). Víctimas: temperatura de la superficie de una persona vestida
    # (≈ 30–34 °C; la temperatura interna de 37 °C no se ve desde fuera).
    ambient_temperature_c: float = 20.0
    victim_temperature_c: float = 34.0
    # Escombros "tibios" (calentados por el sol, tuberías): fuentes de falsos positivos térmicos.
    # Se eligen entre los escombros ya colocados con un flujo aleatorio propio (no cambian la
    # disposición del escenario).
    n_warm_debris: int = 0
    warm_debris_temperature_range: Tuple[float, float] = (28.0, 45.0)
    wall_height: float = 1.0
    wall_thickness: float = 0.2
    # Reintentos de generación completa (cada uno con una sub-semilla derivada).
    max_attempts: int = 50
    # Intentos de colocación aleatoria por objeto dentro de un mismo reintento.
    max_placement_tries: int = 2000

    def validate(self) -> None:
        """Lanza ValueError si algún parámetro es inválido."""
        if self.seed < 0:
            raise ValueError('seed debe ser un entero >= 0')
        width, length = self.size
        if width <= 0 or length <= 0:
            raise ValueError('size debe ser positivo')
        lo, hi = self.obstacle_size_range
        if not 0 < lo <= hi:
            raise ValueError('obstacle_size_range debe cumplir 0 < min <= max')
        lo, hi = self.obstacle_height_range
        if not 0 < lo <= hi:
            raise ValueError('obstacle_height_range debe cumplir 0 < min <= max')
        if not 0.0 <= self.box_ratio <= 1.0:
            raise ValueError('box_ratio debe estar en [0, 1]')
        if min(self.n_obstacles, self.n_victims, self.n_low_debris, self.n_warm_debris) < 0:
            raise ValueError('n_obstacles, n_victims, n_low_debris y n_warm_debris no pueden ser '
                             'negativos')
        if self.n_warm_debris > self.n_obstacles + self.n_low_debris:
            raise ValueError('n_warm_debris no puede superar el número de escombros')
        lo, hi = self.warm_debris_temperature_range
        if not lo <= hi:
            raise ValueError('warm_debris_temperature_range debe cumplir min <= max')
        if not -40.0 <= self.ambient_temperature_c <= 60.0:
            raise ValueError('ambient_temperature_c debe estar entre -40 y 60 °C')
        if self.victim_appearance not in VICTIM_APPEARANCES:
            raise ValueError(f'victim_appearance debe ser uno de {VICTIM_APPEARANCES}')
        lo, hi = self.low_debris_height_range
        if not 0 < lo <= hi:
            raise ValueError('low_debris_height_range debe cumplir 0 < min <= max')
        if self.robot_radius <= 0 or self.grid_resolution <= 0:
            raise ValueError('robot_radius y grid_resolution deben ser positivos')
        if self.spawn_clearance < self.robot_radius:
            raise ValueError('spawn_clearance debe ser >= robot_radius')
        if self.max_attempts < 1 or self.max_placement_tries < 1:
            raise ValueError('max_attempts y max_placement_tries deben ser >= 1')
        x, y, _ = self.robot_spawn
        margin = self.spawn_clearance
        if abs(x) > width / 2 - margin or abs(y) > length / 2 - margin:
            raise ValueError('robot_spawn (con su zona libre) debe quedar dentro del área')

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Límites interiores (x_min, x_max, y_min, y_max)."""
        width, length = self.size
        return (-width / 2, width / 2, -length / 2, length / 2)

    def to_dict(self) -> Dict[str, Any]:
        """Diccionario serializable (tuplas como listas)."""
        data = asdict(self)
        return {k: list(v) if isinstance(v, tuple) else v for k, v in data.items()}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'WorldParams':
        """Crea parámetros desde un diccionario; rechaza claves desconocidas."""
        known = {f.name: f for f in fields(cls)}
        unknown = set(data) - set(known)
        if unknown:
            raise ValueError(f'Parámetros desconocidos: {sorted(unknown)}')
        kwargs = {}
        for key, value in data.items():
            kwargs[key] = tuple(value) if isinstance(value, (list, tuple)) else value
        params = cls(**kwargs)
        params._normalize_types()
        return params

    def merged(self, overrides: Optional[Dict[str, Any]]) -> 'WorldParams':
        """Devuelve una copia con los valores no nulos de overrides aplicados."""
        data = asdict(self)
        for key, value in (overrides or {}).items():
            if value is not None:
                data[key] = value
        return WorldParams.from_dict(data)

    def _normalize_types(self) -> None:
        """Fuerza tipos numéricos (p. ej. enteros leídos de YAML como float)."""
        self.seed = int(self.seed)
        self.size = tuple(float(v) for v in self.size)
        self.obstacle_size_range = tuple(float(v) for v in self.obstacle_size_range)
        self.obstacle_height_range = tuple(float(v) for v in self.obstacle_height_range)
        self.low_debris_height_range = tuple(float(v) for v in self.low_debris_height_range)
        self.warm_debris_temperature_range = tuple(
            float(v) for v in self.warm_debris_temperature_range)
        self.victim_appearance = str(self.victim_appearance)
        self.robot_spawn = tuple(float(v) for v in self.robot_spawn)
        for name in ('n_obstacles', 'n_victims', 'n_low_debris', 'n_warm_debris', 'max_attempts',
                     'max_placement_tries'):
            setattr(self, name, int(getattr(self, name)))
        for name in ('box_ratio', 'min_gap', 'spawn_clearance', 'robot_radius',
                     'grid_resolution', 'victim_reach_distance', 'wall_height',
                     'wall_thickness', 'ambient_temperature_c', 'victim_temperature_c'):
            setattr(self, name, float(getattr(self, name)))


def load_params_yaml(path: str) -> Dict[str, Any]:
    """Lee un YAML de parámetros θ y devuelve el diccionario (sin validar)."""
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: el YAML de parámetros debe ser un diccionario')
    return data
