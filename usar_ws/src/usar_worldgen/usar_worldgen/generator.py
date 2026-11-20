"""
Generador procedural E = G(θ, s) de escenarios de escombros.

El generador coloca escombros (cajas y cilindros) y víctimas por muestreo con rechazo, valida
las restricciones del escenario y, si alguna falla, reintenta con una sub-semilla derivada de
forma determinista de la semilla original. Misma semilla y mismos θ ⇒ mismo mundo exacto.
"""

from dataclasses import dataclass, field
import logging
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .geometry import Box, Circle, point_distance, Shape, shape_in_bounds, shapes_intersect
from .grid import clearance_map, explorable_mask, GridSpec
from .params import WorldParams

LOGGER = logging.getLogger('usar_worldgen')

# Huella en planta de la víctima acostada (largo a lo largo de su eje x local, ancho).
VICTIM_LENGTH = 1.7
VICTIM_WIDTH = 0.6

# Colores de escombros (RGB): grises y marrón desaturado, lejos del naranja de las víctimas.
RUBBLE_PALETTE = (
    (0.55, 0.55, 0.52),
    (0.40, 0.40, 0.40),
    (0.45, 0.40, 0.35),
    (0.65, 0.63, 0.60),
)

# Escombros bajos que se confunden geométricamente con una víctima acostada (Fase 8): huella en
# planta (rango del largo, rango del ancho) en m. La altura sale de low_debris_height_range.
#   plank: tablón largo y estrecho; slab: losa ancha; block: bloque pequeño.
LOW_DEBRIS_TYPES = (
    ('plank', (1.2, 2.2), (0.12, 0.25)),
    ('slab', (0.9, 1.5), (0.9, 1.5)),
    ('block', (0.25, 0.5), (0.25, 0.5)),
)

# Decimales con que se redondean todas las magnitudes generadas (reproducibilidad textual).
DECIMALS = 4


class GenerationError(RuntimeError):
    """No se encontró un escenario válido tras agotar los reintentos."""


@dataclass
class Obstacle:
    """Escombro rígido estático."""

    idx: int
    kind: str  # 'box' o 'cylinder'
    shape: Shape
    yaw: float
    height: float
    color: Tuple[float, float, float]
    category: str = 'rubble'  # 'rubble' o el tipo de escombro bajo (plank, slab, block)
    #: Temperatura propia en °C (escombro tibio); None = temperatura ambiente.
    temperature_c: Optional[float] = None


@dataclass
class Victim:
    """Víctima simulada (humanoide acostado; color según params.victim_appearance)."""

    idx: int
    shape: Box
    reach_distance: float = math.inf  # distancia a la celda alcanzable más cercana


@dataclass
class GeneratedWorld:
    """Resultado completo de una generación."""

    params: WorldParams
    attempt: int
    sub_seed: int
    obstacles: List[Obstacle]
    victims: List[Victim]
    grid: GridSpec
    explorable: np.ndarray
    retries: List[Dict[str, object]] = field(default_factory=list)

    @property
    def explorable_area_m2(self) -> float:
        """Área del espacio alcanzable por el centro del robot."""
        return float(self.explorable.sum()) * self.grid.resolution ** 2


def derive_sub_seed(seed: int, attempt: int) -> int:
    """Sub-semilla determinista (uint32) para el reintento attempt de la semilla seed."""
    state = np.random.SeedSequence([seed, attempt]).generate_state(1, dtype=np.uint32)
    return int(state[0])


def _r(value: float) -> float:
    """Redondea a DECIMALS y elimina el cero negativo."""
    out = round(float(value), DECIMALS)
    return 0.0 if out == 0 else out


def _sample_obstacle(rng: np.random.Generator, params: WorldParams, idx: int) -> Obstacle:
    """Muestrea un escombro con posición, tamaño, altura y yaw aleatorios."""
    x_min, x_max, y_min, y_max = params.bounds
    lo, hi = params.obstacle_size_range
    h_lo, h_hi = params.obstacle_height_range
    kind = 'box' if rng.random() < params.box_ratio else 'cylinder'
    x, y = _r(rng.uniform(x_min, x_max)), _r(rng.uniform(y_min, y_max))
    yaw = _r(rng.uniform(-math.pi, math.pi))
    height = _r(rng.uniform(h_lo, h_hi))
    if kind == 'box':
        shape = Box(x, y, _r(rng.uniform(lo, hi)), _r(rng.uniform(lo, hi)), yaw)
    else:
        shape = Circle(x, y, _r(rng.uniform(lo, hi) / 2))
    color = RUBBLE_PALETTE[int(rng.integers(len(RUBBLE_PALETTE)))]
    return Obstacle(idx, kind, shape, yaw, height, color)


def _sample_low_debris(rng: np.random.Generator, params: WorldParams, idx: int) -> Obstacle:
    """Muestrea un escombro bajo (tablón, losa o bloque) con posición, tamaño y yaw aleatorios."""
    x_min, x_max, y_min, y_max = params.bounds
    kind_idx = int(rng.integers(len(LOW_DEBRIS_TYPES)))
    category, (l_lo, l_hi), (w_lo, w_hi) = LOW_DEBRIS_TYPES[kind_idx]
    x, y = _r(rng.uniform(x_min, x_max)), _r(rng.uniform(y_min, y_max))
    yaw = _r(rng.uniform(-math.pi, math.pi))
    height = _r(rng.uniform(*params.low_debris_height_range))
    shape = Box(x, y, _r(rng.uniform(l_lo, l_hi)), _r(rng.uniform(w_lo, w_hi)), yaw)
    color = RUBBLE_PALETTE[int(rng.integers(len(RUBBLE_PALETTE)))]
    return Obstacle(idx, 'box', shape, yaw, height, color, category)


def _sample_victim(rng: np.random.Generator, params: WorldParams, idx: int) -> Victim:
    """Muestrea la pose en planta de una víctima."""
    x_min, x_max, y_min, y_max = params.bounds
    x, y = _r(rng.uniform(x_min, x_max)), _r(rng.uniform(y_min, y_max))
    yaw = _r(rng.uniform(-math.pi, math.pi))
    return Victim(idx, Box(x, y, VICTIM_LENGTH, VICTIM_WIDTH, yaw))


def _is_free(shape: Shape, placed: List[Shape], params: WorldParams, spawn_zone: Circle) -> bool:
    """Dentro de límites, fuera de la zona del spawn y sin solapes con lo ya colocado."""
    if not shape_in_bounds(shape, params.bounds):
        return False
    if shapes_intersect(shape, spawn_zone):
        return False
    return not any(shapes_intersect(shape, other, params.min_gap) for other in placed)


def _place_low_debris(params: WorldParams, sub_seed: int, debris_try: int,
                      placed: List[Shape], spawn_zone: Circle
                      ) -> Tuple[Optional[List[Obstacle]], str]:
    """Coloca los escombros bajos sin solapes; (None, motivo) si no caben."""
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([sub_seed, 1, debris_try])))
    debris: List[Obstacle] = []
    shapes = list(placed)
    for k in range(params.n_low_debris):
        idx = params.n_obstacles + k
        for _ in range(params.max_placement_tries):
            candidate = _sample_low_debris(rng, params, idx)
            if _is_free(candidate.shape, shapes, params, spawn_zone):
                debris.append(candidate)
                shapes.append(candidate.shape)
                break
        else:
            return None, (f'no se pudo colocar el escombro bajo {k} tras '
                          f'{params.max_placement_tries} intentos')
    return debris, ''


def _assign_warm_debris(params: WorldParams, sub_seed: int, obstacles: List[Obstacle]) -> None:
    """
    Elige n_warm_debris escombros y les da una temperatura propia (flujo aleatorio aparte).

    Con n_warm_debris = 0 no se consume ningún número aleatorio: el mundo no cambia.
    """
    if not params.n_warm_debris:
        return
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([sub_seed, 2])))
    chosen = rng.choice(len(obstacles), size=params.n_warm_debris, replace=False)
    lo, hi = params.warm_debris_temperature_range
    for i in sorted(int(c) for c in chosen):
        obstacles[i].temperature_c = _r(rng.uniform(lo, hi))


def _check_reachability(params: WorldParams, shapes: List[Shape], victims: List[Victim]
                        ) -> Tuple[GridSpec, np.ndarray, str]:
    """Área explorable y alcanzabilidad de las víctimas; motivo del fallo o ''."""
    spawn_x, spawn_y, _ = params.robot_spawn
    spec = GridSpec.from_bounds(params.bounds, params.grid_resolution)
    clearance = clearance_map(spec, params.bounds, shapes)
    explorable = explorable_mask(spec, clearance, params.robot_radius, (spawn_x, spawn_y))
    if not explorable.any():
        return spec, explorable, 'el spawn no es transitable para el robot'
    xg, yg = spec.cell_centers()
    for victim in victims:
        dist = point_distance(victim.shape, xg, yg)
        victim.reach_distance = _r(dist[explorable].min())
        if victim.reach_distance > params.victim_reach_distance:
            return spec, explorable, (
                f'víctima {victim.idx} inalcanzable (celda alcanzable más cercana a '
                f'{victim.reach_distance:.2f} m)')
    return spec, explorable, ''


def _try_generate(params: WorldParams, attempt: int,
                  sub_seed: int) -> Tuple[Optional[GeneratedWorld], str]:
    """Un intento completo. Devuelve (mundo, '') o (None, motivo del fallo)."""
    rng = np.random.Generator(np.random.PCG64(sub_seed))
    spawn_x, spawn_y, _ = params.robot_spawn
    spawn_zone = Circle(spawn_x, spawn_y, params.spawn_clearance)
    placed: List[Shape] = []

    obstacles: List[Obstacle] = []
    for idx in range(params.n_obstacles):
        for _ in range(params.max_placement_tries):
            obstacle = _sample_obstacle(rng, params, idx)
            if _is_free(obstacle.shape, placed, params, spawn_zone):
                obstacles.append(obstacle)
                placed.append(obstacle.shape)
                break
        else:
            return None, (f'no se pudo colocar el escombro {idx} tras '
                          f'{params.max_placement_tries} intentos')

    victims: List[Victim] = []
    for idx in range(params.n_victims):
        for _ in range(params.max_placement_tries):
            victim = _sample_victim(rng, params, idx)
            if _is_free(victim.shape, placed, params, spawn_zone):
                victims.append(victim)
                placed.append(victim.shape)
                break
        else:
            return None, (f'no se pudo colocar la víctima {idx} tras '
                          f'{params.max_placement_tries} intentos')

    # Escombros bajos con un flujo aleatorio propio: con n_low_debris = 0 no se consume ningún
    # número aleatorio extra y el mundo es idéntico al de versiones anteriores. Si dejan alguna
    # víctima inalcanzable se vuelven a sortear solo ellos, sin cambiar el resto del escenario.
    reason = ''
    for debris_try in range(params.max_attempts if params.n_low_debris else 1):
        debris, reason = _place_low_debris(params, sub_seed, debris_try, placed, spawn_zone)
        if debris is None:
            continue
        spec, explorable, reason = _check_reachability(
            params, placed + [d.shape for d in debris], victims)
        if not reason:
            obstacles = obstacles + debris
            break
    else:
        return None, reason

    _assign_warm_debris(params, sub_seed, obstacles)
    world = GeneratedWorld(params, attempt, sub_seed, obstacles, victims, spec, explorable)
    return world, ''


def generate_world(params: WorldParams) -> GeneratedWorld:
    """
    Genera un escenario válido E = G(θ, s).

    Lanza GenerationError si ningún reintento satisface las restricciones.
    """
    params.validate()
    retries: List[Dict[str, object]] = []
    for attempt in range(params.max_attempts):
        sub_seed = derive_sub_seed(params.seed, attempt)
        world, reason = _try_generate(params, attempt, sub_seed)
        if world is not None:
            world.retries = retries
            return world
        LOGGER.warning('Intento %d (sub-semilla %d) descartado: %s', attempt, sub_seed, reason)
        retries.append({'attempt': attempt, 'sub_seed': sub_seed, 'reason': reason})
    raise GenerationError(
        f'Semilla {params.seed}: ningún escenario válido en {params.max_attempts} intentos. '
        f'Último motivo: {retries[-1]["reason"]}')
