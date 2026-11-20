"""Exportación del escenario generado a un mundo SDF para Gazebo Harmonic."""

import math
from typing import List, Optional, Sequence, Tuple

from .generator import GeneratedWorld, Obstacle, Victim
from .geometry import Box

SDF_VERSION = '1.9'
KELVIN = 273.15

VICTIM_RGB = (1.0, 0.4, 0.0)  # naranja intenso
VICTIM_EMISSIVE = (0.25, 0.1, 0.0)
# Víctima "cubierta de polvo": gris parduzco dentro de la gama de los escombros (saturación
# baja), de modo que el color no la distingue.
VICTIM_DUSTY_RGB = (0.52, 0.49, 0.45)
VICTIM_MATERIALS = {
    'orange': (VICTIM_RGB, VICTIM_EMISSIVE),
    'dusty': (VICTIM_DUSTY_RGB, (0.0, 0.0, 0.0)),
}

# Partes del humanoide acostado en el marco de la víctima (eje x: pies → cabeza).
# (nombre, geometría, dimensiones, pose x y z roll pitch yaw)
VICTIM_PARTS = (
    ('head', 'sphere', (0.11,), (0.73, 0.0, 0.11, 0.0, 0.0, 0.0)),
    ('torso', 'box', (0.6, 0.36, 0.2), (0.32, 0.0, 0.1, 0.0, 0.0, 0.0)),
    ('leg_left', 'cylinder', (0.07, 0.85), (-0.415, 0.1, 0.07, 0.0, math.pi / 2, 0.0)),
    ('leg_right', 'cylinder', (0.07, 0.85), (-0.415, -0.1, 0.07, 0.0, math.pi / 2, 0.0)),
    ('arm_left', 'cylinder', (0.05, 0.6), (0.3, 0.25, 0.05, 0.0, math.pi / 2, 0.0)),
    ('arm_right', 'cylinder', (0.05, 0.6), (0.3, -0.25, 0.05, 0.0, math.pi / 2, 0.0)),
)


def _f(value: float) -> str:
    """Formato numérico fijo (sin cero negativo) para que el SDF sea reproducible."""
    text = f'{value:.4f}'
    return '0.0000' if text == '-0.0000' else text


def _vec(values: Sequence[float]) -> str:
    return ' '.join(_f(v) for v in values)


def _geometry(kind: str, dims: Sequence[float]) -> str:
    if kind == 'box':
        return f'<box><size>{_vec(dims)}</size></box>'
    if kind == 'cylinder':
        return (f'<cylinder><radius>{_f(dims[0])}</radius>'
                f'<length>{_f(dims[1])}</length></cylinder>')
    if kind == 'sphere':
        return f'<sphere><radius>{_f(dims[0])}</radius></sphere>'
    raise ValueError(f'geometría desconocida: {kind}')


def _material(rgb: Tuple[float, float, float],
              emissive: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
    return (f'<material><ambient>{_vec(rgb)} 1</ambient><diffuse>{_vec(rgb)} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular><emissive>{_vec(emissive)} 1</emissive>'
            f'</material>')


def _static_model(name: str, pose: Sequence[float], parts: List[str]) -> str:
    body = '\n'.join(parts)
    return (f'    <model name="{name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{_vec(pose)}</pose>\n'
            f'      <link name="link">\n{body}\n'
            f'      </link>\n'
            f'    </model>')


def _thermal(temperature_c: Optional[float]) -> str:
    """Componente térmico de Gazebo (temperatura propia del visual, en kelvin)."""
    if temperature_c is None:
        return ''
    return ('<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">'
            f'<temperature>{_f(temperature_c + KELVIN)}</temperature></plugin>')


def _part(name: str, kind: str, dims: Sequence[float], pose: Sequence[float],
          material: str, temperature_c: Optional[float] = None) -> str:
    geom = _geometry(kind, dims)
    return (f'        <collision name="{name}_collision"><pose>{_vec(pose)}</pose>'
            f'<geometry>{geom}</geometry></collision>\n'
            f'        <visual name="{name}_visual"><pose>{_vec(pose)}</pose>'
            f'<geometry>{geom}</geometry>{material}{_thermal(temperature_c)}</visual>')


def _walls(world: GeneratedWorld) -> List[str]:
    x_min, x_max, y_min, y_max = world.params.bounds
    t, h = world.params.wall_thickness, world.params.wall_height
    width, length = x_max - x_min, y_max - y_min
    cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
    walls = (
        ('wall_north', (cx, y_max + t / 2), (width + 2 * t, t, h)),
        ('wall_south', (cx, y_min - t / 2), (width + 2 * t, t, h)),
        ('wall_east', (x_max + t / 2, cy), (t, length, h)),
        ('wall_west', (x_min - t / 2, cy), (t, length, h)),
    )
    material = _material((0.7, 0.7, 0.68))
    return [_static_model(name, (x, y, h / 2, 0, 0, 0),
                          [_part('wall', 'box', size, (0, 0, 0, 0, 0, 0), material)])
            for name, (x, y), size in walls]


def _obstacle_model(obstacle: Obstacle) -> str:
    shape = obstacle.shape
    if isinstance(shape, Box):
        dims = (shape.size_x, shape.size_y, obstacle.height)
    else:
        dims = (shape.radius, obstacle.height)
    part = _part('rubble', obstacle.kind, dims, (0, 0, 0, 0, 0, 0), _material(obstacle.color),
                 obstacle.temperature_c)
    pose = (shape.x, shape.y, obstacle.height / 2, 0, 0, obstacle.yaw)
    return _static_model(f'obstacle_{obstacle.idx:03d}', pose, [part])


def _victim_model(victim: Victim, appearance: str, temperature_c: float) -> str:
    material = _material(*VICTIM_MATERIALS[appearance])
    parts = [_part(name, kind, dims, pose, material, temperature_c)
             for name, kind, dims, pose in VICTIM_PARTS]
    pose = (victim.shape.x, victim.shape.y, 0, 0, 0, victim.shape.yaw)
    return _static_model(f'victim_{victim.idx:02d}', pose, parts)


def world_to_sdf(world: GeneratedWorld) -> str:
    """Texto SDF completo del escenario."""
    params = world.params
    width, length = params.size
    ground_size = _vec((width + 10, length + 10))
    models = [*_walls(world), *(_obstacle_model(o) for o in world.obstacles),
              *(_victim_model(v, params.victim_appearance, params.victim_temperature_c)
                for v in world.victims)]
    models_text = '\n'.join(models)
    return f"""<?xml version="1.0" ?>
<!-- Generado por usar_worldgen: semilla {params.seed}, intento {world.attempt},
     sub-semilla {world.sub_seed}. No editar a mano: regenerar con la misma semilla. -->
<sdf version="{SDF_VERSION}">
  <world name="usar_world">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Temperatura ambiente (cámara térmica): suelo, muros y escombros sin temperatura
         propia. Sin gradiente vertical. -->
    <atmosphere type="adiabatic">
      <temperature>{_f(params.ambient_temperature_c + KELVIN)}</temperature>
      <temperature_gradient>0.0</temperature_gradient>
    </atmosphere>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size}</size></plane></geometry>
          {_material((0.6, 0.58, 0.55))}
        </visual>
      </link>
    </model>

{models_text}
  </world>
</sdf>
"""


def write_world_sdf(world: GeneratedWorld, path: str) -> None:
    """Escribe el SDF del escenario en path."""
    with open(path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write(world_to_sdf(world))
