"""
Simulación (Capa 2): escenario procedural + Gazebo Harmonic + robot findbot + puente + RViz.

Uso:
    ros2 launch usar_bringup sim.launch.py seed:=42
    ros2 launch usar_bringup sim.launch.py seed:=7 gui:=false rviz:=false   # sin ventanas

Si worlds/seed<N>/ no existe, se genera con usar_worldgen. El robot aparece en la pose de
spawn.yaml (archivo público); este launch NO lee ground_truth.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from usar_worldgen.cli import main as generate_world_cli
from usar_worldgen.spawn import load_spawn, SPAWN_FILE
import xacro

ROBOT_NAME = 'findbot'
# Altura de aparición sobre el suelo para evitar interpenetración inicial.
SPAWN_Z = 0.02


def _ensure_world(seed: int, worlds_dir: str, world_params: str) -> str:
    """Devuelve el directorio del mundo de la semilla; lo genera si falta."""
    world_dir = os.path.join(worlds_dir, f'seed{seed}')
    required = (os.path.join(world_dir, 'world.sdf'), os.path.join(world_dir, SPAWN_FILE))
    if not all(os.path.isfile(path) for path in required):
        args = ['--seed', str(seed), '--out', world_dir]
        if world_params:
            args += ['--params', world_params]
        if generate_world_cli(args) != 0:
            raise RuntimeError(f'No se pudo generar el mundo de la semilla {seed}')
    return world_dir


def _launch_setup(context):
    seed = int(LaunchConfiguration('seed').perform(context))
    worlds_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('worlds_dir').perform(context)))
    world_params = LaunchConfiguration('world_params').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    robot_params = LaunchConfiguration('robot_params').perform(context)

    world_dir = _ensure_world(seed, worlds_dir, world_params)
    world_sdf = os.path.join(world_dir, 'world.sdf')
    spawn = load_spawn(os.path.join(world_dir, SPAWN_FILE))

    description_share = get_package_share_directory('usar_description')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'findbot.urdf.xacro'),
        mappings={'robot_params': robot_params}).toxml()

    # -r: arrancar la simulación; sin GUI: solo servidor con renderizado headless (sensores).
    gz_args = f'-r -v 2 {world_sdf}' if gui else f'-r -s -v 2 --headless-rendering {world_sdf}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', ROBOT_NAME,
                   '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(SPAWN_Z),
                   '-Y', str(spawn['yaw'])])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(description_share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        LogInfo(msg=f'[sim] Semilla {seed} | mundo: {world_sdf} | spawn: {spawn}'),
        gazebo, robot_state_publisher, spawn_robot, bridge, rviz,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('usar_bringup')
    description_share = get_package_share_directory('usar_description')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42',
                              description='Semilla del escenario procedural'),
        DeclareLaunchArgument('worlds_dir', default_value='worlds',
                              description='Directorio de mundos generados (seed<N>/)'),
        DeclareLaunchArgument('world_params', default_value='',
                              description='YAML θ opcional para generar el mundo si no existe'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz gráfica de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(bringup_share, 'rviz', 'sim.rviz')),
        DeclareLaunchArgument('robot_params',
                              default_value=os.path.join(description_share, 'config',
                                                         'robot.yaml'),
                              description='Parámetros del robot (dimensiones y sensores)'),
        OpaqueFunction(function=_launch_setup),
    ])
