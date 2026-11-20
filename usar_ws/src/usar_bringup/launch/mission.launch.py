"""
Misión completa con evaluación (Fase 6): un solo comando de punta a punta.

exploration.launch.py (simulación, SLAM, Nav2, explorador, detector) + evaluator (usar_metrics).

Uso:
    ros2 launch usar_bringup mission.launch.py seed:=42
    ros2 launch usar_bringup mission.launch.py seed:=7 headless:=true     # sin Gazebo ni RViz
    ros2 launch usar_bringup mission.launch.py seed:=42 quick:=true       # prueba de 5 min
    ros2 launch usar_bringup mission.launch.py seed:=42 world_params:=dusty  # víctimas grises

Si worlds/seed<N>/ no existe, se genera. La misión termina sola: cuando el explorador acaba
(sin fronteras o por tiempo), el evaluador escribe en results_dir
seed<N>_<fecha>.json / .csv / _map.pgm / _map.yaml / _comparison.png y se cierra todo el launch.
El evaluador es el único proceso que lee el ground truth.
"""

import os
from pathlib import Path
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from usar_worldgen.worlds import ensure_world, resolve_world_params

FORWARDED_ARGS = ['seed', 'worlds_dir', 'world_params', 'gui', 'rviz', 'quick', 'detection',
                  'results_dir', 'use_thermal']


def _ensure_ground_truth(seed: int, worlds_dir: str, world_params: str) -> str:
    """Ruta del ground truth de la semilla (y variante); genera el mundo si falta."""
    return os.path.join(ensure_world(seed, worlds_dir, world_params), 'ground_truth.yaml')


def _interrupt_mission(context):
    """
    Cierra la misión como un Ctrl+C: SIGINT a todo el grupo de procesos del launch.

    El evento Shutdown de launch solo señala a los procesos hijos directos, y el servidor de
    Gazebo (nieto, bajo el envoltorio `gz sim` en Ruby) quedaba vivo. Con SIGINT al grupo, cada
    proceso recibe la señal igual que al pulsar Ctrl+C en la terminal.
    """
    os.killpg(os.getpgrp(), signal.SIGINT)
    return []


def _result_prefix(seed: int, context) -> str:
    """seed<N> o seed<N>_<variante> (p. ej. seed42_dusty) para los archivos de resultados."""
    _, tag = resolve_world_params(LaunchConfiguration('world_params').perform(context))
    return f'seed{seed}' + (f'_{tag}' if tag else '')


def generate_launch_description():
    bringup_share = get_package_share_directory('usar_bringup')
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'exploration.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in FORWARDED_ARGS}.items())

    def evaluator(context):
        """Evaluador; al terminar (resultados escritos) cierra la misión."""
        seed = int(LaunchConfiguration('seed').perform(context))
        worlds_dir = os.path.abspath(os.path.expanduser(
            LaunchConfiguration('worlds_dir').perform(context)))
        gt_file = _ensure_ground_truth(seed, worlds_dir,
                                       LaunchConfiguration('world_params').perform(context))
        node = Node(package='usar_metrics', executable='evaluator', name='evaluator',
                    output='screen',
                    parameters=[LaunchConfiguration('evaluator_params').perform(context),
                                {'use_sim_time': True,
                                 'ground_truth_file': gt_file,
                                 'results_dir': LaunchConfiguration('results_dir')
                                 .perform(context),
                                 'result_prefix': _result_prefix(seed, context)}])
        shutdown = RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            LogInfo(msg='[mission] Evaluación terminada: cerrando la misión'),
            OpaqueFunction(function=_interrupt_mission)]))
        return [node, shutdown]

    headless = LaunchConfiguration('headless')
    windows = PythonExpression(["'false' if '", headless, "'.lower() == 'true' else 'true'"])
    # <ws>/install/usar_bringup/share/usar_bringup → <repo>/results (junto a usar_ws)
    default_results = str(Path(bringup_share).resolve().parents[3].parent / 'results')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42',
                              description='Semilla del escenario procedural'),
        DeclareLaunchArgument('worlds_dir', default_value='worlds'),
        DeclareLaunchArgument('world_params', default_value='',
                              description='Variante del escenario: vacío (por defecto), un '
                              'nombre (dusty) o la ruta a un YAML de parámetros θ'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Sin Gazebo GUI ni RViz (para lotes de semillas)'),
        DeclareLaunchArgument('gui', default_value=windows,
                              description='Abrir la interfaz gráfica de Gazebo'),
        DeclareLaunchArgument('rviz', default_value=windows, description='Abrir RViz'),
        DeclareLaunchArgument('use_thermal', default_value='false',
                              description='Cámara térmica MLX90640 y detector de calor (fase 2)'),
        DeclareLaunchArgument('quick', default_value='false',
                              description='Prueba corta (5 min de exploración)'),
        DeclareLaunchArgument('detection', default_value='true',
                              description='Ejecutar el detector de víctimas'),
        DeclareLaunchArgument('results_dir', default_value=default_results,
                              description='Carpeta de resultados'),
        DeclareLaunchArgument('evaluator_params', default_value=os.path.join(
            get_package_share_directory('usar_metrics'), 'config', 'evaluator.yaml')),
        exploration,
        OpaqueFunction(function=evaluator),
    ])
