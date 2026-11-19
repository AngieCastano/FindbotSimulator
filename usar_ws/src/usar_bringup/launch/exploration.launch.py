"""
Exploración autónoma (Capa 4a): navegación (nav.launch.py) + frontier_explorer.

Uso:
    ros2 launch usar_bringup exploration.launch.py seed:=42
    ros2 launch usar_bringup exploration.launch.py seed:=42 gui:=false rviz:=false
    ros2 launch usar_bringup exploration.launch.py seed:=42 quick:=true   # prueba de 5 min

El robot explora solo y se detiene cuando no quedan fronteras o se agota max_time
(estado en /exploration/status; fronteras en /exploration/frontiers).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FORWARDED_ARGS = ['seed', 'worlds_dir', 'world_params', 'gui', 'rviz', 'rviz_config']


def generate_launch_description():
    bringup_share = get_package_share_directory('usar_bringup')
    exploration_share = get_package_share_directory('usar_exploration')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'nav.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in FORWARDED_ARGS}.items())
    exploration_config = os.path.join(exploration_share, 'config')

    def explorer(context):
        """Nodo explorador; con quick:=true añade encima los parámetros de la prueba corta."""
        params = [LaunchConfiguration('explorer_params').perform(context)]
        if LaunchConfiguration('quick').perform(context).lower() == 'true':
            params.append(os.path.join(exploration_config, 'explorer_quick.yaml'))
        return [Node(package='usar_exploration', executable='frontier_explorer',
                     name='frontier_explorer', output='screen',
                     parameters=[*params, {'use_sim_time': True}])]
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42',
                              description='Semilla del escenario procedural'),
        DeclareLaunchArgument('worlds_dir', default_value='worlds'),
        DeclareLaunchArgument('world_params', default_value=''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz gráfica de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            bringup_share, 'rviz', 'exploration.rviz')),
        DeclareLaunchArgument('explorer_params', default_value=os.path.join(
            exploration_config, 'explorer.yaml'),
            description='Parámetros del explorador (α, β, umbrales, tiempo máximo)'),
        DeclareLaunchArgument('quick', default_value='false',
                              description='Prueba corta (explorer_quick.yaml: 5 min)'),
        nav,
        OpaqueFunction(function=explorer),
    ])
