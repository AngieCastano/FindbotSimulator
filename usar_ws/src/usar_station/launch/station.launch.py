"""
Estación remota de teleoperación y enlace robot ↔ estación.

Lanza (lo incluye exploration.launch.py con teleop:=true):
  - comm_relay: el enlace de datos (con comm:=true, retrasos y pérdidas según /comm/link),
  - link_failsafe: a bordo del robot, parada segura si se pierde el latido de la estación,
  - sim_operator: latido, estado de la misión y, con operator:=simulated, metas de barrido,
  - RViz de la estación (solo lo que llega por el enlace, /station/*), si rviz:=true.

Conducir con el teclado desde la estación (otra terminal): scripts/station_teleop.sh
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STATION_TF = [('/tf', '/station/tf'), ('/tf_static', '/station/tf_static')]


def _setup(context):
    share = get_package_share_directory('usar_station')
    cfg = os.path.join(share, 'config')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    log_dir = arg('log_dir')
    prefix = arg('log_prefix')

    def log(name):
        return os.path.join(log_dir, f'{prefix}_{name}.csv') if log_dir else ''

    size = [float(v) for v in arg('area_size').split(',')]
    nodes = [
        Node(package='usar_station', executable='comm_relay', output='screen',
             parameters=[{'use_sim_time': True, 'links_file': os.path.join(cfg, 'link.yaml'),
                          'log_csv': log('link'), 'seed': int(arg('seed'))}]),
        Node(package='usar_station', executable='link_failsafe', output='screen',
             parameters=[os.path.join(cfg, 'failsafe.yaml'),
                         {'use_sim_time': True, 'log_csv': log('heartbeat')}]),
        Node(package='usar_station', executable='sim_operator', output='screen',
             remappings=STATION_TF,
             parameters=[os.path.join(cfg, 'operator.yaml'),
                         {'use_sim_time': True, 'mode': arg('operator'), 'area_size': size,
                          'log_csv': log('station')}]),
    ]
    if arg('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='station_rviz', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'station.rviz')],
                          remappings=STATION_TF, parameters=[{'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('operator', default_value='human',
                              description='human (teclado / metas en RViz) o simulated'),
        DeclareLaunchArgument('area_size', default_value='20.0,20.0',
                              description='Tamaño del área "ancho,largo" (m), dato del sitio'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz de la estación'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='Carpeta de los registros del enlace ("" = sin)'),
        DeclareLaunchArgument('log_prefix', default_value='station'),
        OpaqueFunction(function=_setup),
    ])
