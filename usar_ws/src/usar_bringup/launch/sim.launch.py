"""
Simulación (Capa 2): escenario procedural + Gazebo Harmonic + robot findbot + puente + RViz.

Uso:
    ros2 launch usar_bringup sim.launch.py seed:=42
    ros2 launch usar_bringup sim.launch.py seed:=7 gui:=false rviz:=false   # sin ventanas
    ros2 launch usar_bringup sim.launch.py use_thermal:=true   # con la cámara térmica (fase 2)
    ros2 launch usar_bringup sim.launch.py use_co2:=true       # con el sensor de CO₂ (fase 2)
    ros2 launch usar_bringup sim.launch.py use_sound:=true     # con los micrófonos (fase 2)

Si worlds/seed<N>/ no existe, se genera con usar_worldgen. El robot aparece en la pose de
spawn.yaml (archivo público). Este launch no lee ground_truth.yaml: solo pasa su ruta a los
simuladores de sensores de usar_sim_sensors (excepción del ground truth, ver CLAUDE.md).
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
from usar_worldgen.spawn import load_spawn, SPAWN_FILE
from usar_worldgen.worlds import ensure_world
import xacro

ROBOT_NAME = 'findbot'
# Altura de aparición sobre el suelo para evitar interpenetración inicial.
SPAWN_Z = 0.02


def _launch_setup(context):
    seed = int(LaunchConfiguration('seed').perform(context))
    worlds_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('worlds_dir').perform(context)))
    world_params = LaunchConfiguration('world_params').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    robot_params = LaunchConfiguration('robot_params').perform(context)
    use_thermal = LaunchConfiguration('use_thermal').perform(context).lower() == 'true'
    use_co2 = LaunchConfiguration('use_co2').perform(context).lower() == 'true'
    use_sound = LaunchConfiguration('use_sound').perform(context).lower() == 'true'
    # Pose real del robot (siempre en simulación): la leen los simuladores de gases/sonido/radar
    # y el evaluador (error de localización). Ningún nodo del robot la usa.
    sim_pose = True

    # worlds/seed<N>/ o, con world_params:=<nombre>, worlds/seed<N>_<nombre>/ (se genera si falta)
    world_dir = ensure_world(seed, worlds_dir, world_params)
    world_sdf = os.path.join(world_dir, 'world.sdf')
    spawn = load_spawn(os.path.join(world_dir, SPAWN_FILE))

    description_share = get_package_share_directory('usar_description')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'findbot.urdf.xacro'),
        mappings={'robot_params': robot_params,
                  'use_thermal': 'true' if use_thermal else 'false',
                  'use_co2': 'true' if use_co2 else 'false',
                  'use_sound': 'true' if use_sound else 'false',
                  'sim_pose': 'true' if sim_pose else 'false'}).toxml()

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

    # Cámara térmica (fase 2): puente de la imagen cruda y modelo de ruido del MLX90640.
    thermal = []
    if use_thermal:
        thermal = [
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='thermal_bridge', output='screen',
                 parameters=[{'config_file': os.path.join(description_share, 'config',
                                                          'bridge_thermal.yaml'),
                              'use_sim_time': True}]),
            Node(package='usar_sim_sensors', executable='thermal_camera_sim', output='screen',
                 parameters=[os.path.join(get_package_share_directory('usar_sim_sensors'),
                                          'config', 'thermal_camera_sim.yaml'),
                             {'use_sim_time': True}]),
        ]

    # Pose real y simuladores de la Capa 2 que la necesitan (excepción del ground truth).
    sim_sensors = []
    sim_sensors_share = get_package_share_directory('usar_sim_sensors')
    # Ruido de los encoders: siempre en simulación (DiffDrive solo da la odometría ideal).
    sim_sensors.append(Node(
        package='usar_sim_sensors', executable='odom_noise_sim', output='screen',
        parameters=[os.path.join(sim_sensors_share, 'config', 'odom_noise_sim.yaml'),
                    {'use_sim_time': True, 'seed': seed}]))
    if sim_pose:
        sim_sensors.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='sim_pose_bridge',
            output='screen',
            parameters=[{'config_file': os.path.join(description_share, 'config',
                                                     'bridge_sim_pose.yaml'),
                         'use_sim_time': True}]))
    if use_co2:
        sim_sensors.append(Node(
            package='usar_sim_sensors', executable='co2_sim', output='screen',
            parameters=[os.path.join(sim_sensors_share, 'config', 'co2_sim.yaml'),
                        {'use_sim_time': True,
                         'ground_truth_file': os.path.join(world_dir, 'ground_truth.yaml')}]))
    if use_sound:
        sim_sensors.append(Node(
            package='usar_sim_sensors', executable='sound_sim', output='screen',
            parameters=[os.path.join(sim_sensors_share, 'config', 'sound_sim.yaml'),
                        {'use_sim_time': True, 'seed': seed,
                         'ground_truth_file': os.path.join(world_dir, 'ground_truth.yaml')}]))

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        LogInfo(msg=f'[sim] Semilla {seed} | mundo: {world_sdf} | spawn: {spawn}'),
        gazebo, robot_state_publisher, spawn_robot, bridge, *thermal, *sim_sensors, rviz,
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
        DeclareLaunchArgument('use_thermal', default_value='false',
                              description='Cámara térmica MLX90640 (fase 2)'),
        DeclareLaunchArgument('use_co2', default_value='false',
                              description='Sensor de CO₂ TGS4161 (fase 2)'),
        DeclareLaunchArgument('use_sound', default_value='false',
                              description='Arreglo de micrófonos ReSpeaker (fase 2)'),
        OpaqueFunction(function=_launch_setup),
    ])
