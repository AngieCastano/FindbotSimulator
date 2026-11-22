"""
Navegación (Capa 3): simulación, EKF, slam_toolbox (online async), Nav2 y nube de profundidad.

El EKF (robot_localization) fusiona la odometría de las ruedas con la IMU.

Uso:
    ros2 launch usar_bringup nav.launch.py seed:=42
    ros2 launch usar_bringup nav.launch.py seed:=42 gui:=false rviz:=false   # sin ventanas

Cadena de velocidad (los dos filtros de colisión usan LiDAR + nube de profundidad):
    controller/behavior → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
        → collision_monitor → /cmd_vel
    teleop (scripts/teleop.sh) → /cmd_vel_teleop → collision_monitor_teleop → /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server',
              'velocity_smoother', 'collision_monitor', 'collision_monitor_teleop',
              'bt_navigator']
SIM_TIME = {'use_sim_time': True}


def _depth_cloud_container(params_file: str) -> ComposableNodeContainer:
    """Diezma /camera/depth_image y lo convierte en /camera/depth/points (frame óptico)."""
    with open(params_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    return ComposableNodeContainer(
        name='depth_cloud_container', namespace='', package='rclcpp_components',
        executable='component_container', output='screen', parameters=[SIM_TIME],
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc', plugin='image_proc::CropDecimateNode',
                name='depth_decimate', parameters=[params['depth_decimate'], SIM_TIME],
                remappings=[('in/image_raw', '/camera/depth_image'),
                            ('in/camera_info', '/camera/camera_info'),
                            ('out/image_raw', '/camera/depth/decimated'),
                            ('out/camera_info', '/camera/depth/camera_info')],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzNode',
                name='depth_to_cloud', parameters=[params['depth_to_cloud'], SIM_TIME],
                remappings=[('image_rect', '/camera/depth/decimated'),
                            ('camera_info', '/camera/depth/camera_info'),
                            ('points', '/camera/depth/points')],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ])


def _nav2_nodes(params_file: str, bt_xml: str):
    """Servidores de Nav2 estrictamente necesarios + su gestor de ciclo de vida."""
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    params = [params_file, SIM_TIME]
    return [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=params, remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=params, remappings=to_nav),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=params,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='usar_bringup', executable='costmap_obstacle_cloud',
             name='costmap_obstacle_cloud', output='screen', parameters=params),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=params),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor_teleop', output='screen', parameters=params),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[*params, {'default_nav_to_pose_bt_xml': bt_xml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[params_file, SIM_TIME, {'node_names': NAV2_NODES}]),
    ]


def _launch_setup(context):
    bringup_share = get_package_share_directory('usar_bringup')
    config = LaunchConfiguration
    # Se evalúa antes del include: sus launch_arguments (rviz:=false) sobrescriben la
    # configuración del launch padre.
    open_rviz = config('rviz').perform(context).lower() == 'true'

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'seed': config('seed'), 'worlds_dir': config('worlds_dir'),
            'world_params': config('world_params'), 'gui': config('gui'),
            'robot_params': config('robot_params'), 'use_thermal': config('use_thermal'),
            'use_co2': config('use_co2'), 'use_sound': config('use_sound'),
            'use_radar': config('use_radar'), 'firmware_sil': config('firmware_sil'),
            'comm': config('comm'),
            'rviz': 'false',  # este launch abre su propio RViz con mapa y costmaps
        }.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': config('slam_params'),
                          'use_sim_time': 'true',
                          # Lo activa lifecycle_manager_slam: el arranque automático del launch de
                          # slam_toolbox falla de forma intermitente (se queda en "Configuring" y
                          # nunca publica map→odom, lo que aborta el arranque de Nav2).
                          'use_lifecycle_manager': 'true'}.items())
    slam_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        parameters=[config('nav2_params').perform(context), SIM_TIME])

    # EKF: /odom (ruedas) + /imu/data_raw (giróscopo) → /odometry/filtered y TF odom → base.
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen', parameters=[config('ekf_params').perform(context), SIM_TIME])

    rviz = [Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', config('rviz_config').perform(context)],
        parameters=[SIM_TIME])] if open_rviz else []

    nav2 = _nav2_nodes(config('nav2_params').perform(context), config('bt_xml').perform(context))
    depth = _depth_cloud_container(config('depth_params').perform(context))
    return [sim, ekf, slam, slam_manager, depth, *nav2, *rviz]


def generate_launch_description():
    bringup_share = get_package_share_directory('usar_bringup')
    description_share = get_package_share_directory('usar_description')
    cfg = os.path.join(bringup_share, 'config')
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='42',
                              description='Semilla del escenario procedural'),
        DeclareLaunchArgument('worlds_dir', default_value='worlds'),
        DeclareLaunchArgument('world_params', default_value=''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz gráfica de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('use_thermal', default_value='false',
                              description='Cámara térmica MLX90640 (fase 2)'),
        DeclareLaunchArgument('use_co2', default_value='false',
                              description='Sensor de CO₂ TGS4161 (fase 2)'),
        DeclareLaunchArgument('use_sound', default_value='false',
                              description='Arreglo de micrófonos ReSpeaker (fase 2)'),
        DeclareLaunchArgument('use_radar', default_value='false',
                              description='Radar Doppler HB100 (fase 2)'),
        DeclareLaunchArgument('firmware_sil', default_value='false',
                              description='Firmware del ESP32 en el lazo (PID y odometría del '
                                          'firmware en vez de DiffDrive)'),
        DeclareLaunchArgument('comm', default_value='false',
                              description='Modelo de comunicación WiFi robot ↔ estación '
                                          'remota (pérdidas por distancia y escombros)'),
        DeclareLaunchArgument('robot_params', default_value=os.path.join(
            description_share, 'config', 'robot.yaml')),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(cfg, 'slam_toolbox.yaml')),
        DeclareLaunchArgument('ekf_params', default_value=os.path.join(cfg, 'ekf.yaml'),
                              description='Fusión de odometría y IMU (robot_localization)'),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(cfg, 'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            bringup_share, 'behavior_trees', 'navigate_to_pose_no_backup.xml'),
            description='Árbol de comportamiento de NavigateToPose'),
        DeclareLaunchArgument('depth_params', default_value=os.path.join(cfg, 'depth_cloud.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            bringup_share, 'rviz', 'nav.rviz')),
        OpaqueFunction(function=_launch_setup),
    ])
