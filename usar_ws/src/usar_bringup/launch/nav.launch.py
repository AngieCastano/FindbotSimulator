"""
Navegación (Capa 3): simulación + slam_toolbox (online async) + Nav2 + nube de profundidad.

Uso:
    ros2 launch usar_bringup nav.launch.py seed:=42
    ros2 launch usar_bringup nav.launch.py seed:=42 gui:=false rviz:=false   # sin ventanas

Cadena de velocidad: controller/behavior → /cmd_vel_nav → velocity_smoother → /cmd_vel.
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
              'velocity_smoother', 'bt_navigator']
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
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
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
            'robot_params': config('robot_params'),
            'rviz': 'false',  # este launch abre su propio RViz con mapa y costmaps
        }.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': config('slam_params'),
                          'use_sim_time': 'true'}.items())

    rviz = [Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', config('rviz_config').perform(context)],
        parameters=[SIM_TIME])] if open_rviz else []

    nav2 = _nav2_nodes(config('nav2_params').perform(context), config('bt_xml').perform(context))
    return [sim, slam, _depth_cloud_container(config('depth_params').perform(context)),
            *nav2, *rviz]


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
        DeclareLaunchArgument('robot_params', default_value=os.path.join(
            description_share, 'config', 'robot.yaml')),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(cfg, 'slam_toolbox.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(cfg, 'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            bringup_share, 'behavior_trees', 'navigate_to_pose_no_backup.xml'),
            description='Árbol de comportamiento de NavigateToPose'),
        DeclareLaunchArgument('depth_params', default_value=os.path.join(cfg, 'depth_cloud.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            bringup_share, 'rviz', 'nav.rviz')),
        OpaqueFunction(function=_launch_setup),
    ])
