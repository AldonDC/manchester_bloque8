"""
bug0_launch — Sistema completo Bug 0 (Task 1)
==============================================
Mini Challenge · Reactive Navigation · Week 6

Levanta:
    1) Gazebo Harmonic + Puzzlebot con LiDAR  (vía bringup_launch.py)
    2) lidar_processor   — sectores frontal/izq/der desde /scan
    3) localisation      — dead reckoning + Σ_k del Reto 4
    4) bug0              — máquina de estados GO_TO_GOAL ↔ FOLLOW_WALL
    5) goal_publisher    — publica el objetivo (x_T, y_T)
    6) rviz2             — visualización

Argumentos:
    world       mundo Gazebo a cargar     (default 'bug_easy.world')
    target_x    objetivo X [m]             (default 3.0)
    target_y    objetivo Y [m]             (default 0.0)
    x, y, yaw   pose inicial del robot     (default 0,0,0)

Uso:
    ros2 launch puzzlebot_bug_w6 bug0_launch.py
    ros2 launch puzzlebot_bug_w6 bug0_launch.py world:=bug_medium.world target_x:=4.0

Nota técnica:
    Los nodos de navegación arrancan con un delay de 5 s mediante TimerAction
    para que el bridge ros_gz haya publicado /clock antes de que ellos creen
    sus subscripciones con use_sim_time=true. Sin este delay, los timestamps
    son incoherentes y los Message Filters de RViz/Bug0 descartan los mensajes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'puzzlebot_bug_w6'

    # ── Argumentos ───────────────────────────────────────────────────────────
    declare_world = DeclareLaunchArgument('world',    default_value='bug_easy.world')
    declare_tx    = DeclareLaunchArgument('target_x', default_value='3.0')
    declare_ty    = DeclareLaunchArgument('target_y', default_value='0.0')
    declare_x     = DeclareLaunchArgument('x',        default_value='0.0')
    declare_y     = DeclareLaunchArgument('y',        default_value='0.0')
    declare_yaw   = DeclareLaunchArgument('yaw',      default_value='0.0')

    # ── Bringup (Gazebo + robot + bridges) ───────────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'yaw':   LaunchConfiguration('yaw'),
        }.items(),
    )

    # ── Stack ROS arriba del simulador ───────────────────────────────────────
    common_params = [{'use_sim_time': True}]

    lidar = Node(package=pkg, executable='lidar_processor',
                 name='lidar_processor', output='screen',
                 parameters=common_params)

    # Convierte /joint_states (de Gazebo) en /wr y /wl para localisation.
    wheels = Node(package=pkg, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen',
                  parameters=common_params)

    loc = Node(package=pkg, executable='localisation',
               name='localisation', output='screen',
               parameters=common_params + [{
                   'x_init':     LaunchConfiguration('x'),
                   'y_init':     LaunchConfiguration('y'),
                   'theta_init': LaunchConfiguration('yaw'),
               }])

    coord_tf = Node(package=pkg, executable='coordinate_transform',
                    name='coordinate_transform', output='screen',
                    parameters=common_params)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common_params,
    )

    goal = Node(package=pkg, executable='goal_publisher',
                name='goal_publisher', output='screen',
                parameters=common_params + [{
                    'target_x': LaunchConfiguration('target_x'),
                    'target_y': LaunchConfiguration('target_y'),
                }])

    bug = Node(package=pkg, executable='bug0',
               name='bug0', output='screen',
               parameters=common_params + [{
                   'target_x': LaunchConfiguration('target_x'),
                   'target_y': LaunchConfiguration('target_y'),
               }])

    # Controller PID del Mini Challenge 2 — recibe /set_point del bug y
    # publica /cmd_vel suavizado con PID + estrategia "alinear primero".
    # Sin este nodo el bug no podría moverse: él sólo emite waypoints.
    controller = Node(package=pkg, executable='controller',
                      name='controller', output='screen',
                      parameters=common_params + [{
                          # Tolerancia chica para que persiga sub-waypoints
                          'dist_tolerance': 0.10,
                      }])

    rviz_cfg = os.path.join(get_package_share_directory(pkg),
                            'rviz', 'puzzlebot_bug.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen',
                parameters=common_params)

    # ── Retrasamos el stack de navegación 5 s ────────────────────────────────
    # Permite que Gazebo + bridge ya estén publicando /clock y /scan antes de
    # que los nodos de navegación creen sus subscripciones con use_sim_time.
    delayed_nav = TimerAction(
        period=8.0,
        actions=[static_tf, lidar, wheels, loc, coord_tf, goal, controller, bug, rviz],
    )

    return LaunchDescription([
        declare_world, declare_tx, declare_ty, declare_x, declare_y, declare_yaw,
        bringup,
        delayed_nav,
    ])
