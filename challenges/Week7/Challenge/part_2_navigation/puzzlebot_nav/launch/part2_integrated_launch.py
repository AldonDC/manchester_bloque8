"""
part2_integrated_launch — Part 2 + Opcional A + Opcional B (INTEGRACIÓN FULL)

Implementa la arquitectura del PDF Final Challenge MCR2:

    [Goals/Waypoints] → [Multi-point Navigation] ──→ /pre_cmd_vel
                                                         ↓
                                                  [Obstacle Avoidance] → /cmd_vel
                                                         ↑
    [LiDAR /scan] ────────────────────────────────────────┘

    [Camera /camera/image_raw] → [ArUco Detection] → /aruco/observations
                                                              ↓
    [Encoders /wr /wl] ───────────────────────────→ [EKF] → /ekf/odom
                                                              ↑          ↓
                                                       [ArUco Map] ALIMENTA NAV
                                                                     ↓
                                              goto_goal usa /ekf/odom (pose corregida)
                                              en vez de /odom (encoders crudos)

CUMPLE:
    Part 2 obligatoria  : 4 waypoints + controlador propio + FSM + cerrada
    Opcional Sección A  : Bug 0/1/2 disponibles vía launch arg `bug`
    Opcional Sección B  : EKF Visual + ArUcos durante navegación

USO:
    ros2 launch puzzlebot_nav part2_integrated_launch.py
        bug:=goto_goal              # default: FSM nueva con EKF
        bug:=bug2                   # opcional A puro: Bug 2 reactivo
        use_ekf_pose:=true|false    # default true (integración completa)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction,
                             SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = 'puzzlebot_nav'
    ekf_pkg = 'puzzlebot_ekf'

    nav_share = get_package_share_directory(nav_pkg)
    ekf_share = get_package_share_directory(ekf_pkg)

    waypoints_yaml = os.path.join(nav_share, 'config', 'waypoints_maze.yaml')
    ekf_params_yaml = os.path.join(ekf_share, 'config', 'ekf_params.yaml')
    aruco_map_yaml = os.path.join(ekf_share, 'config', 'aruco_map.yaml')

    no_user_site = SetEnvironmentVariable(name='PYTHONNOUSERSITE', value='1')

    # ── Launch arguments ──────────────────────────────────────────────────────
    # SPAWN topológicamente correcto: (0.9, 1.7) yaw=π/2.
    # Análisis: el spawn de Part 1 era (0.9, 0.8), pero ese punto está
    # DENTRO del chamber iv2 (x∈[0.5,1.8], y∈[-0.3,1.3]). Para llegar
    # a cualquier waypoint el robot tendría que cruzar la pared iv2_east
    # (sólida en x=1.8) — imposible. Mover el spawn 0.9m al norte (y=1.7)
    # lo deja en el corredor central libre (entre el techo abierto de iv2
    # y el divisor ih2 que está en y=2.0). Spawn libre de muros a 0.4m de
    # cada obstáculo cercano. EKF de Part 1 funciona desde cualquier
    # spawn porque tiene corrección visual por ArUcos.
    declare_x = DeclareLaunchArgument('x',   default_value='0.9')
    declare_y = DeclareLaunchArgument('y',   default_value='1.7')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5708')
    declare_bug = DeclareLaunchArgument(
        'bug', default_value='goto_goal',
        description="Nav algorithm: 'goto_goal' (FSM), 'bug0', 'bug1', 'bug2'")
    declare_world = DeclareLaunchArgument(
        'world', default_value='ekf_arena.world',
        description='Arena with 4 ArUco markers (Part 1)')
    declare_use_ekf_pose = DeclareLaunchArgument(
        'use_ekf_pose', default_value='true',
        description='If true, nav uses /ekf/odom; if false, uses raw /odom')

    # ── Gazebo + Puzzlebot ────────────────────────────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    common = [{'use_sim_time': True}]

    # ── PERCEPCIÓN: LiDAR + ruedas + ArUco (P1) ───────────────────────────────
    wheels = Node(package=nav_pkg, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen', parameters=common)

    lidar = Node(package=nav_pkg, executable='lidar_processor',
                 name='lidar_processor', output='screen', parameters=common)

    # ArUco detector de Part 1 (Sección B integración).
    aruco = Node(package=ekf_pkg, executable='aruco_detector',
                 name='aruco_detector', output='screen',
                 parameters=common + [ekf_params_yaml])

    # ── LOCALIZACIÓN: dual stack (encoders + EKF visual) ──────────────────────
    # 1) Localisation simple de Part 2 (publica /odom desde encoders).
    #    Necesaria como fallback si use_ekf_pose=false.
    def _make_loc(context):
        return [Node(
            package=nav_pkg, executable='localisation', name='localisation',
            output='screen',
            parameters=common + [{
                'x_init':     float(LaunchConfiguration('x').perform(context)),
                'y_init':     float(LaunchConfiguration('y').perform(context)),
                'theta_init': float(LaunchConfiguration('yaw').perform(context)),
            }],
        )]
    loc = OpaqueFunction(function=_make_loc)

    # 2) EKF Visual de Part 1 (publica /ekf/odom con corrección por ArUcos).
    #    Recibe spawn pose para inicializar x_init/y_init/theta_init.
    def _make_ekf_node(context):
        x = LaunchConfiguration('x').perform(context)
        y = LaunchConfiguration('y').perform(context)
        yaw = LaunchConfiguration('yaw').perform(context)
        report_dir = os.path.dirname(os.path.dirname(__file__))
        init_yaml = os.path.join('/tmp', 'ekf_init_part2_integrated.yaml')
        # Override mínimo: solo los 3 valores de inicialización.
        with open(init_yaml, 'w') as f:
            f.write(
                'ekf_node:\n'
                '  ros__parameters:\n'
                f'    x_init: {float(x)}\n'
                f'    y_init: {float(y)}\n'
                f'    theta_init: {float(yaw)}\n'
                f"    aruco_map_yaml: '{aruco_map_yaml}'\n"
                f"    report_dir: '{report_dir}'\n"
            )
        return [Node(
            package=ekf_pkg, executable='ekf_node',
            name='ekf_node', output='screen',
            parameters=common + [ekf_params_yaml, init_yaml],
        )]
    ekf = OpaqueFunction(function=_make_ekf_node)

    # 3) Publicador de elipse de covarianza para RViz (mismo de Part 1).
    cov = Node(package=ekf_pkg, executable='covariance_publisher',
               name='covariance_publisher', output='screen',
               parameters=common)

    coord_tf = Node(package=nav_pkg, executable='coordinate_transform',
                    name='coordinate_transform', output='screen',
                    parameters=common)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common,
    )

    # ── PLANIFICACIÓN: waypoint manager ───────────────────────────────────────
    # En modo integrated, el waypoint_manager también usa pose corregida
    # /ekf/odom para detección de arrival por proximidad (independiente
    # del bug). Esto resuelve casos edge donde el bug llega al WP pero
    # no publica GOAL_REACHED correctamente.
    def _make_wpm(context):
        use_ekf = LaunchConfiguration('use_ekf_pose').perform(context).lower() == 'true'
        remaps = [('odom', 'ekf/odom')] if use_ekf else []
        return [Node(package=nav_pkg, executable='waypoint_manager',
                     name='waypoint_manager', output='screen',
                     parameters=common + [waypoints_yaml],
                     remappings=remaps)]
    wpm = OpaqueFunction(function=_make_wpm)

    # ── NAVEGACIÓN: bug o goto_goal con fuente de odometría configurable ──────
    # use_ekf_pose=true → nav usa /ekf/odom (Sección B: pose corregida por
    # ArUcos). use_ekf_pose=false → /odom puro (encoders).
    def _make_bug(context):
        use_ekf = LaunchConfiguration('use_ekf_pose').perform(context).lower() == 'true'
        bug_exec = LaunchConfiguration('bug').perform(context)
        # Topic remapping: el nodo se suscribe a 'odom' internamente, lo
        # remapeamos a /ekf/odom si use_ekf_pose=true.
        remaps = []
        if use_ekf:
            remaps = [('odom', 'ekf/odom')]
        return [Node(
            package=nav_pkg, executable=bug_exec,
            name='bug', output='screen',
            parameters=common,
            remappings=remaps,
        )]
    bug = OpaqueFunction(function=_make_bug)

    # ── Controller PID externo (solo para bug0/bug1/bug2) ─────────────────────
    def _make_controller(context):
        bug_choice = LaunchConfiguration('bug').perform(context)
        if bug_choice == 'goto_goal':
            return []
        return [Node(package=nav_pkg, executable='controller',
                     name='controller', output='screen',
                     parameters=common + [{'dist_tolerance': 0.15}])]
    controller = OpaqueFunction(function=_make_controller)

    # ── VISUALIZACIÓN: RViz con TODO ──────────────────────────────────────────
    # Mundo (muros del laberinto) + banderas (waypoints) + elipses + ArUcos.
    world_viz = Node(package=ekf_pkg, executable='world_visualizer',
                     name='world_visualizer', output='screen',
                     parameters=common)

    # Usamos el RViz de PART 1 (ekf.rviz) que tiene:
    #   - Camera (ArUco overlay) → ves la cámara en vivo con los IDs
    #   - ArUco Map (con labels) → posiciones de los 10 markers + IDs
    #   - Covariance Ellipse → la elipse crece/encoge con observaciones
    #   - Maze geometry → muros visibles
    # Le agregamos los displays de nav (Waypoints, Bug FSM Markers, LiDAR)
    # para tener TODO en una sola ventana. Editado en el ekf.rviz original.
    rviz_cfg = os.path.join(ekf_share, 'rviz', 'ekf.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen', parameters=common)

    # ── Arranque diferido (5s tras Gazebo) ────────────────────────────────────
    delayed = TimerAction(
        period=5.0,
        actions=[static_tf, wheels, lidar, aruco, loc, ekf, cov, coord_tf,
                 wpm, controller, bug, world_viz, rviz],
    )

    return LaunchDescription([
        no_user_site,
        declare_world, declare_bug, declare_x, declare_y, declare_yaw,
        declare_use_ekf_pose,
        bringup,
        delayed,
    ])
