"""
part2_in_maze_launch — Part 2 ("Unknown exploration") run in the same
hamster-maze arena used by Part 1.

Why a separate launch (not reusing nav_multipoint_launch.py):
    nav_multipoint_launch.py spawns a Puzzlebot in one of the bug_X.world
    files from Week 6 — those worlds are simple test grounds for Bug 0/2.
    The PDF Part 2 explicitly shows the navigation happening *inside the
    arena* the student built for Part 1 (the maze with the 4 ArUco
    markers). So this launch:
      * boots Gazebo with `ekf_arena.world` (Part 1's plywood-style maze)
      * spawns the camera+LiDAR Puzzlebot via Part 1's bringup_launch
      * starts the Week 6 Bug 0/2 + waypoint_manager stack
      * preloads a waypoints.yaml whose 4 target points live INSIDE the
        maze (corridors between the inner chambers)

If you want the original Week-6-style test, use nav_multipoint_launch.py.
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

    # Disable ~/.local so apt numpy 1.21 wins over pyenv's numpy 2.x.
    no_user_site = SetEnvironmentVariable(name='PYTHONNOUSERSITE', value='1')

    # Spawn AL NORTE DEL DIVISOR CENTRAL, alineado al gap. Ubicación
    # (1.9, 2.3): 0.5m al norte del divisor ih2 (y=2.0), 0.5m de cada
    # muro del gap (ih2_left termina en x=1.4, ih2_right empieza en
    # x=2.4). Apunta al sur (yaw=-π/2) para bajar directo al primer
    # waypoint WP0=(1.9, 0.0) cruzando por el gap. Distancia inicial:
    # 2.3m sin obstáculos en medio. v1 spawneaba en (0.9, 0.8) DENTRO
    # de iv2. v2 en (1.95, 2.0) sobre el divisor. v3 en (1.9, 1.0)
    # también atorado contra iv2_east. v4: ABOVE divisor, in pure gap.
    declare_x = DeclareLaunchArgument('x',   default_value='1.9')
    declare_y = DeclareLaunchArgument('y',   default_value='2.3')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='-1.5708')
    declare_bug = DeclareLaunchArgument(
        'bug', default_value='bug2',
        description="Reactive nav algorithm: 'bug0' or 'bug2'")
    declare_world = DeclareLaunchArgument(
        'world', default_value='ekf_arena.world',
        description='Arena from Part 1 (hamster-style maze)')

    # Reuse Part 1's bringup_launch (Gazebo + camera+LiDAR Puzzlebot +
    # ros_gz bridges). That launch already exports the same software-
    # render env that makes the robot visible on this machine.
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

    # /joint_states -> /wr, /wl (Week 6 reuse)
    wheels = Node(package=nav_pkg, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen', parameters=common)

    lidar = Node(package=nav_pkg, executable='lidar_processor',
                 name='lidar_processor', output='screen', parameters=common)

    # `localisation` lee x_init/y_init/theta_init como DOUBLES. Si los
    # pasamos como LaunchConfiguration (string) ROS 2 los ignora y deja
    # los defaults en 0.0 → la odometría arranca en (0,0,0) pero el
    # robot está en (0.6, 0.8, π/2) → el Bug ve el goal en la dirección
    # equivocada y el controller gira en círculo. Resolvemos con un
    # OpaqueFunction que materializa los valores como floats reales.
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

    coord_tf = Node(package=nav_pkg, executable='coordinate_transform',
                    name='coordinate_transform', output='screen',
                    parameters=common)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common,
    )

    # Waypoint manager loads waypoints_maze.yaml (4 points inside the maze).
    wpm = Node(package=nav_pkg, executable='waypoint_manager',
               name='waypoint_manager', output='screen',
               parameters=common + [waypoints_yaml])

    # Bug 0 or Bug 2 (selectable via launch arg).
    bug = Node(package=nav_pkg, executable=LaunchConfiguration('bug'),
               name='bug', output='screen', parameters=common)

    # El controller PID convencional (set_point → cmd_vel) se lanza solo
    # para los Bug clásicos (bug0/bug1/bug2), porque ESOS publican un
    # set_point reactivo y delegan el control al PID externo. En cambio
    # goto_goal NUEVO tiene FSM interna con velocidades adaptativas por
    # estado y publica DIRECTAMENTE a /cmd_vel — si lanzáramos también
    # el controller, ambos publicarían a /cmd_vel y se pelearían.
    def _make_controller(context):
        bug_choice = LaunchConfiguration('bug').perform(context)
        if bug_choice == 'goto_goal':
            return []  # goto_goal hace su propio control, no necesita PID externo
        return [Node(package=nav_pkg, executable='controller',
                     name='controller', output='screen',
                     parameters=common + [{'dist_tolerance': 0.15}])]
    controller = OpaqueFunction(function=_make_controller)

    # Visualise the maze walls in RViz so the view matches Gazebo.
    world_viz = Node(package=ekf_pkg, executable='world_visualizer',
                     name='world_visualizer', output='screen',
                     parameters=common)

    # Reuse Part 1's RViz config (already shows maze + robot + ellipse).
    # Part 2 usa el RViz config DE NAV (no el de EKF) para que se vean las
    # banderas verdes en /visualization_marker_array, los markers del Bug 2
    # y la línea cerrada que une los 4 waypoints. El ekf.rviz no carga esos
    # displays y por eso las banderas no aparecían en la primera corrida.
    rviz_cfg = os.path.join(nav_share, 'rviz', 'puzzlebot_nav.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen', parameters=common)

    # Arrancamos todo el stack a la vez tras 5 s (suficiente para que
    # /clock y los bridges Gazebo↔ROS estén publicando). Combinado con el
    # `startup_delay=5.0` del waypoint_manager, queda así:
    #   t = 0 s   ros2 launch
    #   t = 3 s   spawn del robot en Gazebo, RViz aún cargando
    #   t = 5 s   bug, manager, EKF, RViz, banderas todos arriba
    #   t =10 s   waypoint_manager publica el primer /goal → el robot arranca
    # El bug además espera a tener /odom y /scan reales antes de moverse.
    delayed = TimerAction(
        period=5.0,
        actions=[static_tf, wheels, lidar, loc, coord_tf,
                 wpm, controller, bug, world_viz, rviz],
    )

    return LaunchDescription([
        no_user_site,
        declare_world, declare_bug, declare_x, declare_y, declare_yaw,
        bringup,
        delayed,
    ])
