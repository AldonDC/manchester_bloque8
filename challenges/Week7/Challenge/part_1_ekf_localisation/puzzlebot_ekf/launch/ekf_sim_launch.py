"""
ekf_sim_launch — Full Part 1 stack in Gazebo.

Brings up:
    1) Gazebo Harmonic + puzzlebot_jetson_ed + bridges    (bringup_launch.py)
    2) wheel_state_bridge   — /joint_states -> /wr, /wl
    3) aruco_detector       — /camera/image_raw -> /aruco/observations
    4) ekf_node             — predict + correct -> /ekf/odom
    5) covariance_publisher — /ekf/odom -> /ekf/covariance_ellipse
    6) rviz2

The default world (puzzlebot_aruco_markers.world) already has 4 markers
placed by MCR2 at:
    id=0  ( 2.5, -0.5)  yaw=pi
    id=1  ( 2.5,  2.5)  yaw=3pi/2
    id=2  (-0.5,  2.5)  yaw=0
    id=3  (-0.5, -0.5)  yaw=pi/2
These match config/aruco_map.yaml so the EKF can immediately correct
against them.

Usage:
    ros2 launch puzzlebot_ekf ekf_sim_launch.py
    ros2 launch puzzlebot_ekf ekf_sim_launch.py world:=puzzlebot_arena_markers.world
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, SetEnvironmentVariable, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'puzzlebot_ekf'
    pkg_nav = 'puzzlebot_nav'   # we borrow wheel_state_bridge from Part 2
    share = get_package_share_directory(pkg)

    # ── NumPy / OpenCV ABI guard ─────────────────────────────────────────────
    # The Ubuntu-shipped python3-opencv (apt) is compiled against numpy 1.x.
    # If ~/.local/lib/python3.10/site-packages has a pip-installed numpy 2.x,
    # the import collides at runtime:
    #     "AttributeError: _ARRAY_API not found"
    #     "ImportError: numpy.core.multiarray failed to import"
    # PYTHONNOUSERSITE=1 disables ~/.local in this launch's child processes
    # only, so the apt numpy 1.21 wins and cv2 imports cleanly. It is purely
    # additive — does not touch system-wide python paths.
    no_user_site = SetEnvironmentVariable(name='PYTHONNOUSERSITE', value='1')

    # ── Render env (Week 6 pattern, must be at TOP-LEVEL launch) ────────────
    # If we leave these only inside bringup_launch.py and we IncludeLaunchDescription
    # it from here, the SetEnvironmentVariable in the included launch only applies
    # to that launch's own subprocesses — NOT to `gz sim`, which is launched
    # from inside its own OpaqueFunction. To guarantee `gz sim` inherits them,
    # we set them at the top level (this file) BEFORE the include.
    render_envs = [
        SetEnvironmentVariable(name='OGRE_RTT_MODE',       value='Copy'),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM',     value='xcb'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='mesa'),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='GALLIUM_DRIVER',      value='llvmpipe'),
        SetEnvironmentVariable(name='MESA_LOADER_DRIVER_OVERRIDE', value='kms_swrast'),
    ]

    ekf_yaml = os.path.join(share, 'config', 'ekf_params.yaml')
    aruco_map_yaml = os.path.join(share, 'config', 'aruco_map.yaml')

    declare_world = DeclareLaunchArgument(
        'world', default_value='ekf_arena.world',
        description='Default: custom enclosed arena. Use puzzlebot_aruco_markers.world '
                    'for the MCR2-shipped flat-floor variant.')
    # Robot DENTRO del laberinto en (0.6, 0.8) mirando norte (yaw=pi/2).
    # marker_1 está directamente al frente en (0.6, 1.92) a 1.12 m,
    # centrado en el FOV. El robot ve el ArUco patrón nítido al arrancar.
    declare_x = DeclareLaunchArgument('x', default_value='0.6')
    declare_y = DeclareLaunchArgument('y', default_value='0.8')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='1.5708')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    common = [{'use_sim_time': True}]

    # /joint_states -> /wr, /wl (reused from Part 2 / Week 6).
    wheels = Node(package=pkg_nav, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen',
                  parameters=common)

    aruco = Node(package=pkg, executable='aruco_detector',
                 name='aruco_detector', output='screen',
                 parameters=common + [ekf_yaml])

    # The EKF needs its x_init/y_init/theta_init as actual floats matching
    # the Gazebo spawn pose. We write a tiny override YAML at runtime with
    # ONLY those three keys and pass it AFTER the main ekf_params.yaml so
    # ROS 2's parameter loader (which merges YAMLs in order) takes our
    # spawn pose as the final value. Mixing dict + YAML in `parameters=[...]`
    # had inconsistent precedence on Humble — the file-based override is
    # the only form that ROS 2 reliably applies last.
    def _make_ekf_node(context):
        x_val = float(LaunchConfiguration('x').perform(context))
        y_val = float(LaunchConfiguration('y').perform(context))
        yaw_val = float(LaunchConfiguration('yaw').perform(context))
        override_path = '/tmp/puzzlebot_ekf_init_override.yaml'
        with open(override_path, 'w') as f:
            f.write(
                'ekf_node:\n'
                '  ros__parameters:\n'
                f'    x_init: {x_val}\n'
                f'    y_init: {y_val}\n'
                f'    theta_init: {yaw_val}\n'
                f"    aruco_map_yaml: '{aruco_map_yaml}'\n"
            )
        return [Node(
            package=pkg, executable='ekf_node',
            name='ekf_node', output='screen',
            parameters=common + [ekf_yaml, override_path],
        )]
    ekf = OpaqueFunction(function=_make_ekf_node)

    cov = Node(package=pkg, executable='covariance_publisher',
               name='covariance_publisher', output='screen',
               parameters=common + [ekf_yaml])

    # Publish the maze geometry as a MarkerArray so RViz shows the same
    # arena as Gazebo. This is *especially* useful here because the camera
    # sensor pipeline in Gazebo Harmonic is broken on this machine (EGL
    # PBuffer driver too old), so the in-Gazebo visualisation is unreliable;
    # RViz becomes the canonical view of where the robot is.
    world_viz = Node(package=pkg, executable='world_visualizer',
                     name='world_visualizer', output='screen',
                     parameters=common)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common,
    )

    rviz_cfg = os.path.join(share, 'rviz', 'ekf.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen',
                parameters=common)

    # Optional automated demo that drives the robot through the 3 scenarios
    # from the PDF (multi-marker / no-marker / partial). Off by default;
    # enable with `auto_demo:=true` (or use the part1-sim-demo CLI shortcut).
    declare_auto_demo = DeclareLaunchArgument(
        'auto_demo', default_value='false',
        description='If true, launches the auto_demo node that drives the '
                    'robot through the PDF Part-1 scenarios automatically.')

    def _make_auto_demo_node(context):
        if LaunchConfiguration('auto_demo').perform(context).lower() != 'true':
            return []
        return [Node(
            package=pkg, executable='auto_demo',
            name='auto_demo', output='screen',
            parameters=common + [{'warmup_s': 10.0,
                                   'publish_rate_hz': 20.0}],
        )]
    auto_demo = OpaqueFunction(function=_make_auto_demo_node)

    # Delay 8s after Gazebo to ensure /clock and /joint_states are flowing
    # before the EKF starts predicting against use_sim_time.
    delayed = TimerAction(
        period=8.0,
        actions=[static_tf, wheels, aruco, ekf, cov, world_viz, rviz,
                 auto_demo],
    )

    return LaunchDescription([
        no_user_site,
        *render_envs,
        declare_world, declare_x, declare_y, declare_yaw, declare_auto_demo,
        bringup,
        delayed,
    ])
