"""
Launch principal — Mini Challenge 4  ·  Task 1
================================================
Sistema completo con propagación de covarianza visualizada en RViz.

CUMPLE LA TOPOLOGÍA EXACTA DEL DIAGRAMA DEL PDF:

    Trajectory ─► Controller ─► Real/Sim Robot ─► Localisation ─► RViz
                                                   (con Σ_k 3x3)    ▲
                                                        │           │
                                                        └► JointState┘
                                                        └► CoordinateTransform

NODOS QUE LANZA (8 en total):
    robot_state_publisher        Procesa el URDF
    real_sim_robot               Simula la cinemática del Puzzlebot
    localisation                 ⭐ Dead reckoning + propaga Σ_k
    joint_state_publisher        Anima ruedas del modelo
    coordinate_transform         TF dinámica odom → base_footprint
    controller                   PID hacia waypoints
    trajectory_set_point_gen     Genera figura geométrica
    static_map_to_odom           TF estática para anclar la escena
    rviz2                        Visualización con elipse activada

ARGUMENTOS:
    shape        square | triangle | hexagon       (default 'square')
    k_r          ganancia ruido rueda derecha       (default 0.05)
    k_l          ganancia ruido rueda izquierda     (default 0.05)
    sample_time  período de integración [s]         (default 0.02)

USO:
    ros2 launch puzzlebot_sim_w5 puzzlebot_challenge5_launch.py
    ros2 launch puzzlebot_sim_w5 puzzlebot_challenge5_launch.py shape:=hexagon k_r:=0.10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'puzzlebot_sim_w5'

    # ── Argumentos configurables desde CLI ───────────────────────────────────
    shape_arg = DeclareLaunchArgument(
        'shape',
        default_value='square',
        description='Trajectory shape: square | triangle | hexagon',
    )
    kr_arg = DeclareLaunchArgument(
        'k_r',
        default_value='0.05',
        description='Right wheel noise gain (Σ_Δ)',
    )
    kl_arg = DeclareLaunchArgument(
        'k_l',
        default_value='0.05',
        description='Left wheel noise gain (Σ_Δ)',
    )
    dt_arg = DeclareLaunchArgument(
        'sample_time',
        default_value='0.02',
        description='Sample time for integration [s]',
    )

    shape  = LaunchConfiguration('shape')
    k_r    = LaunchConfiguration('k_r')
    k_l    = LaunchConfiguration('k_l')
    dt     = LaunchConfiguration('sample_time')

    # ── Rutas de archivos compartidos ────────────────────────────────────────
    urdf_file   = os.path.join(
        get_package_share_directory(package_name), 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'puzzlebot_covariance.rviz')

    # ── Nodos ROS 2 ──────────────────────────────────────────────────────────

    # Procesa el URDF para que RViz conozca la estructura del Puzzlebot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}],
    )

    # Real/Sim Robot — integra (v, ω) y publica wr, wl + ground truth.
    kinematic_sim = Node(
        package=package_name,
        executable='part1_kinematic_sim',
        name='real_sim_robot',
        output='screen',
        parameters=[{'sample_time': dt}],
    )

    # Localisation — dead reckoning + propagación de Σ_k (corazón del reto).
    localisation = Node(
        package=package_name,
        executable='part2_localisation',
        name='localisation',
        output='screen',
        parameters=[{
            'sample_time': dt,
            'k_r':         k_r,
            'k_l':         k_l,
        }],
    )

    # Joint State Publisher — anima ruedas del URDF en RViz.
    joint_state_pub = Node(
        package=package_name,
        executable='part2_joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'sample_time': dt}],
    )

    # Coordinate Transform — publica TF dinámica odom → base_footprint.
    coordinate_transform = Node(
        package=package_name,
        executable='part2_coordinate_transform',
        name='coordinate_transform',
        output='screen',
    )

    # Controller — PID que sigue los waypoints del generador.
    control = Node(
        package=package_name,
        executable='part3_control',
        name='controller',
        output='screen',
        parameters=[{'target_x': 0.0, 'target_y': 0.0}],
    )

    # Trajectory Set-Point Generator — figuras geométricas.
    trajectory = Node(
        package=package_name,
        executable='part3_trajectory_generator',
        name='trajectory_set_point_generator',
        output='screen',
        parameters=[{'shape': shape}],
    )

    # TF estático map → odom para anclar visualización en RViz.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # RViz con configuración pre-cargada y display de covarianza activado.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        shape_arg,
        kr_arg,
        kl_arg,
        dt_arg,
        robot_state_publisher,
        kinematic_sim,
        localisation,
        joint_state_pub,
        coordinate_transform,
        control,
        trajectory,
        static_tf,
        rviz,
    ])
