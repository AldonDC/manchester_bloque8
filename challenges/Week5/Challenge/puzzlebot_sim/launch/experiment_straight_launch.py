"""
Experimento 1 — Recta de 1 metro (Task 2)
==========================================
Calibra k_r, k_l comparando el crecimiento de la elipse de incertidumbre
contra el desplazamiento real del Puzzlebot a lo largo de 1 m en línea recta.

Configuración:
    v          = 0.15 m/s   (velocidad lineal constante)
    ω          = 0          (sin giro)
    duración   = 6.67 s     (≈ 1 m a 0.15 m/s)

Cómo usar:
    ros2 launch puzzlebot_sim_w5 experiment_straight_launch.py
    # o ajustando ganancias:
    ros2 launch puzzlebot_sim_w5 experiment_straight_launch.py k_r:=0.08 k_l:=0.08
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'puzzlebot_sim_w5'

    # ── Argumentos ───────────────────────────────────────────────────────────
    kr_arg = DeclareLaunchArgument('k_r', default_value='0.05')
    kl_arg = DeclareLaunchArgument('k_l', default_value='0.05')
    v_arg  = DeclareLaunchArgument('linear_velocity', default_value='0.15')
    dur_arg = DeclareLaunchArgument('duration', default_value='6.67')

    k_r = LaunchConfiguration('k_r')
    k_l = LaunchConfiguration('k_l')
    v   = LaunchConfiguration('linear_velocity')
    dur = LaunchConfiguration('duration')

    urdf_file = os.path.join(
        get_package_share_directory(package_name), 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'puzzlebot_covariance.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}],
    )

    kinematic_sim = Node(
        package=package_name,
        executable='part1_kinematic_sim',
        name='real_sim_robot',
        output='screen',
    )

    localisation = Node(
        package=package_name,
        executable='part2_localisation',
        name='localisation',
        output='screen',
        parameters=[{'k_r': k_r, 'k_l': k_l}],
    )

    joint_state_pub = Node(
        package=package_name,
        executable='part2_joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    coordinate_transform = Node(
        package=package_name,
        executable='part2_coordinate_transform',
        name='coordinate_transform',
        output='screen',
    )

    # Driver open-loop: NO usa control PID, NO usa trajectory generator.
    open_loop = Node(
        package=package_name,
        executable='part2_open_loop_driver',
        name='open_loop_driver',
        output='screen',
        parameters=[{
            'linear_velocity':  v,
            'angular_velocity': 0.0,
            'duration':         dur,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        kr_arg, kl_arg, v_arg, dur_arg,
        robot_state_publisher,
        kinematic_sim,
        localisation,
        joint_state_pub,
        coordinate_transform,
        open_loop,
        static_tf,
        rviz,
    ])
