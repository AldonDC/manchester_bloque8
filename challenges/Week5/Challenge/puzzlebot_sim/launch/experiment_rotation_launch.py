"""
Experimento 2 — Rotación en sitio (Task 2)
===========================================
Calibra k_r, k_l observando cómo crece la incertidumbre angular cuando el
Puzzlebot gira en su propio eje (v = 0, ω constante).

Configuración:
    v          = 0
    ω          = π/2 rad/s ≈ 1.5708 rad/s  (un cuarto de vuelta por segundo)
    duración   = 4.0 s    (≈ una vuelta completa)

Este experimento aísla el componente σ_θθ de la matriz Σ_k, porque al no
haber traslación neta, x y y casi no cambian — la elipse crece principalmente
en orientación.

Cómo usar:
    ros2 launch puzzlebot_sim_w5 experiment_rotation_launch.py
    ros2 launch puzzlebot_sim_w5 experiment_rotation_launch.py k_r:=0.10 k_l:=0.10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'puzzlebot_sim_w5'

    kr_arg  = DeclareLaunchArgument('k_r', default_value='0.05')
    kl_arg  = DeclareLaunchArgument('k_l', default_value='0.05')
    w_arg   = DeclareLaunchArgument('angular_velocity', default_value='1.5708')
    dur_arg = DeclareLaunchArgument('duration', default_value='4.0')

    k_r = LaunchConfiguration('k_r')
    k_l = LaunchConfiguration('k_l')
    w   = LaunchConfiguration('angular_velocity')
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

    open_loop = Node(
        package=package_name,
        executable='part2_open_loop_driver',
        name='open_loop_driver',
        output='screen',
        parameters=[{
            'linear_velocity':  0.0,
            'angular_velocity': w,
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
        kr_arg, kl_arg, w_arg, dur_arg,
        robot_state_publisher,
        kinematic_sim,
        localisation,
        joint_state_pub,
        coordinate_transform,
        open_loop,
        static_tf,
        rviz,
    ])
