"""
Launch: multi_robot_launch.py  — Lanzador Principal del Mini Challenge 3
=========================================================================
Orquesta la simulación de DOS Puzzlebots simultáneos en un único entorno ROS 2.

Este archivo implementa exactamente el diagrama de bloques del PDF del profesor
(Mini Challenge 3 — Manchester Robotics), replicando el stack completo de nodos
DOS VECES: una por robot, cada instancia completamente aislada bajo su propio
namespace y prefijo de TF.

Arquitectura (según diagrama del PDF):
    ┌─────────────────────────────────────────────────────────────────────┐
    │  Namespace: robot1                                                  │
    │  robot1/controller → robot1/robot → robot1/localisation            │
    │                                  ↓                                 │
    │              robot1/joint state Pub → robot1/robot state publisher │
    └─────────────────────────────────────────────────────────────────────┘
    ┌─────────────────────────────────────────────────────────────────────┐
    │  Namespace: robot2                                                  │
    │  robot2/controller → robot2/robot → robot2/localisation            │
    │                                  ↓                                 │
    │              robot2/joint state Pub → robot2/robot state publisher │
    └─────────────────────────────────────────────────────────────────────┘
    Ambos grupos comparten: .STL Files → URDF File → RViz

Configuración de los robots:
    Robot 1: namespace='robot1', inicio=(0, 0), color=rojo,  trayectoria=CUADRADO
    Robot 2: namespace='robot2', inicio=(3, 0), color=azul,  trayectoria=TRIÁNGULO

Ejecución:
    ros2 launch puzzlebot_sim multi_robot_launch.py

Dependencias:
    robot_launch.py  — plantilla de stack de un solo robot (incluida dos veces)
    multi_puzzlebot_rviz.rviz — configuración de RViz con ambos robots
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg       = 'puzzlebot_sim'
    pkg_share = get_package_share_directory(pkg)

    # Rutas a los archivos de configuración instalados
    robot_launch = os.path.join(pkg_share, 'launch', 'robot_launch.py')
    rviz_config  = os.path.join(pkg_share, 'rviz',   'multi_puzzlebot_rviz.rviz')

    # ── Robot 1 ───────────────────────────────────────────────────────────────
    # Inicia en el origen (0, 0) y recorre un cuadrado de 1.5 m de lado.
    # Color del chasis: ROJO (asignado automáticamente en robot_launch.py).
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'robot_ns':   'robot1',
            'x_init':     '0.0',
            'y_init':     '0.0',
            'theta_init': '0.0',
            'shape':      'square',
        }.items()
    )

    # ── Robot 2 ───────────────────────────────────────────────────────────────
    # Inicia desplazado 3 m a la derecha y recorre un triángulo.
    # Color del chasis: AZUL (asignado automáticamente en robot_launch.py).
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'robot_ns':   'robot2',
            'x_init':     '3.0',
            'y_init':     '0.0',
            'theta_init': '0.0',
            'shape':      'triangle',
        }.items()
    )

    # ── RViz — visualización compartida de ambos robots ───────────────────────
    # La configuración multi_puzzlebot_rviz.rviz incluye:
    #   • Modelo 3D de robot1 (rojo) y robot2 (azul)
    #   • Trayectorias de odometría de ambos robots
    #   • MarkerArrays con los waypoints de cada trayectoria
    #   • Frame global 'map' como referencia
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── Generador automático de TF Tree ───────────────────────────────────────
    # Ejecuta 'view_frames' después de 10 segundos para asegurar que el árbol
    # de transformadas ya esté poblado por los nodos de localización.
    tf_tree = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([robot1, robot2, rviz, tf_tree])
