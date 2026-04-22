import os
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = get_package_share_directory("puzzlebot_sim")
    urdf_file = os.path.join(pkg_share, "urdf", "puzzlebot.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "puzzlebot_rviz.rviz")

    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # Fix for spaces in paths
    pkg_path = "file://" + urllib.parse.quote(pkg_share)
    robot_desc = robot_desc.replace("package://puzzlebot_sim", pkg_path)


    # Asegurar que el directorio de salida para PDFs existe
    output_pdf_path = "/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/output_pdf"
    if not os.path.exists(output_pdf_path):
        os.makedirs(output_pdf_path)

    return LaunchDescription(
        [
            # Publicador del estado del robot (procesa el URDF)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
            ),
            # Simulador cinematico (publica odom -> base_footprint)
            Node(
                package="puzzlebot_sim",
                executable="joint_state_publisher",
                name="puzzlebot_kinematic_sim",
            ),
            # Visualizador RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
            # Generador automatico de reporte en PDF
            ExecuteProcess(
                cmd=["ros2", "run", "tf2_tools", "view_frames"],
                output="screen",
                cwd="/home/alfonso/Documents/8 Semestre/manchester_bloque/challenges/output_pdf",  # Guardar aqui el PDF
            ),
        ]
    )