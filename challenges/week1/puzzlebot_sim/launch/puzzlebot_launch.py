import os
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("puzzlebot_sim")
    urdf_file = os.path.join(pkg_share, "urdf", "puzzlebot.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "puzzlebot_rviz.rviz")

    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # Fix for spaces in paths
    pkg_path = "file://" + urllib.parse.quote(pkg_share)
    robot_desc = robot_desc.replace("package://puzzlebot_sim", pkg_path)

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="puzzlebot_sim",
                executable="joint_state_publisher",
                name="puzzlebot_kinematic_sim",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )