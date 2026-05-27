"""
ekf_only_launch — Bring up just the EKF stack (no simulator).

Useful when:
    * playing back a rosbag with /camera/image_raw, /camera/camera_info,
      /wr, /wl already in it
    * connecting to the real Puzzlebot from another machine
    * iterating on EKF parameters without the cost of restarting Gazebo

Brings up:
    aruco_detector        — image -> /aruco/observations
    ekf_node              — /wr, /wl, /aruco/observations -> /ekf/odom
    covariance_publisher  — /ekf/odom -> /ekf/covariance_ellipse

Usage:
    ros2 launch puzzlebot_ekf ekf_only_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'puzzlebot_ekf'
    share = get_package_share_directory(pkg)

    # Disable ~/.local so apt's numpy 1.x wins over pip's numpy 2.x and the
    # apt-built cv2 imports cleanly. See ekf_sim_launch.py for details.
    no_user_site = SetEnvironmentVariable(name='PYTHONNOUSERSITE', value='1')

    ekf_yaml = os.path.join(share, 'config', 'ekf_params.yaml')
    aruco_map_yaml = os.path.join(share, 'config', 'aruco_map.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description="Use Gazebo /clock when true (default for simulation)")

    common = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]

    aruco = Node(package=pkg, executable='aruco_detector',
                 name='aruco_detector', output='screen',
                 parameters=common + [ekf_yaml])

    ekf = Node(package=pkg, executable='ekf_node',
               name='ekf_node', output='screen',
               parameters=common + [ekf_yaml,
                                    {'aruco_map_yaml': aruco_map_yaml}])

    cov = Node(package=pkg, executable='covariance_publisher',
               name='covariance_publisher', output='screen',
               parameters=common + [ekf_yaml])

    rviz_cfg = os.path.join(share, 'rviz', 'ekf.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen',
                parameters=common)

    return LaunchDescription([
        no_user_site,
        declare_use_sim_time,
        aruco, ekf, cov, rviz,
    ])
