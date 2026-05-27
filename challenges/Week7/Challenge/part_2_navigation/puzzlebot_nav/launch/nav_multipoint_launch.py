"""
nav_multipoint_launch — Final Challenge Part 2: closed multi-waypoint navigation.

Brings up:
    1) Gazebo Harmonic + Puzzlebot with LiDAR  (via bringup_launch.py)
    2) lidar_processor      — front/left/right sectors from /scan
    3) wheel_state_bridge   — /joint_states -> /wr, /wl
    4) localisation         — dead reckoning + Sigma_k
    5) coordinate_transform — broadcasts TF odom->base_link
    6) bug2 (or bug0)       — single-goal reactive nav with obstacle avoidance
    7) waypoint_manager     — sequences >=4 closed-loop waypoints to /goal
    8) controller           — PID on /set_point from the Bug
    9) rviz2                — visualisation

Usage:
    ros2 launch puzzlebot_nav nav_multipoint_launch.py
    ros2 launch puzzlebot_nav nav_multipoint_launch.py world:=bug_hard.world
    ros2 launch puzzlebot_nav nav_multipoint_launch.py bug:=bug0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'puzzlebot_nav'

    declare_world = DeclareLaunchArgument('world', default_value='bug_medium.world')
    declare_bug = DeclareLaunchArgument(
        'bug', default_value='bug2',
        description="Reactive nav algorithm: 'bug0' or 'bug2'")
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    common = [{'use_sim_time': True}]
    waypoints_yaml = os.path.join(
        get_package_share_directory(pkg), 'config', 'waypoints.yaml')

    lidar = Node(package=pkg, executable='lidar_processor',
                 name='lidar_processor', output='screen', parameters=common)
    wheels = Node(package=pkg, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen', parameters=common)

    loc = Node(package=pkg, executable='localisation',
               name='localisation', output='screen',
               parameters=common + [{
                   'x_init': LaunchConfiguration('x'),
                   'y_init': LaunchConfiguration('y'),
                   'theta_init': LaunchConfiguration('yaw'),
               }])

    coord_tf = Node(package=pkg, executable='coordinate_transform',
                    name='coordinate_transform', output='screen', parameters=common)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common,
    )

    # The waypoint manager drives /goal; the Bug node consumes /goal and
    # publishes /bug/state. The manager listens to /bug/state to advance.
    wpm = Node(package=pkg, executable='waypoint_manager',
               name='waypoint_manager', output='screen',
               parameters=common + [waypoints_yaml])

    bug = Node(package=pkg, executable=LaunchConfiguration('bug'),
               name='bug', output='screen', parameters=common)

    controller = Node(package=pkg, executable='controller',
                      name='controller', output='screen',
                      parameters=common + [{'dist_tolerance': 0.10}])

    rviz_cfg = os.path.join(get_package_share_directory(pkg),
                            'rviz', 'puzzlebot_nav.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen', parameters=common)

    delayed_nav = TimerAction(
        period=8.0,
        actions=[static_tf, lidar, wheels, loc, coord_tf,
                 wpm, controller, bug, rviz],
    )

    return LaunchDescription([
        declare_world, declare_bug, declare_x, declare_y, declare_yaw,
        bringup,
        delayed_nav,
    ])
