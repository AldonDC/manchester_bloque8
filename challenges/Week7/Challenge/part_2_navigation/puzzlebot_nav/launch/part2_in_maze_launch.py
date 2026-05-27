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
                             TimerAction, SetEnvironmentVariable)
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

    # Robot spawns inside the maze, same spot as Part 1's default.
    declare_x = DeclareLaunchArgument('x',   default_value='1.95')
    declare_y = DeclareLaunchArgument('y',   default_value='2.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
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

    loc = Node(package=nav_pkg, executable='localisation',
               name='localisation', output='screen',
               parameters=common + [{
                   'x_init': LaunchConfiguration('x'),
                   'y_init': LaunchConfiguration('y'),
                   'theta_init': LaunchConfiguration('yaw'),
               }])

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

    controller = Node(package=nav_pkg, executable='controller',
                      name='controller', output='screen',
                      parameters=common + [{'dist_tolerance': 0.15}])

    # Visualise the maze walls in RViz so the view matches Gazebo.
    world_viz = Node(package=ekf_pkg, executable='world_visualizer',
                     name='world_visualizer', output='screen',
                     parameters=common)

    # Reuse Part 1's RViz config (already shows maze + robot + ellipse).
    rviz_cfg = os.path.join(ekf_share, 'rviz', 'ekf.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen', parameters=common)

    # Delay the whole stack ~8 s so /clock + bridge are up first.
    delayed = TimerAction(
        period=8.0,
        actions=[static_tf, wheels, lidar, loc, coord_tf,
                 wpm, controller, bug, world_viz, rviz],
    )

    return LaunchDescription([
        no_user_site,
        declare_world, declare_bug, declare_x, declare_y, declare_yaw,
        bringup,
        delayed,
    ])
