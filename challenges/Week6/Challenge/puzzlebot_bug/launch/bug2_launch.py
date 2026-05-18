"""
bug2_launch — Sistema completo Bug 2 (Task 2)
==============================================
Mini Challenge · Reactive Navigation · Week 6

Idéntico a bug0_launch.py pero arranca el nodo 'bug2' en vez de 'bug0'.

Uso:
    ros2 launch puzzlebot_bug_w6 bug2_launch.py
    ros2 launch puzzlebot_bug_w6 bug2_launch.py world:=bug_hard.world target_x:=2.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'puzzlebot_bug_w6'

    declare_world = DeclareLaunchArgument('world',    default_value='bug_easy.world')
    declare_tx    = DeclareLaunchArgument('target_x', default_value='3.0')
    declare_ty    = DeclareLaunchArgument('target_y', default_value='0.0')
    declare_x     = DeclareLaunchArgument('x',        default_value='0.0')
    declare_y     = DeclareLaunchArgument('y',        default_value='0.0')
    declare_yaw   = DeclareLaunchArgument('yaw',      default_value='0.0')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x':     LaunchConfiguration('x'),
            'y':     LaunchConfiguration('y'),
            'yaw':   LaunchConfiguration('yaw'),
        }.items(),
    )

    common_params = [{'use_sim_time': True}]

    lidar = Node(package=pkg, executable='lidar_processor',
                 name='lidar_processor', output='screen', parameters=common_params)

    wheels = Node(package=pkg, executable='wheel_state_bridge',
                  name='wheel_state_bridge', output='screen',
                  parameters=common_params)

    loc = Node(package=pkg, executable='localisation',
               name='localisation', output='screen',
               parameters=common_params + [{
                   'x_init':     LaunchConfiguration('x'),
                   'y_init':     LaunchConfiguration('y'),
                   'theta_init': LaunchConfiguration('yaw'),
               }])

    coord_tf = Node(package=pkg, executable='coordinate_transform',
                    name='coordinate_transform', output='screen',
                    parameters=common_params)

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=common_params,
    )

    goal = Node(package=pkg, executable='goal_publisher',
                name='goal_publisher', output='screen',
                parameters=common_params + [{
                    'target_x': LaunchConfiguration('target_x'),
                    'target_y': LaunchConfiguration('target_y'),
                }])

    bug = Node(package=pkg, executable='bug2',
               name='bug2', output='screen',
               parameters=common_params + [{
                   'target_x': LaunchConfiguration('target_x'),
                   'target_y': LaunchConfiguration('target_y'),
               }])

    controller = Node(package=pkg, executable='controller',
                      name='controller', output='screen',
                      parameters=common_params + [{
                          'dist_tolerance': 0.10,
                      }])

    rviz_cfg = os.path.join(get_package_share_directory(pkg),
                            'rviz', 'puzzlebot_bug.rviz')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen', parameters=common_params)

    # Delay 5 s para que Gazebo + bridge ya estén publicando /clock antes de
    # que arranquen los nodos con use_sim_time=true.
    delayed_nav = TimerAction(
        period=8.0,
        actions=[static_tf, lidar, wheels, loc, coord_tf, goal, controller, bug, rviz],
    )

    return LaunchDescription([
        declare_world, declare_tx, declare_ty, declare_x, declare_y, declare_yaw,
        bringup,
        delayed_nav,
    ])
