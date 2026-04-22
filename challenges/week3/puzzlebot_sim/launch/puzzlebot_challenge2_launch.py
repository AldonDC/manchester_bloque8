import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'puzzlebot_sim'
    
    # --- Launch Arguments ---
    shape_arg = DeclareLaunchArgument(
        'shape',
        default_value='square',
        description='Trajectory shape: square, triangle, or hexagon'
    )
    shape_config = LaunchConfiguration('shape')
    
    # --- Paths ---
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'puzzlebot_rviz.rviz')
    
    # --- Nodes ---
    
    # 1. Robot State Publisher (Static TFs from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # 2. Kinematic Simulator (Part 1 -> "Real / Sim Robot" in diagram)
    kinematic_sim = Node(
        package=package_name,
        executable='part1_kinematic_sim',
        name='real_sim_robot',
        output='screen'
    )
    
    # 3. Localisation (Part 2)
    localisation = Node(
        package=package_name,
        executable='part2_localisation',
        name='localisation',
        output='screen'
    )
    
    # 3.1 Joint State Publisher (Part 2 - User Defined)
    joint_state_pub = Node(
        package=package_name,
        executable='part2_joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # 4. Control (Part 3 -> "Controller" in diagram)
    # Este nodo lleva al robot hacia el objetivo
    control = Node(
        package=package_name,
        executable='part3_control',
        name='controller',
        output='screen',
        parameters=[{'target_x': 0.0, 'target_y': 0.0}]
    )
    
    # 4.1 Trajectory Generator (Extra Part 3)
    # Este nodo le manda los puntos al control para hacer el cuadrado, triangulo, etc.
    trajectory = Node(
        package=package_name,
        executable='part3_trajectory_generator',
        name='trajectory_set_point_generator',
        output='screen',
        parameters=[{'shape': shape_config}]
    )
    
    # 5. Coordinate Transform
    coordinate_transform = Node(
        package=package_name,
        executable='part2_coordinate_transform',
        name='coordinate_transform',
        output='screen'
    )
    
    # 6. Static TF (Map -> Odom) para que RViz no se queje
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # 7. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        shape_arg,
        robot_state_publisher,
        kinematic_sim,
        localisation,
        joint_state_pub,
        control,
        trajectory,
        coordinate_transform,
        static_tf,
        rviz
    ])
