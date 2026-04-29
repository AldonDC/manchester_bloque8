import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Sistema de Lanzamiento para el Reto 4 (Trayectorias Autónomas).
    Este archivo orquesta la arquitectura completa de nodos de Manchester Robotics.
    """
    package_name = 'puzzlebot_sim'
    
    # ---------------------------------------------------------
    # ARGUMENTOS DE LANZAMIENTO (Configuración dinámica)
    # ---------------------------------------------------------
    # Permite al usuario elegir qué figura geométrica trazar desde la terminal.
    shape_arg = DeclareLaunchArgument(
        'shape',
        default_value='square',
        description='Figura de la trayectoria: square, triangle, or hexagon'
    )
    shape_config = LaunchConfiguration('shape')
    
    # ---------------------------------------------------------
    # RUTAS DE ARCHIVOS (Dependencias de configuración)
    # ---------------------------------------------------------
    # Ubicamos el modelo físico (URDF) y la vista de RViz guardada.
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'puzzlebot_rviz.rviz')
    
    # ---------------------------------------------------------
    # PARTE 1: SIMULACIÓN DERIVADA (Robot Físico)
    # ---------------------------------------------------------
    # Procesa el URDF para que ROS 2 entienda la estructura de joints y links.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # Nodo "Real / Sim Robot": El motor de física del robot.
    kinematic_sim = Node(
        package=package_name,
        executable='part1_kinematic_sim',
        name='real_sim_robot',
        output='screen'
    )
    
    # ---------------------------------------------------------
    # PARTE 2: LOCALIZACIÓN Y ESTADO DE JUNTAS
    # ---------------------------------------------------------
    # "Localisation": Estima la posición mediante dead-reckoning (Pose: x, y, th).
    localisation = Node(
        package=package_name,
        executable='part2_localisation',
        name='localisation',
        output='screen'
    )
    
    # "Coordinate Transform": Convierte la odometría en transformadas TF (odom -> footprint).
    coordinate_transform = Node(
        package=package_name,
        executable='part2_coordinate_transform',
        name='coordinate_transform',
        output='screen'
    )
    
    # Publica el estado de las llantas para que se vean girar en RViz.
    joint_state_pub = Node(
        package=package_name,
        executable='part2_joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # ---------------------------------------------------------
    # PARTE 3: CONTROLADOR Y GENERADOR (Inteligencia Autónoma)
    # ---------------------------------------------------------
    # "Controller": El cerebro PID que calcula cmd_vel para llegar al punto tx, ty.
    control = Node(
        package=package_name,
        executable='part3_control',
        name='controller',
        output='screen',
        parameters=[{'target_x': 0.0, 'target_y': 0.0}]
    )
    
    # "Set Point Generator": Administra la lista de puntos de la figura elegida.
    trajectory = Node(
        package=package_name,
        executable='part3_trajectory_generator',
        name='trajectory_set_point_generator',
        output='screen',
        parameters=[{'shape': shape_config}]
    )
    
    # ---------------------------------------------------------
    # UTILIDADES (Visualización y Estática)
    # ---------------------------------------------------------
    # Referencia mundial: Vincula el "mapa" con el inicio de nuestra odometría.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Interfaz Gráfica (RViz)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    # ---------------------------------------------------------
    # LANZAMIENTO GLOBAL
    # ---------------------------------------------------------
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
