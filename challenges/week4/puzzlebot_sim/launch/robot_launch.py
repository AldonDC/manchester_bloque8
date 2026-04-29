"""
Launch: robot_launch.py  — Stack completo de UN robot (plantilla reutilizable)
===============================================================================
Este archivo es la plantilla parametrizable que levanta todos los nodos
necesarios para simular y controlar UN Puzzlebot de forma independiente.

Diseño:
    Se utiliza en conjunto con multi_robot_launch.py, que lo incluye DOS veces
    (una por robot) con diferentes argumentos. Esto sigue el principio DRY:
    cualquier cambio al stack de un robot se aplica automáticamente a todos.

Nodos que lanza (8 en total, todos dentro del namespace del robot):
    1. robot_state_publisher    — publica TF estático del URDF (links y joints)
    2. kinematic_sim            — integra cmd_vel → pose + wr/wl  (planta virtual)
    3. localisation             — dead reckoning wr/wl → odom
    4. joint_state_publisher    — acumula ángulos de rueda → joint_states
    5. coordinate_transform     — publica TF dinámica odom → base_footprint
    6. trajectory_generator     — secuencia de waypoints → set_point
    7. controller               — PID de posición odom + set_point → cmd_vel
    8. static_transform_pub     — TF estática map → robotN/odom  (fija en origen)

Argumentos de launch disponibles:
    robot_ns    — namespace del robot y prefijo de TF  (ej. 'robot1')
    x_init      — posición X inicial  [m]
    y_init      — posición Y inicial  [m]
    theta_init  — orientación inicial [rad]
    target_x    — objetivo X inicial para el controlador (anulado por trajectory)
    target_y    — objetivo Y inicial para el controlador (anulado por trajectory)
    shape       — figura de la trayectoria: square | triangle | hexagon

Notas de implementación:
    • El URDF es prefixado dinámicamente (función _make_prefixed_urdf) para
      que cada robot tenga sus propios link/joint names únicos.
    • El color del chasis se asigna por namespace: robot1=rojo, robot2=azul.
    • Los paths de meshes con espacios se recodifican con URL encoding.
    • La TF estática map→odom se pone en (0,0,0) porque localisation ya
      inicializa el integrador en (x_init, y_init). Poner el offset aquí
      produciría una doble traslación del modelo en RViz.
"""

import os
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


# ── Utilidad: prefixar el URDF para multi-robot ───────────────────────────────

def _make_prefixed_urdf(urdf_text: str, prefix: str, pkg_share: str) -> str:
    """
    Modifica el URDF en memoria para aislar este robot en un escenario multi-robot.

    Cambios que aplica:
      1. Prefixar todos los nombres de link y joint  →  "robotN/base_link", etc.
      2. Cambiar el color del chasis según el namespace del robot.
      3. Recodificar las rutas de las meshes con URL encoding para manejar espacios
         en el path del sistema de archivos.

    El ordenado por longitud descendente evita que 'base_link' sea sustituido
    dentro de 'base_link_joint' antes de que ese string más largo sea procesado.
    """

    # Nombres de link y joint definidos en el URDF base
    names = [
        'base_footprint', 'base_link', 'wheel_r', 'wheel_l', 'caster',
        'base_link_joint', 'wheel_r_joint', 'wheel_l_joint', 'caster_joint',
    ]
    # Ordenar de mayor a menor longitud para evitar sustituciones parciales
    names.sort(key=len, reverse=True)

    result = urdf_text
    for name in names:
        result = result.replace(f'"{name}"', f'"{prefix}/{name}"')

    # Asignar color de chasis por robot (el color base del URDF es light_grey)
    color_map = {
        'robot1': '<color rgba="0.9 0.2 0.2 1.0"/>',   # rojo
        'robot2': '<color rgba="0.2 0.4 0.9 1.0"/>',   # azul
    }
    if prefix in color_map:
        result = result.replace(
            '<color rgba="0.8 0.8 0.8 1.0"/>',
            color_map[prefix]
        )

    # Reemplazar paths relativos de meshes con rutas absolutas codificadas en URL.
    # Necesario porque el path del workspace puede contener espacios (ej. "8 Semestre")
    # y las URIs de URDF requieren que los espacios sean '%20'.
    meshes_path  = os.path.join(pkg_share, 'meshes')
    encoded_path = urllib.parse.quote(meshes_path)
    result = result.replace(
        'package://puzzlebot_sim/meshes/',
        f'file://{encoded_path}/'
    )

    return result


# ── Función principal de launch (con contexto) ────────────────────────────────

def _launch_setup(context, *args, **kwargs):
    """Construye y retorna la lista de acciones de launch con argumentos resueltos."""

    pkg       = 'puzzlebot_sim'
    pkg_share = get_package_share_directory(pkg)
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    # Leer el URDF base desde el directorio de instalación del paquete
    with open(urdf_file, 'r') as f:
        raw_urdf = f.read()

    # Resolver todos los argumentos de launch a sus valores de string
    ns  = LaunchConfiguration('robot_ns').perform(context)
    x0  = LaunchConfiguration('x_init').perform(context)
    y0  = LaunchConfiguration('y_init').perform(context)
    th0 = LaunchConfiguration('theta_init').perform(context)
    tx  = LaunchConfiguration('target_x').perform(context)
    ty  = LaunchConfiguration('target_y').perform(context)
    shp = LaunchConfiguration('shape').perform(context)

    # Generar el URDF específico para este robot (prefixado y con color asignado)
    prefixed_urdf = _make_prefixed_urdf(raw_urdf, ns, pkg_share)

    # ── Grupo de nodos del robot (todos bajo el mismo namespace) ─────────────
    # PushRosNamespace convierte 'cmd_vel' → '/robotN/cmd_vel' automáticamente
    # para todos los nodos dentro del GroupAction.
    robot_group = GroupAction([
        PushRosNamespace(ns),

        # 1. Robot State Publisher
        # Lee el URDF prefixado y publica las TF estáticas de los links del robot
        # (base_link, wheel_r, wheel_l, caster). Se alimenta de joint_states para
        # las articulaciones continuas (ruedas).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': prefixed_urdf,
                'use_sim_time': False,
            }]
        ),

        # 2. Simulador Cinemático (kinematic_sim — Parte 1)
        # Integra cmd_vel → pose y calcula wr/wl.
        # x_init/y_init inicializan el integrador en la posición de arranque del robot.
        Node(
            package=pkg,
            executable='part1_kinematic_sim',
            name='robot',
            output='screen',
            parameters=[{
                'x_init':     float(x0),
                'y_init':     float(y0),
                'theta_init': float(th0),
            }]
        ),

        # 3. Localización (localisation — Parte 2)
        # Dead reckoning: wr/wl → odom.
        # Debe inicializarse en la MISMA posición que kinematic_sim para coherencia.
        Node(
            package=pkg,
            executable='part2_localisation',
            name='localisation',
            output='screen',
            parameters=[{
                'x_init':     float(x0),
                'y_init':     float(y0),
                'theta_init': float(th0),
                'tf_prefix':  ns,   # ← necesario para frames únicos en multi-robot
            }]
        ),

        # 4. Joint State Publisher (joint_state_publisher — Parte 2)
        # Acumula ángulos de rueda desde odom → joint_states para animar el URDF.
        Node(
            package=pkg,
            executable='part2_joint_state_publisher',
            name='joint_state_pub',
            output='screen',
            parameters=[{'tf_prefix': ns}]
        ),

        # 5. Coordinate Transform (coordinate_transform — Parte 2)
        # Publica la TF dinámica: robotN/odom → robotN/base_footprint.
        Node(
            package=pkg,
            executable='part2_coordinate_transform',
            name='coordinate_transform',
            output='screen',
            parameters=[{'tf_prefix': ns}]
        ),

        # 6. Generador de Trayectorias (trajectory_generator — Parte 3)
        # Secuencia los waypoints de la figura seleccionada y los envía al controlador.
        Node(
            package=pkg,
            executable='part3_trajectory_generator',
            name='trajectory_set_point_generator',
            output='screen',
            parameters=[{
                'shape':     shp,
                'tf_prefix': ns,
                'x_init':    float(x0),
                'y_init':    float(y0),
            }]
        ),

        # 7. Controlador PID (control — Parte 3)
        # Lazo cerrado: odom + set_point → cmd_vel.
        Node(
            package=pkg,
            executable='part3_control',
            name='controller',
            output='screen',
            parameters=[{
                'target_x': float(tx),
                'target_y': float(ty),
            }]
        ),

        # 8. TF estática map → robotN/odom
        # IMPORTANTE: los argumentos x, y, yaw son SIEMPRE 0.0.
        # Razón: localisation ya inicializa su integrador interno en (x_init, y_init).
        # Si añadiéramos el offset aquí, RViz aplicaría la traslación DOS VECES
        # y el modelo visual se desplazaría del punto matemático del controlador.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', f'{ns}/odom',
            ]
        ),
    ])

    return [robot_group]


# ── Generador de LaunchDescription ───────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        # Argumentos configurables desde la línea de comandos o desde un launch padre
        DeclareLaunchArgument('robot_ns',    default_value='robot1',
                              description='Namespace del robot y prefijo de TF'),
        DeclareLaunchArgument('x_init',      default_value='0.0',
                              description='Posición X inicial [m]'),
        DeclareLaunchArgument('y_init',      default_value='0.0',
                              description='Posición Y inicial [m]'),
        DeclareLaunchArgument('theta_init',  default_value='0.0',
                              description='Orientación inicial [rad]'),
        DeclareLaunchArgument('target_x',    default_value='0.0',
                              description='Objetivo X inicial del controlador [m]'),
        DeclareLaunchArgument('target_y',    default_value='0.0',
                              description='Objetivo Y inicial del controlador [m]'),
        DeclareLaunchArgument('shape',       default_value='square',
                              description='Figura de trayectoria: square | triangle | hexagon'),

        # OpaqueFunction resuelve los argumentos en tiempo de launch (no en parseo)
        # permitiendo usarlos como strings en la función _launch_setup.
        OpaqueFunction(function=_launch_setup),
    ])
