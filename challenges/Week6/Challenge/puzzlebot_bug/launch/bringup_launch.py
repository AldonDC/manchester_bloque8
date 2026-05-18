"""
bringup_launch — Levanta Gazebo Harmonic + Puzzlebot + bridges
================================================================
Mini Challenge · Reactive Navigation · Week 6

Este launch es el "bottom half" del sistema: arranca el simulador con un
mundo elegible, spawnea el Puzzlebot con LiDAR y abre los puentes ROS↔Gazebo.

Lo USA quien quiera correr el robot en Gazebo. Los launches de Bug 0 y
Bug 2 lo incluyen y le agregan los nodos de navegación arriba.

Argumentos:
    world          nombre del archivo .world  (default 'bug_easy.world')
    x, y, yaw      pose inicial del robot     (default 0.0)
    robot_type     tipo de Puzzlebot          (default 'puzzlebot_jetson_lidar_ed')

Nota técnica sobre rutas con espacios:
    La ruta del workspace contiene un espacio ("8 Semestre"). La función
    launch.substitutions.Command pasa el comando por shlex.split, lo que
    rompe la ruta. Por eso ejecutamos `xacro` con subprocess al tiempo de
    generación del launch description y pasamos el resultado como string
    estática al parámetro `robot_description`.
"""

import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess,
    OpaqueFunction, TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _resolve_world(context):
    """Devuelve el path absoluto al world, probando varios paquetes."""
    world_name = LaunchConfiguration('world').perform(context)
    candidates = [
        os.path.join(get_package_share_directory('puzzlebot_bug_w6'),
                     'worlds', world_name),
        os.path.join(get_package_share_directory('puzzlebot_gazebo'),
                     'worlds', world_name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise RuntimeError(
        f"World '{world_name}' no encontrado. Probé:\n  " + "\n  ".join(candidates)
    )


def _expand_xacro(robot_type: str) -> str:
    """
    Ejecuta `xacro` sobre el .xacro correspondiente y devuelve el URDF
    expandido como string. Lo hacemos vía subprocess para evitar el problema
    de shlex.split con rutas que contienen espacios.
    """
    pkg_desc = get_package_share_directory('puzzlebot_description')
    xacro_path = os.path.join(
        pkg_desc, 'urdf', 'mcr2_robots', f'{robot_type}.xacro')
    if not os.path.isfile(xacro_path):
        raise RuntimeError(f"No existe el xacro: {xacro_path}")

    try:
        urdf = subprocess.check_output([
            'xacro', xacro_path,
            'prefix:=',
            'lidar_frame:=laser_frame',
            'camera_frame:=camera_link_optical',
            'tof_frame:=tof_link',
        ], text=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro falló: {e}") from e

    # Workaround para el espacio en '/8 Semestre/...':
    # Gazebo no soporta espacios crudos NI URL-encoded en file:// URIs;
    # RViz acepta %20 pero no espacios crudos. La solución limpia es
    # crear un symlink sin espacios apuntando al install/share del paquete
    # de descripción, y reescribir los paths en el URDF para usarlo.
    real_share = get_package_share_directory('puzzlebot_description')
    if ' ' in real_share:
        symlink = '/tmp/puzzlebot_description_share'
        # Recrear el symlink si no existe o apunta a otro lado
        if (not os.path.islink(symlink) or
                os.readlink(symlink) != real_share):
            try:
                if os.path.islink(symlink):
                    os.unlink(symlink)
                os.symlink(real_share, symlink)
            except OSError:
                pass  # ya existe, lo dejamos
        urdf = urdf.replace(real_share, symlink)
    return urdf


def _build_simulation(context):
    """Construye gz_sim + robot_state_publisher + spawn + bridge."""
    world_path = _resolve_world(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_description = _expand_xacro(robot_type)

    # Ejecutamos `gz sim` directo con argv-list (ExecuteProcess), evitando
    # el ros_gz_sim launcher que concatena `gz_args` con espacios y rompe
    # rutas que contienen espacios (como '/8 Semestre/...').
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', name='robot_spawner',
        arguments=[
            '-name',  'puzzlebot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-Y', LaunchConfiguration('yaw').perform(context),
        ],
        output='screen',
    )

    # Bridge ROS↔Gazebo.
    # NOTA: ya no bridgeamos /VelocityEncR,L porque el plugin del profesor
    # que los publicaba (DiffDynamicPlugin) no carga en Gazebo Harmonic.
    # En su lugar, /joint_states (que sí publica el JointStatePublisher
    # built-in de Gazebo) es procesado por wheel_state_bridge para
    # generar /wr y /wl.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_ros_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # robot_state_publisher, spawn y bridge se retrasan 3 s para que Gazebo
    # ya tenga el world cargado y /clock publicándose. Sin este retraso,
    # robot_state_publisher recibe joint_states con timestamps simulados
    # antes de que su propio reloj se sincronice, causando "Moved backwards
    # in time" continuo y reseteo del buffer TF en RViz.
    delayed = TimerAction(period=3.0, actions=[rsp, spawn, bridge])

    return [gz, delayed]


# ─────────────────────────────────────────────────────────────────────────────

def generate_launch_description():
    declare_world = DeclareLaunchArgument(
        'world', default_value='bug_easy.world',
        description='Archivo .world (busca en puzzlebot_bug_w6 y luego en puzzlebot_gazebo)')
    declare_x   = DeclareLaunchArgument('x',   default_value='0.0')
    declare_y   = DeclareLaunchArgument('y',   default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_robot = DeclareLaunchArgument(
        'robot_type', default_value='puzzlebot_jetson_lidar_ed',
        description='Variante del Puzzlebot (necesita LiDAR para Bug)')

    # Variables de entorno: que Gazebo encuentre modelos/plugins del profe.
    gazebo_resources = get_package_share_directory('puzzlebot_gazebo')
    set_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(gazebo_resources, 'models'))
    set_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(gazebo_resources, 'plugins'))

    # ── Workaround de rendering server-side ──────────────────────────────────
    # En equipos donde EGL no negocia bien con la GPU (síntoma:
    #   "libEGL warning: egl: failed to create dri2 screen"
    # y meshes STL del robot que NO se dibujan aunque el spawn diga "OK"),
    # forzamos a Ogre2 a usar Copy-mode para los render targets y a glX
    # como backend GL. Esto NO afecta el cómputo físico ni los sensores;
    # sólo evita el camino EGL→DRI2 que está roto en muchos drivers Intel.
    set_ogre_rtt = SetEnvironmentVariable(
        name='OGRE_RTT_MODE', value='Copy')
    set_qt_xcb = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='xcb')

    # ── Rendering: usar iGPU Intel/Mesa ───────────────────────────────────────
    # Antes intentábamos PRIME render-offload con NVIDIA, pero en esta máquina
    # el driver NVIDIA está mal configurado (`nvidia-smi` falla con "No devices
    # were found"). Mesa/Intel sí funciona. Si NVIDIA se arregla en el futuro,
    # se puede volver al offload exportando __NV_PRIME_RENDER_OFFLOAD=1 y
    # __GLX_VENDOR_LIBRARY_NAME=nvidia antes del launch (también en run.sh).
    extra_render_env = [
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='mesa'),
    ]

    # Todo lo dinámico se construye en runtime con OpaqueFunction porque
    # depende del valor real de los launch arguments.
    sim = OpaqueFunction(function=_build_simulation)

    return LaunchDescription([
        declare_world, declare_x, declare_y, declare_yaw, declare_robot,
        set_resources, set_plugins,
        set_ogre_rtt, set_qt_xcb,
        *extra_render_env,
        sim,
    ])
