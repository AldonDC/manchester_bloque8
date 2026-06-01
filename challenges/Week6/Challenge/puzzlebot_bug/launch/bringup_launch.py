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
    """Expand the robot xacro to a URDF string, handling spaces in paths."""
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
    # crear un real copy bajo /tmp sin espacios.
    real_share = get_package_share_directory('puzzlebot_description')
    if ' ' in real_share:
        copy_dir = '/tmp/puzzlebot_description_share'
        import shutil
        if os.path.exists(copy_dir):
            try:
                if os.path.islink(copy_dir):
                    os.unlink(copy_dir)
                else:
                    shutil.rmtree(copy_dir)
            except Exception:
                pass
        try:
            shutil.copytree(real_share, copy_dir)
        except Exception:
            pass
        urdf = urdf.replace(real_share, copy_dir)
    return urdf


def _fix_urdf_for_gazebo_harmonic(urdf: str) -> str:
    """Fix the URDF so Gazebo Harmonic's Ogre2 renderer can display the robot.

    Root cause of the "invisible robot":
        The xacro defines URDF-level materials like <material name="yellow">
        with RGBA colors. When Gazebo's URDF→SDF converter processes these,
        it generates Ogre1-style fixed-function materials (e.g.
        "Default/TransGreen") that Ogre2 cannot render.

    Fix strategy:
        1. Strip all URDF-level material definitions and references.
        2. Inject proper SDF-compatible <material> blocks with <ambient>/
           <diffuse> RGBA values into <gazebo reference="link_name"> blocks
           for every visual link.
    """
    import re

    # ── 1. Strip legacy script materials and URDF-level materials ───────────
    urdf = re.sub(r'\s*<material>Gazebo/[^<]+</material>\s*', '\n  ', urdf)
    urdf = re.sub(r'<material\s+name="[^"]*"\s*/>', '', urdf)
    urdf = re.sub(r'<material\s+name="[^"]*">.*?</material>', '', urdf, flags=re.DOTALL)

    # ── 2. Inject SDF-compatible <material> blocks ───────────────────────────
    link_colors = {
        'base_link':           '0.95 0.85 0.10 1',   # yellow chassis
        'wheel_left_link':     '0.15 0.15 0.15 1',   # dark grey wheels
        'wheel_right_link':    '0.15 0.15 0.15 1',
        'wheel_caster_link':   '0.15 0.15 0.15 1',
        'caster_holder_link':  '0.15 0.15 0.15 1',
        'jetson_link':         '0.40 0.40 0.45 1',   # grey Jetson
        'bracket_base_link':   '0.40 0.40 0.45 1',   # grey bracket
        'lidar_base_link':     '0.40 0.40 0.45 1',   # grey LiDAR
    }

    for link_name, rgba in link_colors.items():
        material_block = (
            f'\n  <visual>\n'
            f'    <material>\n'
            f'      <ambient>{rgba}</ambient>\n'
            f'      <diffuse>{rgba}</diffuse>\n'
            f'      <specular>0.3 0.3 0.3 1</specular>\n'
            f'    </material>\n'
            f'  </visual>'
        )

        tag = f'<gazebo reference="{link_name}">'
        if tag in urdf:
            urdf = urdf.replace(tag, tag + material_block, 1)
        else:
            urdf = urdf.replace(
                '</robot>',
                f'<gazebo reference="{link_name}">{material_block}\n</gazebo>\n</robot>',
                1,
            )

    # ── 3. Clean up miscellaneous URDF issues ────────────────────────────────
    urdf = re.sub(r'&quot;\s*', '', urdf)
    urdf = urdf.replace('name="Puzzlebot_Jetson_Ed."', 'name="puzzlebot_jetson_lidar_ed"')
    urdf = urdf.replace('name="Puzzlebot_Jetson_Ed"', 'name="puzzlebot_jetson_lidar_ed"')
    urdf = re.sub(r'<gz_frame_id>\s*</gz_frame_id>\s*', '', urdf)

    return urdf



def _build_simulation(context):
    """Construye gz_sim + robot_state_publisher + spawn + bridge."""
    world_path = _resolve_world(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_description = _fix_urdf_for_gazebo_harmonic(_expand_xacro(robot_type))

    # Ejecutamos `gz sim` directo con argv-list (ExecuteProcess), evitando
    # el ros_gz_sim launcher que concatena `gz_args` con espacios y rompe
    # rutas que contienen espacios (como '/8 Semestre/...').
    sw_env = {
        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'GALLIUM_DRIVER': 'llvmpipe',
        'MESA_LOADER_DRIVER_OVERRIDE': 'kms_swrast',
        'OGRE_RTT_MODE': 'Copy',
        'QT_QPA_PLATFORM': 'xcb',
    }
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path],
        additional_env=sw_env,
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
    # Síntoma que arreglamos aquí: el robot se spawnea OK en Gazebo (los
    # topics /scan, /joint_states, /ground_truth publican; DiffDrive
    # responde a /cmd_vel) pero los meshes NO se dibujan en la viewport.
    # En RViz sí se ven porque RViz usa su propio Ogre1 vía glX, no EGL.
    #
    # Causa raíz en esta máquina (híbrida Intel iGPU + NVIDIA dGPU):
    #   "libEGL warning: egl: failed to create dri2 screen"
    #   "GBM platform: eglInitialize failed"
    # Ogre2 (el render engine de Gazebo Harmonic) intenta EGL→GBM por
    # default y eso falla porque Mesa+NVIDIA no negocian GBM bien. La
    # plataforma EGL→X11 sí funciona en esta máquina.
    set_ogre_rtt = SetEnvironmentVariable(
        name='OGRE_RTT_MODE', value='Copy')
    set_qt_xcb = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='xcb')

    # ── Rendering: software (llvmpipe) ───────────────────────────────────────
    # Estado del hardware en esta máquina:
    #   * NVIDIA: driver mal configurado, `nvidia-smi` → "No devices found".
    #   * Intel Alder Lake-P iGPU (PCI 0x28a1): Mesa iris NO la soporta,
    #     tira "MESA: warning: Driver does not support the 0x28a1 PCI ID"
    #     y "libEGL warning: egl: failed to create dri2 screen". Resultado:
    #     Gazebo Harmonic levanta el server pero la GUI Ogre2 nunca crea
    #     un contexto de render → el robot se spawnea (los topics /scan,
    #     /joint_states, /ground_truth publican) pero NO se dibuja en la
    #     viewport. En RViz sí se ve porque RViz usa Ogre1+glX, que
    #     tolera ese estado.
    #
    # Fix: forzar llvmpipe (rasterización por CPU). Es más lento pero
    # garantizado a funcionar, y para Bug 0/Bug 2 con un robot diferencial
    # y un LiDAR a 50 Hz es suficiente. Una vez que NVIDIA se reinstale
    # bien, basta cambiar a __GLX_VENDOR_LIBRARY_NAME=nvidia + el offload
    # PRIME que estaba antes en este archivo.
    #
    #   LIBGL_ALWAYS_SOFTWARE=1       → glX → llvmpipe
    #   GALLIUM_DRIVER=llvmpipe       → confirma gallium llvmpipe
    #   __GLX_VENDOR_LIBRARY_NAME=mesa→ libglvnd toma el ICD Mesa, no NVIDIA
    # NOTA Mesa 23+: LIBGL_ALWAYS_SOFTWARE=1 lo respeta glX pero EGL emite
    # "Not allowed to force software rendering when API explicitly selects
    # a hardware device". Aun así, el server de física + sensores arranca
    # y publica (verificado: /scan a ~23 Hz). La GUI viewport de Gazebo
    # puede quedar pintando solo wireframe o sin meshes. Si Gazebo no
    # consigue render context para nada, prueba a comentar este bloque
    # y arrancar con NVIDIA en lugar de Mesa.
    extra_render_env = [
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='mesa'),
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='GALLIUM_DRIVER', value='llvmpipe'),
        SetEnvironmentVariable(name='MESA_LOADER_DRIVER_OVERRIDE', value='kms_swrast'),
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
