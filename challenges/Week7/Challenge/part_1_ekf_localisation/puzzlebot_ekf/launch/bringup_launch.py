"""
bringup_launch — Gazebo Harmonic + Puzzlebot + bridges (Part 1).

Copia 1:1 del bringup_launch.py de Week 6 (que funciona), con la única
diferencia de que el bridge añade /camera, /camera_info y /scan para que
el aruco_detector y la EKF puedan consumirlos.
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess,
    OpaqueFunction, TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_world(context):
    """Path absoluto al world, probando paquetes locales y MCR2."""
    world_name = LaunchConfiguration('world').perform(context)
    candidates = [
        os.path.join(get_package_share_directory('puzzlebot_ekf'),
                     'worlds', world_name),
        os.path.join(get_package_share_directory('puzzlebot_gazebo'),
                     'worlds', world_name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise RuntimeError(
        f"World '{world_name}' not found. Tried:\n  " + "\n  ".join(candidates))


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

    # Workaround for the space in '/8 Semestre/...': real copy in /tmp
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
    """Same Week 6 fix: strip URDF <material> and inject SDF blocks in
    <gazebo reference> tags so Ogre2 has shader-compatible materials.
    Nota: el URDF en RViz está deshabilitado (muy pesado: 13 links).
    En su lugar usamos un Marker cilindro rojo desde astar_nav_node.
    Gazebo sí usa el URDF completo (con sus colores originales).
    """
    import re

    urdf = re.sub(r'\s*<material>Gazebo/[^<]+</material>\s*', '\n  ', urdf)
    urdf = re.sub(r'<material\s+name="[^"]*"\s*/>', '', urdf)
    urdf = re.sub(r'<material\s+name="[^"]*">.*?</material>', '', urdf, flags=re.DOTALL)

    link_colors = {
        'base_link':           '0.95 0.85 0.10 1',
        'wheel_left_link':     '0.15 0.15 0.15 1',
        'wheel_right_link':    '0.15 0.15 0.15 1',
        'wheel_caster_link':   '0.15 0.15 0.15 1',
        'caster_holder_link':  '0.15 0.15 0.15 1',
        'jetson_link':         '0.40 0.40 0.45 1',
        'bracket_base_link':   '0.40 0.40 0.45 1',
        'lidar_base_link':     '0.40 0.40 0.45 1',
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

    urdf = re.sub(r'&quot;\s*', '', urdf)
    urdf = urdf.replace('name="Puzzlebot_Jetson_Ed."', 'name="puzzlebot_jetson_lidar_ed"')
    urdf = urdf.replace('name="Puzzlebot_Jetson_Ed"',  'name="puzzlebot_jetson_lidar_ed"')
    urdf = re.sub(r'<gz_frame_id>\s*</gz_frame_id>\s*', '', urdf)
    return urdf


def _build_simulation(context):
    world_path = _resolve_world(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_description = _fix_urdf_for_gazebo_harmonic(_expand_xacro(robot_type))

    # Render: Ogre1 para GUI Y sensors. En software (Mesa+llvmpipe) Ogre1
    # es 2-3x más rápido que Ogre2. -v 1 baja spam de logs (sincronización
    # stderr cuesta CPU).
    #
    # HEADLESS=1 omite la GUI de Gazebo (RViz pinta TODO lo necesario).
    # Esto libera el 40-50% de CPU porque Gazebo GUI hace su propio
    # render Ogre + Qt + scene broadcaster — todo redundante si miras RViz.
    headless = os.environ.get('HEADLESS') == '1'
    if headless:
        gz_cmd = ['gz', 'sim', '-r', '-s', '-v', '1', world_path]  # -s = server only
    else:
        gz_cmd = ['gz', 'sim', '-r', '-v', '1',
                  '--render-engine-gui', 'ogre', world_path]
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

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

    # Bridge: same as Week 6 + add /camera and /camera_info for the EKF.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_ros_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera',      '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Same 3 s delay Week 6 uses so /clock is up before RSP/spawn/bridge.
    delayed = TimerAction(period=3.0, actions=[rsp, spawn, bridge])
    return [gz, delayed]


def generate_launch_description():
    declare_world = DeclareLaunchArgument(
        'world', default_value='ekf_arena.world',
        description='Archivo .world (busca en puzzlebot_ekf y luego en puzzlebot_gazebo)')
    declare_x   = DeclareLaunchArgument('x',   default_value='0.0')
    declare_y   = DeclareLaunchArgument('y',   default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_robot = DeclareLaunchArgument(
        'robot_type', default_value='puzzlebot_jetson_lidar_ed',
        description='Variante del Puzzlebot (con cámara y LiDAR)')

    gazebo_resources_mcr2 = get_package_share_directory('puzzlebot_gazebo')
    ekf_resources = get_package_share_directory('puzzlebot_ekf')
    # ekf models first so our local clean aruco_marker_X wins over MCR2 ones.
    resource_paths = os.pathsep.join([
        os.path.join(ekf_resources, 'models'),
        os.path.join(gazebo_resources_mcr2, 'models'),
    ])
    set_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_paths)
    set_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(gazebo_resources_mcr2, 'plugins'))

    # Render env heredado del shell (run.sh). No forzamos nada aquí.
    sim = OpaqueFunction(function=_build_simulation)

    return LaunchDescription([
        declare_world, declare_x, declare_y, declare_yaw, declare_robot,
        set_resources, set_plugins,
        sim,
    ])
