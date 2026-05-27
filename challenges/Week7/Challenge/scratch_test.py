import sys
import os
import re
import shutil
from ament_index_python.packages import get_package_share_directory

try:
    # 1. Simulate _expand_xacro with copy tree
    import subprocess
    pkg_desc = get_package_share_directory('puzzlebot_description')
    xacro_path = os.path.join(pkg_desc, 'urdf', 'mcr2_robots', 'puzzlebot_jetson_lidar_ed.xacro')
    
    urdf = subprocess.check_output([
        'xacro', xacro_path,
        'prefix:=',
        'camera_frame:=camera_link_optical',
        'tof_frame:=tof_link',
    ], text=True)
    
    real_share = get_package_share_directory('puzzlebot_description')
    if ' ' in real_share:
        copy_dir = '/tmp/puzzlebot_description_share'
        if os.path.islink(copy_dir):
            try:
                os.unlink(copy_dir)
            except OSError:
                pass
        if not os.path.isdir(copy_dir):
            try:
                shutil.copytree(real_share, copy_dir)
            except Exception as e:
                print(f"Copy failed: {e}")
        urdf = urdf.replace(real_share, copy_dir)

    # 2. Simulate _fix_urdf_for_gazebo_harmonic with material stripping
    urdf = re.sub(r'\s*<material>Gazebo/[^<]+</material>\s*', '\n  ', urdf)
    
    # Strip URDF-level materials
    urdf = re.sub(r'<material\s+name="[^"]*"\s*/>', '', urdf)
    urdf = re.sub(r'<material\s+name="[^"]*">.*?</material>', '', urdf, flags=re.DOTALL)
    
    # Inject PBR materials
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
    urdf = urdf.replace('name="Puzzlebot_Jetson_Ed"', 'name="puzzlebot_jetson_lidar_ed"')
    urdf = re.sub(r'<gz_frame_id>\s*</gz_frame_id>\s*', '', urdf)

    with open('/tmp/test_fixed_urdf.urdf', 'w') as f:
        f.write(urdf)
    print("SUCCESS: Test URDF written to /tmp/test_fixed_urdf.urdf")

except Exception as e:
    import traceback
    traceback.print_exc()
