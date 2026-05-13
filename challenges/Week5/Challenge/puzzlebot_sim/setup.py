import os
from glob import glob

from setuptools import setup

# Nombre del paquete ROS (debe coincidir con package.xml).
# Sufijo _w5 evita colisión con paquetes de semanas previas dentro del workspace.
package_name = "puzzlebot_sim_w5"

# Paquete Python interno (conserva el nombre original para no tocar imports).
python_package = "puzzlebot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        python_package,
        f"{python_package}.simulation",      # Real/Sim Robot + open-loop driver
        f"{python_package}.localisation",    # ⭐ Dead reckoning + propagación Σₖ
        f"{python_package}.visualisation",   # TF dinámica + joint states
        f"{python_package}.control",         # PID + generador de trayectorias
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[yma]*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.stl")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.urdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alfonso Diaz",
    maintainer_email="personaldiaz01@gmail.com",
    description="Mini Challenge 4 — Puzzlebot sim with pose covariance propagation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        # Los nombres de los executables se conservan (part1_, part2_, part3_)
        # para mantener compatibilidad con los launch files existentes.
        "console_scripts": [
            # ── Simulation ─────────────────────────────────────────────────
            "part1_kinematic_sim         = puzzlebot_sim.simulation.kinematic_sim:main",
            "part2_open_loop_driver      = puzzlebot_sim.simulation.open_loop_driver:main",
            # ── Localisation (núcleo del reto) ─────────────────────────────
            "part2_localisation          = puzzlebot_sim.localisation.localisation:main",
            # ── Visualisation ──────────────────────────────────────────────
            "part2_joint_state_publisher = puzzlebot_sim.visualisation.joint_state_publisher:main",
            "part2_coordinate_transform  = puzzlebot_sim.visualisation.coordinate_transform:main",
            # ── Control ────────────────────────────────────────────────────
            "part3_control               = puzzlebot_sim.control.controller:main",
            "part3_trajectory_generator  = puzzlebot_sim.control.trajectory_generator:main",
        ],
    },
)
