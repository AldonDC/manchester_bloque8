import os
from glob import glob

from setuptools import setup

# Nombre del paquete ROS (debe coincidir con package.xml).
package_name = "puzzlebot_bug_w6"

# Paquete Python interno
python_package = "puzzlebot_bug"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        python_package,
        f"{python_package}.perception",
        f"{python_package}.navigation",
        f"{python_package}.localisation",
        f"{python_package}.control",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.world")) +
            glob(os.path.join("worlds", "*.sdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alfonso Diaz",
    maintainer_email="personaldiaz01@gmail.com",
    description="Mini Challenge — Reactive Navigation (Bug 0 / Bug 2) on Gazebo Harmonic",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ── Perception ─────────────────────────────────────────────────
            "lidar_processor      = puzzlebot_bug.perception.lidar_processor:main",
            "wheel_state_bridge   = puzzlebot_bug.perception.wheel_state_bridge:main",
            # ── Navigation (núcleo del reto) ──────────────────────────────
            "bug0              = puzzlebot_bug.navigation.bug0:main",
            "bug2              = puzzlebot_bug.navigation.bug2:main",
            # ── Localisation (reutilizado del Reto 4) ─────────────────────
            "localisation      = puzzlebot_bug.localisation.localisation:main",
            "coordinate_transform = puzzlebot_bug.localisation.coordinate_transform:main",
            # ── Control ────────────────────────────────────────────────────
            "controller        = puzzlebot_bug.control.controller:main",
            "goal_publisher    = puzzlebot_bug.control.goal_publisher:main",
        ],
    },
)
