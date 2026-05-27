import os
from glob import glob

from setuptools import setup

package_name = "puzzlebot_nav"
python_package = "puzzlebot_nav"

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
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
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
    description="Final Challenge Part 2 — Multi-waypoint reactive navigation (Bug 0 / Bug 2) on Gazebo Harmonic",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_processor      = puzzlebot_nav.perception.lidar_processor:main",
            "wheel_state_bridge   = puzzlebot_nav.perception.wheel_state_bridge:main",
            "bug0                 = puzzlebot_nav.navigation.bug0:main",
            "bug2                 = puzzlebot_nav.navigation.bug2:main",
            "waypoint_manager     = puzzlebot_nav.navigation.waypoint_manager:main",
            "localisation         = puzzlebot_nav.localisation.localisation:main",
            "coordinate_transform = puzzlebot_nav.localisation.coordinate_transform:main",
            "controller           = puzzlebot_nav.control.controller:main",
            "goal_publisher       = puzzlebot_nav.control.goal_publisher:main",
        ],
    },
)
