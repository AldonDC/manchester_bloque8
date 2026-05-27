import os
from glob import glob

from setuptools import setup

package_name = "puzzlebot_ekf"
python_package = "puzzlebot_ekf"


def collect_model_files(root_dir):
    """Walk models/ and emit (install_dir, [src_files]) pairs for setuptools.

    Needed because the plywood maze ships nested folders (materials/,
    meshes/, textures/) and setuptools data_files does not glob recursively.
    Each subdir becomes its own entry under share/<pkg>/models/...
    """
    entries = []
    for dirpath, _dirnames, filenames in os.walk(root_dir):
        files = [os.path.join(dirpath, f) for f in filenames
                 if not f.startswith('.')]
        if files:
            install_path = os.path.join("share", package_name, dirpath)
            entries.append((install_path, files))
    return entries

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        python_package,
        f"{python_package}.perception",
        f"{python_package}.localisation",
        f"{python_package}.map",
        f"{python_package}.utils",
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
        *collect_model_files("models"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alfonso Diaz",
    maintainer_email="personaldiaz01@gmail.com",
    description="Final Challenge Part 1 — Camera-based EKF localisation with ArUco markers and wheel odometry for the Puzzlebot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector       = puzzlebot_ekf.perception.aruco_detector:main",
            "ekf_node             = puzzlebot_ekf.localisation.ekf_node:main",
            "covariance_publisher = puzzlebot_ekf.localisation.covariance_publisher:main",
            "world_visualizer     = puzzlebot_ekf.localisation.world_visualizer:main",
            "auto_demo            = puzzlebot_ekf.localisation.auto_demo:main",
        ],
    },
)
