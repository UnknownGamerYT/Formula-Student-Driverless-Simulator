from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "fsds_autonomy"

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml", "README.md", "requirements-ml.txt"]),
    (f"share/{package_name}/config", glob("config/*.yaml")),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    (f"share/{package_name}/tools", [str(path) for path in Path("tools").glob("*.py")]),
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hard",
    maintainer_email="hard@example.com",
    description="FSDS-first autonomy stack for mapping, perception, planning, control, and training.",
    license="Apache-License-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = fsds_autonomy.nodes.mission_manager:main",
            "state_estimator = fsds_autonomy.nodes.state_estimator:main",
            "lidar_cone_detector = fsds_autonomy.nodes.lidar_cone_detector:main",
            "camera_detector = fsds_autonomy.nodes.camera_detector:main",
            "sensor_fusion = fsds_autonomy.nodes.sensor_fusion:main",
            "saved_map_publisher = fsds_autonomy.nodes.saved_map_publisher:main",
            "mapper = fsds_autonomy.nodes.mapper:main",
            "raceline_planner = fsds_autonomy.nodes.raceline_planner:main",
            "behavior_planner = fsds_autonomy.nodes.behavior_planner:main",
            "controller = fsds_autonomy.nodes.controller:main",
            "drive_recorder = fsds_autonomy.nodes.drive_recorder:main",
            "dataset_builder = fsds_autonomy.nodes.dataset_builder:main",
            "foxglove_visualizer = fsds_autonomy.nodes.foxglove_visualizer:main",
            "offtrack_reset_monitor = fsds_autonomy.nodes.offtrack_reset_monitor:main",
        ],
    },
)
