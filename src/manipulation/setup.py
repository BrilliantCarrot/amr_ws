from setuptools import find_packages, setup
import os
from glob import glob

package_name = "manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    scripts=[
        "scripts/follow_joint_trajectory_to_joint_state_server",
        "scripts/pick_place_node",
        "scripts/perception_pick_commander_node",
        "scripts/red_cube_detector_node",
        "scripts/return_to_origin_trigger_node",
        "scripts/w7_docking_supervisor_node",
        "scripts/yolo_pick_pose_node",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "moveit_config"), glob("moveit_config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyj",
    maintainer_email="leeyj950322@gmail.com",
    description="Stationary Franka bringup and pick-place scaffolding for the Isaac AMR warehouse scenario.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
