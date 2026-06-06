#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("manipulation")
    config_path = os.path.join(pkg_share, "config", "red_cube_detector.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Isaac Sim /clock 기준으로 실행",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    detector_node = Node(
        package="manipulation",
        executable="red_cube_detector_node",
        name="red_cube_detector_node",
        output="screen",
        parameters=[
            config_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        detector_node,
    ])
