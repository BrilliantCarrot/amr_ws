#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("manipulation")
    moveit_launch = os.path.join(pkg_share, "launch", "moveit_isaac_franka.launch.py")
    config_path = os.path.join(pkg_share, "config", "pick_place_sequence.yaml")
    supervisor_config_path = os.path.join(pkg_share, "config", "w7_docking_supervisor.yaml")

    declare_auto_start = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="true이면 launch 직후 W7 pick/place 시퀀스를 1회 실행",
    )
    declare_start_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="true이면 RViz MotionPlanning UI를 실행",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Isaac Sim /clock 기준으로 실행",
    )
    declare_start_supervisor = DeclareLaunchArgument(
        "start_supervisor",
        default_value="false",
        description="true이면 AMR docking 후 pick/place를 시작하는 supervisor도 실행",
    )

    auto_start = LaunchConfiguration("auto_start")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_supervisor = LaunchConfiguration("start_supervisor")

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            "start_rviz": start_rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    pick_place_node = Node(
        package="manipulation",
        executable="pick_place_node",
        name="pick_place_node",
        output="screen",
        parameters=[
            config_path,
            {
                "auto_start": auto_start,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    supervisor_node = Node(
        package="manipulation",
        executable="w7_docking_supervisor_node",
        name="w7_docking_supervisor_node",
        output="screen",
        parameters=[
            supervisor_config_path,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_supervisor),
    )

    return LaunchDescription([
        declare_auto_start,
        declare_start_rviz,
        declare_use_sim_time,
        declare_start_supervisor,
        moveit,
        pick_place_node,
        supervisor_node,
    ])
