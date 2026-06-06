#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("manipulation")
    panda_moveit_share = get_package_share_directory("moveit_resources_panda_moveit_config")

    controllers_path = os.path.join(pkg_share, "config", "moveit_isaac_controllers.yaml")
    bridge_config_path = os.path.join(pkg_share, "config", "follow_joint_trajectory_bridge.yaml")
    rviz_config_path = os.path.join(panda_moveit_share, "launch", "moveit.rviz")

    declare_start_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="true이면 RViz MotionPlanning UI를 실행",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Isaac Sim /clock 기준으로 MoveIt2와 RViz를 실행",
    )

    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_panda",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "panda_link0",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        remappings=[("/joint_states", "/franka/joint_states")],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        remappings=[("/joint_states", "/franka/joint_states")],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/joint_states", "/franka/joint_states")],
        condition=IfCondition(start_rviz),
    )

    trajectory_action_bridge = Node(
        package="manipulation",
        executable="follow_joint_trajectory_to_joint_state_server",
        name="follow_joint_trajectory_to_joint_state_server",
        output="screen",
        parameters=[bridge_config_path],
    )

    return LaunchDescription([
        declare_start_rviz,
        declare_use_sim_time,
        static_tf_node,
        robot_state_publisher,
        trajectory_action_bridge,
        move_group_node,
        rviz_node,
    ])
