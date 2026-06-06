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
    perception_pick_launch = os.path.join(pkg_share, "launch", "w7_perception_pick.launch.py")
    return_config = os.path.join(pkg_share, "config", "return_to_origin_trigger.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Isaac Sim /clock 기준으로 실행",
    )
    declare_start_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="true이면 RViz MotionPlanning UI를 실행",
    )
    declare_start_return_trigger = DeclareLaunchArgument(
        "start_return_trigger",
        default_value="true",
        description="true이면 manipulation DONE 후 /goal_pose 복귀 goal을 발행",
    )
    declare_return_x = DeclareLaunchArgument("return_x", default_value="0.0")
    declare_return_y = DeclareLaunchArgument("return_y", default_value="0.0")
    declare_return_yaw = DeclareLaunchArgument("return_yaw", default_value="0.0")
    declare_detector_transform_mode = DeclareLaunchArgument(
        "detector_transform_mode",
        default_value="manual",
        description="red detector pick pose transform mode: manual or tf",
    )
    declare_start_camera_tf = DeclareLaunchArgument(
        "start_camera_tf",
        default_value="false",
        description="true이면 world->station camera static TF를 함께 publish",
    )
    declare_camera_tf_x = DeclareLaunchArgument("camera_tf_x", default_value="1.53886")
    declare_camera_tf_y = DeclareLaunchArgument("camera_tf_y", default_value="-1.34240")
    declare_camera_tf_z = DeclareLaunchArgument("camera_tf_z", default_value="0.81309")
    declare_camera_tf_roll = DeclareLaunchArgument("camera_tf_roll", default_value="-1.88128")
    declare_camera_tf_pitch = DeclareLaunchArgument("camera_tf_pitch", default_value="-0.01250")
    declare_camera_tf_yaw = DeclareLaunchArgument("camera_tf_yaw", default_value="0.78420")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_return_trigger = LaunchConfiguration("start_return_trigger")
    return_x = LaunchConfiguration("return_x")
    return_y = LaunchConfiguration("return_y")
    return_yaw = LaunchConfiguration("return_yaw")
    detector_transform_mode = LaunchConfiguration("detector_transform_mode")
    start_camera_tf = LaunchConfiguration("start_camera_tf")
    camera_tf_x = LaunchConfiguration("camera_tf_x")
    camera_tf_y = LaunchConfiguration("camera_tf_y")
    camera_tf_z = LaunchConfiguration("camera_tf_z")
    camera_tf_roll = LaunchConfiguration("camera_tf_roll")
    camera_tf_pitch = LaunchConfiguration("camera_tf_pitch")
    camera_tf_yaw = LaunchConfiguration("camera_tf_yaw")

    perception_pick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_pick_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_rviz": start_rviz,
            "detector_transform_mode": detector_transform_mode,
            "start_camera_tf": start_camera_tf,
            "camera_tf_x": camera_tf_x,
            "camera_tf_y": camera_tf_y,
            "camera_tf_z": camera_tf_z,
            "camera_tf_roll": camera_tf_roll,
            "camera_tf_pitch": camera_tf_pitch,
            "camera_tf_yaw": camera_tf_yaw,
        }.items(),
    )

    return_trigger = Node(
        package="manipulation",
        executable="return_to_origin_trigger_node",
        name="return_to_origin_trigger_node",
        output="screen",
        parameters=[
            return_config,
            {
                "use_sim_time": use_sim_time,
                "return_x": return_x,
                "return_y": return_y,
                "return_yaw": return_yaw,
            },
        ],
        condition=IfCondition(start_return_trigger),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_rviz,
        declare_start_return_trigger,
        declare_return_x,
        declare_return_y,
        declare_return_yaw,
        declare_detector_transform_mode,
        declare_start_camera_tf,
        declare_camera_tf_x,
        declare_camera_tf_y,
        declare_camera_tf_z,
        declare_camera_tf_roll,
        declare_camera_tf_pitch,
        declare_camera_tf_yaw,
        perception_pick,
        return_trigger,
    ])
