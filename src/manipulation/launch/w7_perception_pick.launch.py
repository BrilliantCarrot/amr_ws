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
    pick_place_launch = os.path.join(pkg_share, "launch", "w7_pick_place.launch.py")
    detector_config = os.path.join(pkg_share, "config", "red_cube_detector.yaml")
    commander_config = os.path.join(pkg_share, "config", "perception_pick_commander.yaml")

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
    declare_detector_transform_mode = DeclareLaunchArgument(
        "detector_transform_mode",
        default_value="manual",
        description="red detector pick pose transform mode: manual or tf",
    )
    declare_planning_frame = DeclareLaunchArgument(
        "planning_frame",
        default_value="world",
        description="MoveIt planning frame used for pick poses",
    )
    declare_camera_frame = DeclareLaunchArgument(
        "camera_frame",
        default_value="station_camera_rgb",
        description="Camera optical frame used when image header frame_id is empty",
    )
    declare_start_camera_tf = DeclareLaunchArgument(
        "start_camera_tf",
        default_value="false",
        description="true이면 world->camera_frame static TF를 함께 publish",
    )
    declare_camera_tf_x = DeclareLaunchArgument("camera_tf_x", default_value="1.53886")
    declare_camera_tf_y = DeclareLaunchArgument("camera_tf_y", default_value="-1.34240")
    declare_camera_tf_z = DeclareLaunchArgument("camera_tf_z", default_value="0.81309")
    declare_camera_tf_roll = DeclareLaunchArgument("camera_tf_roll", default_value="-1.88128")
    declare_camera_tf_pitch = DeclareLaunchArgument("camera_tf_pitch", default_value="-0.01250")
    declare_camera_tf_yaw = DeclareLaunchArgument("camera_tf_yaw", default_value="0.78420")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    detector_transform_mode = LaunchConfiguration("detector_transform_mode")
    planning_frame = LaunchConfiguration("planning_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    start_camera_tf = LaunchConfiguration("start_camera_tf")
    camera_tf_x = LaunchConfiguration("camera_tf_x")
    camera_tf_y = LaunchConfiguration("camera_tf_y")
    camera_tf_z = LaunchConfiguration("camera_tf_z")
    camera_tf_roll = LaunchConfiguration("camera_tf_roll")
    camera_tf_pitch = LaunchConfiguration("camera_tf_pitch")
    camera_tf_yaw = LaunchConfiguration("camera_tf_yaw")

    pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pick_place_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_rviz": start_rviz,
            "auto_start": "false",
            "start_supervisor": "false",
        }.items(),
    )

    detector_node = Node(
        package="manipulation",
        executable="red_cube_detector_node",
        name="red_cube_detector_node",
        output="screen",
        parameters=[
            detector_config,
            {
                "use_sim_time": use_sim_time,
                "transform_mode": detector_transform_mode,
                "planning_frame": planning_frame,
                "camera_frame_fallback": camera_frame,
            },
        ],
        remappings=[
            ("~/pick_pose_candidate", "/perception/pick_pose"),
        ],
    )

    camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_station_camera",
        output="log",
        arguments=[
            "--x", camera_tf_x,
            "--y", camera_tf_y,
            "--z", camera_tf_z,
            "--roll", camera_tf_roll,
            "--pitch", camera_tf_pitch,
            "--yaw", camera_tf_yaw,
            "--frame-id", planning_frame,
            "--child-frame-id", camera_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_camera_tf),
    )

    commander_node = Node(
        package="manipulation",
        executable="perception_pick_commander_node",
        name="perception_pick_commander_node",
        output="screen",
        parameters=[
            commander_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_rviz,
        declare_detector_transform_mode,
        declare_planning_frame,
        declare_camera_frame,
        declare_start_camera_tf,
        declare_camera_tf_x,
        declare_camera_tf_y,
        declare_camera_tf_z,
        declare_camera_tf_roll,
        declare_camera_tf_pitch,
        declare_camera_tf_yaw,
        pick_place,
        camera_tf_node,
        detector_node,
        commander_node,
    ])
