#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_default_min_confidence(config_path: str) -> str:
    with open(config_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    params = data.get("yolo_pick_pose_node", {}).get("ros__parameters", {})
    return str(params.get("min_confidence", 0.25))


def generate_launch_description():
    manipulation_share = get_package_share_directory("manipulation")
    pick_place_launch = os.path.join(manipulation_share, "launch", "w7_pick_place.launch.py")
    yolo_pick_config = os.path.join(manipulation_share, "config", "yolo_pick_pose.yaml")
    commander_config = os.path.join(manipulation_share, "config", "perception_pick_commander.yaml")
    default_yolo_confidence = _load_default_min_confidence(yolo_pick_config)

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_start_rviz = DeclareLaunchArgument("start_rviz", default_value="true")
    declare_model_path = DeclareLaunchArgument("model_path", default_value="yolov8m.pt")
    declare_yolo_device = DeclareLaunchArgument("yolo_device", default_value="0")
    declare_yolo_confidence = DeclareLaunchArgument("yolo_confidence", default_value=default_yolo_confidence)
    declare_target_class = DeclareLaunchArgument("target_class", default_value="")

    declare_camera_tf_x = DeclareLaunchArgument("camera_tf_x", default_value="1.53886")
    declare_camera_tf_y = DeclareLaunchArgument("camera_tf_y", default_value="-1.34240")
    declare_camera_tf_z = DeclareLaunchArgument("camera_tf_z", default_value="0.81309")
    declare_camera_tf_roll = DeclareLaunchArgument("camera_tf_roll", default_value="-1.88128")
    declare_camera_tf_pitch = DeclareLaunchArgument("camera_tf_pitch", default_value="-0.01250")
    declare_camera_tf_yaw = DeclareLaunchArgument("camera_tf_yaw", default_value="0.78420")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    model_path = LaunchConfiguration("model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    target_class = LaunchConfiguration("target_class")
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
            "--frame-id", "world",
            "--child-frame-id", "station_camera_rgb",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    yolo_detector = Node(
        package="perception",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": "/station_camera/image_raw",
                "output_image_topic": "/detection/image",
                "output_objects_topic": "/detection/objects",
                "model_path": model_path,
                "confidence_threshold": yolo_confidence,
                "device": ParameterValue(yolo_device, value_type=str),
                "max_fps": 5.0,
            }
        ],
    )

    yolo_pick_pose = Node(
        package="manipulation",
        executable="yolo_pick_pose_node",
        name="yolo_pick_pose_node",
        output="screen",
        parameters=[
            yolo_pick_config,
            {
                "use_sim_time": use_sim_time,
                "target_class": target_class,
            },
        ],
        remappings=[
            ("~/pick_pose", "/perception/pick_pose"),
        ],
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
        declare_model_path,
        declare_yolo_device,
        declare_yolo_confidence,
        declare_target_class,
        declare_camera_tf_x,
        declare_camera_tf_y,
        declare_camera_tf_z,
        declare_camera_tf_roll,
        declare_camera_tf_pitch,
        declare_camera_tf_yaw,
        pick_place,
        camera_tf_node,
        yolo_detector,
        yolo_pick_pose,
        commander_node,
    ])
