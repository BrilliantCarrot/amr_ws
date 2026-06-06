#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("perception")

    yolo_config = os.path.join(pkg_share, "config", "yolo.yaml")
    object_3d_config = os.path.join(pkg_share, "config", "object_3d.yaml")
    camera_obstacle_config = os.path.join(pkg_share, "config", "camera_obstacle.yaml")    

    # base_link 기준 카메라 optical frame 고정 TF
    # 의미:
    #   카메라는 로봇 기준 앞쪽 위쪽 0.6m에 장착되어 있음
    #   camera_rgb_optical_frame의 z축이 카메라가 보는 전방 방향이 되도록 회전시킴
    static_tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera_rgb_optical",
        output="screen",
        arguments=[
            "--x", "0.08917",
            "--y", "0.0",
            "--z", "0.3265",
            "--roll", "-1.57079632679",
            "--pitch", "0.0",
            "--yaw", "-1.57079632679",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_rgb_optical_frame",
        ],
    )

    yolo_detector_node = Node(
        package="perception",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[yolo_config],
    )

    object_3d_projector_node = Node(
        package="perception",
        executable="object_3d_projector_node",
        name="object_3d_projector_node",
        output="screen",
        parameters=[object_3d_config],
    )

    camera_obstacle_node = Node(
        package="perception",
        executable="camera_obstacle_node",
        name="camera_obstacle_node",
        output="screen",
        parameters=[camera_obstacle_config],
    )    

    return LaunchDescription([
        static_tf_base_to_camera,
        yolo_detector_node,
        object_3d_projector_node,
        camera_obstacle_node,
    ])