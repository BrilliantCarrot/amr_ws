#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray


class CameraObstacleNode(Node):
    def __init__(self):
        super().__init__("camera_obstacle_node")

        # 입력: base_link 기준 3D detection 결과
        self.declare_parameter("input_topic", "/detection/objects_3d_base")

        # 출력 1: 다른 노드가 쓰기 쉬운 장애물 위치 목록
        self.declare_parameter("output_obstacles_topic", "/obstacles/camera_detected")

        # 출력 2: RViz 시각화용 marker
        self.declare_parameter("output_markers_topic", "/obstacles/camera_markers")

        # 카메라 detection 중 어떤 class를 장애물로 볼지 설정함
        self.declare_parameter("target_classes", ["person", "box", "bottle", "chair"])

        # 너무 가깝거나 너무 먼 detection은 제외함
        self.declare_parameter("min_x", 0.1)
        self.declare_parameter("max_x", 8.0)

        # 로봇 좌우로 너무 멀리 떨어진 객체는 일단 제외함
        self.declare_parameter("max_abs_y", 3.0)

        # confidence가 너무 낮은 detection은 제외함
        self.declare_parameter("min_confidence", 0.45)

        # 장애물 marker 크기
        self.declare_parameter("marker_radius", 0.25)

        # 로그 출력 주기
        self.declare_parameter("log_period", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_obstacles_topic = self.get_parameter("output_obstacles_topic").value
        self.output_markers_topic = self.get_parameter("output_markers_topic").value

        self.target_classes = list(self.get_parameter("target_classes").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.max_abs_y = float(self.get_parameter("max_abs_y").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.marker_radius = float(self.get_parameter("marker_radius").value)
        self.log_period = float(self.get_parameter("log_period").value)

        self.last_log_time = 0.0

        self.detection_sub = self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self.detection_callback,
            10,
        )

        self.obstacle_pub = self.create_publisher(
            PoseArray,
            self.output_obstacles_topic,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.output_markers_topic,
            10,
        )

        self.get_logger().info("Camera obstacle node started")
        self.get_logger().info(f"Subscribe: {self.input_topic}")
        self.get_logger().info(f"Publish obstacles: {self.output_obstacles_topic}")
        self.get_logger().info(f"Publish markers  : {self.output_markers_topic}")
        self.get_logger().info(f"Target classes   : {self.target_classes}")

    def detection_callback(self, msg: Detection3DArray):
        obstacle_msg = PoseArray()
        obstacle_msg.header = msg.header
        obstacle_msg.header.frame_id = "base_link"

        marker_array = MarkerArray()

        valid_count = 0

        for det in msg.detections:
            if len(det.results) == 0:
                continue

            class_id = det.results[0].hypothesis.class_id
            score = float(det.results[0].hypothesis.score)

            if class_id not in self.target_classes:
                continue

            if score < self.min_confidence:
                continue

            p = det.bbox.center.position

            x = float(p.x)
            y = float(p.y)
            z = float(p.z)

            # base_link 기준에서 x는 로봇 전방 거리로 사용함
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            if x < self.min_x or x > self.max_x:
                continue

            if abs(y) > self.max_abs_y:
                continue

            pose = Pose()
            pose.position.x = x
            pose.position.y = y

            # AMR 회피에서는 바닥 평면 장애물로 다루기 위해 z는 0으로 둠
            # 실제 detection 높이는 marker text나 debug에서 확인 가능함
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            obstacle_msg.poses.append(pose)

            marker_array.markers.append(
                self.make_obstacle_marker(
                    marker_id=valid_count,
                    class_id=class_id,
                    score=score,
                    x=x,
                    y=y,
                    z=0.0,
                    stamp=msg.header.stamp,
                )
            )

            marker_array.markers.append(
                self.make_text_marker(
                    marker_id=1000 + valid_count,
                    class_id=class_id,
                    score=score,
                    x=x,
                    y=y,
                    z=0.45,
                    stamp=msg.header.stamp,
                )
            )

            valid_count += 1

        # 이전 marker가 남아있는 현상을 줄이기 위해 빈 결과일 때 delete marker를 추가함
        if valid_count == 0:
            marker_array.markers.append(self.make_delete_all_marker(msg.header.stamp))

        self.obstacle_pub.publish(obstacle_msg)
        self.marker_pub.publish(marker_array)

        now = time.time()
        if now - self.last_log_time >= self.log_period:
            self.get_logger().info(
                f"Camera obstacles: {valid_count} | "
                f"frame={obstacle_msg.header.frame_id}"
            )

            for i, pose in enumerate(obstacle_msg.poses):
                self.get_logger().info(
                    f"  obstacle[{i}] base_xy=({pose.position.x:.3f}, "
                    f"{pose.position.y:.3f}) m"
                )

            self.last_log_time = now

    def make_obstacle_marker(self, marker_id, class_id, score, x, y, z, stamp):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacles"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.marker_radius * 2.0
        marker.scale.y = self.marker_radius * 2.0
        marker.scale.z = 0.30

        # 색상은 사람과 일반 물체를 구분하기 위함임
        if class_id == "person":
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8
        else:
            marker.color.r = 0.2
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.8

        marker.lifetime.sec = 1
        return marker

    def make_text_marker(self, marker_id, class_id, score, x, y, z, stamp):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacle_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"{class_id} {score:.2f}"
        marker.lifetime.sec = 1
        return marker

    def make_delete_all_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = stamp
        marker.ns = "camera_obstacles"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CameraObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()