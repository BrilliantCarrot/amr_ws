#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from amr_msgs.msg import ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ObstacleMarkerNode(Node):
    def __init__(self):
        super().__init__("obstacle_marker_node")

        self.declare_parameter("obstacle_topic", "/obstacles/detected")
        self.declare_parameter("marker_topic", "/obstacles/markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("velocity_scale", 2.0)

        obstacle_topic = self.get_parameter("obstacle_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.prev_marker_count = 0

        self.sub = self.create_subscription(
            ObstacleArray,
            obstacle_topic,
            self.obstacle_callback,
            10,
        )

        self.pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.get_logger().info(
            f"Obstacle marker node started: {obstacle_topic} -> {marker_topic}"
        )

    def obstacle_callback(self, msg: ObstacleArray):
        marker_array = MarkerArray()

        frame_id = msg.header.frame_id if msg.header.frame_id else self.get_parameter("frame_id").value
        stamp = self.get_clock().now().to_msg()

        n = int(msg.count)

        for i in range(n):
            x = msg.x[i]
            y = msg.y[i]
            r = msg.radius[i]
            vx = msg.vx[i]
            vy = msg.vy[i]

            # 장애물 원기둥 마커임
            obs_marker = Marker()
            obs_marker.header.frame_id = frame_id
            obs_marker.header.stamp = stamp
            obs_marker.ns = "detected_obstacles"
            obs_marker.id = i
            obs_marker.type = Marker.CYLINDER
            obs_marker.action = Marker.ADD

            obs_marker.pose.position.x = float(x)
            obs_marker.pose.position.y = float(y)
            obs_marker.pose.position.z = 0.05
            obs_marker.pose.orientation.w = 1.0

            obs_marker.scale.x = float(2.0 * r)
            obs_marker.scale.y = float(2.0 * r)
            obs_marker.scale.z = 0.10

            obs_marker.color.r = 1.0
            obs_marker.color.g = 0.2
            obs_marker.color.b = 0.1
            obs_marker.color.a = 0.65

            marker_array.markers.append(obs_marker)

            # 속도 벡터 화살표 마커임
            vel_marker = Marker()
            vel_marker.header.frame_id = frame_id
            vel_marker.header.stamp = stamp
            vel_marker.ns = "obstacle_velocity"
            vel_marker.id = 1000 + i
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD

            start = Point()
            start.x = float(x)
            start.y = float(y)
            start.z = 0.20

            end = Point()
            end.x = float(x + self.velocity_scale * vx)
            end.y = float(y + self.velocity_scale * vy)
            end.z = 0.20

            vel_marker.points.append(start)
            vel_marker.points.append(end)

            vel_marker.scale.x = 0.04
            vel_marker.scale.y = 0.08
            vel_marker.scale.z = 0.08

            vel_marker.color.r = 0.1
            vel_marker.color.g = 0.4
            vel_marker.color.b = 1.0
            vel_marker.color.a = 0.9

            marker_array.markers.append(vel_marker)

            # obstacle id 텍스트 마커임
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = "obstacle_id"
            text_marker.id = 2000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.45
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.18
            text_marker.text = f"obs {i}\nr={r:.2f}\nv=({vx:.2f},{vy:.2f})"

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        # 이전보다 장애물 수가 줄었을 때 남은 마커 삭제함
        for i in range(n, self.prev_marker_count):
            for ns, offset in [
                ("detected_obstacles", 0),
                ("obstacle_velocity", 1000),
                ("obstacle_id", 2000),
            ]:
                delete_marker = Marker()
                delete_marker.header.frame_id = frame_id
                delete_marker.header.stamp = stamp
                delete_marker.ns = ns
                delete_marker.id = offset + i
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

        self.prev_marker_count = n
        self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()