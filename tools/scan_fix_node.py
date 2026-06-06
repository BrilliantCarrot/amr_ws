#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFixNode(Node):
    def __init__(self):
        super().__init__("scan_fix_node")

        # /scan_raw 받아서 /scan으로 보정해서 내보냄
        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.scan_callback,
            10,
        )

        self.pub = self.create_publisher(
            LaserScan,
            "/scan",
            10,
        )

        self.get_logger().info("scan_fix_node started: /scan_raw -> /scan")

    def scan_callback(self, msg):
        fixed = LaserScan()
        fixed.header = msg.header

        fixed.angle_min = msg.angle_min
        fixed.angle_increment = msg.angle_increment

        # ranges 개수 기준으로 angle_max 다시 계산함
        # slam_toolbox의 expected range count mismatch 방지용임
        n = len(msg.ranges)
        if n > 1:
            fixed.angle_max = fixed.angle_min + fixed.angle_increment * (n - 1)
        else:
            fixed.angle_max = msg.angle_max

        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max


        clean_ranges = []

        for r in msg.ranges:
            # Isaac Sim invalid range(-1.0 등) 제거
            if r < msg.range_min or r > msg.range_max:
                clean_ranges.append(float("inf"))
            else:
                clean_ranges.append(r)

        fixed.ranges = clean_ranges
        

        fixed.intensities = msg.intensities

        self.pub.publish(fixed)


def main():
    rclpy.init()
    node = ScanFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()