#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ReturnToOriginTriggerNode(Node):
    """Publish an AMR return goal after the manipulation commander reports DONE."""

    def __init__(self):
        super().__init__("return_to_origin_trigger_node")

        self.declare_parameter("commander_status_topic", "/perception_pick_commander_node/status")
        self.declare_parameter("done_status", "DONE")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("return_frame", "map")
        self.declare_parameter("return_x", 0.0)
        self.declare_parameter("return_y", 0.0)
        self.declare_parameter("return_yaw", 0.0)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("publish_repeat_count", 5)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("auto_enable", True)

        self.commander_status_topic = self.get_parameter("commander_status_topic").value
        self.done_status = self.get_parameter("done_status").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.return_frame = self.get_parameter("return_frame").value
        self.return_x = float(self.get_parameter("return_x").value)
        self.return_y = float(self.get_parameter("return_y").value)
        self.return_yaw = float(self.get_parameter("return_yaw").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.publish_repeat_count = int(self.get_parameter("publish_repeat_count").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.enabled = bool(self.get_parameter("auto_enable").value)

        self.triggered = False
        self.pending = False
        self.remaining_publishes = 0
        self.last_commander_status = "UNKNOWN"
        self.settle_timer = None
        self.publish_timer = None

        self.status_sub = self.create_subscription(
            String,
            self.commander_status_topic,
            self._on_commander_status,
            10,
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        status_qos = QoSProfile(depth=10)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, "~/status", status_qos)
        self.enable_srv = self.create_service(Trigger, "~/enable", self._on_enable)
        self.reset_srv = self.create_service(Trigger, "~/reset", self._on_reset)
        self.publish_now_srv = self.create_service(Trigger, "~/publish_now", self._on_publish_now)

        self._publish_status("WAITING")
        self.get_logger().info(
            "Return trigger ready: "
            f"status_topic={self.commander_status_topic}, goal_topic={self.goal_topic}, "
            f"return=({self.return_x:.3f}, {self.return_y:.3f}, {self.return_yaw:.3f}rad), "
            f"auto_enable={self.enabled}"
        )

    def _on_enable(self, request, response):
        del request
        self.enabled = True
        self._publish_status("ENABLED")
        response.success = True
        response.message = "return trigger enabled"
        return response

    def _on_reset(self, request, response):
        del request
        self.triggered = False
        self.pending = False
        self.remaining_publishes = 0
        self._cancel_timers()
        self._publish_status("RESET")
        response.success = True
        response.message = "return trigger reset"
        return response

    def _on_publish_now(self, request, response):
        del request
        if self.triggered or self.pending:
            response.success = False
            response.message = "return goal already pending or published"
            return response

        self._schedule_return_goal("manual publish_now service")
        response.success = True
        response.message = "return goal scheduled"
        return response

    def _on_commander_status(self, msg: String):
        previous = self.last_commander_status
        self.last_commander_status = msg.data

        if not self.enabled or self.triggered or self.pending:
            return

        if previous == self.done_status:
            return

        if msg.data == self.done_status:
            self._schedule_return_goal("manipulation commander DONE")

    def _schedule_return_goal(self, reason: str):
        self.pending = True
        self._publish_status("SETTLING")
        self.get_logger().info(f"{reason}; waiting {self.settle_sec:.2f}s before return")

        if self.settle_sec <= 0.0:
            self._begin_publishing()
            return

        self.settle_timer = self.create_timer(self.settle_sec, self._on_settle_done)

    def _on_settle_done(self):
        if self.settle_timer is not None:
            self.destroy_timer(self.settle_timer)
            self.settle_timer = None
        self._begin_publishing()

    def _begin_publishing(self):
        self.remaining_publishes = max(1, self.publish_repeat_count)
        self._publish_status("PUBLISHING_RETURN_GOAL")
        self._publish_return_goal()

        if self.remaining_publishes > 0:
            self.publish_timer = self.create_timer(
                max(0.05, self.publish_period_sec),
                self._on_publish_timer,
            )

    def _on_publish_timer(self):
        self._publish_return_goal()
        if self.remaining_publishes <= 0:
            if self.publish_timer is not None:
                self.destroy_timer(self.publish_timer)
                self.publish_timer = None
            self.pending = False
            self.triggered = True
            self._publish_status("DONE")
            self.get_logger().info("return goal published")

    def _publish_return_goal(self):
        if self.remaining_publishes <= 0:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.return_frame
        msg.pose.position.x = self.return_x
        msg.pose.position.y = self.return_y
        msg.pose.position.z = 0.0

        half_yaw = 0.5 * self.return_yaw
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        self.goal_pub.publish(msg)
        self.remaining_publishes -= 1

    def _cancel_timers(self):
        if self.settle_timer is not None:
            self.destroy_timer(self.settle_timer)
            self.settle_timer = None
        if self.publish_timer is not None:
            self.destroy_timer(self.publish_timer)
            self.publish_timer = None

    def _publish_status(self, status: str):
        msg = String()
        msg.data = (
            f"{status}; enabled={self.enabled}; triggered={self.triggered}; "
            f"last_commander_status={self.last_commander_status}"
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToOriginTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
