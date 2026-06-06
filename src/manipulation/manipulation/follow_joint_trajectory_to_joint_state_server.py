#!/usr/bin/env python3

from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class FollowJointTrajectoryToJointStateServer(Node):
    """Expose a FollowJointTrajectory action and command Isaac Sim through JointState."""

    def __init__(self):
        super().__init__("follow_joint_trajectory_to_joint_state_server")

        self.declare_parameter("action_name", "/panda_arm_controller/follow_joint_trajectory")
        self.declare_parameter("joint_command_topic", "/franka/joint_command")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("goal_tolerance_rad", 0.08)
        self.declare_parameter("hold_final_point", True)
        self.declare_parameter("arm_joint_names", Parameter.Type.STRING_ARRAY)

        self.action_name = self.get_parameter("action_name").value
        self.joint_command_topic = self.get_parameter("joint_command_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.goal_tolerance_rad = float(self.get_parameter("goal_tolerance_rad").value)
        self.hold_final_point = bool(self.get_parameter("hold_final_point").value)
        self.allowed_joint_names = set(self._string_list_param("arm_joint_names"))

        self.command_pub = self.create_publisher(JointState, self.joint_command_topic, 10)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.final_command: Optional[JointState] = None
        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.hold_timer = self.create_timer(timer_period, self._hold_final_command)

        self.get_logger().info(
            f"FollowJointTrajectory bridge ready: {self.action_name} -> {self.joint_command_topic}"
        )

    def _string_list_param(self, name: str) -> List[str]:
        return list(self.get_parameter(name).get_parameter_value().string_array_value)

    def _goal_callback(self, goal_request):
        joint_names = list(goal_request.trajectory.joint_names)
        if not joint_names:
            self.get_logger().warn("rejecting trajectory goal with empty joint_names")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().warn("rejecting trajectory goal with no points")
            return GoalResponse.REJECT

        unknown = [name for name in joint_names if name not in self.allowed_joint_names]
        if unknown:
            self.get_logger().warn(f"rejecting trajectory goal with unknown joints: {unknown}")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"accepted FollowJointTrajectory goal: joints={joint_names}, "
            f"points={len(goal_request.trajectory.points)}"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        self.get_logger().info("trajectory goal cancel requested")
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = list(trajectory.points)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        start_time = self.get_clock().now()
        rate = self.create_rate(max(self.publish_rate_hz, 1.0))
        final_positions = list(points[-1].positions)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.ERROR,
                    "trajectory canceled",
                )

            elapsed = self._duration_to_sec(self.get_clock().now() - start_time)
            sample = self._sample_positions(points, elapsed)
            if sample is None:
                break

            self._publish_command(joint_names, sample)
            feedback.desired.positions = sample
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        self._publish_command(joint_names, final_positions)
        goal_handle.succeed()
        self.get_logger().info("trajectory goal succeeded")
        return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory executed")

    def _sample_positions(
        self,
        points: List[JointTrajectoryPoint],
        elapsed_sec: float,
    ) -> Optional[List[float]]:
        if not points:
            return None

        if len(points) == 1:
            final_t = self._duration_to_sec(points[0].time_from_start)
            if elapsed_sec > final_t:
                return None
            return [float(value) for value in points[0].positions]

        first_t = self._duration_to_sec(points[0].time_from_start)
        if elapsed_sec <= first_t:
            return [float(value) for value in points[0].positions]

        for idx in range(1, len(points)):
            prev = points[idx - 1]
            cur = points[idx]
            prev_t = self._duration_to_sec(prev.time_from_start)
            cur_t = self._duration_to_sec(cur.time_from_start)
            if elapsed_sec <= cur_t:
                span = max(cur_t - prev_t, 1e-6)
                alpha = min(max((elapsed_sec - prev_t) / span, 0.0), 1.0)
                return [
                    float(p0 + alpha * (p1 - p0))
                    for p0, p1 in zip(prev.positions, cur.positions)
                ]

        return None

    def _publish_command(self, joint_names: List[str], positions: List[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(value) for value in positions]
        self.command_pub.publish(msg)
        self.final_command = msg

    def _hold_final_command(self):
        if not self.hold_final_point or self.final_command is None:
            return
        self.final_command.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(self.final_command)

    def _duration_to_sec(self, duration_msg) -> float:
        if hasattr(duration_msg, "nanoseconds"):
            return float(duration_msg.nanoseconds) * 1e-9
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    def _result(self, error_code: int, error_string: str):
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FollowJointTrajectoryToJointStateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
