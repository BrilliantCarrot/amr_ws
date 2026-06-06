#!/usr/bin/env python3

from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class PerceptionPickCommanderNode(Node):
    """Run pick_place_node from a generic perception PoseStamped input."""

    def __init__(self):
        super().__init__("perception_pick_commander_node")

        self.declare_parameter("pick_pose_topic", "/perception/pick_pose")
        self.declare_parameter("pick_place_node_name", "/pick_place_node")
        self.declare_parameter("pose_param_names", ["pre_pick_pose", "pick_pose"])
        self.declare_parameter("sequence_steps", [
            "open_gripper",
            "go_pre_pick",
            "close_gripper",
            "go_lift",
        ])
        self.declare_parameter("update_lift_pose", False)
        self.declare_parameter("lift_z", 0.50)
        self.declare_parameter("max_pose_age_sec", 2.0)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("step_timeout_sec", 20.0)

        self.pick_pose_topic = self.get_parameter("pick_pose_topic").value
        self.pick_place_node_name = self.get_parameter("pick_place_node_name").value.rstrip("/")
        self.pose_param_names = list(self.get_parameter("pose_param_names").value)
        self.sequence_steps = list(self.get_parameter("sequence_steps").value)
        self.update_lift_pose = bool(self.get_parameter("update_lift_pose").value)
        self.lift_z = float(self.get_parameter("lift_z").value)
        self.max_pose_age_sec = float(self.get_parameter("max_pose_age_sec").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.step_timeout_sec = float(self.get_parameter("step_timeout_sec").value)

        self.latest_pose: Optional[PoseStamped] = None
        self.running = False
        self.current_step_index = 0
        self.waiting_for_step_done = False
        self.step_deadline = self.get_clock().now()

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pick_pose_topic, self._on_pick_pose, 10
        )
        self.done_sub = self.create_subscription(
            Bool, f"{self.pick_place_node_name}/done", self._on_pick_place_done, 10
        )
        self.status_sub = self.create_subscription(
            String, f"{self.pick_place_node_name}/status", self._on_pick_place_status, 10
        )
        status_qos = QoSProfile(depth=10)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, "~/status", status_qos)
        self.start_srv = self.create_service(Trigger, "~/start", self._on_start)

        self.param_client = self.create_client(
            SetParameters, f"{self.pick_place_node_name}/set_parameters"
        )
        self.step_clients: Dict[str, rclpy.client.Client] = {
            name: self.create_client(Trigger, f"{self.pick_place_node_name}/{name}")
            for name in self.sequence_steps
        }

        self.timer = self.create_timer(0.10, self._on_timer)
        self.get_logger().info(
            "Perception pick commander ready: "
            f"pose_topic={self.pick_pose_topic}, pick_place_node={self.pick_place_node_name}"
        )
        self._publish_status("IDLE")

    def _on_pick_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _on_pick_place_done(self, msg: Bool):
        if not self.running or not self.waiting_for_step_done or not msg.data:
            return

        finished_step = self.sequence_steps[self.current_step_index]
        self.get_logger().info(f"pick_place step done: {finished_step}")
        self.current_step_index += 1
        self.waiting_for_step_done = False
        self._start_next_step()

    def _on_pick_place_status(self, msg: String):
        if self.running and msg.data.startswith("FAILED"):
            self.get_logger().error(f"pick_place_node failed: {msg.data}")
            self.running = False
            self.waiting_for_step_done = False
            self._publish_status(f"FAILED:{msg.data}")

    def _on_start(self, request, response):
        del request
        if self.running:
            response.success = False
            response.message = "perception pick sequence already running"
            return response

        if self.latest_pose is None:
            response.success = False
            response.message = "no pick pose received yet"
            return response

        pose_age = self._pose_age_sec(self.latest_pose)
        if pose_age is not None and pose_age > self.max_pose_age_sec:
            response.success = False
            response.message = f"latest pick pose is too old: {pose_age:.2f}s"
            return response

        if not self._wait_for_clients():
            response.success = False
            response.message = "pick_place_node services are not available"
            return response

        self.running = True
        self.current_step_index = 0
        self.waiting_for_step_done = False
        self._publish_status("UPDATING_PICK_POSE")
        self._set_pick_place_pose_params(self.latest_pose)

        response.success = True
        response.message = "perception pick sequence started"
        return response

    def _wait_for_clients(self) -> bool:
        if not self.param_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            self.get_logger().error(f"service not available: {self.param_client.srv_name}")
            return False

        for name, client in self.step_clients.items():
            if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error(f"service not available for step {name}: {client.srv_name}")
                return False
        return True

    def _set_pick_place_pose_params(self, pose_msg: PoseStamped):
        request = SetParameters.Request()
        pose_values = self._pose_to_param_values(pose_msg)
        names = list(self.pose_param_names)
        if self.update_lift_pose:
            names.append("lift_pose")

        for name in names:
            values = pose_values
            if name == "lift_pose":
                values = list(pose_values)
                values[2] = self.lift_z
            request.parameters.append(self._double_array_param(name, values))

        future = self.param_client.call_async(request)
        future.add_done_callback(self._on_pose_params_set)

    def _on_pose_params_set(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"failed to set pick_place_node params: {exc}")
            self.running = False
            self._publish_status("FAILED:SET_PARAMETERS_EXCEPTION")
            return

        failed = [item.reason for item in result.results if not item.successful]
        if failed:
            self.get_logger().error(f"pick_place_node rejected params: {failed}")
            self.running = False
            self._publish_status(f"FAILED:SET_PARAMETERS:{failed[0]}")
            return

        self.get_logger().info(f"updated pose params: {self.pose_param_names}")
        self._publish_status("RUNNING")
        self._start_next_step()

    def _start_next_step(self):
        if self.current_step_index >= len(self.sequence_steps):
            self.running = False
            self.waiting_for_step_done = False
            self._publish_status("DONE")
            self.get_logger().info("perception pick sequence done")
            return

        step_name = self.sequence_steps[self.current_step_index]
        client = self.step_clients[step_name]
        self._publish_status(f"STEP:{step_name}")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done_future: self._on_step_service_result(done_future, step_name))

    def _on_step_service_result(self, future, step_name: str):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"{step_name}: service call failed: {exc}")
            self.running = False
            self.waiting_for_step_done = False
            self._publish_status(f"FAILED:{step_name}:SERVICE_EXCEPTION")
            return

        if not result.success:
            self.get_logger().error(f"{step_name}: service rejected: {result.message}")
            self.running = False
            self.waiting_for_step_done = False
            self._publish_status(f"FAILED:{step_name}:SERVICE_REJECTED")
            return

        self.get_logger().info(f"{step_name}: accepted")
        self.waiting_for_step_done = True
        self.step_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.step_timeout_sec
        )

    def _on_timer(self):
        if not self.running or not self.waiting_for_step_done:
            return

        if self.get_clock().now() <= self.step_deadline:
            return

        step_name = self.sequence_steps[self.current_step_index]
        self.get_logger().error(f"{step_name}: timed out waiting for /done")
        self.running = False
        self.waiting_for_step_done = False
        self._publish_status(f"FAILED:{step_name}:TIMEOUT")

    def _pose_age_sec(self, pose_msg: PoseStamped) -> Optional[float]:
        stamp = pose_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        age = self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
        return age.nanoseconds / 1e9

    @staticmethod
    def _pose_to_param_values(pose_msg: PoseStamped) -> List[float]:
        pose = pose_msg.pose
        return [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]

    @staticmethod
    def _double_array_param(name: str, values: List[float]) -> Parameter:
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param.value.double_array_value = [float(value) for value in values]
        return param

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPickCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
