#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class PoseStep:
    name: str
    pose: Pose


@dataclass(frozen=True)
class GripperStep:
    name: str
    positions: List[float]


class PickPlaceNode(Node):
    """MoveIt2 pose-goal pick/place sequence for the fixed Franka station."""

    def __init__(self):
        super().__init__("pick_place_node")

        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("joint_command_topic", "/franka/joint_command")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("group_name", "panda_arm")
        self.declare_parameter("end_effector_link", "panda_hand")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.20)
        self.declare_parameter("acceleration_scaling", 0.20)
        self.declare_parameter("position_tolerance", 0.025)
        self.declare_parameter("orientation_tolerance", 0.20)
        self.declare_parameter("workspace_min", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("workspace_max", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("gripper_joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("gripper_open_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("gripper_closed_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("gripper_command_repeat_count", 5)
        self.declare_parameter("gripper_command_period_sec", 0.10)
        self.declare_parameter("gripper_settle_sec", 1.00)
        self.declare_parameter("pre_pick_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pick_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("lift_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pre_place_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("place_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("home_pose", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("execute_home_at_start", False)
        self.declare_parameter("execute_home_at_end", True)

        self.move_group_action = self.get_parameter("move_group_action").value
        self.joint_command_topic = self.get_parameter("joint_command_topic").value
        self.group_name = self.get_parameter("group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.pipeline_id = self.get_parameter("pipeline_id").value
        self.planner_id = self.get_parameter("planner_id").value
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.workspace_min = self._float_list_param("workspace_min")
        self.workspace_max = self._float_list_param("workspace_max")
        self.gripper_joint_names = self._string_list_param("gripper_joint_names")
        self.gripper_open_positions = self._float_list_param("gripper_open_positions")
        self.gripper_closed_positions = self._float_list_param("gripper_closed_positions")
        self.gripper_repeat_count = int(self.get_parameter("gripper_command_repeat_count").value)
        self.gripper_period_sec = float(self.get_parameter("gripper_command_period_sec").value)
        self.gripper_settle_sec = float(self.get_parameter("gripper_settle_sec").value)
        self.execute_home_at_start = bool(self.get_parameter("execute_home_at_start").value)
        self.execute_home_at_end = bool(self.get_parameter("execute_home_at_end").value)

        self.pre_pick_pose = self._pose_param("pre_pick_pose")
        self.pick_pose = self._pose_param("pick_pose")
        self.lift_pose = self._pose_param("lift_pose")
        self.pre_place_pose = self._pose_param("pre_place_pose")
        self.place_pose = self._pose_param("place_pose")
        self.home_pose = self._pose_param("home_pose")
        self._validate_config()

        self.move_group_client = ActionClient(self, MoveGroup, self.move_group_action)
        self.gripper_pub = self.create_publisher(JointState, self.joint_command_topic, 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.done_pub = self.create_publisher(Bool, "~/done", 10)
        self.start_srv = self.create_service(Trigger, "~/start", self._on_start)
        self.go_pre_pick_srv = self.create_service(
            Trigger, "~/go_pre_pick", self._make_pose_service("pre_pick")
        )
        self.go_pick_srv = self.create_service(
            Trigger, "~/go_pick", self._make_pose_service("pick")
        )
        self.go_lift_srv = self.create_service(
            Trigger, "~/go_lift", self._make_pose_service("lift")
        )
        self.go_pre_place_srv = self.create_service(
            Trigger, "~/go_pre_place", self._make_pose_service("pre_place")
        )
        self.go_place_srv = self.create_service(
            Trigger, "~/go_place", self._make_pose_service("place")
        )
        self.go_home_srv = self.create_service(
            Trigger, "~/go_home", self._make_pose_service("home")
        )
        self.open_gripper_srv = self.create_service(
            Trigger,
            "~/open_gripper",
            self._make_gripper_service("gripper_open"),
        )
        self.close_gripper_srv = self.create_service(
            Trigger,
            "~/close_gripper",
            self._make_gripper_service("gripper_close"),
        )

        self.sequence = self._make_sequence()
        self.current_step_index = 0
        self.running = False
        self.waiting_for_action = False
        self.pending_gripper_step: Optional[GripperStep] = None
        self.gripper_repeat_remaining = 0
        self.next_gripper_publish_time = self.get_clock().now()
        self.next_step_time = self.get_clock().now()
        self.done_reported = False

        self.timer = self.create_timer(0.02, self._on_timer)
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.get_logger().info(
            f"Pick/place node ready. MoveGroup={self.move_group_action}, "
            f"group={self.group_name}, ee={self.end_effector_link}"
        )
        self._publish_status("IDLE")

        if bool(self.get_parameter("auto_start").value):
            self._start_sequence()

    def _string_list_param(self, name: str) -> List[str]:
        return list(self.get_parameter(name).get_parameter_value().string_array_value)

    def _float_list_param(self, name: str) -> List[float]:
        return list(self.get_parameter(name).get_parameter_value().double_array_value)

    def _pose_param(self, name: str) -> Pose:
        values = self._float_list_param(name)
        if len(values) != 7:
            raise ValueError(f"{name} must contain [x, y, z, qx, qy, qz, qw]")
        pose = Pose()
        pose.position.x = float(values[0])
        pose.position.y = float(values[1])
        pose.position.z = float(values[2])
        pose.orientation.x = float(values[3])
        pose.orientation.y = float(values[4])
        pose.orientation.z = float(values[5])
        pose.orientation.w = float(values[6])
        return pose

    def _validate_config(self):
        if len(self.workspace_min) != 3 or len(self.workspace_max) != 3:
            raise ValueError("workspace_min and workspace_max must have three values")
        if len(self.gripper_open_positions) != len(self.gripper_joint_names):
            raise ValueError("gripper_open_positions length does not match gripper_joint_names")
        if len(self.gripper_closed_positions) != len(self.gripper_joint_names):
            raise ValueError("gripper_closed_positions length does not match gripper_joint_names")
        if self.gripper_repeat_count < 1:
            raise ValueError("gripper_command_repeat_count must be >= 1")

    def _make_sequence(self):
        sequence = []
        if self.execute_home_at_start:
            sequence.append(PoseStep("home_start", self.home_pose))
        sequence.extend([
            GripperStep("gripper_open", self.gripper_open_positions),
            PoseStep("pick", self.pick_pose),
            GripperStep("gripper_close", self.gripper_closed_positions),
            PoseStep("lift", self.lift_pose),
            PoseStep("place", self.place_pose),
            GripperStep("gripper_open", self.gripper_open_positions),
        ])
        if self.execute_home_at_end:
            sequence.append(PoseStep("home_end", self.home_pose))
        return sequence

    def _on_start(self, request, response):
        del request
        if self.running:
            response.success = False
            response.message = "pick/place sequence already running"
            return response

        self._start_sequence()
        response.success = True
        response.message = "pick/place sequence started"
        return response

    def _make_pose_service(self, name: str):
        def callback(request, response):
            del request
            return self._start_single_step(PoseStep(name, self._pose_by_name(name)), response)

        return callback

    def _make_gripper_service(self, name: str):
        def callback(request, response):
            del request
            return self._start_single_step(GripperStep(name, self._gripper_positions_by_name(name)), response)

        return callback

    def _start_single_step(self, step, response):
        if self.running:
            response.success = False
            response.message = "pick/place sequence already running"
            return response

        self.sequence = [step]
        self.current_step_index = 0
        self.running = True
        self.waiting_for_action = False
        self.pending_gripper_step = None
        self.done_reported = False
        self.get_logger().info(f"starting single W7 step: {step.name}")
        self._publish_done(False)
        self._publish_status(f"RUNNING_SINGLE:{step.name}")
        self._start_current_step()
        response.success = True
        response.message = f"{step.name} started"
        return response

    def _pose_by_name(self, name: str) -> Pose:
        poses = {
            "pre_pick": self.pre_pick_pose,
            "pick": self.pick_pose,
            "lift": self.lift_pose,
            "pre_place": self.pre_place_pose,
            "place": self.place_pose,
            "home": self.home_pose,
        }
        return poses[name]

    def _gripper_positions_by_name(self, name: str) -> List[float]:
        positions = {
            "gripper_open": self.gripper_open_positions,
            "gripper_close": self.gripper_closed_positions,
        }
        return positions[name]

    def _on_parameters_changed(self, params):
        pose_param_names = {
            "pre_pick_pose",
            "pick_pose",
            "lift_pose",
            "pre_place_pose",
            "place_pose",
            "home_pose",
        }
        gripper_param_names = {
            "gripper_open_positions",
            "gripper_closed_positions",
        }
        scalar_param_names = {
            "velocity_scaling",
            "acceleration_scaling",
            "position_tolerance",
            "orientation_tolerance",
            "allowed_planning_time",
            "gripper_settle_sec",
        }

        updates = {}
        gripper_updates = {}
        scalar_updates = {}
        for param in params:
            if param.name in pose_param_names:
                if param.type_ != Parameter.Type.DOUBLE_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a double array [x, y, z, qx, qy, qz, qw]",
                    )
                values = list(param.value)
                if len(values) != 7:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must contain 7 values",
                    )
                updates[param.name] = self._pose_from_values(values)
                continue

            if param.name in gripper_param_names:
                if param.type_ != Parameter.Type.DOUBLE_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a double array",
                    )
                values = [float(value) for value in param.value]
                if len(values) != len(self.gripper_joint_names):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{param.name} must contain {len(self.gripper_joint_names)} values"
                        ),
                    )
                gripper_updates[param.name] = values
                continue

            if param.name in scalar_param_names:
                if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be numeric",
                    )
                scalar_updates[param.name] = float(param.value)

        for name, pose in updates.items():
            setattr(self, name, pose)
            self.get_logger().info(f"updated {name}: {self._pose_to_list(pose)}")

        for name, positions in gripper_updates.items():
            setattr(self, name, positions)
            self.get_logger().info(f"updated {name}: {positions}")

        for name, value in scalar_updates.items():
            setattr(self, name, value)
            self.get_logger().info(f"updated {name}: {value}")

        return SetParametersResult(successful=True)

    def _pose_from_values(self, values: List[float]) -> Pose:
        pose = Pose()
        pose.position.x = float(values[0])
        pose.position.y = float(values[1])
        pose.position.z = float(values[2])
        pose.orientation.x = float(values[3])
        pose.orientation.y = float(values[4])
        pose.orientation.z = float(values[5])
        pose.orientation.w = float(values[6])
        return pose

    def _pose_to_list(self, pose: Pose) -> List[float]:
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def _start_sequence(self):
        self.sequence = self._make_sequence()
        self.current_step_index = 0
        self.running = True
        self.waiting_for_action = False
        self.pending_gripper_step = None
        self.done_reported = False
        self.get_logger().info("starting W7 pose-based pick/place sequence")
        self._publish_done(False)
        self._publish_status("RUNNING")
        self._start_current_step()

    def _start_current_step(self):
        if self.current_step_index >= len(self.sequence):
            self.running = False
            self.waiting_for_action = False
            self.pending_gripper_step = None
            self._publish_status("DONE")
            self._publish_done(True)
            return

        step = self.sequence[self.current_step_index]
        self.get_logger().info(
            f"step {self.current_step_index + 1}/{len(self.sequence)}: {step.name}"
        )
        self._publish_status(f"STEP:{step.name}")
        if isinstance(step, PoseStep):
            self._send_pose_goal(step)
        else:
            self._start_gripper_step(step)

    def _send_pose_goal(self, step: PoseStep):
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"MoveGroup action server not available: {self.move_group_action}")
            self._publish_status("FAILED:MOVE_GROUP_UNAVAILABLE")
            self.running = False
            return

        goal = MoveGroup.Goal()
        goal.request = self._make_motion_request(step.pose)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.20
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.waiting_for_action = True
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda done_future: self._on_goal_response(done_future, step))

    def _make_motion_request(self, pose: Pose):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.planning_frame
        goal_pose.pose = pose

        request = MoveGroup.Goal().request
        request.workspace_parameters.header = goal_pose.header
        request.workspace_parameters.min_corner.x = self.workspace_min[0]
        request.workspace_parameters.min_corner.y = self.workspace_min[1]
        request.workspace_parameters.min_corner.z = self.workspace_min[2]
        request.workspace_parameters.max_corner.x = self.workspace_max[0]
        request.workspace_parameters.max_corner.y = self.workspace_max[1]
        request.workspace_parameters.max_corner.z = self.workspace_max[2]
        request.start_state.is_diff = True
        request.goal_constraints = [self._pose_constraints(goal_pose)]
        request.pipeline_id = self.pipeline_id
        request.planner_id = self.planner_id
        request.group_name = self.group_name
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        return request

    def _pose_constraints(self, pose_stamped: PoseStamped) -> Constraints:
        constraints = Constraints()
        constraints.name = "ee_pose_goal"

        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        diameter = max(self.position_tolerance * 2.0, 1e-4)
        region.dimensions = [diameter, diameter, diameter]

        position = PositionConstraint()
        position.header = pose_stamped.header
        position.link_name = self.end_effector_link
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [pose_stamped.pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = pose_stamped.header
        orientation.link_name = self.end_effector_link
        orientation.orientation = pose_stamped.pose.orientation
        orientation.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation.weight = 1.0

        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def _on_goal_response(self, future, step: PoseStep):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"{step.name}: failed to send MoveGroup goal: {exc}")
            self._publish_status(f"FAILED:{step.name}:SEND_GOAL")
            self.running = False
            self.waiting_for_action = False
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"{step.name}: MoveGroup goal rejected")
            self._publish_status(f"FAILED:{step.name}:REJECTED")
            self.running = False
            self.waiting_for_action = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done_future: self._on_motion_result(done_future, step))

    def _on_motion_result(self, future, step: PoseStep):
        self.waiting_for_action = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"{step.name}: MoveGroup result failed: {exc}")
            self._publish_status(f"FAILED:{step.name}:RESULT")
            self.running = False
            return

        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{step.name}: MoveGroup failed with error_code={code}")
            self._publish_status(f"FAILED:{step.name}:ERROR_CODE_{code}")
            self.running = False
            return

        self.get_logger().info(f"{step.name}: motion executed")
        self.current_step_index += 1
        self._start_current_step()

    def _start_gripper_step(self, step: GripperStep):
        self.pending_gripper_step = step
        self.gripper_repeat_remaining = self.gripper_repeat_count
        now = self.get_clock().now()
        self.next_gripper_publish_time = now
        self.next_step_time = now + Duration(seconds=self.gripper_settle_sec)

    def _on_timer(self):
        if not self.running:
            if not self.done_reported and self.sequence:
                self.get_logger().info("W7 pick/place sequence done")
                self.done_reported = True
            return

        if self.waiting_for_action or self.pending_gripper_step is None:
            return

        now = self.get_clock().now()
        if self.gripper_repeat_remaining > 0 and now >= self.next_gripper_publish_time:
            self._publish_gripper_command(self.pending_gripper_step.positions)
            self.gripper_repeat_remaining -= 1
            self.next_gripper_publish_time = now + Duration(seconds=self.gripper_period_sec)

        if now >= self.next_step_time:
            self.current_step_index += 1
            self.pending_gripper_step = None
            self._start_current_step()

    def _publish_gripper_command(self, positions: List[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.gripper_joint_names)
        msg.position = [float(value) for value in positions]
        self.gripper_pub.publish(msg)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_done(self, done: bool):
        msg = Bool()
        msg.data = bool(done)
        self.done_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
