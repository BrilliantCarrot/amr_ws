#!/usr/bin/env python3
"""
One-shot AMR demo scenario.

Scenario:
  1. Start Gazebo AMR bringup with global planner and mock_link.
  2. Start estimation, localization, safety, obstacle tracking, and MPC nodes.
  3. Move the Monte Carlo dynamic obstacle across the planned path.
  4. Inject communication loss and localization loss once each.
  5. Wait for goal arrival, then keep the robot visible briefly for docking/stop.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from amr_msgs.msg import SafetyStatus
from std_msgs.msg import String


AMR_WS = "/home/lyj/amr_ws"


STALE_PROCESS_PATTERNS = [
    "ros2 launch scenarios bringup.launch.py",
    "ros2 launch localization slam.launch.py",
    "async_slam_toolbox_node",
    "slam_toolbox",
    "ekf_node",
    "map_ekf_node",
    "localization_monitor_node",
    "watchdog_node",
    "state_machine_node",
    "pose_rmse_node",
    "tracking_rmse_node",
    "obstacle_tracker_node",
    "obstacle_controller_node.py",
    "mpc_node",
    "path_planner_node",
    "mock_link_node",
    "parameter_bridge",
    "robot_state_publisher",
    "ign gazebo",
    "gz sim",
]


def ros_cmd(cmd, quiet=False):
    prefix = (
        "source /opt/ros/humble/setup.bash && "
        f"source {AMR_WS}/install/setup.bash && "
    )
    stdout = subprocess.DEVNULL if quiet else None
    stderr = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(
        ["/bin/bash", "-lc", prefix + " ".join(cmd)],
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )


def run_quiet(cmd, timeout=None):
    if cmd and cmd[0] == "ros2":
        prefix = (
            "source /opt/ros/humble/setup.bash && "
            f"source {AMR_WS}/install/setup.bash && "
        )
        return subprocess.run(
            ["/bin/bash", "-lc", prefix + " ".join(cmd)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    return subprocess.run(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=timeout,
    )


def set_drop_rate(rate):
    run_quiet(["ros2", "param", "set", "/mock_link_node", "drop_rate", str(rate)], timeout=5)


def reset_robot_pose():
    req = (
        'name: "amr_robot" '
        "position {x: 0.0 y: 0.0 z: 0.1} "
        "orientation {w: 1.0}"
    )
    run_quiet([
        "ign", "service",
        "-s", "/world/simple_room/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", req,
    ], timeout=5)


def set_model_pose(name, x, y, z):
    req = (
        f'name: "{name}" '
        f"position {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} "
        "orientation {w: 1.0}"
    )
    run_quiet([
        "ign", "service",
        "-s", "/world/simple_room/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "1000",
        "--req", req,
    ], timeout=2)


def set_model_velocity(name, vx, vy=0.0, wz=0.0):
    msg = (
        f"linear: {{x: {vx:.4f}, y: {vy:.4f}, z: 0.0}} "
        f"angular: {{x: 0.0, y: 0.0, z: {wz:.4f}}}"
    )
    run_quiet([
        "ign", "topic",
        "-t", f"/model/{name}/cmd_vel",
        "-m", "ignition.msgs.Twist",
        "-p", msg,
    ], timeout=1)


def stop_process(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=2)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def kill_matching_processes(patterns, sig="-TERM"):
    for pattern in patterns:
        try:
            run_quiet(["pkill", sig, "-f", pattern], timeout=1)
        except Exception:
            pass


def cleanup_existing_processes():
    kill_matching_processes(STALE_PROCESS_PATTERNS, "-TERM")
    time.sleep(1.0)
    kill_matching_processes(STALE_PROCESS_PATTERNS, "-KILL")
    time.sleep(2.0)


class DemoMonitor(Node):
    def __init__(self, goal_x, goal_y, goal_tol):
        super().__init__("demo_fault_scenario_monitor")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_tol = goal_tol
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.have_odom = False
        self.goal_reached = False
        self.docked = False
        self.mission_phase = "UNKNOWN"
        self.safety_name = "UNKNOWN"
        self.create_subscription(Odometry, "/map_ekf/odom", self._odom_cb, 10)
        self.create_subscription(SafetyStatus, "/safety/state", self._safety_cb, 10)
        mission_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/mpc/mission_phase", self._mission_phase_cb, mission_qos)

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.have_odom = True
        dist = math.hypot(self.cur_x - self.goal_x, self.cur_y - self.goal_y)
        self.goal_reached = dist <= self.goal_tol

    def _safety_cb(self, msg):
        if msg.state_name != self.safety_name:
            self.safety_name = msg.state_name
            self.get_logger().info(f"safety state -> {self.safety_name}")

    def _mission_phase_cb(self, msg):
        if msg.data != self.mission_phase:
            self.mission_phase = msg.data
            self.get_logger().info(f"mission phase -> {self.mission_phase}")
        self.docked = msg.data == "DOCKED"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=5.0)
    parser.add_argument("--goal-tol", type=float, default=0.30)
    parser.add_argument("--world", default="monte_carlo_room")
    parser.add_argument("--comm-start", type=float, default=28.0)
    parser.add_argument("--comm-duration", type=float, default=3.0)
    parser.add_argument("--loc-start", type=float, default=42.0)
    parser.add_argument("--loc-restart-delay", type=float, default=1.0)
    parser.add_argument("--obstacle-start", type=float, default=43.0)
    parser.add_argument("--obstacle-period", type=float, default=60.0)
    parser.add_argument("--obstacle-update-period", type=float, default=0.10)
    parser.add_argument("--obstacle-center-x", type=float, default=0.25)
    parser.add_argument("--obstacle-amplitude-x", type=float, default=1.75)
    parser.add_argument("--obstacle-y", type=float, default=3.5)
    parser.add_argument("--obstacle-z", type=float, default=0.4)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--post-goal-hold", type=float, default=10.0)
    parser.add_argument("--post-docked-hold", type=float, default=3.0)
    parser.add_argument("--avoid-policy", choices=["safe", "fast"], default="safe")
    parser.add_argument("--planner-type", choices=["astar", "rrt_star"], default="astar")
    parser.add_argument("--keep-running", action="store_true")
    parser.add_argument("--quiet-nodes", action="store_true")
    args = parser.parse_args()
    fast_escape_speed = 0.28

    procs = []
    slam_proc = None
    interrupted = False
    rclpy.init()
    monitor = DemoMonitor(args.goal_x, args.goal_y, args.goal_tol)

    try:
        print("[demo] cleaning up stale ROS/Gazebo processes")
        cleanup_existing_processes()

        print("[demo] starting Gazebo bringup")
        procs.append(ros_cmd([
            "ros2", "launch", "scenarios", "bringup.launch.py",
            "use_planner:=true",
            f"goal_x:={args.goal_x}",
            f"goal_y:={args.goal_y}",
            f"world:={args.world}",
            f"planner_type:={args.planner_type}",
            "mock_link:=true",
            "drop_rate:=0.0",
        ], quiet=args.quiet_nodes))

        time.sleep(10.0)
        print("[demo] starting SLAM localization")
        slam_proc = ros_cmd(["ros2", "launch", "localization", "slam.launch.py"], quiet=args.quiet_nodes)
        procs.append(slam_proc)

        time.sleep(6.0)
        print("[demo] resetting robot pose")
        reset_robot_pose()
        set_drop_rate(0.0)

        print("[demo] starting AMR nodes")
        node_cmds = [
            ["ros2", "run", "estimation", "ekf_node"],
            ["ros2", "run", "estimation", "map_ekf_node"],
            ["ros2", "run", "localization", "localization_monitor_node"],
            ["ros2", "run", "safety", "watchdog_node"],
            ["ros2", "run", "safety", "state_machine_node"],
            ["ros2", "run", "estimation", "pose_rmse_node"],
            [
                "ros2", "run", "control_mpc", "tracking_rmse_node",
                "--ros-args",
                "-p", "use_path_error:=true",
            ],
            ["ros2", "run", "control_mpc", "obstacle_tracker_node"],
            [
                "ros2", "run", "control_mpc", "mpc_node",
                "--ros-args",
                "-p", "use_global_planner:=true",
                "-p", "odom_topic:=/map_ekf/odom",
                "-p", "v_ref_:=0.1",
                "-p", "v_max:=0.18",
                "-p", "tracking_min_forward_speed:=0.0",
                "-p", "front_slow_distance:=1.30",
                "-p", "front_stop_distance:=0.45",
                "-p", "front_corridor_half_width:=0.80",
                "-p", "front_predict_time:=1.20",
                "-p", "front_avoid_turn:=0.90",
                "-p", "front_avoid_min_forward_speed:=0.12",
                "-p", f"front_avoid_policy:={args.avoid_policy}",
                "-p", "front_fast_escape_min_clearance:=0.08",
                "-p", "front_fast_escape_forward_distance:=0.45",
                "-p", "front_fast_escape_lateral_distance:=0.20",
                "-p", f"front_fast_escape_speed:={fast_escape_speed}",
                "-p", "front_fast_escape_turn:=0.18",
                "-p", "front_fast_escape_hold_sec:=1.20",
                "-p", "front_hold_release_distance:=0.85",
                "-p", "front_reverse_enter_distance:=0.32",
                "-p", "front_reverse_exit_distance:=0.55",
                "-p", "front_reverse_speed:=0.12",
                "-p", "front_reverse_duration:=1.40",
                "-p", "front_approach_rate_threshold:=0.020",
                "-p", "local_static_front_safety_enabled:=false",
                "-p", "cbf_d_safe:=0.35",
                "-p", "cbf_lookahead:=0.45",
                "-p", "cbf_react_dist:=1.50",
                "-p", "cbf_max_active_obstacles:=3",
                "-p", "enable_docking:=true",
            ],
        ]
        for cmd in node_cmds:
            procs.append(ros_cmd(cmd, quiet=args.quiet_nodes))
            time.sleep(0.4)

        t0 = time.time()
        comm_active = False
        comm_done = False
        loc_done = False
        loc_restart_done = False
        loc_restore_done = False
        obstacle_started = False
        obstacle_motion_start = None
        last_obstacle_update = 0.0
        goal_time = None
        docked_time = None

        print("[demo] mission started")
        print(f"[demo] comm fault at t={args.comm_start:.1f}s for {args.comm_duration:.1f}s")
        print(f"[demo] localization loss at t={args.loc_start:.1f}s")

        while True:
            now = time.time()
            elapsed = now - t0
            rclpy.spin_once(monitor, timeout_sec=0.05)

            if not obstacle_started and elapsed >= args.obstacle_start:
                obstacle_started = True
                obstacle_motion_start = elapsed
                left_x = args.obstacle_center_x - args.obstacle_amplitude_x
                set_model_pose("dyn_obs_1", left_x, args.obstacle_y, args.obstacle_z)
                set_model_velocity("dyn_obs_1", 0.0)
                print(f"[demo] dynamic obstacle motion started at t={elapsed:.1f}s")

            if obstacle_started and now - last_obstacle_update >= args.obstacle_update_period:
                motion_t = elapsed - obstacle_motion_start
                omega = 2.0 * math.pi / max(args.obstacle_period, 1e-6)
                # x(t)=center-amplitude*cos(omega*t). Gazebo integrates this velocity smoothly.
                vx = args.obstacle_amplitude_x * omega * math.sin(omega * motion_t)
                set_model_velocity("dyn_obs_1", vx)
                last_obstacle_update = now

            if not comm_done and elapsed >= args.comm_start:
                print(f"[demo] injecting communication loss at t={elapsed:.1f}s")
                set_drop_rate(1.0)
                comm_active = True
                comm_done = True

            if comm_active and elapsed >= args.comm_start + args.comm_duration:
                print(f"[demo] restoring communication at t={elapsed:.1f}s")
                set_drop_rate(0.0)
                comm_active = False

            if not loc_done and elapsed >= args.loc_start:
                print(f"[demo] injecting localization loss at t={elapsed:.1f}s")
                set_drop_rate(1.0)
                run_quiet(["pkill", "-f", "slam_toolbox"])
                loc_done = True

            if loc_done and not loc_restart_done and elapsed >= args.loc_start + args.loc_restart_delay:
                print(f"[demo] restarting SLAM at t={elapsed:.1f}s")
                slam_proc = ros_cmd(["ros2", "launch", "localization", "slam.launch.py"], quiet=args.quiet_nodes)
                procs.append(slam_proc)
                loc_restart_done = True

            if loc_restart_done and not loc_restore_done and elapsed >= args.loc_start + args.loc_restart_delay + 3.0:
                print(f"[demo] restoring communication after SLAM restart at t={elapsed:.1f}s")
                set_drop_rate(0.0)
                loc_restore_done = True

            if monitor.goal_reached and goal_time is None:
                goal_time = elapsed
                print(f"[demo] goal proximity reached at t={elapsed:.1f}s; waiting for DOCKED")

            if monitor.docked and docked_time is None:
                docked_time = elapsed
                print(f"[demo] docking complete at t={elapsed:.1f}s; holding for stop")

            if docked_time is not None and elapsed >= docked_time + args.post_docked_hold:
                if args.keep_running:
                    print("[demo] docked hold complete; keeping processes running. Ctrl-C to stop.")
                    while True:
                        rclpy.spin_once(monitor, timeout_sec=0.2)
                break

            if elapsed > args.timeout:
                print(f"[demo] timeout at t={elapsed:.1f}s")
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        interrupted = True
        print("\n[demo] interrupted")
    finally:
        try:
            set_drop_rate(0.0)
            set_model_velocity("dyn_obs_1", 0.0)
        except Exception:
            pass
        if interrupted or not args.keep_running:
            print("[demo] stopping processes")
            for proc in reversed(procs):
                stop_process(proc)
            kill_matching_processes(STALE_PROCESS_PATTERNS, "-TERM")
            time.sleep(1.0)
            kill_matching_processes(STALE_PROCESS_PATTERNS, "-KILL")
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
