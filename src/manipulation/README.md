# Manipulation Bringup

This package contains the fixed Franka station bringup used by the Isaac AMR
warehouse scenario.

The current W7 path uses MoveIt2 pose goals for the Franka arm and a direct
JointState command topic for the gripper.

## Expected Isaac Sim Setup

- AMR base is already stable with the W5 LiDAR-only navigation scene.
- A robot arm USD is mounted on the AMR.
- The arm root frame is represented as `arm_base_link` or passed via launch
  argument.
- Isaac Sim publishes `/joint_states`.
- Isaac Sim subscribes to the configured arm and gripper trajectory topics.

Default topics:

- `/arm_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`
- `/joint_states`
- `/cmd_vel`

## Build

```bash
cd ~/isaac_amr_ws
colcon build --packages-select manipulation
source install/setup.bash
```

## Run MoveIt2 With Isaac Sim

Use this as the base bringup after `/franka/joint_states` and
`/franka/joint_command` are available from Isaac Sim:

```bash
ros2 launch manipulation moveit_isaac_franka.launch.py use_sim_time:=true
```

In RViz MotionPlanning:

1. Select `panda_arm`.
2. Set start state to current.
3. Set a valid goal such as `ready` or `extended`.
4. Click Plan, then Execute.

The launch starts a `FollowJointTrajectory` action bridge that converts MoveIt2
execution commands to `/franka/joint_command`.

## Run W7 Pose-Based Pick/Place

W7 uses task-space pose targets, not manually tuned 7-DOF joint angles. The
node sends end-effector pose goals to MoveIt2 and uses `/franka/joint_command`
only for gripper open/close.

Start Isaac Sim first, press Play, and make sure these topics exist:

- `/franka/joint_states`
- `/franka/joint_command`
- `/clock`

Then run:

```bash
cd ~/isaac_amr_ws
source install/setup.bash
ros2 launch manipulation w7_pick_place.launch.py auto_start:=false
```

After RViz/MoveIt2 is up, start the sequence manually:

```bash
ros2 service call /pick_place_node/start std_srvs/srv/Trigger {}
```

For pose tuning, run one step at a time instead of the full sequence:

```bash
ros2 service call /pick_place_node/go_pre_pick std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/go_pick std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/close_gripper std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/go_lift std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/go_pre_place std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/go_place std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/open_gripper std_srvs/srv/Trigger {}
ros2 service call /pick_place_node/go_home std_srvs/srv/Trigger {}
```

Pose parameters can be updated while the launch is still running:

```bash
ros2 param set /pick_place_node pick_pose "[0.56, 0.00, 0.27, 1.0, 0.0, 0.0, 0.0]"
ros2 service call /pick_place_node/go_pick std_srvs/srv/Trigger {}
```

Runtime parameter updates are not written back to YAML automatically. After a
pose is tuned, copy the final values into `config/pick_place_sequence.yaml`.

Gripper open/close values can also be updated at runtime:

```bash
ros2 param set /pick_place_node gripper_open_positions "[0.04, 0.04]"
ros2 service call /pick_place_node/open_gripper std_srvs/srv/Trigger {}

ros2 param set /pick_place_node gripper_closed_positions "[0.015, 0.015]"
ros2 service call /pick_place_node/close_gripper std_srvs/srv/Trigger {}
```

For Isaac Sim Franka, `0.04` is fully open and values near `0.0` are closed.
When grasping a box, start with a slightly nonzero close value such as
`[0.01, 0.01]` or `[0.015, 0.015]` to avoid over-squeezing/tunneling artifacts.

Tune the task-space poses in `config/pick_place_sequence.yaml`:

- `pre_pick_pose`
- `pick_pose`
- `lift_pose`
- `pre_place_pose`
- `place_pose`
- `home_pose`

Each pose is `[x, y, z, qx, qy, qz, qw]` in the configured planning frame.
The default values are only starting templates; use RViz MotionPlanning and the
actual pick table/tray location to record reachable values.

## Configuration

Edit `config/follow_joint_trajectory_bridge.yaml` for the MoveIt2 action bridge.
Edit `config/pick_place_sequence.yaml` for W7 pose-based pick/place.
Edit `config/red_cube_detector.yaml` and `config/perception_pick_commander.yaml`
for RGB-D perception-driven pick/place.

## Run W8 Pick/Place + AMR Return Trigger

W8 adds a small mission trigger that listens for
`/perception_pick_commander_node/status == DONE` and publishes a return
`/goal_pose` for the AMR planner after a short settle delay.

Start the navigation stack separately with the planner and MPC enabled. Then
run the manipulation side:

```bash
ros2 launch manipulation w8_perception_pick_return.launch.py \
  use_sim_time:=true \
  return_x:=0.0 \
  return_y:=0.0 \
  return_yaw:=0.0
```

Start the perception pick sequence:

```bash
ros2 service call /perception_pick_commander_node/start std_srvs/srv/Trigger {}
```

When the commander reaches `DONE`, `return_to_origin_trigger_node` publishes
the return goal on `/goal_pose`. Tune the return pose in
`config/return_to_origin_trigger.yaml` or with the launch arguments above.

For generalized W8 tabletop picking, use the measured station-camera TF:

```bash
ros2 launch manipulation w7_perception_pick.launch.py \
  use_sim_time:=true \
  detector_transform_mode:=tf \
  start_camera_tf:=true
```

The default `camera_tf_*` values in the launch are calibrated from five
camera/object correspondence samples for the current station camera and Franka
pose.

To replace the red threshold detector with YOLO while keeping the same
`/perception/pick_pose` commander interface:

```bash
ros2 launch manipulation w8_yolo_perception_pick.launch.py \
  use_sim_time:=true \
  start_rviz:=true \
  target_class:=""
```

Monitor YOLO and pick pose output before starting the arm:

```bash
ros2 topic echo /detection/objects
ros2 topic echo /yolo_pick_pose_node/status
ros2 topic echo /perception/pick_pose
```

If YOLO detects multiple objects, set `target_class` to the class label you want
to pick, for example `target_class:=cup`.

## Run W7 Docking Supervisor

After the fixed-pose pick/place works, enable the supervisor that waits for the
AMR to stop at the docking pose, starts `/pick_place_node/start`, and keeps
publishing zero `/cmd_vel` while manipulation runs.

Set the actual AMR docking pose in `config/w7_docking_supervisor.yaml`:

- `docking_x`
- `docking_y`
- `docking_yaw`

Then run:

```bash
ros2 launch manipulation w7_pick_place.launch.py start_supervisor:=true auto_start:=false
```

Start the supervisor manually:

```bash
ros2 service call /w7_docking_supervisor_node/start std_srvs/srv/Trigger {}
```

Monitor:

```bash
ros2 topic echo /w7_docking_supervisor_node/status
ros2 topic echo /pick_place_node/status
```
