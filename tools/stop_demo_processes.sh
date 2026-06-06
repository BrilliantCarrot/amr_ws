#!/usr/bin/env bash
set -u

patterns=(
  "demo_fault_scenario.py"
  "ros2 launch scenarios bringup.launch.py"
  "ros2 launch localization slam.launch.py"
  "async_slam_toolbox_node"
  "slam_toolbox"
  "ekf_node"
  "map_ekf_node"
  "localization_monitor_node"
  "watchdog_node"
  "state_machine_node"
  "pose_rmse_node"
  "obstacle_tracker_node"
  "obstacle_controller_node.py"
  "mpc_node"
  "path_planner_node"
  "mock_link_node"
  "parameter_bridge"
  "robot_state_publisher"
  "ign gazebo"
  "gz sim"
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

pgrep -af "ros2|ekf_node|map_ekf_node|localization_monitor_node|watchdog_node|state_machine_node|pose_rmse_node|obstacle_tracker_node|mpc_node|path_planner_node|mock_link_node|parameter_bridge|robot_state_publisher|ign gazebo|gz sim" || true
