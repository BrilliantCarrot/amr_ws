#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Isaac Sim 전용 navigation bringup launch v2.

    전제:
      - Isaac Sim은 별도로 실행되어 있고 Play 상태여야 함
      - Isaac Sim에서 /clock, /odom, /imu, /scan_raw, /cmd_vel bridge가 살아 있어야 함
      - Gazebo, ros_gz_bridge, robot spawn은 실행하지 않음

    v2 변경점:
      - ekf_node에는 use_sim_time을 넘기지 않음
        ekf_node 내부 IMU predict dt 계산이 this->now() 기반이라 use_sim_time=true면 dt=0 경고가 반복될 수 있음.
      - slam_toolbox에만 use_sim_time을 전달함
      - map_ekf_node / planner / mpc도 기존 수동 실행과 동일하게 use_sim_time 파라미터를 넘기지 않음
      - startup delay를 늘려 TF/map 초기화 전에 후속 노드가 먼저 뜨는 문제를 줄임
      - static_transform_publisher를 Humble 권장 new-style argument로 실행함
    """

    # -------------------------
    # Launch arguments
    # -------------------------
    declare_scan_fix_script = DeclareLaunchArgument(
        "scan_fix_script",
        default_value="/home/lyj/isaac_amr_ws/tools/scan_fix_node.py",
        description="scan_raw를 보정해 /scan으로 재발행하는 Python script 경로",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="slam_toolbox에서 Isaac Sim /clock 사용 여부",
    )

    declare_lidar_z = DeclareLaunchArgument(
        "lidar_z",
        default_value="0.25",
        description="base_link 기준 sim_lidar z 위치 [m]",
    )

    declare_start_planner = DeclareLaunchArgument(
        "start_planner",
        default_value="false",
        description="true이면 path_planner_node 실행",
    )

    declare_start_mpc = DeclareLaunchArgument(
        "start_mpc",
        default_value="false",
        description="true이면 mpc_node 실행",
    )

    declare_use_global_planner = DeclareLaunchArgument(
        "use_global_planner",
        default_value="false",
        description="true이면 MPC가 /planned_path를 추종",
    )

    declare_goal_x = DeclareLaunchArgument(
        "goal_x",
        default_value="1.0",
        description="path planner 목표 x [map frame]",
    )

    declare_goal_y = DeclareLaunchArgument(
        "goal_y",
        default_value="0.0",
        description="path planner 목표 y [map frame]",
    )

    declare_trajectory_type = DeclareLaunchArgument(
        "trajectory_type",
        default_value="straight",
        description="MPC 단독 테스트용 trajectory type",
    )

    declare_v_ref = DeclareLaunchArgument(
        "v_ref",
        default_value="0.15",
        description="MPC reference speed [m/s]",
    )

    declare_v_max = DeclareLaunchArgument(
        "v_max",
        default_value="0.15",
        description="MPC max linear speed [m/s]",
    )

    declare_w_max = DeclareLaunchArgument(
        "w_max",
        default_value="0.35",
        description="MPC max angular speed [rad/s]",
    )

    declare_cbf_enabled = DeclareLaunchArgument(
        "cbf_enabled",
        default_value="true",
        description="MPC CBF obstacle avoidance 활성화 여부",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_fix_script = LaunchConfiguration("scan_fix_script")
    lidar_z = LaunchConfiguration("lidar_z")
    start_planner = LaunchConfiguration("start_planner")
    start_mpc = LaunchConfiguration("start_mpc")
    use_global_planner = LaunchConfiguration("use_global_planner")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    trajectory_type = LaunchConfiguration("trajectory_type")
    v_ref = LaunchConfiguration("v_ref")
    v_max = LaunchConfiguration("v_max")
    w_max = LaunchConfiguration("w_max")
    cbf_enabled = LaunchConfiguration("cbf_enabled")

    pkg_localization = get_package_share_directory("localization")
    slam_launch_path = os.path.join(pkg_localization, "launch", "slam.launch.py")

    # -------------------------
    # Core nodes
    # -------------------------
    scan_fix = ExecuteProcess(
        cmd=["python3", scan_fix_script],
        output="screen",
    )

    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_sim_lidar",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", lidar_z,
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "sim_lidar",
        ],
    )

    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_imu",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
    )

    # 기존 수동 실행과 동일하게 use_sim_time을 넘기지 않음.
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="estimation",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
            )
        ],
    )

    # slam_toolbox만 /clock 기반으로 동작.
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_path),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    map_ekf_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="estimation",
                executable="map_ekf_node",
                name="map_ekf_node",
                output="screen",
            )
        ],
    )

    # -------------------------
    # Optional path planner
    # -------------------------
    path_planner_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="planning",
                executable="path_planner_node",
                name="path_planner_node",
                output="screen",
                condition=IfCondition(start_planner),
                parameters=[
                    {
                        "goal_x": goal_x,
                        "goal_y": goal_y,
                        "robot_radius": 0.30,
                        "replan_period_sec": 2.0,
                        "replan_obs_dist": 0.5,
                        "wp_spacing": 0.05,
                        "goal_tolerance": 0.20,
                        "periodic_replan_enabled": False,
                        "off_path_replan_enabled": True,
                        "off_path_replan_dist": 0.90,
                        "off_path_replan_cooldown_sec": 5.00,
                        "planner_lidar_astar_mode": "all",
                        "planner_replan_on_lidar_obstacles": False,
                        "map_update_replan_enabled": True,
                    }
                ],
            )
        ],
    )

    # -------------------------
    # Optional MPC
    # -------------------------
    mpc_node = TimerAction(
        period=17.0,
        actions=[
            Node(
                package="control_mpc",
                executable="mpc_node",
                name="mpc_node",
                output="screen",
                condition=IfCondition(start_mpc),
                parameters=[
                    {
                        "use_global_planner": use_global_planner,
                        "trajectory_type": trajectory_type,
                        "v_ref_": v_ref,
                        "v_max": v_max,
                        "v_min": 0.0,
                        "w_max": w_max,
                        "w_min": -0.35,
                        "dw_max": 0.10,
                        "dw_min": -0.10,
                        "r_dw": 70.0,
                        "q_th": 2.0,
                        "q_y": 5.0,
                        "cbf_enabled": cbf_enabled,
                        "cbf_d_safe": 0.22,
                        "cbf_gamma": 1.5,
                        "cbf_react_dist": 0.9,
                        "cbf_max_active_obstacles": 3,
                        "tracking_min_forward_speed": 0.09,
                        "tracking_heading_slow_angle": 1.05,
                        "tracking_lateral_slow_error": 0.30,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_scan_fix_script,
        declare_use_sim_time,
        declare_lidar_z,
        declare_start_planner,
        declare_start_mpc,
        declare_use_global_planner,
        declare_goal_x,
        declare_goal_y,
        declare_trajectory_type,
        declare_v_ref,
        declare_v_max,
        declare_w_max,
        declare_cbf_enabled,
        scan_fix,
        static_tf_lidar,
        static_tf_imu,
        ekf_node,
        slam_toolbox,
        map_ekf_node,
        path_planner_node,
        mpc_node,
    ])
