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
    Isaac Sim W5 navigation bringup — LiDAR-only stable navigation version.

    설계 방침:
      - 주행 중에는 카메라/YOLO/RGB-D perception을 navigation loop에 넣지 않음.
      - 주행 안전은 LiDAR obstacle_tracker_node + MPC/CBF가 담당.
      - 카메라 perception은 정지 후 manipulation/pick 단계에서 별도 launch로 실행.
      - W4/W5 디버깅에서 확인한 안정 MPC 튜닝값을 launch argument 기본값으로 반영.

    전제:
      - Isaac Sim은 별도 실행 및 Play 상태.
      - Isaac Sim에서 /clock, /odom, /imu, /scan_raw, /cmd_vel bridge가 살아 있어야 함.
      - 안정 주행이 검증된 USD를 사용해야 함. 문제가 있던 4주차 USD는 drive graph를 건드리지 말 것.
    """

    # -------------------------
    # Launch arguments
    # -------------------------
    declare_scan_fix_script = DeclareLaunchArgument(
        "scan_fix_script",
        default_value="/home/lyj/isaac_amr_ws/tools/scan_fix_node.py",
        description="/scan_raw를 보정해 /scan으로 재발행하는 Python script 경로",
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

    declare_start_obstacle_tracker = DeclareLaunchArgument(
        "start_obstacle_tracker",
        default_value="true",
        description="true이면 LiDAR 기반 obstacle_tracker_node 실행",
    )

    declare_start_planner = DeclareLaunchArgument(
        "start_planner",
        default_value="true",
        description="true이면 path_planner_node 실행",
    )

    declare_start_mpc = DeclareLaunchArgument(
        "start_mpc",
        default_value="true",
        description="true이면 mpc_node 실행",
    )

    declare_start_map_ekf = DeclareLaunchArgument(
        "start_map_ekf",
        default_value="false",
        description="true이면 기존 /map_ekf/odom 생성을 위해 map_ekf_node 실행",
    )

    declare_use_global_planner = DeclareLaunchArgument(
        "use_global_planner",
        default_value="true",
        description="true이면 MPC가 /planned_path를 추종",
    )

    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="planner/mpc가 사용할 odometry topic",
    )

    declare_goal_x = DeclareLaunchArgument(
        "goal_x",
        default_value="2.62",
        description="path planner 목표 x [map frame]",
    )

    declare_goal_y = DeclareLaunchArgument(
        "goal_y",
        default_value="-3.3",
        description="path planner 목표 y [map frame]",
    )

    declare_docking_mode = DeclareLaunchArgument(
        "docking_mode",
        default_value="true",
        description="true이면 goal 주변에서만 planner inflation을 완화해 docking pose 접근을 허용",
    )

    declare_docking_relax_radius = DeclareLaunchArgument(
        "docking_relax_radius",
        default_value="1.6",
        description="docking_mode에서 완화된 inflation을 적용할 goal 주변 반경 [m]",
    )

    declare_docking_robot_radius = DeclareLaunchArgument(
        "docking_robot_radius",
        default_value="0.18",
        description="docking_mode goal 주변에서 사용할 planner robot radius [m]",
    )

    declare_docking_obstacle_margin = DeclareLaunchArgument(
        "docking_obstacle_margin",
        default_value="0.05",
        description="docking_mode goal 주변에서 사용할 obstacle margin [m]",
    )

    declare_docking_goal_search_radius = DeclareLaunchArgument(
        "docking_goal_search_radius",
        default_value="0.60",
        description="goal이 완화 후에도 blocked일 때 근접 자유 셀을 찾는 반경 [m]",
    )

    declare_trajectory_type = DeclareLaunchArgument(
        "trajectory_type",
        default_value="straight",
        description="MPC 단독 테스트용 trajectory type",
    )

    # 0.15 m/s global-planner 주행 검증 튜닝값
    declare_v_ref = DeclareLaunchArgument(
        "v_ref",
        default_value="0.15",
        description="MPC reference speed [m/s]",
    )

    declare_v_min = DeclareLaunchArgument(
        "v_min",
        default_value="0.0",
        description="MPC min linear speed [m/s]. W5 주행에서는 후진 방지를 위해 기본 0.0",
    )

    declare_v_max = DeclareLaunchArgument(
        "v_max",
        default_value="0.15",
        description="MPC max linear speed [m/s]",
    )

    declare_w_min = DeclareLaunchArgument(
        "w_min",
        default_value="-0.42",
        description="MPC min angular speed [rad/s]",
    )

    declare_w_max = DeclareLaunchArgument(
        "w_max",
        default_value="0.42",
        description="MPC max angular speed [rad/s]",
    )

    declare_dw_min = DeclareLaunchArgument(
        "dw_min",
        default_value="-0.08",
        description="MPC min angular acceleration increment",
    )

    declare_dw_max = DeclareLaunchArgument(
        "dw_max",
        default_value="0.08",
        description="MPC max angular acceleration increment",
    )

    declare_r_dw = DeclareLaunchArgument(
        "r_dw",
        default_value="95.0",
        description="MPC angular command rate penalty",
    )

    declare_q_th = DeclareLaunchArgument(
        "q_th",
        default_value="1.7",
        description="MPC heading error weight",
    )

    declare_q_y = DeclareLaunchArgument(
        "q_y",
        default_value="6.0",
        description="MPC lateral error weight",
    )

    declare_cbf_enabled = DeclareLaunchArgument(
        "cbf_enabled",
        default_value="true",
        description="MPC CBF obstacle avoidance 활성화 여부",
    )

    declare_cbf_gamma = DeclareLaunchArgument(
        "cbf_gamma",
        default_value="1.8",
        description="CBF class-K gain",
    )

    declare_cbf_d_safe = DeclareLaunchArgument(
        "cbf_d_safe",
        default_value="0.30",
        description="CBF extra safety margin [m]",
    )

    declare_cbf_lookahead = DeclareLaunchArgument(
        "cbf_lookahead",
        default_value="0.25",
        description="CBF lookahead distance [m]",
    )

    declare_cbf_react_dist = DeclareLaunchArgument(
        "cbf_react_dist",
        default_value="1.35",
        description="CBF obstacle reaction distance [m]",
    )

    declare_cbf_max_active_obstacles = DeclareLaunchArgument(
        "cbf_max_active_obstacles",
        default_value="2",
        description="CBF max active obstacles",
    )

    declare_tracking_min_forward_speed = DeclareLaunchArgument(
        "tracking_min_forward_speed",
        default_value="0.09",
        description="Global-path tracking forward speed floor [m/s]",
    )

    declare_tracking_heading_slow_angle = DeclareLaunchArgument(
        "tracking_heading_slow_angle",
        default_value="1.05",
        description="Heading error angle where tracking floor slows down [rad]",
    )

    declare_tracking_lateral_slow_error = DeclareLaunchArgument(
        "tracking_lateral_slow_error",
        default_value="0.22",
        description="Lateral error where tracking floor slows down [m]",
    )

    declare_tracking_floor_min_ratio = DeclareLaunchArgument(
        "tracking_floor_min_ratio",
        default_value="0.85",
        description="Minimum ratio for tracking forward speed floor",
    )

    # LaunchConfiguration handles
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_fix_script = LaunchConfiguration("scan_fix_script")
    lidar_z = LaunchConfiguration("lidar_z")
    start_obstacle_tracker = LaunchConfiguration("start_obstacle_tracker")
    start_planner = LaunchConfiguration("start_planner")
    start_mpc = LaunchConfiguration("start_mpc")
    start_map_ekf = LaunchConfiguration("start_map_ekf")
    use_global_planner = LaunchConfiguration("use_global_planner")
    odom_topic = LaunchConfiguration("odom_topic")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    docking_mode = LaunchConfiguration("docking_mode")
    docking_relax_radius = LaunchConfiguration("docking_relax_radius")
    docking_robot_radius = LaunchConfiguration("docking_robot_radius")
    docking_obstacle_margin = LaunchConfiguration("docking_obstacle_margin")
    docking_goal_search_radius = LaunchConfiguration("docking_goal_search_radius")
    trajectory_type = LaunchConfiguration("trajectory_type")
    v_ref = LaunchConfiguration("v_ref")
    v_min = LaunchConfiguration("v_min")
    v_max = LaunchConfiguration("v_max")
    w_min = LaunchConfiguration("w_min")
    w_max = LaunchConfiguration("w_max")
    dw_min = LaunchConfiguration("dw_min")
    dw_max = LaunchConfiguration("dw_max")
    r_dw = LaunchConfiguration("r_dw")
    q_th = LaunchConfiguration("q_th")
    q_y = LaunchConfiguration("q_y")
    cbf_enabled = LaunchConfiguration("cbf_enabled")
    cbf_gamma = LaunchConfiguration("cbf_gamma")
    cbf_d_safe = LaunchConfiguration("cbf_d_safe")
    cbf_lookahead = LaunchConfiguration("cbf_lookahead")
    cbf_react_dist = LaunchConfiguration("cbf_react_dist")
    cbf_max_active_obstacles = LaunchConfiguration("cbf_max_active_obstacles")
    tracking_min_forward_speed = LaunchConfiguration("tracking_min_forward_speed")
    tracking_heading_slow_angle = LaunchConfiguration("tracking_heading_slow_angle")
    tracking_lateral_slow_error = LaunchConfiguration("tracking_lateral_slow_error")
    tracking_floor_min_ratio = LaunchConfiguration("tracking_floor_min_ratio")

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

    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="estimation",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
                parameters=[
                    {
                        "publish_tf": False,
                    }
                ],
            )
        ],
    )

    odom_tf_broadcaster_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="estimation",
                executable="odom_tf_broadcaster_node",
                name="odom_tf_broadcaster_node",
                output="screen",
                parameters=[
                    {
                        "odom_topic": odom_topic,
                    }
                ],
            )
        ],
    )

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
                condition=IfCondition(start_map_ekf),
            )
        ],
    )

    obstacle_tracker_node = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="control_mpc",
                executable="obstacle_tracker_node",
                name="obstacle_tracker_node",
                output="screen",
                condition=IfCondition(start_obstacle_tracker),
                parameters=[
                    {
                        "max_range_factor": 0.95,
                        "cluster_tolerance": 0.35,
                        "min_cluster_points": 3,
                        "max_cluster_radius": 0.90,
                        "ema_alpha": 0.25,
                        "match_dist_thresh": 0.70,
                        "min_obstacle_radius": 0.50,
                        "persistence_timeout": 0.60,
                        "merge_close_obstacles": True,
                        "obstacle_merge_distance": 0.65,
                        "merge_extra_radius": 0.05,
                    }
                ],
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
                        "odom_topic": odom_topic,
                        "robot_radius": 0.45,
                        "replan_period_sec": 3.0,
                        "replan_obs_dist": 0.5,
                        "wp_spacing": 0.10,
                        "goal_tolerance": 0.20,
                        "periodic_replan_enabled": False,
                        "off_path_replan_enabled": True,
                        "off_path_replan_dist": 0.90,
                        "off_path_replan_cooldown_sec": 5.00,
                        "planner_lidar_astar_mode": "all",
                        "planner_replan_on_lidar_obstacles": False,
                        "map_update_replan_enabled": True,
                        "map_update_goal_stop_radius": 1.00,
                        "docking_mode": docking_mode,
                        "docking_relax_radius": docking_relax_radius,
                        "docking_robot_radius": docking_robot_radius,
                        "docking_obstacle_margin": docking_obstacle_margin,
                        "docking_goal_search_radius": docking_goal_search_radius,
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
                        "odom_topic": odom_topic,
                        "trajectory_type": trajectory_type,
                        "v_ref_": v_ref,
                        "v_min": v_min,
                        "v_max": v_max,
                        "w_min": w_min,
                        "w_max": w_max,
                        "dw_min": dw_min,
                        "dw_max": dw_max,
                        "r_dw": r_dw,
                        "q_th": q_th,
                        "q_y": q_y,
                        "docking_enter_radius": 1.10,
                        "fixed_dock_goal_enabled": True,
                        "fixed_dock_goal_x": goal_x,
                        "fixed_dock_goal_y": goal_y,
                        "cbf_enabled": cbf_enabled,
                        "cbf_fail_stop_enabled": True,
                        "cbf_gamma": cbf_gamma,
                        "cbf_d_safe": cbf_d_safe,
                        "cbf_lookahead": cbf_lookahead,
                        "cbf_react_dist": cbf_react_dist,
                        "cbf_max_active_obstacles": cbf_max_active_obstacles,
                        # 동적 장애물 proxy가 경로를 가로지를 때 늦게 반응하지 않도록
                        # 전방 corridor와 예측 horizon을 W5 동적 장애물 테스트용으로 보수화함.
                        "front_slow_distance": 1.35,
                        "front_stop_distance": 0.35,
                        "front_corridor_half_width": 0.60,
                        "front_predict_time": 0.70,
                        "local_static_front_safety_enabled": True,
                        "local_static_front_distance": 1.25,
                        "local_static_front_half_width": 0.65,
                        "tracking_min_forward_speed": tracking_min_forward_speed,
                        "tracking_heading_slow_angle": tracking_heading_slow_angle,
                        "tracking_lateral_slow_error": tracking_lateral_slow_error,
                        "tracking_floor_min_ratio": tracking_floor_min_ratio,
                    }
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_scan_fix_script,
        declare_use_sim_time,
        declare_lidar_z,
        declare_start_obstacle_tracker,
        declare_start_planner,
        declare_start_mpc,
        declare_start_map_ekf,
        declare_use_global_planner,
        declare_odom_topic,
        declare_goal_x,
        declare_goal_y,
        declare_docking_mode,
        declare_docking_relax_radius,
        declare_docking_robot_radius,
        declare_docking_obstacle_margin,
        declare_docking_goal_search_radius,
        declare_trajectory_type,
        declare_v_ref,
        declare_v_min,
        declare_v_max,
        declare_w_min,
        declare_w_max,
        declare_dw_min,
        declare_dw_max,
        declare_r_dw,
        declare_q_th,
        declare_q_y,
        declare_cbf_enabled,
        declare_cbf_gamma,
        declare_cbf_d_safe,
        declare_cbf_lookahead,
        declare_cbf_react_dist,
        declare_cbf_max_active_obstacles,
        declare_tracking_min_forward_speed,
        declare_tracking_heading_slow_angle,
        declare_tracking_lateral_slow_error,
        declare_tracking_floor_min_ratio,
        scan_fix,
        static_tf_lidar,
        static_tf_imu,
        ekf_node,
        odom_tf_broadcaster_node,
        slam_toolbox,
        map_ekf_node,
        obstacle_tracker_node,
        path_planner_node,
        mpc_node,
    ])
