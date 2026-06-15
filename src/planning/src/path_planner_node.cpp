#include "planning/path_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace planning{

// ============================================================
// PathPlannerNode 생성자
//
// [역할]
//   ROS2 노드를 초기화하고, 파라미터 선언/로드, 토픽 구독/발행,
//   주기 타이머를 모두 여기서 설정함.
//
// [파라미터 설명]
//   goal_x / goal_y      : 목표 위치 (world 좌표, 미터 단위)
//                          런치 파일의 goal_x/goal_y 인자로 외부 주입 가능
//   robot_radius         : 로봇 외접원 반경 [m]
//                          inflateMap()의 팽창 반경 계산에 사용됨
//   replan_period_sec    : 주기 기반 재계획 간격 [초]
//                          이 주기마다 replanTimerCallback()이 호출됨
//   replan_obs_dist      : 동적 장애물이 경로에서 이 거리[m] 이내로 접근하면
//                          이벤트 기반 즉시 재계획 트리거
//   wp_spacing           : 재샘플링 간격 [m]
//                          경로를 이 간격으로 균일하게 나눠 waypoint 생성
//   goal_tolerance       : 목표 도달 판정 거리 [m]
//                          로봇이 목표에서 이 거리 이내면 "도달"로 간주
//
// [토픽 연결]
//   구독:
//     /map               → OccupancyGrid (slam_toolbox 발행)
//                          transient_local QoS: 나중에 구독해도 마지막 맵 수신 가능
//     /map_ekf/odom      → EKF 융합 위치 추정값 (로봇 현재 위치)
//     /goal_pose         → 외부에서 목표 지점 주입
//     /obstacles/detected→ 동적 장애물 배열 (obstacle_tracker_node 발행)
//   발행:
//     /planned_path      → A* 결과 경로 (MPC 노드가 구독)
//
// [타이머]
//   replan_timer_: replan_period_sec_ 주기로 replanTimerCallback() 호출
//   주기 재계획 외에, 이벤트(새 목표, 동적 장애물)로도 즉시 plan() 호출 가능
//   → 하이브리드 재계획 구조
// ============================================================
PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions & options)
: Node("path_planner_node", options),
  last_plan_time_(this->now()),
  last_replan_attempt_time_(this->now())
{
  // 파라미터 선언 (기본값 포함) — launch 인자로 오버라이드 가능
  this->declare_parameter("goal_x", 4.0);
  this->declare_parameter("goal_y", 0.0);
  this->declare_parameter("robot_radius", 0.45);
  this->declare_parameter("odom_topic", std::string("/map_ekf/odom"));
  this->declare_parameter("replan_period_sec", 3.0);
  this->declare_parameter("replan_obs_dist", 0.5);
  this->declare_parameter("wp_spacing", 0.10);
  this->declare_parameter("goal_tolerance", 0.20);

  // W5 안정화: A*는 정적/전역 경로만 담당하고, 동적 장애물 회피는 MPC-CBF가 담당함.
  // periodic_replan_enabled=false이면 초기 경로/목표 변경/큰 이탈 상황에서만 전역 경로를 다시 만듦.
  this->declare_parameter("periodic_replan_enabled", false);
  this->declare_parameter("off_path_replan_enabled", true);
  this->declare_parameter("off_path_replan_dist", 0.90);
  this->declare_parameter("off_path_replan_cooldown_sec", 5.00);
  this->declare_parameter("planner_replan_on_lidar_obstacles", true);

  // A*가 /obstacles/detected를 반영할지 선택함.
  // 기본값 all: RViz에서 보이는 LiDAR 장애물이 global path에도 반영되도록 함.
  // 필요 시 static/dynamic/off로 바꿔 전역 경로 안정성과 장애물 반영 강도를 조절할 수 있음.
  this->declare_parameter("planner_lidar_astar_mode", std::string("all"));
  this->declare_parameter("planner_static_speed_threshold", 0.05);
  this->declare_parameter("map_update_replan_enabled", true);
  this->declare_parameter("map_update_replan_cooldown_sec", 2.00);
  this->declare_parameter("map_update_goal_stop_radius", 1.00);

  // 발행/재계획 안정화 파라미터
  this->declare_parameter("replan_cooldown_sec", 1.20);
  this->declare_parameter("planner_publish_min_interval_sec", 1.20);
  this->declare_parameter("planner_publish_force_interval_sec", 4.00);
  this->declare_parameter("planner_path_avg_change_threshold", 0.12);
  this->declare_parameter("planner_path_max_change_threshold", 0.35);
  this->declare_parameter("planner_side_switch_lock_sec", 12.00);
  this->declare_parameter("planner_side_switch_min_score", 0.20);
  this->declare_parameter("planner_side_switch_clearance", 0.12);
  this->declare_parameter("planner_clearance_cost_radius", 0.35);
  this->declare_parameter("planner_clearance_cost_weight", 8.0);
  this->declare_parameter("smoothing_los_safety_cells", 2);
  this->declare_parameter("path_viz_republish_period_sec", 1.0);

  // Planner backend 선택.
  // astar: 기존 grid 기반 A*, rrt_star/prm: sampling-based planner.
  this->declare_parameter("planner_type", std::string("astar"));
  this->declare_parameter("rrt_max_iterations", 3000);
  this->declare_parameter("rrt_step_size", 0.25);
  this->declare_parameter("rrt_goal_sample_rate", 0.10);
  this->declare_parameter("rrt_goal_tolerance", 0.25);
  this->declare_parameter("rrt_rewire_radius", 0.60);
  this->declare_parameter("rrt_collision_check_resolution", 0.05);
  this->declare_parameter("rrt_random_seed", 7);
  this->declare_parameter("prm_num_samples", 1200);
  this->declare_parameter("prm_k_neighbors", 12);
  this->declare_parameter("prm_connection_radius", 1.00);
  this->declare_parameter("prm_collision_check_resolution", 0.05);
  this->declare_parameter("prm_random_seed", 11);

  // A*에 반영할 LiDAR 장애물 안전 처리 파라미터임.
  // 사람 다리처럼 두 개로 나뉘는 장애물이 path planner에서 빈 공간으로 해석되지 않게 하기 위함임.
  this->declare_parameter("planner_min_obstacle_radius", 0.50);
  this->declare_parameter("planner_obstacle_margin", 0.10);
  this->declare_parameter("planner_obstacle_hold_sec", 1.00);
  this->declare_parameter("planner_use_velocity_filter", false);
  this->declare_parameter("planner_merge_obstacles", true);
  this->declare_parameter("planner_obstacle_merge_distance", 0.75);
  this->declare_parameter("planner_merge_extra_radius", 0.10);
  this->declare_parameter("planner_max_merged_obstacle_radius", 1.40);

  // Fixed manipulator 피킹 스테이션 docking용 완화 모드.
  // 일반 주행 safety margin은 유지하고, goal 주변에서만 더 작은 inflation을 사용함.
  this->declare_parameter("docking_mode", false);
  this->declare_parameter("docking_relax_radius", 1.00);
  this->declare_parameter("docking_robot_radius", 0.30);
  this->declare_parameter("docking_obstacle_margin", 0.03);
  this->declare_parameter("docking_goal_search_radius", 1.20);

  // 선언된 파라미터 값을 멤버 변수에 로드
  goal_x_            = this->get_parameter("goal_x").as_double();
  goal_y_            = this->get_parameter("goal_y").as_double();
  odom_topic_        = this->get_parameter("odom_topic").as_string();
  robot_radius_      = this->get_parameter("robot_radius").as_double();
  replan_period_sec_ = this->get_parameter("replan_period_sec").as_double();
  replan_obs_dist_   = this->get_parameter("replan_obs_dist").as_double();
  wp_spacing_        = this->get_parameter("wp_spacing").as_double();
  goal_tolerance_    = this->get_parameter("goal_tolerance").as_double();
  periodic_replan_enabled_ = this->get_parameter("periodic_replan_enabled").as_bool();
  off_path_replan_enabled_ = this->get_parameter("off_path_replan_enabled").as_bool();
  off_path_replan_dist_ = this->get_parameter("off_path_replan_dist").as_double();
  off_path_replan_cooldown_sec_ = this->get_parameter("off_path_replan_cooldown_sec").as_double();
  planner_replan_on_lidar_obstacles_ = this->get_parameter("planner_replan_on_lidar_obstacles").as_bool();
  planner_lidar_astar_mode_ = this->get_parameter("planner_lidar_astar_mode").as_string();
  planner_static_speed_threshold_ = this->get_parameter("planner_static_speed_threshold").as_double();
  map_update_replan_enabled_ = this->get_parameter("map_update_replan_enabled").as_bool();
  map_update_replan_cooldown_sec_ = this->get_parameter("map_update_replan_cooldown_sec").as_double();
  map_update_goal_stop_radius_ = this->get_parameter("map_update_goal_stop_radius").as_double();
  planner_min_obstacle_radius_ = this->get_parameter("planner_min_obstacle_radius").as_double();
  planner_obstacle_margin_ = this->get_parameter("planner_obstacle_margin").as_double();
  planner_obstacle_hold_sec_ = this->get_parameter("planner_obstacle_hold_sec").as_double();
  planner_use_velocity_filter_ = this->get_parameter("planner_use_velocity_filter").as_bool();
  planner_merge_obstacles_ = this->get_parameter("planner_merge_obstacles").as_bool();
  planner_obstacle_merge_distance_ = this->get_parameter("planner_obstacle_merge_distance").as_double();
  planner_merge_extra_radius_ = this->get_parameter("planner_merge_extra_radius").as_double();
  planner_max_merged_obstacle_radius_ = this->get_parameter("planner_max_merged_obstacle_radius").as_double();
  docking_mode_ = this->get_parameter("docking_mode").as_bool();
  docking_relax_radius_ = this->get_parameter("docking_relax_radius").as_double();
  docking_robot_radius_ = this->get_parameter("docking_robot_radius").as_double();
  docking_obstacle_margin_ = this->get_parameter("docking_obstacle_margin").as_double();
  docking_goal_search_radius_ = this->get_parameter("docking_goal_search_radius").as_double();
  replan_cooldown_sec_ = this->get_parameter("replan_cooldown_sec").as_double();
  planner_publish_min_interval_sec_ = this->get_parameter("planner_publish_min_interval_sec").as_double();
  planner_publish_force_interval_sec_ = this->get_parameter("planner_publish_force_interval_sec").as_double();
  planner_path_avg_change_threshold_ = this->get_parameter("planner_path_avg_change_threshold").as_double();
  planner_path_max_change_threshold_ = this->get_parameter("planner_path_max_change_threshold").as_double();
  planner_side_switch_lock_sec_ = this->get_parameter("planner_side_switch_lock_sec").as_double();
  planner_side_switch_min_score_ = this->get_parameter("planner_side_switch_min_score").as_double();
  planner_side_switch_clearance_ = this->get_parameter("planner_side_switch_clearance").as_double();
  planner_clearance_cost_radius_ = this->get_parameter("planner_clearance_cost_radius").as_double();
  planner_clearance_cost_weight_ = this->get_parameter("planner_clearance_cost_weight").as_double();
  smoothing_los_safety_cells_ = this->get_parameter("smoothing_los_safety_cells").as_int();
  path_viz_republish_period_sec_ =
    this->get_parameter("path_viz_republish_period_sec").as_double();
  planner_type_ = this->get_parameter("planner_type").as_string();
  rrt_max_iterations_ = this->get_parameter("rrt_max_iterations").as_int();
  rrt_step_size_ = this->get_parameter("rrt_step_size").as_double();
  rrt_goal_sample_rate_ = this->get_parameter("rrt_goal_sample_rate").as_double();
  rrt_goal_tolerance_ = this->get_parameter("rrt_goal_tolerance").as_double();
  rrt_rewire_radius_ = this->get_parameter("rrt_rewire_radius").as_double();
  rrt_collision_check_resolution_ =
    this->get_parameter("rrt_collision_check_resolution").as_double();
  rrt_random_seed_ = this->get_parameter("rrt_random_seed").as_int();
  prm_num_samples_ = this->get_parameter("prm_num_samples").as_int();
  prm_k_neighbors_ = this->get_parameter("prm_k_neighbors").as_int();
  prm_connection_radius_ = this->get_parameter("prm_connection_radius").as_double();
  prm_collision_check_resolution_ =
    this->get_parameter("prm_collision_check_resolution").as_double();
  prm_random_seed_ = this->get_parameter("prm_random_seed").as_int();

  // 런치 파라미터로 목표가 이미 주입됐으므로 초기부터 has_goal_ = true
  has_goal_ = true;

  // RELIABLE QoS: 메시지 유실 없이 전달 보장 (depth=10)
  auto qos_reliable = rclcpp::QoS(10).reliable();

  // /map 구독: transient_local = 늦게 구독해도 마지막 발행 맵을 즉시 수신
  // slam_toolbox는 맵 완성 후 1회 발행하는 경우가 많아서 필수 설정
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

  // Odometry 구독: cur_x_, cur_y_ 갱신에 사용됨.
  // Isaac Sim 검증에서는 /odom을 직접 사용해서 MPC와 planner의 pose 기준을 맞춘다.
  // 기존 LiDAR fusion 기준이 필요하면 odom_topic:=/map_ekf/odom 으로 되돌릴 수 있다.
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, qos_reliable,
    std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

  // /goal_pose 구독: 외부(RViz2 NavGoal 또는 실험 스크립트)에서 목표 주입
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", qos_reliable,
    std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

  // /obstacles/detected 구독: 동적 장애물 위치/속도/반경 배열
  // obstacle_tracker_node가 LiDAR 클러스터링 결과를 발행
  // /obstacles/detected는 perception/LiDAR 계열 데이터라 QoS를 너무 강하게 묶지 않음.
  // RELIABLE/BEST_EFFORT mismatch가 나면 planner가 장애물을 못 받아 A*가 계속 직선 경로를 만들 수 있음.
  obs_sub_ = this->create_subscription<amr_msgs::msg::ObstacleArray>(
    "/obstacles/detected", rclcpp::QoS(10),
    std::bind(&PathPlannerNode::obstacleCallback, this, std::placeholders::_1));

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", qos_reliable,
    std::bind(&PathPlannerNode::locCallback, this, std::placeholders::_1));

  // /planned_path 발행: 후처리(스무딩+재샘플링)된 경로를 MPC 노드에 전달
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planned_path", rclcpp::QoS(1).reliable().transient_local());
  path_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planned_path_viz", rclcpp::QoS(1).reliable().transient_local());

  // 주기 재계획 타이머 설정 (replan_period_sec_ 초마다 실행)
  // double → milliseconds 변환 후 wall_timer 생성
  int period_ms = static_cast<int>(replan_period_sec_ * 1000.0);
  replan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PathPlannerNode::replanTimerCallback, this));

  if (path_viz_republish_period_sec_ > 0.0) {
    int viz_period_ms = static_cast<int>(path_viz_republish_period_sec_ * 1000.0);
    viz_period_ms = std::max(100, viz_period_ms);
    path_viz_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(viz_period_ms),
      std::bind(&PathPlannerNode::pathVizTimerCallback, this));
  }

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 초기화 완료 | goal=(%.2f, %.2f) | "
    "robot_r=%.2fm | periodic_replan=%d replan=%.1fs off_path=%d(%.2fm) | "
    "lidar_astar_mode=%s lidar_replan=%d map_update_replan=%d | docking=%d relax=%.2fm | wp_spacing=%.2fm | "
    "publish_min=%.1fs force=%.1fs avg_thr=%.2fm max_thr=%.2fm | clearance_cost=%.2fm/%.1f los_safety=%dcell | "
    "planner=%s rrt_iter=%d step=%.2fm rewire=%.2fm | "
    "prm_samples=%d k=%d radius=%.2fm | viz=%.1fs",
    goal_x_, goal_y_, robot_radius_,
    periodic_replan_enabled_ ? 1 : 0, replan_period_sec_,
    off_path_replan_enabled_ ? 1 : 0, off_path_replan_dist_,
    planner_lidar_astar_mode_.c_str(), planner_replan_on_lidar_obstacles_ ? 1 : 0,
    map_update_replan_enabled_ ? 1 : 0,
    docking_mode_ ? 1 : 0, docking_relax_radius_,
    wp_spacing_, planner_publish_min_interval_sec_, planner_publish_force_interval_sec_,
    planner_path_avg_change_threshold_, planner_path_max_change_threshold_,
    planner_clearance_cost_radius_, planner_clearance_cost_weight_, smoothing_los_safety_cells_,
    planner_type_.c_str(), rrt_max_iterations_, rrt_step_size_, rrt_rewire_radius_,
    prm_num_samples_, prm_k_neighbors_, prm_connection_radius_,
    path_viz_republish_period_sec_);
}

void PathPlannerNode::locCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  loc_status_ = msg->status;
}

void PathPlannerNode::pathVizTimerCallback()
{
  if (!path_viz_pub_ || current_path_.empty()) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";
  path_msg.poses = current_path_;

  for (auto & pose : path_msg.poses) {
    pose.header.stamp = path_msg.header.stamp;
    pose.header.frame_id = "map";
  }

  path_viz_pub_->publish(path_msg);
}

// ============================================================
// mapCallback() — /map 수신 콜백
//
// slam_toolbox가 완성한 OccupancyGrid 맵을 map_ 멤버에 저장.
// 맵이 처음 도착하고, odom과 goal도 준비됐다면 initial plan 실행.
// (세 조건이 모두 갖춰진 순간 딱 한 번만 initial_plan_done_ 가드로 보호)
// ============================================================
void PathPlannerNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // 맵 전체를 멤버 변수에 저장 (이후 plan()에서 참조)
  map_ = *msg;
  has_map_ = true;

  // RCLCPP_INFO_ONCE: 로그가 한 번만 출력됨 (맵 수신 확인용)
  RCLCPP_INFO_ONCE(this->get_logger(),
    "[PathPlanner] 맵 수신 완료 | %d×%d 셀, 해상도=%.3fm/cell",
    msg->info.width, msg->info.height, msg->info.resolution);

  // 맵이 마지막으로 도착한 경우를 위한 초기 계획 트리거
  // has_odom_, has_goal_이 이미 true면 즉시 초기 경로 계획 실행
  if (!initial_plan_done_ && has_odom_ && has_goal_) {
    initial_plan_done_ = true;
    plan();
    return;
  }

  // SLAM/localization 모드에서 /map이 늦게 갱신되면, 첫 계획 시점에는
  // 장애물이 아직 맵에 충분히 반영되지 않아 직선 경로가 발행될 수 있음.
  // 새 맵이 들어오면 쿨다운을 두고 재계획해서 정적 장애물 반영을 보장함.
  if (map_update_replan_enabled_ && initial_plan_done_ && has_odom_ && has_goal_) {
    const double dist_to_goal = std::hypot(goal_x_ - cur_x_, goal_y_ - cur_y_);
    if (dist_to_goal < map_update_goal_stop_radius_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[PathPlanner] 목표 근접 (%.2fm) — map update 재계획 중단, MPC/도킹 마무리 중",
        dist_to_goal);
      return;
    }

    const double elapsed = (this->now() - last_plan_time_).seconds();
    if (elapsed >= map_update_replan_cooldown_sec_) {
      RCLCPP_INFO(this->get_logger(),
        "[PathPlanner] 맵 업데이트 수신 — 정적 장애물 반영 재계획 (elapsed=%.1fs)",
        elapsed);
      plan();
    }
  }
}

// ============================================================
// odomCallback() — odometry 수신 콜백
//
// 선택된 odometry topic의 위치를 cur_x_, cur_y_에 업데이트.
// 첫 수신 시 has_odom_ 플래그를 세우고 초기 계획 조건 확인.
// ============================================================
void PathPlannerNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 로봇 현재 위치 갱신 (map frame 기준 world 좌표 [m])
  cur_x_ = msg->pose.pose.position.x;
  cur_y_ = msg->pose.pose.position.y;

  // 첫 odom 수신 처리
  if (!has_odom_) {
    has_odom_ = true;
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] odom 첫 수신 | pos=(%.2f, %.2f)", cur_x_, cur_y_);

    // odom이 마지막으로 도착한 경우를 위한 초기 계획 트리거
    if (!initial_plan_done_ && has_map_ && has_goal_) {
      initial_plan_done_ = true;
      plan();
    }
  }
}

// ============================================================
// goalCallback() — /goal_pose 수신 콜백
//
// 외부에서 새 목표가 주입되면 goal_x_, goal_y_를 갱신하고
// 즉시 plan()을 호출해 새 경로를 계산함.
// (타이머를 기다리지 않고 이벤트 기반으로 즉시 반응)
// ============================================================
void PathPlannerNode::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 새 목표 좌표를 멤버 변수에 저장
  goal_x_ = msg->pose.position.x;
  goal_y_ = msg->pose.position.y;
  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 새 목표 수신: (%.2f, %.2f) → 즉시 재계획",
    goal_x_, goal_y_);

  // 새 목표 수신 즉시 경로 재계획 (이벤트 기반 트리거)
  plan();
}

// ============================================================
// obstacleCallback() — /obstacles/detected 수신 콜백
//
// 동적 장애물 배열을 latest_obs_에 저장.
// 쿨다운(replan_cooldown_sec_) 이후, 장애물이 현재 경로에
// 너무 가까우면 즉시 재계획 트리거.
//
// [쿨다운이 필요한 이유]
//   장애물 토픽은 빠른 주기로 발행됨.
//   쿨다운 없이 isObstacleNearPath()마다 plan()을 호출하면
//   매 프레임 A*가 실행되어 CPU 부하가 폭발적으로 증가함.
//   쿨다운으로 재계획 최소 간격을 보장함.
// ============================================================
void PathPlannerNode::obstacleCallback(
  const amr_msgs::msg::ObstacleArray::SharedPtr msg)
{
  // 최신 LiDAR 장애물 정보를 저장하고 planner hold 버퍼를 갱신함.
  // planner_lidar_astar_mode가 off가 아니면 plan()에서 이 버퍼를 A* grid에 마킹함.
  latest_obs_ = *msg;
  has_obs_ = true;
  updateHeldObstacles(*msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[PathPlanner] /obstacles/detected 수신 | vector=%zu count_field=%d held=%zu mode=%s replan=%d",
    msg->x.size(), msg->count, held_obstacles_.size(),
    planner_lidar_astar_mode_.c_str(), planner_replan_on_lidar_obstacles_ ? 1 : 0);

  if (!planner_replan_on_lidar_obstacles_) {
    return;
  }

  const double dist_to_goal = std::hypot(goal_x_ - cur_x_, goal_y_ - cur_y_);
  if (dist_to_goal < 1.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 목표 근접 (%.2fm) — 장애물 트리거 전역 재계획 중단",
      dist_to_goal);
    return;
  }

  const double elapsed = (this->now() - last_replan_attempt_time_).seconds();
  if (elapsed < replan_cooldown_sec_) return;

  auto planning_obs = getMergedPlannerObstacles();
  if (!current_path_.empty() && isObstacleNearPath(planning_obs)) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] LiDAR 장애물 경로 근접 — 선택적 전역 재계획 (%.1fs 경과, obs=%zu)",
      elapsed, planning_obs.size());
    last_replan_attempt_time_ = this->now();
    plan();
  }
}

// ============================================================
// replanTimerCallback() — 주기 재계획 타이머 콜백
//
// replan_period_sec_ 주기마다 자동으로 호출됨.
// 필수 데이터(맵/odom/목표) 미준비 시 스킵.
// 목표에 충분히 가까우면 MPC가 마무리하도록 재계획 중단.
// ============================================================
void PathPlannerNode::replanTimerCallback()
{
  if (!has_map_ || !has_odom_ || !has_goal_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 대기 중 (map=%d, odom=%d, goal=%d)",
      has_map_, has_odom_, has_goal_);
    return;
  }

  const double dist_to_goal = std::hypot(goal_x_ - cur_x_, goal_y_ - cur_y_);
  if (dist_to_goal < 1.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 목표 근접 (%.2fm) — 전역 재계획 중단, MPC 마무리 중",
      dist_to_goal);
    return;
  }

  // 주기 재계획은 기본 비활성.
  // A*를 계속 현재 위치 기준으로 다시 만들면, MPC reference가 흔들려 stop-and-go가 생김.
  if (periodic_replan_enabled_) {
    plan();
    return;
  }

  // CBF 회피로 global path에서 크게 벗어난 경우에만 복귀용 전역 재계획을 허용함.
  // 일반적인 동적 장애물 회피 중에는 기존 A* 경로를 기준선으로 유지함.
  if (off_path_replan_enabled_ && !current_path_.empty()) {
    const double path_dist = distanceToCurrentPath(cur_x_, cur_y_);
    const double elapsed = (this->now() - last_plan_time_).seconds();

    if (path_dist > off_path_replan_dist_ && elapsed > off_path_replan_cooldown_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "[PathPlanner] global path 큰 이탈 감지 — 복귀용 재계획 | dist=%.2fm elapsed=%.1fs",
        path_dist, elapsed);
      plan();
      return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 기존 global path 유지 | path_dist=%.2fm periodic_replan=OFF",
      path_dist);
  }
}

// ============================================================
// plan() — 전체 경로 계획 파이프라인 (핵심 함수)
//
// [SingleThread 전제]
//   ROS2 SingleThreadedExecutor 사용 → 콜백이 동시에 실행되지 않음
//   → mutex 불필요 (map_, latest_obs_ 등 공유 상태 안전하게 접근 가능)
//
// [파이프라인 순서]
//   Step 1: 유효성 검사 (데이터 준비 여부, 목표 도달 여부)
//   Step 2: 좌표 변환 (world → grid)
//   Step 3: 맵 팽창 + 동적 장애물 마킹
//   Step 4: 시작/목표 셀 보정 (팽창 영역 내부면 근접 자유 셀로 이동)
//   Step 5: A* 탐색 → raw_path (격자 좌표 시퀀스)
//   Step 6: 스무딩 (string-pulling으로 불필요한 꺾임 제거)
//   Step 7: 재샘플링 (wp_spacing 간격으로 균일한 waypoint 생성)
//   Step 8: 시작 근처 waypoint 제거 (이미 지나친 앞점 트림)
//   Step 9: /planned_path 발행 + 상태 갱신
// ============================================================
bool PathPlannerNode::plan()
{
  // localization LOST 상태에서는 계획 스킵
  // 맵 좌표계가 깨진 상태에서 A*를 돌리면 엉뚱한 경로 생성
  if (loc_status_ == 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] localization LOST — 경로 계획 스킵");
    return false;
  }

  // Step 1: 필수 데이터 준비 여부 확인
  if (!has_map_ || !has_odom_ || !has_goal_) return false;

  // 맵 메타데이터 추출
  int W      = static_cast<int>(map_.info.width);    // 격자 가로 셀 수
  int H      = static_cast<int>(map_.info.height);   // 격자 세로 셀 수
  double res = map_.info.resolution;                 // 셀 크기 [m/cell]
  double ox  = map_.info.origin.position.x;          // 맵 원점 x [m] (world)
  double oy  = map_.info.origin.position.y;          // 맵 원점 y [m] (world)

  // Step 2: world 좌표 → grid 셀 좌표 변환 람다
  // OccupancyGrid는 원점(ox, oy)에서 시작하는 row-major 격자
  // world 좌표를 원점 기준 상대 거리로 변환 후 해상도로 나눔
  auto worldToGrid = [&](double wx, double wy) -> std::pair<int,int> {
    int gx = static_cast<int>((wx - ox) / res);
    int gy = static_cast<int>((wy - oy) / res);
    return {gx, gy};
  };
  auto gridToWorld = [&](int gx, int gy) -> std::pair<double,double> {
    double wx = gx * res + ox + 0.5 * res;
    double wy = gy * res + oy + 0.5 * res;
    return {wx, wy};
  };

  // 목표까지 직선 거리 계산 (goal_tolerance 이내면 이미 도달)
  double dist_to_goal = std::hypot(goal_x_ - cur_x_, goal_y_ - cur_y_);
  if (dist_to_goal < goal_tolerance_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 목표 도달 (dist=%.2fm) — 재계획 스킵", dist_to_goal);
    return true;
  }

  // 현재 위치와 목표를 격자 좌표로 변환
  auto [sx, sy] = worldToGrid(cur_x_, cur_y_);  // 시작 셀
  auto [ex, ey] = worldToGrid(goal_x_, goal_y_); // 목표 셀

  // 시작/목표 셀이 격자 범위를 벗어나면 계획 불가
  if (sx < 0 || sx >= W || sy < 0 || sy >= H) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 시작점이 맵 범위 밖: grid=(%d,%d)", sx, sy);
    return false;
  }
  if (ex < 0 || ex >= W || ey < 0 || ey >= H) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 목표점이 맵 범위 밖: grid=(%d,%d)", ex, ey);
    return false;
  }

  // Step 3: 맵 팽창
  // inflation_cells = ceil(로봇 반경 / 해상도)
  // 예) robot_radius=0.45m, res=0.05m → inflation_cells=9
  int inflation_cells = static_cast<int>(std::ceil(robot_radius_ / res));
  std::vector<int8_t> inflated =
    astar_.inflateMap(map_.data, W, H, inflation_cells);

  // Docking mode: fixed Franka station 근처에서는 AMR이 tray를 작업영역에 넣기 위해
  // 목표점 주변에 한정해 더 작은 robot radius로 계산한 inflation을 사용한다.
  // 원본 occupied cell은 relaxed grid에서도 유지되므로 실제 장애물을 지우지는 않는다.
  if (docking_mode_) {
    int relaxed_inflation_cells =
      static_cast<int>(std::ceil(std::max(0.0, docking_robot_radius_) / res));
    std::vector<int8_t> relaxed =
      astar_.inflateMap(map_.data, W, H, relaxed_inflation_cells);

    int relax_cells = static_cast<int>(std::ceil(std::max(0.0, docking_relax_radius_) / res));
    int copied_cells = 0;
    for (int dy = -relax_cells; dy <= relax_cells; ++dy) {
      for (int dx = -relax_cells; dx <= relax_cells; ++dx) {
        if (dx * dx + dy * dy > relax_cells * relax_cells) continue;
        int nx = ex + dx;
        int ny = ey + dy;
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
        inflated[ny * W + nx] = relaxed[ny * W + nx];
        copied_cells++;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] docking mode: goal 주변 %.2fm 완화 적용 | robot_r %.2f→%.2f | cells=%d",
      docking_relax_radius_, robot_radius_, docking_robot_radius_, copied_cells);
  }

  // Step 3-1: LiDAR 장애물의 A* 반영 여부
  // 기본값 planner_lidar_astar_mode=all.
  // RViz에서 보이는 /obstacles/detected가 global path에도 반영되도록 inflated grid에 마킹함.
  // 동적 장애물 때문에 global path가 흔들리는 실험에서는 launch/param에서 off로 낮출 수 있음.
  // 선택 모드:
  //   off     : /obstacles/detected를 A* grid에 반영하지 않음. 경로 안정성 실험용.
  //   static  : 속도가 작은 LiDAR 장애물만 A* grid에 반영함. 맵에 없는 정적 물체 검증용.
  //   dynamic : 속도가 큰 LiDAR 장애물만 A* grid에 반영함. 비권장, 실험용.
  //   all     : 모든 LiDAR 장애물을 A* grid에 반영함. 기본값.
  if (has_obs_ && planner_lidar_astar_mode_ != "off") {
    auto planning_obs = getMergedPlannerObstacles();
    int marked_count = 0;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[PathPlanner] planner obstacle buffer | mode=%s latest_vector=%zu count_field=%d held=%zu merged=%zu",
      planner_lidar_astar_mode_.c_str(), latest_obs_.x.size(), latest_obs_.count,
      held_obstacles_.size(), planning_obs.size());

    for (const auto & obs : planning_obs) {
      const double speed = std::hypot(obs.vx, obs.vy);

      if (planner_lidar_astar_mode_ == "static" && speed > planner_static_speed_threshold_) {
        continue;
      }
      if (planner_lidar_astar_mode_ == "dynamic" && speed <= planner_static_speed_threshold_) {
        continue;
      }

      const double dist_obs_to_goal = std::hypot(obs.x - goal_x_, obs.y - goal_y_);
      const bool use_docking_clearance =
        docking_mode_ && dist_obs_to_goal <= docking_relax_radius_;
      const double mark_robot_radius =
        use_docking_clearance ? docking_robot_radius_ : robot_radius_;
      const double mark_margin =
        use_docking_clearance ? docking_obstacle_margin_ : planner_obstacle_margin_;

      markPlannerObstacleOnGrid(
        inflated, W, H, ox, oy, res, obs, mark_robot_radius, mark_margin);
      marked_count++;
    }

    if (marked_count > 0) {
      RCLCPP_INFO(this->get_logger(),
        "[PathPlanner] LiDAR 장애물 %d개를 planner grid에 선택 반영 | mode=%s",
        marked_count, planner_lidar_astar_mode_.c_str());
    }
  }

  // Step 4: 시작 셀 보정
  // 로봇이 팽창 영역(벽 근처) 위에 있을 수 있음.
  // inflated[sy*W+sx] > 50 이면 팽창 영역 내부 → A*가 시작점을 막힌 곳으로 인식.
  // 해결: 시작점에서 가장 가까운 자유 셀(≤50)을 BFS 방식으로 탐색해 이동.
  // (팽창 반경을 줄이지 않는 이유: 안전 거리가 줄어들어 벽 충돌 위험)
  bool start_cell_corrected = false;
  if (inflated[sy * W + sx] > 50) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 시작점이 팽창 영역 내부 — 근접 자유 셀로 보정");

    const double goal_dx = goal_x_ - cur_x_;
    const double goal_dy = goal_y_ - cur_y_;
    const double goal_norm = std::max(std::hypot(goal_dx, goal_dy), 1e-6);
    const double ux = goal_dx / goal_norm;
    const double uy = goal_dy / goal_norm;

    bool found = false;
    int best_sx = sx;
    int best_sy = sy;
    double best_score = std::numeric_limits<double>::infinity();

    // 가까운 자유 셀 중에서도 목표 방향으로 복귀하기 쉬운 셀을 선택한다.
    for (int r = 1; r <= 20; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
          int nx = sx + dx, ny = sy + dy;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          if (inflated[ny * W + nx] <= 50) {
            auto [wx, wy] = gridToWorld(nx, ny);
            const double rx = wx - cur_x_;
            const double ry = wy - cur_y_;
            const double dist = std::hypot(rx, ry);
            const double proj = rx * ux + ry * uy;
            const double lateral = std::abs(-uy * rx + ux * ry);
            const double score =
              dist + 0.60 * lateral +
              0.80 * std::max(0.0, -proj) -
              0.20 * std::min(std::max(0.0, proj), 0.60);

            if (score < best_score) {
              best_score = score;
              best_sx = nx;
              best_sy = ny;
              found = true;
            }
          }
        }
      }
    }
    if (!found) {
      RCLCPP_WARN(this->get_logger(),
        "[PathPlanner] 시작점 보정 실패 — 이전 경로 유지");
      return false;
    }

    sx = best_sx;
    sy = best_sy;
    start_cell_corrected = true;
    auto [start_wx, start_wy] = gridToWorld(sx, sy);
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] 시작점 보정 결과 | corrected=(%.2f, %.2f) current=(%.2f, %.2f)",
      start_wx, start_wy, cur_x_, cur_y_);
  }

  // 목표 셀 보정 (동일한 방식, 탐색 반경 최대 15)
  if (inflated[ey * W + ex] > 50) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 목표점이 팽창 영역 내부 — 근접 자유 셀로 이동");
    bool found = false;
    int goal_search_cells = docking_mode_
      ? static_cast<int>(std::ceil(std::max(docking_goal_search_radius_, res) / res))
      : 15;
    for (int r = 1; r <= goal_search_cells && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          int nx = ex + dx, ny = ey + dy;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          if (inflated[ny * W + nx] <= 50) {
            ex = nx; ey = ny; found = true;
          }
        }
      }
    }
    if (!found) {
      RCLCPP_ERROR(this->get_logger(),
        "[PathPlanner] 목표 근처에 자유 셀 없음 — 계획 실패");
      return false;
    }
  }

  // Step 5: planner backend 경로 탐색
  // 팽창된 맵 위에서 (sx,sy) → (ex,ey) 경로를 찾고,
  // 이후 smoothing/resampling/publish hysteresis는 모든 planner가 공통으로 사용한다.
  const int clearance_cost_cells =
    static_cast<int>(std::ceil(std::max(0.0, planner_clearance_cost_radius_) / res));
  auto t0 = std::chrono::high_resolution_clock::now();
  std::vector<std::pair<int,int>> raw_path;
  std::string used_planner = planner_type_;

  if (planner_type_ == "rrt_star" || planner_type_ == "rrt*" || planner_type_ == "rrtstar") {
    RRTStarParams rrt_params;
    rrt_params.max_iterations = rrt_max_iterations_;
    rrt_params.step_size_cells =
      std::max(rrt_step_size_, res) / std::max(res, 1e-6);
    rrt_params.goal_sample_rate = rrt_goal_sample_rate_;
    rrt_params.goal_tolerance_cells =
      std::max(rrt_goal_tolerance_, res) / std::max(res, 1e-6);
    rrt_params.rewire_radius_cells =
      std::max(rrt_rewire_radius_, rrt_step_size_) / std::max(res, 1e-6);
    rrt_params.collision_check_step_cells =
      std::max(rrt_collision_check_resolution_, res) / std::max(res, 1e-6);
    rrt_params.random_seed = static_cast<unsigned int>(std::max(0, rrt_random_seed_));

    raw_path = rrt_star_.findPath(
      inflated, W, H, sx, sy, ex, ey, rrt_params);
    used_planner = "rrt_star";
  } else if (planner_type_ == "prm") {
    PRMParams prm_params;
    prm_params.num_samples = prm_num_samples_;
    prm_params.k_neighbors = prm_k_neighbors_;
    prm_params.connection_radius_cells =
      std::max(prm_connection_radius_, res) / std::max(res, 1e-6);
    prm_params.collision_check_step_cells =
      std::max(prm_collision_check_resolution_, res) / std::max(res, 1e-6);
    prm_params.random_seed = static_cast<unsigned int>(std::max(0, prm_random_seed_));

    raw_path = prm_.findPath(
      inflated, W, H, sx, sy, ex, ey, prm_params);
    used_planner = "prm";
  } else {
    if (planner_type_ != "astar") {
      RCLCPP_WARN(this->get_logger(),
        "[PathPlanner] 알 수 없는 planner_type='%s' — astar로 fallback",
        planner_type_.c_str());
      used_planner = "astar";
    }
    raw_path = astar_.findPath(
      inflated, W, H, sx, sy, ex, ey,
      clearance_cost_cells, planner_clearance_cost_weight_);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  double plan_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // Planner가 경로를 찾지 못한 경우 (목표 고립, 완전 막힘 등)
  if (raw_path.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] %s 경로 탐색 실패 "
      "(start=(%d,%d) goal=(%d,%d)) — 이전 경로 유지",
      used_planner.c_str(), sx, sy, ex, ey);
    return false;
  }

  if (used_planner == "rrt_star") {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] RRT* 완료: raw=%zu pts | 계획시간=%.1fms | iter=%d step=%.2fm goal_bias=%.2f rewire=%.2fm",
      raw_path.size(), plan_ms, rrt_max_iterations_, rrt_step_size_,
      rrt_goal_sample_rate_, rrt_rewire_radius_);
  } else if (used_planner == "prm") {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] PRM 완료: raw=%zu pts | 계획시간=%.1fms | samples=%d k=%d radius=%.2fm",
      raw_path.size(), plan_ms, prm_num_samples_, prm_k_neighbors_, prm_connection_radius_);
  } else {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] A* 완료: raw=%zu pts | 계획시간=%.1fms | clearance_cost=%dcells/%.1f",
      raw_path.size(), plan_ms, clearance_cost_cells, planner_clearance_cost_weight_);
  }

  // Step 6: String-Pulling 스무딩
  // A* raw_path는 격자 특성상 계단식 꺾임이 많고,
  // RRT*/PRM raw_path는 sampling 특성상 들쭉날쭉할 수 있음.
  // smoothPath()로 시야각(LOS) 기반 불필요한 중간 점을 제거해 직선화.
  // 결과: 꺾임이 줄어들고 MPC가 더 부드럽게 추종 가능
  std::vector<std::pair<int,int>> smooth =
    smoothPath(inflated, W, H, raw_path);

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 스무딩: %zu → %zu pts",
    raw_path.size(), smooth.size());

  // Step 7: 균일 간격 재샘플링
  // 스무딩 후 점 간격이 불균일함 → wp_spacing_ 간격으로 균일하게 재샘플링
  // 결과: MPC가 일정 간격의 waypoint를 참조 경로로 사용 가능
  std::vector<geometry_msgs::msg::PoseStamped> resampled =
    resamplePath(smooth, ox, oy, res, wp_spacing_);

  if (resampled.empty()) {
    RCLCPP_WARN(this->get_logger(), "[PathPlanner] 재샘플링 결과 없음");
    return false;
  }

  // Step 8: 로봇 바로 앞 waypoint 트림
  // 로봇이 이미 지나쳤거나 너무 가까운 첫 waypoint 제거
  // trim_dist=0.15m 이내의 앞 점들을 pop해서 MPC look-ahead가 적절히 동작하도록 함
  double trim_dist = 0.15;
  while (resampled.size() > 2) {
    double dx = resampled.front().pose.position.x - cur_x_;
    double dy = resampled.front().pose.position.y - cur_y_;
    if (std::hypot(dx, dy) < trim_dist) {
      resampled.erase(resampled.begin());
    } else {
      break;
    }
  }

  if (start_cell_corrected && !resampled.empty()) {
    const double dx = resampled.front().pose.position.x - cur_x_;
    const double dy = resampled.front().pose.position.y - cur_y_;
    if (std::hypot(dx, dy) > 0.05) {
      const double yaw = std::atan2(dy, dx);
      geometry_msgs::msg::PoseStamped cur_pose;
      cur_pose.header.frame_id = "map";
      cur_pose.pose.position.x = cur_x_;
      cur_pose.pose.position.y = cur_y_;
      cur_pose.pose.orientation.z = std::sin(yaw * 0.5);
      cur_pose.pose.orientation.w = std::cos(yaw * 0.5);
      resampled.insert(resampled.begin(), cur_pose);
    }
  }

  // Step 9 이전: 경로 발행 hysteresis
  // A*는 장애물 위치가 조금만 흔들려도 매번 약간 다른 경로를 만들 수 있음.
  // 이때 /planned_path를 매번 발행하면 MPC가 새 reference를 계속 받아 선속도를 0으로 낮추는 stop-and-go가 생김.
  // 따라서 기존 경로와 새 경로가 거의 같으면 계산은 성공으로 보되, 발행만 생략함.
  if (!shouldPublishPath(resampled)) {
    last_plan_time_ = this->now();  // 재계획 시도도 쿨다운에 반영해서 같은 경로를 과도하게 재계산하지 않음
    return true;
  }

  // Step 9: /planned_path 발행
  // nav_msgs/Path 메시지로 패킹 후 발행
  // MPC 노드의 /planned_path 구독 콜백이 이 경로를 waypoint로 변환해 사용
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp    = this->now();
  path_msg.header.frame_id = "map";  // map frame 기준 경로
  path_msg.poses           = resampled;
  path_pub_->publish(path_msg);

  // 발행된 경로를 멤버에 저장 (isObstacleNearPath() 비교용)
  current_path_    = resampled;
  pathVizTimerCallback();
  // 마지막 계획 시각 갱신 (쿨다운 계산에 사용)
  last_plan_time_  = this->now();

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 경로 발행 완료 | %zu waypoints | "
    "(%.2f,%.2f)→(%.2f,%.2f) | dist=%.2fm",
    resampled.size(), cur_x_, cur_y_, goal_x_, goal_y_, dist_to_goal);

  return true;
}

// ============================================================
// smoothPath() — String-Pulling 스무딩
//
// [String-Pulling이란?]
//   팽팽하게 당긴 실처럼 경로를 직선화하는 알고리즘.
//   anchor(고정점)와 test(검사점) 사이에 장애물 없이 직선 시야가 확보되면
//   중간 점들을 모두 제거하고 직선으로 연결함.
//
// [동작 순서]
//   1. anchor = 0, test = 2 로 초기화
//   2. anchor → test 사이 LOS(Line-of-Sight) 확인
//      - LOS 확보: test를 한 칸 전진 (test++)
//      - LOS 불가: test-1을 결과에 추가, anchor를 test-1로 이동
//   3. 마지막 점(goal)은 항상 결과에 추가
//
// [효과]
//   A* raw_path의 계단식 꺾임이 사라지고 부드러운 직선 구간이 생김
//   → MPC 참조 경로 품질 향상 → 추종 오차 감소
// ============================================================
// int, double 같은 작은 타입은 & 없이 그냥 값으로 받는 게 일반적이고
// std::vector처럼 크기가 큰 컨테이너는 const &로 받는 것이 C++ 관례
std::vector<std::pair<int,int>> PathPlannerNode::smoothPath(
  const std::vector<int8_t> & inflated_grid, int w, int h,
  const std::vector<std::pair<int,int>> & raw_path) const
{
  int n = static_cast<int>(raw_path.size());
  // 점이 2개 이하면 스무딩 불필요 (직선 그대로 반환)
  if (n <= 2) return raw_path;

  std::vector<std::pair<int,int>> result;
  result.push_back(raw_path[0]); // 시작점은 항상 포함

  int anchor = 0; // 현재 고정점 인덱스
  int test   = 2; // 검사 중인 점 인덱스 (anchor+2부터 시작)

  while (test < n) {
    auto [ax, ay] = raw_path[anchor];
    auto [tx, ty] = raw_path[test];

    // anchor → test 사이 직선에 장애물이 없으면 test를 더 뻗음
    if (!hasLineOfSight(inflated_grid, w, h, ax, ay, tx, ty)) {
      // LOS 확보 불가: test-1까지는 직선 가능 → test-1을 경로에 추가
      result.push_back(raw_path[test - 1]);
      anchor = test - 1;  // anchor를 test-1로 이동
      test   = anchor + 2;
    } else {
      ++test; // LOS 확보: 더 멀리 시도
    }
  }

  // 목표점은 항상 결과에 포함
  result.push_back(raw_path.back());
  return result;
}

// ============================================================
// hasLineOfSight() — Bresenham 직선 그리기 기반 LOS 검사
//
// [Bresenham 알고리즘이란?]
//   (x0,y0) → (x1,y1) 사이의 격자 직선을 부동소수점 없이
//   정수 연산만으로 그리는 알고리즘 (빠르고 정확함).
//   직선 위의 모든 격자 셀을 순서대로 방문하며 장애물 여부를 확인.
//
// [반환값]
//   true  : 두 점 사이 직선 경로에 장애물이 없음 (LOS 확보)
//   false : 도중에 occupied(>50) 또는 unknown(<0) 셀을 만남 (LOS 차단)
// ============================================================
bool PathPlannerNode::hasLineOfSight(
  const std::vector<int8_t> & grid, int w, int h,
  int x0, int y0, int x1, int y1) const
{
  // ----------------------------------------------------------------
  // Fat-LOS (두꺼운 시야각) 검사
  //
  // 기존: Bresenham 직선 위의 셀만 검사
  //   → 직선이 모서리를 아슬아슬하게 통과해도 통과 판정
  //   → 스트링풀링이 격벽 모서리 근처를 가로지르는 직선 경로 생성
  //   → 로봇이 코너를 돌 때 벽 접촉 발생
  //
  // 수정: 직선 위 각 셀에서 safety 반경 내 주변 셀까지 모두 검사
  //   → 직선이 장애물에서 safety 이상 떨어져 있어야만 통과 판정
  //   → 스트링풀링이 코너 근처 직선 생성 불가 → 기존 A* 경로 유지
  //
  // smoothing_los_safety_cells_ 만큼 추가 안전 마진을 둔다.
  // launch에서 코너 clearance를 키워 string-pulling shortcut을 보수적으로 만들 수 있다.
  // ----------------------------------------------------------------
  const int safety = std::max(0, smoothing_los_safety_cells_);

  int dx =  std::abs(x1 - x0);
  int dy =  std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0, y = y0;
  while (true) {
    // 현재 셀 중심으로 safety 반경 원형 영역 전체 검사
    for (int ky = -safety; ky <= safety; ++ky) {
      for (int kx = -safety; kx <= safety; ++kx) {
        // 원형 마스크: 사각형 아닌 원 형태로 검사
        if (kx * kx + ky * ky > safety * safety) continue;
        int nx = x + kx;
        int ny = y + ky;
        // 맵 범위 밖 = 장애물로 간주
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) return false;
        int8_t v = grid[ny * w + nx];
        // occupied(>50) 또는 unknown(<0) → LOS 차단
        if (v > 50 || v < 0) return false;
      }
    }

    if (x == x1 && y == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 <  dx) { err += dx; y += sy; }
  }

  return true;
}

// ============================================================
// resamplePath() — 고정 간격 균일 재샘플링
//
// [왜 필요한가?]
//   string-pulling 후 점 간격이 제각각임.
//   MPC는 일정 간격의 참조 waypoint가 있어야 look-ahead 거리와
//   제어 주기가 일관되게 동작함.
//   → spacing_m 간격으로 경로 위의 점을 균일하게 배치함.
//
// [구현 방식]
//   세그먼트(연속된 두 점 사이 선분)를 순회하며
//   accumulated(이전 세그먼트에서 남은 거리)와 spacing_m을 비교해
//   waypoint를 선형 보간으로 삽입함.
//
// [좌표 변환]
//   toWorld 람다: 격자 셀 (gx,gy) → world 좌표 (wx,wy)
//   셀 중심 좌표로 변환: wx = gx*res + ox + 0.5*res
//
// [orientation 계산]
//   각 waypoint의 방향(yaw)을 세그먼트 방향(atan2)으로 설정
//   quaternion으로 변환: z=sin(yaw/2), w=cos(yaw/2)
//   (x=0, y=0 — 2D 평면 회전이므로 z축 rotation만 사용)
// ============================================================
std::vector<geometry_msgs::msg::PoseStamped> PathPlannerNode::resamplePath(
  const std::vector<std::pair<int,int>> & smooth_path,
  double origin_x, double origin_y, double resolution,
  double spacing_m) const
{
  // 격자 셀 → world 좌표 변환 람다
  // 셀 중심 기준: 셀 번호 * 해상도 + 원점 + 셀 크기의 절반
  auto toWorld = [&](int gx, int gy) -> std::pair<double,double> {
    double wx = gx * resolution + origin_x + 0.5 * resolution;
    double wy = gy * resolution + origin_y + 0.5 * resolution;
    return {wx, wy};
  };

  std::vector<geometry_msgs::msg::PoseStamped> result;
  if (smooth_path.empty()) return result;

  // 이전 세그먼트에서 spacing_m까지 남은 거리 (이월 거리)
  double accumulated = 0.0;

  // 첫 번째 점을 world 좌표로 변환
  auto [wx0, wy0] = toWorld(smooth_path[0].first, smooth_path[0].second);

  // 시작점을 결과에 추가 (orientation은 다음 세그먼트 방향으로 나중에 설정 가능,
  // 여기서는 단위 쿼터니언 w=1.0 으로 초기화)
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id    = "map";
  pose.pose.position.x    = wx0;
  pose.pose.position.y    = wy0;
  pose.pose.orientation.w = 1.0;
  result.push_back(pose);

  // 세그먼트 순회 (연속된 두 스무딩 점 사이)
  for (size_t i = 1; i < smooth_path.size(); ++i) {
    // 세그먼트의 시작/끝 world 좌표
    auto [wx_prev, wy_prev] = toWorld(
      smooth_path[i - 1].first, smooth_path[i - 1].second);
    auto [wx_cur, wy_cur]   = toWorld(
      smooth_path[i].first, smooth_path[i].second);

    // 세그먼트 길이 [m]
    double seg_len = std::hypot(wx_cur - wx_prev, wy_cur - wy_prev);
    if (seg_len < 1e-6) continue; // 길이 0 세그먼트 스킵

    // 세그먼트 내에서 다음 waypoint를 삽입할 t 값 계산
    // t: 세그먼트 위의 매개변수 (0=시작, 1=끝)
    // 이전 세그먼트에서 남은 거리(accumulated)를 고려해 첫 삽입 위치 결정
    double t = (spacing_m - accumulated) / seg_len;

    // t <= 1.0 인 동안 이 세그먼트 내에 waypoint 삽입
    while (t <= 1.0 + 1e-9) {
      double tx = t > 1.0 ? 1.0 : t;  // 1.0 초과 방지 (보간 범위 클램프)
      double px = wx_prev + tx * (wx_cur - wx_prev); // 보간된 x
      double py = wy_prev + tx * (wy_cur - wy_prev); // 보간된 y

      // 세그먼트 방향을 yaw로 설정 (atan2: x방향 기준 각도)
      double yaw = std::atan2(wy_cur - wy_prev, wx_cur - wx_prev);

      // 2D 평면 회전 쿼터니언: q = (0, 0, sin(yaw/2), cos(yaw/2))
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id    = "map";
      p.pose.position.x    = px;
      p.pose.position.y    = py;
      p.pose.orientation.z = std::sin(yaw * 0.5);
      p.pose.orientation.w = std::cos(yaw * 0.5);
      result.push_back(p);

      t += spacing_m / seg_len; // 다음 waypoint 위치
    }

    // 이 세그먼트에서 마지막 waypoint 이후 남은 거리 이월
    double last_t   = t - spacing_m / seg_len;
    accumulated     = seg_len * (1.0 - last_t);
    if (accumulated >= spacing_m - 1e-6) accumulated = 0.0;
  }

  // 마지막 점(목표 셀 world 좌표) 처리
  // 마지막 waypoint와 목표 사이 거리가 spacing_m * 0.5 이상이면 별도 추가
  // (목표점이 재샘플링 그리드에서 빠지는 것을 방지)
  auto [wxL, wyL] = toWorld(
    smooth_path.back().first, smooth_path.back().second);
  if (!result.empty()) {
    double dlast = std::hypot(
      wxL - result.back().pose.position.x,
      wyL - result.back().pose.position.y);
    if (dlast > spacing_m * 0.5) {
      geometry_msgs::msg::PoseStamped last_pose;
      last_pose.header.frame_id    = "map";
      last_pose.pose.position.x    = wxL;
      last_pose.pose.position.y    = wyL;
      last_pose.pose.orientation.w = 1.0;
      result.push_back(last_pose);
    }
  }

  return result;
}


// ============================================================
// distanceToCurrentPath() — 현재 위치가 global path에서 얼마나 벗어났는지 계산
//
// [역할 분리]
//   CBF가 동적 장애물을 피하면서 일시적으로 A* path를 벗어나는 것은 정상임.
//   다만 너무 멀리 밀려나서 기존 path 복귀가 어렵다면 그때만 A*를 다시 호출함.
// ============================================================
double PathPlannerNode::distanceToCurrentPath(double x, double y) const
{
  if (current_path_.empty()) return 1e9;

  double best = 1e9;
  for (const auto & p : current_path_) {
    const double dx = p.pose.position.x - x;
    const double dy = p.pose.position.y - y;
    best = std::min(best, std::hypot(dx, dy));
  }
  return best;
}

// ============================================================
// computePathDifference() — 새 경로와 기존 경로의 기하학적 차이 계산
//
// [목적]
//   같은 복도/같은 우회 방향의 경로인데 시작점만 조금 전진한 경우에는
//   /planned_path를 다시 발행할 필요가 없음.
//   새 경로의 여러 점이 기존 경로와 얼마나 떨어져 있는지 평균/최대 거리로 판단함.
// ============================================================
double PathPlannerNode::computePathDifference(
  const std::vector<geometry_msgs::msg::PoseStamped> & new_path,
  const std::vector<geometry_msgs::msg::PoseStamped> & old_path,
  double & max_dist) const
{
  max_dist = 0.0;
  if (new_path.empty() || old_path.empty()) {
    max_dist = 1e9;
    return 1e9;
  }

  double sum = 0.0;
  int count = 0;

  // 너무 촘촘하게 비교하면 계산량만 늘고 의미가 작음.
  // 5개 간격으로 샘플링하면 wp_spacing=0.1m 기준 약 0.5m 간격 비교임.
  for (size_t i = 0; i < new_path.size(); i += 5) {
    const double nx = new_path[i].pose.position.x;
    const double ny = new_path[i].pose.position.y;

    double best = 1e9;
    for (size_t j = 0; j < old_path.size(); j += 5) {
      const double ox = old_path[j].pose.position.x;
      const double oy = old_path[j].pose.position.y;
      best = std::min(best, std::hypot(nx - ox, ny - oy));
    }

    sum += best;
    max_dist = std::max(max_dist, best);
    ++count;
  }

  return (count > 0) ? (sum / static_cast<double>(count)) : 1e9;
}

// ============================================================
// shouldPublishPath() — /planned_path 발행 여부 결정
//
// [핵심]
//   A* 계산 성공과 /planned_path 발행은 분리함.
//   - 새 경로가 기존 경로와 거의 같으면 발행 생략
//   - 일정 시간 이상 지나면 force publish 허용
//   - 경로가 크게 달라졌으면 즉시 발행 허용
//
// [효과]
//   MPC가 비슷한 path를 계속 새 path로 받아 reference를 리셋하는 현상을 줄임.
// ============================================================
bool PathPlannerNode::shouldPublishPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & new_path) const
{
  if (current_path_.empty()) return true;

  double max_diff = 0.0;
  const double avg_diff = computePathDifference(new_path, current_path_, max_diff);
  const double elapsed = (this->now() - last_plan_time_).seconds();

  auto sideScore = [&](const std::vector<geometry_msgs::msg::PoseStamped> & path) {
    const double dx_goal = goal_x_ - cur_x_;
    const double dy_goal = goal_y_ - cur_y_;
    const double norm = std::hypot(dx_goal, dy_goal);
    if (norm < 1e-6 || path.empty()) return 0.0;

    const double ux = dx_goal / norm;
    const double uy = dy_goal / norm;
    double sum = 0.0;
    int count = 0;

    for (const auto & p : path) {
      const double px = p.pose.position.x - cur_x_;
      const double py = p.pose.position.y - cur_y_;
      const double forward = px * ux + py * uy;
      if (forward < 0.15) continue;
      if (forward > 2.00) break;

      const double lateral = ux * py - uy * px;
      sum += lateral;
      ++count;
    }

    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  };

  auto currentPathMinClearance = [&]() {
    const auto planning_obs = getMergedPlannerObstacles();
    if (planning_obs.empty() || current_path_.empty()) return 1e9;

    double best_clearance = 1e9;
    for (size_t pi = 0; pi < current_path_.size(); pi += 3) {
      const double px = current_path_[pi].pose.position.x;
      const double py = current_path_[pi].pose.position.y;

      for (const auto & o : planning_obs) {
        const double dist = std::hypot(o.x - px, o.y - py);
        const double clearance =
          dist - std::max(o.radius, planner_min_obstacle_radius_) - planner_obstacle_margin_;
        best_clearance = std::min(best_clearance, clearance);
      }
    }

    return best_clearance;
  };

  const double old_side = sideScore(current_path_);
  const double new_side = sideScore(new_path);
  const bool side_switch =
    std::abs(old_side) > planner_side_switch_min_score_ &&
    std::abs(new_side) > planner_side_switch_min_score_ &&
    old_side * new_side < 0.0;

  if (side_switch && elapsed < planner_side_switch_lock_sec_) {
    const double min_clearance = currentPathMinClearance();
    if (min_clearance > planner_side_switch_clearance_) {
      RCLCPP_WARN(this->get_logger(),
        "[PathPlanner] 좌/우 우회 방향 전환 보류 | old_side=%.2f new_side=%.2f "
        "clear=%.2fm avg=%.3fm max=%.3fm elapsed=%.2fs",
        old_side, new_side, min_clearance, avg_diff, max_diff, elapsed);
      return false;
    }

    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 기존 경로 근접 장애물로 우회 방향 전환 허용 | "
      "old_side=%.2f new_side=%.2f clear=%.2fm elapsed=%.2fs",
      old_side, new_side, min_clearance, elapsed);
  }

  // 경로가 크게 달라졌으면 최소 발행 간격 안이라도 허용함.
  // 예: 장애물이 실제로 경로를 막아서 좌측 우회 → 우측 우회로 바뀌는 상황.
  const bool clearly_different =
    (avg_diff > planner_path_avg_change_threshold_ * 2.0) ||
    (max_diff > planner_path_max_change_threshold_ * 1.5);
  if (clearly_different) {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] 새 경로 차이 큼 — 발행 허용 | avg=%.3fm max=%.3fm elapsed=%.2fs",
      avg_diff, max_diff, elapsed);
    return true;
  }

  // 너무 짧은 간격으로 거의 같은 경로를 보내면 MPC가 계속 리셋됨.
  if (elapsed < planner_publish_min_interval_sec_) {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] 유사 경로 발행 생략(최소간격) | avg=%.3fm max=%.3fm elapsed=%.2fs",
      avg_diff, max_diff, elapsed);
    return false;
  }

  // 강제 발행 시간이 지나기 전까지는 거의 같은 경로를 계속 누르지 않음.
  const bool almost_same =
    (avg_diff < planner_path_avg_change_threshold_) &&
    (max_diff < planner_path_max_change_threshold_);
  if (almost_same && elapsed < planner_publish_force_interval_sec_) {
    RCLCPP_INFO(this->get_logger(),
      "[PathPlanner] 유사 경로 발행 생략 | avg=%.3fm max=%.3fm elapsed=%.2fs",
      avg_diff, max_diff, elapsed);
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 경로 발행 필요 | avg=%.3fm max=%.3fm elapsed=%.2fs",
    avg_diff, max_diff, elapsed);
  return true;
}

// ============================================================
// updateHeldObstacles() — planner 내부 장애물 hold 버퍼 갱신
//
// [목적]
//   /obstacles/detected가 한 프레임 튀거나 사라져도 A* grid에서 바로 지우지 않음.
//   사람이 다리 두 개로 보이거나 동적 cylinder가 움직일 때 경로가 갑자기 직진으로 튀는 문제를 줄임.
// ============================================================
void PathPlannerNode::updateHeldObstacles(
  const amr_msgs::msg::ObstacleArray & obs)
{
  // planner hold 버퍼의 last_seen은 메시지 header stamp가 아니라
  // planner 노드가 실제로 수신한 시각(this->now())을 사용함.
  // 이유: Isaac Sim/ROS time 설정에 따라 /obstacles/detected header.stamp와
  //       path_planner_node의 now() 기준이 어긋나면,
  //       방금 받은 장애물도 오래된 장애물로 판단되어 즉시 삭제될 수 있음.
  const rclcpp::Time stamp = this->now();

  std::vector<bool> matched(held_obstacles_.size(), false);

  // 중요: ObstacleArray의 count 필드가 0이거나 늦게 갱신되어도
  // x/y/radius vector에는 실제 장애물이 들어오는 경우가 있음.
  // 그래서 planner에서는 count 필드보다 vector 길이를 기준으로 장애물을 처리함.
  const int n = std::min({
    static_cast<int>(obs.x.size()),
    static_cast<int>(obs.y.size()),
    static_cast<int>(obs.radius.size())
  });

  for (int i = 0; i < n; ++i) {
    PlannerObstacle cur;
    cur.x = obs.x[i];
    cur.y = obs.y[i];
    cur.vx = (i < static_cast<int>(obs.vx.size())) ? obs.vx[i] : 0.0;
    cur.vy = (i < static_cast<int>(obs.vy.size())) ? obs.vy[i] : 0.0;
    cur.radius = std::max(obs.radius[i], planner_min_obstacle_radius_);
    cur.last_seen = stamp;

    int best_idx = -1;
    double best_dist = planner_obstacle_merge_distance_;
    for (size_t j = 0; j < held_obstacles_.size(); ++j) {
      if (matched[j]) continue;
      double d = std::hypot(cur.x - held_obstacles_[j].x, cur.y - held_obstacles_[j].y);
      if (d < best_dist) {
        best_dist = d;
        best_idx = static_cast<int>(j);
      }
    }

    if (best_idx >= 0) {
      auto & prev = held_obstacles_[static_cast<size_t>(best_idx)];
      matched[static_cast<size_t>(best_idx)] = true;

      // planner용 버퍼는 tracker보다 빠르게 따라가도 되므로 alpha를 크게 둠.
      constexpr double alpha = 0.45;
      prev.x = (1.0 - alpha) * prev.x + alpha * cur.x;
      prev.y = (1.0 - alpha) * prev.y + alpha * cur.y;
      prev.vx = (1.0 - alpha) * prev.vx + alpha * cur.vx;
      prev.vy = (1.0 - alpha) * prev.vy + alpha * cur.vy;
      prev.radius = std::max((1.0 - alpha) * prev.radius + alpha * cur.radius,
                             planner_min_obstacle_radius_);
      prev.last_seen = stamp;
    } else {
      held_obstacles_.push_back(cur);
      matched.push_back(true);
    }
  }

  // 일정 시간 이상 새로 보이지 않은 장애물은 제거함.
  auto now = this->now();
  held_obstacles_.erase(
    std::remove_if(
      held_obstacles_.begin(), held_obstacles_.end(),
      [&](const PlannerObstacle & o) {
        return (now - o.last_seen).seconds() > planner_obstacle_hold_sec_;
      }),
    held_obstacles_.end());
}

// ============================================================
// getMergedPlannerObstacles() — 가까운 planner 장애물 병합
//
// [목적]
//   tracker에서 병합했더라도 planner에서 한 번 더 병합함.
//   사람 다리 두 개, 움직이는 cylinder와 주변 scan 조각 등이 가까우면 하나의 큰 안전 영역으로 취급함.
// ============================================================
std::vector<PlannerObstacle> PathPlannerNode::getMergedPlannerObstacles() const
{
  if (!planner_merge_obstacles_ || held_obstacles_.empty()) {
    return held_obstacles_;
  }

  std::vector<PlannerObstacle> result;
  std::vector<bool> used(held_obstacles_.size(), false);

  for (size_t i = 0; i < held_obstacles_.size(); ++i) {
    if (used[i]) continue;

    std::vector<size_t> group;
    std::vector<size_t> queue;
    used[i] = true;
    queue.push_back(i);

    while (!queue.empty()) {
      size_t idx = queue.back();
      queue.pop_back();
      group.push_back(idx);

      for (size_t j = 0; j < held_obstacles_.size(); ++j) {
        if (used[j]) continue;
        double d = std::hypot(
          held_obstacles_[idx].x - held_obstacles_[j].x,
          held_obstacles_[idx].y - held_obstacles_[j].y);
        if (d <= planner_obstacle_merge_distance_) {
          used[j] = true;
          queue.push_back(j);
        }
      }
    }

    if (group.size() == 1) {
      PlannerObstacle single = held_obstacles_[group.front()];
      single.radius = std::max(single.radius, planner_min_obstacle_radius_);
      result.push_back(single);
      continue;
    }

    double sum_w = 0.0;
    double mx = 0.0, my = 0.0;
    double mvx = 0.0, mvy = 0.0;
    rclcpp::Time latest = held_obstacles_[group.front()].last_seen;

    for (size_t idx : group) {
      const auto & o = held_obstacles_[idx];
      double w = std::max(o.radius, 0.05);
      sum_w += w;
      mx += w * o.x;
      my += w * o.y;
      mvx += o.vx;
      mvy += o.vy;
      if (o.last_seen > latest) latest = o.last_seen;
    }

    mx /= std::max(sum_w, 1e-6);
    my /= std::max(sum_w, 1e-6);
    mvx /= static_cast<double>(group.size());
    mvy /= static_cast<double>(group.size());

    double mr = planner_min_obstacle_radius_;
    for (size_t idx : group) {
      const auto & o = held_obstacles_[idx];
      double cover_r = std::hypot(o.x - mx, o.y - my) + std::max(o.radius, planner_min_obstacle_radius_);
      mr = std::max(mr, cover_r);
    }
    mr = std::min(mr + planner_merge_extra_radius_, planner_max_merged_obstacle_radius_);

    PlannerObstacle merged;
    merged.x = mx;
    merged.y = my;
    merged.vx = mvx;
    merged.vy = mvy;
    merged.radius = mr;
    merged.last_seen = latest;
    result.push_back(merged);
  }

  return result;
}

// ============================================================
// markPlannerObstacleOnGrid() — planner 장애물을 A* grid에 원형 마킹
// ============================================================
void PathPlannerNode::markPlannerObstacleOnGrid(
  std::vector<int8_t> & grid, int w, int h,
  double origin_x, double origin_y, double resolution,
  const PlannerObstacle & obs,
  double robot_radius_for_mark,
  double obstacle_margin_for_mark) const
{
  int obs_gx = static_cast<int>((obs.x - origin_x) / resolution);
  int obs_gy = static_cast<int>((obs.y - origin_y) / resolution);

  // 로봇 외접 반경 + 장애물 반경 + planner 여유 마진을 모두 반영함.
  double effective_radius =
    std::max(obs.radius, planner_min_obstacle_radius_) +
    std::max(0.0, robot_radius_for_mark) +
    std::max(0.0, obstacle_margin_for_mark);
  int r_cells = static_cast<int>(std::ceil(effective_radius / resolution));

  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if (dx * dx + dy * dy > r_cells * r_cells) continue;
      int nx = obs_gx + dx;
      int ny = obs_gy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      grid[ny * w + nx] = 100;
    }
  }
}

// ============================================================
// isObstacleNearPath() — 장애물 경로 근접 검사
// ============================================================
bool PathPlannerNode::isObstacleNearPath(
  const std::vector<PlannerObstacle> & obs) const
{
  if (current_path_.empty() || obs.empty()) return false;

  for (size_t pi = 0; pi < current_path_.size(); pi += 5) {
    double px = current_path_[pi].pose.position.x;
    double py = current_path_[pi].pose.position.y;

    for (const auto & o : obs) {
      if (planner_use_velocity_filter_) {
        double speed = std::hypot(o.vx, o.vy);
        if (speed < 0.05) continue;
      }

      double dist = std::hypot(o.x - px, o.y - py);
      double clearance = dist - std::max(o.radius, planner_min_obstacle_radius_) - planner_obstacle_margin_;

      if (clearance < replan_obs_dist_) {
        return true;
      }
    }
  }

  return false;
}

} // namespace planning
