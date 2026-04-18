#include "planning/path_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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
  last_plan_time_(this->now())
{
  // 파라미터 선언 (기본값 포함) — launch 인자로 오버라이드 가능
  this->declare_parameter("goal_x", 2.0);
  this->declare_parameter("goal_y", 0.0);
  this->declare_parameter("robot_radius", 0.45);
  this->declare_parameter("replan_period_sec", 3.0);
  this->declare_parameter("replan_obs_dist", 0.5);
  this->declare_parameter("wp_spacing", 0.10);
  this->declare_parameter("goal_tolerance", 0.20);

  // 선언된 파라미터 값을 멤버 변수에 로드
  goal_x_            = this->get_parameter("goal_x").as_double();
  goal_y_            = this->get_parameter("goal_y").as_double();
  robot_radius_      = this->get_parameter("robot_radius").as_double();
  replan_period_sec_ = this->get_parameter("replan_period_sec").as_double();
  replan_obs_dist_   = this->get_parameter("replan_obs_dist").as_double();
  wp_spacing_        = this->get_parameter("wp_spacing").as_double();
  goal_tolerance_    = this->get_parameter("goal_tolerance").as_double();

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

  // /map_ekf/odom 구독: map_ekf_node가 발행하는 EKF 융합 위치
  // cur_x_, cur_y_ 갱신에 사용됨
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/map_ekf/odom", qos_reliable,
    std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

  // /goal_pose 구독: 외부(RViz2 NavGoal 또는 실험 스크립트)에서 목표 주입
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", qos_reliable,
    std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

  // /obstacles/detected 구독: 동적 장애물 위치/속도/반경 배열
  // obstacle_tracker_node가 LiDAR 클러스터링 결과를 발행
  obs_sub_ = this->create_subscription<amr_msgs::msg::ObstacleArray>(
    "/obstacles/detected", qos_reliable,
    std::bind(&PathPlannerNode::obstacleCallback, this, std::placeholders::_1));

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", qos_reliable,
    std::bind(&PathPlannerNode::locCallback, this, std::placeholders::_1));






  // /planned_path 발행: 후처리(스무딩+재샘플링)된 경로를 MPC 노드에 전달
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planned_path", qos_reliable);

  // 주기 재계획 타이머 설정 (replan_period_sec_ 초마다 실행)
  // double → milliseconds 변환 후 wall_timer 생성
  int period_ms = static_cast<int>(replan_period_sec_ * 1000.0);
  replan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&PathPlannerNode::replanTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] 초기화 완료 | goal=(%.2f, %.2f) | "
    "robot_r=%.2fm | replan=%.1fs | wp_spacing=%.2fm",
    goal_x_, goal_y_, robot_radius_, replan_period_sec_, wp_spacing_);
}

void PathPlannerNode::locCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  loc_status_ = msg->status;
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
  }
}

// ============================================================
// odomCallback() — /map_ekf/odom 수신 콜백
//
// EKF 융합 위치 추정값을 cur_x_, cur_y_에 업데이트.
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
  // 최신 장애물 정보를 저장 (plan()에서 동적 장애물 마킹에 사용)
  latest_obs_ = *msg;
  has_obs_ = true;

  // 마지막 계획으로부터 경과 시간 계산
  double elapsed = (this->now() - last_plan_time_).seconds();

  // 쿨다운 미충족 → 재계획 억제
  if (elapsed < replan_cooldown_sec_) return;

  // 현재 경로가 존재하고, 동적 장애물이 경로 근방에 침범했다면 재계획
  if (!current_path_.empty() && isObstacleNearPath(*msg)) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 동적 장애물 경로 근접 — 재계획 (%.1fs 경과)", elapsed);
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
  // 필수 데이터가 아직 준비 안 됐으면 대기 (3초마다 로그 출력)
  if (!has_map_ || !has_odom_ || !has_goal_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 대기 중 (map=%d, odom=%d, goal=%d)",
      has_map_, has_odom_, has_goal_);
    return;
  }

  // 목표와의 거리 계산 (유클리드 직선 거리)
  double dist_to_goal = std::hypot(goal_x_ - cur_x_, goal_y_ - cur_y_);

  // 목표 1m 이내: 전역 경로 재계획보다 MPC 로컬 제어로 마무리하는 것이 적합
  // → 재계획 중단
  if (dist_to_goal < 1.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[PathPlanner] 목표 근접 (%.2fm) — 재계획 중단, MPC 마무리 중",
      dist_to_goal);
    return;
  }

  // 조건 만족 → 주기 재계획 실행
  plan();
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

  // Step 3-1: 동적 장애물을 팽창된 맵 위에 추가 마킹
  // 정적 장애물은 slam_toolbox 맵에 이미 반영돼 있음.
  // 동적 장애물(속도 > 0.05 m/s)만 따로 마킹해서 A*가 회피하도록 함.
  if (has_obs_) {
    int dynamic_count = 0;
    for (int i = 0; i < latest_obs_.count; ++i) {
      // 속도 벡터(vx, vy)의 크기로 동적/정적 구분
      double speed = std::hypot(latest_obs_.vx[i], latest_obs_.vy[i]);
      if (speed < 0.05) continue; // 정적 장애물 스킵

      // 장애물 중심을 격자 좌표로 변환
      int obs_gx = static_cast<int>((latest_obs_.x[i] - ox) / res);
      int obs_gy = static_cast<int>((latest_obs_.y[i] - oy) / res);

      // 팽창 반경 = (장애물 반경 + 로봇 반경) / 해상도
      // 로봇이 장애물 반경 + 로봇 반경 이상 떨어져야 안전
      int r_cells = static_cast<int>(
        std::ceil((latest_obs_.radius[i] + robot_radius_) / res));

      // 원형 마킹 (inflateMap()과 동일한 방식)
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          if (dx * dx + dy * dy > r_cells * r_cells) continue;
          int nx = obs_gx + dx;
          int ny = obs_gy + dy;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          inflated[ny * W + nx] = 100;
        }
      }
      dynamic_count++;
    }

    if (dynamic_count > 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[PathPlanner] 동적 장애물 %d개 맵에 마킹", dynamic_count);
    }
  }

  // Step 4: 시작 셀 보정
  // 로봇이 팽창 영역(벽 근처) 위에 있을 수 있음.
  // inflated[sy*W+sx] > 50 이면 팽창 영역 내부 → A*가 시작점을 막힌 곳으로 인식.
  // 해결: 시작점에서 가장 가까운 자유 셀(≤50)을 BFS 방식으로 탐색해 이동.
  // (팽창 반경을 줄이지 않는 이유: 안전 거리가 줄어들어 벽 충돌 위험)
  if (inflated[sy * W + sx] > 50) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 시작점이 팽창 영역 내부 — 근접 자유 셀로 보정");
    bool found = false;
    // 반경 r=1부터 20까지 점점 확장하며 자유 셀 탐색
    for (int r = 1; r <= 20 && !found; ++r) {
      for (int dy = -r; dy <= r && !found; ++dy) {
        for (int dx = -r; dx <= r && !found; ++dx) {
          int nx = sx + dx, ny = sy + dy;
          if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
          if (inflated[ny * W + nx] <= 50) {
            sx = nx; sy = ny; found = true;
          }
        }
      }
    }
    if (!found) {
      RCLCPP_WARN(this->get_logger(),
        "[PathPlanner] 시작점 보정 실패 — 이전 경로 유지");
      return false;
    }
  }

  // 목표 셀 보정 (동일한 방식, 탐색 반경 최대 15)
  if (inflated[ey * W + ex] > 50) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] 목표점이 팽창 영역 내부 — 근접 자유 셀로 이동");
    bool found = false;
    for (int r = 1; r <= 15 && !found; ++r) {
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

  // Step 5: A* 경로 탐색
  // 팽창된 맵 위에서 (sx,sy) → (ex,ey) 최단 경로 탐색
  // raw_path: 격자 셀 좌표 시퀀스 [(x,y), ...]
  // 계획 시간도 측정해서 로그에 출력
  auto t0 = std::chrono::high_resolution_clock::now();
  std::vector<std::pair<int,int>> raw_path =
    astar_.findPath(inflated, W, H, sx, sy, ex, ey);
  auto t1 = std::chrono::high_resolution_clock::now();
  double plan_ms =
    std::chrono::duration<double, std::milli>(t1 - t0).count();

  // A*가 경로를 찾지 못한 경우 (목표 고립, 완전 막힘 등)
  if (raw_path.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "[PathPlanner] A* 경로 탐색 실패 "
      "(start=(%d,%d) goal=(%d,%d)) — 이전 경로 유지",
      sx, sy, ex, ey);
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
    "[PathPlanner] A* 완료: raw=%zu pts | 계획시간=%.1fms",
    raw_path.size(), plan_ms);

  // Step 6: String-Pulling 스무딩
  // A* raw_path는 격자 특성상 계단식 꺾임이 많음.
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
  // safety = 2 셀 = 0.10m 추가 안전 마진
  //   기존 inflation 0.45m + 0.10m = 0.55m 유효 clearance
  //   로봇 엣지 clearance: 0.55 - 0.20(로봇반경) = 0.35m (기존 0.25m → 개선)
  // ----------------------------------------------------------------
  constexpr int safety = 2;

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
// isObstacleNearPath() — 동적 장애물 경로 근접 검사
//
// [역할]
//   obstacleCallback()에서 호출되어 이벤트 기반 재계획을 결정함.
//   현재 발행된 경로(current_path_) 위의 waypoint와
//   동적 장애물 사이의 거리를 계산해 replan_obs_dist_ 이내면 true 반환.
//
// [최적화]
//   경로의 모든 waypoint를 검사하면 느림 → 5개 간격으로 샘플링 (pi += 5)
//   정확도보다 반응 속도를 우선하는 설계
//
// [clearance 계산]
//   dist = 장애물 중심 → 경로점 거리
//   clearance = dist - 장애물 반경
//   (장애물 반경을 빼야 로봇 몸체 기준 실제 여유 공간이 됨)
// ============================================================
bool PathPlannerNode::isObstacleNearPath(
  const amr_msgs::msg::ObstacleArray & obs) const
{
  // 경로 없거나 장애물 없으면 검사 불필요
  if (current_path_.empty() || obs.count == 0) return false;

  // 경로 waypoint를 5개 간격으로 샘플링 (연산량 절감)
  // current_path_에 예를 들어 100개의 waypoint가 있다면, 0, 5, 10, 15 ... 번째 waypoint만 검사한다는 뜻
  for (size_t pi = 0; pi < current_path_.size(); pi += 5) {
    double px = current_path_[pi].pose.position.x;
    double py = current_path_[pi].pose.position.y;

    // 모든 동적 장애물에 대해 검사
    for (int oi = 0; oi < obs.count; ++oi) {
      // 정적 장애물(속도 < 0.05 m/s)은 맵에 이미 반영됨 → 스킵
      double speed = std::hypot(obs.vx[oi], obs.vy[oi]);
      if (speed < 0.05) continue;

      // 장애물 중심 → 경로점 거리
      double dist = std::hypot(obs.x[oi] - px, obs.y[oi] - py);
      // 실제 여유 공간 = 거리 - 장애물 반경
      // 중심 간 거리(dist)만 쓰면 장애물이 클수록 손해라서, 반경을 빼서 실제 표면 기준 여유 공간으로 판단
      double clearance = dist - obs.radius[oi];

      // 여유 공간이 임계값 이하면 재계획 필요
      if (clearance < replan_obs_dist_) {
        return true;
      }
    }
  }

  return false; // 모든 검사 통과 → 재계획 불필요
}

} // namespace planning