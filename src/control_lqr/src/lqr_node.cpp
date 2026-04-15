#include "control_lqr/lqr_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace control_lqr
{

// ============================================================
// 생성자
// ============================================================
LqrNode::LqrNode(const rclcpp::NodeOptions & options)
: Node("lqr_node", options)
{
  // --- ROS2 파라미터 선언 ---
  // Q 가중치 (MPC의 q_x/y/th/v/w와 동일 의미)
  this->declare_parameter("q_x",   10.0);
  this->declare_parameter("q_y",   10.0);
  this->declare_parameter("q_th",   5.0);
  this->declare_parameter("q_v",    1.0);
  this->declare_parameter("q_w",    1.0);

  // R 가중치 (클수록 smooth 제어)
  this->declare_parameter("r_dv",  10.0);
  this->declare_parameter("r_dw",   5.0);

  // 속도 제한 (MPC와 동일 기본값)
  this->declare_parameter("v_max",   0.5);
  this->declare_parameter("v_min",  -0.1);
  this->declare_parameter("w_max",   1.0);
  this->declare_parameter("w_min",  -1.0);

  // Riccati 반복 설정
  this->declare_parameter("riccati_iter", 500);
  this->declare_parameter("riccati_tol",  1e-2);

  // 경로/제어 파라미터
  this->declare_parameter("v_ref_",         0.1);
  this->declare_parameter("goal_tolerance",  0.20);
  this->declare_parameter("dt",             0.02);

  // 제어기 검증용 
  this->declare_parameter("trajectory_type", std::string("straight"));
  trajectory_type_ = this->get_parameter("trajectory_type").as_string();
  this->declare_parameter("use_global_planner", false);
  use_global_planner_ = this->get_parameter("use_global_planner").as_bool();

  this->declare_parameter("lap_count", 1);
  lap_count_ = this->get_parameter("lap_count").as_int();

  // 파라미터 로드 및 LQR 초기화
  lqr_params_ = loadLqrParams();
  lqr_core_.init(lqr_params_);
  v_ref_          = this->get_parameter("v_ref_").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

  RCLCPP_INFO(this->get_logger(),
    "[LQR] 초기화 완료: dt=%.3f, riccati_iter=%d, v_ref=%.2f",
    lqr_params_.dt, lqr_params_.riccati_iter, v_ref_);
  RCLCPP_INFO(this->get_logger(),
    "[LQR] Q: x=%.1f y=%.1f th=%.1f v=%.1f w=%.1f",
    lqr_params_.q_x, lqr_params_.q_y, lqr_params_.q_th,
    lqr_params_.q_v, lqr_params_.q_w);
  RCLCPP_INFO(this->get_logger(),
    "[LQR] R: dv=%.1f dw=%.1f", lqr_params_.r_dv, lqr_params_.r_dw);

  // u_prev 초기화 (정지 상태)
  u_prev_ = LqrInputVec::Zero();

  // --- QoS 설정 ---
  auto reliable_qos = rclcpp::QoS(10).reliable();
  auto sensor_qos   = rclcpp::SensorDataQoS();

  // --- 구독 ---
  // /map_ekf/odom: map frame 기준 LiDAR fusion EKF 출력
  // LQR reference waypoints도 map frame → 반드시 map frame 상태를 써야 함
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/map_ekf/odom",
    sensor_qos,
    std::bind(&LqrNode::odomCallback, this, std::placeholders::_1));

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status",
    reliable_qos,
    std::bind(&LqrNode::locStatusCallback, this, std::placeholders::_1));

  safety_sub_ = this->create_subscription<amr_msgs::msg::SafetyStatus>(
    "/safety/state",
    reliable_qos,
    std::bind(&LqrNode::safetyStateCallback, this, std::placeholders::_1));

  // /planned_path: path_planner_node(A*)가 발행하는 글로벌 경로
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planned_path",
    reliable_qos,
    std::bind(&LqrNode::pathCallback, this, std::placeholders::_1));

  // --- 발행 ---
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", reliable_qos);

  // LQR 전용 latency 토픽 (MPC의 /metrics/control_latency_ms와 별도)
  // 비교 실험 시 두 토픽을 동시에 rosbag으로 기록해 비교
  latency_pub_ = this->create_publisher<amr_msgs::msg::ControlLatency>(
    "/metrics/lqr_control_latency_ms", reliable_qos);

  // tracking_rmse_node가 구독할 LQR 목표 지점
  // tracking_rmse_node 실행 시 --ros-args -p ref_topic:=/lqr/reference_pose 로 변경 필요
  ref_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/lqr/reference_pose", reliable_qos);

  min_dist_pub_ = this->create_publisher<amr_msgs::msg::MinObstacleDistance>(
    "/metrics/lqr_min_obstacle_distance", reliable_qos);

  // --- 50Hz 제어 타이머 ---
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LqrNode::controlCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[LQR] 노드 시작. /planned_path 대기 중...");
}

// ============================================================
// odomCallback() — /map_ekf/odom 수신 (50Hz)
//
//   MpcNode::odomCallback과 동일한 continuous yaw 처리.
//   yaw가 ±π 경계에서 튀면 LQR 오차 계산 시 180° 오차 발생.
//   → atan2 정규화 금지, 이전값과의 차이로 누적
// ============================================================
void LqrNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // x, y: map frame 기준 위치
  x0_(0) = msg->pose.pose.position.x;
  x0_(1) = msg->pose.pose.position.y;

  // quaternion → yaw 변환
  // Gazebo는 항상 [-π, π] 범위로 반환 (raw yaw)
  const double qx = msg->pose.pose.orientation.x;
  const double qy = msg->pose.pose.orientation.y;
  const double qz = msg->pose.pose.orientation.z;
  const double qw = msg->pose.pose.orientation.w;
  double raw_theta = std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));

  // --------------------------------------------------------
  // continuous yaw 누적 (MpcNode와 동일 패턴)
  //
  //   raw_theta는 ±π에서 점프함.
  //   예: 179° → 181°일 때 raw는 179° → -179° (360° 점프)
  //   delta를 [-π, π]로 정규화하면 실제 변화량(2°)만 추출 가능.
  //   이를 누적하면 연속적인 yaw를 얻을 수 있음.
  // --------------------------------------------------------
  double delta = raw_theta - prev_raw_theta_;
  // delta를 [-π, π] 범위로 정규화
  delta = std::atan2(std::sin(delta), std::cos(delta));
  continuous_theta_ += delta;
  prev_raw_theta_    = raw_theta;

  x0_(2) = continuous_theta_;  // 연속 yaw 사용
  x0_(3) = msg->twist.twist.linear.x;   // v
  x0_(4) = msg->twist.twist.angular.z;  // ω

  has_odom_        = true;
  last_odom_stamp_ = this->get_clock()->now();
}

// ============================================================
// locStatusCallback()
// ============================================================
void LqrNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  loc_status_ = msg->status;

  if (loc_status_ == 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[LQR] Localization LOST — 정지 명령 발행 중");
  }
}

// ============================================================
// safetyStateCallback()
// ============================================================
void LqrNode::safetyStateCallback(
  const amr_msgs::msg::SafetyStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  safety_state_ = msg->state;
}

// 제어기 검증용 경로계획 없는 경로 설정
void LqrNode::initWaypoints()
{
  waypoints_.clear();

  if (trajectory_type_ == "circle") {
    double r     = 0.5;
    double period = 2.0 * M_PI * r / v_ref_;
    double dt_wp = lqr_params_.dt;
    int n_pts = static_cast<int>(period / dt_wp);
    double w_ref  = v_ref_ / r;

    for (int i = 0; i <= n_pts; ++i) {
      double t   = i * dt_wp;
      double ang = w_ref * t;
      LqrStateVec wp;
      wp(0) = r * std::cos(ang) - r;  // 원점에서 출발하도록 오프셋
      wp(1) = r * std::sin(ang);
      wp(2) = ang + M_PI / 2.0;       // 접선 방향
      wp(3) = v_ref_;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "[LQR] 원형 경로 초기화: %zu pts (r=%.1f)", waypoints_.size(), r);

  } else if (trajectory_type_ == "figure8") {
    double r     = 0.7;
    double dt_wp  = lqr_params_.dt;
    double w_ref  = v_ref_ / r;
    int    n_half = static_cast<int>(M_PI * r / v_ref_ / dt_wp);

    for (int i = 0; i <= n_half * 2; ++i) {
      double ang = w_ref * i * dt_wp;
      LqrStateVec wp;
      wp(0) =  r * std::cos(ang) - r;
      wp(1) =  r * std::sin(ang);
      wp(2) = 0.0;
      wp(3) = v_ref_;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    for (int i = 1; i <= n_half * 2; ++i) {
      double ang = M_PI - w_ref * i * dt_wp;
      LqrStateVec wp;
      wp(0) =  r * std::cos(ang) + r;
      wp(1) =  r * std::sin(ang);
      wp(2) = 0.0;
      wp(3) = v_ref_;
      wp(4) = -w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "[LQR] 8자 경로 초기화: %zu pts", waypoints_.size());
  }
}

// ============================================================
// pathCallback() — /planned_path 수신
//
//   MpcNode::pathCallback과 동일한 처리.
//   PoseStamped[] → LqrStateVec[] 변환 후 waypoints_ 교체.
//   θ는 generateReference()에서 실시간 계산.
// ============================================================
void LqrNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "[LQR] 빈 /planned_path 수신 — 무시");
    return;
  }

  std::vector<LqrStateVec> new_waypoints;
  new_waypoints.reserve(msg->poses.size());

  for (const auto & pose_stamped : msg->poses) {
    LqrStateVec wp;
    wp(0) = pose_stamped.pose.position.x;
    wp(1) = pose_stamped.pose.position.y;
    wp(2) = 0.0;     // θ: generateReference에서 실시간 계산
    wp(3) = v_ref_;  // v_ref
    wp(4) = 0.0;     // ω: LQR이 계산
    new_waypoints.push_back(wp);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    waypoints_      = new_waypoints;
    mission_done_   = false;
    closest_wp_idx_ = 0;   // 항상 경로 첫점부터 추종 (MpcNode와 동일)
  } 

  RCLCPP_INFO(this->get_logger(),
    "[LQR] 새 경로 수신: %zu waypoints | 첫점(%.2f, %.2f) → 끝점(%.2f, %.2f)",
    new_waypoints.size(),
    new_waypoints.front()(0), new_waypoints.front()(1),
    new_waypoints.back()(0),  new_waypoints.back()(1));
}

// ============================================================
// findClosestWaypoint() — 순서 기반 waypoint 진행
//
//   MpcNode의 방식과 동일:
//   WP_REACH_DIST 이내 도달 시 다음 waypoint로 진행.
//   nearest-neighbor 전체 탐색 금지 (우회 경로 건너뜀 방지).
// ============================================================
int LqrNode::findClosestWaypoint(const LqrStateVec & x0)
{
  int n = static_cast<int>(waypoints_.size());

  // --------------------------------------------------------
  // 경로 유형별 탐색 방식 분리
  //
  //   A* 경로(straight): 순서 기반 진행 (우회 경로 건너뜀 방지)
  //   원형/8자:          nearest-neighbor 탐색 (순환 경로이므로)
  //                      앞 50개 범위 내에서 가장 가까운 점 탐색
  // --------------------------------------------------------
  if (trajectory_type_ == "straight") {
    // 기존 순서 기반 진행 로직 유지
    double dx = x0(0) - waypoints_[closest_wp_idx_](0);
    double dy = x0(1) - waypoints_[closest_wp_idx_](1);
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < WP_REACH_DIST && closest_wp_idx_ < n - 1) {
      closest_wp_idx_++;
    }
    if (closest_wp_idx_ >= n - 2) {
      mission_done_ = true;
      RCLCPP_INFO(this->get_logger(),
        "[LQR] 미션 완료! (idx: %d / 총 %d)", closest_wp_idx_, n - 1);
    }
    return closest_wp_idx_;
  }

  // --------------------------------------------------------
  // 원형/8자: nearest-neighbor (앞 50개 탐색) + 순환
  //
  //   MPC의 findClosestWaypoint와 동일한 방식.
  //   closest_wp_idx_ 기준 앞 50개 중 가장 가까운 점을 찾음.
  //   50개 범위를 넘어가면 modulo로 순환 처리.
  // --------------------------------------------------------
  int search_range = 25;
  double min_dist = 1e9;
  int    best_idx = closest_wp_idx_;

  for (int i = 0; i < search_range; ++i) {
    // 순환 인덱스: 끝을 넘으면 처음으로
    int idx = (closest_wp_idx_ + i) % n;
    double dx   = x0(0) - waypoints_[idx](0);
    double dy   = x0(1) - waypoints_[idx](1);
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = idx;
    }
  }

  closest_wp_idx_ = best_idx;

  // 순환 경로: 끝점 도달 시 처음으로 돌아감 (미션 완료 없음)
  if (closest_wp_idx_ >= n - 2) {
    current_lap_++;
    if (current_lap_ >= lap_count_) {
        mission_done_ = true;
        RCLCPP_INFO(this->get_logger(), "[LQR] 미션 완료");
    } else {
        closest_wp_idx_ = 0;
      }
  }

  return closest_wp_idx_;
}

// ============================================================
// generateReference() — 현재 스텝 reference 상태 1개 반환
//
//   MpcNode::generateReference와 달리 N+1개가 아닌 1개만 반환.
//   LQR은 단일 스텝 참조 + Riccati로 infinite horizon을 근사.
//
//   거리 기반 lookahead:
//     step_dist = max(v_ref * dt, 0.025) 만큼 앞의 waypoint 사용
//     → 속도에 비례한 적절한 lookahead로 smooth 추종
//
//   heading 계산:
//     현재 waypoint → 다음 waypoint 방향으로 실시간 계산
//     continuous yaw와 연속성 유지
// ============================================================
LqrStateVec LqrNode::generateReference(const LqrStateVec & x0)
{
  closest_wp_idx_ = findClosestWaypoint(x0);

  int n = static_cast<int>(waypoints_.size());

  // 거리 기반 1스텝 lookahead
  double step_dist = std::max(v_ref_ * lqr_params_.dt, 0.025);

  // --------------------------------------------------------
  // 현재 waypoint에서 step_dist만큼 앞의 waypoint 탐색
  //
  //   원형/8자는 끝점을 넘어서 순환해야 하므로
  //   modulo 연산으로 인덱스를 순환 처리함.
  //   직선은 n-1에서 멈춤 (기존 방식 유지).
  // --------------------------------------------------------
  int idx = closest_wp_idx_;
  double accumulated = 0.0;
  bool is_circular = (trajectory_type_ != "straight");

  for (int step = 0; step < n; ++step) {
    int cur  = idx % n;
    int next = (idx + 1) % n;

    // 직선 경로는 끝에서 멈춤
    if (!is_circular && cur >= n - 1) break;

    double ddx = waypoints_[next](0) - waypoints_[cur](0);
    double ddy = waypoints_[next](1) - waypoints_[cur](1);
    double seg = std::sqrt(ddx * ddx + ddy * ddy);
    if (accumulated + seg >= step_dist) break;
    accumulated += seg;
    idx++;
  }

  int ref_idx      = idx % n;
  int ref_idx_next = (idx + 1) % n;

  LqrStateVec x_ref = waypoints_[ref_idx];

  // heading 계산
  double dx_wp = waypoints_[ref_idx_next](0) - waypoints_[ref_idx](0);
  double dy_wp = waypoints_[ref_idx_next](1) - waypoints_[ref_idx](1);

  if (std::abs(dx_wp) > 1e-6 || std::abs(dy_wp) > 1e-6) {
    x_ref(2) = std::atan2(dy_wp, dx_wp);
  } else {
    x_ref(2) = x0(2);
  }

  x_ref(3) = v_ref_;
  x_ref(4) = 0.0;

  return x_ref;
}

// ============================================================
// controlCallback() — 50Hz LQR 제어 루프
//
//   [처리 순서]
//   1. Data Copy-out (뮤텍스 최소 유지)
//   2. 경로/상태 확인
//   3. reference 생성 (1개)
//   4. lqr_core_.compute()
//   5. /cmd_vel 발행
//   6. latency/min_dist 발행
// ============================================================
void LqrNode::controlCallback()
{
  // --------------------------------------------------------
  // Data Copy-out: 뮤텍스 락을 최소한으로 유지
  // MpcNode와 동일한 패턴
  // --------------------------------------------------------
  LqrStateVec local_x0;
  bool        local_has_odom;
  uint8_t     local_safety_state;
  bool        local_mission_done;
  LqrInputVec local_u_prev;
  std::vector<LqrStateVec> local_waypoints;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    local_x0           = x0_;
    local_has_odom     = has_odom_;
    local_safety_state = safety_state_;
    local_mission_done = mission_done_;
    local_u_prev       = u_prev_;
    local_waypoints    = waypoints_;
  }

  if (!local_has_odom) {
    return;
  }

  // if (local_waypoints.empty()) {
  //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //     "[LQR] /planned_path 대기 중...");
  //   publishStop();
  //   return;
  // }

if (local_waypoints.empty()) {
    if (!use_global_planner_) {
        // 하드코딩 경로 모드: 최초 1회 waypoints 초기화
        initWaypoints();
        return;
    }
    // 글로벌 경로계획 모드: /planned_path 대기
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[LQR] /planned_path 대기 중...");
    publishStop();
    return;
}

  if (local_safety_state == 2) {
    publishStop();
    return;
  }

  if (local_mission_done) {
    publishStop();
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  // --------------------------------------------------------
  // reference 1개 추출 (LQR은 단일 스텝 참조)
  // --------------------------------------------------------
  LqrStateVec x_ref = generateReference(local_x0);

  // /lqr/reference_pose 발행 (tracking_rmse_node 연동)
  {
    geometry_msgs::msg::PoseStamped ref_msg;
    ref_msg.header.stamp    = this->now();
    ref_msg.header.frame_id = "map";
    ref_msg.pose.position.x = x_ref(0);
    ref_msg.pose.position.y = x_ref(1);
    ref_msg.pose.position.z = 0.0;
    double half_yaw = x_ref(2) * 0.5;
    ref_msg.pose.orientation.x = 0.0;
    ref_msg.pose.orientation.y = 0.0;
    ref_msg.pose.orientation.z = std::sin(half_yaw);
    ref_msg.pose.orientation.w = std::cos(half_yaw);
    ref_pose_pub_->publish(ref_msg);
  }

  // --------------------------------------------------------
  // TVLQR 계산
  //   lqr_core_.compute()가 내부에서:
  //   1. 선형화 → A_c, B_c
  //   2. 이산화 → A_d, B_d
  //   3. Riccati 반복 → P
  //   4. K 계산
  //   5. Δu = -K·e
  // --------------------------------------------------------
  LqrSolution sol = lqr_core_.compute(local_x0, x_ref, local_u_prev);

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!sol.success) {
    // WARN → DEBUG: tol 미달은 정상 동작, 로그 도배 방지
    RCLCPP_DEBUG(this->get_logger(),
      "[LQR] Riccati tol 미달 — warm-start P로 제어 수행");
  }

  // --------------------------------------------------------
  // 최종 속도 계산 및 클리핑
  //   sol.u0 = [Δv, Δω] (lqr_core 내부에서 이미 클리핑됨)
  //   여기서 추가로 안전 상태(DEGRADED) 속도 제한 적용
  // --------------------------------------------------------
  double v_cmd = local_u_prev(0) + sol.u0(0);
  double w_cmd = local_u_prev(1) + sol.u0(1);

  // DEGRADED 상태: 속도 50% 제한
  if (local_safety_state == 1) {
    v_cmd *= 0.5;
    w_cmd *= 0.5;
  }

  // 최종 클리핑 (안전망)
  v_cmd = std::clamp(v_cmd, lqr_params_.v_min, lqr_params_.v_max);
  w_cmd = std::clamp(w_cmd, lqr_params_.w_min, lqr_params_.w_max);

  publishCmdVel(v_cmd, w_cmd);

  // --------------------------------------------------------
  // u_prev 업데이트 (다음 스텝의 Δu 기준값)
  // --------------------------------------------------------
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    u_prev_(0) = v_cmd;
    u_prev_(1) = w_cmd;
  }

  // --------------------------------------------------------
  // e2e latency 계산
  //   odom 수신 시각 → cmd_vel 발행 시각 사이의 총 지연
  // --------------------------------------------------------
  double e2e_ms = 0.0, avg_e2e = 0.0, p99_e2e = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto now_stamp = this->get_clock()->now();
    e2e_ms = (now_stamp - last_odom_stamp_).seconds() * 1000.0;

    e2e_history_.push_back(e2e_ms);
    if (static_cast<int>(e2e_history_.size()) > 500) {
      e2e_history_.erase(e2e_history_.begin());
    }

    for (double val : e2e_history_) avg_e2e += val;
    avg_e2e /= e2e_history_.size();

    if (!e2e_history_.empty()) {
      std::vector<double> sorted = e2e_history_;
      std::sort(sorted.begin(), sorted.end());
      int idx = static_cast<int>(sorted.size() * 0.99);
      p99_e2e = sorted[std::min(idx, static_cast<int>(sorted.size()) - 1)];
    }
  }

  // --------------------------------------------------------
  // latency 통계 발행 (/metrics/lqr_control_latency_ms)
  // --------------------------------------------------------
  latency_history_.push_back(elapsed_ms);
  if (static_cast<int>(latency_history_.size()) > 500) {
    latency_history_.erase(latency_history_.begin());
  }

  double avg_ms = 0.0;
  for (double val : latency_history_) avg_ms += val;
  avg_ms /= latency_history_.size();

  double max_ms = *std::max_element(latency_history_.begin(), latency_history_.end());

  amr_msgs::msg::ControlLatency lat_msg;
  lat_msg.header.stamp   = this->now();
  lat_msg.latency_ms     = elapsed_ms;
  lat_msg.avg_latency_ms = avg_ms;
  lat_msg.max_latency_ms = max_ms;
  lat_msg.e2e_latency_ms = e2e_ms;
  lat_msg.avg_e2e_ms     = avg_e2e;
  lat_msg.p99_e2e_ms     = p99_e2e;
  latency_pub_->publish(lat_msg);

  // --------------------------------------------------------
  // 장애물 최소 거리 발행 (/metrics/lqr_min_obstacle_distance)
  //   LQR은 use_global_planner=true 전용이므로
  //   장애물 처리는 path_planner_node가 담당.
  //   여기서는 거리 측정만 하여 KPI 기록용으로 발행.
  //   obstacles_는 별도로 구독하지 않으므로
  //   로봇 자체 크기(0.2m)를 기준으로 safe zone 경고 처리.
  //   (정밀 장애물 거리 측정이 필요하면 /obstacles/detected 구독 추가 가능)
  // --------------------------------------------------------
  {
    amr_msgs::msg::MinObstacleDistance dist_msg;
    dist_msg.header.stamp = this->now();
    // 글로벌 경로계획이 장애물을 회피하므로 로봇-장애물 거리 측정 생략
    // 비교 실험에서는 /obstacles/detected를 같은 방식으로 구독하여 계산 가능
    dist_msg.min_distance_m = 999.0;  // 미측정 표시
    dist_msg.is_critical    = false;
    min_dist_pub_->publish(dist_msg);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    "[LQR] solve=%.3fms avg=%.3fms | e2e=%.2fms p99=%.2fms | v=%.3f w=%.3f | wp=%d",
    elapsed_ms, avg_ms, e2e_ms, p99_e2e, v_cmd, w_cmd, closest_wp_idx_);
}

// ============================================================
// publishCmdVel()
// ============================================================
void LqrNode::publishCmdVel(double v, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = v;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

// ============================================================
// publishStop()
// ============================================================
void LqrNode::publishStop()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);
}

// ============================================================
// loadLqrParams() — ROS2 파라미터 → LqrParams 변환
// ============================================================
LqrParams LqrNode::loadLqrParams()
{
  LqrParams p;
  p.dt           = this->get_parameter("dt").as_double();
  p.q_x          = this->get_parameter("q_x").as_double();
  p.q_y          = this->get_parameter("q_y").as_double();
  p.q_th         = this->get_parameter("q_th").as_double();
  p.q_v          = this->get_parameter("q_v").as_double();
  p.q_w          = this->get_parameter("q_w").as_double();
  p.r_dv         = this->get_parameter("r_dv").as_double();
  p.r_dw         = this->get_parameter("r_dw").as_double();
  p.v_max        = this->get_parameter("v_max").as_double();
  p.v_min        = this->get_parameter("v_min").as_double();
  p.w_max        = this->get_parameter("w_max").as_double();
  p.w_min        = this->get_parameter("w_min").as_double();
  p.riccati_iter = this->get_parameter("riccati_iter").as_int();
  p.riccati_tol  = this->get_parameter("riccati_tol").as_double();
  return p;
}

}  // namespace control_lqr

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_lqr::LqrNode>());
  rclcpp::shutdown();
  return 0;
}
