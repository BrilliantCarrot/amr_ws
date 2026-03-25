#include "control_mpc/mpc_node.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>

namespace control_mpc
{

// ============================================================
// 생성자
// ============================================================

MpcNode::MpcNode(const rclcpp::NodeOptions & options)
: Node("mpc_node", options)
{
  // --- ROS2 파라미터 선언 ---
  // MPC 파라미터
  this->declare_parameter("N",          20);
  this->declare_parameter("dt",         0.02);
  this->declare_parameter("q_x",        10.0);
  this->declare_parameter("q_y",        10.0);
  this->declare_parameter("q_th",       5.0);
  this->declare_parameter("q_v",        1.0);
  this->declare_parameter("q_w",        1.0);
  this->declare_parameter("r_dv",       10.0);
  this->declare_parameter("r_dw",       5.0);
  this->declare_parameter("v_max",      0.5);
  this->declare_parameter("v_min",     -0.5);
  this->declare_parameter("w_max",      1.0);
  this->declare_parameter("w_min",     -1.0);
  this->declare_parameter("dv_max",     0.1);
  this->declare_parameter("dv_min",    -0.1);
  this->declare_parameter("dw_max",     0.2);
  this->declare_parameter("dw_min",    -0.2);

  // 경로 관련 파라미터
  this->declare_parameter("trajectory_type", std::string("straight"));
  this->declare_parameter("goal_tolerance",  0.1);

  // 파라미터 로드
  mpc_params_       = loadMpcParams();
  trajectory_type_  = this->get_parameter("trajectory_type").as_string();
  goal_tolerance_   = this->get_parameter("goal_tolerance").as_double();

  // --- MPC 초기화 ---
  mpc_core_.init(mpc_params_);
  RCLCPP_INFO(this->get_logger(),
    "MPC initialized: N=%d, dt=%.3f, traj=%s",
    mpc_params_.N, mpc_params_.dt, trajectory_type_.c_str());

  // --- Waypoint 초기화 ---
  initWaypoints();

  // --- 상태 초기화 ---
  x0_.setZero();
  u_prev_.setZero();
  loc_status_ = 0;

  // --- subscriber ---
  // /map_ekf/odom: map frame 기준 fused pose (W5에서 완성)
  // RELIABLE QoS — MPC 입력이므로 손실 없이 받아야 함
  auto qos_reliable = rclcpp::QoS(10).reliable();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/map_ekf/odom", qos_reliable,
    std::bind(&MpcNode::odomCallback, this, std::placeholders::_1));

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", qos_reliable,
    std::bind(&MpcNode::locStatusCallback, this, std::placeholders::_1));

  // --- publisher ---
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10).reliable());

  latency_pub_ = this->create_publisher<amr_msgs::msg::ControlLatency>(
    "/metrics/control_latency_ms", rclcpp::QoS(10).reliable());

  // --- 50Hz 제어 타이머 ---
  // dt = 20ms 주기로 controlCallback() 호출
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(mpc_params_.dt * 1000)),
    std::bind(&MpcNode::controlCallback, this));

  RCLCPP_INFO(this->get_logger(), "MpcNode started. Waiting for /map_ekf/odom...");
}

// ============================================================
// odomCallback() — /map_ekf/odom 수신 (50Hz)
//
//   nav_msgs::Odometry → StateVec [x, y, θ, v, ω] 변환
// ============================================================

void MpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 위치: map frame 기준
  x0_(0) = msg->pose.pose.position.x;
  x0_(1) = msg->pose.pose.position.y;

  // heading: quaternion → yaw 변환
  // quaternion (qx, qy, qz, qw) → yaw = atan2(2*(qw*qz + qx*qy), 1-2*(qy²+qz²))
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  x0_(2) = std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));

  // 속도: Odometry의 twist는 child_frame(base_link) 기준
  // Diff-Drive이므로 linear.x = v, angular.z = ω
  x0_(3) = msg->twist.twist.linear.x;
  x0_(4) = msg->twist.twist.angular.z;

  has_odom_ = true;
}

// ============================================================
// locStatusCallback() — /localization/status 수신 (10Hz)
// ============================================================

void MpcNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  loc_status_ = msg->status;

  // LOST 상태 경고
  if (loc_status_ == 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Localization LOST — MPC 정지 명령 발행 중");
  }
}

// ============================================================
// controlCallback() — 50Hz 제어 루프
//
//   매 20ms마다 실행:
//   1. 상태 확인 (odom 수신 여부, localization 상태)
//   2. reference trajectory 생성
//   3. MPC solve
//   4. u0 → v, ω 변환 후 /cmd_vel 발행
//   5. latency 발행
// ============================================================

void MpcNode::controlCallback()
{
  // odom 미수신 시 대기
  if (!has_odom_) {
    return;
  }

  // Localization LOST 시 정지
  if (loc_status_ == 2) {
    publishStop();
    return;
  }

  // 미션 완료 시 정지
  if (mission_done_) {
    publishStop();
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  // --- 1. reference trajectory 생성 ---
  std::vector<StateVec> x_ref = generateReference(x0_);

  // --- 2. MPC solve ---
  MpcSolution sol = mpc_core_.solve(x0_, x_ref, u_prev_);

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!sol.success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "MPC solve 실패 — 정지 명령");
    publishStop();
    return;
  }

  // --- 3. u0 → 실제 v, ω 계산 ---
  // u0 = [Δv, Δω] → v = v_prev + Δv, ω = ω_prev + Δω
  double v_cmd = u_prev_(0) + sol.u0(0);
  double w_cmd = u_prev_(1) + sol.u0(1);

  // DEGRADED 상태: 속도 50% 제한
  if (loc_status_ == 1) {
    v_cmd *= 0.5;
    w_cmd *= 0.5;
  }

  // 안전 클리핑 (혹시 모를 수치 오차 대비)
  v_cmd = std::clamp(v_cmd, mpc_params_.v_min, mpc_params_.v_max);
  w_cmd = std::clamp(w_cmd, mpc_params_.w_min, mpc_params_.w_max);

  // --- 4. /cmd_vel 발행 ---
  publishCmdVel(v_cmd, w_cmd);

  // 이전 입력 갱신 (다음 스텝 Δu 계산용)
  u_prev_(0) = v_cmd;
  u_prev_(1) = w_cmd;

  // --- 5. latency 통계 발행 ---
  latency_history_.push_back(elapsed_ms);

  // 최근 500샘플(10초)만 유지
  if (static_cast<int>(latency_history_.size()) > 500) {
    latency_history_.erase(latency_history_.begin());
  }

  // 평균 계산
  double avg_ms = 0.0;
  for (double v : latency_history_) avg_ms += v;
  avg_ms /= latency_history_.size();

  // 최대값
  double max_ms = *std::max_element(latency_history_.begin(), latency_history_.end());

  amr_msgs::msg::ControlLatency lat_msg;
  lat_msg.latency_ms     = elapsed_ms;
  lat_msg.avg_latency_ms = avg_ms;
  lat_msg.max_latency_ms = max_ms;
  latency_pub_->publish(lat_msg);

  // 주기적 로그 (1Hz)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[MPC] solve=%.2fms avg=%.2fms | v=%.3f w=%.3f | cost=%.2f",
    elapsed_ms, avg_ms, v_cmd, w_cmd, sol.cost);
}

// ============================================================
// initWaypoints() — 경로 waypoint 초기화
//
//   trajectory_type_ 파라미터에 따라 3가지 경로 생성:
//   - "straight" : 직선 5m
//   - "circle"   : 반지름 1.5m 원
//   - "figure8"  : 8자 경로
//
//   StateVec = [x, y, θ, v, ω]
//   v는 목표 속도 (reference에서 Q로 추적)
// ============================================================

void MpcNode::initWaypoints()
{
  waypoints_.clear();
  double v_ref = 0.3;   // 기준 주행 속도 [m/s]

  if (trajectory_type_ == "straight") {
    // 직선: x 방향으로 5m, 0.1m 간격
    int n_pts = 50;
    for (int i = 0; i <= n_pts; ++i) {
      StateVec wp;
      wp(0) = i * 0.1;   // x
      wp(1) = 0.0;       // y
      wp(2) = 0.0;       // θ (정면)
      wp(3) = v_ref;     // v
      wp(4) = 0.0;       // ω
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 직선 경로 %zu pts", waypoints_.size());

  } else if (trajectory_type_ == "circle") {
    // 원: 반지름 1.5m, 반시계 방향
    double r      = 1.5;
    double period = 2.0 * M_PI * r / v_ref;    // 한 바퀴 걸리는 시간 [s]
    double dt_wp  = 0.1;                         // waypoint 간격 [s]
    int    n_pts  = static_cast<int>(period / dt_wp);
    double w_ref  = v_ref / r;                   // ω = v/r

    for (int i = 0; i <= n_pts; ++i) {
      double t   = i * dt_wp;
      double ang = w_ref * t;   // 각도 진행
      StateVec wp;
      wp(0) = r * std::cos(ang) - r;   // 원점에서 출발하도록 오프셋
      wp(1) = r * std::sin(ang);
      wp(2) = ang + M_PI / 2.0;        // 접선 방향 (θ = 각도 + 90°)
      wp(2) = std::atan2(std::sin(wp(2)), std::cos(wp(2)));  // 정규화
      wp(3) = v_ref;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 원형 경로 %zu pts (r=%.1f)", waypoints_.size(), r);

  } else if (trajectory_type_ == "figure8") {
    // 8자: lemniscate 근사 (두 원을 이어붙인 형태)
    double r      = 1.0;
    double dt_wp  = 0.1;
    double w_ref  = v_ref / r;
    int    n_half = static_cast<int>(M_PI * r / v_ref / dt_wp);   // 반원 점 수

    // 첫 번째 원 (반시계)
    for (int i = 0; i <= n_half * 2; ++i) {
      double ang = w_ref * i * dt_wp;
      StateVec wp;
      wp(0) =  r * std::cos(ang) - r;
      wp(1) =  r * std::sin(ang);
      wp(2) = ang + M_PI / 2.0;
      wp(2) = std::atan2(std::sin(wp(2)), std::cos(wp(2)));
      wp(3) = v_ref;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    // 두 번째 원 (시계)
    for (int i = 0; i <= n_half * 2; ++i) {
      double ang = -w_ref * i * dt_wp;
      StateVec wp;
      wp(0) =  r * std::cos(ang) + r;
      wp(1) = -r * std::sin(ang);
      wp(2) = ang - M_PI / 2.0;
      wp(2) = std::atan2(std::sin(wp(2)), std::cos(wp(2)));
      wp(3) = v_ref;
      wp(4) = -w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 8자 경로 %zu pts", waypoints_.size());

  } else {
    RCLCPP_WARN(this->get_logger(),
      "알 수 없는 trajectory_type: %s → 직선으로 대체", trajectory_type_.c_str());
    // 재귀로 직선 생성
    trajectory_type_ = "straight";
    initWaypoints();
  }
}

// ============================================================
// findClosestWaypoint() — 현재 위치와 가장 가까운 waypoint 탐색
//
//   직전 인덱스(closest_wp_idx_) 근처에서만 탐색 (효율화)
//   로봇이 뒤로 가지 않는다는 가정 → 인덱스 단조 증가
// ============================================================

int MpcNode::findClosestWaypoint(const StateVec & x0)
{
  int n = static_cast<int>(waypoints_.size());
  int search_end = std::min(closest_wp_idx_ + 20, n);   // 앞으로 20개만 탐색

  double min_dist = 1e9;
  int    best_idx = closest_wp_idx_;

  for (int i = closest_wp_idx_; i < search_end; ++i) {
    double dx   = x0(0) - waypoints_[i](0);
    double dy   = x0(1) - waypoints_[i](1);
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }

  // goal tolerance 도달 시 미션 완료
  if (best_idx >= n - 1) {
    double dx = x0(0) - waypoints_.back()(0);
    double dy = x0(1) - waypoints_.back()(1);
    if (std::sqrt(dx * dx + dy * dy) < goal_tolerance_) {
      mission_done_ = true;
      RCLCPP_INFO(this->get_logger(), "미션 완료! 목표 지점 도달");
    }
  }

  return best_idx;
}

// ============================================================
// generateReference() — N+1개의 reference 상태 생성
//
//   closest waypoint에서 시작해서 N개를 순서대로 추출.
//   waypoint 부족하면 마지막 waypoint 반복.
// ============================================================

std::vector<StateVec> MpcNode::generateReference(const StateVec & x0)
{
  // 가장 가까운 waypoint 갱신
  closest_wp_idx_ = findClosestWaypoint(x0);

  int n = static_cast<int>(waypoints_.size());
  std::vector<StateVec> x_ref(mpc_params_.N + 1);

  for (int k = 0; k <= mpc_params_.N; ++k) {
    int idx = std::min(closest_wp_idx_ + k, n - 1);
    x_ref[k] = waypoints_[idx];
  }

  return x_ref;
}

// ============================================================
// publishCmdVel() — /cmd_vel 발행
// ============================================================

void MpcNode::publishCmdVel(double v, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = v;      // 선속도 [m/s]
  msg.angular.z = omega;  // 각속도 [rad/s]
  cmd_vel_pub_->publish(msg);
}

// ============================================================
// publishStop() — 정지 명령 발행
// ============================================================

void MpcNode::publishStop()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);

  // 이전 입력도 0으로 초기화
  u_prev_.setZero();
}

// ============================================================
// loadMpcParams() — ROS2 파라미터 → MpcParams 변환
// ============================================================

MpcParams MpcNode::loadMpcParams()
{
  MpcParams p;
  p.N      = this->get_parameter("N").as_int();
  p.dt     = this->get_parameter("dt").as_double();
  p.q_x    = this->get_parameter("q_x").as_double();
  p.q_y    = this->get_parameter("q_y").as_double();
  p.q_th   = this->get_parameter("q_th").as_double();
  p.q_v    = this->get_parameter("q_v").as_double();
  p.q_w    = this->get_parameter("q_w").as_double();
  p.r_dv   = this->get_parameter("r_dv").as_double();
  p.r_dw   = this->get_parameter("r_dw").as_double();
  p.v_max  = this->get_parameter("v_max").as_double();
  p.v_min  = this->get_parameter("v_min").as_double();
  p.w_max  = this->get_parameter("w_max").as_double();
  p.w_min  = this->get_parameter("w_min").as_double();
  p.dv_max = this->get_parameter("dv_max").as_double();
  p.dv_min = this->get_parameter("dv_min").as_double();
  p.dw_max = this->get_parameter("dw_max").as_double();
  p.dw_min = this->get_parameter("dw_min").as_double();
  return p;
}

}  // namespace control_mpc