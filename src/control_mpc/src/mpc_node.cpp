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
  this->declare_parameter("r_dv",       30.0);
  this->declare_parameter("r_dw",       20.0);
  this->declare_parameter("v_max",      0.5);
  this->declare_parameter("v_min",     -0.5);
  this->declare_parameter("w_max",      1.0);
  this->declare_parameter("w_min",     -1.0);
  this->declare_parameter("dv_max",     0.3);
  this->declare_parameter("dv_min",    -0.1);
  this->declare_parameter("dw_max",     0.2);
  this->declare_parameter("dw_min",    -0.2);

  // 장애물 관련 파라미터 (없다면 추가 선언)
  this->declare_parameter("obs_weight", 30.0);    // 기존 200.0 -> 50.0으로 수정
  this->declare_parameter("obs_safe_dist", 0.4);  // 기존 0.6 -> 0.4로 수정

  // 경로 관련 파라미터
  this->declare_parameter("trajectory_type", std::string("straight"));
  this->declare_parameter("goal_tolerance",  0.15);
  this->declare_parameter("delay_ms",        0.0); // 기본값은 지연 없음
  this->declare_parameter("slip_ratio",      0.0); // 기본값은 슬립 없음

  // 파라미터 로드
  mpc_params_       = loadMpcParams();
  trajectory_type_  = this->get_parameter("trajectory_type").as_string();
  goal_tolerance_   = this->get_parameter("goal_tolerance").as_double();
  delay_ms_         = this->get_parameter("delay_ms").as_double();
  slip_ratio_       = this->get_parameter("slip_ratio").as_double();

  // --- MPC 초기화 ---
  mpc_core_.init(mpc_params_);
  RCLCPP_INFO(this->get_logger(),
    "MPC initialized: N=%d, dt=%.3f, traj=%s",
    mpc_params_.N, mpc_params_.dt, trajectory_type_.c_str());

  // W8: 장애물 설정 (simple_room.sdf 기준)
  // obstacle_1: (1.0, 0.5), 크기 0.3×0.3 → 반경 0.15 + 로봇 반경 0.2 = 0.35
  // obstacle_2: (-1.0, -0.5), 크기 0.3×0.3 → 반경 0.35
  // obstacles_ = {
  //   {1.0,   0.5,  0.35},
  //   {-1.0, -0.5,  0.35}
  // };
  // mpc_core_.setObstacles(obstacles_);
  // RCLCPP_INFO(this->get_logger(), "W8: 장애물 %zu개 설정 완료", obstacles_.size());

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

    // 생성자에 추가 (qos_reliable 설정 부분 근처)
  obs_sub_ = this->create_subscription<amr_msgs::msg::ObstacleArray>(
    "/obstacles/detected", rclcpp::QoS(10).reliable(),
    std::bind(&MpcNode::obsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "W9: Obstacle Tracker 구독 시작"); // 확인용 로그 추가

  // --- publisher ---
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10).reliable());

  latency_pub_ = this->create_publisher<amr_msgs::msg::ControlLatency>(
    "/metrics/control_latency_ms", rclcpp::QoS(10).reliable());

  // tracking_rmse_node가 구독할 목표 지점 토픽
  // MPC가 현재 추종하려는 x_ref[0]을 map frame PoseStamped로 발행
  ref_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mpc/reference_pose", rclcpp::QoS(10).reliable());

  // W8: 최소 장애물 거리 발행
  min_dist_pub_ = this->create_publisher<amr_msgs::msg::MinObstacleDistance>(
    "/metrics/min_obstacle_distance", rclcpp::QoS(10).reliable());

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
  // x0_(2) = std::atan2(2.0 * (qw * qz + qx * qy),
  //                     1.0 - 2.0 * (qy * qy + qz * qz));
  double raw_theta = std::atan2(2.0 * (qw * qz + qx * qy),  
                                1.0 - 2.0 * (qy * qy + qz * qz));
  double delta = std::atan2(std::sin(raw_theta - prev_raw_theta_),
                             std::cos(raw_theta - prev_raw_theta_));
  continuous_theta_ += delta;
  prev_raw_theta_    = raw_theta;
  x0_(2) = continuous_theta_;

  // 속도: Odometry의 twist는 child_frame(base_link) 기준
  // Diff-Drive이므로 linear.x = v, angular.z = ω
  x0_(3) = msg->twist.twist.linear.x;
  x0_(4) = msg->twist.twist.angular.z;

  has_odom_ = true;

  // 첫 odom 수신 시 waypoint 재초기화 (로봇 실제 시작 위치 반영)
  // if (!has_odom_) {
  //   has_odom_ = true;
  //   init_waypoint_timer_ = this->create_wall_timer(
  //       std::chrono ::milliseconds(500),
  //       [this]()
  //       {
  //         initWaypoints();
  //         init_waypoint_timer_->cancel();
  //       });
    
  //   return;
  // }
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

// 소스 하단에 콜백 함수 구현
void MpcNode::obsCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg)
{
  constexpr double ROBOT_RADIUS = 0.2;
  obstacles_.clear();
  for (int i = 0; i < msg->count; ++i) {
    Obstacle obs;
    obs.x = msg->x[i];
    obs.y = msg->y[i];
    obs.radius = msg->radius[i] + ROBOT_RADIUS;
    obs.vx = msg->vx[i];
    obs.vy = msg->vy[i];
    obstacles_.push_back(obs);
  }
  mpc_core_.setObstacles(obstacles_);

  // 디버깅: 수신된 장애물 개수가 0보다 클 때만 출력
  if (!obstacles_.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "장애물 %zu개 수신 중...", obstacles_.size());
  } 
}

// ============================================================
// controlCallback() — 50Hz 제어 루프
//
//   매 20ms마다 실행:
//   1. 상태 확인 (odom 수신 여부, localization 상태)
//   2. 상태 예측 지연 보상 (Delay Compensation) - 연속성 유지
//   3. reference trajectory 생성 (예측된 미래 상태 기준)
//   4. MPC solve
//   5. u0 → v, ω 변환 후 /cmd_vel 발행 (지연 큐로 전달)
//   6. latency 발행
// ============================================================

void MpcNode::controlCallback()
{
  // 1. odom 미수신 시 대기
  if (!has_odom_) {
    return;
  }

  // 2. Localization LOST 시 즉시 정지 및 루프 탈출
  if (loc_status_ == 2) {
    publishStop();
    return;
  }

  // 3. 미션 완료 시 정지 명령 유지 및 제어 루프 완전 차단
  // (이 return이 없으면 목표점 도달 후에도 로봇이 멈추지 않고 계속 맴돌게 됩니다)
  if (mission_done_) {
    publishStop();
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  // ----------------------------------------------------------------
  // [지연 보상 (Delay Compensation) 로직]
  // 큐에 대기 중인 과거의 명령들을 이용해, 지연 시간(delay_ms_) 후 
  // 로봇이 실제로 도달해 있을 미래 위치를 물리 모델로 미리 계산
  // 현재 상태(x0_)에서 출발하여, 큐에 쌓여있는(아직 실행 안 된) 명령들을 
  // 운동학 모델로 미리 시뮬레이션하여 지연 시간 이후의 미래 상태를 예측함.
  // ----------------------------------------------------------------
  StateVec x_pred = x0_;
  if (delay_ms_ > 0.0 && !delay_queue_.empty()) {
    double dt_step = mpc_params_.dt;
    for (const auto & msg : delay_queue_) {
      double v = msg.linear.x;
      double w = msg.angular.z;

      // Forward Kinematics (예측)
      x_pred(0) += v * std::cos(x_pred(2)) * dt_step;
      x_pred(1) += v * std::sin(x_pred(2)) * dt_step;
      x_pred(2) += w * dt_step;
      x_pred(3) = v;
      x_pred(4) = w;
    }
    // 주의: 여기서 x_pred(2)에 대해 atan2 정규화를 하지 않음
    // EKF에서 올라온 누적 연속 Yaw 값을 그대로 유지해야 2\pi 점프에 의한 발산이 안 생깁니다.
  }

  // --- 1. reference trajectory 생성 ---
  // 현재 위치(x0_)가 아닌 '예측된 미래 위치(x_pred)'를 기준으로 목표 궤적을 깎음
  std::vector<StateVec> x_ref = generateReference(x_pred);

  // x_ref[0] = MPC가 현재 스텝에서 추종하려는 목표 상태 [x,y,θ, v, ω], 이것을 PoseStamped로 변환
  // x_ref[0]을 /mpc/reference_pose로 발행 (tracking_rmse_node용)
  // 왜 x_ref[0]인가? x_ref는 N+1개(현재~N스텝 후)의 목표 시퀀스인데, 현재 스텝에서 로봇이 있어야 할 
  // 위치는 x_ref[0]. "지금 이 순간의 목표"를 비교 기준으로 삼는 것.
  {
    geometry_msgs::msg::PoseStamped ref_msg;
    ref_msg.header.stamp    = this->now();
    ref_msg.header.frame_id = "map"; // map 프레임 기준
    ref_msg.pose.position.x = x_ref[0](0); // 목표 X
    ref_msg.pose.position.y = x_ref[0](1); // 목표 Y
    ref_msg.pose.position.z = 0.0;
    // 2D 평면이므로 roll=pitch=0, yaw=θ 만 사용
    // quaternion = (0, 0, sin(θ/2), cos(θ/2))
    double half_yaw = x_ref[0](2) * 0.5;
    ref_msg.pose.orientation.x = 0.0;
    ref_msg.pose.orientation.y = 0.0;
    ref_msg.pose.orientation.z = std::sin(half_yaw);
    ref_msg.pose.orientation.w = std::cos(half_yaw);
    ref_pose_pub_->publish(ref_msg);
  }

  // --- 2. MPC solve ---
  // 최적화 시작점도 '미래 위치'로 설정합니다.
  MpcSolution sol = mpc_core_.solve(x_pred, x_ref, u_prev_);

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // 최적해 도출 실패 시 비상 정지
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

  // DEGRADED 상태: 속도를 안전하게 50%로 제한
  if (loc_status_ == 1) {
    v_cmd *= 0.5;
    w_cmd *= 0.5;
  }

  // 물리적 한계치 클리핑 (파라미터 제약 준수)
  v_cmd = std::clamp(v_cmd, mpc_params_.v_min, mpc_params_.v_max);

  // 장애물 거리 기반 속도 스케일링
  // obs_safe_dist(0.6m) 이내로 들어오면 속도를 점진적으로 줄임
  // v_min_ratio=0.3: 최소 30% 속도는 보장 (정지 방지)
  if (!obstacles_.empty()) {
    double min_dist = 1e9;
    for (const auto & obs : obstacles_) {
      double d = std::hypot(x0_(0)-obs.x, x0_(1)-obs.y) - obs.radius;
      min_dist = std::min(min_dist, d);
    }
    constexpr double slow_start = 0.5;  // 감속 시작 거리 [m]
    constexpr double v_min_ratio = 0.3; // 최소 속도 비율 (30%)
    if (min_dist < slow_start && v_cmd > 0.0) {
      // double scale = std::max(min_dist / slow_start, v_min_ratio);
      // v_cmd *= scale;
      v_cmd = std::max(v_cmd, 0.2);
    }
  }

  w_cmd = std::clamp(w_cmd, mpc_params_.w_min, mpc_params_.w_max);

  // --- 4. /cmd_vel 발행 (큐를 통해 지연 발행됨) ---
  publishCmdVel(v_cmd, w_cmd);

  // 이전 입력 갱신 (다음 스텝의 Δu 계산 및 지연 보상에 사용)
  u_prev_(0) = v_cmd;
  u_prev_(1) = w_cmd;

  // --- 5. latency 통계 산출 및 발행 ---
  latency_history_.push_back(elapsed_ms);

  // 최근 500샘플(10초)만 유지
  if (static_cast<int>(latency_history_.size()) > 500) {
    latency_history_.erase(latency_history_.begin());
  }

  // 평균 계산
  double avg_ms = 0.0;
  for (double v : latency_history_) avg_ms += v;
  avg_ms /= latency_history_.size();

  // 최대값 계산
  double max_ms = *std::max_element(latency_history_.begin(), latency_history_.end());

  amr_msgs::msg::ControlLatency lat_msg;
  lat_msg.latency_ms     = elapsed_ms;
  lat_msg.avg_latency_ms = avg_ms;
  lat_msg.max_latency_ms = max_ms;
  latency_pub_->publish(lat_msg);

  // W8: 현재 위치(x0_)에서 각 장애물까지 최소 거리 계산 및 발행
  {
    double min_dist = 1e9;
    for (const auto & obs : obstacles_) {
      double dx = x0_(0) - obs.x;
      double dy = x0_(1) - obs.y;
      double clearance = std::sqrt(dx * dx + dy * dy) - obs.radius;
      min_dist = std::min(min_dist, clearance);
    }
    amr_msgs::msg::MinObstacleDistance dist_msg;
    dist_msg.min_distance_m = min_dist;
    min_dist_pub_->publish(dist_msg);

    // 경고: 안전 거리(0.2m) 이내 진입 시 로그
    if (min_dist < -0.2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "장애물 근접! min_clearance=%.3fm", min_dist);
    }
  }

  // 주기적 터미널 로그 (1Hz)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
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
  double v_ref = 0.15;  // 원형 경로: 선형화 오차 줄이기 위해 속도 절반
  // v=0.3이면 N=20 구간(0.4s)에서 heading이 0.12rad 변화
  // v=0.15이면 0.06rad 변화 → linearization 오차 절반

  if (trajectory_type_ == "straight") {
    // 직선: x 방향으로 5m, 0.1m 간격
    int n_pts = 50;
    for (int i = 0; i <= n_pts; ++i) {
      StateVec wp;
      wp(0) = i * 0.1;   // x
      wp(1) = 0.0;       // y
      // wp(1) = x0_(1); // 로봇 초기 y 위치로 맞춤
      wp(2) = 0.0;     // θ (정면)
      wp(3) = 0.3;     // v
      wp(4) = 0.0;       // ω
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 직선 경로 %zu pts", waypoints_.size());

  } else if (trajectory_type_ == "circle") {
    // 원: 반지름 1.5m, 반시계 방향
    double r      = 0.5;
    double period = 2.0 * M_PI * r / v_ref;    // 한 바퀴 걸리는 시간 [s]
    // MPC 예측 주기와 동일하게 맞춤 (0.02s)
    double dt_wp  = mpc_params_.dt;                         // waypoint 간격 [s]
    int    n_pts  = static_cast<int>(period / dt_wp);
    double w_ref  = v_ref / r;                   // ω = v/r

    for (int i = 0; i <= n_pts; ++i) {
      double t   = i * dt_wp;
      double ang = w_ref * t;   // 각도 진행
      StateVec wp;
      wp(0) = r * std::cos(ang) - r;   // 원점에서 출발하도록 오프셋
      wp(1) = r * std::sin(ang);
      wp(2) = ang + M_PI / 2.0;        // 접선 방향 (θ = 각도 + 90°)
      wp(3) = v_ref;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 원형 경로 %zu pts (r=%.1f)", waypoints_.size(), r);

  } else if (trajectory_type_ == "figure8") {
    // 8자: lemniscate 근사 (두 원을 이어붙인 형태)
    double r      = 0.7;
    double dt_wp  = mpc_params_.dt;  // 예측 주기와 동일하게 맞춤 (0.02)
    double w_ref  = v_ref / r;
    int    n_half = static_cast<int>(M_PI * r / v_ref / dt_wp);

    // 첫 번째 원 (반시계, 왼쪽 원)
    for (int i = 0; i <= n_half * 2; ++i) {
      double ang = w_ref * i * dt_wp;
      StateVec wp;
      wp(0) =  r * std::cos(ang) - r;
      wp(1) =  r * std::sin(ang);
      // wp(2)는 generateReference에서 실시간 계산하므로 생략
      wp(3) = v_ref;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    
    // 두 번째 원 (시계, 오른쪽 원)
    // i = 1부터 시작하여 첫 번째 원의 마지막 점(0,0)과 중복되는 것을 방지
    for (int i = 1; i <= n_half * 2; ++i) {
      double ang = M_PI - w_ref * i * dt_wp;
      StateVec wp;
      wp(0) =  r * std::cos(ang) + r;
      wp(1) =  r * std::sin(ang);
      // wp(2)는 실시간 계산하므로 생략
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

  // ----------------------------------------------------------------
  // [종료 조건] 거리(Tolerance)나 특정 좌표에 얽매이지 않고,
  // '로봇이 추종해야 할 궤적의 맨 끝단(마지막 2개 포인트)에 도달했는가?'로 판정
  // n-2 도달 시 목표를 달성한 것으로 간주하고 제어 루프를 종료함.
  // ----------------------------------------------------------------
  if (best_idx >= n - 2) {
    // double dx = x0(0) - waypoints_.back()(0);
    // double dy = x0(1) - waypoints_.back()(1);
    // if (std::sqrt(dx * dx + dy * dy) < goal_tolerance_) {
      mission_done_ = true;
      RCLCPP_INFO(this->get_logger(), "미션 완료! 목표 지점 도달 (현재 idx: %d / 총 %d)", best_idx, n-1);
    // }
  }

  return best_idx;
}

// ============================================================
// generateReference() — N+1개의 reference 상태 생성
//
//   closest waypoint에서 시작해서 N개를 순서대로 추출.
//   waypoint 부족하면 마지막 waypoint 반복.
// ============================================================
/**
 * @brief MPC의 예측 지평선(N) 동안 로봇이 추종해야 할 목표 상태(Reference Trajectory)를 생성합니다.
 * @param x0 현재 로봇의 상태 [x, y, θ, v, ω] (지연 보상이 적용된 예측 위치일 수 있음)
 * @return std::vector<StateVec> N+1개의 목표 상태 벡터 시퀀스
 */
std::vector<StateVec> MpcNode::generateReference(const StateVec & x0)
{
  // 가장 가까운 waypoint 갱신
  // 1. 현재 위치에서 전체 경로(Waypoints) 중 가장 가까운 점의 인덱스를 찾습니다.
  // 이 인덱스를 기준으로 MPC의 예측 지평선(Horizon)을 전개합니다.
  closest_wp_idx_ = findClosestWaypoint(x0);

  int n = static_cast<int>(waypoints_.size());
  // 2. 결과를 담을 벡터를 선언합니다. 크기는 N+1입니다.
  // (현재 시점 k=0부터 예측 종료 시점 k=N까지 총 N+1개의 상태가 필요함)
  std::vector<StateVec> x_ref(mpc_params_.N + 1);
  // 3. 예측 지평선의 각 스텝(k)에 대해 목표 상태를 결정합니다.
for (int k = 0; k <= mpc_params_.N; ++k) {
    // [인덱싱 관리]
    // 현재 인덱스에서 k만큼 떨어진 점을 가져오되, 전체 경로의 마지막 점(n-1)을 넘지 않도록 제한(std::min)합니다.
    int idx      = std::min(closest_wp_idx_ + k, n - 1);
    int idx_next = std::min(idx + 1, n - 1);
    // 기본 정보(좌표, 속도 등)는 미리 저장된 waypoints_ 배열에서 복사합니다.
    x_ref[k] = waypoints_[idx];

    // 실시간 Heading 계산
    // -------------------------------------------------------------------------
    // [실시간 Heading(Yaw) 및 연속성 계산]
    // 경로 데이터에는 좌표만 있는 경우가 많으므로, 두 점 사이의 기하학적 각도를 실시간으로 계산합니다.
    // -------------------------------------------------------------------------
    // 현재 점과 다음 점 사이의 위치 차이를 계산합니다.
    double dx = waypoints_[idx_next](0) - waypoints_[idx](0);
    double dy = waypoints_[idx_next](1) - waypoints_[idx](1);

    // 이전 x_ref yaw 기준으로 연속성 유지 (-π~π 점프 방지)
    // [연속성 유지를 위한 기준 각도 설정]
    // k=0(현재 시점)이면 로봇의 현재 각도(x0(2))를 기준으로 삼고,
    // k>0이면 직전 스텝에서 계산된 목표 각도(x_ref[k-1](2))를 기준으로 삼습니다.
    double ref_yaw_base = (k == 0) ? x0(2) : x_ref[k - 1](2);
    double raw_yaw;

    // 두 waypoint가 너무 가까워서 dx, dy가 0에 수렴하는 경우 방어 (atan2(0,0) 발산 방지)
    // [수학적 특이점 방어 (Singularity Defense)]
    // 두 점 사이의 거리가 너무 가까우면(1e-5 미만) atan2(0,0)에 의해 각도가 0으로 발산할 수 있습니다.
    // 이 경우 방향을 정의할 수 없으므로, 이전의 기준 각도를 그대로 유지하여 급격한 회전을 방지합니다.
    if (std::abs(dx) < 1e-5 && std::abs(dy) < 1e-5) {
      raw_yaw = ref_yaw_base; // 방향을 알 수 없으므로 직전의 헤딩을 그대로 유지
    } else {
      // 일반적인 경우, 두 점이 이루는 벡터의 탄젠트 각도를 계산합니다.
      raw_yaw = std::atan2(dy, dx);
    }

    // 연속성 유지의 핵심 로직
    // [각도 불연속성 해결 (Yaw Continuity Logic)]
    // atan2는 -π ~ π 사이의 값만 반환하므로, 각도가 경계선을 지날 때 2π(360도)만큼 값이 튀는 현상이 발생합니다.
    // 이를 방지하기 위해 '기준 각도'와 '계산된 각도' 사이의 최소 차이(diff)를 구합니다.
    // sin/cos을 사용한 atan2 방식은 각도 차이를 항상 -π ~ π 범위로 정규화해 주는 표준적인 기법입니다.
    double diff = std::atan2(
      std::sin(raw_yaw - ref_yaw_base),
      std::cos(raw_yaw - ref_yaw_base));
    // 기준 각도에 최소 차이만큼만 더해줌으로써, 각도가 경계선을 넘어도 
    // 값이 튀지 않고 3.14 -> 3.15... 처럼 선형적으로 증가(Continuous Yaw)하게 만듭니다.
    // 이는 선형화 MPC의 안정성을 결정짓는 가장 중요한 수치 처리 과정입니다.
    x_ref[k](2) = ref_yaw_base + diff;
  }

  return x_ref;
}

// ============================================================
// publishCmdVel() — /cmd_vel 발행 (지연 및 슬립 외란 적용)
// 자율주행에서 제어 지연이란 "로봇의 뇌(CPU)가 멈추는 것"이 아니라, "CPU는 50Hz로 빠르게 생각하는데, 
// 명령이 바퀴(모터)까지 전달되는 데 시간이 걸리는 것(통신/기구학적 지연)"을 의미.
// ============================================================
void MpcNode::publishCmdVel(double v, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = v;      
  msg.angular.z = omega;  

  if (delay_ms_ > 0.0) {
    // 1. 계산된 원본 명령을 큐에 보관 (MPC는 이 값을 바탕으로 미래를 예측함)
    delay_queue_.push_back(msg);

    int delay_steps = static_cast<int>(delay_ms_ / (mpc_params_.dt * 1000.0));

    if (static_cast<int>(delay_queue_.size()) > delay_steps) {
      geometry_msgs::msg::Twist pop_msg = delay_queue_.front();
      delay_queue_.pop_front();
      
      // [외란 주입] 실제 모터로 전달되기 직전에 슬립 발생
      // 로봇은 MPC가 의도한 속도보다 slip_ratio_ 만큼 느리게 움직이게 됨
      geometry_msgs::msg::Twist slip_msg;
      slip_msg.linear.x  = pop_msg.linear.x * (1.0 - slip_ratio_);
      slip_msg.angular.z = pop_msg.angular.z * (1.0 - slip_ratio_);
      cmd_vel_pub_->publish(slip_msg);
    } else {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
    }
  } else {
    // 지연이 없을 때도 슬립 외란 적용
    geometry_msgs::msg::Twist slip_msg;
    slip_msg.linear.x  = msg.linear.x * (1.0 - slip_ratio_);
    slip_msg.angular.z = msg.angular.z * (1.0 - slip_ratio_);
    cmd_vel_pub_->publish(slip_msg);
  }
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

  // 이전 입력 초기화 및 남아있는 지연 명령 싹 비우기
  u_prev_.setZero();
  delay_queue_.clear();
}
// // ============================================================
// // publishCmdVel() — /cmd_vel 발행
// // ============================================================

// void MpcNode::publishCmdVel(double v, double omega)
// {
//   geometry_msgs::msg::Twist msg;
//   msg.linear.x  = v;      // 선속도 [m/s]
//   msg.angular.z = omega;  // 각속도 [rad/s]
//   cmd_vel_pub_->publish(msg);
// }

// // ============================================================
// // publishStop() — 정지 명령 발행
// // ============================================================

// void MpcNode::publishStop()
// {
//   geometry_msgs::msg::Twist msg;
//   msg.linear.x  = 0.0;
//   msg.angular.z = 0.0;
//   cmd_vel_pub_->publish(msg);

//   // 이전 입력도 0으로 초기화
//   u_prev_.setZero();
// }

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
  // W8/W9 추가 파라미터 로드 확인
  p.obs_weight = this->get_parameter("obs_weight").as_double();
  p.obs_safe_dist = this->get_parameter("obs_safe_dist").as_double();
  return p;
}

}  // namespace control_mpc