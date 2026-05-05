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
  this->declare_parameter("v_max",      0.15); // 로봇 최대 선속도
  this->declare_parameter("v_min",     -0.1); // 로봇 최소 선속도
  this->declare_parameter("w_max",      1.0);
  this->declare_parameter("w_min",     -1.0);
  this->declare_parameter("dv_max",     0.3);
  this->declare_parameter("dv_min",    -0.1);
  this->declare_parameter("dw_max",     0.2);
  this->declare_parameter("dw_min",    -0.2);
  this->declare_parameter("v_ref_", 0.1); // 로봇 이동 선속도

  // 장애물 관련 파라미터 (없다면 추가 선언)
  this->declare_parameter("obs_weight", 30.0);    // 기존 200.0에서
  this->declare_parameter("obs_safe_dist", 0.2);  // 기존 0.6->0.4

  // 경로 관련 파라미터
  this->declare_parameter("trajectory_type", std::string("straight"));
  this->declare_parameter("goal_tolerance",  0.15);
  this->declare_parameter("delay_ms",        0.0); // 기본값은 지연 없음
  this->declare_parameter("slip_ratio",      0.0); // 기본값은 슬립 없음
  this->declare_parameter("artificial_load_ms", 0.0); // 기본 cpu 부하는 없음
  // W12: 글로벌 경로계획 연동 파라미터
  // true: path_planner_node의 /planned_path 수신 후 제어 시작
  // false: 기존 hardcoded 경로(trajectory_type) 사용 (하위호환)
  this->declare_parameter("use_global_planner", false);

  // 도킹 관련 파라미터 선언
  // dock_yaw: 도킹 목표 헤딩 [rad] — 예) 1.5708 = 90° (좌측 도킹)
  this->declare_parameter("dock_yaw",         0.0);
  // dock_pos_tol: 위치 완료 판정 임계값 [m]
  this->declare_parameter("dock_pos_tol",     0.05);
  // dock_yaw_tol_deg: 헤딩 완료 판정 임계값 [deg], 내부에서 rad로 변환
  this->declare_parameter("dock_yaw_tol_deg", 3.0);

  // CBF Safety Filter 파라미터 선언
  // cbf_gamma   : class-K 계수 (α(h) = γ*h) — 추천 1.0~3.0
  // cbf_slack_p : soft CBF 페널티 — 추천 500.0
  // cbf_d_safe  : 추가 안전 마진 [m] — 0이면 obs.radius만 사용
  // cbf_enabled : false로 하면 CBF 건너뜀 (비교 실험용)
  this->declare_parameter("cbf_gamma",   2.0);
  this->declare_parameter("cbf_slack_p", 1000.0);
  this->declare_parameter("cbf_d_safe",  0.20);
  this->declare_parameter("cbf_lookahead", 0.25);
  this->declare_parameter("cbf_enabled", true);

  // 파라미터 로드
  mpc_params_       = loadMpcParams();
  trajectory_type_  = this->get_parameter("trajectory_type").as_string();
  goal_tolerance_   = this->get_parameter("goal_tolerance").as_double();
  delay_ms_         = this->get_parameter("delay_ms").as_double();
  slip_ratio_       = this->get_parameter("slip_ratio").as_double();
  artificial_load_ms_ = this->get_parameter("artificial_load_ms").as_double();
  use_global_planner_ = this->get_parameter("use_global_planner").as_bool();

  // 도킹 파라미터 로드
  dock_yaw_     = this->get_parameter("dock_yaw").as_double();
  dock_pos_tol_ = this->get_parameter("dock_pos_tol").as_double();
  dock_yaw_tol_ = this->get_parameter("dock_yaw_tol_deg").as_double()
                  * M_PI / 180.0;  // deg → rad 변환

  // CBF 파라미터 로드 및 필터 초기화
  cbf_gamma_   = this->get_parameter("cbf_gamma").as_double();
  cbf_slack_p_ = this->get_parameter("cbf_slack_p").as_double();
  cbf_d_safe_  = this->get_parameter("cbf_d_safe").as_double();
  cbf_enabled_ = this->get_parameter("cbf_enabled").as_bool();
  double cbf_lookahead = this->get_parameter("cbf_lookahead").as_double();

  CbfFilter::Params cbf_params;
  cbf_params.gamma   = cbf_gamma_;
  cbf_params.slack_p = cbf_slack_p_;
  cbf_params.d_safe  = cbf_d_safe_;
  cbf_params.v_max   = mpc_params_.v_max;
  cbf_params.v_min   = mpc_params_.v_min;
  cbf_params.w_max   = mpc_params_.w_max;
  cbf_params.w_min   = mpc_params_.w_min;
  cbf_params.lookahead = cbf_lookahead;
  cbf_params.react_dist = 0.35;
  cbf_params.rear_margin = 0.05;
  cbf_params.max_active_obstacles = 1;
  cbf_filter_ = std::make_unique<CbfFilter>(cbf_params);



  RCLCPP_INFO(this->get_logger(),
      "[CBF] 초기화 완료 | gamma=%.2f slack_p=%.1f d_safe=%.3fm L=%.3fm enabled=%s",
    cbf_gamma_, cbf_slack_p_, cbf_d_safe_, cbf_lookahead, cbf_enabled_ ? "true" : "false");

  RCLCPP_INFO(this->get_logger(),
    "[Docking] 파라미터 로드 완료 | dock_yaw=%.2f° pos_tol=%.3fm yaw_tol=%.2f°",
    dock_yaw_ * 180.0 / M_PI, dock_pos_tol_,
    dock_yaw_tol_ * 180.0 / M_PI);

  // 런타임 파라미터 변경 콜백
  // ros2 param set으로 변경 시 즉시 멤버변수에 반영
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      for (const auto & p : params) {
        if (p.get_name() == "artificial_load_ms")
          artificial_load_ms_ = p.as_double();
        else if (p.get_name() == "slip_ratio")
          slip_ratio_ = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  v_ref_ = this->get_parameter("v_ref_").as_double();

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
  // 글로벌 경로계획 모드: hardcoded 경로 초기화 건너뜀
  // → /planned_path 수신 시 pathCallback에서 waypoints_ 갱신
  if (!use_global_planner_) {
    initWaypoints();
  } else {
    RCLCPP_INFO(this->get_logger(),
      "[MPC] 글로벌 경로계획 모드: /planned_path 대기 중...");
  }

  // --- 상태 초기화 ---
  x0_.setZero();
  u_prev_.setZero();
  loc_status_ = 0; // 추후 삭제 가능

  // --- subscriber ---
  // /map_ekf/odom: map frame 기준 fused pose (W5에서 완성)
  // RELIABLE QoS — MPC 입력이므로 손실 없이 받아야 함

  sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sensor_opt = rclcpp::SubscriptionOptions();
  sensor_opt.callback_group = sensor_cb_group_;

  auto qos_reliable = rclcpp::QoS(10).reliable();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/map_ekf/odom", qos_reliable,
    std::bind(&MpcNode::odomCallback, this, std::placeholders::_1), sensor_opt);

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", qos_reliable,
    std::bind(&MpcNode::locStatusCallback, this, std::placeholders::_1), sensor_opt);

    // 생성자에 추가 (qos_reliable 설정 부분 근처)
  obs_sub_ = this->create_subscription<amr_msgs::msg::ObstacleArray>(
    "/obstacles/detected", rclcpp::QoS(10).reliable(),
    std::bind(&MpcNode::obsCallback, this, std::placeholders::_1), sensor_opt);

  safety_sub_ = this->create_subscription<amr_msgs::msg::SafetyStatus>(
  "/safety/state", qos_reliable,
  std::bind(&MpcNode::safetyStateCallback, this, std::placeholders::_1), sensor_opt);

  // W12: /planned_path 구독 — path_planner_node가 발행한 A* 경로 수신
  // use_global_planner=true일 때만 의미있으나, 항상 구독해두어 런타임 전환 가능
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planned_path", rclcpp::QoS(10).reliable(),
    std::bind(&MpcNode::pathCallback, this, std::placeholders::_1), sensor_opt);

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

  // 긴급 정지용 publisher — Gazebo 브릿지 직접 연결
  emergency_stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_delayed", rclcpp::QoS(10).reliable());

  // --- 50Hz 제어 타이머 ---
  // dt = 20ms 주기로 controlCallback() 호출
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(mpc_params_.dt * 1000)),
    std::bind(&MpcNode::controlCallback, this), control_cb_group_);

  RCLCPP_INFO(this->get_logger(), "MpcNode started. Waiting for /map_ekf/odom...");
}

// ============================================================
// odomCallback() — /map_ekf/odom 수신 (50Hz)
//
//   nav_msgs::Odometry → StateVec [x, y, θ, v, ω] 변환
// ============================================================

void MpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

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
  last_odom_stamp_ = this->get_clock()->now();

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
// locStatusCallback() — /localization/status 수신 (10Hz), 추후 삭제
// ============================================================

void MpcNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  loc_status_ = msg->status;

  // LOST 상태 경고
  if (loc_status_ == 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Localization LOST — MPC 정지 명령 발행 중");
  }
}

void MpcNode::safetyStateCallback(
  const amr_msgs::msg::SafetyStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  safety_state_ = msg->state;
}

// void MpcNode::obsCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(state_mutex_);

//   // ---------------------------------------------------------------
//   // 글로벌 경로계획 모드(use_global_planner=true)에서는
//   // 장애물 회피를 path_planner_node(A*)가 전담하므로
//   // MPC에 obstacle penalty를 전달하지 않음.
//   //
//   // [역할 분리 원칙]
//   //   path_planner_node : 경로계획 + 장애물 회피 (정적/동적 모두)
//   //   mpc_node          : 순수 경로 추종만 담당
//   //
//   // MPC가 동시에 obstacle penalty를 적용하면:
//   //   A*가 만든 회피 경로 ↔ MPC penalty 방향이 충돌
//   //   → QP infeasible → solve 실패 반복
//   // ---------------------------------------------------------------
//   if (use_global_planner_) {
//     return;
//   }

//   // 하드코딩 모드(use_global_planner=false)에서만 기존 동작 유지
//   constexpr double ROBOT_RADIUS = 0.2;
//   // 로봇을 점(point)으로 취급하는 대신, 장애물 반경을 미리 로봇 반경만큼 키워두는 트릭
//   // path_planner_node의 inflateMap()과 동일한 개념
//   obstacles_.clear(); // 새 메시지가 올 때마다 이전 프레임을 지우고 장애물을 최신 상태로 갱신
//   // 왜? 로봇이 움직이면서 장애물 위치도 상대적으로 움직이기 때문
//   // 안하면, 로봇이 사라진 장애물이 여전히 있다고 인식해서 불필요하게 회피하거나 경로가 이상해짐
//   // 장애물 반경 팽창 후 저장
//   for (int i = 0; i < msg->count; ++i) {
//     Obstacle obs;
//     obs.x      = msg->x[i];
//     obs.y      = msg->y[i];
//     obs.radius = msg->radius[i] + ROBOT_RADIUS;
//     obs.vx     = msg->vx[i];
//     obs.vy     = msg->vy[i];
//     obstacles_.push_back(obs);
//   }
//   mpc_core_.setObstacles(obstacles_); // 가공된 장애물 목록을 MPC 코어에 넘김
//   // 왜 use_global_planner=false 일 때만:
//   // use_global_planner=true(A* 연동 모드)에서는 path_planner_node가 이미 장애물을 맵에 마킹하고 회피 경로를 만들어서 
//   // /planned_path로 줌. MPC는 그 경로를 그냥 추종하면 되므로 별도로 장애물을 받을 필요가 없음 
//   // 반면 하드코딩 모드에서는 전역 경로 계획이 없으므로 MPC가 직접 장애물을 인식해서 회피해

//   if (!obstacles_.empty()) {
//     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//       "장애물 %zu개 수신 중...", obstacles_.size());
//   }
// }

void MpcNode::obsCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  constexpr double ROBOT_RADIUS = 0.2;
  // 동적 장애물 판별 속도 임계값 [m/s]
  // EMA 필터(α=0.08) 특성상 정지 장애물도 초기 몇 프레임은
  // 속도가 튈 수 있으므로 0.05보다 약간 높은 0.08로 설정
  constexpr double DYNAMIC_SPEED_THRESH = 0.08;

  obstacles_.clear();

  for (int i = 0; i < msg->count; ++i) {
    double speed = std::hypot(msg->vx[i], msg->vy[i]);

    if (use_global_planner_) {
      // ---------------------------------------------------------------
      // 글로벌 경로계획 모드: 동적 장애물만 MPC에 전달
      //
      // [역할 분리]
      //   정적 장애물 → A*가 inflated 맵에서 전담 회피
      //   동적 장애물 → MPC penalty로 실시간 회피
      //
      // A*가 경로를 재계획하는 주기(2~3초) 사이의 공백에서
      // 동적 장애물이 경로에 침범할 수 있으므로
      // MPC가 실시간으로 penalty를 적용해 회피함
      // ---------------------------------------------------------------
      if (speed < DYNAMIC_SPEED_THRESH) continue; // 정적 장애물 스킵
    }
    // use_global_planner_=false이면 기존처럼 전체 장애물 처리

    Obstacle obs;
    obs.x      = msg->x[i];
    obs.y      = msg->y[i];
    obs.radius = msg->radius[i] + ROBOT_RADIUS;
    obs.vx     = msg->vx[i];
    obs.vy     = msg->vy[i];
    obstacles_.push_back(obs);
  }

  mpc_core_.setObstacles(obstacles_);

  if (!obstacles_.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[MPC] 장애물 수신: 전체=%d, MPC 전달=%zu (동적 필터 적용: %s)",
      msg->count, obstacles_.size(),
      use_global_planner_ ? "ON" : "OFF");
  }
}

// ============================================================
// pathCallback() — /planned_path 수신 (W12 글로벌 경로계획 연동)
//
//   path_planner_node가 A*로 계산한 경로를 수신해서
//   waypoints_를 교체함. 기존 hardcoded 경로 대신 사용.
//
// [처리]
//   nav_msgs/Path (PoseStamped[]) → StateVec[] 변환
//   - x, y: 경로 점의 세계 좌표
//   - θ    : 0으로 초기화 → generateReference()에서 연속 yaw 재계산
//   - v    : v_ref_ (파라미터)
//   - ω    : 0 (MPC가 각속도 제어)
//
// [안전]
//   SingleThreadedExecutor이므로 controlCallback과 동시 실행 없음.
//   state_mutex_는 향후 MultiThread 전환 대비 이중 보호.
// ============================================================
void MpcNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "[MPC] 빈 /planned_path 수신 — 무시");
    return;
  }

  // PoseStamped → StateVec 변환
  std::vector<StateVec> new_waypoints;
  new_waypoints.reserve(msg->poses.size());
  for (const auto & pose_stamped : msg->poses) {
    StateVec wp;
    wp(0) = pose_stamped.pose.position.x;
    wp(1) = pose_stamped.pose.position.y;
    wp(2) = 0.0;    // θ: generateReference()에서 실시간 계산
    wp(3) = v_ref_;
    wp(4) = 0.0;
    new_waypoints.push_back(wp);
  }

  // ── 항상 idx=0부터 추종 시작 ─────────────────────────────────
  // [중요] 이전에 "현재 위치 기준 최근접 waypoint"를 찾아 시작했으나
  // 이 방식이 문제를 일으킴:
  //   A* 우회 경로에서 로봇 직선 방향에 있는 경로 중간점이 최근접으로
  //   탐색되어 우회 구간 전체를 건너뛰고 직진하게 됨.
  //
  // path_planner_node가 이미 Step 8(trim)에서 현재 위치에서
  // 0.15m 이내 점을 제거하므로 경로 첫점은 이미 현재 위치 근처.
  // → idx=0부터 순서대로 추종하면 우회 경로를 올바르게 따라감.

  // 더 구체적인 설명: 문제가 됐던 방식: 최근접 waypoint 탐색
  // 예전 코드는 새 경로가 들어오면 현재 로봇 위치에서 가장 가까운 waypoint를 찾아서 거기서부터 추종을 시작했습니다.
  // 이게 왜 문제냐면, A*가 장애물을 우회하는 경로를 짰을 때 아래처럼 됩니다.
  
  // 로봇 현재위치 ──── [wp0] ── [wp1] ── [wp2(우회 시작)]
  //                                           ↓
  //                                        [wp3]
  //                                           ↓
  //                                        [wp4(우회 끝)] ── [wp5] ── 목표
  // 로봇 기준으로 직선 거리만 따지면 우회 경로의 중간쯤인 wp4가 실제로 가장 가까울 수 있습니다. 그러면 코드가 wp4를 시작점으로 잡아버리고, 
  // wp2, wp3 우회 구간 전체를 건너뛰고 직진하게 됩니다. 결과적으로 장애물을 피하라고 A*가 짠 우회 경로가 완전히 무용지물이 됩니다.
  // 해결책: 항상 idx=0부터 시작
  // path_planner_node의 plan() Step 8(trim) 에서 이미 현재 위치에서 0.15m 이내의 앞 waypoint들을 제거하고 발행합니다.
  // 즉, /planned_path로 들어오는 경로의 첫 번째 점(idx=0)은 이미 현재 로봇 위치 근처입니다.
  // 따라서 굳이 "가장 가까운 점"을 탐색할 필요 없이, 무조건 idx=0부터 순서대로 따라가면 우회 경로도 순서대로 올바르게 추종하게 됩니다.
  
  // 발행된 경로:
  // [wp0(현재위치 근처)] → [wp1] → [wp2 우회] → [wp3 우회] → [wp4] → 목표
  //    ↑
  //   여기서 시작 (항상)

  {
    std::lock_guard<std::mutex> lock(state_mutex_); // MPC 제어 루프와 이 콜백이 동시에 waypoints_를 읽으면
    // 데이터 레이스 발생 -> mutex로 보호
    waypoints_      = new_waypoints; // 새 경로로 교체
    mission_done_   = false; // 새 경로 왔으니 미션 완료 상태 해제
    closest_wp_idx_ = 0;  // 항상 경로 첫점부터 추종 시작(핵심)
    // continuous_theta_ 리셋하지 않음 — 현재 heading 연속성 유지해야 MPC 각도 계산 시 ±π 경계에서 값이 튀는 현상을 방지
  }

  RCLCPP_INFO(this->get_logger(),
    "[MPC] 새 경로 수신: %zu waypoints | "
    "첫점 (%.2f, %.2f) → 끝점 (%.2f, %.2f)",
    new_waypoints.size(),
    new_waypoints.front()(0), new_waypoints.front()(1),
    new_waypoints.back()(0),  new_waypoints.back()(1));
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

// ============================================================
// controlCallback() — 50Hz 제어 루프
//
//   매 20ms마다 실행:
//   1. Data Copy-out: 뮤텍스 락을 최소한으로 유지하여 상태 복사
//   2. 상태 예측 지연 보상 (Delay Compensation)
//   3. reference trajectory 생성
//   4. MPC solve (무거운 연산 시 락 해제 상태 유지)
//   5. u0 → v, ω 변환 후 /cmd_vel 발행
//   6. latency 통계 산출 및 발행
// ============================================================

void MpcNode::controlCallback()
{
  // ----------------------------------------------------------------
  // [수정 위치 1] Data Copy-out 기법 적용
  // 원리: MultiThreadedExecutor 환경에서는 MPC solve(약 2~20ms 소요)가 도는 동안 
  // 락(lock)을 쥐고 있으면 odom, loc 등 다른 콜백이 실행되지 못하고 데드락에 빠집니다.
  // 따라서 아주 짧은 시간 동안만 락을 걸어 공유 자원(멤버 변수)을 로컬 변수로 복사한 뒤,
  // 락을 해제하고 복사된 로컬 변수로만 무거운 알고리즘 연산을 진행합니다.
  // ----------------------------------------------------------------
  StateVec local_x0;
  bool local_has_odom;
  uint8_t local_safety_state;
  bool local_mission_done;
  std::vector<Obstacle> local_obstacles;
  std::deque<geometry_msgs::msg::Twist> local_delay_queue;
  
  { // 변수 복사를 위한 스코프(Scope) 시작
    std::lock_guard<std::mutex> lock(state_mutex_);
    local_x0 = x0_;
    local_has_odom = has_odom_;
    local_safety_state = safety_state_;
    local_mission_done = mission_done_;
    local_obstacles = obstacles_;
    local_delay_queue = delay_queue_;
  } // 스코프 종료와 동시에 락이 해제되어 센서 콜백들이 정상 동작할 수 있습니다.

  // 멤버 변수 대신 복사해 온 로컬 변수로 상태를 체크합니다.
  if (!local_has_odom) {
    return;
  }

  // W12: 글로벌 경로계획 모드에서 경로 미수신 시 대기
  // SingleThread이므로 waypoints_를 락 없이 읽어도 안전
  if (use_global_planner_ && waypoints_.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[MPC] /planned_path 대기 중... (path_planner_node 실행 여부 확인)");
    publishStop();
    return;
  }
  if (local_safety_state == 2){
    publishStop();
    return;
  }
  // ── NAVIGATING → DOCKING 전환 ──────────────────────────────
  // A* 경로의 마지막 waypoint 도달(mission_done_) 시 정지 대신 DOCKING 진입
  // dock_x_, dock_y_를 마지막 waypoint 위치(= path_planner의 goal)로 확정
  if (mission_phase_ == MissionPhase::NAVIGATING && local_mission_done) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!waypoints_.empty()) {
        dock_x_ = waypoints_.back()(0);
        dock_y_ = waypoints_.back()(1);
      }
    }
    mission_phase_ = MissionPhase::DOCKING;
    RCLCPP_INFO(this->get_logger(),
      "[Docking] NAVIGATING → DOCKING | "
      "dock_pos=(%.3f, %.3f) dock_yaw=%.2f°",
      dock_x_, dock_y_, dock_yaw_ * 180.0 / M_PI);
  }

  // DOCKED: 완료 상태 유지 (정지 명령 반복 발행)
  if (mission_phase_ == MissionPhase::DOCKED) {
    publishStop();
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  // ----------------------------------------------------------------
  // [수정 위치 2] 변수명 치환 (x0_ -> local_x0, delay_queue_ -> local_delay_queue)
  // ----------------------------------------------------------------
  StateVec x_pred = local_x0;
  if (delay_ms_ > 0.0 && !local_delay_queue.empty()) {
    double dt_step = mpc_params_.dt;
    for (const auto & msg : local_delay_queue) {
      double v = msg.linear.x;
      double w = msg.angular.z;

      // Forward Kinematics (예측)
      x_pred(0) += v * std::cos(x_pred(2)) * dt_step;
      x_pred(1) += v * std::sin(x_pred(2)) * dt_step;
      x_pred(2) += w * dt_step;
      x_pred(3) = v;
      x_pred(4) = w;
    }
  }

  // --- 1. reference trajectory 생성 ---
  // DOCKING 단계는 이 아래 블록에서 P제어로 처리 후 return하므로
  // generateReference()는 NAVIGATING일 때만 실행됨
  std::vector<StateVec> x_ref = generateReference(x_pred);

  {
    geometry_msgs::msg::PoseStamped ref_msg;
    ref_msg.header.stamp    = this->now();
    ref_msg.header.frame_id = "map";
    ref_msg.pose.position.x = x_ref[0](0);
    ref_msg.pose.position.y = x_ref[0](1);
    ref_msg.pose.position.z = 0.0;
    double half_yaw = x_ref[0](2) * 0.5;
    ref_msg.pose.orientation.x = 0.0;
    ref_msg.pose.orientation.y = 0.0;
    ref_msg.pose.orientation.z = std::sin(half_yaw);
    ref_msg.pose.orientation.w = std::cos(half_yaw);
    ref_pose_pub_->publish(ref_msg);
  }

  // ----------------------------------------------------------------
  // [수정 위치 3] 장애물 변수 명시적 세팅
  // MPC 코어가 사용할 장애물 정보도 복사본(local_obstacles)을 넘겨 스레드 충돌을 방지합니다.
  // ----------------------------------------------------------------
  mpc_core_.setObstacles(local_obstacles);

  // ----------------------------------------------------------------
  // [수정 위치 4] solve 함수 u_prev_ 분리
  // u_prev_ 멤버 변수 역시 공유 자원이므로 잠시 락을 걸어 복사한 뒤 넘깁니다.
  // ----------------------------------------------------------------
  InputVec current_u_prev;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_u_prev = u_prev_;
  }
  
  // 무거운 행렬 연산 구간입니다. 이때 락이 풀려있으므로 데드락이 발생하지 않습니다.
  MpcSolution sol = mpc_core_.solve(x_pred, x_ref, current_u_prev);

  // 인위적 CPU 부하 주입 로직
  if (artificial_load_ms_ > 0.0) {
    const int iterations = static_cast<int>(artificial_load_ms_);
    Eigen::MatrixXd m = Eigen::MatrixXd::Random(50, 50);
    for (int i = 0; i < iterations; ++i) {
      m = m * m.transpose(); 
      m = m / m.norm();      
    }
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!sol.success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "MPC solve 실패 — 정지 명령");
    publishStop();
    return;
  }

  // ----------------------------------------------------------------
  // [수정 위치 5] 속도 계산 락 추가
  // 결과물인 v_cmd, w_cmd를 산출할 때, 공유 자원인 u_prev_를 사용하므로 락으로 보호합니다.
  // ----------------------------------------------------------------
  double v_cmd, w_cmd;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    v_cmd = u_prev_(0) + sol.u0(0);
    w_cmd = u_prev_(1) + sol.u0(1);
  }

  if (local_safety_state == 1){
    v_cmd *= 0.5;
    w_cmd *= 0.5;
  }

  v_cmd = std::clamp(v_cmd, mpc_params_.v_min, mpc_params_.v_max);
  
    if (!use_global_planner_) {
    v_cmd = std::max(v_cmd, 0.07);
  }

  if (!local_obstacles.empty()) {
    double min_dist = 1e9;
    for (const auto & obs : local_obstacles) {
      double d = std::hypot(local_x0(0) - obs.x, local_x0(1) - obs.y) - obs.radius;
      min_dist = std::min(min_dist, d);
    }

    constexpr double slow_start = 0.7;
    if (min_dist < slow_start && v_cmd > 0.0) {
      const double scale = std::clamp(min_dist / slow_start, 0.0, 1.0);
      v_cmd *= scale;

      // 장애물 매우 근접 시 전진 하한 제거함
      if (min_dist > 0.25) {
        v_cmd = std::max(v_cmd, 0.03);
      }
    }
  }

  // ── DOCKING 단계: MPC 우회 → P제어기로 직접 제어 ─────────────────
  //
  // [왜 MPC를 우회하는가?]
  //   MPC 비용함수에는 r_dv=30, r_dw=20 (입력 변화 페널티)이 설정돼 있음.
  //   v_ref=0 reference에서 0.1m 오차는 MPC가 "가만히 있는 게 최적"으로
  //   판단할 만큼 작은 값이라 사실상 zero 명령만 생성함.
  //   → 도킹처럼 오차가 수 cm인 정밀 구간에서는 단순 P제어가 더 효과적.
  //
  // [P제어 도킹 로직 — 2단계]
  //   Step 1 (위치 오차 > dock_pos_tol_):
  //     dock 목표 방향으로 회전하면서 전진 (pos 수렴)
  //     v = Kp_v * pos_err   (거리 비례 속도)
  //     ω = Kp_w * heading_err (dock 방향으로 헤딩 보정)
  //
  //   Step 2 (위치 수렴 후):
  //     dock_yaw_ 맞추기 (최종 헤딩 정렬)
  //     v = 0
  //     ω = Kp_w * yaw_err_to_dock_yaw
  // ──────────────────────────────────────────────────────────────────
  if (mission_phase_ == MissionPhase::DOCKING) {

    // 현재 위치 → dock 목표까지 벡터
    double dx_to_dock  = dock_x_ - local_x0(0);
    double dy_to_dock  = dock_y_ - local_x0(1);
    double pos_err     = std::hypot(dx_to_dock, dy_to_dock);

    // dock 방향 헤딩 오차: 현재 yaw와 "dock 목표 방향" 사이 최단 각도
    double angle_to_dock  = std::atan2(dy_to_dock, dx_to_dock);
    double heading_err    = std::atan2(
      std::sin(angle_to_dock - local_x0(2)),
      std::cos(angle_to_dock - local_x0(2)));

    // dock_yaw_ 최종 정렬 오차
    double raw_diff  = local_x0(2) - dock_yaw_;
    double yaw_err   = std::atan2(std::sin(raw_diff), std::cos(raw_diff));

    // P제어 게인 (tunable)
    // Kp_v: 거리 1m당 0.3m/s — DOCK_V_MAX(0.05)로 클램핑되므로 과도 응답 없음
    // Kp_w: 각도 1rad당 0.8rad/s — w_max(1.0)로 클램핑됨
    constexpr double Kp_v      = 0.3;
    constexpr double Kp_w      = 0.8;
    constexpr double DOCK_V_MAX = 0.05;  // 최대 도킹 속도 [m/s]

    double v_dock = 0.0;
    double w_dock = 0.0;

    if (pos_err > dock_pos_tol_) {
      // Step 1: dock 방향으로 이동하면서 헤딩 보정
      v_dock = std::clamp(Kp_v * pos_err, -DOCK_V_MAX, DOCK_V_MAX);
      w_dock = std::clamp(Kp_w * heading_err,
                          mpc_params_.w_min, mpc_params_.w_max);
    } else {
      // Step 2: 위치 도달 후 최종 헤딩 정렬
      v_dock = 0.0;
      w_dock = std::clamp(-Kp_w * yaw_err,
                          mpc_params_.w_min, mpc_params_.w_max);
    }

    // 도킹 완료 판정: 위치 AND 헤딩 동시 만족
    if (pos_err < dock_pos_tol_ && std::abs(yaw_err) < dock_yaw_tol_) {
      mission_phase_ = MissionPhase::DOCKED;
      RCLCPP_INFO(this->get_logger(),
        "[Docking] ✅ 도킹 완료! pos_err=%.3fm  yaw_err=%.2f°",
        pos_err, yaw_err * 180.0 / M_PI);
      publishStop();
      return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "[Docking] pos_err=%.3fm  yaw_err=%.2f°  heading_err=%.2f°  "
      "v=%.3f  w=%.3f  step=%s",
      pos_err, yaw_err * 180.0 / M_PI, heading_err * 180.0 / M_PI,
      v_dock, w_dock,
      (pos_err > dock_pos_tol_) ? "1(위치수렴)" : "2(헤딩정렬)");

    publishCmdVel(v_dock, w_dock);
    return;  // MPC 이후 로직 전부 건너뜀
  }

  w_cmd = std::clamp(w_cmd, mpc_params_.w_min, mpc_params_.w_max);

  // reactive yaw bias 추가함
  // 부호 튐 방지하려고 hold + smoothing 넣음
  if (false && !use_global_planner_ && !local_obstacles.empty()) {
    double best_forward = 1e9;
    double best_lateral = 0.0;
    double best_clear = 1e9;
    bool found_front_obs = false;

    for (const auto & obs : local_obstacles) {
      const double dx = obs.x - local_x0(0);
      const double dy = obs.y - local_x0(1);

      // body frame 변환함
      const double forward =
        std::cos(local_x0(2)) * dx + std::sin(local_x0(2)) * dy;
      const double lateral =
        -std::sin(local_x0(2)) * dx + std::cos(local_x0(2)) * dy;

      const double clear = std::hypot(dx, dy) - obs.radius;

      // 너무 넓게 보면 대표 장애물이 자주 바뀌어서 부호가 튐
      if (forward > 0.0 && forward < 0.55 && std::abs(lateral) < 0.32) {
        if (clear < best_clear) {
          best_forward = forward;
          best_lateral = lateral;
          best_clear = clear;
          found_front_obs = true;
        }
      }
    }

    // 상태 유지용 static
    static double bias_state = 0.0;
    static int turn_dir_hold = 0;
    static bool bias_active = false;

    const double trigger_clear = 0.12;
    const double release_clear = 0.21;
    const double lateral_deadband = 0.06;

    double gain = 0.0;
    double target_bias = 0.0;

    if (found_front_obs && best_clear < release_clear) {
      gain = (release_clear - best_clear) / (release_clear - trigger_clear);
      gain = std::clamp(gain, 0.0, 1.0);

      // lateral이 0 근처면 방향 바꾸지 않고 이전 방향 유지함
      if (!bias_active) {
        if (best_lateral > lateral_deadband) turn_dir_hold = -1;      // 장애물 좌측 → 우회전
        else if (best_lateral < -lateral_deadband) turn_dir_hold = 1; // 장애물 우측 → 좌회전
      } else {
        if (best_lateral > lateral_deadband && turn_dir_hold == 0) turn_dir_hold = -1;
        else if (best_lateral < -lateral_deadband && turn_dir_hold == 0) turn_dir_hold = 1;
      }

      if (turn_dir_hold == 0) {
        turn_dir_hold = (best_lateral >= 0.0) ? -1 : 1;
      }

      bias_active = true;

      // 0.5는 너무 큼, 먼저 0.28~0.32 범위로 제한함
      const double w_bias_max = 0.30;
      target_bias = static_cast<double>(turn_dir_hold) * w_bias_max * gain;
    } else {
      // 장애물 벗어나면 바로 0으로 끊지 말고 서서히 복귀시킴
      bias_active = false;
      turn_dir_hold = 0;
      target_bias = 0.0;
    }

    // bias를 저역통과 필터처럼 부드럽게 만듦
    const double alpha_rise = 0.18;
    const double alpha_fall = 0.10;
    const double alpha =
      (std::abs(target_bias) > std::abs(bias_state)) ? alpha_rise : alpha_fall;

    bias_state = (1.0 - alpha) * bias_state + alpha * target_bias;

    w_cmd += bias_state;
    w_cmd = std::clamp(w_cmd, mpc_params_.w_min, mpc_params_.w_max);

    // 속도는 지금 정도면 괜찮으니 과감속만 조금 완화함
    if (v_cmd > 0.0) {
      double v_scale = 1.0 - 0.12 * gain;
      v_cmd *= v_scale;
      v_cmd = std::max(v_cmd, 0.08);
    }
  }

  // ── CBF Safety Filter 적용 ────────────────────────────────
  // MPC nominal 입력을 CBF-QP에 통과시켜 최종 안전 입력으로 바꿈
  double v_pub = v_cmd;
  double w_pub = w_cmd;

  if (cbf_enabled_ && cbf_filter_ && !local_obstacles.empty()) {
    std::vector<CbfObstacle> cbf_obstacles;
    cbf_obstacles.reserve(local_obstacles.size());

    for (const auto & obs : local_obstacles) {
      CbfObstacle cbf_obs;
      cbf_obs.x = obs.x;
      cbf_obs.y = obs.y;
      cbf_obs.radius = obs.radius;  // obsCallback에서 이미 로봇 반경 포함됨
      cbf_obstacles.push_back(cbf_obs);
    }

    double v_safe = v_cmd;
    double w_safe = w_cmd;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "[CBF call] obs=%zu u_nom=(%.3f, %.3f) pose=(%.3f, %.3f, %.3f)",
      cbf_obstacles.size(),
      v_cmd,
      w_cmd,
      local_x0(0),
      local_x0(1),
      local_x0(2)
    );

    const bool cbf_ok = cbf_filter_->filter(
      local_x0(0),
      local_x0(1),
      local_x0(2),
      v_cmd,
      w_cmd,
      cbf_obstacles,
      v_safe,
      w_safe
    );

    if (cbf_ok) {
      v_pub = std::clamp(v_safe, mpc_params_.v_min, mpc_params_.v_max);
      w_pub = std::clamp(w_safe, mpc_params_.w_min, mpc_params_.w_max);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "[CBF] filter 실패. nominal 명령 사용함"
      );
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "[CBF result] ok=%s u_nom=(%.3f, %.3f) u_safe=(%.3f, %.3f)",
      cbf_ok ? "true" : "false",
      v_cmd,
      w_cmd,
      v_pub,
      w_pub
    );
  }




// CBF 출력 튐 완화용 rate limiter임
static bool cmd_filter_initialized = false;
static double prev_v_pub = 0.0;
static double prev_w_pub = 0.0;

auto rateLimit = [](double target, double prev, double max_step) {
  return prev + std::clamp(target - prev, -max_step, max_step);
};

if (!cmd_filter_initialized) {
  prev_v_pub = v_pub;
  prev_w_pub = w_pub;
  cmd_filter_initialized = true;
} else {
  // 감속은 빠르게 허용하고, 가속은 천천히 허용함
  const double dv_up_step = 0.010;
  const double dv_down_step = 0.040;
  const double dv_step = (v_pub < prev_v_pub) ? dv_down_step : dv_up_step;

  // 각속도는 한 주기당 변화량 제한함
  const double dw_step = 0.080;

  v_pub = rateLimit(v_pub, prev_v_pub, dv_step);
  w_pub = rateLimit(w_pub, prev_w_pub, dw_step);

  prev_v_pub = v_pub;
  prev_w_pub = w_pub;
}






  // 내부에서 delay_queue_ 조작 시 락이 걸리도록 이미 수정된 함수입니다.
  publishCmdVel(v_pub, w_pub);

  // ----------------------------------------------------------------
  // [수정 위치 6] 상태 저장 및 통계 갱신 락 추가
  // 외부로 발행(Publish)할 변수들은 스코프 밖에서 미리 선언해두어야 합니다.
  // ----------------------------------------------------------------
  double e2e_ms = 0.0, avg_e2e = 0.0, p99_e2e = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    auto now_stamp = this->get_clock()->now();
    e2e_ms = (now_stamp - last_odom_stamp_).seconds() * 1000.0;
    
    e2e_history_.push_back(e2e_ms);
    if (static_cast<int>(e2e_history_.size()) > 500) {
      e2e_history_.erase(e2e_history_.begin());
    }

    // e2e 평균 계산
    for (double v : e2e_history_) avg_e2e += v;
    avg_e2e /= e2e_history_.size();

    // e2e 99th percentile 계산
    if (!e2e_history_.empty()) {
      std::vector<double> sorted = e2e_history_;
      std::sort(sorted.begin(), sorted.end());
      int idx = static_cast<int>(sorted.size() * 0.99);
      p99_e2e = sorted[std::min(idx, static_cast<int>(sorted.size()) - 1)];
    }

    // 이전 입력 갱신 (다음 스텝의 계산을 위해 저장)
    // u_prev_(0) = v_cmd;
    // u_prev_(1) = w_cmd;
    u_prev_(0) = v_pub;
    u_prev_(1) = w_pub;
  }

  // --- 5. latency 통계 산출 및 발행 ---
  // 아래 변수들은 멤버 변수가 아닌 로컬 변수이므로 락이 불필요합니다.
  latency_history_.push_back(elapsed_ms);
  if (static_cast<int>(latency_history_.size()) > 500) {
    latency_history_.erase(latency_history_.begin());
  }

  double avg_ms = 0.0;
  for (double v : latency_history_) avg_ms += v;
  avg_ms /= latency_history_.size();

  double max_ms = *std::max_element(latency_history_.begin(), latency_history_.end());

  amr_msgs::msg::ControlLatency lat_msg;
  lat_msg.latency_ms     = elapsed_ms;
  lat_msg.avg_latency_ms = avg_ms;
  lat_msg.max_latency_ms = max_ms;
  lat_msg.e2e_latency_ms = e2e_ms;
  lat_msg.avg_e2e_ms     = avg_e2e;
  lat_msg.p99_e2e_ms     = p99_e2e;
  latency_pub_->publish(lat_msg);

  // W8: 현재 위치에서 각 장애물까지 최소 거리 계산 및 발행 (로컬 변수 사용)
  {
    double min_dist = 1e9;
    for (const auto & obs : local_obstacles) {
      double dx = local_x0(0) - obs.x;
      double dy = local_x0(1) - obs.y;
      double clearance = std::sqrt(dx * dx + dy * dy) - obs.radius;
      min_dist = std::min(min_dist, clearance);
    }
    amr_msgs::msg::MinObstacleDistance dist_msg;
    dist_msg.min_distance_m = min_dist;
    min_dist_pub_->publish(dist_msg);

    if (min_dist < -0.2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "장애물 근접! min_clearance=%.3fm", min_dist);
    }
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    "[MPC] solve=%.2fms avg=%.2fms | e2e=%.2fms avg_e2e=%.2fms p99_e2e=%.2fms | "
    "u_nom=(%.3f, %.3f) u_pub=(%.3f, %.3f)",
    elapsed_ms, avg_ms, e2e_ms, avg_e2e, p99_e2e,
    v_cmd, w_cmd, v_pub, w_pub);
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
  // double v_ref_ = 0.15;  // 원형 경로: 선형화 오차 줄이기 위해 속도 절반
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
      wp(3) = v_ref_;     // v
      wp(4) = 0.0;       // ω
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 직선 경로 %zu pts", waypoints_.size());

  } else if (trajectory_type_ == "circle") {
    // 원: 반지름 1.5m, 반시계 방향
    double r      = 0.5;
    double period = 2.0 * M_PI * r / v_ref_;    // 한 바퀴 걸리는 시간 [s]
    // MPC 예측 주기와 동일하게 맞춤 (0.02s)
    double dt_wp  = mpc_params_.dt;                         // waypoint 간격 [s]
    int    n_pts  = static_cast<int>(period / dt_wp);
    double w_ref  = v_ref_ / r;                   // ω = v/r

    for (int i = 0; i <= n_pts; ++i) {
      double t   = i * dt_wp;
      double ang = w_ref * t;   // 각도 진행
      StateVec wp;
      wp(0) = r * std::cos(ang) - r;   // 원점에서 출발하도록 오프셋
      wp(1) = r * std::sin(ang);
      wp(2) = ang + M_PI / 2.0;        // 접선 방향 (θ = 각도 + 90°)
      wp(3) = v_ref_;
      wp(4) = w_ref;
      waypoints_.push_back(wp);
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints: 원형 경로 %zu pts (r=%.1f)", waypoints_.size(), r);

  } else if (trajectory_type_ == "figure8") {
    // 8자: lemniscate 근사 (두 원을 이어붙인 형태)
    double r      = 0.7;
    double dt_wp  = mpc_params_.dt;  // 예측 주기와 동일하게 맞춤 (0.02)
    double w_ref  = v_ref_ / r;
    int    n_half = static_cast<int>(M_PI * r / v_ref_ / dt_wp);

    // 첫 번째 원 (반시계, 왼쪽 원)
    for (int i = 0; i <= n_half * 2; ++i) {
      double ang = w_ref * i * dt_wp;
      StateVec wp;
      wp(0) =  r * std::cos(ang) - r;
      wp(1) =  r * std::sin(ang);
      // wp(2)는 generateReference에서 실시간 계산하므로 생략
      wp(3) = v_ref_;
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
      wp(3) = v_ref_;
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

  // ----------------------------------------------------------------
  // [수정] 전체 탐색으로 변경
  //
  // 기존: closest_wp_idx_ + 20개만 탐색
  //   → 직선 경로에서는 충분하지만, A* 우회 경로처럼 꺾인 경로에서는
  //     실제 가장 가까운 waypoint가 탐색 범위 밖에 있어 잘못된 idx를 잡음
  //     → MPC가 엉뚱한 방향을 reference로 잡아 재계획 트리거 연속 발생
  //
  // 수정: closest_wp_idx_부터 전체 탐색
  //   → 우회 경로에서도 올바른 waypoint를 찾아 안정적으로 추종
  //   → 단, closest_wp_idx_를 하한으로 유지해 뒤로 가지 않음
  // ----------------------------------------------------------------
  // ----------------------------------------------------------------
  // [탐색 범위 제한]
  // 전체 탐색(i < n)을 하면 우회 경로에서 경로 후반부(목표 근처)가
  // 최근접으로 탐색되어 우회 구간 전체를 건너뛰는 문제 발생.
  // → 현재 idx에서 앞으로 50개만 탐색하여 경로 순서를 보장.
  //   (wp_spacing=0.05m 기준 50개 = 2.5m 커버)
  // ----------------------------------------------------------------
  int search_end = std::min(closest_wp_idx_ + 50, n);
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
      // RCLCPP_INFO(this->get_logger(), "미션 완료! 목표 지점 도달 (현재 idx: %d / 총 %d)", best_idx, n-1);
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
  closest_wp_idx_ = findClosestWaypoint(x0);

  int n = static_cast<int>(waypoints_.size());
  std::vector<StateVec> x_ref(mpc_params_.N + 1);

  // ----------------------------------------------------------------
  // [핵심 수정] 거리 기반 waypoint 인덱싱
  //
  // 기존: idx = closest_wp_idx_ + k
  //   k=20, wp_spacing=0.05m → 1.0m 앞 waypoint를 reference로 잡음
  //   v=0.1m/s, dt=0.02s → 실제 이동 가능 = 0.04m
  //   → 25배 차이 → MPC가 엉뚱한 방향을 가리킴
  //
  // 수정: k스텝에서 로봇이 실제로 이동할 거리(v*dt*k)만큼
  //       경로 위를 따라간 위치를 reference로 사용
  //   → 경로 방향을 정확하게 따라감
  // ----------------------------------------------------------------

  // k스텝에서 예상 이동 거리: v_ref * dt * k
  // 이 거리만큼 경로를 따라간 waypoint 인덱스를 구함
  double step_dist = std::max(v_ref_ * mpc_params_.dt, 0.025);  // 한 스텝당 이동 거리 [m]

  for (int k = 0; k <= mpc_params_.N; ++k) {
    // k스텝 후 경로 위 목표 거리
    double target_dist = step_dist * k;

    // 현재 waypoint에서 target_dist만큼 앞의 waypoint 인덱스 탐색
    int idx = closest_wp_idx_;
    double accumulated = 0.0;

    while (idx < n - 1) {
      double dx = waypoints_[idx + 1](0) - waypoints_[idx](0);
      double dy = waypoints_[idx + 1](1) - waypoints_[idx](1);
      double seg = std::sqrt(dx * dx + dy * dy);
      if (accumulated + seg >= target_dist) break;
      accumulated += seg;
      idx++;
    }
    idx = std::min(idx, n - 1);
    int idx_next = std::min(idx + 1, n - 1);

    x_ref[k] = waypoints_[idx];

    // 실시간 Heading 계산 + 연속성 유지
    double dx = waypoints_[idx_next](0) - waypoints_[idx](0);
    double dy = waypoints_[idx_next](1) - waypoints_[idx](1);

    double ref_yaw_base = (k == 0) ? x0(2) : x_ref[k - 1](2);
    double raw_yaw;

    if (std::abs(dx) < 1e-5 && std::abs(dy) < 1e-5) {
      raw_yaw = ref_yaw_base;
    } else {
      raw_yaw = std::atan2(dy, dx);
    }

    double diff = std::atan2(
      std::sin(raw_yaw - ref_yaw_base),
      std::cos(raw_yaw - ref_yaw_base));
    x_ref[k](2) = ref_yaw_base + diff;
  }

  return x_ref;
}

// ============================================================
// generateDockingReference() — 도킹 전용 reference 생성
//
// [역할]
//   DOCKING 단계에서 MPC가 dock 포즈(dock_x_, dock_y_, dock_yaw_)로
//   정밀하게 수렴하도록 N+1개의 동일한 reference를 생성함.
//
// [generateReference()와의 차이]
//   - waypoint 리스트를 따라가지 않고 dock 포즈 1점만 반복
//   - v_ref = 0, ω_ref = 0: "정지 상태"를 목표로 설정
//     → MPC가 dock 포즈에 수렴하면서 자연스럽게 감속
//   - 헤딩은 continuous_theta_ 공간에서 최단 방향으로 보정
//     → ±π 경계에서 값이 튀는 현상 방지
//
// [속도 제한]
//   v_cmd는 controlCallback에서 DOCK_V_MAX(0.05m/s)로 클램핑됨.
//   이 함수 자체는 참조값만 생성하고 속도 제한은 하지 않음.
// ============================================================
std::vector<StateVec> MpcNode::generateDockingReference(const StateVec & x0)
{
  std::vector<StateVec> x_ref(mpc_params_.N + 1);

  // 도킹 목표 헤딩을 현재 continuous_theta_ 공간으로 변환
  // raw_diff: dock_yaw_와 현재 yaw의 최단 각도 차이 ([-π, π])
  // dock_yaw_cont: continuous 공간에서의 dock_yaw 표현
  //   → x0(2)가 2π를 넘어가도 MPC 내부에서 yaw 오차가 튀지 않음
  double raw_diff      = std::atan2(
    std::sin(dock_yaw_ - x0(2)),
    std::cos(dock_yaw_ - x0(2)));
  double dock_yaw_cont = x0(2) + raw_diff;

  // dock 목표 상태 벡터 구성
  StateVec dock_state;
  dock_state(0) = dock_x_;        // 목표 x [m]
  dock_state(1) = dock_y_;        // 목표 y [m]
  dock_state(2) = dock_yaw_cont;  // 목표 헤딩 [rad, continuous]
  dock_state(3) = 0.0;            // v_ref = 0 (정지 목표)
  dock_state(4) = 0.0;            // ω_ref = 0

  // N+1개 전부 동일한 dock_state로 채움
  // MPC는 이 고정 목표를 향해 최적 입력(v, ω)을 계산
  for (int k = 0; k <= mpc_params_.N; ++k) {
    x_ref[k] = dock_state;
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
    // [원리 설명]
    // 여기서부터 큐에 데이터를 넣고 빼는 작업이 시작됩니다.
    // 락(Lock)을 걸어 이 작업이 끝날 때까지 publishStop() 등 다른 함수가 
    // delay_queue_에 접근하지 못하도록 보호합니다.
    {
      std::lock_guard<std::mutex> lock(state_mutex_); 
      delay_queue_.push_back(msg);

      int delay_steps = static_cast<int>(delay_ms_ / (mpc_params_.dt * 1000.0));

      if (static_cast<int>(delay_queue_.size()) > delay_steps) {
        geometry_msgs::msg::Twist pop_msg = delay_queue_.front();
        delay_queue_.pop_front();
        
        geometry_msgs::msg::Twist slip_msg;
        slip_msg.linear.x  = pop_msg.linear.x * (1.0 - slip_ratio_);
        slip_msg.angular.z = pop_msg.angular.z * (1.0 - slip_ratio_);
        cmd_vel_pub_->publish(slip_msg);
      } else {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
      }
    } // 중괄호가 끝나는 이 시점에서 lock_guard의 수명이 다하며 자동으로 락이 해제됩니다.
  } else {
    // 지연이 없을 때는 큐를 사용하지 않으므로 락이 필요 없습니다.
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
  // 정지 토픽 발행 자체는 ROS2 미들웨어가 알아서 처리하므로 락이 필요 없습니다.
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);

  // /cmd_vel_delayed 직접 발행 (Gazebo 브릿지 직접 연결)
  // → mock_link delay_queue를 우회해서 Gazebo에 즉시 zero 전달
  // → SAFE_STOP, localization LOST 등 긴급 상황에서 즉시 정지 보장
  emergency_stop_pub_->publish(msg);


  // [원리 설명]
  // 이전 입력 초기화 및 남아있는 지연 명령 싹 비우기
  // 이 순간 controlCallback에서 x_pred를 예측하기 위해 delay_queue_를 복사하려 하거나, 
  // u_prev_를 참조하려 할 수 있습니다. 이를 방지하기 위해 락을 획득합니다.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    u_prev_.setZero();
    delay_queue_.clear();
  } // 초기화가 안전하게 완료되면 락이 해제됩니다.
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