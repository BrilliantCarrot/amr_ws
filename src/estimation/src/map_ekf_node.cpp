#include "estimation/map_ekf_node.hpp"
#include <csignal>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace estimation
{

// ============================================================
// 생성자
//   노드 이름: "map_ekf_node"
//   subscriber, publisher, TF 리스너, 파라미터, EKF 노이즈 설정
// ============================================================
MapEkfNode::MapEkfNode(const rclcpp::NodeOptions & options)
: Node("map_ekf_node", options),
  first_imu_(true),
  is_ekf_initialized_(false),
  cached_loc_status_(STATUS_NORMAL)  // 초기값: NORMAL (안전한 방향)
  {
  // --------------------------------------------------------
  // ROS2 파라미터 선언 및 로드
  //
  //   r_lidar_normal_xy_    : NORMAL 상태 위치 노이즈 (기본 0.05m²)
  //   r_lidar_normal_yaw_   : NORMAL 상태 yaw 노이즈 (기본 0.05rad²)
  //   r_lidar_degraded_xy_  : DEGRADED 상태 위치 노이즈 (기본 0.5m²)
  //   r_lidar_degraded_yaw_ : DEGRADED 상태 yaw 노이즈 (기본 0.5rad²)
  //
  //   [튜닝 기준]
  //     NORMAL 값이 작을수록 LiDAR를 강하게 신뢰 → 드리프트 보정 강함
  //     DEGRADED 값은 NORMAL의 약 10배가 경험적으로 적절함
  //     너무 작으면 LiDAR 노이즈에 EKF가 흔들리고,
  //     너무 크면 드리프트 보정 효과가 없어짐
  // --------------------------------------------------------
  this->declare_parameter<double>("r_lidar_normal_xy",    0.05);
  this->declare_parameter<double>("r_lidar_normal_yaw",   0.05);
  this->declare_parameter<double>("r_lidar_degraded_xy",  0.5);
  this->declare_parameter<double>("r_lidar_degraded_yaw", 0.5);
  this->declare_parameter<double>("initial_x",   0.0);
  this->declare_parameter<double>("initial_y",   0.0);
  this->declare_parameter<double>("initial_yaw", 0.0);

  r_lidar_normal_xy_    = this->get_parameter("r_lidar_normal_xy").as_double();
  r_lidar_normal_yaw_   = this->get_parameter("r_lidar_normal_yaw").as_double();
  r_lidar_degraded_xy_  = this->get_parameter("r_lidar_degraded_xy").as_double();
  r_lidar_degraded_yaw_ = this->get_parameter("r_lidar_degraded_yaw").as_double();

  // --------------------------------------------------------
  // QoS 설정
  //   IMU  → BEST_EFFORT (고속, 손실 허용)
  //   Odom → RELIABLE (보정 소스, 손실 불허)
  //   localization_status → RELIABLE (상태 판단에 사용)
  // --------------------------------------------------------
  auto sensor_qos   = rclcpp::SensorDataQoS();
  auto reliable_qos = rclcpp::QoS(10).reliable();

    // 콜백 그룹 생성
  sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sensor_opt = rclcpp::SubscriptionOptions();
  sensor_opt.callback_group = sensor_cb_group_;

  // --------------------------------------------------------
  // Subscriber 생성
  // --------------------------------------------------------
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu",
    sensor_qos,
    std::bind(&MapEkfNode::imuCallback, this, std::placeholders::_1), sensor_opt
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    reliable_qos,
    std::bind(&MapEkfNode::odomCallback, this, std::placeholders::_1), sensor_opt
  );

  loc_status_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status",
    reliable_qos,
    std::bind(&MapEkfNode::localizationStatusCallback, this, std::placeholders::_1), sensor_opt
  );

  // --------------------------------------------------------
  // Publisher 생성
  //   /map_ekf/odom: map 프레임 기준 fused pose
  //   frame_id = "map" (ekf_node의 "odom"과 구분)
  // --------------------------------------------------------
  map_ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/map_ekf/odom", reliable_qos
  );

  // W10 Step2: EKF innovation norm 발행 (watchdog_node 구독용)
  // odom update 직후 innovation_norm을 50Hz로 발행
  innovation_norm_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/map_ekf/innovation_norm", reliable_qos
  );


  // --------------------------------------------------------
  // TF 리스너 초기화
  //   tf_buffer_: 과거 TF를 일정 시간 보관하는 창고
  //   tf_listener_: /tf 토픽 구독 → buffer에 자동으로 채워줌
  //
  //   [map→base_link TF 조회 방법]
  //     slam_toolbox가 map→odom을 발행하고
  //     ekf_node가 odom→base_link를 발행하면
  //     TF 트리에서 map→base_link가 자동으로 연결됨.
  //     tf_buffer_->lookupTransform("map", "base_link", ...) 으로 조회 가능.
  // --------------------------------------------------------
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --------------------------------------------------------
  // LiDAR 업데이트 타이머 (10Hz)
  //   slam_toolbox TF 발행 주기(~10Hz)에 맞춰 설정.
  //   타이머 주기가 TF 발행 주기보다 빨라도 괜찮음
  //   (TF 조회 실패 시 그냥 스킵하면 되니까).
  // --------------------------------------------------------
  lidar_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10Hz
    std::bind(&MapEkfNode::lidarUpdateCallback, this), sensor_cb_group_
  );

    // 50Hz (20ms) EKF 상태 발행 타이머 추가
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&MapEkfNode::publishTimerCallback, this), pub_timer_cb_group_
  );

  // --------------------------------------------------------
  // EKF 노이즈 파라미터 설정
  //   Q (프로세스 노이즈): ekf_node와 동일한 값으로 시작
  //   R (Odom 노이즈): ekf_node와 동일한 값으로 시작
  //   → 비교 실험 시 두 노드의 Q, R을 동일하게 유지해야
  //     LiDAR fusion의 순수한 효과를 비교할 수 있음
  // --------------------------------------------------------
  StateMat Q;
  Q.setZero();
  Q(0, 0) = 0.01;   // x 위치 노이즈 [m²]
  Q(1, 1) = 0.01;   // y 위치 노이즈 [m²]
  Q(2, 2) = 0.001;  // θ 노이즈 [rad²]
  Q(3, 3) = 0.1;    // vx 노이즈 [m²/s²]
  Q(4, 4) = 0.1;    // vy 노이즈 [m²/s²]
  Q(5, 5) = 0.01;   // ω 노이즈 [rad²/s²]
  ekf_.setProcessNoise(Q);

  Eigen::Matrix<double, OBS_DIM, OBS_DIM> R;
  R.setZero();
  R(0, 0) = 0.1;   // vx 관측 노이즈
  R(1, 1) = 0.1;   // vy 관측 노이즈
  R(2, 2) = 0.05;  // ω 관측 노이즈
  ekf_.setMeasurementNoise(R);

  RCLCPP_INFO(this->get_logger(), "[map_ekf_node] 초기화 완료. map frame 기반 LiDAR fusion EKF 대기 중...");
}

// ============================================================
// IMU 콜백 (200Hz) — predict()
// ============================================================
void MapEkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!is_ekf_initialized_) {
    // EKF 미초기화 → 초기화 시도 (odom + TF 준비 확인)
    initializeEkf();
    return;
  }

  const rclcpp::Time current_time = msg->header.stamp;

  // 첫 IMU 메시지는 dt 계산 기준점만 설정
  // dt`는 이전 IMU 메시지와 현재 IMU 메시지 사이의 시간 간격.
  // 그런데 맨 처음 IMU 메시지가 왔을 때를 생각하면, `last_imu_time_`이 아직 초기화가 안 된 상태. 
  // 이 상태에서 `current_time - last_imu_time_`을 계산하면 의미 없는 엄청 큰 값이 나옴.
  // 예를 들어 시뮬레이션이 t=100초에 시작했는데 `last_imu_time_`이 0으로 초기화되어 있으면:
  // dt = 100초 - 0초 = 100초
  // dt = 100초짜리 predict()가 실행되면 운동 방정식에서 위치가 엄청나게 튀어버림. 
  // 즉 EKF가 처음부터 발산.
  // 그래서 첫 번째 IMU 메시지는 `dt` 계산에 사용하지 않고, "앞으로 dt를 계산할 기준 시간점"으로만 저장하고 return
  // 흐름으로 보면:
  // 첫 번째 IMU 메시지 도착
  //     → last_imu_time_ = 지금 시간  (기준점 저장)
  //     → first_imu_ = false          (다음부터는 이 블록 안 들어옴)
  //     → return                       (predict 실행 안 함)
  // 두 번째 IMU 메시지 도착
  //     → dt = 지금 시간 - last_imu_time_  (정상적인 5ms)
  //     → last_imu_time_ = 지금 시간       (다음을 위해 갱신)
  //     → predict(dt) 실행                 (정상 동작)

// 세 번째, 네 번째 ... 계속 정상 동작
  if (first_imu_) {
    last_imu_time_ = current_time;
    first_imu_ = false;
    return;
  }

  // dt 계산
  const double dt = (current_time - last_imu_time_).seconds();
  last_imu_time_ = current_time;

  // dt 유효성 검사
  // [Gazebo Fortress 주의]
  //   W2에서 경험한 duplicate timestamp 문제: dt=0이 올 수 있음.
  //   dt <= 0 이면 ekf_core.cpp의 predict()에서 이미 걸러지지만
  //   여기서도 명시적으로 체크해서 불필요한 호출 방지.
  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  // body frame 선가속도, 각속도 추출
  // [좌표계 주의]
  //   Gazebo에서 오는 IMU는 body frame 기준임.
  //   motionModel()도 body frame 입력을 가정하므로 그대로 전달.
  const double ax        = msg->linear_acceleration.x;
  const double ay        = msg->linear_acceleration.y;
  const double omega_imu = msg->angular_velocity.z;

  std::lock_guard<std::mutex> lock(ekf_mutex_); // EKF 연산 직전 락
  ekf_.predict(ax, ay, omega_imu, dt);
}

// ============================================================
// Odom 콜백 (50Hz) — update() + publish
// ============================================================
void MapEkfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!is_ekf_initialized_) {
    // EKF 미초기화 → 초기화 시도
    initializeEkf();
    return;
  }

  // Odom에서 body frame 속도 추출
  const double vx_odom    = msg->twist.twist.linear.x;
  const double vy_odom    = msg->twist.twist.linear.y;
  const double omega_odom = msg->twist.twist.angular.z;

  {
    std::lock_guard<std::mutex> lock(ekf_mutex_); // EKF 연산 부분만 락
    // Odom으로 속도 보정
    ekf_.update(vx_odom, vy_odom, omega_odom);
  }
  // 보정된 상태 발행
  // publishMapEkfOdom(msg->header.stamp);
}

// ============================================================
// Localization 상태 콜백 (10Hz)
//   상태를 캐시해두고 lidarUpdateCallback에서 R_lidar 결정에 사용.
// ============================================================
void MapEkfNode::localizationStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  cached_loc_status_ = msg->status;
}

// ============================================================
// LiDAR 업데이트 타이머 콜백 (10Hz)
//   TF 조회 → 상태 판단 → R_lidar 결정 → updateLidar() 호출
// ============================================================
void MapEkfNode::lidarUpdateCallback()
{
  if (!is_ekf_initialized_) {
    return;
  }

  // LOST 상태면 LiDAR 관측 자체를 신뢰할 수 없으므로 스킵
  // → IMU + Odom만으로 dead reckoning 유지
  if (cached_loc_status_ == STATUS_LOST) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[map_ekf_node] localization LOST — LiDAR update 스킵 중");
    return;
  }

  // map→base_link TF 조회
  double map_x, map_y, map_yaw;
  if (!getMapPose(map_x, map_y, map_yaw)) {
    // TF 조회 실패: 아직 TF 트리가 안 쌓였거나 slam_toolbox 지연
    // 경고 로그만 남기고 스킵 (EKF 상태는 그대로 유지)
    return;
  }

  // --------------------------------------------------------
  // R_lidar 구성 — localization 상태에 따라 가변
  //
  //   [왜 대각 행렬인가?]
  //     x, y, θ 오차가 서로 독립적이라고 가정.
  //     실제론 상관관계가 있을 수 있지만
  //     실용적인 EKF 구현에선 대각 근사를 많이 씀.
  //
  //   [NORMAL vs DEGRADED 수치 비교]
  //     NORMAL   xy: 0.05m²  → 표준편차 ≈ 0.22m  (LiDAR 신뢰)
  //     DEGRADED xy: 0.5m²   → 표준편차 ≈ 0.71m  (LiDAR 불신)
  //     약 10배 차이 → DEGRADED일 때 칼만 게인 K가 대폭 줄어들어
  //     LiDAR가 EKF에 주는 영향력이 작아짐

  // R_lidar 작을 때 (NORMAL 상태)
  // S = H·P·Hᵀ + (작은 R)  →  S 작아짐
  // K = P·Hᵀ·S⁻¹           →  S 작으니까 K 커짐
  // x = x_pred + K·y        →  K 크니까 LiDAR 방향으로 크게 보정
  // "LiDAR 믿을 만하니까 LiDAR가 알려준 위치로 강하게 끌어당긴다"
  // R_lidar 클 때 (DEGRADED 상태)
  // S = H·P·Hᵀ + (큰 R)  →  S 커짐
  // K = P·Hᵀ·S⁻¹          →  S 크니까 K 작아짐
  // x = x_pred + K·y       →  K 작으니까 LiDAR 방향으로 조금만 보정
  // "LiDAR가 좀 흔들리는 것 같으니까 절반만 믿고 살짝만 보정한다"
  // --------------------------------------------------------
  LidarNoiseMat R_lidar;
  R_lidar.setZero();

  if (cached_loc_status_ == STATUS_NORMAL) {
    R_lidar(0, 0) = r_lidar_normal_xy_;
    R_lidar(1, 1) = r_lidar_normal_xy_;
    R_lidar(2, 2) = r_lidar_normal_yaw_;
  } else {
    // DEGRADED
    R_lidar(0, 0) = r_lidar_degraded_xy_;
    R_lidar(1, 1) = r_lidar_degraded_xy_;
    R_lidar(2, 2) = r_lidar_degraded_yaw_;
  }

  std::lock_guard<std::mutex> lock(ekf_mutex_); // EKF 연산 직전 락
  // LiDAR 관측으로 EKF 보정
  ekf_.updateLidar(map_x, map_y, map_yaw, R_lidar);
}

// ============================================================
// EKF 결과 발행
//   map 프레임 기준 fused pose를 /map_ekf/odom으로 발행.
// ============================================================
void MapEkfNode::publishTimerCallback()
{
  if (!is_ekf_initialized_) return;

  // 1. 상태 빠르게 복사 후 락 해제
  StateVec local_x;
  double local_innov_norm;
  {
    std::lock_guard<std::mutex> lock(ekf_mutex_);
    local_x = ekf_.getState();
    local_innov_norm = ekf_.getInnovationNorm();
  } // 락 해제 완료 (센서 콜백 블로킹 없음)

  // 2. 복사된 상태로 발행 로직 수행
  nav_msgs::msg::Odometry msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = "map";
  msg.child_frame_id  = "base_link";

  msg.pose.pose.position.x = local_x(0);
  msg.pose.pose.position.y = local_x(1);
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, local_x(2));
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x  = local_x(3);
  msg.twist.twist.linear.y  = local_x(4);
  msg.twist.twist.angular.z = local_x(5);

  map_ekf_pub_->publish(msg);

  std_msgs::msg::Float64 innov_msg;
  innov_msg.data = local_innov_norm;
  innovation_norm_pub_->publish(innov_msg);
}

// ============================================================
// EKF 초기화
//   조건: map→base_link TF 조회 성공
//   초기 상태: map frame 기준 위치 + 속도 0
// ============================================================
void MapEkfNode::initializeEkf()
{
  // 변경 후
  double init_x, init_y, init_yaw;
  if (!getMapPose(init_x, init_y, init_yaw)) {
    // TF 조회 실패 시 파라미터로 설정된 초기 위치 사용
    init_x   = this->get_parameter("initial_x").as_double();
    init_y   = this->get_parameter("initial_y").as_double();
    init_yaw = this->get_parameter("initial_yaw").as_double();
    RCLCPP_WARN(this->get_logger(),
      "[map_ekf_node] TF 조회 실패 — 파라미터 초기값 사용: x=%.3f, y=%.3f",
      init_x, init_y);
  }

  // 초기 상태 벡터: map frame 기준 위치 + 속도 0
  StateVec x0;
  x0 << init_x, init_y, init_yaw, 0.0, 0.0, 0.0;

  // 초기 공분산
  // 위치는 TF로 알았으니 작게, 속도는 모르니 크게
  StateMat P0;
  P0.setZero();
  P0(0, 0) = 0.1;   // x 초기 불확실성 [m²]
  P0(1, 1) = 0.1;   // y 초기 불확실성 [m²]
  P0(2, 2) = 0.05;  // θ 초기 불확실성 [rad²]
  P0(3, 3) = 1.0;   // vx 초기 불확실성 (속도 모름) [m²/s²]
  P0(4, 4) = 1.0;   // vy 초기 불확실성
  P0(5, 5) = 0.5;   // ω 초기 불확실성 [rad²/s²]

  // ekf_core와 초기화 시 차이가 존재하는 이유: 역할 분리 때문
  // ekf_core는 순수 수학 클래스. "로봇이 어디서 시작하는지", "TF 정확도가 얼마인지" 같은 걸 
  // 알 수도 없고 알아서도 안 됨.
  // map_ekf_node는 실제 상황을 아는 ROS2 노드. TF를 조회했고, 그 TF가 얼마나 믿을 만한지도 알고 있음.
  // ekf_core.cpp 생성자:
  //     P_.setZero()   ← "나는 P가 뭔지 모름. 누가 init()으로 줄 때까지 대기"
  // map_ekf_node.cpp initializeEkf():
  //     P0(0,0) = 0.1  ← "TF로 위치를 알았으니 위치 불확실성은 작게"
  //     P0(3,3) = 1.0  ← "속도는 아직 모르니 크게"
  //     ekf_.init(x0, P0)  ← "이제 줄게"
  // 흐름으로 보면
  // ekf_core 생성자 실행
  //     P_ = 0  (그냥 빈 상태. 아직 의미 없음)
  //     is_initialized_ = false
  //          ↓
  //     (TF 준비될 때까지 대기)
  //          ↓
  // initializeEkf() 호출 (map_ekf_node)
  //     실제 상황에 맞는 P0 결정
  //     ekf_.init(x0, P0) 호출
  //          ↓
  //     ekf_core 내부: P_ = P0 (비로소 의미 있는 값)
  //     is_initialized_ = true
  //          ↓
  //     predict(), update() 허용
  // 만약 ekf_core 생성자에 P0 값을 넣으면 무슨 문제가 생기냐?
  // // ekf_core 생성자에서
  // P_(0,0) = 0.1;   // x 불확실성
  // P_(3,3) = 1.0;   // vx 불확실성
  // ...
  // 이렇게 하면 ekf_core가 "초기 위치 불확실성은 0.1m²"라는 도메인 지식을 직접 들고 있게 됨.
  // 그런데 이 값은 상황마다 달라짐:
  // 로봇이 TF로 초기화하면 → 0.1m²가 적절
  // 로봇이 GPS로 초기화하면 → 0.01m²가 적절
  // 로봇 위치를 전혀 모르면 → 10.0m²가 적절
  // ekf_core에 이 값을 하드코딩하면 다른 상황에서 재사용하기 어려워짐. 
  // 노드에서 init(x0, P0)로 주입하는 구조이기 때문에 ekf_core는 어떤 상황에서도 그냥 받아서 쓰면 됨.
  {
    std::lock_guard<std::mutex> lock(ekf_mutex_); // 초기화 시 락
    ekf_.init(x0, P0);
    is_ekf_initialized_ = true;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "[map_ekf_node] EKF 초기화 완료. 초기 map pose: x=%.3f, y=%.3f, yaw=%.3f",
    init_x, init_y, init_yaw
  );
}

// ============================================================
// map→base_link TF 조회 헬퍼
// 왜 있는가?
// map_ekf_node가 LiDAR 관측값으로 EKF를 보정하려면 "로봇이 map 기준으로 지금 어디 있는가" 를 알아야 함.
// 이 정보는 slam_toolbox가 TF로 발행하고 있음. 그런데 TF를 읽는 코드가 여러 곳에서 필요:
// initializeEkf()    ← EKF 초기화할 때 map 기준 초기 위치 필요
// lidarUpdateCallback() ← 10Hz마다 LiDAR 관측값으로 update할 때 필요
// 이 두 곳에서 똑같이 TF를 읽어야 하니까, 중복을 없애고 한 곳에서 관리하려고 별도 함수로 분리한 것.

// TF 트리에서 어떻게 위치를 얻는가
// 현재 TF 트리 구조:
// map
//  └── odom          ← slam_toolbox가 발행 (map→odom)
//       └── base_link ← ekf_node가 발행 (odom→base_link)
// map→base_link는 직접 발행되는 TF가 아님. 그런데 TF 시스템은 트리를 자동으로 따라가서 간접 변환도 계산.
// lookupTransform("map", "base_link")
//     = map→odom (slam_toolbox) + odom→base_link (ekf_node)
//     = 자동으로 합성해서 반환
// 이것이 가능한 이유가 tf_buffer_. /tf 토픽을 구독해서 모든 TF를 저장, 요청이 오면 트리를 따라가며 변환을 계산.
// ============================================================
bool MapEkfNode::getMapPose(double & x, double & y, double & yaw)
{
  try {
    // lookupTransform("target_frame", "source_frame", time)
    // time = tf2::TimePointZero → 가장 최근 TF 반환
    // "map" 기준으로 "base_link"가 어디 있는가 를 물어보는 것.
    const auto tf = tf_buffer_->lookupTransform(
      "map", "base_link",
      tf2::TimePointZero
    );

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    // z는 2D 로봇이라 항상 0이니까 무시.

    // 쿼터니언 → yaw 추출
    // TF의 방향은 쿼터니언 형식. (x, y, z, w) 4개 숫자로 3D 회전을 표현하는 방식인데, 
    // 로봇은 yaw(θ) 하나만 필요해. tf2::getYaw()가 쿼터니언에서 yaw만 뽑음.
    // tf2::getYaw(): roll, pitch 무시하고 yaw만 반환
    // 2D 로봇이므로 roll=pitch=0 가정 → 문제 없음
    yaw = tf2::getYaw(tf.transform.rotation);
    return true;

  } catch (const tf2::TransformException & ex) {
    // TF 아직 없음, 타임아웃 등 — 정상적인 초기화 과정에서 발생할 수 있음
    //     TF 조회는 실패할 수 있음. 실패 상황 두 가지:
    // 상황 1 — 초기화 직후
    //     slam_toolbox가 아직 map→odom을 발행 안 했거나
    //     ekf_node가 아직 odom→base_link를 발행 안 한 경우
    //     → TF 트리가 완성되지 않음 → 예외 발생
    // 상황 2 — 런타임 중
    //     slam_toolbox가 잠깐 멈추거나
    //     네트워크 지연으로 TF가 늦게 도착한 경우
    //     → 예외 발생

    // WARN_THROTTLE으로 반복 로그 억제 (2초에 한 번만 출력)
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[map_ekf_node] map→base_link TF 조회 실패: %s", ex.what()
    );
    return false;
  }
}

}  // namespace estimation

// ============================================================
// main
// ============================================================
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_map_executor;

void signalHandlerMap(int signum)
{
  (void)signum;
  if (g_map_executor) {
    g_map_executor->cancel();
  }
  rclcpp::shutdown();
}

// 멀티스레드용 main 함수
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   std::signal(SIGINT, signalHandlerMap);
  
//   auto node = std::make_shared<estimation::MapEkfNode>();
//   g_map_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
//   g_map_executor->add_node(node);
  
//   g_map_executor->spin();
  
//   g_map_executor->remove_node(node);
//   return 0;
// }

// 싱글 스레드용 main 함수
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimation::MapEkfNode>());
  rclcpp::shutdown();
  return 0;
}