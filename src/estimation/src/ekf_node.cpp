#include "estimation/ekf_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace estimation
{

// ============================================================
// 생성자
//   노드 이름: "ekf_node"
//   여기서 subscriber, publisher, TF broadcaster,
//   파라미터, EKF 초기 노이즈를 모두 설정함.
// ============================================================
EkfNode::EkfNode(const rclcpp::NodeOptions & options)
: Node("ekf_node", options),
  first_imu_(true),
  is_ekf_initialized_(false),
  publish_tf_(true)
{
  // --------------------------------------------------------
  // ROS2 파라미터 선언
  //   declare_parameter로 미리 선언해야 외부에서 설정 가능.
  //   두 번째 인자가 기본값.
  //   실행 시 오버라이드 예시:
  //     ros2 run estimation ekf_node --ros-args -p publish_tf:=false
  // --------------------------------------------------------
  this->declare_parameter<bool>("publish_tf", true);
  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  // --------------------------------------------------------
  // QoS 프로파일 설정
  //
  //   [QoS란?]
  //     Quality of Service — ROS2에서 토픽 통신의 품질을 설정하는 옵션.
  //     DDS(Data Distribution Service) 기반이라 다양한 정책 지원.
  //
  //   sensor_data_qos: BEST_EFFORT + volatile (손실 허용, 지연 최소화)
  //     → IMU처럼 고속으로 들어오는 센서 데이터에 적합
  //
  //   reliable_qos: RELIABLE + volatile (손실 불허, 재전송 보장)
  //     → Odom처럼 EKF 보정에 반드시 필요한 데이터에 적합
  // --------------------------------------------------------
  auto sensor_qos    = rclcpp::SensorDataQoS();
  auto reliable_qos  = rclcpp::QoS(10).reliable();

  // --------------------------------------------------------
  // Subscriber 생성
  //   /imu  → 200Hz, BEST_EFFORT
  //   /odom → 50Hz,  RELIABLE
  //
  //   std::bind(&EkfNode::imuCallback, this, _1):
  //     멤버 함수를 콜백으로 등록하는 C++ 방식.
  //     _1은 첫 번째 인자(메시지 포인터)를 그대로 전달함.
  // --------------------------------------------------------
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu",
    sensor_qos,
    std::bind(&EkfNode::imuCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    reliable_qos,
    std::bind(&EkfNode::odomCallback, this, std::placeholders::_1)
  );

  // --------------------------------------------------------
  // Publisher 생성
  //   /ekf/odom → EKF 융합 결과 발행
  //   RELIABLE: 상위 모듈(MPC, SLAM)이 이 값을 반드시 받아야 함
  // --------------------------------------------------------
  ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/ekf/odom",
    reliable_qos
  );

  // --------------------------------------------------------
  // TF Broadcaster 생성
  //   odom → base_link 변환을 EKF 추정값으로 브로드캐스트.
  //   rviz, slam_toolbox 등이 이 TF를 기반으로 동작함.
  // --------------------------------------------------------
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // --------------------------------------------------------
  // EKF 노이즈 파라미터 초기 설정
  //   Q (프로세스 노이즈): 모델 불확실성
  //   R (관측 노이즈):     Odom 센서 불확실성
  //
  //   [튜닝 가이드]
  //     실험 결과를 보고 W7에서 본격 튜닝할 예정.
  //     지금은 보수적인 초기값으로 시작.
  //     Q를 키우면 → 관측(Odom)을 더 따라감 (빠른 반응)
  //     R을 키우면 → 예측(IMU 모델)을 더 믿음 (안정적)
  // --------------------------------------------------------
  StateMat Q;
  Q.setZero();
  Q(0, 0) = 0.01;   // x 위치
  Q(1, 1) = 0.01;   // y 위치
  Q(2, 2) = 0.001;  // yaw
  Q(3, 3) = 0.1;    // vx
  Q(4, 4) = 0.1;    // vy
  Q(5, 5) = 0.01;   // ω
  ekf_.setProcessNoise(Q);

  Eigen::Matrix<double, OBS_DIM, OBS_DIM> R;
  R.setZero();
  R(0, 0) = 0.1;    // vx 관측 노이즈
  R(1, 1) = 0.1;    // vy 관측 노이즈
  R(2, 2) = 0.05;   // ω 관측 노이즈
  ekf_.setMeasurementNoise(R);

  RCLCPP_INFO(this->get_logger(), "[EkfNode] 초기화 완료. /imu, /odom 대기 중...");
}

// ============================================================
// EKF 초기화 (첫 번째 Odom 수신 시 호출)
//
//   초기 상태:
//     위치(x, y): 0 (odom frame 원점에서 시작)
//     yaw(θ):    0
//     속도(vx, vy, ω): 첫 Odom 메시지의 속도값 사용
//
//   초기 공분산 P0:
//     대각 성분이 클수록 "초기 상태를 모른다"는 의미.
//     위치는 원점에서 시작하므로 작게,
//     속도는 불확실하므로 크게 설정.
// ============================================================
void EkfNode::initializeEkf(const nav_msgs::msg::Odometry::SharedPtr & odom_msg)
{
  StateVec x0;
  x0.setZero();
  // 첫 Odom 속도를 초기 속도로 사용
  x0(3) = odom_msg->twist.twist.linear.x;
  x0(4) = odom_msg->twist.twist.linear.y;
  x0(5) = odom_msg->twist.twist.angular.z;

  StateMat P0;
  P0.setZero();
  P0(0, 0) = 0.1;   // x 초기 불확실성 [m²]
  P0(1, 1) = 0.1;   // y 초기 불확실성 [m²]
  P0(2, 2) = 0.05;  // θ 초기 불확실성 [rad²]
  P0(3, 3) = 0.5;   // vx 초기 불확실성
  P0(4, 4) = 0.5;   // vy 초기 불확실성
  P0(5, 5) = 0.1;   // ω 초기 불확실성

  ekf_.init(x0, P0);
  is_ekf_initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "[EkfNode] EKF 초기화 완료.");
}

// ============================================================
// IMU 콜백 — Predict 단계 (200Hz)
//
//   [처리 흐름]
//     1. 첫 메시지면 타임스탬프만 저장하고 리턴 (dt 계산 불가)
//     2. dt 계산 (현재 타임스탬프 - 이전 타임스탬프)
//     3. IMU에서 가속도(ax, ay), 각속도(ω) 추출
//     4. ekf_.predict() 호출
//
//   [IMU 좌표계 주의]
//     ROS2 IMU 메시지의 linear_acceleration은 body frame 기준.
//     Gazebo IMU는 중력(9.81 m/s²)을 포함해서 출력함.
//     → 중력 보정 없이 그대로 사용하면 수직 가속도 오차 발생.
//     → 2D 평면 주행이라 az는 무시하고 ax, ay만 사용.
//     → 완벽한 구현은 중력 벡터를 IMU orientation으로 회전해서 빼줘야 함.
//        (W5 EKF 고도화에서 개선 예정)
// ============================================================
void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // EKF 초기화 전에는 IMU 처리 안 함
  // (초기 상태가 없으면 predict 결과가 무의미)
  if (!is_ekf_initialized_) {
    return;
  }

  // msg->header.stamp 대신 시스템 시간 사용
  // [왜?]
  //   Gazebo Fortress IMU는 연속된 메시지에 동일한 타임스탬프를
  //   찍는 버그가 있음. 메시지 헤더 시간으로 dt를 계산하면
  //   dt=0이 반복 발생 → predict()가 매번 무시됨.
  //   시스템 시간(this->now())은 실제 콜백 호출 간격을 반영하므로
  //   정상적인 dt를 얻을 수 있음.
  const rclcpp::Time current_time = this->now();

  // 첫 IMU 메시지: dt 계산 기준 타임스탬프만 저장
  if (first_imu_) {
    last_imu_time_ = current_time;
    first_imu_ = false;
    return;
  }

  // dt 계산 [s]
  const double dt = (current_time - last_imu_time_).seconds();
  last_imu_time_ = current_time;

  // 비정상적인 dt 방어 (타임스탬프 역전, 너무 큰 간격 등)
  if (dt <= 0.0 || dt > 0.5) {
    RCLCPP_WARN(this->get_logger(),
      "[EkfNode] 비정상 dt=%.4f 무시", dt);
    return;
  }

  // IMU 메시지에서 입력값 추출
  // linear_acceleration: body frame 기준 가속도 [m/s²]
  // angular_velocity:    body frame 기준 각속도 [rad/s]
  const double ax    = msg->linear_acceleration.x;
  const double ay    = msg->linear_acceleration.y;
  const double omega = msg->angular_velocity.z;  // yaw rate

  // EKF Predict 단계 실행
  ekf_.predict(ax, ay, omega, dt);
}

// ============================================================
// Odom 콜백 — Update 단계 (50Hz)
//
//   [처리 흐름]
//     1. 첫 Odom이면 EKF 초기화
//     2. Odom에서 vx, vy, ω 추출
//     3. ekf_.update() 호출
//     4. publishEkfOdom() 호출로 결과 발행
//
//   [innovation norm 모니터링]
//     update 후 innovation_norm이 임계값 초과 시 경고.
//     W10 Watchdog에서 이 값을 기반으로 Fail-safe 트리거 예정.
// ============================================================
void EkfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 첫 Odom 수신 시 EKF 초기화
  if (!is_ekf_initialized_) {
    initializeEkf(msg);
    return;
  }

  // Odom 메시지에서 관측값 추출
  // twist.twist.linear: body frame 기준 선속도 [m/s]
  // twist.twist.angular.z: 각속도 [rad/s]
  const double vx    = msg->twist.twist.linear.x;
  const double vy    = msg->twist.twist.linear.y;
  const double omega = msg->twist.twist.angular.z;

  // EKF Update 단계 실행
  ekf_.update(vx, vy, omega);

  // innovation norm 경고 (EKF divergence 조기 감지)
  // 임계값 5.0은 경험적 초기값 — W10에서 Watchdog 연동 시 정밀 튜닝 예정
  constexpr double INNOVATION_WARN_THRESHOLD = 5.0;
  if (ekf_.getInnovationNorm() > INNOVATION_WARN_THRESHOLD) {
    RCLCPP_WARN(this->get_logger(),
      "[EkfNode] innovation norm 과대: %.3f (EKF divergence 의심)",
      ekf_.getInnovationNorm());
  }

  // 결과 발행
  publishEkfOdom(msg->header.stamp);
}

// ============================================================
// EKF 결과 발행
//
//   EkfCore 상태 벡터 → nav_msgs::Odometry 메시지 변환 후 발행.
//   동시에 odom → base_link TF 브로드캐스트.
//
//   [공분산 채우기]
//     nav_msgs::Odometry의 pose.covariance는 6×6 = 36개 원소.
//     순서: [x, y, z, roll, pitch, yaw]
//     EKF P 행렬의 관련 성분을 매핑해서 채움.
//     나머지(z, roll, pitch)는 2D 주행이라 0 또는 큰 값으로 채움.
// ============================================================
void EkfNode::publishEkfOdom(const rclcpp::Time & stamp)
{
  const auto & x = ekf_.getState();
  const auto & P = ekf_.getCovariance();

  // ---- Odometry 메시지 구성 ----
  nav_msgs::msg::Odometry ekf_msg;
  ekf_msg.header.stamp    = stamp;
  ekf_msg.header.frame_id = "odom";       // 기준 좌표계
  ekf_msg.child_frame_id  = "base_link";  // 로봇 좌표계

  // 위치 (x, y) — z는 2D 주행이라 0
  ekf_msg.pose.pose.position.x = x(0);
  ekf_msg.pose.pose.position.y = x(1);
  ekf_msg.pose.pose.position.z = 0.0;

  // yaw → 쿼터니언 변환
  // [쿼터니언이란?]
  //   3D 회전을 4개 숫자(x, y, z, w)로 표현하는 방식.
  //   gimbal lock(특이점) 없이 모든 회전을 표현 가능.
  //   ROS2 메시지는 쿼터니언 형식만 지원하므로 변환 필요.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x(2));  // roll=0, pitch=0, yaw=θ
  ekf_msg.pose.pose.orientation = tf2::toMsg(q);

  // 속도 (vx, vy, ω)
  ekf_msg.twist.twist.linear.x  = x(3);
  ekf_msg.twist.twist.linear.y  = x(4);
  ekf_msg.twist.twist.angular.z = x(5);

  // 공분산 채우기 (6×6 → 36개 원소, row-major)
  // 인덱스: 0=x, 1=y, 2=z, 3=roll, 4=pitch, 5=yaw
  ekf_msg.pose.covariance.fill(0.0);
  ekf_msg.pose.covariance[0]  = P(0, 0);   // x-x
  ekf_msg.pose.covariance[7]  = P(1, 1);   // y-y
  ekf_msg.pose.covariance[35] = P(2, 2);   // yaw-yaw

  ekf_msg.twist.covariance.fill(0.0);
  ekf_msg.twist.covariance[0]  = P(3, 3);  // vx-vx
  ekf_msg.twist.covariance[7]  = P(4, 4);  // vy-vy
  ekf_msg.twist.covariance[35] = P(5, 5);  // ω-ω

  ekf_pub_->publish(ekf_msg);

  // ---- TF 브로드캐스트 ----
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link";

    tf_msg.transform.translation.x = x(0);
    tf_msg.transform.translation.y = x(1);
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace estimation