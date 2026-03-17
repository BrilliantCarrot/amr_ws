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
  publish_tf_ = this->get_parameter("publish_tf").as_bool(); // "publish_tf"라는 이름의 파라미터 객체 반환

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
  // this->create_subscription<MessageType>(
  // '<>' 안이 템플릿 인자, 어떤 메시지 타입을 구독할 것인가를 지정함
  // C++ 템플릿이란?
  // 함수/클래스를 타입에 관계없이 재사용하게 해주는 문법.
  // create_subscription 함수 하나로
  // Imu도, Odometry도, LaserScan도 구독 가능한 이유가 이것 때문.
  //
  //     "토픽이름",
  //     QoS설정,
  //        콜백함수);
  //   [왜 this가 필요한가?]
  //   imuCallback은 EkfNode의 멤버 함수라서
  //   어떤 객체의 함수인지 알아야 호출 가능함.
  //   클래스 인스턴스가 여러 개일 수도 있으니까.

  //   예: ekf_node_1.imuCallback(msg)
  //       ekf_node_2.imuCallback(msg)
  //   → this가 "나(현재 객체)의 imuCallback"을 가리킴
  // 

  // `std::placeholders::_1`
  // 
  // "첫 번째 인자는 나중에 채울게"라는 의미의 자리 표시자.

  // [왜 필요한가?]
  //   create_subscription은 나중에 메시지가 오면
  //   콜백을 이렇게 호출함:
  //     callback(msg)

  //   std::bind로 콜백을 등록할 때
  //   msg 자리를 비워두는 것이 _1.
  //   메시지가 실제로 왔을 때 _1 자리에 msg가 들어감.

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

  RCLCPP_INFO(this->get_logger(), "[EkfNode] 초기화 완료. /imu, /odom 대기 중...");
}

// ============================================================
// EKF 초기화 (첫 번째 Odom 수신 시 호출)
// 무엇으로 초기화할지 결정(정책)
// 실제로 초기화 실행은 ekf_core.cpp에서 EKFCore::init() 에서 수행
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
// 구체적으로 하는 일
// EkfNode::initializeEkf()가 하는 일:
// 1. 첫 Odom 메시지에서 초기 속도 추출
// 2. P0 값을 어떻게 설정할지 결정
// 3. EkfCore::init()을 호출해서 실제 초기화 위임
// 4. is_ekf_initialized_ = true 플래그 설정
//    → 이후 IMU 콜백에서 predict() 시작 허용
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
  // 만약 `return`이 없으면:
  // IMU 콜백 호출
  //   → is_ekf_initialized_ = false인데
  //   → ekf_.predict() 호출
  //   → EkfCore 내부에서 is_initialized_ = false 확인
  //   → throw runtime_error 예외 발생
  //   → 노드 크래시
  // `return`이 있으면:
  // IMU 콜백 호출
  //   → is_ekf_initialized_ = false 확인
  //   → return → 함수 즉시 종료 (predict 호출 안 함)
  //   → 안전하게 다음 IMU 메시지 대기

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
  //   update()가 반환값 없이 void인데도 결과가 저장되는 이유는, 반환하는 게 아니라 클래스 내부 멤버 변수를 직접 수정하기 때문이야.
  // // ekf_core.cpp의 update() 마지막 부분
  // x_ = x_ + K * y;   // ← 반환이 아니라 멤버변수 x_를 덮어씀
  // P_ = (I - K * H_) * P_;  // ← 멤버변수 P_를 덮어씀

  // `x_`와 `P_`는 `EkfCore` 클래스의 멤버 변수야. 함수가 끝나도 **객체가 살아있는 한 값이 유지됨.
  // 메모리 관점으로 보면
  // EkfCore 객체 (ekf_) 가 메모리에 올라가 있음
  // ┌─────────────────────────┐
  // │  EkfCore                │
  // │  ├── x_  : [0.5, 0.3, ...]  ← 상태 벡터 (멤버 변수)
  // │  ├── P_  : [[0.01, ...]]    ← 공분산 (멤버 변수)
  // │  ├── Q_  : [[0.01, ...]]
  // │  └── R_  : [[0.1, ...]]
  // └─────────────────────────┘

  // update() 호출
  //   → x_, P_ 값을 내부에서 수정
  //   → 함수 종료

  // publishEkfOdom() 호출
  //   → ekf_.getState() 로 x_ 읽음  ← 같은 객체의 같은 메모리
  //   → 이미 update()가 수정해둔 값이 그대로 있음

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
  // 공분산 행렬이 뭔가
  // 공분산 행렬 P는 추정값의 불확실성을 나타내는 행렬이야.

  // 단순히 "얼마나 틀렸냐"가 아니라 "어느 방향으로 얼마나 틀렸냐" 까지 담고 있어.

  // P = [σ_xx   σ_xy   σ_xθ  ...]
  //     [σ_yx   σ_yy   σ_yθ  ...]
  //     [σ_θx   σ_θy   σ_θθ  ...]
  //     [  ...                  ]

  // 대각 성분 σ_xx, σ_yy, ...  → 각 상태의 분산 (얼마나 불확실한지)
  // 비대각 성분 σ_xy, σ_xθ, ...→ 두 상태 간의 상관관계
  // 예를 들어 σ_xθ (x와 θ의 공분산)가 크면:

  // "x 추정이 틀리면 θ 추정도 같이 틀릴 가능성이 높다"
  // → 로봇이 대각선으로 달릴 때 위치 오차와 각도 오차가 연동됨
  // nav_msgs/Odometry의 공분산 구조
  // ROS2의 nav_msgs/Odometry는 공분산을 6×6 = 36개 원소의 1차원 배열로 저장해.

  // 6개 성분의 순서는 고정되어 있어:

  // 인덱스: 0=x, 1=y, 2=z, 3=roll, 4=pitch, 5=yaw
  // 6×6 행렬을 row-major(행 우선)로 펼치면:

  // 행렬 위치 (행, 열) → 1D 배열 인덱스
  // (0,0)=x-x    → [0]   (0,1)=x-y    → [1]   ... (0,5)=x-yaw  → [5]
  // (1,0)=y-x    → [6]   (1,1)=y-y    → [7]   ... (1,5)=y-yaw  → [11]
  // (2,0)=z-x    → [12]  (2,1)=z-y    → [13]  ... (2,5)=z-yaw  → [17]
  // (3,0)=roll-x → [18]  ...
  // (4,0)=pitch-x→ [24]  ...
  // (5,0)=yaw-x  → [30]  ...          ... (5,5)=yaw-yaw → [35]
  // 대각 성분만 추출하면:

  // [0]  = x-x       인덱스 계산: 0×6+0 = 0
  // [7]  = y-y       인덱스 계산: 1×6+1 = 7
  // [14] = z-z       인덱스 계산: 2×6+2 = 14
  // [21] = roll-roll  인덱스 계산: 3×6+3 = 21
  // [28] = pitch-pitch 인덱스 계산: 4×6+4 = 28
  // [35] = yaw-yaw   인덱스 계산: 5×6+5 = 35
  // 우리 코드에서 한 일
  // 우리 EKF 상태벡터는 [x, y, θ, vx, vy, ω] 6개인데, ROS 공분산은 [x, y, z, roll, pitch, yaw] 기준이야. 2D 주행이라 z, roll, pitch는 없으니까 대각 성분만 매핑한 거야:

  // EKF P 행렬          ROS covariance 배열
  // P(0,0) = x 분산  →  [0]  = x-x       ✅
  // P(1,1) = y 분산  →  [7]  = y-y       ✅
  // P(2,2) = θ 분산  →  [35] = yaw-yaw   ✅ (z,roll,pitch 건너뜀)

  // P(3,3) = vx 분산 →  twist[0]  = vx-vx  ✅
  // P(4,4) = vy 분산 →  twist[7]  = vy-vy  ✅
  // P(5,5) = ω 분산  →  twist[35] = ω-ω    ✅
  // 비대각 성분은 왜 0으로 뒀나
  // ekf_msg.pose.covariance.fill(0.0);
  // 우리 EKF P 행렬 내부엔 비대각 성분도 계산되어 있어. 근데 ROS 메시지에 넣을 때 0으로 채운 이유:

  // 1. 2D 주행이라 z, roll, pitch 관련 공분산은 실제로 0
  // 2. 상위 모듈(MPC, slam_toolbox)이 대각 성분만 주로 활용
  // 3. 완전한 공분산을 넣으려면 비대각 인덱스 매핑이 복잡해짐
  //    → 포트폴리오 수준에서는 대각 성분만으로 충분

  // 완전한 구현이라면:
  //   covariance[1] = P(0,1)   // x-y 상관관계
  //   covariance[5] = P(0,2)   // x-yaw 상관관계
  //   ... 등을 추가해야 함
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
  //   EKF가 추정한 로봇 위치를 TF 트리에 등록하는 코드.
  // TF 트리란?
  //   ROS2에서 모든 좌표계(frame)의 관계를 트리 구조로 관리하는 시스템.
  //   "A 좌표계에서 B 좌표계로 가려면 어떻게 변환하나?"를
  //   전체 시스템이 공유함.
  // 현재 TF 트리:
  //   odom → base_link  ← 이 변환을 EKF가 담당
  //           ├── imu_link
  //           ├── laser_link
  //           └── wheel들
  //   "odom 좌표계 기준으로 base_link가 어디 있는지"를 선언.
  // 방향: odom → base_link
  // 의미: "odom 원점에서 로봇(base_link)까지의 변환"
  // odom 원점 = 로봇이 처음 켜진 위치 (0, 0, 0)
  // [왜 odom이 부모고 base_link가 자식인가?]
  //   TF 트리는 부모 → 자식 방향으로 변환을 정의함.
  //   odom은 고정된 기준점, base_link는 움직이는 로봇.
  //   로봇이 움직이면 odom→base_link 변환이 바뀌는 것.

  // world 좌표계:
  //   Gazebo가 사용하는 절대 좌표계.
  //   Gazebo를 켤 때부터 존재하는 진짜 고정 원점.
  //   Ground Truth가 이 기준으로 나옴.

  // odom 좌표계:
  //   로봇이 켜진 순간의 위치를 원점으로 잡는 좌표계.
  //   EKF, wheel encoder가 이 기준으로 위치를 적분.
  //   시간이 지나면 드리프트가 쌓여서 world와 어긋남.

  // map 좌표계 (W3에서 등장):
  //   SLAM이 만든 지도의 기준 좌표계.
  //   loop closure로 드리프트를 보정하므로 world에 가까움.
  //   GPS-denied에서 odom을 보정하는 역할.
  //   세 좌표계 관계
  // world (Gazebo 절대 원점)
  //   │
  //   │  ← 이 차이가 드리프트 (W5에서 LiDAR로 보정)
  //   ▼
  // map (SLAM 지도 원점, W3에서 추가)
  //   │
  //   │  ← slam_toolbox가 발행하는 TF
  //   ▼
  // odom (로봇 시작 위치)
  //   │
  //   │  ← EKF가 발행하는 TF (지금 우리가 만든 것)
  //   ▼
  // base_link (로봇 현재 위치)
  // world랑 같은 좌표계?
  // 처음 켤 때는 거의 같음.
  //   로봇 스폰 위치 = (0, 0)이면 odom 원점 ≈ world 원점

  // 시간이 지나면 달라짐.
  //   EKF는 적분으로 위치를 추적하므로 오차가 쌓임.
  //   → odom이 world에서 점점 어긋남 (드리프트)
  //   → 이게 W2 실험에서 RMSE가 커지는 이유
  // world에서 body로 바꾸는 거?
  // 엄밀히 말하면 아님.

  // 우리 코드가 하는 것:
  //   odom → base_link 변환을 발행

  // 즉:
  //   "odom 좌표계에서 base_link가 어디 있는가"
  //   = "로봇 시작점 기준으로 로봇이 얼마나 이동했는가"

  // world → body 변환을 하려면:
  //   world → map → odom → base_link
  //   전체 체인이 연결되어야 함.
  //   → W3 SLAM이 map→odom을 채워주면 완성
  // 전체 그림으로 정리
  // 지금 W2 TF 트리:
  //   odom ──(EKF)──▶ base_link

  // W3 완성 후 TF 트리:
  //   map ──(slam_toolbox)──▶ odom ──(EKF)──▶ base_link

  // W5 완성 후 (이상적):
  //   world ≈ map ──▶ odom ──▶ base_link
  //   (LiDAR fusion으로 map이 world에 가까워짐)

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp; // 이 변환이 유효한 시간
    tf_msg.header.frame_id = "odom"; // 부모 프레임(기준 좌표계)
    tf_msg.child_frame_id  = "base_link"; // 자식 프레임(변환 대상)

    tf_msg.transform.translation.x = x(0); // EKF 추정 X 위치
    tf_msg.transform.translation.y = x(1); // EKF 추정 y 위치
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q); 
  //   [쿼터니언을 쓰는 이유]
  // ROS2 TF는 rotation을 쿼터니언으로만 표현함.
  // 오일러각(roll, pitch, yaw)은 gimbal lock 문제가 있어서
  // 3D 회전을 안정적으로 표현하기 위해 쿼터니언을 씀.
  // 2D 주행이라 사실상 yaw만 쓰지만 형식은 맞춰야 함.

    tf_broadcaster_->sendTransform(tf_msg);
  //     완성된 변환 메시지를 TF 시스템에 브로드캐스트.
  // 내부 동작:
  //   /tf 토픽으로 메시지를 발행함.
  //   ROS2의 모든 노드는 /tf를 구독해서
  //   최신 좌표 변환 정보를 유지함.
  // 브로드캐스트하면:
  //   rviz → 로봇 위치 자동 업데이트
  //   slam_toolbox → map→odom TF 계산에 활용
  //   MPC(W6~) → map frame 기준 로봇 위치 파악에 활용
  }
}

}  // namespace estimation