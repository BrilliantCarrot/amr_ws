#ifndef ESTIMATION__MAP_EKF_NODE_HPP_
#define ESTIMATION__MAP_EKF_NODE_HPP_

// ============================================================
// map_ekf_node.hpp — map 프레임 기반 LiDAR fusion EKF 노드
//
// [W5 목표]
//   기존 ekf_node는 odom 프레임 기준 (IMU + Odom만 fusion).
//   이 노드는 map 프레임 기준으로 EKF를 운영하며
//   LiDAR pose(slam_toolbox 추정값)를 세 번째 관측으로 추가함.
//
// [ekf_node와의 차이]
//   항목              ekf_node              map_ekf_node
//   기준 프레임        odom                  map
//   관측 소스          IMU + Odom            IMU + Odom + LiDAR pose
//   출력 토픽          /ekf/odom             /map_ekf/odom
//   TF 브로드캐스트    odom→base_link        없음 (ekf_node와 충돌 방지)
//   용도              비교 기준(baseline)    W5 검증 대상
//
// [실행 의존 순서]
//   ekf_node → slam_toolbox → map_ekf_node
//   이유: map→base_link TF는
//         ekf_node(odom→base_link)와 slam_toolbox(map→odom)가
//         둘 다 떠 있어야 TF 트리에서 조회 가능함.
//
// [LiDAR 관측 노이즈 R_lidar 가변 전략]
//   localization_monitor가 발행하는 /localization/status를 구독해서
//   상태에 따라 R_lidar를 동적으로 조정함:
//     NORMAL   → R_lidar 작게 (LiDAR 적극 반영, 드리프트 강하게 보정)
//     DEGRADED → R_lidar 크게 (LiDAR 반신반의, 보정 약하게)
//     LOST     → updateLidar() 자체를 호출하지 않음 (Odom+IMU만으로 유지)

// frame_id가 중요한 이유:
// msg.header.frame_id = "map"; // ekf_node는 "odom"
// msg.child_frame_id  = "base_link";
// frame_id = "map"은 "이 위치 데이터가 map 좌표계 기준이다"라는 선언.
// ekf_node는 frame_id = "odom"을 쓰고, map_ekf_node는 frame_id = "map"을 씀. 
// rviz나 다른 노드들이 이 필드를 보고 어느 좌표계에서 그려야 할지 결정하기 때문에 정확히 써야 함.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <amr_msgs/msg/localization_status.hpp>
#include <mutex>
#include "estimation/ekf_core.hpp"
#include "std_msgs/msg/float64.hpp"

namespace estimation
{

class MapEkfNode : public rclcpp::Node
{
public:
  // --------------------------------------------------------
  // 생성자
  //   노드 이름: "map_ekf_node"
  //   subscriber, publisher, TF 리스너, EKF 초기화 수행.
  // --------------------------------------------------------
  explicit MapEkfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // IMU 콜백 (200Hz) — predict()
  //   ekf_node의 imuCallback과 동일한 역할.
  //   map 프레임이어도 운동 방정식(body frame 기준)은 동일함.
  // --------------------------------------------------------
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // --------------------------------------------------------
  // Odom 콜백 (50Hz) — update() + publish
  //   속도(vx, vy, ω) 관측으로 IMU 적분 오차를 보정.
  //   보정 후 /map_ekf/odom 발행.
  //
  //   [왜 Odom을 map frame EKF에서도 쓰나?]
  //     LiDAR pose는 10Hz로 느리게 들어옴.
  //     그 사이 50Hz Odom이 속도 보정을 계속 해줘야
  //     LiDAR 업데이트 사이 구간에서 드리프트를 최소화할 수 있음.
  // --------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --------------------------------------------------------
  // Localization 상태 콜백 (10Hz)
  //   /localization/status를 구독해서 현재 localization 품질을 캐시.
  //   LiDAR update 타이머에서 R_lidar 결정에 사용.
  // --------------------------------------------------------
  void localizationStatusCallback(
    const amr_msgs::msg::LocalizationStatus::SharedPtr msg);

  // --------------------------------------------------------
  // LiDAR 업데이트 타이머 콜백 (10Hz)
  //   TF 버퍼에서 map→base_link를 조회해서
  //   EkfCore::updateLidar()를 호출함.
  //
  //   [호출 조건]
  //     - EKF가 초기화된 상태
  //     - localization status가 LOST가 아님
  //     - map→base_link TF 조회 성공
  //
  //   [R_lidar 결정]
  //     NORMAL   → r_lidar_normal_ 사용   (작은 값, 신뢰)
  //     DEGRADED → r_lidar_degraded_ 사용 (큰 값, 불신)
  // --------------------------------------------------------
  void lidarUpdateCallback();

  // --------------------------------------------------------
  // EKF 결과 발행
  //   EKF 현재 상태를 nav_msgs::Odometry로 변환해
  //   /map_ekf/odom으로 발행.
  //   frame_id = "map", child_frame_id = "base_link"

  // 질문: publishMapEkfOdom()이 하는 일
  // 이 함수가 하는 일을 한 문장으로 요약하면:
  // EkfCore 안에 있는 숫자 6개를 ROS2가 이해하는 메시지 형식으로 변환해서 토픽으로 내보내는 것.
  //
  //   ※ TF 브로드캐스트는 하지 않음.
  //     ekf_node가 이미 odom→base_link를 브로드캐스트하고 있으므로
  //     여기서 map→base_link를 추가로 브로드캐스트하면
  //     TF 트리에 루프가 생겨 충돌함.
  // --------------------------------------------------------
  // void publishMapEkfOdom(const rclcpp::Time & stamp);
  void publishTimerCallback(); 

  // --------------------------------------------------------
  // EKF 초기화
  //   조건: 첫 odom 수신 + map→base_link TF 조회 성공
  //   초기 상태: map frame 기준 위치(x, y, θ) + 속도 0으로 시작
  //
  //   [왜 map→base_link TF로 초기화하나?]
  //     EKF 상태가 map 프레임 기준이므로
  //     초기값도 map 프레임의 실제 위치에서 시작해야 함.
  //     odom 프레임(0,0,0)에서 시작하면 초기 LiDAR innovation이
  //     매우 커져서 EKF가 불안정해질 수 있음.

  // 질문: Q, R 초기화가 중복 아닌가
  // ekf_core.cpp 기본값 → map_ekf_node.cpp에서 덮어쓰기 순서로 실행됨.
  // 왜 중복으로 설계했냐면:
  // ekf_core는 ROS2 의존성이 없는 순수 수학 클래스. 어디서든 쓸 수 있어야 함. 
  // 그래서 생성자에서 "일단 쓸 수 있는 기본값"을 설정하는 게 맞음.
  // 반면 map_ekf_node는 실제 로봇 시스템에서 쓰이는 노드. 
  // 여기서 명시적으로 Q, R을 설정하면 두 가지 장점이 생김:
  // 첫째, 가독성. 코드를 읽는 사람이 map_ekf_node.cpp만 봐도 "이 노드가 어떤 노이즈 값을 쓰는지" 바로 알 수 있음.
  // ekf_core.cpp까지 뒤질 필요가 없음.
  // 둘째, 비교 실험 보장. ekf_node와 map_ekf_node가 동일한 Q, R로 시작하는 것을 
  // 명시적으로 보장할 수 있음. 나중에 누군가 ekf_core.cpp 기본값을 바꿔도 map_ekf_node.cpp의 값은 그대로 유지됨.
  // 만약 map_ekf_node.cpp에서 명시하지 않으면 ekf_core.cpp 기본값에 
  // 암묵적으로 의존하게 되는데, 이건 나중에 버그를 만들기 쉬운 구조.
  // --------------------------------------------------------
  void initializeEkf();

  // --------------------------------------------------------
  // map→base_link TF 조회 헬퍼
  //   성공 시 true, x/y/yaw에 값 채워서 반환.
  //   실패 시 false 반환 (TF 없음, 타임아웃 등).
  // --------------------------------------------------------
  bool getMapPose(double & x, double & y, double & yaw);

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr             imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr  loc_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               map_ekf_pub_;
  // W10 Step2: watchdog_node의 EKF_DIVERGED 조건 감지용
  // EkfCore::getInnovationNorm() 값을 50Hz로 발행
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                innovation_norm_pub_;
  rclcpp::TimerBase::SharedPtr                                        lidar_timer_;
  // W11 Step5-2: ekf_ 객체 보호용 mutex
  // MultiThreadedExecutor에서 imu/odom/lidar 콜백이 병렬 실행될 때
  // ekf_ 상태(x_, P_)에 대한 동시 접근을 방지
  std::mutex ekf_mutex_;
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr pub_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr     pub_timer_;

  // --- TF ---
  // map→base_link TF 조회용
  // (slam_toolbox의 map→odom + ekf_node의 odom→base_link 조합으로 자동 구성됨)
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- EKF 핵심 연산 객체 ---
  EkfCore ekf_;

  // --- 시간 관리 ---
  rclcpp::Time last_imu_time_;
  bool         first_imu_;
  bool         is_ekf_initialized_;

  // --- Localization 상태 캐시 ---
  // localizationStatusCallback에서 갱신, lidarUpdateCallback에서 읽음
  uint8_t cached_loc_status_;  // 0=NORMAL, 1=DEGRADED, 2=LOST
  // LocalizationStatus.msg의 status 상수와 맞춤
  static constexpr uint8_t STATUS_NORMAL   = 0;
  static constexpr uint8_t STATUS_DEGRADED = 1;
  static constexpr uint8_t STATUS_LOST     = 2;

  // --- LiDAR 관측 노이즈 파라미터 ---
  // NORMAL / DEGRADED 상태별 R_lidar 대각 성분
  // ROS2 파라미터로 런타임 조정 가능
  double r_lidar_normal_xy_;    // NORMAL 상태 위치 노이즈 [m²]
  double r_lidar_normal_yaw_;   // NORMAL 상태 yaw 노이즈 [rad²]
  double r_lidar_degraded_xy_;  // DEGRADED 상태 위치 노이즈 [m²] (더 큰 값)
  double r_lidar_degraded_yaw_; // DEGRADED 상태 yaw 노이즈 [rad²] (더 큰 값)
};

}  // namespace estimation

#endif  // ESTIMATION__MAP_EKF_NODE_HPP_
