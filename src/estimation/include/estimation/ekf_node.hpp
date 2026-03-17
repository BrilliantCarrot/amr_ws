#ifndef ESTIMATION__EKF_NODE_HPP_
#define ESTIMATION__EKF_NODE_HPP_

// ============================================================
// ekf_node.hpp — EKF ROS2 노드 헤더
//
// [이 파일의 역할]
//   ekf_core.hpp/cpp 가 순수 수식 담당이라면,
//   이 파일은 ROS2와의 연결을 담당함.
//   - /imu 토픽 구독 → predict() 호출
//   - /odom 토픽 구독 → update() 호출
//   - /ekf/odom 토픽 발행 → 융합된 추정값 출력
//
// [설계 철학: 관심사 분리 (Separation of Concerns)]
//   EkfCore  : 수학/알고리즘 (ROS 의존성 없음 → 단위 테스트 가능)
//   EkfNode  : ROS2 인터페이스 (토픽, 타이머, 파라미터 관리)
//   이렇게 분리하면 ROS 없이도 EKF 로직만 따로 검증할 수 있음.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "estimation/ekf_core.hpp"

namespace estimation
{

class EkfNode : public rclcpp::Node
{
public:
  // --------------------------------------------------------
  // 생성자
  //   rclcpp::Node를 상속받아 "ekf_node"라는 이름으로 노드 생성.
  //   내부에서 subscriber, publisher, 파라미터 초기화 수행.
  // --------------------------------------------------------
  explicit EkfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // explicit: 암묵적 형변환을 막고 명시적 형변환만 허용
  // rclcpp::NodeOptions 란?
  // 노드를 생성할 때 옵션을 묶어서 전달하는 설정 객체야.
  // rclcpp::NodeOptions options;
  // options.use_intra_process_comms(true);   // 같은 프로세스 내 통신 최적화
  // options.allow_undeclared_parameters(true); // 미선언 파라미터 허용
  // options.automatically_declare_parameters_from_overrides(true); // 런타임 파라미터 자동 선언

  // ROS2에서 NodeOptions를 생성자에 넣는 이유:
  // ROS2 컴포넌트 시스템(component) 때문이야.
  // 여러 노드를 하나의 프로세스에 합쳐서 실행할 때
  // (성능 최적화 목적) NodeOptions가 필요함.

private:
  // --------------------------------------------------------
  // IMU 콜백 (200Hz)
  //   /imu 토픽 수신 시 호출됨.
  //   IMU 메시지에서 선가속도(ax, ay)와 각속도(ω)를 추출해
  //   EkfCore::predict()를 호출함.
  //
  //   [QoS 설정: BEST_EFFORT]
  //     IMU는 고속(200Hz) 센서라 일부 패킷 손실보다
  //     지연 없이 빠르게 받는 게 더 중요함.
  //     RELIABLE은 재전송을 보장하지만 지연이 생길 수 있음.
  //     → 실시간성이 우선인 센서 데이터엔 BEST_EFFORT가 적합.
  // --------------------------------------------------------
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // --------------------------------------------------------
  // Odom 콜백 (50Hz)
  //   /odom 토픽 수신 시 호출됨.
  //   Odom 메시지에서 선속도(vx, vy)와 각속도(ω)를 추출해
  //   EkfCore::update()를 호출한 뒤 결과를 publish함.
  //
  //   [QoS 설정: RELIABLE]
  //     Odom은 EKF 보정의 유일한 관측 소스라서
  //     데이터 손실이 생기면 안 됨.
  //     → RELIABLE로 수신 보장.
  // --------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --------------------------------------------------------
  // EKF 결과 발행
  //   EkfCore의 현재 상태를 nav_msgs::Odometry 형식으로 변환해
  //   /ekf/odom 토픽으로 발행함.
  //   동시에 TF(odom → base_link)도 브로드캐스트함.
  //
  //   [왜 TF도 브로드캐스트하나?]
  //     ROS2에서 좌표 변환은 TF 트리로 관리됨.
  //     EKF 추정값이 odom → base_link TF를 대체하면
  //     rviz, slam_toolbox 등 상위 모듈이 자동으로 활용 가능.
  // --------------------------------------------------------
  void publishEkfOdom(const rclcpp::Time & stamp);

  // --------------------------------------------------------
  // EKF 초기화
  //   첫 번째 Odom 메시지가 들어왔을 때 호출됨.
  //   초기 상태와 공분산을 설정하고 EkfCore::init()을 호출.
  //
  //   [왜 생성자가 아니라 첫 메시지에서 초기화하나?]
  //     Gazebo 시작 직후 센서값이 불안정할 수 있음.
  //     첫 실제 메시지를 받은 시점에 초기화하면
  //     그 시점의 실제 상태에서 EKF를 시작할 수 있어 더 안정적.
  // --------------------------------------------------------
  void initializeEkf(const nav_msgs::msg::Odometry::SharedPtr & odom_msg);

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         ekf_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;

  // --- EKF 핵심 연산 객체 ---
  EkfCore ekf_;

  // --- 시간 관리 ---
  // IMU는 200Hz로 들어오므로 연속된 두 메시지 사이의 dt 계산이 필요함.
  // last_imu_time_: 이전 IMU 메시지의 타임스탬프 저장용
  rclcpp::Time last_imu_time_;
  bool         first_imu_;   // 첫 IMU 메시지 여부 (dt 계산 시작 기준)
  bool         is_ekf_initialized_;  // EKF init() 완료 여부

  // --- 파라미터 ---
  // ROS2 파라미터 시스템으로 런타임에 외부에서 조정 가능
  // (ros2 param set /ekf_node publish_tf true 같은 방식)
  bool publish_tf_;  // TF 브로드캐스트 여부 (기본 true)
};

}  // namespace estimation

#endif  // ESTIMATION__EKF_NODE_HPP_