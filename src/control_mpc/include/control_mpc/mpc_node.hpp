#ifndef CONTROL_MPC__MPC_NODE_HPP_
#define CONTROL_MPC__MPC_NODE_HPP_

// ============================================================
// mpc_node.hpp — MPC ROS2 노드 헤더
//
// [이 파일의 역할]
//   mpc_core가 순수 수학/최적화를 담당한다면,
//   이 노드는 ROS2와의 연결을 담당함.
//   - /map_ekf/odom 구독 → 현재 상태 추출
//   - reference trajectory 생성 (waypoints 기반)
//   - mpc_core.solve() 호출
//   - /cmd_vel 발행 → Gazebo로 전달
//   - /metrics/control_latency_ms 발행 → KPI 측정
//
// [ekf_node와의 설계 패턴 동일]
//   MpcCore  : 수학/알고리즘 (ROS 의존성 없음)
//   MpcNode  : ROS2 인터페이스 (토픽, 타이머, 파라미터)
//
// [실행 의존 순서]
//   bringup → ekf_node → slam_toolbox → map_ekf_node → mpc_node
//   이유: /map_ekf/odom이 먼저 발행되어야 MPC 입력이 들어옴
//
// [제어 주기]
//   50Hz (20ms) 타이머로 구동
//   타이머 콜백에서 매번 QP를 풀고 /cmd_vel 발행
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <amr_msgs/msg/control_latency.hpp>
#include <amr_msgs/msg/localization_status.hpp>

#include "control_mpc/mpc_core.hpp"

namespace control_mpc
{

class MpcNode : public rclcpp::Node
{
public:
  explicit MpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // /map_ekf/odom 콜백 (50Hz)
  //   map frame 기준 현재 로봇 상태를 수신해서
  //   x0_ (현재 상태 벡터)를 갱신함.
  //
  //   [왜 /ekf/odom 아닌 /map_ekf/odom 쓰나?]
  //     /ekf/odom  : odom frame 기준 → 드리프트 누적
  //     /map_ekf/odom: map frame 기준 → LiDAR fusion으로 보정됨
  //     MPC reference waypoints도 map frame으로 정의하므로
  //     반드시 map frame 기준 상태를 써야 함
  // --------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --------------------------------------------------------
  // localization 상태 콜백 (10Hz)
  //   DEGRADED 상태에서는 MPC 속도 제한을 강화함.
  //   LOST 상태에서는 MPC를 중단하고 정지 명령 발행.
  //   → W10 Fail-safe와의 연계 준비
  // --------------------------------------------------------
  void locStatusCallback(
    const amr_msgs::msg::LocalizationStatus::SharedPtr msg);

  // --------------------------------------------------------
  // MPC 제어 루프 타이머 콜백 (50Hz = 20ms)
  //   매 주기마다:
  //   1. 현재 상태(x0_) 확인
  //   2. reference trajectory 생성
  //   3. mpc_core_.solve() 호출
  //   4. u0 → v, ω로 변환 후 /cmd_vel 발행
  //   5. solve time /metrics/control_latency_ms 발행
  // --------------------------------------------------------
  void controlCallback();

  // --------------------------------------------------------
  // reference trajectory 생성
  //   waypoints_ 리스트에서 현재 위치 기준으로
  //   N+1개의 목표 상태를 뽑아서 반환함.
  //
  //   [방식]
  //     가장 가까운 waypoint 탐색 →
  //     그 이후 N개를 순서대로 추출.
  //     waypoint가 부족하면 마지막 waypoint 반복.
  //
  //   x0: 현재 상태 (closest waypoint 탐색에 사용)
  //   반환: N+1개의 StateVec (x_ref[0] ~ x_ref[N])
  // --------------------------------------------------------
  std::vector<StateVec> generateReference(const StateVec & x0);

  // --------------------------------------------------------
  // waypoint 초기화
  //   3가지 경로 중 하나를 파라미터로 선택:
  //     "straight" : 직선 경로 (5m)
  //     "circle"   : 원형 경로 (반지름 1.5m)
  //     "figure8"  : 8자 경로
  //   map frame 기준으로 정의됨
  // --------------------------------------------------------
  void initWaypoints();

  // --------------------------------------------------------
  // 현재 상태에서 가장 가까운 waypoint 인덱스 탐색
  // --------------------------------------------------------
  int findClosestWaypoint(const StateVec & x0);

  // --------------------------------------------------------
  // /cmd_vel 발행 헬퍼
  //   v, ω → geometry_msgs::Twist 변환 후 발행
  //   Diff-Drive이므로 linear.x = v, angular.z = ω 만 사용
  // --------------------------------------------------------
  void publishCmdVel(double v, double omega);

  // --------------------------------------------------------
  // 정지 명령 발행
  //   localization LOST 또는 비상 정지 시 v=0, ω=0 발행
  // --------------------------------------------------------
  void publishStop();

  // --------------------------------------------------------
  // MPC 파라미터 로드
  //   ROS2 파라미터에서 읽어서 MpcParams 구조체로 변환
  // --------------------------------------------------------
  MpcParams loadMpcParams();

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
  rclcpp::Publisher<amr_msgs::msg::ControlLatency>::SharedPtr       latency_pub_;
  rclcpp::TimerBase::SharedPtr                                      control_timer_;

  // --- MPC 핵심 객체 ---
  MpcCore mpc_core_;

  // --- 현재 상태 ---
  StateVec x0_;              // 현재 로봇 상태 [x, y, θ, v, ω]
  bool     has_odom_ = false; // 첫 odom 수신 여부

  // --- 이전 입력 (입력 연속성 제약용) ---
  InputVec u_prev_;          // [v_prev, ω_prev] — Δu 계산에 사용

  // --- Reference waypoints ---
  // map frame 기준 목표 상태 시퀀스
  // StateVec = [x, y, θ, v, ω]
  std::vector<StateVec> waypoints_;
  int   closest_wp_idx_ = 0;     // 현재 가장 가까운 waypoint 인덱스
  bool  mission_done_   = false;  // 마지막 waypoint 도달 여부

  // --- Localization 상태 캐시 ---
  uint8_t loc_status_ = 0;   // 0=NORMAL, 1=DEGRADED, 2=LOST

  // --- 파라미터 ---
  std::string trajectory_type_;  // "straight" / "circle" / "figure8"
  double      goal_tolerance_;   // waypoint 도달 판정 거리 [m]
  double      delay_ms_;         // 제어 지연 주입용 파라미터 [ms]
  // W7: 바퀴 슬립 외란 주입용 파라미터 (0.0 ~ 1.0)
  // 예: 0.3이면 명령한 동력의 30%가 헛바퀴로 날아감을 의미
  double      slip_ratio_;       

  // W7: 진짜 제어 지연 주입을 위한 큐
  std::deque<geometry_msgs::msg::Twist> delay_queue_;

  // MPC 파라미터 (ROS2 파라미터에서 로드)
  MpcParams mpc_params_;

  // --- latency 통계 ---
  // 99th percentile 계산을 위해 최근 solve time을 저장
  std::vector<double> latency_history_;

  double prev_raw_theta_    = 0.0;   // 직전 raw atan2 yaw 값 (래핑 delta 계산용)
  double continuous_theta_  = 0.0;   // 누적 연속 yaw (래핑 없는 절대값)
};

}  // namespace control_mpc

#endif  // CONTROL_MPC__MPC_NODE_HPP_