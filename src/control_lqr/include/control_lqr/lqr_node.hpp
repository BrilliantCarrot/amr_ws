#ifndef CONTROL_LQR__LQR_NODE_HPP_
#define CONTROL_LQR__LQR_NODE_HPP_

// ============================================================
// lqr_node.hpp — TVLQR ROS2 노드 헤더
//
// [설계 패턴 — MpcNode와 동일]
//   LqrCore  : 순수 수학/알고리즘 (ROS 의존성 없음)
//   LqrNode  : ROS2 인터페이스 (토픽, 타이머, 파라미터)
//
// [MpcNode와의 주요 차이점]
//   1. mpc_core_.solve() 대신 lqr_core_.compute() 호출
//   2. OSQP 의존성 없음 (Eigen 행렬 연산만 사용)
//   3. reference는 N+1개가 아닌 현재 스텝 1개만 사용
//      (LQR은 단일 스텝 최적화 + Riccati로 infinite horizon 근사)
//   4. /metrics/lqr_control_latency_ms 별도 발행 (MPC와 동시 비교용)
//   5. /lqr/reference_pose 발행 (tracking_rmse_node 연동)
//
// [발행 토픽 — MPC와 동일 구조로 설계해 비교 실험 가능]
//   /cmd_vel                       : 제어 출력
//   /metrics/lqr_control_latency_ms: 제어 지연 (MpcNode의 /metrics/control_latency_ms와 비교)
//   /lqr/reference_pose            : 현재 목표 지점 (tracking_rmse_node 연동)
//   /metrics/lqr_min_obstacle_distance: 장애물 최소 거리
//
// [실행 의존 순서]
//   bringup → ekf_node → slam_toolbox → map_ekf_node
//   → path_planner_node → lqr_node
//   MPC와 LQR은 동시에 실행하지 않음 (같은 /cmd_vel 발행 충돌)
//
// [제어 주기]
//   50Hz (20ms) — MpcNode와 동일
// ============================================================

#include <deque>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <amr_msgs/msg/control_latency.hpp>
#include <amr_msgs/msg/localization_status.hpp>
#include <amr_msgs/msg/min_obstacle_distance.hpp>
#include <amr_msgs/msg/safety_status.hpp>

#include "control_lqr/lqr_core.hpp"

namespace control_lqr
{

class LqrNode : public rclcpp::Node
{
public:
  explicit LqrNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // /map_ekf/odom 콜백 (50Hz)
  //   MpcNode::odomCallback과 동일한 로직.
  //   continuous yaw 누적 처리 필수
  //   (atan2 정규화 금지 — yaw가 ±π 경계에서 튀면
  //    LQR 오차 계산 시 180° 오차 발생)
  // --------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // --------------------------------------------------------
  // /localization/status 콜백 (10Hz)
  // --------------------------------------------------------
  void locStatusCallback(
    const amr_msgs::msg::LocalizationStatus::SharedPtr msg);

  // --------------------------------------------------------
  // /safety/state 콜백
  //   SAFE_STOP(2) 시 즉시 정지 명령
  //   DEGRADED(1) 시 속도 50% 제한
  // --------------------------------------------------------
  void safetyStateCallback(
    const amr_msgs::msg::SafetyStatus::SharedPtr msg);

  // --------------------------------------------------------
  // /planned_path 콜백
  //   path_planner_node가 A*로 계산한 경로 수신.
  //   MpcNode::pathCallback과 동일한 로직으로 waypoints_ 교체.
  //   idx=0부터 순서 추종 (우회 경로 건너뜀 방지)
  // --------------------------------------------------------
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // --------------------------------------------------------
  // LQR 제어 루프 타이머 콜백 (50Hz = 20ms)
  //   매 주기:
  //   1. 상태 복사 (Data Copy-out)
  //   2. 경로 대기 확인
  //   3. 현재 waypoint 기준 reference 추출
  //   4. lqr_core_.compute() 호출
  //   5. /cmd_vel 발행
  //   6. latency 발행
  // --------------------------------------------------------
  void controlCallback();

  // --------------------------------------------------------
  // 현재 위치 기준 reference 상태 1개 반환
  //   MpcNode::generateReference와 달리 1개만 반환.
  //   (LQR은 단일 스텝 참조만 필요)
  //   거리 기반 lookahead로 waypoint 인덱싱
  // --------------------------------------------------------
  LqrStateVec generateReference(const LqrStateVec & x0);

  // --------------------------------------------------------
  // 현재 waypoint 진행 로직
  //   WP_REACH_DIST 이내 도달 시 다음 waypoint로 진행.
  //   순서 기반 진행 (nearest-neighbor 탐색 금지)
  //   이유: A* 우회 경로에서 후반부 점이 최근접으로 탐색되어
  //         우회 구간을 건너뛰는 문제 방지
  // --------------------------------------------------------
  int findClosestWaypoint(const LqrStateVec & x0);

  // --------------------------------------------------------
  // /cmd_vel 발행 헬퍼
  // --------------------------------------------------------
  void publishCmdVel(double v, double omega);

  // --------------------------------------------------------
  // 정지 명령 발행 (v=0, ω=0)
  // --------------------------------------------------------
  void publishStop();

  // --------------------------------------------------------
  // LQR 파라미터 로드 (ROS2 파라미터 → LqrParams)
  // --------------------------------------------------------
  LqrParams loadLqrParams();

  // 경로계획 없이 제어기 성능 검증용
  void initWaypoints();

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_;
  rclcpp::Subscription<amr_msgs::msg::SafetyStatus>::SharedPtr       safety_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr               path_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr              cmd_vel_pub_;
  rclcpp::Publisher<amr_msgs::msg::ControlLatency>::SharedPtr          latency_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr        ref_pose_pub_;
  rclcpp::Publisher<amr_msgs::msg::MinObstacleDistance>::SharedPtr     min_dist_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  bool use_global_planner_ = false;

  // 스레드 간 공유 자원 보호용 뮤텍스
  std::mutex state_mutex_;

  // --- LQR 핵심 객체 ---
  LqrCore lqr_core_;

  // --- 현재 상태 ---
  LqrStateVec x0_;
  bool        has_odom_    = false;
  rclcpp::Time last_odom_stamp_;

  // --- 이전 실제 속도 ([v, ω], Δu → u 변환에 사용) ---
  LqrInputVec u_prev_;

  // --- Reference waypoints ---
  std::vector<LqrStateVec> waypoints_;
  int  closest_wp_idx_ = 0;
  bool mission_done_   = false;

  // --- 안전 상태 캐시 ---
  uint8_t loc_status_    = 0;  // 0=NORMAL, 1=DEGRADED, 2=LOST
  uint8_t safety_state_  = 0;  // 0=NORMAL, 1=DEGRADED, 2=SAFE_STOP, 3=MANUAL_OVERRIDE

  // --- 파라미터 ---
  double v_ref_          = 0.1;   // 목표 선속도 [m/s]
  double goal_tolerance_ = 0.20;  // 목표 도달 판정 거리 [m]

  // --- continuous yaw 누적 (MpcNode와 동일 패턴) ---
  double prev_raw_theta_   = 0.0;  // 직전 raw atan2 yaw
  double continuous_theta_ = 0.0;  // 누적 연속 yaw

  // --- LQR 파라미터 ---
  LqrParams lqr_params_;

  // --- latency 통계 ---
  std::vector<double> latency_history_;
  std::vector<double> e2e_history_;

  // 거리 기반 lookahead 한 스텝당 이동 거리 하한값
  static constexpr double WP_REACH_DIST = 0.15;  // waypoint 도달 판정 거리 [m]

  std::string trajectory_type_ = "straight";  // "circle" / "figure8"

  int lap_count_ = 1;       // 목표 바퀴 수
  int current_lap_ = 0;     // 현재 바퀴 수
};

}  // namespace control_lqr

#endif  // CONTROL_LQR__LQR_NODE_HPP_
