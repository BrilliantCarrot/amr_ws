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
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <amr_msgs/msg/control_latency.hpp>
#include <amr_msgs/msg/localization_status.hpp> // 추후 삭제 가능
#include <amr_msgs/msg/safety_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>   // x_ref[0] → /mpc/reference_pose 발행용
#include <amr_msgs/msg/obstacle_array.hpp>
#include <nav_msgs/msg/path.hpp>               // 글로벌 경로계획 연동 (W12 추가)
#include "control_mpc/mpc_core.hpp"
#include "amr_msgs/msg/min_obstacle_distance.hpp" // w8 min clearance 발행용
#include "control_mpc/cbf_filter.hpp"
#include <memory>

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

  void safetyStateCallback(
  const amr_msgs::msg::SafetyStatus::SharedPtr msg);

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
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_; // 추후 삭제 가능
  rclcpp::Subscription<amr_msgs::msg::SafetyStatus>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_vel_pub_;
  rclcpp::Publisher<amr_msgs::msg::ControlLatency>::SharedPtr       latency_pub_;
  rclcpp::TimerBase::SharedPtr                                      control_timer_;

  // 센서 데이터 수신용 상호 배제 그룹 (odom, loc, safety, obs 등 데이터 업데이트 전용)
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  
  // 제어 타이머 전용 상호 배제 그룹 (MPC 최적화 연산 전용)
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  
  // 스레드 간 공유 자원(x0_, obstacles_, delay_queue_ 등)의 동시 접근을 막아주는 뮤텍스
  std::mutex state_mutex_;
  
  // W8 추가: tracking_rmse_node가 구독하는 현재 목표 지점 발행 publisher
  // x_ref[0] (map frame) → geometry_msgs/PoseStamped 형태로 발행
  // pose_rmse_node      : EKF 추정값 vs GT → "추정기가 얼마나 정확한가"
  // tracking_rmse_node  : MPC 목표 지점 vs GT → "제어기가 경로를 얼마나 잘 따라가는가"
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr         ref_pose_pub_;

  // W8: 최소 장애물 거리 발행 publisher
  rclcpp::Publisher<amr_msgs::msg::MinObstacleDistance>::SharedPtr min_dist_pub_;
  // W8: 장애물 목록 (map frame 기준, 생성자에서 설정)
  std::vector<Obstacle> obstacles_;

  // 클래스 private 멤버 영역에 추가
  rclcpp::Subscription<amr_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  void obsCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg);

  // --------------------------------------------------------
  // /planned_path 콜백 (W12 글로벌 경로계획 연동)
  //   path_planner_node가 발행한 nav_msgs/Path를 수신해
  //   waypoints_를 교체함. 기존 hardcoded 경로 대신 사용.
  // --------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  bool use_global_planner_ = false;  // true: /planned_path 대기, false: 하드코딩 경로

  // 긴급 정지용 — /cmd_vel_delayed 직접 발행 (mock_link 우회)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_stop_pub_;

  // --- MPC 핵심 객체 ---
  MpcCore mpc_core_;

  // --- 현재 상태 ---
  StateVec x0_;              // 현재 로봇 상태 [x, y, θ, v, ω]
  bool     has_odom_ = false; // 첫 odom 수신 여부
  rclcpp::Time last_odom_stamp_;          // W11 Step7: e2e latency 측정용
  std::vector<double> e2e_history_;       // e2e latency 히스토리

  // --- 이전 입력 (입력 연속성 제약용) ---
  InputVec u_prev_;          // [v_prev, ω_prev] — Δu 계산에 사용

  // --- Reference waypoints ---
  // map frame 기준 목표 상태 시퀀스
  // StateVec = [x, y, θ, v, ω]
  std::vector<StateVec> waypoints_;
  int   closest_wp_idx_ = 0;     // 현재 가장 가까운 waypoint 인덱스
  bool  mission_done_   = false;  // 마지막 waypoint 도달 여부
  // bool waypoints_rerouted_ = false;  // rerouteWaypointsAroundObstacles 중복 실행 방지

  // --- Localization 상태 캐시 ---
  uint8_t loc_status_ = 0;   // 0=NORMAL, 1=DEGRADED, 2=LOST, 추후 삭제 가능

  uint8_t safety_state_ = 0;  // /safety/state → 0=NORMAL, 1=DEGRADED, 2=SAFE_STOP, 3=MANUAL_OVERRIDE

  // --- 파라미터 ---
  std::string trajectory_type_;  // "straight" / "circle" / "figure8"
  double      goal_tolerance_;   // waypoint 도달 판정 거리 [m]
  double      delay_ms_;         // 제어 지연 주입용 파라미터 [ms]
  // W7: 바퀴 슬립 외란 주입용 파라미터 (0.0 ~ 1.0)
  // 예: 0.3이면 명령한 동력의 30%가 헛바퀴로 날아감을 의미
  double      slip_ratio_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  double artificial_load_ms_ = 0.0;
  double v_ref_ = 0.1;

  // W7: 진짜 제어 지연 주입을 위한 큐
  std::deque<geometry_msgs::msg::Twist> delay_queue_;

  // MPC 파라미터 (ROS2 파라미터에서 로드)
  MpcParams mpc_params_;

  // --- latency 통계 ---
  // 99th percentile 계산을 위해 최근 solve time을 저장
  std::vector<double> latency_history_;

  // // W9: 정적 장애물 회피를 위한 x_ref 측방 이동
  // void shiftReferenceAroundObstacles(std::vector<StateVec> & x_ref);

  // void rerouteWaypointsAroundObstacles();

  double prev_raw_theta_    = 0.0;   // 직전 raw atan2 yaw 값 (래핑 delta 계산용)
  double continuous_theta_  = 0.0;   // 누적 연속 yaw (래핑 없는 절대값)
  // ── CBF Safety Filter ──────────────────────────────────────
  // MPC 출력 u_nom → CBF-QP → u_safe 파이프라인
  std::unique_ptr<CbfFilter> cbf_filter_;

  // CBF 파라미터 (생성자에서 로드)
  double cbf_gamma_   = 1.0;    // class-K 계수
  double cbf_slack_p_ = 500.0;  // soft slack 페널티
  double cbf_d_safe_  = 0.0;    // 추가 안전 마진 [m]
  bool   cbf_enabled_ = true;   // CBF 활성화 여부 (런타임 토글용)

  // ──────────────────────────────────────────────────────────
  // 도킹 관련 멤버
  //
  //   미션 단계:
  //     NAVIGATING → A* waypoint 추종 중
  //     DOCKING    → 마지막 waypoint 도달 후 dock 포즈 정밀 정렬
  //     DOCKED     → 완료 판정 통과, 정지 유지
  //
  //   dock_x_, dock_y_: NAVIGATING→DOCKING 전환 시
  //                     waypoints_.back() 위치로 자동 갱신
  //   dock_yaw_       : 파라미터 "dock_yaw" [rad]으로 외부 설정
  // ──────────────────────────────────────────────────────────
  enum class MissionPhase : uint8_t {
    NAVIGATING = 0,  // A* 경로 추종 중
    DOCKING    = 1,  // dock 포즈 수렴 중
    DOCKED     = 2   // 도킹 완료, 정지 유지
  };
  MissionPhase mission_phase_ = MissionPhase::NAVIGATING;

  // 도킹 목표 포즈 (map frame)
  double dock_x_   = 0.0;  // 마지막 waypoint x (자동 설정)
  double dock_y_   = 0.0;  // 마지막 waypoint y (자동 설정)
  double dock_yaw_ = 0.0;  // 도킹 목표 헤딩 [rad] (파라미터)

  // 도킹 완료 판정 임계값
  double dock_pos_tol_ = 0.0;  // 위치 허용 오차 [m]  (파라미터 "dock_pos_tol")
  double dock_yaw_tol_ = 0.0;  // 헤딩 허용 오차 [rad] (파라미터 "dock_yaw_tol_deg")

  // 도킹 전용 reference 생성
  // N+1개를 모두 dock 포즈로 채워 MPC가 수렴하도록 유도
  std::vector<StateVec> generateDockingReference(const StateVec & x0);
};

}  // namespace control_mpc

#endif  // CONTROL_MPC__MPC_NODE_HPP_