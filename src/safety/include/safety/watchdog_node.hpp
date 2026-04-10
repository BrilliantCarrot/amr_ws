#ifndef SAFETY__WATCHDOG_NODE_HPP_
#define SAFETY__WATCHDOG_NODE_HPP_

// ============================================================
// watchdog_node.hpp — 이상 조건 감시 노드 헤더
//
// [역할]
//   state_machine_node가 처리하는 localization NORMAL/DEGRADED/LOST
//   상태 전환 위에, 더 심각하거나 독립적인 이상 조건을 추가로 감시함.
//   이상 감지 시 /safety/watchdog_event 토픽으로 이벤트를 발행하고
//   state_machine_node가 이를 구독해서 상태 전환에 반영함.
//
// [감시 조건 3가지]
//   ① LOC_JUMP_CRITICAL : tf_jump_m > 0.5m
//       localization_monitor는 0.3m에서 DEGRADED 판정.
//       Watchdog는 더 심각한 0.5m 점프를 감지 → 즉시 SAFE_STOP 유도.
//
//   ② EKF_DIVERGED : innovation_norm > 임계값 (기본 5.0)
//       마스터 플랜 기준: ekf_core::getInnovationNorm() 활용.
//       map_ekf_node가 /map_ekf/innovation_norm으로 발행한 값을 구독.
//       innovation이 폭증하면 EKF가 현실과 동떨어지고 있다는 신호.
//
//   ③ CMD_TIMEOUT : /cmd_vel 수신 없음 > cmd_timeout_sec_
//       MpcNode가 크래시/중단된 경우 감지.
//       /cmd_vel을 watchdog_node가 직접 구독해서 수신 시각 감시.
//
// [cooldown 설계]
//   동일 이벤트가 10Hz로 반복 발행되면 state_machine_node가 불필요하게
//   계속 상태 전환을 시도하게 됨.
//   각 이벤트마다 마지막 발행 시각을 추적해서 cooldown_sec_ 이내
//   재발행을 억제함 (기본 1.0s).
//
// [역할 분리 — localization_monitor_node와의 관계]
//   localization_monitor_node : TF jump 0.3m / TF timeout → NORMAL/DEGRADED/LOST
//   watchdog_node             : TF jump 0.5m / EKF diverge / cmd timeout → 이벤트
//   state_machine_node        : 두 소스를 조합해서 최종 상태 전환 결정
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <amr_msgs/msg/localization_status.hpp>
#include <amr_msgs/msg/watchdog_event.hpp>

namespace safety
{

class WatchdogNode : public rclcpp::Node
{
public:
  explicit WatchdogNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // 콜백
  // --------------------------------------------------------
  // /localization/status → tf_jump_m 캐시
  void locStatusCallback(
    const amr_msgs::msg::LocalizationStatus::SharedPtr msg);
  // /map_ekf/innovation_norm → innovation_norm_ 캐시
  void innovationNormCallback(
    const std_msgs::msg::Float64::SharedPtr msg);
  // /cmd_vel → 수신 시각만 기록
  void cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg);
  // 10Hz 감시 루프 — 조건 평가 → 이벤트 발행
  void timerCallback();

  // --------------------------------------------------------
  // 이벤트 발행 헬퍼
  //   cooldown 체크 후 /safety/watchdog_event 발행
  // --------------------------------------------------------
  void publishEvent(
    uint8_t           event_type,
    const std::string & event_name,
    double            value,
    double            threshold,
    const std::string & detail,
    rclcpp::Time    & last_event_time,   // cooldown 추적용 (참조)
    bool            & first_event);      // 첫 발행 여부 (참조)

  // --------------------------------------------------------
  // ROS2 인터페이스
  // --------------------------------------------------------
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             innov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          cmd_vel_sub_;
  rclcpp::Publisher<amr_msgs::msg::WatchdogEvent>::SharedPtr          event_pub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;

  // --------------------------------------------------------
  // 센서 캐시
  // --------------------------------------------------------
  double  loc_tf_jump_m_   = 0.0;
  bool    has_loc_status_  = false;

  double  innovation_norm_ = 0.0;
  bool    has_innovation_  = false;

  rclcpp::Time last_cmd_vel_time_;
  bool         has_cmd_vel_ = false;  // 첫 cmd_vel 수신 여부

  // --------------------------------------------------------
  // 파라미터
  // --------------------------------------------------------
  double jump_critical_m_;    // LOC_JUMP_CRITICAL 임계값 [m]   (기본: 0.5)
  double ekf_diverge_thresh_; // EKF_DIVERGED innovation 임계값 (기본: 5.0)
  double cmd_timeout_sec_;    // CMD_TIMEOUT 기준 [s]           (기본: 0.5)
  double cooldown_sec_;       // 동일 이벤트 재발행 최소 간격 [s] (기본: 1.0)

  // --------------------------------------------------------
  // cooldown 추적 — 이벤트 종류별로 마지막 발행 시각 및 첫 발행 여부
  // --------------------------------------------------------
  rclcpp::Time last_jump_event_time_;
  rclcpp::Time last_ekf_event_time_;
  rclcpp::Time last_cmd_event_time_;
  bool first_jump_event_ = true;
  bool first_ekf_event_  = true;
  bool first_cmd_event_  = true;
};

}  // namespace safety

#endif  // SAFETY__WATCHDOG_NODE_HPP_