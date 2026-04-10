#ifndef SAFETY__STATE_MACHINE_NODE_HPP_
#define SAFETY__STATE_MACHINE_NODE_HPP_

// ============================================================
// state_machine_node.hpp — Fail-safe 상태 머신 노드 헤더
//
// [W10 Step2 변경사항]
//   - /safety/watchdog_event 구독 추가 (watchdog_node 연계)
//   - /cmd_vel 직접 구독 제거 (cmd timeout 감시를 watchdog_node에 이관)
//   - pending_trigger_ 멤버 추가 (watchdog 이벤트 → 트리거 변환용)
//   - watchdogEventCallback() 추가
//
// [역할 분리 후 구조]
//   state_machine_node : /localization/status + /safety/watchdog_event → 상태 전환
//   watchdog_node      : 이상 조건 감지 → /safety/watchdog_event 발행
//   localization_monitor_node : TF 감시 → /localization/status 발행
//
// [상태 전환 다이어그램]
//
//   ┌─────────────┐  LOC_DEGRADED   ┌─────────────┐
//   │   NORMAL    │────────────────▶│  DEGRADED   │
//   │  (정상 운행) │◀────────────────│ (속도 50% ↓) │
//   └──────┬──────┘  RECOVERY_OK   └──────┬──────┘
//          │                              │
//          │ LOC_LOST / CMD_TIMEOUT       │ LOC_LOST / CMD_TIMEOUT
//          │ (watchdog 이벤트 포함)        │
//          ▼                              ▼
//   ┌─────────────────────────────────────────┐
//   │                SAFE_STOP               │
//   │              (완전 정지)                │
//   └──────────────────┬──────────────────────┘
//                      │ RECOVERY_OK
//                      ▼
//   ┌─────────────┐
//   │   NORMAL    │
//   └─────────────┘
//
//   Any → MANUAL_OVERRIDE_ON  → MANUAL_OVERRIDE
//   MANUAL_OVERRIDE → MANUAL_OVERRIDE_OFF → NORMAL
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <amr_msgs/msg/localization_status.hpp>
#include <amr_msgs/msg/safety_status.hpp>
#include <amr_msgs/msg/watchdog_event.hpp>

namespace safety
{

// ============================================================
// SafetyState — 4가지 상태
// ============================================================
enum class SafetyState : uint8_t {
  NORMAL          = 0,
  DEGRADED        = 1,
  SAFE_STOP       = 2,
  MANUAL_OVERRIDE = 3,
};

// ============================================================
// TransitionTrigger — 상태 전환 이벤트
// ============================================================
enum class TransitionTrigger : uint8_t {
  NONE                = 0,
  LOC_DEGRADED        = 1,
  LOC_LOST            = 2,
  CMD_TIMEOUT         = 3,
  RECOVERY_OK         = 4,
  MANUAL_OVERRIDE_ON  = 5,
  MANUAL_OVERRIDE_OFF = 6,
};

class StateMachineNode : public rclcpp::Node
{
public:
  explicit StateMachineNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // 콜백
  // --------------------------------------------------------
  void locStatusCallback(
    const amr_msgs::msg::LocalizationStatus::SharedPtr msg);
  // [Step2 신규] watchdog_node가 감지한 이벤트 수신
  void watchdogEventCallback(
    const amr_msgs::msg::WatchdogEvent::SharedPtr msg);
  void manualOverrideCallback(
    const std_msgs::msg::Bool::SharedPtr msg);
  void timerCallback();

  // --------------------------------------------------------
  // 상태 머신 핵심 로직
  // --------------------------------------------------------
  TransitionTrigger evaluateTriggers();
  void transition(TransitionTrigger trigger);

  // --------------------------------------------------------
  // 발행 헬퍼
  // --------------------------------------------------------
  void publishSafetyState();
  void publishDiagnostics(TransitionTrigger trigger);

  // --------------------------------------------------------
  // 이름 변환
  // --------------------------------------------------------
  std::string stateName(SafetyState s)         const;
  std::string triggerName(TransitionTrigger t)  const;

  // --------------------------------------------------------
  // ROS2 인터페이스
  // --------------------------------------------------------
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_;
  // [Step2 신규] watchdog_node 이벤트 구독
  rclcpp::Subscription<amr_msgs::msg::WatchdogEvent>::SharedPtr      watchdog_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                manual_sub_;
  rclcpp::Publisher<amr_msgs::msg::SafetyStatus>::SharedPtr           state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr                                        timer_;

  // --------------------------------------------------------
  // 상태 머신 변수
  // --------------------------------------------------------
  SafetyState  current_state_    = SafetyState::NORMAL;
  rclcpp::Time state_entry_time_;
  std::string  last_trigger_str_ = "NONE";

  // [Step2 신규] watchdog_node로부터 수신한 트리거를 임시 보관.
  //   watchdogEventCallback(비동기)이 세팅 → timerCallback이 소비.
  //   SingleThreadedExecutor에서는 순차 실행이라 mutex 불필요.
  //   (W11 MultiThreadedExecutor 전환 시 mutex 추가 예정)
  TransitionTrigger pending_trigger_ = TransitionTrigger::NONE;

  // --------------------------------------------------------
  // 센서 캐시
  // --------------------------------------------------------
  uint8_t  loc_status_     = 0;
  double   loc_tf_jump_m_  = 0.0;
  bool     has_loc_status_ = false;

  bool manual_override_request_ = false;

  // --------------------------------------------------------
  // 파라미터
  // --------------------------------------------------------
  double recovery_hold_sec_;  // SAFE_STOP 최소 유지 시간 [s] (기본: 2.0)
};

}  // namespace safety

#endif  // SAFETY__STATE_MACHINE_NODE_HPP_