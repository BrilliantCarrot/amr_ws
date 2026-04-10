#include "safety/state_machine_node.hpp"

namespace safety
{

// ============================================================
// 생성자
// ============================================================
StateMachineNode::StateMachineNode(const rclcpp::NodeOptions & options)
: Node("state_machine_node", options)
{
  // --------------------------------------------------------
  // 파라미터 선언
  //
  //   recovery_hold_sec:
  //     SAFE_STOP 진입 후 이 시간이 지나야 NORMAL 복귀 가능.
  //     localization이 순간 복구됐다 다시 나빠지는 경우를 걸러내는 히스테리시스.
  //
  //   [Step2 변경] cmd_timeout_sec 파라미터 제거.
  //     cmd_vel timeout 감시는 watchdog_node로 이관.
  // --------------------------------------------------------
  this->declare_parameter<double>("recovery_hold_sec", 2.0);
  recovery_hold_sec_ = this->get_parameter("recovery_hold_sec").as_double();

  auto reliable_qos = rclcpp::QoS(10).reliable();

  // --------------------------------------------------------
  // 구독자 설정
  // --------------------------------------------------------
  // /localization/status: NORMAL/DEGRADED/LOST 상태 전환의 핵심 입력
  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", reliable_qos,
    std::bind(&StateMachineNode::locStatusCallback, this, std::placeholders::_1));

  // [Step2 신규] /safety/watchdog_event: watchdog_node가 감지한 이상 조건 수신
  //   LOC_JUMP_CRITICAL / EKF_DIVERGED → LOC_LOST 트리거 유발
  //   CMD_TIMEOUT                       → CMD_TIMEOUT 트리거 유발
  watchdog_sub_ = this->create_subscription<amr_msgs::msg::WatchdogEvent>(
    "/safety/watchdog_event", reliable_qos,
    std::bind(&StateMachineNode::watchdogEventCallback, this, std::placeholders::_1));

  // /safety/manual_override: 수동 모드 진입/해제
  manual_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety/manual_override", reliable_qos,
    std::bind(&StateMachineNode::manualOverrideCallback, this, std::placeholders::_1));

  // --------------------------------------------------------
  // 발행자 설정
  // --------------------------------------------------------
  state_pub_ = this->create_publisher<amr_msgs::msg::SafetyStatus>(
    "/safety/state", reliable_qos);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", reliable_qos);

  // 10Hz 메인 타이머
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StateMachineNode::timerCallback, this));

  state_entry_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "[StateMachine] 초기화 완료. recovery_hold=%.1fs",
    recovery_hold_sec_);
}

// ============================================================
// locStatusCallback() — /localization/status 수신
// ============================================================
void StateMachineNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  loc_status_     = msg->status;
  loc_tf_jump_m_  = msg->tf_jump_m;
  has_loc_status_ = true;
}

// ============================================================
// watchdogEventCallback() — /safety/watchdog_event 수신
//
//   watchdog_node가 감지한 이상 조건을 TransitionTrigger로 변환해서
//   pending_trigger_에 임시 저장.
//   timerCallback의 evaluateTriggers()에서 소비됨.
//
//   [이벤트 → 트리거 매핑]
//     LOC_JUMP_CRITICAL : 심각한 위치 점프 → 즉시 SAFE_STOP (LOC_LOST 재활용)
//     EKF_DIVERGED      : EKF 발산 → 즉시 SAFE_STOP (LOC_LOST 재활용)
//     CMD_TIMEOUT       : 제어 루프 단절 → SAFE_STOP (CMD_TIMEOUT)
//
//   [왜 pending_trigger_ 패턴인가?]
//     콜백(watchdogEventCallback)과 타이머(timerCallback)는
//     SingleThreadedExecutor에서 순차 실행됨.
//     콜백에서 직접 transition()을 호출해도 되지만,
//     evaluateTriggers()의 우선순위 로직을 우회하게 됨.
//     pending_trigger_를 통해 다음 타이머 주기에서 정규 경로로 처리.
// ============================================================
void StateMachineNode::watchdogEventCallback(
  const amr_msgs::msg::WatchdogEvent::SharedPtr msg)
{
  TransitionTrigger t = TransitionTrigger::NONE;

  switch (msg->event_type) {
    // 심각한 localization 이상 → LOC_LOST 트리거로 매핑 (즉시 SAFE_STOP)
    case amr_msgs::msg::WatchdogEvent::LOC_JUMP_CRITICAL:
    case amr_msgs::msg::WatchdogEvent::EKF_DIVERGED:
      t = TransitionTrigger::LOC_LOST;
      break;

    // 제어 루프 단절
    case amr_msgs::msg::WatchdogEvent::CMD_TIMEOUT:
      t = TransitionTrigger::CMD_TIMEOUT;
      break;

    default:
      RCLCPP_WARN(this->get_logger(),
        "[StateMachine] 알 수 없는 watchdog 이벤트 타입: %d", msg->event_type);
      return;
  }

  // pending_trigger_ 갱신: 이미 다른 트리거가 대기 중이면 덮어씀.
  // 우선순위가 더 높은 트리거(LOC_LOST > CMD_TIMEOUT)가 나중에 오면 덮어쓰는 게 안전함.
  // evaluateTriggers에서 MANUAL_OVERRIDE 다음에 바로 소비되므로 덮어씀 위험 낮음.
  pending_trigger_ = t;

  RCLCPP_WARN(this->get_logger(),
    "[StateMachine] Watchdog 이벤트 수신: %s → 트리거: %s (값=%.3f / 임계=%.3f)",
    msg->event_name.c_str(),
    triggerName(t).c_str(),
    msg->value,
    msg->threshold);
}

// ============================================================
// manualOverrideCallback()
// ============================================================
void StateMachineNode::manualOverrideCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  manual_override_request_ = msg->data;
  RCLCPP_INFO(this->get_logger(),
    "[StateMachine] manual_override 요청: %s",
    manual_override_request_ ? "ON" : "OFF");
}

// ============================================================
// timerCallback() — 10Hz 메인 루프
// ============================================================
void StateMachineNode::timerCallback()
{
  const TransitionTrigger trigger = evaluateTriggers();
  transition(trigger);
  publishSafetyState();
}

// ============================================================
// evaluateTriggers() — 트리거 결정
//
// [우선순위]
//   1. MANUAL_OVERRIDE_ON/OFF   (수동 모드)
//   2. pending_trigger_         (watchdog 이벤트: LOC_LOST, CMD_TIMEOUT)
//   3. LOC_LOST                 (/localization/status 직접 판정)
//   4. LOC_DEGRADED             (/localization/status 직접 판정)
//   5. RECOVERY_OK              (재획득 성공)
//
// [Step2 변경]
//   - cmd_vel 직접 판정 로직 제거 (watchdog_node에 이관)
//   - pending_trigger_ 체크 추가 (우선순위 2)
// ============================================================
TransitionTrigger StateMachineNode::evaluateTriggers()
{
  // 1. 수동 모드 진입 요청
  if (manual_override_request_ &&
      current_state_ != SafetyState::MANUAL_OVERRIDE) {
    return TransitionTrigger::MANUAL_OVERRIDE_ON;
  }

  // 2. 수동 모드 해제 요청
  if (!manual_override_request_ &&
      current_state_ == SafetyState::MANUAL_OVERRIDE) {
    return TransitionTrigger::MANUAL_OVERRIDE_OFF;
  }

  // MANUAL_OVERRIDE 중에는 나머지 조건 무시
  if (current_state_ == SafetyState::MANUAL_OVERRIDE) {
    return TransitionTrigger::NONE;
  }

  // 3. watchdog 이벤트 처리 (pending_trigger_ 소비)
  //   watchdogEventCallback이 세팅한 트리거를 여기서 소비.
  //   소비 후 NONE으로 초기화해서 중복 처리 방지.
  if (pending_trigger_ != TransitionTrigger::NONE) {
    const TransitionTrigger t = pending_trigger_;
    pending_trigger_ = TransitionTrigger::NONE;  // 소비 완료
    return t;
  }

  // 4. localization LOST (/localization/status 직접 판정)
  if (has_loc_status_ && loc_status_ == 2) {
    return TransitionTrigger::LOC_LOST;
  }

  // 5. localization DEGRADED
  if (has_loc_status_ && loc_status_ == 1) {
    return TransitionTrigger::LOC_DEGRADED;
  }

  // 6. 재획득 성공
  //   DEGRADED → 즉시 NORMAL 복귀 허용
  //   SAFE_STOP → recovery_hold_sec_ 경과 후 복귀
  if ((current_state_ == SafetyState::DEGRADED ||
       current_state_ == SafetyState::SAFE_STOP) &&
      has_loc_status_ && loc_status_ == 0)
  {
    if (current_state_ == SafetyState::DEGRADED) {
      return TransitionTrigger::RECOVERY_OK;
    }
    const double time_in_state =
      (this->now() - state_entry_time_).seconds();
    if (time_in_state >= recovery_hold_sec_) {
      return TransitionTrigger::RECOVERY_OK;
    }
  }

  return TransitionTrigger::NONE;
}

// ============================================================
// transition() — 상태 전환 수행
// ============================================================
void StateMachineNode::transition(TransitionTrigger trigger)
{
  if (trigger == TransitionTrigger::NONE) {
    return;
  }

  SafetyState new_state = current_state_;

  switch (current_state_) {
    case SafetyState::NORMAL:
      if (trigger == TransitionTrigger::MANUAL_OVERRIDE_ON) {
        new_state = SafetyState::MANUAL_OVERRIDE;
      } else if (trigger == TransitionTrigger::LOC_LOST ||
                 trigger == TransitionTrigger::CMD_TIMEOUT) {
        new_state = SafetyState::SAFE_STOP;
      } else if (trigger == TransitionTrigger::LOC_DEGRADED) {
        new_state = SafetyState::DEGRADED;
      }
      break;

    case SafetyState::DEGRADED:
      if (trigger == TransitionTrigger::MANUAL_OVERRIDE_ON) {
        new_state = SafetyState::MANUAL_OVERRIDE;
      } else if (trigger == TransitionTrigger::LOC_LOST ||
                 trigger == TransitionTrigger::CMD_TIMEOUT) {
        new_state = SafetyState::SAFE_STOP;
      } else if (trigger == TransitionTrigger::RECOVERY_OK) {
        new_state = SafetyState::NORMAL;
      }
      break;

    case SafetyState::SAFE_STOP:
      if (trigger == TransitionTrigger::MANUAL_OVERRIDE_ON) {
        new_state = SafetyState::MANUAL_OVERRIDE;
      } else if (trigger == TransitionTrigger::RECOVERY_OK) {
        new_state = SafetyState::NORMAL;
      }
      break;

    case SafetyState::MANUAL_OVERRIDE:
      if (trigger == TransitionTrigger::MANUAL_OVERRIDE_OFF) {
        new_state = SafetyState::NORMAL;
      }
      break;
  }

  if (new_state != current_state_) {
    RCLCPP_INFO(this->get_logger(),
      "[StateMachine] %s → %s  (트리거: %s)",
      stateName(current_state_).c_str(),
      stateName(new_state).c_str(),
      triggerName(trigger).c_str());

    publishDiagnostics(trigger);

    current_state_     = new_state;
    state_entry_time_  = this->now();
    last_trigger_str_  = triggerName(trigger);
  }
}

// ============================================================
// publishSafetyState()
// ============================================================
void StateMachineNode::publishSafetyState()
{
  amr_msgs::msg::SafetyStatus msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = "base_link";
  msg.state           = static_cast<uint8_t>(current_state_);
  msg.state_name      = stateName(current_state_);
  msg.last_trigger    = last_trigger_str_;
  msg.time_in_state   = (this->now() - state_entry_time_).seconds();
  state_pub_->publish(msg);
}

// ============================================================
// publishDiagnostics()
// ============================================================
void StateMachineNode::publishDiagnostics(TransitionTrigger trigger)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name        = "safety/state_machine";
  status.hardware_id = "amr_robot";
  status.message =
    stateName(current_state_) + " --[" + triggerName(trigger) + "]--> next";

  switch (current_state_) {
    case SafetyState::NORMAL:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case SafetyState::DEGRADED:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case SafetyState::SAFE_STOP:
    case SafetyState::MANUAL_OVERRIDE:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
  }

  auto addKV = [&](const std::string & k, const std::string & v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key   = k;
    kv.value = v;
    status.values.push_back(kv);
  };

  addKV("trigger",     triggerName(trigger));
  addKV("from_state",  stateName(current_state_));
  addKV("loc_status",  std::to_string(static_cast<int>(loc_status_)));
  addKV("tf_jump_m",   std::to_string(loc_tf_jump_m_));

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

// ============================================================
// stateName()
// ============================================================
std::string StateMachineNode::stateName(SafetyState s) const
{
  switch (s) {
    case SafetyState::NORMAL:          return "NORMAL";
    case SafetyState::DEGRADED:        return "DEGRADED";
    case SafetyState::SAFE_STOP:       return "SAFE_STOP";
    case SafetyState::MANUAL_OVERRIDE: return "MANUAL_OVERRIDE";
    default:                           return "UNKNOWN";
  }
}

// ============================================================
// triggerName()
// ============================================================
std::string StateMachineNode::triggerName(TransitionTrigger t) const
{
  switch (t) {
    case TransitionTrigger::NONE:                return "NONE";
    case TransitionTrigger::LOC_DEGRADED:        return "LOC_DEGRADED";
    case TransitionTrigger::LOC_LOST:            return "LOC_LOST";
    case TransitionTrigger::CMD_TIMEOUT:         return "CMD_TIMEOUT";
    case TransitionTrigger::RECOVERY_OK:         return "RECOVERY_OK";
    case TransitionTrigger::MANUAL_OVERRIDE_ON:  return "MANUAL_OVERRIDE_ON";
    case TransitionTrigger::MANUAL_OVERRIDE_OFF: return "MANUAL_OVERRIDE_OFF";
    default:                                     return "UNKNOWN";
  }
}

}  // namespace safety

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}