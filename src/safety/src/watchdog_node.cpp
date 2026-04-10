#include "safety/watchdog_node.hpp"

namespace safety
{

// ============================================================
// 생성자
// ============================================================
WatchdogNode::WatchdogNode(const rclcpp::NodeOptions & options)
: Node("watchdog_node", options)
{
  // --------------------------------------------------------
  // 파라미터 선언
  //
  //   jump_critical_m:
  //     localization_monitor의 DEGRADED 임계값(0.3m)보다 큰 값.
  //     이 크기를 넘는 점프는 slam_toolbox가 크게 보정했다는 신호로
  //     단순 DEGRADED가 아니라 즉시 SAFE_STOP이 필요한 수준.
  //
  //   ekf_diverge_thresh:
  //     innovation_norm 임계값. innovation = 관측값 - 예측값.
  //     이 값이 폭증하면 EKF 상태가 현실과 크게 어긋남을 의미.
  //     5.0은 경험적 기본값. 실제 운용 환경에서 정상 구간을 먼저
  //     측정한 뒤 그 3~5배로 설정하는 것을 권장.
  //
  //   cmd_timeout_sec:
//     mock_link_node 출력인 /cmd_vel_delayed를 감시.
//     드롭이 Gazebo까지 전달되는 경로를 감시해야 실제 통신 단절 감지 가능.
  //
  //   cooldown_sec:
  //     동일 이벤트 재발행 억제 시간.
  //     state_machine_node가 이미 상태를 전환했을 시간적 여유를 주어
  //     이벤트 폭주를 방지함.
  // --------------------------------------------------------
  this->declare_parameter<double>("jump_critical_m",    0.5);
  this->declare_parameter<double>("ekf_diverge_thresh", 5.0);
  this->declare_parameter<double>("cmd_timeout_sec",    0.5);
  this->declare_parameter<double>("cooldown_sec",       1.0);

  jump_critical_m_    = this->get_parameter("jump_critical_m").as_double();
  ekf_diverge_thresh_ = this->get_parameter("ekf_diverge_thresh").as_double();
  cmd_timeout_sec_    = this->get_parameter("cmd_timeout_sec").as_double();
  cooldown_sec_       = this->get_parameter("cooldown_sec").as_double();

  auto reliable_qos = rclcpp::QoS(10).reliable();

  // --------------------------------------------------------
  // 구독자 설정
  // --------------------------------------------------------
  // /localization/status: tf_jump_m 필드만 사용
  // (status 필드는 state_machine_node가 직접 처리)
  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", reliable_qos,
    std::bind(&WatchdogNode::locStatusCallback, this, std::placeholders::_1));

  // /map_ekf/innovation_norm: map_ekf_node가 50Hz로 발행하는 EKF innovation norm
  // W10에서 map_ekf_node에 추가한 토픽
  innov_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/map_ekf/innovation_norm", reliable_qos,
    std::bind(&WatchdogNode::innovationNormCallback, this, std::placeholders::_1));

  // /cmd_vel_delayed: mock_link_node 출력을 감시
  //   /cmd_vel이 아닌 /cmd_vel_delayed를 구독해야
  //   mock_link의 드롭이 실제로 CMD_TIMEOUT에 반영됨
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_delayed", reliable_qos,
    std::bind(&WatchdogNode::cmdVelCallback, this, std::placeholders::_1));

  // --------------------------------------------------------
  // 발행자 설정
  // --------------------------------------------------------
  // /safety/watchdog_event: state_machine_node가 구독해서 트리거로 반영
  event_pub_ = this->create_publisher<amr_msgs::msg::WatchdogEvent>(
    "/safety/watchdog_event", reliable_qos);

  // --------------------------------------------------------
  // 10Hz 감시 타이머
  //   localization_monitor_node와 동일 주기(10Hz)로 동기화.
  //   tf_jump_m은 10Hz로 갱신되므로, 더 빠른 주기는 의미가 없음.
  // --------------------------------------------------------
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WatchdogNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[Watchdog] 초기화 완료. "
    "jump_critical=%.2fm, ekf_thresh=%.1f, cmd_timeout=%.1fs, cooldown=%.1fs",
    jump_critical_m_, ekf_diverge_thresh_, cmd_timeout_sec_, cooldown_sec_);
}

// ============================================================
// locStatusCallback() — /localization/status 수신
//   tf_jump_m만 캐시. status 필드는 state_machine_node가 처리.
// ============================================================
void WatchdogNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr msg)
{
  loc_tf_jump_m_  = msg->tf_jump_m;
  has_loc_status_ = true;
}

// ============================================================
// innovationNormCallback() — /map_ekf/innovation_norm 수신
//   map_ekf_node의 publishMapEkfOdom()에서 50Hz로 발행되는 값.
//   EkfCore::getInnovationNorm() 값 (odom update 직후 기준).
// ============================================================
void WatchdogNode::innovationNormCallback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  innovation_norm_ = msg->data;
  has_innovation_  = true;
}

// ============================================================
// cmdVelCallback() — /cmd_vel 수신
//   내용은 안 보고 수신 시각만 기록.
//   timerCallback에서 경과 시간으로 timeout 판정.
// ============================================================
void WatchdogNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr /*msg*/)
{
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_       = true;
}

// ============================================================
// timerCallback() — 10Hz 감시 루프
//
//   [감시 순서 및 이유]
//   ① LOC_JUMP_CRITICAL 먼저: localization 점프는 즉각적이고
//      명확한 이상 신호라 가장 먼저 감지해야 함.
//   ② EKF_DIVERGED: innovation 폭증은 EKF 내부 문제로 localization
//      점프보다는 느리게 나타나지만 위험도가 높음.
//   ③ CMD_TIMEOUT: 제어 루프 단절 감지.
// ============================================================
void WatchdogNode::timerCallback()
{
  // ---- ① LOC_JUMP_CRITICAL 감시 ----
  // tf_jump_m: 직전 모니터링 스텝(100ms) 대비 로봇 위치 변화량 [m]
  // localization_monitor_node의 DEGRADED 기준(0.3m)보다 심각한 수준 감지
  if (has_loc_status_ && loc_tf_jump_m_ > jump_critical_m_) {
    publishEvent(
      amr_msgs::msg::WatchdogEvent::LOC_JUMP_CRITICAL,
      "LOC_JUMP_CRITICAL",
      loc_tf_jump_m_,
      jump_critical_m_,
      "localization TF 점프가 critical 임계값을 초과함",
      last_jump_event_time_,
      first_jump_event_
    );
  }

  // ---- ② EKF_DIVERGED 감시 ----
  // innovation_norm: ||y - H*x_pred||, 관측값과 EKF 예측의 차이 크기.
  // 이 값이 임계값 이상이면 EKF가 현실과 크게 어긋나고 있다는 신호.
  // has_innovation_: map_ekf_node가 아직 뜨지 않았을 때 오판 방지.
  if (has_innovation_ && innovation_norm_ > ekf_diverge_thresh_) {
    publishEvent(
      amr_msgs::msg::WatchdogEvent::EKF_DIVERGED,
      "EKF_DIVERGED",
      innovation_norm_,
      ekf_diverge_thresh_,
      "EKF innovation norm이 임계값을 초과 — EKF 발산 의심",
      last_ekf_event_time_,
      first_ekf_event_
    );
  }

  // ---- ③ CMD_TIMEOUT 감시 ----
  // has_cmd_vel_: 시스템 기동 초기에 MpcNode가 아직 발행 전인 경우를
  // timeout으로 잘못 판정하지 않도록 첫 수신 여부 확인.
  if (has_cmd_vel_) {
    const double elapsed = (this->now() - last_cmd_vel_time_).seconds();
    if (elapsed > cmd_timeout_sec_) {
      publishEvent(
        amr_msgs::msg::WatchdogEvent::CMD_TIMEOUT,
        "CMD_TIMEOUT",
        elapsed,
        cmd_timeout_sec_,
        "/cmd_vel_delayed 수신 없음 — mock_link 완전 차단 또는 MpcNode 크래시 의심",
        last_cmd_event_time_,
        first_cmd_event_
      );
    }
  }
}

// ============================================================
// publishEvent() — 이벤트 발행 (cooldown 포함)
//
//   [cooldown 동작 원리]
//     상태 전환이 이미 일어난 후에도 이상 조건이 지속되면
//     같은 이벤트가 10Hz로 계속 발행됨. state_machine_node는
//     이미 SAFE_STOP 상태라 추가 전환 효과는 없지만 불필요한
//     로그와 토픽 발행이 쌓임.
//     cooldown_sec_ 이내 재발행을 억제해서 이를 방지함.
//
//   last_event_time / first_event: 이벤트 종류별로 별도 관리.
//   참조(reference)로 받아서 함수 안에서 갱신.
// ============================================================
void WatchdogNode::publishEvent(
  uint8_t           event_type,
  const std::string & event_name,
  double            value,
  double            threshold,
  const std::string & detail,
  rclcpp::Time    & last_event_time,
  bool            & first_event)
{
  const rclcpp::Time now = this->now();

  // cooldown 체크
  if (!first_event) {
    const double since_last = (now - last_event_time).seconds();
    if (since_last < cooldown_sec_) {
      return;  // cooldown 중 → 발행 억제
    }
  }

  // 이벤트 발행
  amr_msgs::msg::WatchdogEvent msg;
  msg.header.stamp = now;
  msg.header.frame_id = "";
  msg.event_type  = event_type;
  msg.event_name  = event_name;
  msg.value       = value;
  msg.threshold   = threshold;
  msg.detail      = detail;
  event_pub_->publish(msg);

  // cooldown 시각 갱신
  last_event_time = now;
  first_event     = false;

  RCLCPP_WARN(this->get_logger(),
    "[Watchdog] 이벤트 발행: %s | 값=%.3f / 임계=%.3f | %s",
    event_name.c_str(), value, threshold, detail.c_str());
}

}  // namespace safety

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety::WatchdogNode>());
  rclcpp::shutdown();
  return 0;
}