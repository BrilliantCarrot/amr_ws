#include "safety/mock_link_node.hpp"

#include <sstream>
#include <iomanip>

namespace safety
{

// ============================================================
// 생성자
// ============================================================
MockLinkNode::MockLinkNode(const rclcpp::NodeOptions & options)
: Node("mock_link_node", options),
  rng_(std::random_device{}()),   // 하드웨어 엔트로피로 시드 초기화
  prob_dist_(0.0, 1.0)            // [0.0, 1.0) 균일 분포
{
  // --------------------------------------------------------
  // 파라미터 선언 및 로드
  //
  //   enabled:
  //     false면 /cmd_vel을 그대로 /cmd_vel_delayed로 투명 전달.
  //     실험 중 mock_link 기능만 빠르게 off하고 싶을 때 사용.
  //
  //   delay_ms:
  //     모든 패킷에 동일하게 적용되는 고정 전파 지연.
  //     실제 무선 네트워크에서 발생하는 propagation delay 모사.
  //     예) 100ms → MPC가 계산한 명령이 100ms 뒤에 로봇에 전달됨.
  //
  //   drop_rate:
  //     패킷 단위 독립 Bernoulli 드롭 확률.
  //     각 패킷마다 독립적으로 판정 → 랜덤 패킷 손실 모사.
  //     예) 0.3 → 평균 30%의 명령이 로봇에 도달하지 않음.
  //
  //   burst_loss_prob / burst_loss_len:
  //     연속 패킷 손실(burst loss) 모사.
  //     burst_loss_prob 확률로 burst 시작 → 연속 burst_loss_len개 드롭.
  //     실제 무선 환경에서 장애물 통과나 간섭 구간에서 발생하는 패턴.
  //
  //   reorder_prob / reorder_max_delay_ms:
  //     패킷 순서 뒤바꿈 모사.
  //     reorder_prob 확률로 해당 패킷에 추가 랜덤 지연 부여.
  //     priority_queue(최소 힙) 특성상 나중에 들어온 패킷이
  //     더 작은 release_time을 가지면 먼저 나가게 됨 → 자동으로 reorder.
  //     예) 현재 패킷에 30ms 추가 지연 → 다음 패킷(20ms 뒤 도착)이 먼저 발행.
  // --------------------------------------------------------
  this->declare_parameter<bool>("enabled",              true);
  this->declare_parameter<double>("delay_ms",           0.0);
  this->declare_parameter<double>("drop_rate",          0.0);
  this->declare_parameter<double>("burst_loss_prob",    0.0);
  this->declare_parameter<int>("burst_loss_len",        3);
  this->declare_parameter<double>("reorder_prob",       0.0);
  this->declare_parameter<double>("reorder_max_delay_ms", 50.0);

  enabled_              = this->get_parameter("enabled").as_bool();
  delay_ms_             = this->get_parameter("delay_ms").as_double();
  drop_rate_            = this->get_parameter("drop_rate").as_double();
  burst_loss_prob_      = this->get_parameter("burst_loss_prob").as_double();
  burst_loss_len_       = this->get_parameter("burst_loss_len").as_int();
  reorder_prob_         = this->get_parameter("reorder_prob").as_double();
  reorder_max_delay_ms_ = this->get_parameter("reorder_max_delay_ms").as_double();

  // --------------------------------------------------------
  // 런타임 파라미터 변경 콜백
  //   ros2 param set으로 변경 시 즉시 멤버변수에 반영.
  //   이 콜백이 없으면 생성자에서 읽은 값이 고정되어
  //   ros2 param set이 적용되지 않음.
  // --------------------------------------------------------
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      for (const auto & p : params) {
        if (p.get_name() == "enabled")
          enabled_ = p.as_bool();
        else if (p.get_name() == "delay_ms")
          delay_ms_ = p.as_double();
        else if (p.get_name() == "drop_rate")
          drop_rate_ = p.as_double();
        else if (p.get_name() == "burst_loss_prob")
          burst_loss_prob_ = p.as_double();
        else if (p.get_name() == "burst_loss_len")
          burst_loss_len_ = p.as_int();
        else if (p.get_name() == "reorder_prob")
          reorder_prob_ = p.as_double();
        else if (p.get_name() == "reorder_max_delay_ms")
          reorder_max_delay_ms_ = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  auto reliable_qos = rclcpp::QoS(20).reliable();

  // --------------------------------------------------------
  // 구독자: /cmd_vel (MpcNode가 발행)
  // --------------------------------------------------------
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", reliable_qos,
    std::bind(&MockLinkNode::cmdVelCallback, this, std::placeholders::_1));

  // --------------------------------------------------------
  // 발행자: /cmd_vel_delayed (Gazebo 브릿지가 구독)
  //   bringup.launch.py에서 브릿지 토픽을 이걸로 교체해야 함.
  // --------------------------------------------------------
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_delayed", reliable_qos);

  // 통계 토픽: String 형태로 사람이 읽기 쉽게 발행
  stats_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/mock_link/stats", rclcpp::QoS(10).reliable());

  // --------------------------------------------------------
  // 50Hz 릴레이 타이머
  //   delay 큐에서 만료된 패킷을 꺼내서 발행함.
  //   MpcNode 제어 주기(50Hz)와 동일하게 맞춤.
  //   더 빠르게(100Hz) 설정하면 latency 정밀도가 높아지지만
  //   W11 목적(통신 이상 주입 검증)에는 50Hz로 충분함.
  // --------------------------------------------------------
  relay_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),  // 50Hz
    std::bind(&MockLinkNode::timerCallback, this));

  // 10Hz 통계 타이머
  stats_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10Hz
    std::bind(&MockLinkNode::statsCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[MockLink] 초기화 완료.\n"
    "  enabled=%s | delay=%.0fms | drop=%.0f%% | "
    "burst_prob=%.0f%%(len=%d) | reorder=%.0f%%",
    enabled_ ? "true" : "false",
    delay_ms_,
    drop_rate_ * 100.0,
    burst_loss_prob_ * 100.0,
    burst_loss_len_,
    reorder_prob_ * 100.0);
}

// ============================================================
// cmdVelCallback() — /cmd_vel 수신
//
//   [처리 순서]
//   1. enabled=false → 즉시 /cmd_vel_delayed 투명 발행 후 리턴
//   2. shouldDrop() → true면 패킷 폐기 후 리턴
//   3. extraDelayMs() → reorder용 추가 지연 계산
//   4. release_time = now + delay_ms_ + extra_delay_ms 계산
//   5. DelayedPacket 생성 후 delay_queue_ 적재
// ============================================================
void MockLinkNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  ++stat_total_rx_;

  // ① enabled=false: 이상 주입 없이 투명 전달
  if (!enabled_) {
    cmd_pub_->publish(*msg);
    ++stat_total_tx_;
    return;
  }

  // ② 드롭 판정
  //   shouldDrop()이 true면 이 패킷을 버리고 리턴
  if (shouldDrop()) {
    ++stat_dropped_;

    // 드롭 시 zero velocity 발행 — Gazebo가 이전 명령을 유지하는 것을 방지
    // 실제 통신 드롭은 패킷이 사라지는 것이지만,
    // Gazebo 브릿지 특성상 마지막 값을 유지하므로
    // 드롭을 모사하려면 명시적으로 정지 명령을 보내야 함
    geometry_msgs::msg::Twist zero_msg;
    zero_msg.linear.x  = 0.0;
    zero_msg.angular.z = 0.0;
    cmd_pub_->publish(zero_msg);

    RCLCPP_DEBUG(this->get_logger(),
      "[MockLink] 패킷 드롭 (총 드롭: %lu / %lu)", stat_dropped_, stat_total_rx_);
    return;
  }

  // ③ reorder 추가 지연 계산
  //   0.0이면 reorder 없음 (정상 패킷)
  //   양수면 이 패킷만 추가 지연 → priority_queue에서 더 늦게 나옴
  const double extra_ms = extraDelayMs();
  if (extra_ms > 0.0) {
    ++stat_reordered_;
  }

  // ④ release_time 계산
  //   delay_ms_ : 기본 전파 지연
  //   extra_ms  : reorder용 추가 지연
  const double total_delay_ms = delay_ms_ + extra_ms;
  const rclcpp::Time release_time =
    this->now() + rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(total_delay_ms * 1e6));  // ms → ns

  // ⑤ delay 큐에 적재
  //   queue_mutex_: W11 Step4 MultiThreadedExecutor 전환 대비
  //   cmdVelCallback(센서 콜백 그룹)과 timerCallback(타이머 그룹)이
  //   병렬 실행될 때 delay_queue_ 데이터 레이스 방지용.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    delay_queue_.push(DelayedPacket{release_time, *msg});
  }
}

// ============================================================
// timerCallback() — 50Hz 릴레이 루프
//
//   delay_queue_ 최상단(가장 빠른 release_time) 패킷을
//   현재 시각과 비교해서 만료된 것을 꺼내 발행함.
//   priority_queue(최소 힙)이므로 top()이 항상 가장 빠른 패킷.
//
//   while 루프: 같은 주기 내에 만료된 패킷이 여러 개일 수 있음.
//   (예: delay_ms=0이고 여러 패킷이 동시에 들어온 경우)
// ============================================================
void MockLinkNode::timerCallback()
{
  const rclcpp::Time now = this->now();

  std::lock_guard<std::mutex> lock(queue_mutex_);

  // 큐가 비어있거나 아직 만료되지 않은 패킷만 남으면 종료
  while (!delay_queue_.empty()) {
    const DelayedPacket & top = delay_queue_.top();

    // release_time이 현재 시각보다 미래면 아직 발행 안 함
    if (top.release_time > now) {
      break;
    }

    // 만료된 패킷 발행
    cmd_pub_->publish(top.twist);
    ++stat_total_tx_;

    delay_queue_.pop();
  }
}

// ============================================================
// statsCallback() — 10Hz 통계 발행
//
//   누적 통계를 사람이 읽기 쉬운 String 형태로 발행.
//   /mock_link/stats를 rqt_topic_monitor로 실시간 확인 가능.
//
//   실제 drop_rate: 수신 패킷 중 드롭된 비율
//   queue_size: 현재 delay 큐에 대기 중인 패킷 수
//     (큰 값이면 delay가 쌓이고 있다는 신호)
// ============================================================
void MockLinkNode::statsCallback()
{
  const double actual_drop_rate =
    (stat_total_rx_ > 0)
    ? (static_cast<double>(stat_dropped_) / stat_total_rx_ * 100.0)
    : 0.0;

  size_t queue_size;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_size = delay_queue_.size();
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1)
      << "[MockLink Stats] "
      << "rx=" << stat_total_rx_
      << " tx=" << stat_total_tx_
      << " drop=" << stat_dropped_
      << "(" << actual_drop_rate << "%)"
      << " burst_drop=" << stat_burst_drop_
      << " reorder=" << stat_reordered_
      << " queue=" << queue_size
      << " | params: delay=" << delay_ms_ << "ms"
      << " drop=" << drop_rate_ * 100.0 << "%"
      << " burst=" << burst_loss_prob_ * 100.0 << "%(len=" << burst_loss_len_ << ")"
      << " reorder=" << reorder_prob_ * 100.0 << "%";

  std_msgs::msg::String stats_msg;
  stats_msg.data = oss.str();
  stats_pub_->publish(stats_msg);
}

// ============================================================
// shouldDrop() — 드롭 판정
//
//   [판정 순서]
//   1. burst_loss_remaining_ > 0: 현재 burst 진행 중 → 무조건 드롭
//      → burst_loss_remaining_ 감소
//   2. burst_loss_prob: 새 burst 시작 여부 판정
//      → 성공 시 burst_loss_remaining_ = burst_loss_len_ - 1
//         (이번 패킷이 burst 첫 번째 드롭이므로 -1)
//      → 무조건 드롭
//   3. drop_rate: 독립 Bernoulli 드롭 판정
//      → prob_dist_(rng_) < drop_rate_ 이면 드롭
//
//   반환: true → 이 패킷을 드롭해야 함
// ============================================================
bool MockLinkNode::shouldDrop()
{
  // ① 현재 burst 진행 중
  if (burst_loss_remaining_ > 0) {
    --burst_loss_remaining_;
    ++stat_burst_drop_;
    return true;
  }

  // ② 새 burst 시작 판정
  //   burst_loss_prob > 0.0 이어야 의미가 있음 (방어 조건)
  if (burst_loss_prob_ > 0.0 && prob_dist_(rng_) < burst_loss_prob_) {
    // burst_loss_len_ - 1: 이번 패킷이 burst의 첫 번째이므로
    //   다음 (burst_loss_len_ - 1)개를 추가로 드롭해야 함
    burst_loss_remaining_ = burst_loss_len_ - 1;
    ++stat_burst_drop_;
    return true;
  }

  // ③ 독립 드롭 판정
  if (drop_rate_ > 0.0 && prob_dist_(rng_) < drop_rate_) {
    return true;
  }

  return false;
}

// ============================================================
// extraDelayMs() — reorder용 추가 지연 계산
//
//   reorder_prob 확률로 [0, reorder_max_delay_ms_] 사이의
//   균일 분포 랜덤값을 반환.
//   이 값을 기본 delay_ms_에 더해서 release_time을 계산하면
//   priority_queue 내에서 순서가 뒤바뀔 수 있음.
//
//   예)
//     패킷A: delay_ms_=50ms, extra=30ms → release = now + 80ms
//     패킷B: delay_ms_=50ms, extra=0ms  → release = now + 50ms
//     B가 A보다 나중에 왔지만 먼저 발행됨 → reorder 완성
// ============================================================
double MockLinkNode::extraDelayMs()
{
  if (reorder_prob_ <= 0.0) {
    return 0.0;
  }

  // reorder 확률 판정
  if (prob_dist_(rng_) >= reorder_prob_) {
    return 0.0;  // reorder 없음
  }

  // [0, reorder_max_delay_ms_] 사이 균일 분포 랜덤 지연
  std::uniform_real_distribution<double> delay_dist(0.0, reorder_max_delay_ms_);
  return delay_dist(rng_);
}

}  // namespace safety

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety::MockLinkNode>());
  rclcpp::shutdown();
  return 0;
}