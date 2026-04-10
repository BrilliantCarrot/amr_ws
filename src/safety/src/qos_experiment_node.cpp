#include "safety/qos_experiment_node.hpp"

#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace safety
{

// ============================================================
// 생성자
// ============================================================
QosExperimentNode::QosExperimentNode(const rclcpp::NodeOptions & options)
: Node("qos_experiment_node", options)
{
  // --------------------------------------------------------
  // 파라미터 선언
  //
  //   reliability:
  //     "reliable"    → rclcpp::ReliabilityPolicy::Reliable
  //                     드롭된 패킷을 재전송 요청함.
  //                     고주파(50Hz) 토픽에서는 재전송이 쌓여서
  //                     오히려 latency spike가 생길 수 있음.
  //     "best_effort" → rclcpp::ReliabilityPolicy::BestEffort
  //                     드롭된 패킷을 그냥 포기.
  //                     고주파 센서 토픽(IMU, LiDAR)에서 선호.
  //                     실시간성이 중요할 때 적합.
  // --------------------------------------------------------
  this->declare_parameter<std::string>("reliability", "reliable");
  reliability_ = this->get_parameter("reliability").as_string();

  // --------------------------------------------------------
  // QoS 프로파일 구성
  //   depth=10: 큐 깊이. RELIABLE에서 재전송 대기 메시지 수.
  //   BEST_EFFORT에서는 depth가 크게 의미 없음 (드롭하니까).
  // --------------------------------------------------------
  rclcpp::QoS qos(10);
  if (reliability_ == "best_effort") {
    // BEST_EFFORT: 드롭된 패킷 재전송 없음
    // 실시간 센서 데이터에 적합 (이전 값보다 새 값이 항상 중요)
    qos.best_effort();
    RCLCPP_INFO(this->get_logger(),
      "[QosExperiment] QoS: BEST_EFFORT 모드로 실행");
  } else {
    // RELIABLE: 드롭된 패킷 재전송 요청
    // 상태 명령, 파라미터 전달 등 손실이 치명적인 토픽에 적합
    qos.reliable();
    RCLCPP_INFO(this->get_logger(),
      "[QosExperiment] QoS: RELIABLE 모드로 실행");
  }

  // --------------------------------------------------------
  // 토픽 통계 초기화
  //   name, target_hz 설정
  //   나머지는 기본값(0, true)으로 초기화됨
  // --------------------------------------------------------
  cmd_stats_.name      = "/cmd_vel";
  cmd_stats_.target_hz = 50.0;  // MpcNode 제어 주기 50Hz

  odom_stats_.name      = "/map_ekf/odom";
  odom_stats_.target_hz = 50.0;  // map_ekf_node 발행 주기 50Hz

  loc_stats_.name      = "/localization/status";
  loc_stats_.target_hz = 10.0;  // localization_monitor_node 발행 주기 10Hz

  // --------------------------------------------------------
  // 구독자 생성 (동일한 QoS 프로파일 적용)
  //
  //   [주의] publisher와 subscriber의 QoS가 호환되어야 함.
  //   RELIABLE publisher ↔ BEST_EFFORT subscriber: 호환 안 됨
  //   RELIABLE publisher ↔ RELIABLE subscriber:    호환 됨
  //   BEST_EFFORT publisher ↔ BEST_EFFORT subscriber: 호환 됨
  //   BEST_EFFORT publisher ↔ RELIABLE subscriber:  호환 안 됨
  //
  //   실험 목적: publisher(기존 노드)는 RELIABLE이므로
  //   subscriber를 BEST_EFFORT로 바꿔도 연결은 되지만
  //   ROS2가 내부적으로 BEST_EFFORT로 강등시킴.
  //   → 드롭 환경에서 재전송을 포기하는 효과를 실험으로 확인.
  // --------------------------------------------------------
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", qos,
    std::bind(&QosExperimentNode::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/map_ekf/odom", qos,
    std::bind(&QosExperimentNode::odomCallback, this, std::placeholders::_1));

  loc_sub_ = this->create_subscription<amr_msgs::msg::LocalizationStatus>(
    "/localization/status", qos,
    std::bind(&QosExperimentNode::locStatusCallback, this, std::placeholders::_1));

  // --------------------------------------------------------
  // 통계 발행자: /qos_experiment/stats (String)
  //   tools/qos_experiment.py가 이 토픽을 구독해서 CSV 저장
  // --------------------------------------------------------
  stats_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/qos_experiment/stats", rclcpp::QoS(10).reliable());

  // 1Hz 통계 계산 + 발행 타이머
  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&QosExperimentNode::statsCallback, this));

  // 실험 시작 시각 기록
  experiment_start_sec_ = this->now().seconds();

  RCLCPP_INFO(this->get_logger(),
    "[QosExperiment] 초기화 완료. reliability=%s | 구독 토픽: %s, %s, %s",
    reliability_.c_str(),
    cmd_stats_.name.c_str(),
    odom_stats_.name.c_str(),
    loc_stats_.name.c_str());
}

// ============================================================
// cmdVelCallback() — /cmd_vel 수신
// ============================================================
void QosExperimentNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr /*msg*/)
{
  // 내용은 보지 않음 — 수신 시각과 빈도만 측정
  std::lock_guard<std::mutex> lock(stats_mutex_);
  recordReceipt(cmd_stats_, this->now());
}

// ============================================================
// odomCallback() — /map_ekf/odom 수신
// ============================================================
void QosExperimentNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  recordReceipt(odom_stats_, this->now());
}

// ============================================================
// locStatusCallback() — /localization/status 수신
// ============================================================
void QosExperimentNode::locStatusCallback(
  const amr_msgs::msg::LocalizationStatus::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  recordReceipt(loc_stats_, this->now());
}

// ============================================================
// recordReceipt() — 공통 수신 처리
//
//   모든 구독 콜백에서 공통으로 호출되는 헬퍼.
//   stats_mutex_는 호출자(콜백)가 이미 잡고 있음.
//
//   [처리 내용]
//   ① rx_count 증가
//   ② 첫 수신이면 first_rx_time 기록 후 리턴
//   ③ 두 번째 이후: 수신 간격(ms) 계산 → intervals_ms 적재
//   ④ intervals_ms가 MAX_INTERVALS 초과하면 앞에서 제거
//      (최근 N개만 유지 — 슬라이딩 윈도우)
// ============================================================
void QosExperimentNode::recordReceipt(
  TopicStats & stats, const rclcpp::Time & now)
{
  ++stats.rx_count;

  if (stats.first_rx) {
    // 첫 수신: 기준 시각만 기록
    stats.first_rx_time = now;
    stats.last_rx_time  = now;
    stats.first_rx      = false;
    return;
  }

  // 수신 간격 [ms] 계산
  const double interval_ms =
    (now - stats.last_rx_time).seconds() * 1000.0;

  // 비정상적으로 큰 간격은 제외 (노드 초기화 직후 등)
  // target_hz가 50Hz면 이상적 간격은 20ms → 200ms 이상은 이상값
  const double max_valid_interval_ms = (1.0 / stats.target_hz) * 10 * 1000.0;
  if (interval_ms < max_valid_interval_ms) {
    stats.intervals_ms.push_back(interval_ms);
    if (stats.intervals_ms.size() > TopicStats::MAX_INTERVALS) {
      stats.intervals_ms.pop_front();  // 슬라이딩 윈도우 유지
    }
  }

  stats.last_rx_time = now;
}

// ============================================================
// statsCallback() — 1Hz 통계 계산 + 발행
//
//   세 토픽의 통계를 계산하고 String 형태로 발행.
//   qos_experiment.py가 이 토픽을 파싱해서 CSV로 저장함.
// ============================================================
void QosExperimentNode::statsCallback()
{
  const rclcpp::Time now = this->now();

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    computeStats(cmd_stats_,  now);
    computeStats(odom_stats_, now);
    computeStats(loc_stats_,  now);
  }

  // ---- 결과 문자열 구성 ----
  // qos_experiment.py가 파싱할 수 있도록 구분자를 통일
  // 형식: [QosExp] reliability=reliable | /cmd_vel hz=49.2 loss=1.6% jitter=0.3ms | ...
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "[QosExp] reliability=" << reliability_ << " | ";

  auto appendStats = [&](const TopicStats & s) {
    oss << s.name
        << " hz="      << s.actual_hz
        << " loss="    << s.loss_rate * 100.0 << "%"
        << " jitter="  << s.jitter_ms << "ms"
        << " rx="      << s.rx_count
        << " | ";
  };

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    appendStats(cmd_stats_);
    appendStats(odom_stats_);
    appendStats(loc_stats_);
  }

  std_msgs::msg::String msg;
  msg.data = oss.str();
  stats_pub_->publish(msg);

  // 터미널 로그 (3초에 1번)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "%s", oss.str().c_str());
}

// ============================================================
// computeStats() — 파생 통계 계산
//
//   [actual_hz]
//     첫 수신 이후 경과 시간 기준으로 계산.
//     elapsed = now - first_rx_time
//     actual_hz = rx_count / elapsed
//
//   [loss_rate]
//     expected = target_hz × elapsed
//     loss_rate = max(0, 1 - rx_count / expected)
//     음수 방지: 실제 hz가 target_hz보다 높을 수 있음 (정상)
//
//   [jitter_ms]
//     intervals_ms의 표준편차.
//     std = sqrt( Σ(xi - mean)² / N )
//     작을수록 수신 간격이 일정 → 안정적인 통신
// ============================================================
void QosExperimentNode::computeStats(
  TopicStats & stats, const rclcpp::Time & now)
{
  if (stats.first_rx || stats.rx_count < 2) {
    // 아직 데이터 부족
    return;
  }

  const double elapsed = (now - stats.first_rx_time).seconds();
  if (elapsed < 0.1) {
    return;  // 시작 직후 방어
  }

  // ① actual_hz
  stats.actual_hz = static_cast<double>(stats.rx_count) / elapsed;

  // ② loss_rate
  const double expected = stats.target_hz * elapsed;
  stats.loss_rate = std::max(0.0,
    1.0 - static_cast<double>(stats.rx_count) / expected);

  // ③ jitter_ms (수신 간격 표준편차)
  if (stats.intervals_ms.size() >= 2) {
    // 평균 계산
    double sum = std::accumulate(
      stats.intervals_ms.begin(), stats.intervals_ms.end(), 0.0);
    double mean = sum / stats.intervals_ms.size();

    // 분산 계산
    double sq_sum = 0.0;
    for (double v : stats.intervals_ms) {
      sq_sum += (v - mean) * (v - mean);
    }
    stats.jitter_ms = std::sqrt(sq_sum / stats.intervals_ms.size());
  }
}

}  // namespace safety

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety::QosExperimentNode>());
  rclcpp::shutdown();
  return 0;
}
