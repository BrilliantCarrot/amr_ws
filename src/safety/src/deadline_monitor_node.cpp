#include "safety/deadline_monitor_node.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace safety
{

// ============================================================
// 생성자
// ============================================================
DeadlineMonitorNode::DeadlineMonitorNode(const rclcpp::NodeOptions & options)
: Node("deadline_monitor_node", options)
{
  // --------------------------------------------------------
  // 파라미터 선언
  //
  //   deadline_ms:
  //     MPC 제어 주기 목표값. 50Hz → 20ms.
  //     이 값을 초과하는 solve는 Deadline Miss로 판정.
  //     실시간 제어 시스템에서 deadline을 넘기면
  //     다음 주기 명령이 지연되어 제어 품질이 저하됨.
  //
  //   warn_threshold:
  //     /diagnostics WARN 발행 기준 miss율 (0.0~1.0).
  //     기본 0.05 = 5% → 100번 중 5번 초과 시 경고.
  // --------------------------------------------------------
  this->declare_parameter<double>("deadline_ms",    20.0);
  this->declare_parameter<double>("warn_threshold", 0.05);

  deadline_ms_    = this->get_parameter("deadline_ms").as_double();
  warn_threshold_ = this->get_parameter("warn_threshold").as_double();

  auto reliable_qos = rclcpp::QoS(10).reliable();

  // /metrics/control_latency_ms 구독
  // mpc_node가 50Hz로 발행하는 MPC solve time
  latency_sub_ = this->create_subscription<amr_msgs::msg::ControlLatency>(
    "/metrics/control_latency_ms", reliable_qos,
    std::bind(&DeadlineMonitorNode::latencyCallback, this, std::placeholders::_1));

  // 통계 발행: /metrics/deadline_miss_stats (String)
  stats_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/metrics/deadline_miss_stats", reliable_qos);

  // /diagnostics 발행
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", reliable_qos);

  // 1Hz 통계 타이머
  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&DeadlineMonitorNode::statsCallback, this));

  RCLCPP_INFO(this->get_logger(),
    "[DeadlineMonitor] 초기화 완료. deadline=%.0fms, warn_threshold=%.0f%%",
    deadline_ms_, warn_threshold_ * 100.0);
}

// ============================================================
// latencyCallback() — /metrics/control_latency_ms 수신
//
//   [처리 내용]
//   ① latency_ms를 슬라이딩 윈도우에 추가
//   ② WINDOW_SIZE 초과 시 가장 오래된 샘플 제거
//   ③ deadline 초과 여부 판정 → miss_count_ 증가
//   ④ total_count_ 증가
// ============================================================
void DeadlineMonitorNode::latencyCallback(
  const amr_msgs::msg::ControlLatency::SharedPtr msg)
{
  const double latency = msg->latency_ms;

  std::lock_guard<std::mutex> lock(window_mutex_);

  latency_window_.push_back(latency);
  if (latency_window_.size() > WINDOW_SIZE) {
    latency_window_.pop_front();
  }

  ++total_count_;

  // Deadline Miss 판정
  // latency_ms > deadline_ms_ → 이번 solve가 제어 주기를 초과함
  if (latency > deadline_ms_) {
    ++miss_count_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[DeadlineMonitor] Deadline Miss! latency=%.2fms (기준 %.0fms)",
      latency, deadline_ms_);
  }
}

// ============================================================
// statsCallback() — 1Hz 통계 계산 + 발행
//
//   [계산 항목]
//   ① window_miss_rate: 윈도우 내 miss 비율
//      (최근 10초 기준 — 누적 miss_count_와 다름)
//   ② max_latency: 윈도우 내 최대값
//   ③ p99_latency: 99th percentile
//      (정렬 후 99% 위치 인덱스 값)
//   ④ avg_latency: 평균
// ============================================================
void DeadlineMonitorNode::statsCallback()
{
  std::unique_lock<std::mutex> lock(window_mutex_);

  if (latency_window_.empty()) {
    return;
  }

  // 윈도우 복사 후 lock 해제 (정렬 중 콜백 블로킹 방지)
  std::vector<double> sorted_window(
    latency_window_.begin(), latency_window_.end());
  const uint64_t total  = total_count_;
  const uint64_t miss   = miss_count_;
  lock.unlock();

  // ① 윈도우 내 miss 횟수 / miss율
  const size_t window_miss = std::count_if(
    sorted_window.begin(), sorted_window.end(),
    [this](double v) { return v > deadline_ms_; });
  const double window_miss_rate =
    static_cast<double>(window_miss) / sorted_window.size();

  // ② 최대값
  const double max_latency =
    *std::max_element(sorted_window.begin(), sorted_window.end());

  // ③ 평균
  const double avg_latency =
    std::accumulate(sorted_window.begin(), sorted_window.end(), 0.0)
    / sorted_window.size();

  // ④ 99th percentile
  //   정렬 후 상위 1% 인덱스 값
  std::sort(sorted_window.begin(), sorted_window.end());
  const size_t p99_idx = static_cast<size_t>(
    std::ceil(0.99 * sorted_window.size())) - 1;
  const double p99_latency = sorted_window[
    std::min(p99_idx, sorted_window.size() - 1)];

  // ---- 통계 문자열 구성 ----
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "[DeadlineMiss] "
      << "window_miss=" << window_miss
      << "(" << window_miss_rate * 100.0 << "%) "
      << "total_miss=" << miss << "/" << total
      << " | avg=" << avg_latency << "ms"
      << " p99=" << p99_latency << "ms"
      << " max=" << max_latency << "ms"
      << " | deadline=" << deadline_ms_ << "ms";

  std_msgs::msg::String stats_msg;
  stats_msg.data = oss.str();
  stats_pub_->publish(stats_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "%s", oss.str().c_str());

  // /diagnostics 발행
  publishDiagnostics(window_miss_rate, max_latency, p99_latency);
}

// ============================================================
// publishDiagnostics()
//
//   miss_rate < warn_threshold_ → OK
//   miss_rate >= warn_threshold_ → WARN
// ============================================================
void DeadlineMonitorNode::publishDiagnostics(
  double miss_rate, double max_latency, double p99_latency)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name        = "control/deadline_miss";
  status.hardware_id = "mpc_node";

  if (miss_rate >= warn_threshold_) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Deadline Miss율이 임계값을 초과했습니다";
  } else {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Deadline Miss 정상 범위";
  }

  auto addKV = [&](const std::string & k, const std::string & v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key   = k;
    kv.value = v;
    status.values.push_back(kv);
  };

  addKV("miss_rate_pct",  std::to_string(miss_rate * 100.0));
  addKV("max_latency_ms", std::to_string(max_latency));
  addKV("p99_latency_ms", std::to_string(p99_latency));
  addKV("deadline_ms",    std::to_string(deadline_ms_));

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

}  // namespace safety

// ============================================================
// main
// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety::DeadlineMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
