#ifndef SAFETY__DEADLINE_MONITOR_NODE_HPP_
#define SAFETY__DEADLINE_MONITOR_NODE_HPP_

// ============================================================
// deadline_monitor_node.hpp — 제어 루프 Deadline Miss 감지 노드
//
// [역할]
//   /metrics/control_latency_ms를 구독해서
//   MPC 제어 루프(50Hz = 20ms 목표)가 deadline을 초과하는
//   횟수와 비율을 측정하고 /diagnostics로 경고를 발행함.
//
// [Deadline Miss 정의]
//   control_latency_ms > deadline_ms_ (기본 20ms)
//   → 한 제어 주기 안에 MPC solve가 끝나지 않음
//   → 다음 주기의 명령이 늦게 발행됨
//   → 실시간 제어 품질 저하
//
// [발행 토픽]
//   /metrics/deadline_miss_stats (String, 1Hz)
//     → miss 횟수, miss율, 최대 latency, 99th percentile
//   /diagnostics
//     → miss율 5% 이상 시 WARN 발행
//
// [W11 Step4 실험 목적]
//   SingleThreadedExecutor에서 artificial_load_ms를 올리면
//   Deadline Miss가 증가하는 것을 수치로 증명.
//   이후 Step6에서 MultiThreadedExecutor로 전환 후
//   같은 부하에서 Deadline Miss가 감소하는 before/after 비교.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <amr_msgs/msg/control_latency.hpp>

#include <deque>
#include <mutex>
#include <string>

namespace safety
{

class DeadlineMonitorNode : public rclcpp::Node
{
public:
  explicit DeadlineMonitorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // /metrics/control_latency_ms 수신 콜백
  //   latency_ms 값을 슬라이딩 윈도우에 적재하고
  //   deadline 초과 여부 즉시 판정
  // --------------------------------------------------------
  void latencyCallback(const amr_msgs::msg::ControlLatency::SharedPtr msg);

  // --------------------------------------------------------
  // 1Hz 통계 발행 콜백
  //   슬라이딩 윈도우 기반 통계 계산 후 발행
  // --------------------------------------------------------
  void statsCallback();

  // --------------------------------------------------------
  // /diagnostics 발행
  //   miss_rate > warn_threshold_ 시 WARN 발행
  // --------------------------------------------------------
  void publishDiagnostics(double miss_rate, double max_latency,
                          double p99_latency);

  // --------------------------------------------------------
  // ROS2 인터페이스
  // --------------------------------------------------------
  rclcpp::Subscription<amr_msgs::msg::ControlLatency>::SharedPtr latency_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             stats_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr                                    stats_timer_;

  // --------------------------------------------------------
  // 슬라이딩 윈도우
  //   최근 window_size_개 latency 샘플 유지
  //   window_size_ = 500 → 10초 (50Hz 기준)
  // --------------------------------------------------------
  std::deque<double> latency_window_;
  static constexpr size_t WINDOW_SIZE = 500;

  // 누적 카운터
  uint64_t total_count_ = 0;
  uint64_t miss_count_  = 0;

  std::mutex window_mutex_;

  // --------------------------------------------------------
  // 파라미터
  // --------------------------------------------------------
  double deadline_ms_;      // Deadline 기준 [ms] (기본: 20ms = 50Hz)
  double warn_threshold_;   // /diagnostics WARN 발행 기준 miss율 (기본: 0.05 = 5%)
};

}  // namespace safety

#endif  // SAFETY__DEADLINE_MONITOR_NODE_HPP_
