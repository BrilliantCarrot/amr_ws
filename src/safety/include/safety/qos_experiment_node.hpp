#ifndef SAFETY__QOS_EXPERIMENT_NODE_HPP_
#define SAFETY__QOS_EXPERIMENT_NODE_HPP_

// ============================================================
// qos_experiment_node.hpp — QoS 비교 실험 노드 헤더
//
// [역할]
//   세 가지 핵심 토픽의 수신 품질을 측정함.
//   파라미터 'reliability'로 RELIABLE/BEST_EFFORT를 선택하고,
//   동일한 drop_rate 조건에서 QoS별 수신 품질을 비교함.
//
// [측정 토픽 3개]
//   /cmd_vel              : MpcNode 제어 명령 (50Hz 목표)
//   /localization/status  : Localization 상태 (10Hz 목표)
//   /map_ekf/odom         : LiDAR fusion EKF 출력 (50Hz 목표)
//
// [측정 지표 (토픽별)]
//   rx_count      : 수신된 총 메시지 수
//   expected_count: 경과 시간 기준 기대 메시지 수 (target_hz × elapsed_sec)
//   loss_rate     : 손실률 = 1 - rx_count / expected_count
//   actual_hz     : 실제 수신 빈도 = rx_count / elapsed_sec
//   jitter_ms     : 수신 간격 표준편차 [ms]
//                   (작을수록 안정적인 수신 주기)
//
// [QoS 선택 방식]
//   파라미터 'reliability' = "reliable"    → RELIABLE QoS
//   파라미터 'reliability' = "best_effort" → BEST_EFFORT QoS
//   노드 실행 시 파라미터로 지정:
//     ros2 run safety qos_experiment_node
//       --ros-args -p reliability:=best_effort
//
// [발행 토픽]
//   /qos_experiment/stats (String, 1Hz)
//     → 세 토픽의 수신 품질 통계를 사람이 읽기 쉬운 형태로 발행
//     → tools/qos_experiment.py가 이 토픽을 구독해서 CSV 저장
//
// [왜 별도 노드로 만드나?]
//   기존 노드(MpcNode, map_ekf_node)의 QoS를 런타임에 바꾸면
//   재빌드가 필요함. 별도 실험 노드를 두면 파라미터 하나로
//   QoS를 전환할 수 있어서 실험 자동화가 쉬워짐.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <amr_msgs/msg/localization_status.hpp>

#include <deque>
#include <string>
#include <mutex>

namespace safety
{

// ============================================================
// TopicStats — 토픽별 수신 품질 통계
//   모든 멤버는 qos_experiment_node가 공유하므로
//   stats_mutex_로 보호됨
// ============================================================
struct TopicStats
{
  std::string name;           // 토픽 이름
  double      target_hz;      // 목표 수신 빈도 [Hz]

  // ---- 수신 카운터 ----
  uint64_t    rx_count = 0;   // 수신된 총 메시지 수

  // ---- 시간 추적 ----
  rclcpp::Time first_rx_time; // 첫 메시지 수신 시각
  rclcpp::Time last_rx_time;  // 직전 메시지 수신 시각
  bool         first_rx = true; // 첫 수신 여부

  // ---- 수신 간격 기록 (jitter 계산용) ----
  // 최근 100개 간격만 유지 (메모리 절약)
  std::deque<double> intervals_ms;
  static constexpr size_t MAX_INTERVALS = 100;

  // ---- 파생 통계 (statsCallback에서 계산) ----
  double actual_hz   = 0.0;   // 실제 수신 빈도 [Hz]
  double loss_rate   = 0.0;   // 손실률 [0.0~1.0]
  double jitter_ms   = 0.0;   // 수신 간격 표준편차 [ms]
};

// ============================================================
// QosExperimentNode
// ============================================================
class QosExperimentNode : public rclcpp::Node
{
public:
  explicit QosExperimentNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // 콜백: 각 토픽 수신 시 TopicStats 갱신
  // --------------------------------------------------------
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void locStatusCallback(const amr_msgs::msg::LocalizationStatus::SharedPtr msg);

  // --------------------------------------------------------
  // 공통 수신 처리 — 콜백마다 중복되는 로직을 통합
  //   stats    : 갱신할 TopicStats 참조
  //   now      : 수신 시각
  // --------------------------------------------------------
  void recordReceipt(TopicStats & stats, const rclcpp::Time & now);

  // --------------------------------------------------------
  // 1Hz 통계 계산 + 발행 콜백
  //   각 TopicStats의 파생 통계를 계산하고
  //   /qos_experiment/stats로 발행
  // --------------------------------------------------------
  void statsCallback();

  // --------------------------------------------------------
  // 통계 계산 헬퍼
  //   stats.intervals_ms → actual_hz, loss_rate, jitter_ms 계산
  // --------------------------------------------------------
  void computeStats(TopicStats & stats, const rclcpp::Time & now);

  // --------------------------------------------------------
  // ROS2 인터페이스
  // --------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr  loc_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                  stats_pub_;
  rclcpp::TimerBase::SharedPtr                                         stats_timer_;

  // --------------------------------------------------------
  // 토픽별 통계 (3개)
  // --------------------------------------------------------
  TopicStats cmd_stats_;   // /cmd_vel
  TopicStats odom_stats_;  // /map_ekf/odom
  TopicStats loc_stats_;   // /localization/status

  // --------------------------------------------------------
  // 멀티스레드 보호용 mutex
  //   statsCallback(타이머)과 각 콜백(구독자)이
  //   MultiThreadedExecutor에서 병렬 실행될 수 있으므로
  //   TopicStats 접근 시 lock 필요
  // --------------------------------------------------------
  std::mutex stats_mutex_;

  // --------------------------------------------------------
  // 파라미터
  // --------------------------------------------------------
  std::string reliability_;  // "reliable" 또는 "best_effort"
  double      experiment_start_sec_;  // 노드 시작 시각 (epoch 기준)
};

}  // namespace safety

#endif  // SAFETY__QOS_EXPERIMENT_NODE_HPP_
