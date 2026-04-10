#ifndef SAFETY__MOCK_LINK_NODE_HPP_
#define SAFETY__MOCK_LINK_NODE_HPP_

// ============================================================
// mock_link_node.hpp — 통신 이상 주입 노드 헤더
//
// [역할]
//   MpcNode가 발행하는 /cmd_vel을 중간에서 가로채서
//   delay / drop / burst_loss / reorder를 주입한 뒤
//   /cmd_vel_delayed로 재발행함.
//
//   bringup.launch.py에서 Gazebo 브릿지 토픽을
//   /cmd_vel → /cmd_vel_delayed로 교체하면
//   실제 로봇(시뮬)에 이상 통신이 걸린 명령이 전달됨.
//
// [토픽 흐름]
//   MpcNode → /cmd_vel
//              └─ [mock_link_node]
//                   ├─ drop/burst_loss 판정 → 드롭 시 폐기
//                   ├─ reorder 판정 → 랜덤 추가 지연 부여
//                   └─ delay 큐에 적재
//                        └─ 50Hz 타이머 → 만료된 패킷 /cmd_vel_delayed 발행
//
// [이상 주입 종류]
//   delay_ms       : 고정 전파 지연 [ms]. 모든 패킷에 동일하게 적용.
//   drop_rate      : 독립 드롭 확률 (0.0~1.0). 패킷 단위 Bernoulli 시행.
//   burst_loss_prob: burst 손실 시작 확률. burst 중에는 연속 N개 드롭.
//   burst_loss_len : burst 연속 드롭 수.
//   reorder_prob   : reorder 확률. 해당 패킷에 랜덤 추가 지연 부여.
//   reorder_max_delay_ms: reorder 시 최대 추가 지연 [ms].
//   enabled        : false면 /cmd_vel을 그대로 /cmd_vel_delayed로 투명 전달.
//
// [검증 토픽]
//   /mock_link/stats (10Hz) : 누적 통계 (sent/dropped/delayed 등)
//
// [W11 MultiThreadedExecutor 전환 시 주의]
//   cmd_vel_callback과 timer_callback이 병렬 실행될 수 있으므로
//   delay_queue_ 접근 시 queue_mutex_ 사용 필수.
//   현재(SingleThreadedExecutor)에서는 mutex가 있어도 비용만 발생하지만
//   W11 Step 4 전환을 대비해 미리 mutex 패턴을 적용해 둠.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <queue>
#include <random>
#include <mutex>
#include <string>

namespace safety
{

// ============================================================
// DelayedPacket — delay 큐에 적재되는 패킷 단위
//   release_time: 이 시각 이후에 발행 허용
//   twist:        실제 명령값
// ============================================================
struct DelayedPacket
{
  rclcpp::Time              release_time;  // 발행 허용 시각
  geometry_msgs::msg::Twist twist;         // 전달할 명령값

  // priority_queue를 최소 힙(가장 빠른 release_time 먼저)으로 쓰기 위한 비교자
  bool operator>(const DelayedPacket & other) const
  {
    return release_time > other.release_time;
  }
};

// ============================================================
// MockLinkNode
// ============================================================
class MockLinkNode : public rclcpp::Node
{
public:
  explicit MockLinkNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // /cmd_vel 수신 콜백
  //   ① enabled=false → 즉시 투명 전달
  //   ② drop 판정 → 드롭 시 폐기
  //   ③ reorder 판정 → 추가 랜덤 지연 부여
  //   ④ delay 큐에 적재
  // --------------------------------------------------------
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // --------------------------------------------------------
  // 50Hz 타이머 콜백
  //   delay 큐에서 release_time이 지난 패킷을 꺼내
  //   /cmd_vel_delayed로 발행함.
  //   우선순위 큐(최소 힙)이므로 top()이 가장 빠른 패킷.
  // --------------------------------------------------------
  void timerCallback();

  // --------------------------------------------------------
  // 10Hz 통계 발행 콜백
  //   누적 통계를 /mock_link/stats (String)으로 발행.
  //   면접/포트폴리오 데모 시 rqt로 실시간 확인 가능.
  // --------------------------------------------------------
  void statsCallback();

  // --------------------------------------------------------
  // 드롭 판정 헬퍼
  //   독립 drop_rate 판정 + burst_loss 판정을 합산.
  //   burst_loss_remaining_ > 0이면 무조건 드롭.
  //   burst_loss_prob로 새 burst 시작 여부 결정.
  //   반환: true → 이 패킷을 드롭해야 함
  // --------------------------------------------------------
  bool shouldDrop();

  // --------------------------------------------------------
  // reorder 추가 지연 계산 헬퍼
  //   reorder_prob 확률로 [0, reorder_max_delay_ms] 사이
  //   균일 분포 랜덤 지연 반환.
  //   반환: 추가 지연 [ms] (reorder 없으면 0.0)
  // --------------------------------------------------------
  double extraDelayMs();

  // --------------------------------------------------------
  // ROS2 인터페이스
  // --------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        stats_pub_;
  rclcpp::TimerBase::SharedPtr                               relay_timer_;
  rclcpp::TimerBase::SharedPtr                               stats_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // --------------------------------------------------------
  // delay 큐 — 최소 힙 (release_time 기준 오름차순)
  //   top() = 가장 빨리 발행해야 할 패킷
  // --------------------------------------------------------
  std::priority_queue<
    DelayedPacket,
    std::vector<DelayedPacket>,
    std::greater<DelayedPacket>   // 최소 힙
  > delay_queue_;

  // W11 MultiThreadedExecutor 대비 mutex
  // SingleThreadedExecutor 현재 환경에서도 패턴을 미리 적용해 둠
  std::mutex queue_mutex_;

  // --------------------------------------------------------
  // 파라미터
  // --------------------------------------------------------
  bool   enabled_;               // 전체 기능 on/off
  double delay_ms_;              // 고정 전파 지연 [ms]
  double drop_rate_;             // 독립 드롭 확률 [0.0~1.0]
  double burst_loss_prob_;       // burst 손실 시작 확률
  int    burst_loss_len_;        // burst 연속 드롭 수
  double reorder_prob_;          // reorder 확률
  double reorder_max_delay_ms_;  // reorder 최대 추가 지연 [ms]

  // --------------------------------------------------------
  // burst 손실 상태 추적
  //   burst_loss_remaining_ > 0 이면 현재 burst 진행 중.
  //   패킷을 드롭할 때마다 1씩 감소.
  // --------------------------------------------------------
  int burst_loss_remaining_ = 0;

  // --------------------------------------------------------
  // 난수 생성기
  //   mt19937: Mersenne Twister, 통계적으로 균일한 난수 보장.
  //   uniform_real_distribution(0,1): drop/burst/reorder 확률 판정용.
  //   uniform_real_distribution(0, reorder_max): reorder 지연량 계산용.
  // --------------------------------------------------------
  std::mt19937                          rng_;
  std::uniform_real_distribution<double> prob_dist_;   // [0.0, 1.0) 균일 분포

  // --------------------------------------------------------
  // 누적 통계 카운터
  // --------------------------------------------------------
  uint64_t stat_total_rx_    = 0;  // 수신한 총 패킷 수
  uint64_t stat_dropped_     = 0;  // 드롭된 패킷 수
  uint64_t stat_burst_drop_  = 0;  // burst로 드롭된 패킷 수
  uint64_t stat_reordered_   = 0;  // reorder된 패킷 수
  uint64_t stat_total_tx_    = 0;  // 발행된 총 패킷 수
};

}  // namespace safety

#endif  // SAFETY__MOCK_LINK_NODE_HPP_