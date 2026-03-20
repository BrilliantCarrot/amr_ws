#ifndef LOCALIZATION__LOCALIZATION_MONITOR_NODE_HPP_
#define LOCALIZATION__LOCALIZATION_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <amr_msgs/msg/localization_status.hpp>

#include <deque>
// 슬라이딩 윈도우 RMSE 버퍼용. deque는 앞뒤로 추가/삭제가 빠른 자료구조라 윈도우 관리에 적합.

// TF: 여러 좌표계 간의 위치/방향 관계를 관리하는 시스템
// "LiDAR가 벽을 감지했는데, 그게 지도 기준으로 어디야?" 같은 걸 계산하려면 좌표계 간 변환이 필요해. TF 시스템이 이걸 트리 구조로 관리해줌.

// map
//  └── odom
//       └── base_link
//            ├── laser_link
//            └── imu_link
// 어느 노드든 /tf 토픽을 구독하면 이 트리 전체를 알 수 있고, map → laser_link 같은 간접 변환도 자동으로 계산해줌.
// TF 관련 두 개는 항상 세트
// Buffer — 과거 TF들을 시간 순으로 저장하는 창고
// TransformListener — /tf 토픽을 구독해서 Buffer에 자동으로 채워주는 역할. 이 둘이 있어야 lookupTransform()으로 TF를 조회할 수 있음

// map->odom TF: slam_toolbox가 발행하는 변환, 의미는: odom 원점이 map 기준으로 어디 있나
// 왜 필요하냐면, odom은 드리프트가 쌓여서 실제 위치랑 점점 어긋남. 로봇이 한 바퀴 돌고 출발점에 돌아왔을 때 odom은 "나 출발점에서 0.3m 떨어져 있어"라고 할 수 있어.
// slam_toolbox는 LiDAR 스캔을 지도와 매칭해서 실제 위치를 알고 있으니까, 그 오차만큼 map→odom TF로 보정값을 발행해줌.
// 실제 로봇 위치 = map→odom (slam 보정) + odom→base_link (EKF 추정)

// TF 하나의 변환 정보(위치 + 회전 + 타임스탬프)를 담는 메시지 타입. lookupTransform() 결과가 이 타입으로 나옴.

// #include <amr_msgs/msg/localization_status.hpp>
// 직접 만든 메시지. /localization/status 발행에 사용.


namespace localization
{

// ============================================================
// LocalizationMonitorNode
//
// [역할]
//   slam_toolbox가 발행하는 map→odom TF를 감시해서
//   localization 품질을 실시간으로 평가함.
//
// [감지 방법]
//   1. TF 타임아웃: map→odom TF가 일정 시간 이상 안 오면 LOST
//   2. TF 점프:    한 스텝에서 위치가 갑자기 크게 바뀌면 DEGRADED
//   3. RMSE:       Ground Truth vs map frame pose 오차 계산
//
// [상태 정의]
//   NORMAL(0)   : 정상 localization
//   DEGRADED(1) : 품질 저하 (점프 감지)
//   LOST(2)     : localization 완전 소실 (TF 타임아웃)
// ============================================================

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  explicit LocalizationMonitorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --------------------------------------------------------
  // 메인 모니터링 루프 (10Hz 타이머)
  //   map→odom TF를 주기적으로 읽어서 상태 판정 후 발행
  // --------------------------------------------------------
  void monitorCallback();

  // --------------------------------------------------------
  // Ground Truth 콜백
  //   pose_rmse_node와 동일하게 Gazebo world frame에서
  //   amr_robot의 실제 위치를 읽어옴
  // --------------------------------------------------------
  void gtCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  // --------------------------------------------------------
  // map frame pose 읽기
  //   TF 버퍼에서 map→base_link 변환을 조회해서
  //   현재 map 프레임 기준 로봇 위치를 반환
  //   실패 시 false 반환
  // --------------------------------------------------------
  bool getMapPose(double & x, double & y, double & yaw);

  // --------------------------------------------------------
  // 상태 판정
  //   TF 타임아웃, 점프 크기를 기반으로
  //   NORMAL / DEGRADED / LOST 결정
  // --------------------------------------------------------
  uint8_t evaluateStatus(double tf_age_sec, double tf_jump_m);

  // --- ROS2 인터페이스 ---
  rclcpp::TimerBase::SharedPtr                               timer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr  gt_sub_;
  rclcpp::Publisher<amr_msgs::msg::LocalizationStatus>::SharedPtr status_pub_;

  // --- TF ---
  // TF 버퍼: 과거 TF를 일정 시간 저장해두는 버퍼
  // TransformListener: /tf 토픽을 구독해서 버퍼에 채워주는 객체
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Ground Truth 캐시 ---
  double gt_x_   = 0.0;
  double gt_y_   = 0.0;
  double gt_yaw_ = 0.0;
  bool   has_gt_ = false;

  // --- 이전 map→odom TF 위치 (점프 감지용) ---
  double prev_map_x_   = 0.0;
  double prev_map_y_   = 0.0;
  bool   has_prev_tf_  = false;

  // --- RMSE 슬라이딩 윈도우 ---
  std::deque<double> buf_x_;
  std::deque<double> buf_y_;
  std::deque<double> buf_yaw_;
  int window_size_;

  // --- 파라미터 ---
  double tf_timeout_sec_;   // 이 시간 초과 시 LOST
  double tf_jump_thresh_m_; // 이 거리 초과 시 DEGRADED
  std::string robot_name_;

  // --- 마지막 정상 TF 시간 ---
  rclcpp::Time last_tf_time_;
  bool         has_last_tf_time_ = false;
};

}  // namespace localization

#endif  // LOCALIZATION__LOCALIZATION_MONITOR_NODE_HPP_