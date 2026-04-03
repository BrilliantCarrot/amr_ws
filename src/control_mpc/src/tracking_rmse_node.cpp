#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <amr_msgs/msg/pose_rmse.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>
#include <deque>
#include <string>

// ============================================================
// tracking_rmse_node.cpp — MPC 추종 오차 실시간 계산 노드
//
// [목적]
//   pose_rmse_node가 "EKF 추정 vs Ground Truth(GT)"를 비교한다면,
//   이 노드는 "MPC 목표 지점 vs Ground Truth(GT)"를 비교함.
//   즉, MPC 제어기가 얼마나 경로를 정확하게 따라가는지를 측정함.
//
// [비교 대상]
//   목표(Reference) : /mpc/reference_pose  → mpc_node가 발행한 x_ref[0] (map frame)
//   실제(GT)        : /world/simple_room/dynamic_pose/info → Gazebo GT 위치
//
// [pose_rmse_node와의 차이]
//   pose_rmse_node : /ekf/odom (EKF 추정값) vs GT → 추정기 성능 측정
//   tracking_rmse_node: /mpc/reference_pose (목표) vs GT → 제어기 성능 측정
//
// [구독/발행 토픽]
//   구독: /world/simple_room/dynamic_pose/info → TFMessage (Gazebo GT)
//   구독: /mpc/reference_pose                  → PoseStamped (MPC 목표 지점)
//   발행: /metrics/tracking_rmse               → PoseRmse (추종 오차)
//
// [슬라이딩 윈도우 RMSE]
//  전체 구간 평균을 내면 초반 불안정 구간이 포함되어 수치가 왜곡되므로
//  최근 N개 샘플만 유지하는 슬라이딩 윈도우를 사용
//   window_size=100, 50Hz 기준 → 최근 2초 구간 RMSE
//   pose_rmse_node와 동일한 방식으로 구현 (일관성 유지)
//  구현은 deque(양방향 큐)
//
// [주의: Continuous Yaw]
//   mpc_node는 시스템 전체에서 누적 연속 Yaw를 사용함.
//   x_ref[0].yaw는 정규화(-π~π)되지 않은 연속값일 수 있음.
//   GT yaw는 Gazebo에서 [-π, π] 범위로 나옴.
//   오차 계산 시에만 atan2(sin, cos) 트릭으로 정규화하여 비교함.
// ============================================================

// 수치를 통해 추정과 제어 상황 진단이 가능
// pose_rmse 작음 + tracking_rmse 작음 → 추정도 제어도 모두 정상 ✅
// pose_rmse 큼  + tracking_rmse 큼  → EKF 문제가 제어에 전파됨
// pose_rmse 작음 + tracking_rmse 큼  → EKF는 정상, MPC 튜닝 문제

namespace control_mpc
{

class TrackingRmseNode : public rclcpp::Node
{
public:
  explicit TrackingRmseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tracking_rmse_node", options),
    has_gt_(false),
    has_ref_(false),
    sample_count_(0)
  {
    // 슬라이딩 윈도우 크기 파라미터
    // 기본 100 → 50Hz 기준 최근 2초 구간 RMSE
    this->declare_parameter<int>("window_size", 100);
    window_size_ = this->get_parameter("window_size").as_int();

    // GT에서 추출할 로봇 이름 (TFMessage child_frame_id로 검색)
    this->declare_parameter<std::string>("robot_name", "amr_robot");
    robot_name_ = this->get_parameter("robot_name").as_string();

    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos   = rclcpp::SensorDataQoS();

    // --------------------------------------------------------
    // Gazebo Ground Truth 구독
    //   /world/simple_room/dynamic_pose/info → tf2_msgs/TFMessage
    //   simple_room_empty 월드에서도 토픽명은 simple_room으로 고정됨
    //   (bringup.launch.py에서 하드코딩된 브릿지 설정)
    //
    //   TFMessage.transforms 배열에서 child_frame_id == "amr_robot"인
    //   항목을 찾아서 gt_x_, gt_y_, gt_yaw_에 저장함.
    // --------------------------------------------------------
    gt_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/simple_room/dynamic_pose/info",
      sensor_qos,
      std::bind(&TrackingRmseNode::gtCallback, this, std::placeholders::_1)
    );

    // --------------------------------------------------------
    // MPC 목표 지점 구독
    //   mpc_node가 controlCallback()에서 매 주기 발행하는
    //   현재 추종 목표 상태 x_ref[0]을 PoseStamped 형태로 수신.
    //   frame_id = "map" (map frame 기준)
    // --------------------------------------------------------
    ref_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mpc/reference_pose",
      reliable_qos,
      std::bind(&TrackingRmseNode::refCallback, this, std::placeholders::_1)
    );

    // 추종 오차 발행 (/metrics/tracking_rmse)
    // amr_msgs/PoseRmse 재사용: rmse_x, rmse_y, rmse_yaw, rmse_total
    rmse_pub_ = this->create_publisher<amr_msgs::msg::PoseRmse>(
      "/metrics/tracking_rmse",
      reliable_qos
    );

    RCLCPP_INFO(this->get_logger(),
      "[TrackingRmseNode] 초기화 완료. robot_name='%s', window_size=%d",
      robot_name_.c_str(), window_size_);
    RCLCPP_INFO(this->get_logger(),
      "[TrackingRmseNode] GT: /world/simple_room/dynamic_pose/info");
    RCLCPP_INFO(this->get_logger(),
      "[TrackingRmseNode] Ref: /mpc/reference_pose");
    RCLCPP_INFO(this->get_logger(),
      "[TrackingRmseNode] 발행: /metrics/tracking_rmse");
  }

private:
  // --------------------------------------------------------
  // Ground Truth 콜백
  // TFMessage 배열에서 amr_robot 항목을 찾아 위치/yaw 캐싱.  
  // pose_rmse_node와 동일한 방식.
  // Gazebo는 월드 안의 모든 모델 포즈를 하나의 TFMessage 배열로 묶어서 발행. 
  // 배열에는 벽, 장애물, 로봇 등이 전부 들어있으므로 child_frame_id == "amr_robot"인 항목만 찾아서 캐싱.
  // --------------------------------------------------------
  void gtCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      if (tf.child_frame_id == robot_name_) {
        gt_x_ = tf.transform.translation.x;
        gt_y_ = tf.transform.translation.y;

        // 쿼터니언 → yaw 변환
        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w
        );
        // tf2::getYaw는 [-π, π] 범위로 반환함
        // GT yaw는 Gazebo 기준 정규화값이므로 그대로 사용
        gt_yaw_  = tf2::getYaw(q);
        has_gt_  = true;
        return;  // 찾았으면 순회 중단
      }
    }
  }

  // --------------------------------------------------------
  // MPC 목표 지점 콜백
  //   mpc_node에서 발행한 x_ref[0] (PoseStamped) 수신.
  //   GT가 준비된 경우 즉시 RMSE 계산 및 발행.
  //
  //   [타이밍 설계]
  //     GT 콜백: SensorDataQoS → 고빈도, 비신뢰성
  //     Ref 콜백: RELIABLE QoS → 50Hz, 신뢰성
  //     ref 수신 시점에 가장 최근 GT를 사용 (캐시 방식)
  // --------------------------------------------------------
  // 오차 계산, reference가 들어올 때 마다 캐시된 GT와 비교
  void refCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_gt_) {
      // GT 아직 미수신 → 스킵 (초기 구동 직후 발생 가능)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[TrackingRmseNode] GT 미수신. /world/simple_room/dynamic_pose/info 확인 요망.");
      return;
    }

    // --- 목표 지점(reference) 추출 ---
    const double ref_x = msg->pose.position.x;
    const double ref_y = msg->pose.position.y;

    // quaternion → yaw 변환
    tf2::Quaternion ref_q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w
    );
    // mpc_node는 연속 Yaw를 사용하지만, 발행 시 sin/cos으로 quaternion 변환했으므로
    // tf2::getYaw는 [-π, π] 범위의 값을 반환함.
    // 오차 계산 시 atan2(sin, cos) 정규화로 처리하므로 문제없음.
    const double ref_yaw = tf2::getYaw(ref_q);

    // --- 오차 계산 ---
    // GT (실제 위치) - Reference (목표 위치)
    // 부호 방향: 양수 = GT가 ref보다 앞서 있음
    const double err_x = gt_x_ - ref_x;
    const double err_y = gt_y_ - ref_y;

    // yaw 오차: [-π, π] 범위로 정규화
    // [왜 atan2(sin, cos) 트릭?]
    //   단순 빼기만 하면 -2π ~ 2π 범위가 됨.
    //   예: gt=170°, ref=-170° → 단순 차이=340° (실제는 -20°)
    //   atan2(sin(diff), cos(diff))는 항상 -π ~ π로 정규화
    const double raw_diff = gt_yaw_ - ref_yaw;
    const double err_yaw  = std::atan2(std::sin(raw_diff), std::cos(raw_diff));

    // 2D 유클리드 위치 오차 (방향 무관한 거리)
    const double err_total = std::sqrt(err_x * err_x + err_y * err_y);

    // --- 슬라이딩 윈도우 RMSE 갱신 ---
    // 오차²를 deque에 push, 윈도우 초과 시 앞에서 pop
    // pose_rmse_node와 동일한 람다 패턴 사용
    auto pushWindow = [&](std::deque<double> & buf, double err_sq) {
      buf.push_back(err_sq); // 새 오차² 추가 (뒤에서 push)
      if (static_cast<int>(buf.size()) > window_size_) {
        buf.pop_front(); // 가장 오래된 값 제거 (앞에서 pop)
      }
    };

    pushWindow(buf_x_,     err_x     * err_x); // x 오차²
    pushWindow(buf_y_,     err_y     * err_y); // y 오차²
    pushWindow(buf_yaw_,   err_yaw   * err_yaw); // yaw 오차²
    pushWindow(buf_total_, err_total * err_total);

    sample_count_++;

    // 최소 샘플 수 미달 시 발행 안 함 (초기 불안정 구간 제외)
    constexpr int MIN_SAMPLES = 10;
    if (sample_count_ < MIN_SAMPLES) {
      return;
    }

    // --- RMSE 계산 ---
    // RMSE = sqrt( Σ(err²) / N )
    auto calcRmse = [](const std::deque<double> & buf) -> double {
      double sum = 0.0;
      for (const auto & v : buf) { sum += v; } // Σ(err²)
      return std::sqrt(sum / static_cast<double>(buf.size()));
    };

    const double rmse_x     = calcRmse(buf_x_);
    const double rmse_y     = calcRmse(buf_y_);
    const double rmse_yaw   = calcRmse(buf_yaw_);
    const double rmse_total = calcRmse(buf_total_);

    // --- /metrics/tracking_rmse 발행 ---
    amr_msgs::msg::PoseRmse rmse_msg;
    rmse_msg.header.stamp    = this->now();
    rmse_msg.header.frame_id = "map";
    rmse_msg.rmse_x          = rmse_x;
    rmse_msg.rmse_y          = rmse_y;
    rmse_msg.rmse_yaw        = rmse_yaw;
    rmse_msg.rmse_total      = rmse_total;
    rmse_pub_->publish(rmse_msg);

    // 터미널 로그 (5초마다, 50Hz * 50 = 1초) 50 Hz 기준 50 샘플: 1초
    if (sample_count_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "[Tracking RMSE] x=%.4fm  y=%.4fm  yaw=%.4frad  total=%.4fm  (n=%d)",
        rmse_x, rmse_y, rmse_yaw, rmse_total, sample_count_);
    }
  }

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr              gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr       ref_sub_;
  rclcpp::Publisher<amr_msgs::msg::PoseRmse>::SharedPtr                  rmse_pub_;

  // --- Ground Truth 캐시 ---
  double gt_x_   = 0.0;
  double gt_y_   = 0.0;
  double gt_yaw_ = 0.0;
  bool   has_gt_;

  // --- 플래그 ---
  bool has_ref_;         // 최초 ref 수신 여부 (현재는 미사용, 확장용)

  // --- 상태 ---
  int         sample_count_;
  int         window_size_;
  std::string robot_name_;

  // --- 슬라이딩 윈도우 버퍼 (오차² 저장) ---
  std::deque<double> buf_x_;
  std::deque<double> buf_y_;
  std::deque<double> buf_yaw_;
  std::deque<double> buf_total_;
};

}  // namespace control_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::TrackingRmseNode>());
  rclcpp::shutdown();
  return 0;
}