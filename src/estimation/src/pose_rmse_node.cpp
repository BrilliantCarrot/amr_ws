#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <amr_msgs/msg/pose_rmse.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>
#include <deque>
#include <string>

// ============================================================
// pose_rmse_node.cpp — Ground Truth vs EKF RMSE 실시간 계산 노드
//
// [Gazebo Fortress의 포즈 발행 방식]
//   Fortress는 개별 모델 포즈를 따로 발행하지 않고,
//   /world/<world_name>/pose/info 토픽 하나에
//   모든 모델의 포즈를 Pose_V(포즈 배열) 형태로 묶어서 발행함.
//
//   브리지를 통해 ROS2에서는 tf2_msgs/TFMessage 타입으로 수신됨.
//   TFMessage.transforms 배열에서 child_frame_id == "amr_robot"인
//   항목을 찾아서 Ground Truth로 사용함.
//
// [구독/발행 토픽]
//   구독: /world/simple_room/pose/info → TFMessage (모든 모델 포즈)
//   구독: /ekf/odom                    → EKF 추정값
//   발행: /metrics/pose_rmse           → RMSE 결과
// ============================================================

namespace estimation
{

class PoseRmseNode : public rclcpp::Node
{
public:
  explicit PoseRmseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pose_rmse_node", options),
    has_gt_(false),
    sample_count_(0)
  {
    // RMSE 슬라이딩 윈도우 크기
    // window_size=100, 50Hz 기준 → 최근 2초 구간의 RMSE
    this->declare_parameter<int>("window_size", 100);
    window_size_ = this->get_parameter("window_size").as_int();

    // 추출할 로봇 이름 (TFMessage에서 child_frame_id로 검색)
    this->declare_parameter<std::string>("robot_name", "amr_robot");
    robot_name_ = this->get_parameter("robot_name").as_string();

    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos   = rclcpp::SensorDataQoS();

    // --------------------------------------------------------
    // Ground Truth 구독
    //   /world/simple_room/pose/info → tf2_msgs/TFMessage
    //   TFMessage.transforms: 월드 내 모든 엔티티의 변환 배열
    //   각 항목의 child_frame_id가 모델/링크 이름에 해당함.
    // --------------------------------------------------------
    gt_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/simple_room/dynamic_pose/info",
      sensor_qos,
      std::bind(&PoseRmseNode::gtCallback, this, std::placeholders::_1)
    );

    // EKF 추정값 구독
    ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ekf/odom",
      reliable_qos,
      std::bind(&PoseRmseNode::ekfCallback, this, std::placeholders::_1)
    );

    // RMSE 결과 발행
    rmse_pub_ = this->create_publisher<amr_msgs::msg::PoseRmse>(
      "/metrics/pose_rmse",
      reliable_qos
    );

    RCLCPP_INFO(this->get_logger(),
      "[PoseRmseNode] 초기화 완료. robot_name='%s', window_size=%d",
      robot_name_.c_str(), window_size_);
  }

private:
  // --------------------------------------------------------
  // Ground Truth 콜백
  //   TFMessage에서 amr_robot의 포즈만 추출해서 캐싱.
  //
  //   [TFMessage 구조]
  //     transforms[]:
  //       header.frame_id    : 부모 프레임 (보통 "world")
  //       child_frame_id     : 자식 프레임 (모델/링크 이름)
  //       transform.translation: 위치 (x, y, z)
  //       transform.rotation:    자세 (쿼터니언)
  //
  //   child_frame_id == robot_name_ 인 항목을 찾아서 저장.
  // --------------------------------------------------------
  void gtCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      if (tf.child_frame_id == robot_name_) {
        gt_x_ = tf.transform.translation.x;
        gt_y_ = tf.transform.translation.y;

        // 쿼터니언에서 yaw 추출
        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w
        );
        gt_yaw_ = tf2::getYaw(q);
        has_gt_ = true;
        return;  // 찾았으면 배열 순회 중단
      }
    }
  }

  // --------------------------------------------------------
  // EKF 콜백
  //   EKF 추정값이 올 때마다 Ground Truth와 비교해서 RMSE 계산.
  // --------------------------------------------------------
  void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_gt_) {
      return;  // Ground Truth 수신 전이면 스킵
    }

    // EKF 추정 위치
    const double ekf_x = msg->pose.pose.position.x;
    const double ekf_y = msg->pose.pose.position.y;

    // EKF yaw 추출
    tf2::Quaternion ekf_q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    const double ekf_yaw = tf2::getYaw(ekf_q);

    // 오차 계산
    const double err_x = gt_x_ - ekf_x;
    const double err_y = gt_y_ - ekf_y;

    // yaw 오차 정규화 (-π ~ π)
    // [왜 atan2(sin, cos) 트릭을 쓰나?]
    //   단순 빼기(gt_yaw - ekf_yaw)만 하면 -2π ~ 2π 범위가 됨.
    //   예: gt=170°, ekf=-170° → 차이=340° (실제로는 -20°)
    //   atan2(sin(diff), cos(diff))는 항상 -π ~ π로 정규화해줌.
    const double raw_diff = gt_yaw_ - ekf_yaw;
    const double err_yaw  = std::atan2(std::sin(raw_diff), std::cos(raw_diff));

    // 2D 유클리드 위치 오차
    const double err_total = std::sqrt(err_x * err_x + err_y * err_y);

    // --------------------------------------------------------
    // 슬라이딩 윈도우 RMSE 계산
    //   deque 앞에서 오래된 샘플 제거, 뒤에서 새 샘플 추가.
    //   항상 최근 window_size_ 개 샘플만 유지.
    // --------------------------------------------------------
    auto pushWindow = [&](std::deque<double> & buf, double err_sq) {
      buf.push_back(err_sq);
      if (static_cast<int>(buf.size()) > window_size_) {
        buf.pop_front();
      }
    };

    pushWindow(buf_x_,     err_x    * err_x);
    pushWindow(buf_y_,     err_y    * err_y);
    pushWindow(buf_yaw_,   err_yaw  * err_yaw);
    pushWindow(buf_total_, err_total * err_total);

    sample_count_++;

    // 최소 샘플 수 미달 시 발행 안 함 (초기 불안정 구간 제외)
    constexpr int MIN_SAMPLES = 10;
    if (sample_count_ < MIN_SAMPLES) {
      return;
    }

    // RMSE = sqrt( sum(err²) / N )
    auto calcRmse = [](const std::deque<double> & buf) -> double {
      double sum = 0.0;
      for (const auto & v : buf) { sum += v; }
      return std::sqrt(sum / static_cast<double>(buf.size()));
    };

    const double rmse_x     = calcRmse(buf_x_);
    const double rmse_y     = calcRmse(buf_y_);
    const double rmse_yaw   = calcRmse(buf_yaw_);
    const double rmse_total = calcRmse(buf_total_);

    // RMSE 메시지 발행
    amr_msgs::msg::PoseRmse rmse_msg;
    rmse_msg.rmse_x     = rmse_x;
    rmse_msg.rmse_y     = rmse_y;
    rmse_msg.rmse_yaw   = rmse_yaw;
    rmse_msg.rmse_total = rmse_total;
    rmse_pub_->publish(rmse_msg);

    // 5초마다 터미널 로그 출력 (50Hz * 250 = 5초)
    if (sample_count_ % 250 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "[RMSE] x=%.4fm  y=%.4fm  yaw=%.4frad  total=%.4fm  (n=%d)",
        rmse_x, rmse_y, rmse_yaw, rmse_total, sample_count_);
    }
  }

  // --- ROS2 인터페이스 ---
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr    gt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     ekf_sub_;
  rclcpp::Publisher<amr_msgs::msg::PoseRmse>::SharedPtr        rmse_pub_;

  // --- Ground Truth 캐시 ---
  double gt_x_   = 0.0;
  double gt_y_   = 0.0;
  double gt_yaw_ = 0.0;
  bool   has_gt_;

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

}  // namespace estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimation::PoseRmseNode>());
  rclcpp::shutdown();
  return 0;
}