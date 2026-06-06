// ============================================================
// camera_lidar_fusion_node.cpp — W5 Camera-LiDAR obstacle fusion node
//
// [역할]
//   LiDAR obstacle_tracker_node가 만든 map frame 장애물과,
//   W4 camera_obstacle_node가 만든 base_link frame 카메라 장애물을 합쳐서
//   MPC/CBF가 바로 사용할 수 있는 /obstacles/fused 를 발행함.
//
// [입력]
//   /obstacles/detected        : amr_msgs/ObstacleArray, map frame, LiDAR 기반
//   /obstacles/camera_detected : geometry_msgs/PoseArray, 보통 base_link frame, 카메라 기반
//
// [출력]
//   /obstacles/fused           : amr_msgs/ObstacleArray, map frame
//
// [설계 의도]
//   - LiDAR는 실제 공간의 geometry 확인 담당임.
//   - Camera/YOLO는 사람/물체 같은 semantic 후보를 제공함.
//   - 현재 PoseArray에는 class/confidence가 없으므로, W5 초기 버전에서는
//     카메라 obstacle을 사람 후보로 보고 더 큰 radius를 적용함.
//   - 기본값은 camera-only obstacle을 사용하지 않음.
//     즉, LiDAR가 확인한 장애물에 카메라가 겹칠 때만 반경을 키움.
//     반사벽 YOLO false positive 때문에 안전한 기본값임.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <amr_msgs/msg/obstacle_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace control_mpc
{

struct SimpleObstacle
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
};

class CameraLidarFusionNode : public rclcpp::Node
{
public:
  CameraLidarFusionNode()
  : Node("camera_lidar_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── Topic / frame 파라미터 ───────────────────────────────
    this->declare_parameter("lidar_obstacle_topic", std::string("/obstacles/detected"));
    this->declare_parameter("camera_obstacle_topic", std::string("/obstacles/camera_detected"));
    this->declare_parameter("fused_obstacle_topic", std::string("/obstacles/fused"));
    this->declare_parameter("target_frame", std::string("map"));

    // ── Fusion 파라미터 ─────────────────────────────────────
    this->declare_parameter("publish_rate", 20.0);
    this->declare_parameter("lidar_timeout", 0.5);
    this->declare_parameter("camera_timeout", 0.5);
    this->declare_parameter("fusion_match_dist", 0.60);
    this->declare_parameter("camera_radius", 0.50);
    this->declare_parameter("camera_only_enabled", false);
    this->declare_parameter("camera_forward_min", 0.05);
    this->declare_parameter("camera_forward_max", 6.0);
    this->declare_parameter("camera_lateral_max", 3.0);

    lidar_topic_  = this->get_parameter("lidar_obstacle_topic").as_string();
    camera_topic_ = this->get_parameter("camera_obstacle_topic").as_string();
    fused_topic_  = this->get_parameter("fused_obstacle_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();

    publish_rate_        = this->get_parameter("publish_rate").as_double();
    lidar_timeout_       = this->get_parameter("lidar_timeout").as_double();
    camera_timeout_      = this->get_parameter("camera_timeout").as_double();
    fusion_match_dist_   = this->get_parameter("fusion_match_dist").as_double();
    camera_radius_       = this->get_parameter("camera_radius").as_double();
    camera_only_enabled_ = this->get_parameter("camera_only_enabled").as_bool();
    camera_forward_min_  = this->get_parameter("camera_forward_min").as_double();
    camera_forward_max_  = this->get_parameter("camera_forward_max").as_double();
    camera_lateral_max_  = this->get_parameter("camera_lateral_max").as_double();

    lidar_sub_ = this->create_subscription<amr_msgs::msg::ObstacleArray>(
      lidar_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&CameraLidarFusionNode::lidarCallback, this, std::placeholders::_1));

    camera_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      camera_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&CameraLidarFusionNode::cameraCallback, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<amr_msgs::msg::ObstacleArray>(
      fused_topic_, rclcpp::QoS(10).reliable());

    const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_rate_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraLidarFusionNode::publishFused, this));

    RCLCPP_INFO(this->get_logger(),
      "Camera-LiDAR Fusion 시작 | lidar=%s camera=%s fused=%s target=%s "
      "match=%.2fm camera_r=%.2fm camera_only=%s",
      lidar_topic_.c_str(), camera_topic_.c_str(), fused_topic_.c_str(),
      target_frame_.c_str(), fusion_match_dist_, camera_radius_,
      camera_only_enabled_ ? "true" : "false");
  }

private:
  void lidarCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_lidar_ = *msg;
    last_lidar_time_ = this->now();
    has_lidar_ = true;
  }

  void cameraCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_camera_ = *msg;
    last_camera_time_ = this->now();
    has_camera_ = true;
  }

  std::vector<SimpleObstacle> convertLidarToList(const amr_msgs::msg::ObstacleArray & msg) const
  {
    std::vector<SimpleObstacle> out;
    const int n = std::min({
      msg.count,
      static_cast<int>(msg.x.size()),
      static_cast<int>(msg.y.size()),
      static_cast<int>(msg.vx.size()),
      static_cast<int>(msg.vy.size()),
      static_cast<int>(msg.radius.size())});

    out.reserve(std::max(0, n));
    for (int i = 0; i < n; ++i) {
      SimpleObstacle obs;
      obs.x = msg.x[i];
      obs.y = msg.y[i];
      obs.vx = msg.vx[i];
      obs.vy = msg.vy[i];
      obs.radius = msg.radius[i];
      out.push_back(obs);
    }
    return out;
  }

  bool cameraPoseToMap(
    const geometry_msgs::msg::PoseArray & camera_msg,
    const geometry_msgs::msg::Pose & pose_in,
    geometry_msgs::msg::PoseStamped & pose_map)
  {
    geometry_msgs::msg::PoseStamped pose_src;
    pose_src.header = camera_msg.header;
    pose_src.pose = pose_in;

    // 카메라 obstacle은 보통 base_link frame임.
    // stamp가 0이면 최신 TF를 쓰도록 TimePointZero로 조회함.
    try {
      geometry_msgs::msg::TransformStamped tf;
      if (pose_src.header.stamp.sec == 0 && pose_src.header.stamp.nanosec == 0) {
        tf = tf_buffer_.lookupTransform(
          target_frame_, pose_src.header.frame_id, tf2::TimePointZero);
      } else {
        tf = tf_buffer_.lookupTransform(
          target_frame_, pose_src.header.frame_id, tf2::TimePointZero);
      }
      tf2::doTransform(pose_src, pose_map, tf);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "camera obstacle TF 변환 실패: %s", ex.what());
      return false;
    }
  }

  void mergeCameraObstacles(
    const geometry_msgs::msg::PoseArray & camera_msg,
    std::vector<SimpleObstacle> & fused)
  {
    for (const auto & pose : camera_msg.poses) {
      // base_link 기준 1차 필터임.
      // 너무 뒤쪽/멀리/옆쪽 검출은 회피 입력에서 제외함.
      if (pose.position.x < camera_forward_min_ || pose.position.x > camera_forward_max_) {
        continue;
      }
      if (std::abs(pose.position.y) > camera_lateral_max_) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose_map;
      if (!cameraPoseToMap(camera_msg, pose, pose_map)) {
        continue;
      }

      const double cx = pose_map.pose.position.x;
      const double cy = pose_map.pose.position.y;

      int best_idx = -1;
      double best_dist = fusion_match_dist_;
      for (size_t i = 0; i < fused.size(); ++i) {
        const double d = std::hypot(cx - fused[i].x, cy - fused[i].y);
        if (d < best_dist) {
          best_dist = d;
          best_idx = static_cast<int>(i);
        }
      }

      if (best_idx >= 0) {
        // LiDAR가 확인한 장애물에 카메라 semantic 후보가 겹침.
        // 위치는 LiDAR를 유지하고, 반경만 사람 후보 기준으로 키움.
        fused[best_idx].radius = std::max(fused[best_idx].radius, camera_radius_);
      } else if (camera_only_enabled_) {
        // 디버그/확장용 옵션임.
        // 반사 false positive가 문제면 false 유지 권장함.
        SimpleObstacle obs;
        obs.x = cx;
        obs.y = cy;
        obs.vx = 0.0;
        obs.vy = 0.0;
        obs.radius = camera_radius_;
        fused.push_back(obs);
      }
    }
  }

  void publishFused()
  {
    amr_msgs::msg::ObstacleArray lidar_copy;
    geometry_msgs::msg::PoseArray camera_copy;
    bool use_lidar = false;
    bool use_camera = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = this->now();
      use_lidar = has_lidar_ && ((now - last_lidar_time_).seconds() <= lidar_timeout_);
      use_camera = has_camera_ && ((now - last_camera_time_).seconds() <= camera_timeout_);
      if (use_lidar) lidar_copy = latest_lidar_;
      if (use_camera) camera_copy = latest_camera_;
    }

    std::vector<SimpleObstacle> fused;
    if (use_lidar) {
      fused = convertLidarToList(lidar_copy);
    }
    if (use_camera && !camera_copy.header.frame_id.empty()) {
      mergeCameraObstacles(camera_copy, fused);
    }

    amr_msgs::msg::ObstacleArray out;
    out.header.stamp = this->now();
    out.header.frame_id = target_frame_;
    out.count = static_cast<int>(fused.size());

    for (const auto & obs : fused) {
      out.x.push_back(obs.x);
      out.y.push_back(obs.y);
      out.vx.push_back(obs.vx);
      out.vy.push_back(obs.vy);
      out.radius.push_back(obs.radius);
    }

    fused_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[Fusion] lidar=%s camera=%s fused=%d",
      use_lidar ? "ON" : "OFF",
      use_camera ? "ON" : "OFF",
      out.count);
  }

  std::mutex mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<amr_msgs::msg::ObstacleArray>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr camera_sub_;
  rclcpp::Publisher<amr_msgs::msg::ObstacleArray>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  amr_msgs::msg::ObstacleArray latest_lidar_;
  geometry_msgs::msg::PoseArray latest_camera_;
  rclcpp::Time last_lidar_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_time_{0, 0, RCL_ROS_TIME};
  bool has_lidar_{false};
  bool has_camera_{false};

  std::string lidar_topic_;
  std::string camera_topic_;
  std::string fused_topic_;
  std::string target_frame_;

  double publish_rate_{20.0};
  double lidar_timeout_{0.5};
  double camera_timeout_{0.5};
  double fusion_match_dist_{0.60};
  double camera_radius_{0.50};
  bool camera_only_enabled_{false};
  double camera_forward_min_{0.05};
  double camera_forward_max_{6.0};
  double camera_lateral_max_{3.0};
};

}  // namespace control_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::CameraLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
