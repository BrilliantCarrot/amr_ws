#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace estimation
{

class OdomTfBroadcasterNode : public rclcpp::Node
{
public:
  explicit OdomTfBroadcasterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("odom_tf_broadcaster_node", options)
  {
    this->declare_parameter("odom_topic", std::string("/odom"));
    this->declare_parameter("parent_frame", std::string(""));
    this->declare_parameter("child_frame", std::string(""));

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    parent_frame_override_ = this->get_parameter("parent_frame").as_string();
    child_frame_override_ = this->get_parameter("child_frame").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(&OdomTfBroadcasterNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Odom TF broadcaster started: topic=%s parent_override=%s child_override=%s",
      odom_topic_.c_str(),
      parent_frame_override_.empty() ? "<from_msg>" : parent_frame_override_.c_str(),
      child_frame_override_.empty() ? "<from_msg>" : child_frame_override_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string parent_frame =
      parent_frame_override_.empty() ? msg->header.frame_id : parent_frame_override_;
    const std::string child_frame =
      child_frame_override_.empty() ? msg->child_frame_id : child_frame_override_;

    if (parent_frame.empty() || child_frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Cannot publish TF: empty parent or child frame in odometry message");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  std::string parent_frame_override_;
  std::string child_frame_override_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimation::OdomTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
