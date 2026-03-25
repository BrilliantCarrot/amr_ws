#include <rclcpp/rclcpp.hpp>
#include "control_mpc/mpc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_mpc::MpcNode>());
  rclcpp::shutdown();
  return 0;
}