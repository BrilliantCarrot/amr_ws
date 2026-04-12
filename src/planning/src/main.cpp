#include <rclcpp/rclcpp.hpp>
#include "planning/path_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning::PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
