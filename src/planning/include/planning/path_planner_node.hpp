#ifndef PLANNING__PATH_PLANNER_NODE_HPP_
#define PLANNING__PATH_PLANNER_NODE_HPP_

// ============================================================
// path_planner_node.hpp — 전역 경로계획 ROS2 노드 헤더
//
// [설계 원칙]
//   SingleThreadedExecutor 전용 — mutex 없음
//   모든 콜백이 순차 실행되므로 멤버변수 직접 접근 안전
//
// [재계획 트리거]
//   1. 주기 타이머: replan_period_sec 마다
//   2. 이벤트: 동적 장애물 경로 근접 시 (쿨다운 0.5초)
//   3. 즉시: /goal_pose 수신 시
// ============================================================

#include <vector>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <amr_msgs/msg/obstacle_array.hpp>

#include "planning/astar.hpp"
#include <amr_msgs/msg/localization_status.hpp>

namespace planning
{

class PathPlannerNode : public rclcpp::Node
{
public:
  explicit PathPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 콜백
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void obstacleCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg);
  void replanTimerCallback();

  // 경로 계획 파이프라인
  bool plan();

  // 스무딩
  std::vector<std::pair<int, int>> smoothPath(
    const std::vector<int8_t> & inflated_grid, int w, int h,
    const std::vector<std::pair<int, int>> & raw_path) const;

  // 재샘플링
  std::vector<geometry_msgs::msg::PoseStamped> resamplePath(
    const std::vector<std::pair<int, int>> & smooth_path,
    double origin_x, double origin_y, double resolution,
    double spacing_m) const;

  // 동적 장애물 근접 판정
  bool isObstacleNearPath(const amr_msgs::msg::ObstacleArray & obs) const;

  // Bresenham LOS
  bool hasLineOfSight(
    const std::vector<int8_t> & grid, int w, int h,
    int x0, int y0, int x1, int y1) const;

  // ROS2 인터페이스
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr   map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<amr_msgs::msg::ObstacleArray>::SharedPtr   obs_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
  rclcpp::TimerBase::SharedPtr                                     replan_timer_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr loc_sub_;
  void locCallback(const amr_msgs::msg::LocalizationStatus::SharedPtr msg);
  uint8_t loc_status_{0};  // 0=NORMAL, 1=DEGRADED, 2=LOST

  // 상태 변수 (SingleThread → mutex 불필요)
  nav_msgs::msg::OccupancyGrid map_;
  double cur_x_{0.0}, cur_y_{0.0};
  double goal_x_{0.0}, goal_y_{0.0};

  bool has_map_{false};
  bool has_odom_{false};
  bool has_goal_{false};
  bool initial_plan_done_{false};

  std::vector<geometry_msgs::msg::PoseStamped> current_path_;
  amr_msgs::msg::ObstacleArray latest_obs_;
  bool has_obs_{false};

  // 파라미터
  double robot_radius_;
  double replan_period_sec_;
  double replan_obs_dist_;
  double wp_spacing_;
  double goal_tolerance_;
  double replan_cooldown_sec_{0.5};

  rclcpp::Time last_plan_time_;

  AStar astar_;
};

}  // namespace planning

#endif  // PLANNING__PATH_PLANNER_NODE_HPP_