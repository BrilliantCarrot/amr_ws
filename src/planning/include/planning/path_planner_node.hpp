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
#include <string>

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

// path planner 내부에서 일정 시간 유지하며 사용하는 장애물 표현임.
// /obstacles/detected 메시지를 그대로 쓰지 않고, hold/merge/inflation을 거친 뒤 A* grid에 마킹함.
struct PlannerObstacle
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
  rclcpp::Time last_seen;
};

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
  void pathVizTimerCallback();

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

  // 동적/근거리 장애물 처리
  void updateHeldObstacles(const amr_msgs::msg::ObstacleArray & obs);
  std::vector<PlannerObstacle> getMergedPlannerObstacles() const;
  void markPlannerObstacleOnGrid(
    std::vector<int8_t> & grid, int w, int h,
    double origin_x, double origin_y, double resolution,
    const PlannerObstacle & obs,
    double robot_radius_for_mark,
    double obstacle_margin_for_mark) const;

  // 장애물 근접 판정
  bool isObstacleNearPath(const std::vector<PlannerObstacle> & obs) const;

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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_viz_pub_;
  rclcpp::TimerBase::SharedPtr                                     replan_timer_;
  rclcpp::TimerBase::SharedPtr                                     path_viz_timer_;
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
  std::vector<PlannerObstacle> held_obstacles_;
  bool has_obs_{false};

  // 파라미터
  std::string odom_topic_{"/map_ekf/odom"};
  double robot_radius_;
  double replan_period_sec_;
  double replan_obs_dist_;
  double wp_spacing_;
  double goal_tolerance_;
  double replan_cooldown_sec_{1.20};

  // A*와 CBF 역할 분리 파라미터
  bool periodic_replan_enabled_{false};
  bool off_path_replan_enabled_{true};
  double off_path_replan_dist_{0.90};
  double off_path_replan_cooldown_sec_{5.00};
  bool planner_replan_on_lidar_obstacles_{true};
  std::string planner_lidar_astar_mode_{"all"};
  double planner_static_speed_threshold_{0.05};
  bool map_update_replan_enabled_{true};
  double map_update_replan_cooldown_sec_{2.00};
  double map_update_goal_stop_radius_{1.00};

  // 동적/근거리 장애물 경로계획 안정화 파라미터
  double planner_min_obstacle_radius_{0.50};
  double planner_obstacle_margin_{0.10};
  double planner_obstacle_hold_sec_{1.00};
  bool planner_use_velocity_filter_{false};
  bool planner_merge_obstacles_{true};
  double planner_obstacle_merge_distance_{0.75};
  double planner_merge_extra_radius_{0.10};
  double planner_max_merged_obstacle_radius_{0.80};

  // 피킹 스테이션 정밀 정차용 완화 파라미터.
  // 일반 구간은 기존 보수적 inflation을 유지하고, goal 주변에서만 더 작은
  // clearance를 허용해 fixed manipulator docking pose에 접근할 수 있게 함.
  bool docking_mode_{false};
  double docking_relax_radius_{1.00};
  double docking_robot_radius_{0.30};
  double docking_obstacle_margin_{0.03};
  double docking_goal_search_radius_{1.20};

  // A* + MPC 통합 시 /planned_path가 너무 자주 바뀌면 MPC reference가 계속 리셋되어 stop-and-go가 생김.
  // 아래 값들은 새 경로가 기존 경로와 크게 다를 때만 발행하도록 하는 hysteresis 파라미터임.
  double planner_publish_min_interval_sec_{1.20};
  double planner_publish_force_interval_sec_{4.00};
  double planner_path_avg_change_threshold_{0.12};
  double planner_path_max_change_threshold_{0.35};
  double planner_side_switch_lock_sec_{12.00};
  double planner_side_switch_min_score_{0.20};
  double planner_side_switch_clearance_{0.12};
  double planner_clearance_cost_radius_{0.35};
  double planner_clearance_cost_weight_{8.0};
  int smoothing_los_safety_cells_{2};
  double path_viz_republish_period_sec_{1.0};

  double distanceToCurrentPath(double x, double y) const;

  double computePathDifference(
    const std::vector<geometry_msgs::msg::PoseStamped> & new_path,
    const std::vector<geometry_msgs::msg::PoseStamped> & old_path,
    double & max_dist) const;

  bool shouldPublishPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & new_path) const;

  rclcpp::Time last_plan_time_;

  AStar astar_;
};

}  // namespace planning

#endif  // PLANNING__PATH_PLANNER_NODE_HPP_
