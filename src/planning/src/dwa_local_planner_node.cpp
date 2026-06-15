#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <amr_msgs/msg/obstacle_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planning
{
namespace
{

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

geometry_msgs::msg::PoseStamped makePose(
  const rclcpp::Time & stamp,
  double x,
  double y,
  double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = stamp;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;
  p.pose.orientation.z = std::sin(yaw * 0.5);
  p.pose.orientation.w = std::cos(yaw * 0.5);
  return p;
}

}  // namespace

class DwaLocalPlannerNode : public rclcpp::Node
{
public:
  explicit DwaLocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dwa_local_planner_node", options)
  {
    declare_parameter("global_path_topic", std::string("/planned_path"));
    declare_parameter("local_path_topic", std::string("/local_path"));
    declare_parameter("odom_topic", std::string("/map_ekf/odom"));
    declare_parameter("obstacle_topic", std::string("/obstacles/detected"));
    declare_parameter("map_topic", std::string("/map"));

    declare_parameter("plan_rate_hz", 10.0);
    declare_parameter("sim_time", 1.6);
    declare_parameter("sim_dt", 0.10);
    declare_parameter("v_min", 0.0);
    declare_parameter("v_max", 0.18);
    declare_parameter("w_max", 0.85);
    declare_parameter("acc_lim_v", 0.45);
    declare_parameter("acc_lim_w", 1.8);
    declare_parameter("use_dynamic_window", false);
    declare_parameter("v_samples", 7);
    declare_parameter("w_samples", 15);
    declare_parameter("lookahead_dist", 1.20);
    declare_parameter("path_weight", 5.0);
    declare_parameter("goal_weight", 2.0);
    declare_parameter("heading_weight", 0.8);
    declare_parameter("path_heading_weight", 1.8);
    declare_parameter("turn_weight", 0.9);
    declare_parameter("w_smooth_weight", 0.8);
    declare_parameter("heading_change_weight", 0.7);
    declare_parameter("obstacle_weight", 0.7);
    declare_parameter("speed_weight", 0.4);
    declare_parameter("obstacle_safety_margin", 0.22);
    declare_parameter("obstacle_timeout_sec", 0.80);
    declare_parameter("map_collision_check", true);
    declare_parameter("robot_radius", 0.45);
    declare_parameter("map_safety_margin", 0.08);
    declare_parameter("map_clearance_weight", 3.0);
    declare_parameter("map_clearance_desired", 0.45);
    declare_parameter("map_clearance_max_search", 0.90);
    declare_parameter("occupied_threshold", 50);
    declare_parameter("unknown_as_occupied", false);
    declare_parameter("min_progress_dist", 0.08);
    declare_parameter("near_goal_dist", 0.35);
    declare_parameter("publish_fallback_global_segment", true);

    global_path_topic_ = get_parameter("global_path_topic").as_string();
    local_path_topic_ = get_parameter("local_path_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    obstacle_topic_ = get_parameter("obstacle_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();

    plan_rate_hz_ = get_parameter("plan_rate_hz").as_double();
    sim_time_ = get_parameter("sim_time").as_double();
    sim_dt_ = get_parameter("sim_dt").as_double();
    v_min_ = get_parameter("v_min").as_double();
    v_max_ = get_parameter("v_max").as_double();
    w_max_ = get_parameter("w_max").as_double();
    acc_lim_v_ = get_parameter("acc_lim_v").as_double();
    acc_lim_w_ = get_parameter("acc_lim_w").as_double();
    use_dynamic_window_ = get_parameter("use_dynamic_window").as_bool();
    v_samples_ = std::max(1, static_cast<int>(get_parameter("v_samples").as_int()));
    w_samples_ = std::max(1, static_cast<int>(get_parameter("w_samples").as_int()));
    lookahead_dist_ = get_parameter("lookahead_dist").as_double();
    path_weight_ = get_parameter("path_weight").as_double();
    goal_weight_ = get_parameter("goal_weight").as_double();
    heading_weight_ = get_parameter("heading_weight").as_double();
    path_heading_weight_ = get_parameter("path_heading_weight").as_double();
    turn_weight_ = get_parameter("turn_weight").as_double();
    w_smooth_weight_ = get_parameter("w_smooth_weight").as_double();
    heading_change_weight_ = get_parameter("heading_change_weight").as_double();
    obstacle_weight_ = get_parameter("obstacle_weight").as_double();
    speed_weight_ = get_parameter("speed_weight").as_double();
    obstacle_safety_margin_ = get_parameter("obstacle_safety_margin").as_double();
    obstacle_timeout_sec_ = get_parameter("obstacle_timeout_sec").as_double();
    map_collision_check_ = get_parameter("map_collision_check").as_bool();
    robot_radius_ = get_parameter("robot_radius").as_double();
    map_safety_margin_ = get_parameter("map_safety_margin").as_double();
    map_clearance_weight_ = get_parameter("map_clearance_weight").as_double();
    map_clearance_desired_ = get_parameter("map_clearance_desired").as_double();
    map_clearance_max_search_ = get_parameter("map_clearance_max_search").as_double();
    occupied_threshold_ =
      static_cast<int>(get_parameter("occupied_threshold").as_int());
    unknown_as_occupied_ = get_parameter("unknown_as_occupied").as_bool();
    min_progress_dist_ = get_parameter("min_progress_dist").as_double();
    near_goal_dist_ = get_parameter("near_goal_dist").as_double();
    publish_fallback_global_segment_ =
      get_parameter("publish_fallback_global_segment").as_bool();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DwaLocalPlannerNode::pathCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DwaLocalPlannerNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(&DwaLocalPlannerNode::odomCallback, this, std::placeholders::_1));

    obs_sub_ = create_subscription<amr_msgs::msg::ObstacleArray>(
      obstacle_topic_,
      rclcpp::QoS(10),
      std::bind(&DwaLocalPlannerNode::obstacleCallback, this, std::placeholders::_1));

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic_,
      rclcpp::QoS(1).reliable().transient_local());

    const double period_sec = 1.0 / std::max(plan_rate_hz_, 1.0);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_sec * 1000.0)),
      std::bind(&DwaLocalPlannerNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "[DWA] started | global=%s local=%s odom=%s obs=%s map=%s sim=%.2fs dt=%.2fs "
      "v=[%.2f, %.2f] w=±%.2f samples=%dx%d map_collision=%d r=%.2f",
      global_path_topic_.c_str(), local_path_topic_.c_str(), odom_topic_.c_str(),
      obstacle_topic_.c_str(), map_topic_.c_str(), sim_time_, sim_dt_, v_min_, v_max_,
      w_max_, v_samples_, w_samples_, map_collision_check_ ? 1 : 0, robot_radius_);
  }

private:
  struct Obstacle
  {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    double radius{0.0};
  };

  struct State
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_global_path_ = *msg;
    has_path_ = !latest_global_path_.poses.empty();
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = *msg;
    has_map_ = latest_map_.info.width > 0 && latest_map_.info.height > 0 &&
      !latest_map_.data.empty();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    cur_x_ = msg->pose.pose.position.x;
    cur_y_ = msg->pose.pose.position.y;
    cur_yaw_ = yawFromQuat(msg->pose.pose.orientation);
    cur_v_ = msg->twist.twist.linear.x;
    cur_w_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void obstacleCallback(const amr_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    obstacles_.clear();
    const size_t n = std::min<size_t>(
      static_cast<size_t>(std::max(msg->count, 0)),
      std::min({msg->x.size(), msg->y.size(), msg->vx.size(), msg->vy.size(), msg->radius.size()}));
    obstacles_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      Obstacle obs;
      obs.x = msg->x[i];
      obs.y = msg->y[i];
      obs.vx = msg->vx[i];
      obs.vy = msg->vy[i];
      obs.radius = msg->radius[i];
      obstacles_.push_back(obs);
    }
    last_obstacle_stamp_ = now();
    has_obstacles_ = true;
  }

  void timerCallback()
  {
    if (!has_path_ || !has_odom_) {
      return;
    }

    const auto stamp = now();
    const int closest_idx = findClosestPathIndex(cur_x_, cur_y_);
    const int target_idx = findLookaheadIndex(closest_idx, lookahead_dist_);
    const auto & target_pose = latest_global_path_.poses[target_idx].pose.position;
    const double target_x = target_pose.x;
    const double target_y = target_pose.y;
    const auto & final_pose = latest_global_path_.poses.back().pose.position;
    const double dist_to_global_goal = std::hypot(final_pose.x - cur_x_, final_pose.y - cur_y_);
    const bool near_global_goal = dist_to_global_goal <= near_goal_dist_;

    const bool obstacles_fresh =
      has_obstacles_ && (stamp - last_obstacle_stamp_).seconds() <= obstacle_timeout_sec_;
    const auto & obs = obstacles_fresh ? obstacles_ : empty_obstacles_;

    std::vector<State> best_traj;
    double best_score = std::numeric_limits<double>::infinity();
    double best_w = 0.0;

    const double v_low = use_dynamic_window_ ?
      std::max(v_min_, cur_v_ - acc_lim_v_ * sim_dt_) : v_min_;
    const double v_high = use_dynamic_window_ ?
      std::min(v_max_, cur_v_ + acc_lim_v_ * sim_dt_) : v_max_;
    const double w_low = use_dynamic_window_ ?
      std::max(-w_max_, cur_w_ - acc_lim_w_ * sim_dt_) : -w_max_;
    const double w_high = use_dynamic_window_ ?
      std::min(w_max_, cur_w_ + acc_lim_w_ * sim_dt_) : w_max_;

    for (int vi = 0; vi < v_samples_; ++vi) {
      const double v = sampleBetween(v_low, v_high, vi, v_samples_);
      for (int wi = 0; wi < w_samples_; ++wi) {
        const double w = sampleBetween(w_low, w_high, wi, w_samples_);
        std::vector<State> traj;
        double min_clearance = std::numeric_limits<double>::infinity();
        const bool collision = simulateTrajectory(v, w, obs, traj, min_clearance);
        if (collision || traj.empty()) {
          continue;
        }

        const auto & end = traj.back();
        const double progress =
          std::hypot(cur_x_ - target_x, cur_y_ - target_y) -
          std::hypot(end.x - target_x, end.y - target_y);
        if (!near_global_goal && progress < min_progress_dist_) {
          continue;
        }

        const double path_dist = averagePathDistance(traj);
        const double goal_dist = std::hypot(end.x - target_x, end.y - target_y);
        const double target_heading = std::atan2(target_y - end.y, target_x - end.x);
        const double heading_err = std::abs(normalizeAngle(target_heading - end.yaw));
        const double path_heading_err =
          std::abs(normalizeAngle(pathHeadingAtIndex(target_idx) - end.yaw));
        const double heading_change = std::abs(normalizeAngle(end.yaw - cur_yaw_));
        const double obs_cost = std::isfinite(min_clearance) ?
          1.0 / std::max(min_clearance, 0.05) : 0.0;
        const double map_clearance = trajectoryMapClearance(traj);
        const double map_clearance_gap = std::max(
          0.0, map_clearance_desired_ - map_clearance) /
          std::max(map_clearance_desired_, 1e-3);
        const double map_clearance_cost = map_clearance_gap * map_clearance_gap;
        const double w_smooth_cost = has_last_selected_w_ ?
          std::abs(w - last_selected_w_) : 0.0;

        const double score =
          path_weight_ * path_dist +
          goal_weight_ * goal_dist +
          heading_weight_ * heading_err +
          path_heading_weight_ * path_heading_err +
          turn_weight_ * std::abs(w) +
          w_smooth_weight_ * w_smooth_cost +
          heading_change_weight_ * heading_change +
          obstacle_weight_ * obs_cost +
          map_clearance_weight_ * map_clearance_cost -
          speed_weight_ * v;

        if (score < best_score) {
          best_score = score;
          best_w = w;
          best_traj = std::move(traj);
        }
      }
    }

    if (!best_traj.empty()) {
      last_selected_w_ = best_w;
      has_last_selected_w_ = true;
      publishTrajectory(best_traj, target_idx, stamp);
      return;
    }

    if (publish_fallback_global_segment_) {
      publishGlobalSegment(closest_idx, stamp);
    } else {
      publishStopPath(stamp);
    }
  }

  static double sampleBetween(double low, double high, int idx, int count)
  {
    if (count <= 1 || std::abs(high - low) < 1e-9) {
      return 0.5 * (low + high);
    }
    const double ratio = static_cast<double>(idx) / static_cast<double>(count - 1);
    return low + ratio * (high - low);
  }

  bool simulateTrajectory(
    double v,
    double w,
    const std::vector<Obstacle> & obs,
    std::vector<State> & traj,
    double & min_clearance) const
  {
    State s{cur_x_, cur_y_, cur_yaw_};
    const int steps = std::max(2, static_cast<int>(std::ceil(sim_time_ / sim_dt_)));
    traj.reserve(static_cast<size_t>(steps + 1));
    traj.push_back(s);

    for (int k = 1; k <= steps; ++k) {
      s.x += v * std::cos(s.yaw) * sim_dt_;
      s.y += v * std::sin(s.yaw) * sim_dt_;
      s.yaw = normalizeAngle(s.yaw + w * sim_dt_);
      traj.push_back(s);

      if (isMapCollision(s.x, s.y)) {
        return true;
      }

      const double t = static_cast<double>(k) * sim_dt_;
      for (const auto & o : obs) {
        const double ox = o.x + o.vx * t;
        const double oy = o.y + o.vy * t;
        const double clearance =
          std::hypot(s.x - ox, s.y - oy) - (o.radius + obstacle_safety_margin_);
        min_clearance = std::min(min_clearance, clearance);
        if (clearance <= 0.0) {
          return true;
        }
      }
    }

    return false;
  }

  bool isMapCollision(double x, double y) const
  {
    if (!map_collision_check_ || !has_map_) {
      return false;
    }

    int mx = 0;
    int my = 0;
    if (!worldToMap(x, y, mx, my)) {
      return true;
    }

    const double resolution = latest_map_.info.resolution;
    const int radius_cells = std::max(
      1, static_cast<int>(std::ceil((robot_radius_ + map_safety_margin_) / resolution)));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int cx = mx + dx;
        const int cy = my + dy;
        if (cx < 0 || cy < 0 ||
            cx >= static_cast<int>(latest_map_.info.width) ||
            cy >= static_cast<int>(latest_map_.info.height)) {
          return true;
        }
        const int idx = cy * static_cast<int>(latest_map_.info.width) + cx;
        const int8_t occ = latest_map_.data[static_cast<size_t>(idx)];
        if (occ < 0) {
          if (unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        if (occ >= occupied_threshold_) {
          return true;
        }
      }
    }

    return false;
  }

  double trajectoryMapClearance(const std::vector<State> & traj) const
  {
    if (!map_collision_check_ || !has_map_) {
      return map_clearance_desired_;
    }

    double min_clearance = map_clearance_max_search_;
    for (size_t i = 0; i < traj.size(); i += 2) {
      min_clearance = std::min(min_clearance, mapClearance(traj[i].x, traj[i].y));
    }
    return min_clearance;
  }

  double mapClearance(double x, double y) const
  {
    int mx = 0;
    int my = 0;
    if (!worldToMap(x, y, mx, my)) {
      return 0.0;
    }

    const double resolution = latest_map_.info.resolution;
    if (resolution <= 0.0) {
      return 0.0;
    }

    const int search_cells = std::max(
      1, static_cast<int>(std::ceil(map_clearance_max_search_ / resolution)));
    double best_center_dist = std::numeric_limits<double>::infinity();

    for (int dy = -search_cells; dy <= search_cells; ++dy) {
      for (int dx = -search_cells; dx <= search_cells; ++dx) {
        const int cx = mx + dx;
        const int cy = my + dy;
        const double center_dist = std::hypot(dx, dy) * resolution;
        if (center_dist > map_clearance_max_search_) {
          continue;
        }

        if (cx < 0 || cy < 0 ||
            cx >= static_cast<int>(latest_map_.info.width) ||
            cy >= static_cast<int>(latest_map_.info.height)) {
          best_center_dist = std::min(best_center_dist, center_dist);
          continue;
        }

        const int idx = cy * static_cast<int>(latest_map_.info.width) + cx;
        const int8_t occ = latest_map_.data[static_cast<size_t>(idx)];
        const bool occupied =
          occ >= occupied_threshold_ || (occ < 0 && unknown_as_occupied_);
        if (occupied) {
          best_center_dist = std::min(best_center_dist, center_dist);
        }
      }
    }

    if (!std::isfinite(best_center_dist)) {
      return map_clearance_max_search_;
    }
    return std::max(0.0, best_center_dist - robot_radius_);
  }

  bool worldToMap(double x, double y, int & mx, int & my) const
  {
    const double origin_x = latest_map_.info.origin.position.x;
    const double origin_y = latest_map_.info.origin.position.y;
    const double resolution = latest_map_.info.resolution;
    if (resolution <= 0.0) {
      return false;
    }

    mx = static_cast<int>(std::floor((x - origin_x) / resolution));
    my = static_cast<int>(std::floor((y - origin_y) / resolution));
    return mx >= 0 && my >= 0 &&
      mx < static_cast<int>(latest_map_.info.width) &&
      my < static_cast<int>(latest_map_.info.height);
  }

  int findClosestPathIndex(double x, double y) const
  {
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < latest_global_path_.poses.size(); ++i) {
      const auto & p = latest_global_path_.poses[i].pose.position;
      const double d = std::hypot(x - p.x, y - p.y);
      if (d < best_dist) {
        best_dist = d;
        best_idx = static_cast<int>(i);
      }
    }
    return best_idx;
  }

  int findLookaheadIndex(int start_idx, double lookahead) const
  {
    if (latest_global_path_.poses.empty()) {
      return 0;
    }

    double accum = 0.0;
    int idx = std::clamp(start_idx, 0, static_cast<int>(latest_global_path_.poses.size()) - 1);
    for (size_t i = static_cast<size_t>(idx + 1); i < latest_global_path_.poses.size(); ++i) {
      const auto & a = latest_global_path_.poses[i - 1].pose.position;
      const auto & b = latest_global_path_.poses[i].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      idx = static_cast<int>(i);
      if (accum >= lookahead) {
        break;
      }
    }
    return idx;
  }

  double pathHeadingAtIndex(int idx) const
  {
    if (latest_global_path_.poses.size() < 2) {
      return cur_yaw_;
    }

    const int last_idx = static_cast<int>(latest_global_path_.poses.size()) - 1;
    const int i = std::clamp(idx, 0, last_idx);
    const int a_idx = (i < last_idx) ? i : i - 1;
    const int b_idx = (i < last_idx) ? i + 1 : i;
    const auto & a = latest_global_path_.poses[static_cast<size_t>(a_idx)].pose.position;
    const auto & b = latest_global_path_.poses[static_cast<size_t>(b_idx)].pose.position;
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  double averagePathDistance(const std::vector<State> & traj) const
  {
    if (traj.empty() || latest_global_path_.poses.empty()) {
      return 0.0;
    }

    double sum = 0.0;
    int count = 0;
    for (size_t i = 0; i < traj.size(); i += 2) {
      sum += distanceToPath(traj[i].x, traj[i].y);
      ++count;
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
  }

  double distanceToPath(double x, double y) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & pose : latest_global_path_.poses) {
      const auto & p = pose.pose.position;
      best = std::min(best, std::hypot(x - p.x, y - p.y));
    }
    return best;
  }

  void publishTrajectory(
    const std::vector<State> & traj,
    int global_tail_start_idx,
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    path.poses.reserve(traj.size() + latest_global_path_.poses.size());
    for (const auto & s : traj) {
      path.poses.push_back(makePose(stamp, s.x, s.y, s.yaw));
    }
    appendGlobalTail(path, global_tail_start_idx);
    local_path_pub_->publish(path);
  }

  void publishGlobalSegment(int start_idx, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";

    path.poses.push_back(makePose(stamp, cur_x_, cur_y_, cur_yaw_));
    appendGlobalTail(path, start_idx);
    local_path_pub_->publish(path);
  }

  void publishStopPath(const rclcpp::Time & stamp)
  {
    std::vector<State> traj;
    traj.push_back(State{cur_x_, cur_y_, cur_yaw_});
    traj.push_back(State{cur_x_, cur_y_, cur_yaw_});
    publishTrajectory(traj, findClosestPathIndex(cur_x_, cur_y_), stamp);
  }

  void appendGlobalTail(nav_msgs::msg::Path & path, int start_idx) const
  {
    if (latest_global_path_.poses.empty()) {
      return;
    }
    const int last_idx = static_cast<int>(latest_global_path_.poses.size()) - 1;
    const int first_idx = std::clamp(start_idx, 0, last_idx);
    for (int i = first_idx; i <= last_idx; ++i) {
      const auto & pose = latest_global_path_.poses[static_cast<size_t>(i)].pose.position;
      double yaw = 0.0;
      if (i < last_idx) {
        const auto & next = latest_global_path_.poses[static_cast<size_t>(i + 1)].pose.position;
        yaw = std::atan2(next.y - pose.y, next.x - pose.x);
      } else if (!path.poses.empty()) {
        const auto & prev = path.poses.back().pose.position;
        yaw = std::atan2(pose.y - prev.y, pose.x - prev.x);
      } else {
        yaw = yawFromQuat(latest_global_path_.poses[static_cast<size_t>(i)].pose.orientation);
      }
      path.poses.push_back(makePose(stampFromPath(path), pose.x, pose.y, yaw));
    }
  }

  rclcpp::Time stampFromPath(const nav_msgs::msg::Path & path) const
  {
    return rclcpp::Time(path.header.stamp);
  }

  std::string global_path_topic_;
  std::string local_path_topic_;
  std::string odom_topic_;
  std::string obstacle_topic_;
  std::string map_topic_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<amr_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path latest_global_path_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  std::vector<Obstacle> obstacles_;
  const std::vector<Obstacle> empty_obstacles_;
  rclcpp::Time last_obstacle_stamp_{0, 0, RCL_ROS_TIME};

  bool has_path_{false};
  bool has_map_{false};
  bool has_odom_{false};
  bool has_obstacles_{false};
  double cur_x_{0.0};
  double cur_y_{0.0};
  double cur_yaw_{0.0};
  double cur_v_{0.0};
  double cur_w_{0.0};

  double plan_rate_hz_{10.0};
  double sim_time_{1.6};
  double sim_dt_{0.10};
  double v_min_{0.0};
  double v_max_{0.18};
  double w_max_{0.85};
  double acc_lim_v_{0.45};
  double acc_lim_w_{1.8};
  bool use_dynamic_window_{false};
  int v_samples_{7};
  int w_samples_{15};
  double lookahead_dist_{1.20};
  double path_weight_{5.0};
  double goal_weight_{2.0};
  double heading_weight_{0.8};
  double path_heading_weight_{1.8};
  double turn_weight_{0.9};
  double w_smooth_weight_{0.8};
  double heading_change_weight_{0.7};
  double obstacle_weight_{0.7};
  double speed_weight_{0.4};
  double obstacle_safety_margin_{0.22};
  double obstacle_timeout_sec_{0.80};
  bool map_collision_check_{true};
  double robot_radius_{0.45};
  double map_safety_margin_{0.08};
  double map_clearance_weight_{3.0};
  double map_clearance_desired_{0.45};
  double map_clearance_max_search_{0.90};
  int occupied_threshold_{50};
  bool unknown_as_occupied_{false};
  double min_progress_dist_{0.08};
  double near_goal_dist_{0.35};
  bool publish_fallback_global_segment_{true};
  bool has_last_selected_w_{false};
  double last_selected_w_{0.0};
};

}  // namespace planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning::DwaLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
