#include "planning/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace planning
{

bool RRTStar::isFree(
  const std::vector<int8_t> & grid,
  int width, int height,
  double x, double y) const
{
  const int gx = static_cast<int>(std::round(x));
  const int gy = static_cast<int>(std::round(y));
  if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
    return false;
  }
  const int8_t v = grid[gy * width + gx];
  return v >= 0 && v <= 50;
}

bool RRTStar::collisionFree(
  const std::vector<int8_t> & grid,
  int width, int height,
  double x0, double y0,
  double x1, double y1,
  double step_cells) const
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dist = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(
    std::ceil(dist / std::max(step_cells, 1e-3))));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    if (!isFree(grid, width, height, x0 + t * dx, y0 + t * dy)) {
      return false;
    }
  }
  return true;
}

int RRTStar::nearestNode(
  const std::vector<Node> & nodes,
  double x, double y) const
{
  int best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < nodes.size(); ++i) {
    const double dx = nodes[i].x - x;
    const double dy = nodes[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

std::vector<int> RRTStar::nearNodes(
  const std::vector<Node> & nodes,
  double x, double y,
  double radius_cells) const
{
  std::vector<int> result;
  const double r2 = radius_cells * radius_cells;
  for (size_t i = 0; i < nodes.size(); ++i) {
    const double dx = nodes[i].x - x;
    const double dy = nodes[i].y - y;
    if (dx * dx + dy * dy <= r2) {
      result.push_back(static_cast<int>(i));
    }
  }
  return result;
}

std::pair<double, double> RRTStar::steer(
  double from_x, double from_y,
  double to_x, double to_y,
  double step_cells) const
{
  const double dx = to_x - from_x;
  const double dy = to_y - from_y;
  const double dist = std::hypot(dx, dy);
  if (dist <= step_cells || dist < 1e-9) {
    return {to_x, to_y};
  }
  const double scale = step_cells / dist;
  return {from_x + dx * scale, from_y + dy * scale};
}

std::vector<std::pair<int, int>> RRTStar::findPath(
  const std::vector<int8_t> & grid,
  int width, int height,
  int sx, int sy,
  int gx, int gy,
  const RRTStarParams & params) const
{
  if (!isFree(grid, width, height, sx, sy) ||
      !isFree(grid, width, height, gx, gy)) {
    return {};
  }

  RRTStarParams p = params;
  p.max_iterations = std::max(1, p.max_iterations);
  p.step_size_cells = std::max(1.0, p.step_size_cells);
  p.goal_sample_rate = std::clamp(p.goal_sample_rate, 0.0, 1.0);
  p.goal_tolerance_cells = std::max(1.0, p.goal_tolerance_cells);
  p.rewire_radius_cells = std::max(p.step_size_cells, p.rewire_radius_cells);
  p.collision_check_step_cells = std::max(0.25, p.collision_check_step_cells);

  std::mt19937 rng(p.random_seed);
  std::uniform_real_distribution<double> x_dist(0.0, static_cast<double>(width - 1));
  std::uniform_real_distribution<double> y_dist(0.0, static_cast<double>(height - 1));
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

  std::vector<Node> nodes;
  nodes.reserve(static_cast<size_t>(p.max_iterations) + 2);
  nodes.push_back(Node{static_cast<double>(sx), static_cast<double>(sy), -1, 0.0});

  int best_goal_idx = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();

  auto tryGoalConnect = [&](int from_idx) {
    const Node & from = nodes[static_cast<size_t>(from_idx)];
    const double d_goal = std::hypot(from.x - gx, from.y - gy);
    if (d_goal > p.goal_tolerance_cells) {
      return;
    }
    if (!collisionFree(
        grid, width, height, from.x, from.y, gx, gy,
        p.collision_check_step_cells)) {
      return;
    }

    const double goal_cost = from.cost + d_goal;
    if (goal_cost >= best_goal_cost) {
      return;
    }

    Node goal_node;
    goal_node.x = static_cast<double>(gx);
    goal_node.y = static_cast<double>(gy);
    goal_node.parent = from_idx;
    goal_node.cost = goal_cost;
    nodes.push_back(goal_node);
    best_goal_idx = static_cast<int>(nodes.size()) - 1;
    best_goal_cost = goal_cost;
  };

  for (int iter = 0; iter < p.max_iterations; ++iter) {
    double sample_x = x_dist(rng);
    double sample_y = y_dist(rng);
    if (unit_dist(rng) < p.goal_sample_rate) {
      sample_x = static_cast<double>(gx);
      sample_y = static_cast<double>(gy);
    }

    if (!isFree(grid, width, height, sample_x, sample_y)) {
      continue;
    }

    const int nearest_idx = nearestNode(nodes, sample_x, sample_y);
    auto [new_x, new_y] = steer(
      nodes[static_cast<size_t>(nearest_idx)].x,
      nodes[static_cast<size_t>(nearest_idx)].y,
      sample_x, sample_y,
      p.step_size_cells);

    if (!isFree(grid, width, height, new_x, new_y)) {
      continue;
    }
    if (!collisionFree(
        grid, width, height,
        nodes[static_cast<size_t>(nearest_idx)].x,
        nodes[static_cast<size_t>(nearest_idx)].y,
        new_x, new_y,
        p.collision_check_step_cells)) {
      continue;
    }

    Node new_node;
    new_node.x = new_x;
    new_node.y = new_y;
    new_node.parent = nearest_idx;
    new_node.cost = nodes[static_cast<size_t>(nearest_idx)].cost +
      std::hypot(
        new_x - nodes[static_cast<size_t>(nearest_idx)].x,
        new_y - nodes[static_cast<size_t>(nearest_idx)].y);

    const auto near = nearNodes(nodes, new_x, new_y, p.rewire_radius_cells);
    for (int near_idx : near) {
      const Node & candidate = nodes[static_cast<size_t>(near_idx)];
      if (!collisionFree(
          grid, width, height,
          candidate.x, candidate.y,
          new_x, new_y,
          p.collision_check_step_cells)) {
        continue;
      }

      const double candidate_cost =
        candidate.cost + std::hypot(new_x - candidate.x, new_y - candidate.y);
      if (candidate_cost < new_node.cost) {
        new_node.parent = near_idx;
        new_node.cost = candidate_cost;
      }
    }

    nodes.push_back(new_node);
    const int new_idx = static_cast<int>(nodes.size()) - 1;

    for (int near_idx : near) {
      if (near_idx == new_node.parent) {
        continue;
      }
      Node & candidate = nodes[static_cast<size_t>(near_idx)];
      const double rewired_cost =
        nodes[static_cast<size_t>(new_idx)].cost +
        std::hypot(candidate.x - new_x, candidate.y - new_y);
      if (rewired_cost >= candidate.cost) {
        continue;
      }
      if (!collisionFree(
          grid, width, height,
          new_x, new_y,
          candidate.x, candidate.y,
          p.collision_check_step_cells)) {
        continue;
      }

      candidate.parent = new_idx;
      candidate.cost = rewired_cost;
    }

    tryGoalConnect(new_idx);
  }

  if (best_goal_idx < 0) {
    const int nearest_goal_idx = nearestNode(nodes, gx, gy);
    if (collisionFree(
        grid, width, height,
        nodes[static_cast<size_t>(nearest_goal_idx)].x,
        nodes[static_cast<size_t>(nearest_goal_idx)].y,
        gx, gy,
        p.collision_check_step_cells)) {
      Node goal_node;
      goal_node.x = static_cast<double>(gx);
      goal_node.y = static_cast<double>(gy);
      goal_node.parent = nearest_goal_idx;
      goal_node.cost = nodes[static_cast<size_t>(nearest_goal_idx)].cost +
        std::hypot(nodes[static_cast<size_t>(nearest_goal_idx)].x - gx,
                   nodes[static_cast<size_t>(nearest_goal_idx)].y - gy);
      nodes.push_back(goal_node);
      best_goal_idx = static_cast<int>(nodes.size()) - 1;
    }
  }

  if (best_goal_idx < 0) {
    return {};
  }

  std::vector<std::pair<int, int>> path;
  int idx = best_goal_idx;
  while (idx >= 0) {
    const Node & n = nodes[static_cast<size_t>(idx)];
    const int px = std::clamp(static_cast<int>(std::round(n.x)), 0, width - 1);
    const int py = std::clamp(static_cast<int>(std::round(n.y)), 0, height - 1);
    if (path.empty() || path.back().first != px || path.back().second != py) {
      path.emplace_back(px, py);
    }
    idx = n.parent;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace planning
