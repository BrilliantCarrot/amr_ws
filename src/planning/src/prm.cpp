#include "planning/prm.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace planning
{

bool PRM::isFree(
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

bool PRM::collisionFree(
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

std::vector<std::pair<int, int>> PRM::findPath(
  const std::vector<int8_t> & grid,
  int width, int height,
  int sx, int sy,
  int gx, int gy,
  const PRMParams & params) const
{
  if (!isFree(grid, width, height, sx, sy) ||
      !isFree(grid, width, height, gx, gy)) {
    return {};
  }

  PRMParams p = params;
  p.num_samples = std::max(1, p.num_samples);
  p.k_neighbors = std::max(1, p.k_neighbors);
  p.connection_radius_cells = std::max(1.0, p.connection_radius_cells);
  p.collision_check_step_cells = std::max(0.25, p.collision_check_step_cells);

  std::mt19937 rng(p.random_seed);
  std::uniform_real_distribution<double> x_dist(0.0, static_cast<double>(width - 1));
  std::uniform_real_distribution<double> y_dist(0.0, static_cast<double>(height - 1));

  std::vector<Node> nodes;
  nodes.reserve(static_cast<size_t>(p.num_samples) + 2);
  nodes.push_back(Node{static_cast<double>(sx), static_cast<double>(sy)});
  nodes.push_back(Node{static_cast<double>(gx), static_cast<double>(gy)});

  int attempts = 0;
  const int max_attempts = std::max(p.num_samples * 20, p.num_samples + 100);
  while (static_cast<int>(nodes.size()) < p.num_samples + 2 && attempts < max_attempts) {
    ++attempts;
    const double x = x_dist(rng);
    const double y = y_dist(rng);
    if (!isFree(grid, width, height, x, y)) {
      continue;
    }
    nodes.push_back(Node{x, y});
  }

  if (nodes.size() < 2) {
    return {};
  }

  std::vector<std::vector<Edge>> graph(nodes.size());
  const double r2 = p.connection_radius_cells * p.connection_radius_cells;

  for (size_t i = 0; i < nodes.size(); ++i) {
    std::vector<std::pair<double, int>> candidates;
    candidates.reserve(nodes.size() - 1);

    for (size_t j = 0; j < nodes.size(); ++j) {
      if (i == j) {
        continue;
      }
      const double dx = nodes[i].x - nodes[j].x;
      const double dy = nodes[i].y - nodes[j].y;
      const double d2 = dx * dx + dy * dy;
      if (d2 <= r2) {
        candidates.emplace_back(d2, static_cast<int>(j));
      }
    }

    std::sort(candidates.begin(), candidates.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });

    const int n_connect = std::min(
      static_cast<int>(candidates.size()), p.k_neighbors);
    for (int c = 0; c < n_connect; ++c) {
      const int j = candidates[static_cast<size_t>(c)].second;

      const double dist = std::sqrt(candidates[static_cast<size_t>(c)].first);
      if (!collisionFree(
          grid, width, height,
          nodes[i].x, nodes[i].y,
          nodes[static_cast<size_t>(j)].x, nodes[static_cast<size_t>(j)].y,
          p.collision_check_step_cells)) {
        continue;
      }

      graph[i].push_back(Edge{j, dist});
      graph[static_cast<size_t>(j)].push_back(Edge{static_cast<int>(i), dist});
    }
  }

  const int start_idx = 0;
  const int goal_idx = 1;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g(nodes.size(), inf);
  std::vector<int> parent(nodes.size(), -1);

  using QItem = std::pair<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  g[static_cast<size_t>(start_idx)] = 0.0;
  open.emplace(0.0, start_idx);

  while (!open.empty()) {
    const auto [cost, idx] = open.top();
    open.pop();

    if (cost > g[static_cast<size_t>(idx)] + 1e-9) {
      continue;
    }
    if (idx == goal_idx) {
      break;
    }

    for (const auto & edge : graph[static_cast<size_t>(idx)]) {
      const double next_cost = cost + edge.cost;
      if (next_cost < g[static_cast<size_t>(edge.to)]) {
        g[static_cast<size_t>(edge.to)] = next_cost;
        parent[static_cast<size_t>(edge.to)] = idx;
        open.emplace(next_cost, edge.to);
      }
    }
  }

  if (!std::isfinite(g[static_cast<size_t>(goal_idx)])) {
    return {};
  }

  std::vector<std::pair<int, int>> path;
  for (int idx = goal_idx; idx >= 0; idx = parent[static_cast<size_t>(idx)]) {
    const Node & n = nodes[static_cast<size_t>(idx)];
    const int px = std::clamp(static_cast<int>(std::round(n.x)), 0, width - 1);
    const int py = std::clamp(static_cast<int>(std::round(n.y)), 0, height - 1);
    if (path.empty() || path.back().first != px || path.back().second != py) {
      path.emplace_back(px, py);
    }
    if (idx == start_idx) {
      break;
    }
  }

  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace planning
