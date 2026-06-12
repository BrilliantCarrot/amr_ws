#ifndef PLANNING__RRT_STAR_HPP_
#define PLANNING__RRT_STAR_HPP_

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace planning
{

struct RRTStarParams
{
  int max_iterations = 3000;
  double step_size_cells = 5.0;
  double goal_sample_rate = 0.10;
  double goal_tolerance_cells = 5.0;
  double rewire_radius_cells = 12.0;
  double collision_check_step_cells = 1.0;
  unsigned int random_seed = 7;
};

class RRTStar
{
public:
  std::vector<std::pair<int, int>> findPath(
    const std::vector<int8_t> & grid,
    int width, int height,
    int sx, int sy,
    int gx, int gy,
    const RRTStarParams & params) const;

private:
  struct Node
  {
    double x{0.0};
    double y{0.0};
    int parent{-1};
    double cost{0.0};
  };

  bool isFree(
    const std::vector<int8_t> & grid,
    int width, int height,
    double x, double y) const;

  bool collisionFree(
    const std::vector<int8_t> & grid,
    int width, int height,
    double x0, double y0,
    double x1, double y1,
    double step_cells) const;

  int nearestNode(
    const std::vector<Node> & nodes,
    double x, double y) const;

  std::vector<int> nearNodes(
    const std::vector<Node> & nodes,
    double x, double y,
    double radius_cells) const;

  std::pair<double, double> steer(
    double from_x, double from_y,
    double to_x, double to_y,
    double step_cells) const;
};

}  // namespace planning

#endif  // PLANNING__RRT_STAR_HPP_
