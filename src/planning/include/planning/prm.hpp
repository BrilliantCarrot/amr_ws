#ifndef PLANNING__PRM_HPP_
#define PLANNING__PRM_HPP_

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace planning
{

struct PRMParams
{
  int num_samples = 1200;
  int k_neighbors = 12;
  double connection_radius_cells = 20.0;
  double collision_check_step_cells = 1.0;
  unsigned int random_seed = 11;
};

class PRM
{
public:
  std::vector<std::pair<int, int>> findPath(
    const std::vector<int8_t> & grid,
    int width, int height,
    int sx, int sy,
    int gx, int gy,
    const PRMParams & params) const;

private:
  struct Node
  {
    double x{0.0};
    double y{0.0};
  };

  struct Edge
  {
    int to{0};
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
};

}  // namespace planning

#endif  // PLANNING__PRM_HPP_
