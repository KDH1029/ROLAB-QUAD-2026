#ifndef EGO_PLANNER__A_STAR_HPP_
#define EGO_PLANNER__A_STAR_HPP_

#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <Eigen/Core>
#include "ego_planner/grid_map.hpp"

namespace ego_planner
{

struct Node2D
{
  int x, y;
  double g_cost;
  double f_cost;
  std::shared_ptr<Node2D> parent;

  Node2D(int x_, int y_, double g_, double f_, std::shared_ptr<Node2D> p_ = nullptr)
  : x(x_), y(y_), g_cost(g_), f_cost(f_), parent(p_) {}
};

struct NodeCompare
{
  bool operator()(const std::shared_ptr<Node2D> & a, const std::shared_ptr<Node2D> & b) const {
    return a->f_cost > b->f_cost;
  }
};

class AStar
{
public:
  AStar() = default;
  ~AStar() = default;

  void init(const std::shared_ptr<GridMap> & grid_map);
  // max_iters: node expansions before giving up; min_partial_progress: accept the closest node
  // as a partial path only if it is at least this far (m, path cost) from the start
  void setSearchParams(int max_iters, double min_partial_progress)
  {
    max_iters_ = max_iters;
    min_partial_progress_ = min_partial_progress;
  }
  bool search(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
              std::vector<Eigen::Vector2d> & path);

private:
  std::shared_ptr<GridMap> grid_map_;
  int max_iters_{15000};
  double min_partial_progress_{0.8};

  double heuristic(int x1, int y1, int x2, int y2) const;
  void simplifyPath(const std::vector<Eigen::Vector2d> & raw_path,
                    std::vector<Eigen::Vector2d> & smooth_path) const;
  bool isLineFree(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2) const;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__A_STAR_HPP_

