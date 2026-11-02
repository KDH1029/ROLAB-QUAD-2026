#include "ego_planner/a_star.hpp"
#include <cmath>

namespace ego_planner
{

void AStar::init(const std::shared_ptr<GridMap> & grid_map)
{
  grid_map_ = grid_map;
}

double AStar::heuristic(int x1, int y1, int x2, int y2) const
{
  const double dx = std::abs(x1 - x2);
  const double dy = std::abs(y1 - y2);
  const double res = grid_map_->getResolution();
  // Octile distance
  return (std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy)) * res;
}

bool AStar::isLineFree(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2) const
{
  const double dist = (p2 - p1).norm();
  const double step = grid_map_->getResolution() * 0.5;
  const int n = static_cast<int>(std::ceil(dist / step));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / std::max(1, n);
    const Eigen::Vector2d pt = (1.0 - t) * p1 + t * p2;
    if (grid_map_->isInflated(pt)) {
      return false;
    }
  }
  return true;
}

void AStar::simplifyPath(const std::vector<Eigen::Vector2d> & raw_path,
                         std::vector<Eigen::Vector2d> & smooth_path) const
{
  smooth_path.clear();
  if (raw_path.empty()) return;
  if (raw_path.size() <= 2) {
    smooth_path = raw_path;
    return;
  }

  smooth_path.push_back(raw_path.front());
  size_t cur = 0;
  while (cur < raw_path.size() - 1) {
    size_t next = raw_path.size() - 1;
    while (next > cur + 1) {
      if (isLineFree(raw_path[cur], raw_path[next])) {
        break;
      }
      --next;
    }
    smooth_path.push_back(raw_path[next]);
    cur = next;
  }
}

bool AStar::search(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
                   std::vector<Eigen::Vector2d> & path)
{
  path.clear();
  if (!grid_map_) return false;

  int sx, sy, gx, gy;
  grid_map_->posToIndex(start, sx, sy);
  grid_map_->posToIndex(goal, gx, gy);

  if (!grid_map_->isInMap(sx, sy) || !grid_map_->isInMap(gx, gy)) {
    return false;
  }

  // If start is inside inflated zone, find nearest free cell (up to 15 cells = 0.75m)
  if (grid_map_->isInflated(sx, sy)) {
    bool found_start_alt = false;
    for (int r = 1; r <= 15 && !found_start_alt; ++r) {
      for (int dx = -r; dx <= r && !found_start_alt; ++dx) {
        for (int dy = -r; dy <= r && !found_start_alt; ++dy) {
          int nx = sx + dx;
          int ny = sy + dy;
          if (grid_map_->isInMap(nx, ny) && !grid_map_->isInflated(nx, ny)) {
            sx = nx;
            sy = ny;
            found_start_alt = true;
          }
        }
      }
    }
  }

  // If goal is inside obstacle/inflated zone, project to nearest free cell (up to 24 cells = 1.20m)
  if (grid_map_->isInflated(gx, gy)) {
    bool found_alt = false;
    for (int r = 1; r <= 24 && !found_alt; ++r) {
      for (int dx = -r; dx <= r && !found_alt; ++dx) {
        for (int dy = -r; dy <= r && !found_alt; ++dy) {
          int nx = gx + dx;
          int ny = gy + dy;
          if (grid_map_->isInMap(nx, ny) && !grid_map_->isInflated(nx, ny)) {
            gx = nx;
            gy = ny;
            found_alt = true;
          }
        }
      }
    }
    if (!found_alt) return false;
  }

  std::priority_queue<std::shared_ptr<Node2D>, std::vector<std::shared_ptr<Node2D>>, NodeCompare> open_list;
  std::unordered_map<int, double> g_score;

  const int size_x = grid_map_->getSizeX();
  auto to_id = [size_x](int x, int y) { return y * size_x + x; };

  auto start_node = std::make_shared<Node2D>(sx, sy, 0.0, heuristic(sx, sy, gx, gy));
  open_list.push(start_node);
  g_score[to_id(sx, sy)] = 0.0;

  const int dxs[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, 1, -1, -1};
  const double weights[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};
  const double res = grid_map_->getResolution();

  std::shared_ptr<Node2D> goal_node = nullptr;
  std::shared_ptr<Node2D> closest_node = nullptr;
  double min_h_dist = 1e9;
  int iters = 0;
  const int max_iters = max_iters_;

  while (!open_list.empty() && ++iters < max_iters) {
    auto current = open_list.top();
    open_list.pop();

    double h_curr = heuristic(current->x, current->y, gx, gy);
    if (h_curr < min_h_dist) {
      min_h_dist = h_curr;
      closest_node = current;
    }

    if (current->x == gx && current->y == gy) {
      goal_node = current;
      break;
    }

    int cur_id = to_id(current->x, current->y);
    if (current->g_cost > g_score[cur_id]) {
      continue;
    }

    for (int i = 0; i < 8; ++i) {
      int nx = current->x + dxs[i];
      int ny = current->y + dys[i];

      if (!grid_map_->isInMap(nx, ny)) continue;
      if (grid_map_->isInflated(nx, ny)) continue;

      double tentative_g = current->g_cost + weights[i] * res;
      int n_id = to_id(nx, ny);

      auto it = g_score.find(n_id);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[n_id] = tentative_g;
        double f = tentative_g + heuristic(nx, ny, gx, gy);
        open_list.push(std::make_shared<Node2D>(nx, ny, tentative_g, f, current));
      }
    }
  }

  if (!goal_node) {
    // Best-effort fallback: if exact goal wasn't reached but we made forward progress (>0.8m),
    // navigate as far forward along the obstacle-free corridor as possible.
    if (closest_node && closest_node->g_cost > min_partial_progress_) {
      goal_node = closest_node;
    } else {
      return false;
    }
  }

  // Reconstruct path
  std::vector<Eigen::Vector2d> raw_path;
  auto curr = goal_node;
  while (curr) {
    Eigen::Vector2d pt;
    grid_map_->indexToPos(curr->x, curr->y, pt);
    raw_path.push_back(pt);
    curr = curr->parent;
  }
  std::reverse(raw_path.begin(), raw_path.end());

  simplifyPath(raw_path, path);
  return true;
}

}  // namespace ego_planner

