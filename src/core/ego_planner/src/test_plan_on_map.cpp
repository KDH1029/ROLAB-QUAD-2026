// Offline planner check on a recorded map: plan between pairs of poses the robot actually
// walked, report success rate and minimum wall clearance of the result. No ROS needed.
// Usage: test_plan_on_map <obstacles.bin> <trajectory.csv> <inflate> <safe_dist> <validate 0|1>
//        [max_vel] [pair_gap_poses]
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "ego_planner/a_star.hpp"
#include "ego_planner/bspline_optimizer.hpp"
#include "ego_planner/grid_map.hpp"

using namespace ego_planner;

int main(int argc, char ** argv)
{
  if (argc < 6) {
    std::cerr << "usage: test_plan_on_map obstacles.bin traj.csv inflate safe_dist validate [max_vel] [gap]\n";
    return 1;
  }
  const double inflate = std::stod(argv[3]), safe = std::stod(argv[4]);
  const bool validate = std::stoi(argv[5]) != 0;
  const double max_vel = argc > 6 ? std::stod(argv[6]) : 0.8;
  const int gap = argc > 7 ? std::stoi(argv[7]) : 30;
  const double lambda_obs = argc > 8 ? std::stod(argv[8]) : 30.0;
  const int iters = argc > 9 ? std::stoi(argv[9]) : 35;
  const bool fallback = argc > 10 ? std::stoi(argv[10]) != 0 : false;

  std::vector<Eigen::Vector2d> traj;
  std::ifstream ifs(argv[2]);
  std::string line;
  while (std::getline(ifs, line)) {
    double x, y, z;
    if (std::sscanf(line.c_str(), "%lf,%lf,%lf", &x, &y, &z) == 3) traj.emplace_back(x, y);
  }

  auto map = std::make_shared<GridMap>();
  map->init(0.1, 20.0, 20.0, inflate);
  if (!map->loadPriorMap(argv[1])) { std::cerr << "cannot load " << argv[1] << "\n"; return 1; }
  const auto & obs = map->getStaticObstacles();

  int n = 0, ok = 0, too_close = 0, f_astar = 0, f_valid = 0, f_short = 0, used_fallback = 0;
  std::vector<double> clear;
  for (size_t i = 0; i + gap < traj.size(); i += 5) {
    const Eigen::Vector2d s = traj[i], g = traj[i + gap];
    if ((g - s).norm() < 1.0) continue;
    ++n;
    map->updateStaticOnly(s);
    AStar astar; astar.init(map);
    BsplineOptimizer opt; opt.init(map, 1.0, lambda_obs, 2.0, max_vel, 1.2, safe);
    opt.setSolverParams(0.2, iters, 0.02, 0.04, 0.8);
    std::vector<Eigen::Vector2d> path;
    BsplineTrajectory bt;
    if (!astar.search(s, g, path)) { ++f_astar; continue; }
    if (!opt.optimizeWithContinuity(path, s, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), bt)) continue;
    if (validate && !opt.isCollisionFree(bt, 0.05)) {
      // Fallback: same control points without optimisation (stays on the A* corridor)
      BsplineOptimizer raw; raw.init(map, 1.0, lambda_obs, 2.0, max_vel, 1.2, safe);
      raw.setSolverParams(0.2, 0, 0.02, 0.04, 0.8);
      if (fallback && raw.optimizeWithContinuity(path, s, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), bt) &&
        raw.isCollisionFree(bt, 0.05)) {
        ++used_fallback;
      } else { ++f_valid; continue; }
    }
    // Count as success only if the trajectory actually ends near the goal (A* may return partial)
    if ((bt.evaluatePosition(bt.getTotalDuration()) - g).norm() > 0.3) { ++f_short; continue; }
    ++ok;
    double m = 1e9;
    for (double t = 0; t <= bt.getTotalDuration(); t += 0.05) {
      const Eigen::Vector2d p = bt.evaluatePosition(t);
      for (const auto & o : obs) {
        const double d = (o - p).squaredNorm();
        if (d < m) m = d;
      }
    }
    m = std::sqrt(m);
    clear.push_back(m);
    if (m < 0.20) ++too_close;   // Go2 half width 0.155 m + 5 cm
  }
  std::sort(clear.begin(), clear.end());
  auto pct = [&](double q) { return clear.empty() ? 0.0 : clear[std::min(clear.size() - 1, size_t(q * clear.size()))]; };
  std::printf("inflate %.2f safe %.2f lam_obs %.0f iters %d validate %d vmax %.1f | success %d/%d (%.0f%%) | min clearance: min %.2f p10 %.2f median %.2f m | <0.20 m: %d | fail: A* %d, cuts inflation %d, partial %d | fallback used %d\n",
    inflate, safe, lambda_obs, iters, validate, max_vel, ok, n, 100.0 * ok / std::max(1, n), pct(0.0), pct(0.1), pct(0.5), too_close, f_astar, f_valid, f_short, used_fallback);
  return 0;
}
