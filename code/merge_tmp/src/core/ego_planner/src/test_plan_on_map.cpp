#include <cstdlib>
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
  const char * gr = std::getenv("GRID_RES");          // grid.resolution (default 0.1 m)
  map->init(gr ? std::atof(gr) : 0.1, 20.0, 20.0, inflate);
  if (!map->loadPriorMap(argv[1])) { std::cerr << "cannot load " << argv[1] << "\n"; return 1; }
  const auto & obs = map->getStaticObstacles();

  // Optional cones on the walked path (env): CONES_EVERY=m between cones, CONES_OFFSET=m sideways, or CONES_FILE
  // (alternating), CONE_INFLATE / CONE_SAFE / CONE_WEIGHT = cones.* parameters (0 weight: off).
  auto envd = [](const char * k, double d) { const char * v = std::getenv(k); return v ? std::atof(v) : d; };
  ConeField cones;
  cones.inflate_radius = envd("CONE_INFLATE", 0.30);
  cones.safe_dist = envd("CONE_SAFE", 0.60);
  cones.weight = envd("CONE_WEIGHT", 150.0);
  const double every = envd("CONES_EVERY", 0.0), offset = envd("CONES_OFFSET", 0.0);
  if (every > 0.0) {
    double acc = 0.0;
    int k = 0;
    for (size_t i = 1; i < traj.size(); ++i) {
      acc += (traj[i] - traj[i - 1]).norm();
      if (acc < every) continue;
      acc = 0.0;
      const Eigen::Vector2d dir = (traj[i] - traj[i - 1]).normalized();
      cones.centers.push_back(traj[i] + ((k++ % 2) ? offset : -offset) * Eigen::Vector2d(-dir.y(), dir.x()));
    }
    map->setCones(cones);
  }
  // Or real cone positions (env CONES_FILE: one "x,y" per line, map frame)
  if (const char * cf = std::getenv("CONES_FILE")) {
    std::ifstream cfs(cf);
    std::string cl;
    while (std::getline(cfs, cl)) {
      double x, y;
      if (std::sscanf(cl.c_str(), "%lf,%lf", &x, &y) == 2) cones.centers.emplace_back(x, y);
    }
    map->setCones(cones);
  }

  int n = 0, ok = 0, too_close = 0, f_astar = 0, f_valid = 0, f_short = 0, used_fallback = 0;
  std::vector<double> clear, cone_clear;
  int skipped_on_cone = 0;
  const size_t step = static_cast<size_t>(envd("PAIR_STEP", 5));
  for (size_t i = 0; i + gap < traj.size(); i += step) {
    const Eigen::Vector2d s = traj[i], g = traj[i + gap];
    if ((g - s).norm() < 1.0) continue;
    bool on_cone = false;   // start / goal on top of a cone: not a request the mission makes
    for (const auto & c : cones.centers) {
      on_cone |= (c - s).norm() < cones.inflate_radius + 0.1 || (c - g).norm() < cones.inflate_radius + 0.1;
    }
    if (on_cone) {++skipped_on_cone; continue;}
    ++n;
    map->updateStaticOnly(s);
    AStar astar; astar.init(map);
    BsplineOptimizer opt; opt.init(map, 1.0, lambda_obs, 2.0, max_vel, 1.2, safe);
    opt.setSolverParams(0.2, iters, 0.02, 0.04, 0.8);
    std::vector<Eigen::Vector2d> path;
    BsplineTrajectory bt;
    if (!astar.search(s, g, path)) { ++f_astar; continue; }
    if (!opt.optimizeWithContinuity(path, s, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), bt)) continue;
    if (const char * dump = std::getenv("DUMP_FILE")) {   // per pair: A* path and the optimised trajectory
      std::ofstream df(dump, std::ios::app);
      df << "pair " << n << " " << s.x() << " " << s.y() << " " << g.x() << " " << g.y() << "\n";
      for (const auto & q : path) df << "a " << q.x() << " " << q.y() << "\n";
      for (double t = 0; t <= bt.getTotalDuration(); t += 0.05) {
        const Eigen::Vector2d q = bt.evaluatePosition(t);
        df << "t " << q.x() << " " << q.y() << " " << (map->isInflated(q) ? 1 : 0) << "\n";
      }
    }
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
      for (const auto & c : cones.centers) {
        const double d = (c - p).norm();
        if ((c - s).norm() > 0.4 && (c - g).norm() > 0.4) cone_clear.push_back(d);  // not start/goal
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
  if (!cones.centers.empty()) {
    std::sort(cone_clear.begin(), cone_clear.end());
    std::printf("cones %zu (every %.1f m, offset %.2f) inflate %.2f safe %.2f weight %.0f | skipped (start/goal on a cone) %d | closest pass to a cone centre: min %.2f m\n",
      cones.centers.size(), every, offset, cones.inflate_radius, cones.safe_dist, cones.weight, skipped_on_cone,
      cone_clear.empty() ? 0.0 : cone_clear.front());
  }
  return 0;
}
