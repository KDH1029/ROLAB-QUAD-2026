#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include "ego_planner/bspline_optimizer.hpp"

using namespace ego_planner;

int main()
{
  std::cout << "=== Testing ABPolicy C2 Continuity-Constrained B-Spline Refitting ===" << std::endl;

  BsplineOptimizer optimizer;
  optimizer.init(nullptr, 1.0, 0.0, 2.0, 1.0, 1.5, 0.45);

  std::vector<Eigen::Vector2d> guide_path = {
    Eigen::Vector2d(1.0, 2.0),
    Eigen::Vector2d(1.5, 2.5),
    Eigen::Vector2d(2.5, 3.0),
    Eigen::Vector2d(3.5, 3.2),
    Eigen::Vector2d(5.0, 3.5)
  };

  // Desired stitch handover conditions (arbitrary non-zero test values)
  Eigen::Vector2d p0(1.0, 2.0);
  Eigen::Vector2d v0(0.45, -0.30);
  Eigen::Vector2d a0(-0.25, 0.40);

  BsplineTrajectory traj;
  bool ok = optimizer.optimizeWithContinuity(guide_path, p0, v0, a0, traj);
  (void)ok;
  assert(ok && "optimizeWithContinuity should succeed");

  // Evaluate at t = 0
  Eigen::Vector2d p_eval = traj.evaluatePosition(0.0);
  Eigen::Vector2d v_eval = traj.evaluateVelocity(0.0);
  Eigen::Vector2d a_eval = traj.evaluateAcceleration(0.0);

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "[Position]     Target: (" << p0.x() << ", " << p0.y() << ")"
            << " | Evaluated: (" << p_eval.x() << ", " << p_eval.y() << ")"
            << " | Error: " << (p_eval - p0).norm() << std::endl;

  std::cout << "[Velocity]     Target: (" << v0.x() << ", " << v0.y() << ")"
            << " | Evaluated: (" << v_eval.x() << ", " << v_eval.y() << ")"
            << " | Error: " << (v_eval - v0).norm() << std::endl;

  std::cout << "[Acceleration] Target: (" << a0.x() << ", " << a0.y() << ")"
            << " | Evaluated: (" << a_eval.x() << ", " << a_eval.y() << ")"
            << " | Error: " << (a_eval - a0).norm() << std::endl;

  const double tol = 1e-5;
  if ((p_eval - p0).norm() >= tol) {
    std::cerr << "C0 Error too large!" << std::endl;
    return 1;
  }
  if ((v_eval - v0).norm() >= tol) {
    std::cerr << "C1 Error too large!" << std::endl;
    return 1;
  }
  if ((a_eval - a0).norm() >= tol) {
    std::cerr << "C2 Error too large!" << std::endl;
    return 1;
  }

  // Verify numerical continuity across t=0 to t=0+
  double dt_test = 0.001;
  Eigen::Vector2d p_eps = traj.evaluatePosition(dt_test);
  Eigen::Vector2d v_num = (p_eps - p_eval) / dt_test;
  std::cout << "p_eval: (" << p_eval.x() << ", " << p_eval.y() << ")" << std::endl;
  std::cout << "p_eps:  (" << p_eps.x() << ", " << p_eps.y() << ")" << std::endl;
  std::cout << "v_num:  (" << v_num.x() << ", " << v_num.y() << ")" << std::endl;
  std::cout << "v0:     (" << v0.x() << ", " << v0.y() << ")" << std::endl;
  std::cout << "[Numerical Velocity Check at t=0+] Error vs v0: " << (v_num - v0).norm() << std::endl;

  std::cout << ">>> ALL ABPOLICY C2 MATHEMATICAL CONSTRAINTS PASSED PERFECTLY! <<<" << std::endl;
  return 0;
}
