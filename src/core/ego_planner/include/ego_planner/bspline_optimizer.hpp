#ifndef EGO_PLANNER__BSPLINE_OPTIMIZER_HPP_
#define EGO_PLANNER__BSPLINE_OPTIMIZER_HPP_

#include <vector>
#include <memory>
#include <Eigen/Core>
#include "ego_planner/grid_map.hpp"

namespace ego_planner
{

class BsplineTrajectory
{
public:
  BsplineTrajectory() = default;

  void setControlPoints(const std::vector<Eigen::Vector2d> & ctrl_pts, double dt);
  Eigen::Vector2d evaluatePosition(double t) const;
  Eigen::Vector2d evaluateVelocity(double t) const;
  Eigen::Vector2d evaluateAcceleration(double t) const;
  double getTotalDuration() const { return duration_; }
  double getDt() const { return dt_; }
  const std::vector<Eigen::Vector2d> & getControlPoints() const { return ctrl_pts_; }

private:
  std::vector<Eigen::Vector2d> ctrl_pts_;
  double dt_{0.2};
  double duration_{0.0};
  int n_span_{0};
};

class BsplineOptimizer
{
public:
  BsplineOptimizer() = default;
  ~BsplineOptimizer() = default;

  void init(const std::shared_ptr<GridMap> & grid_map,
            double lambda_smooth, double lambda_obs, double lambda_feas,
            double max_vel, double max_acc, double safe_dist);
  // Gradient-descent settings (ego_planner_params.yaml: optimizer.*)
  void setSolverParams(double dt, int max_iters, double step_size, double max_step, double ctrl_spacing)
  {
    dt_ = dt;
    max_iters_ = max_iters;
    step_size_ = step_size;
    max_step_ = max_step;
    ctrl_spacing_ = ctrl_spacing;
  }
  // True if traj (sampled every sample_dt s from t0) stays outside the inflated grid.
  // A leading stretch that starts inside the inflation is allowed: when the robot is already
  // closer to a wall than the inflation radius, the only valid plans lead out of it.
  bool isCollisionFree(const BsplineTrajectory & traj, double sample_dt, double t0 = 0.0) const;

  bool optimize(const std::vector<Eigen::Vector2d> & guide_path,
                const Eigen::Vector2d & start_vel,
                BsplineTrajectory & traj);

  bool optimizeWithContinuity(const std::vector<Eigen::Vector2d> & guide_path,
                              const Eigen::Vector2d & p0,
                              const Eigen::Vector2d & v0,
                              const Eigen::Vector2d & a0,
                              BsplineTrajectory & traj);

private:
  std::shared_ptr<GridMap> grid_map_;
  double lambda_smooth_{1.0};
  double lambda_obs_{10.0};
  double lambda_feas_{2.0};
  double max_vel_{1.0};
  double max_acc_{1.5};
  double safe_dist_{0.45};
  double dt_{0.2};
  int max_iters_{35};
  double step_size_{0.02};
  double max_step_{0.04};
  double ctrl_spacing_{0.8};

  void computeCostAndGrad(const std::vector<Eigen::Vector2d> & q,
                          std::vector<Eigen::Vector2d> & grad,
                          double & cost);
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__BSPLINE_OPTIMIZER_HPP_

