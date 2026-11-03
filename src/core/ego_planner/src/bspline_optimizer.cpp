#include "ego_planner/bspline_optimizer.hpp"
#include <algorithm>
#include <cmath>

namespace ego_planner
{

void BsplineTrajectory::setControlPoints(const std::vector<Eigen::Vector2d> & ctrl_pts, double dt)
{
  ctrl_pts_ = ctrl_pts;
  dt_ = dt;
  n_span_ = static_cast<int>(ctrl_pts_.size()) - 3;
  if (n_span_ < 1) {
    duration_ = 0.0;
  } else {
    duration_ = n_span_ * dt_;
  }
}

Eigen::Vector2d BsplineTrajectory::evaluatePosition(double t) const
{
  if (ctrl_pts_.empty()) return Eigen::Vector2d::Zero();
  if (n_span_ < 1) return ctrl_pts_.front();
  if (t <= 0.0) {
    return (ctrl_pts_[0] + 4.0 * ctrl_pts_[1] + ctrl_pts_[2]) / 6.0;
  }
  if (t >= duration_) {
    const size_t N = ctrl_pts_.size();
    return (ctrl_pts_[N - 3] + 4.0 * ctrl_pts_[N - 2] + ctrl_pts_[N - 1]) / 6.0;
  }

  int idx = static_cast<int>(std::floor(t / dt_));
  if (idx >= n_span_) idx = n_span_ - 1;

  const double u = (t - idx * dt_) / dt_;
  const double u2 = u * u;
  const double u3 = u2 * u;

  const double b0 = (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
  const double b1 = (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
  const double b2 = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
  const double b3 = u3 / 6.0;

  return b0 * ctrl_pts_[idx] + b1 * ctrl_pts_[idx + 1] +
         b2 * ctrl_pts_[idx + 2] + b3 * ctrl_pts_[idx + 3];
}

Eigen::Vector2d BsplineTrajectory::evaluateVelocity(double t) const
{
  if (ctrl_pts_.size() < 4 || n_span_ < 1) return Eigen::Vector2d::Zero();
  if (t <= 0.0) {
    return (ctrl_pts_[2] - ctrl_pts_[0]) / (2.0 * dt_);
  }
  if (t >= duration_) {
    const size_t N = ctrl_pts_.size();
    return (ctrl_pts_[N - 1] - ctrl_pts_[N - 3]) / (2.0 * dt_);
  }

  int idx = static_cast<int>(std::floor(t / dt_));
  if (idx >= n_span_) idx = n_span_ - 1;

  const double u = (t - idx * dt_) / dt_;
  const double u2 = u * u;

  const double db0 = (-1.0 + 2.0 * u - u2) * 0.5;
  const double db1 = (-2.0 * u + 1.5 * u2);
  const double db2 = (0.5 + u - 1.5 * u2);
  const double db3 = 0.5 * u2;

  return (db0 * ctrl_pts_[idx] + db1 * ctrl_pts_[idx + 1] +
          db2 * ctrl_pts_[idx + 2] + db3 * ctrl_pts_[idx + 3]) / dt_;
}

Eigen::Vector2d BsplineTrajectory::evaluateAcceleration(double t) const
{
  if (ctrl_pts_.size() < 4 || n_span_ < 1) return Eigen::Vector2d::Zero();
  if (t <= 0.0) {
    return (ctrl_pts_[0] - 2.0 * ctrl_pts_[1] + ctrl_pts_[2]) / (dt_ * dt_);
  }
  if (t >= duration_) {
    const size_t N = ctrl_pts_.size();
    return (ctrl_pts_[N - 3] - 2.0 * ctrl_pts_[N - 2] + ctrl_pts_[N - 1]) / (dt_ * dt_);
  }

  int idx = static_cast<int>(std::floor(t / dt_));
  if (idx >= n_span_) idx = n_span_ - 1;

  const double u = (t - idx * dt_) / dt_;

  const double ddb0 = 1.0 - u;
  const double ddb1 = -2.0 + 3.0 * u;
  const double ddb2 = 1.0 - 3.0 * u;
  const double ddb3 = u;

  return (ddb0 * ctrl_pts_[idx] + ddb1 * ctrl_pts_[idx + 1] +
          ddb2 * ctrl_pts_[idx + 2] + ddb3 * ctrl_pts_[idx + 3]) / (dt_ * dt_);
}

void BsplineOptimizer::init(const std::shared_ptr<GridMap> & grid_map,
                            double lambda_smooth, double lambda_obs, double lambda_feas,
                            double max_vel, double max_acc, double safe_dist)
{
  grid_map_ = grid_map;
  lambda_smooth_ = lambda_smooth;
  lambda_obs_ = lambda_obs;
  lambda_feas_ = lambda_feas;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  safe_dist_ = safe_dist;
}

bool BsplineOptimizer::isCollisionFree(const BsplineTrajectory & traj, double sample_dt, double t0) const
{
  if (!grid_map_) return true;
  const double T = traj.getTotalDuration();
  bool escaping = true;  // still inside the inflation we started in
  for (double t = std::max(0.0, t0); t <= T + 1e-9; t += sample_dt) {
    const bool hit = grid_map_->isInflated(traj.evaluatePosition(std::min(t, T)));
    if (!hit) {
      escaping = false;
    } else if (!escaping) {
      return false;
    }
  }
  return true;
}

void BsplineOptimizer::computeCostAndGrad(const std::vector<Eigen::Vector2d> & q,
                                         std::vector<Eigen::Vector2d> & grad,
                                         double & cost)
{
  const size_t N = q.size();
  grad.assign(N, Eigen::Vector2d::Zero());
  cost = 0.0;

  // 1. Smoothness Cost (acceleration squared)
  // J_smooth = sum_{i=0}^{N-3} ||q_{i+2} - 2q_{i+1} + q_i||^2
  for (size_t i = 0; i + 2 < N; ++i) {
    Eigen::Vector2d diff = q[i + 2] - 2.0 * q[i + 1] + q[i];
    cost += lambda_smooth_ * diff.squaredNorm();

    grad[i]     += lambda_smooth_ * 2.0 * diff;
    grad[i + 1] -= lambda_smooth_ * 4.0 * diff;
    grad[i + 2] += lambda_smooth_ * 2.0 * diff;
  }

  // 2. Obstacle Cost (ESDF-free direct gradient)
  if (grid_map_) {
    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector2d obs_grad;
      double c_obs = grid_map_->evaluateCostAndGrad(q[i], obs_grad, safe_dist_);
      cost += lambda_obs_ * c_obs;
      grad[i] += lambda_obs_ * obs_grad;
      // cones: own range and gain (cones.safe_dist / cones.weight), not lambda_obs
      cost += grid_map_->evaluateConeCostAndGrad(q[i], obs_grad);
      grad[i] += obs_grad;
    }
  }

  // 3. Feasibility Cost (Velocity & Acceleration limits)
  const double dt_inv = 1.0 / dt_;
  const double dt2_inv = dt_inv * dt_inv;

  for (size_t i = 0; i + 1 < N; ++i) {
    Eigen::Vector2d v = (q[i + 1] - q[i]) * dt_inv;
    double v_norm = v.norm();
    if (v_norm > max_vel_) {
      double diff_v = v_norm - max_vel_;
      cost += lambda_feas_ * diff_v * diff_v;
      Eigen::Vector2d dv_grad = 2.0 * diff_v * (v / v_norm) * dt_inv;
      grad[i + 1] += lambda_feas_ * dv_grad;
      grad[i]     -= lambda_feas_ * dv_grad;
    }
  }

  for (size_t i = 0; i + 2 < N; ++i) {
    Eigen::Vector2d a = (q[i + 2] - 2.0 * q[i + 1] + q[i]) * dt2_inv;
    double a_norm = a.norm();
    if (a_norm > max_acc_) {
      double diff_a = a_norm - max_acc_;
      cost += lambda_feas_ * diff_a * diff_a;
      Eigen::Vector2d da_grad = 2.0 * diff_a * (a / a_norm) * dt2_inv;
      grad[i + 2] += lambda_feas_ * da_grad;
      grad[i + 1] -= lambda_feas_ * 2.0 * da_grad;
      grad[i]     += lambda_feas_ * da_grad;
    }
  }
}

bool BsplineOptimizer::optimize(const std::vector<Eigen::Vector2d> & guide_path,
                               const Eigen::Vector2d & start_vel,
                               BsplineTrajectory & traj)
{
  if (guide_path.empty()) return false;
  return optimizeWithContinuity(guide_path, guide_path.front(), start_vel, Eigen::Vector2d::Zero(), traj);
}

bool BsplineOptimizer::optimizeWithContinuity(const std::vector<Eigen::Vector2d> & guide_path,
                                             const Eigen::Vector2d & p0,
                                             const Eigen::Vector2d & v0,
                                             const Eigen::Vector2d & a0,
                                             BsplineTrajectory & traj)
{
  if (guide_path.size() < 2) return false;

  // Resample the guide path into uniformly spaced control points along its length.
  // The original only kept guide-path vertices; after A* line-of-sight simplification those are
  // just the corners, so long segments had no control points and the spline cut the corners
  // into the inflation (13 of 41 recorded-path plans on the 누리관 map).
  std::vector<Eigen::Vector2d> q;
  const double step_dist = std::clamp(max_vel_ * dt_ * ctrl_spacing_, 0.15, 0.35);
  q.push_back(p0);
  double carry = 0.0;  // distance already covered since the last control point
  for (size_t i = 1; i < guide_path.size(); ++i) {
    const Eigen::Vector2d a = guide_path[i - 1];
    const Eigen::Vector2d b = guide_path[i];
    const double seg = (b - a).norm();
    if (seg < 1e-9) continue;
    double s = step_dist - carry;
    while (s <= seg) {
      q.push_back(a + (b - a) * (s / seg));
      s += step_dist;
    }
    carry = seg - (s - step_dist);
  }
  if ((q.back() - guide_path.back()).norm() > 1e-6) {
    q.push_back(guide_path.back());
  }

  // Ensure enough points for C2 boundary conditions at start and zero vel/acc at goal (at least 7 points)
  while (q.size() < 7) {
    size_t best_idx = 0;
    double max_d = 0.0;
    for (size_t i = 0; i + 1 < q.size(); ++i) {
      double d = (q[i + 1] - q[i]).norm();
      if (d > max_d) {
        max_d = d;
        best_idx = i;
      }
    }
    Eigen::Vector2d mid = 0.5 * (q[best_idx] + q[best_idx + 1]);
    q.insert(q.begin() + best_idx + 1, mid);
  }

  const size_t N = q.size();
  const double dt2 = dt_ * dt_;

  // ABPolicy C2 Continuity-Constrained Boundary Formulation:
  // p(0)   = (q0 + 4*q1 + q2) / 6 = p0
  // p'(0)  = (q2 - q0) / (2*dt)   = v0
  // p''(0) = (q0 - 2*q1 + q2)/dt2 = a0
  // Closed-form solution for initial 3 control points:
  q[0] = p0 - dt_ * v0 + (1.0 / 3.0) * dt2 * a0;
  q[1] = p0 - (1.0 / 6.0) * dt2 * a0;
  q[2] = p0 + dt_ * v0 + (1.0 / 3.0) * dt2 * a0;

  // Terminal boundary condition: zero velocity & zero acceleration at goal
  const Eigen::Vector2d goal = guide_path.back();
  q[N - 1] = goal;
  q[N - 2] = goal;
  q[N - 3] = goal;

  // Optimize interior control points q[3] .. q[N - 4] using Gradient Descent
  const int max_iters = max_iters_;
  const double alpha = step_size_;

  std::vector<Eigen::Vector2d> grad;
  double prev_cost = 0.0;

  for (int iter = 0; iter < max_iters; ++iter) {
    computeCostAndGrad(q, grad, prev_cost);

    // Hard boundary constraints: zero gradients for boundary control points
    grad[0].setZero();
    grad[1].setZero();
    grad[2].setZero();
    grad[N - 3].setZero();
    grad[N - 2].setZero();
    grad[N - 1].setZero();

    // Check convergence
    double max_grad = 0.0;
    for (size_t i = 3; i < N - 3; ++i) {
      max_grad = std::max(max_grad, grad[i].norm());
    }
    if (max_grad < 1e-3) break;

    // Gradient step with trust-region clamping (max 0.04m per step to prevent gradient explosion)
    const double max_step = max_step_;
    for (size_t i = 3; i < N - 3; ++i) {
      Eigen::Vector2d step = alpha * grad[i];
      const double step_norm = step.norm();
      if (step_norm > max_step) {
        step = (step / step_norm) * max_step;
      }
      q[i] -= step;
    }
  }

  traj.setControlPoints(q, dt_);
  return true;
}

}  // namespace ego_planner

