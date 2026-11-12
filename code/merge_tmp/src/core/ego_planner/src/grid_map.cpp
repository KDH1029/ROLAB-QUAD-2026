#include "ego_planner/grid_map.hpp"
#include <cstring>
#include <queue>

namespace ego_planner
{

GridMap::GridMap()
{
}

void GridMap::init(double resolution, double map_size_x, double map_size_y, double inflate_radius)
{
  resolution_ = resolution;
  map_size_x_ = map_size_x;
  map_size_y_ = map_size_y;
  inflate_radius_ = inflate_radius;
  inflate_cells_ = static_cast<int>(std::ceil(inflate_radius_ / resolution_));

  size_x_ = static_cast<int>(std::ceil(map_size_x_ / resolution_));
  size_y_ = static_cast<int>(std::ceil(map_size_y_ / resolution_));
  origin_ = Eigen::Vector2d(-map_size_x_ * 0.5, -map_size_y_ * 0.5);

  occupancy_.assign(size_x_ * size_y_, 0);
  inflated_.assign(size_x_ * size_y_, 0);
}

void GridMap::resetLocal(const Eigen::Vector2d & center)
{
  origin_ = center - Eigen::Vector2d(map_size_x_ * 0.5, map_size_y_ * 0.5);
  std::fill(occupancy_.begin(), occupancy_.end(), 0);
  std::fill(inflated_.begin(), inflated_.end(), 0);
}

void GridMap::posToIndex(const Eigen::Vector2d & pos, int & ix, int & iy) const
{
  ix = static_cast<int>(std::floor((pos.x() - origin_.x()) / resolution_));
  iy = static_cast<int>(std::floor((pos.y() - origin_.y()) / resolution_));
}

void GridMap::indexToPos(int ix, int iy, Eigen::Vector2d & pos) const
{
  pos.x() = origin_.x() + (static_cast<double>(ix) + 0.5) * resolution_;
  pos.y() = origin_.y() + (static_cast<double>(iy) + 0.5) * resolution_;
}

bool GridMap::isInMap(const Eigen::Vector2d & pos) const
{
  int ix, iy;
  posToIndex(pos, ix, iy);
  return isInMap(ix, iy);
}

bool GridMap::isInMap(int ix, int iy) const
{
  return ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_;
}

bool GridMap::isOccupied(const Eigen::Vector2d & pos) const
{
  int ix, iy;
  posToIndex(pos, ix, iy);
  return isOccupied(ix, iy);
}

bool GridMap::isOccupied(int ix, int iy) const
{
  if (!isInMap(ix, iy)) return true;
  return occupancy_[toIndex(ix, iy)] > 0;
}

bool GridMap::isInflated(const Eigen::Vector2d & pos) const
{
  int ix, iy;
  posToIndex(pos, ix, iy);
  return isInflated(ix, iy);
}

bool GridMap::isInflated(int ix, int iy) const
{
  if (!isInMap(ix, iy)) return true;
  return inflated_[toIndex(ix, iy)] > 0;
}

bool GridMap::loadPriorMap(const std::string & bin_path)
{
  FILE * f = fopen(bin_path.c_str(), "rb");
  if (!f) return false;
  uint32_t n = 0;
  if (fread(&n, sizeof(uint32_t), 1, f) == 1 && n > 0) {
    std::vector<float> buf(2 * n);
    if (fread(buf.data(), sizeof(float), 2 * n, f) == 2 * n) {
      static_obstacles_.clear();
      static_obstacles_.reserve(n);
      for (size_t i = 0; i < n; ++i) {
        static_obstacles_.emplace_back(buf[2 * i], buf[2 * i + 1]);
      }
    }
  }
  fclose(f);
  return !static_obstacles_.empty();
}

void GridMap::markOccupied(const Eigen::Vector2d & pt, std::vector<std::pair<int, int>> & occupied_indices)
{
  int ix, iy;
  posToIndex(pt, ix, iy);
  if (isInMap(ix, iy)) {
    const int idx = toIndex(ix, iy);
    if (occupancy_[idx] == 0) {
      occupancy_[idx] = 100;
      occupied_indices.emplace_back(ix, iy);
    }
  }
}

void GridMap::insertStatic(std::vector<std::pair<int, int>> & occupied_indices)
{
  for (const auto & pt : static_obstacles_) {
    markOccupied(pt, occupied_indices);
  }
  for (const auto & pt : extra_obstacles_) {
    markOccupied(pt, occupied_indices);
  }
}

void GridMap::inflate(const std::vector<std::pair<int, int>> & occupied_indices)
{
  const int r = inflate_cells_;
  const int r2 = r * r;
  for (const auto & pt : occupied_indices) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (dx * dx + dy * dy <= r2) {
          const int nx = pt.first + dx;
          const int ny = pt.second + dy;
          if (isInMap(nx, ny)) {
            inflated_[toIndex(nx, ny)] = 100;
          }
        }
      }
    }
  }
}

void GridMap::inflateCones()
{
  const double r = cones_.inflate_radius;
  const int rc = static_cast<int>(std::ceil(r / resolution_));
  for (const auto & c : cones_.centers) {
    int cx, cy;
    posToIndex(c, cx, cy);
    for (int dx = -rc; dx <= rc; ++dx) {
      for (int dy = -rc; dy <= rc; ++dy) {
        const int nx = cx + dx, ny = cy + dy;
        if (!isInMap(nx, ny)) continue;
        Eigen::Vector2d p;
        indexToPos(nx, ny, p);
        if ((p - c).norm() <= r) inflated_[toIndex(nx, ny)] = 100;
      }
    }
  }
}

double GridMap::evaluateConeCostAndGrad(const Eigen::Vector2d & pos, Eigen::Vector2d & grad) const
{
  grad.setZero();
  double cost = 0.0;
  const double safe = cones_.safe_dist;
  for (const auto & c : cones_.centers) {
    const Eigen::Vector2d d = pos - c;
    const double dist = d.norm();
    if (dist >= safe) continue;
    // same barrier as the grid term: U = (safe - d)^2 + 0.5 (safe - d), summed over cones
    const double diff = safe - dist;
    cost += cones_.weight * (diff * diff + 0.5 * diff);
    grad += (dist > 1e-4) ? Eigen::Vector2d(-cones_.weight * (0.5 + 2.0 * diff) * d / dist)
                          : Eigen::Vector2d(cones_.weight, 0.0);
  }
  return cost;
}

void GridMap::updateStaticOnly(const Eigen::Vector2d & center)
{
  resetLocal(center);
  std::vector<std::pair<int, int>> occupied_indices;
  occupied_indices.reserve(static_obstacles_.size());
  insertStatic(occupied_indices);
  inflate(occupied_indices);
  inflateCones();
}

void GridMap::updateCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
                          const Eigen::Vector3d & robot_pos,
                          double robot_yaw,
                          double min_h, double max_h)
{
  // Center local map near robot
  resetLocal(Eigen::Vector2d(robot_pos.x(), robot_pos.y()));

  std::vector<std::pair<int, int>> occupied_indices;
  occupied_indices.reserve(static_obstacles_.size() + 2000);

  // 1. Insert static obstacles from prior map (walls, permanent furniture)
  insertStatic(occupied_indices);

  // 2. Insert real-time live LiDAR points
  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : cloud_msg->fields) {
    if (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) return;

  const size_t n_pts = cloud_msg->width * cloud_msg->height;
  const uint8_t * data = cloud_msg->data.data();
  const size_t step = cloud_msg->point_step;
  const double c_yaw = std::cos(robot_yaw);
  const double s_yaw = std::sin(robot_yaw);

  for (size_t i = 0; i < n_pts; ++i) {
    const uint8_t * p = data + i * step;
    float x, y, z;
    std::memcpy(&x, p + off_x, sizeof(float));
    std::memcpy(&y, p + off_y, sizeof(float));
    std::memcpy(&z, p + off_z, sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    // Relative to robot position in world frame
    const double dx = x - robot_pos.x();
    const double dy = y - robot_pos.y();
    const double d_xy = std::hypot(dx, dy);
    if (d_xy < filter_.min_range || d_xy > filter_.max_range) continue;

    // Relative height and body frame coordinates
    const double rel_z = z - robot_pos.z();
    const double bx =  c_yaw * dx + s_yaw * dy;
    const double by = -s_yaw * dx + c_yaw * dy;

    // Geometric body self-filter (Go2 body + leg swing envelope)
    if (bx >= filter_.body_x_min && bx <= filter_.body_x_max &&
      std::abs(by) <= filter_.body_y_abs && rel_z < filter_.body_z_max)
    {
      continue;
    }

    // Distance-adaptive ground clearance filter (walking pitch/roll margin).
    // Note: obstacles are points ABOVE min_h, so lowering min_h marks more ground, not less.
    const double adaptive_min_h = min_h + d_xy * filter_.adaptive_slope;
    if (rel_z < adaptive_min_h || rel_z > max_h) continue;

    markOccupied(Eigen::Vector2d(x, y), occupied_indices);
  }

  inflate(occupied_indices);
  inflateCones();
}

void GridMap::updateScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg,
                         const Eigen::Vector3d & robot_pos,
                         double robot_yaw)
{
  resetLocal(Eigen::Vector2d(robot_pos.x(), robot_pos.y()));
  std::vector<std::pair<int, int>> occupied_indices;
  occupied_indices.reserve(static_obstacles_.size() + scan_msg->ranges.size());
  insertStatic(occupied_indices);

  // The scan frame (base_stab) is levelled at the robot base, so only the yaw is needed.
  const double c_yaw = std::cos(robot_yaw);
  const double s_yaw = std::sin(robot_yaw);
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    const double r = scan_msg->ranges[i];
    if (!std::isfinite(r) || r < filter_.min_range || r > filter_.max_range) continue;
    const double a = scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment;
    const double bx = r * std::cos(a);
    const double by = r * std::sin(a);
    markOccupied(
      Eigen::Vector2d(robot_pos.x() + c_yaw * bx - s_yaw * by, robot_pos.y() + s_yaw * bx + c_yaw * by),
      occupied_indices);
  }
  inflate(occupied_indices);
  inflateCones();
}

double GridMap::evaluateCostAndGrad(const Eigen::Vector2d & pos, Eigen::Vector2d & grad, double safe_dist) const
{
  grad.setZero();
  int ix, iy;
  posToIndex(pos, ix, iy);

  if (!isInMap(ix, iy)) {
    // Outside map boundary: push towards map center
    Eigen::Vector2d center = origin_ + Eigen::Vector2d(map_size_x_ * 0.5, map_size_y_ * 0.5);
    Eigen::Vector2d diff = pos - center;
    grad = diff.normalized();
    return 10.0;
  }

  // Search local neighborhood for closest occupied point
  const int search_r = static_cast<int>(std::ceil(safe_dist / resolution_)) + 2;
  double min_dist_sq = std::numeric_limits<double>::infinity();
  Eigen::Vector2d nearest_obs_pt(0, 0);
  bool found_obs = false;

  for (int dx = -search_r; dx <= search_r; ++dx) {
    for (int dy = -search_r; dy <= search_r; ++dy) {
      int nx = ix + dx;
      int ny = iy + dy;
      if (isInMap(nx, ny) && occupancy_[toIndex(nx, ny)] > 0) {
        Eigen::Vector2d obs_pt;
        indexToPos(nx, ny, obs_pt);
        double dist_sq = (pos - obs_pt).squaredNorm();
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          nearest_obs_pt = obs_pt;
          found_obs = true;
        }
      }
    }
  }

  if (!found_obs) {
    return 0.0;
  }

  double dist = std::sqrt(min_dist_sq);
  if (dist >= safe_dist) {
    return 0.0;
  }

  // Repulsive barrier penalty: U(dist) = (safe_dist - dist)^2 + 0.5 * (safe_dist - dist)
  // Guarantees non-vanishing outward repulsive vector field everywhere within safe_dist
  double diff_dist = safe_dist - dist;
  double cost = diff_dist * diff_dist + 0.5 * diff_dist;

  // Gradient: d(cost)/d(pos) = -(0.5 + 2.0 * diff_dist) * (pos - nearest_obs_pt) / dist
  // Resulting in authoritative repulsive force -alpha * grad pointing outwards away from obstacle
  if (dist > 1e-4) {
    grad = -(0.5 + 2.0 * diff_dist) * (pos - nearest_obs_pt) / dist;
  } else {
    grad = Eigen::Vector2d(1.0, 0.0);
  }

  return cost;
}

}  // namespace ego_planner

