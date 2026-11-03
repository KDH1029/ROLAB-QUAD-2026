// Cone candidates from one LiDAR scan (no ROS). Used by cone_tracker_node and test_cone_lidar.
//
// A 23 cm cone is a small cluster of points between just above the floor and the cone top, with
// nothing right above it, standing clear of the floor around it. Walls and furniture fail on width
// or continue above the cone top (legs, people, boxes); bits of floor that rise into the band
// (thresholds, ramps, body pitch) are flat with the floor next to them. On the 10-30 bag (no cones)
// the floor test took out most of the 1-5 false clusters per scan.
#pragma once

#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace cone_lidar
{
struct Config
{
  double z_min{-0.27};          // base_link z band of a cone: 3 cm above the floor ...
  double z_max{-0.05};          // ... to the cone top plus margin
  double clear_gap{0.08};       // m above z_max where points start to count as "something above"
  double clear_height{0.60};    // m above z_max that must be empty over the cluster
  double cluster_tol{0.10};     // m, points closer than this join one cluster
  double max_width{0.30};       // m, wider clusters are not cones (cone base 14 cm + LiDAR noise)
  int min_points{3};
  double max_range{6.0};        // m from base_link
  double floor_gap{0.03};       // m, cluster mean must be this far above the lowest point around it
  double floor_radius{0.40};    // m, "around" for floor_gap (no floor point there: not judged)
  double surface_to_center{0.05};  // m, the LiDAR sees the near face; shift to the cone axis
};

struct Candidate
{
  Eigen::Vector2d center;  // base_link, on the cone axis
  int points{0};
  double width{0.0};
};

namespace detail
{
inline long long key(int ix, int iy) {return (static_cast<long long>(ix) << 32) ^ (static_cast<unsigned int>(iy));}
}

/// pts: LiDAR points in base_link. lidar_xy: LiDAR position in base_link (for the axis shift).
inline std::vector<Candidate> findCones(const std::vector<Eigen::Vector3d> & pts, const Eigen::Vector2d & lidar_xy,
                                        const Config & c)
{
  const double cell = c.cluster_tol;
  std::vector<Eigen::Vector2d> band;
  std::vector<double> band_z;
  std::unordered_map<long long, int> above;  // cells (cluster_tol) with points above the cone top
  std::unordered_map<long long, double> low;  // lowest z per cell, band and below (floor level)
  for (const auto & p : pts) {
    if (p.head<2>().norm() > c.max_range) continue;
    if (p.z() <= c.z_max && p.z() > c.z_min - 0.30) {
      const long long k = detail::key(static_cast<int>(std::floor(p.x() / cell)), static_cast<int>(std::floor(p.y() / cell)));
      auto it = low.find(k);
      if (it == low.end() || p.z() < it->second) low[k] = p.z();
    }
    if (p.z() >= c.z_min && p.z() <= c.z_max) {
      band.push_back(p.head<2>());
      band_z.push_back(p.z());
    } else if (p.z() > c.z_max + c.clear_gap && p.z() <= c.z_max + c.clear_height) {
      above[detail::key(static_cast<int>(std::floor(p.x() / cell)), static_cast<int>(std::floor(p.y() / cell)))]++;
    }
  }
  // grid-hash clustering: points of neighbouring cells within cluster_tol are connected
  std::unordered_map<long long, std::vector<int>> cells;
  for (int i = 0; i < static_cast<int>(band.size()); ++i) {
    cells[detail::key(static_cast<int>(std::floor(band[i].x() / cell)), static_cast<int>(std::floor(band[i].y() / cell)))].push_back(i);
  }
  std::vector<int> label(band.size(), -1);
  std::vector<Candidate> out;
  int next = 0;
  for (int s = 0; s < static_cast<int>(band.size()); ++s) {
    if (label[s] >= 0) continue;
    std::vector<int> stack{s}, members;
    label[s] = next;
    Eigen::Vector2d lo = band[s], hi = band[s];
    while (!stack.empty()) {
      const int i = stack.back();
      stack.pop_back();
      members.push_back(i);
      lo = lo.cwiseMin(band[i]);
      hi = hi.cwiseMax(band[i]);
      const int cx = static_cast<int>(std::floor(band[i].x() / cell)), cy = static_cast<int>(std::floor(band[i].y() / cell));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          auto it = cells.find(detail::key(cx + dx, cy + dy));
          if (it == cells.end()) continue;
          for (int j : it->second) {
            if (label[j] < 0 && (band[j] - band[i]).norm() <= c.cluster_tol) {
              label[j] = next;
              stack.push_back(j);
            }
          }
        }
      }
    }
    ++next;
    const double width = (hi - lo).norm();
    if (static_cast<int>(members.size()) < c.min_points || width > c.max_width) continue;
    // anything right above the cluster footprint (one cell margin) -> not a cone
    bool covered = false;
    const int x0 = static_cast<int>(std::floor(lo.x() / cell)) - 1, x1 = static_cast<int>(std::floor(hi.x() / cell)) + 1;
    const int y0 = static_cast<int>(std::floor(lo.y() / cell)) - 1, y1 = static_cast<int>(std::floor(hi.y() / cell)) + 1;
    for (int ix = x0; ix <= x1 && !covered; ++ix) {
      for (int iy = y0; iy <= y1 && !covered; ++iy) {
        covered = above.count(detail::key(ix, iy)) > 0;
      }
    }
    if (covered) continue;
    Eigen::Vector2d m = Eigen::Vector2d::Zero();
    double mz = 0.0;
    for (int i : members) {m += band[i]; mz += band_z[i];}
    m /= static_cast<double>(members.size());
    mz /= static_cast<double>(members.size());
    // flat with the floor next to it -> a piece of floor, not a cone
    const int rr = static_cast<int>(std::ceil(c.floor_radius / cell));
    const int mx = static_cast<int>(std::floor(m.x() / cell)), my = static_cast<int>(std::floor(m.y() / cell));
    double floor_z = 1e9;
    for (int dx = -rr; dx <= rr; ++dx) {
      for (int dy = -rr; dy <= rr; ++dy) {
        auto it = low.find(detail::key(mx + dx, my + dy));
        if (it != low.end() && (dx * dx + dy * dy) * cell * cell <= c.floor_radius * c.floor_radius) {
          floor_z = std::min(floor_z, it->second);
        }
      }
    }
    if (floor_z < 1e8 && mz - floor_z < c.floor_gap) continue;
    const Eigen::Vector2d ray = m - lidar_xy;
    if (ray.norm() > 1e-3) m += c.surface_to_center * ray.normalized();
    out.push_back({m, static_cast<int>(members.size()), width});
  }
  return out;
}
}  // namespace cone_lidar
