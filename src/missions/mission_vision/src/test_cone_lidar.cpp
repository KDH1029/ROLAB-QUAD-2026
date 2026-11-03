// Synthetic check of cone_lidar::findCones: XT16 rays (Go2 mount) against cones, a thin tall pole,
// a wall and the floor, with range noise. Prints what was found; exit code 1 on a miss / false hit.
//   ros2 run mission_vision test_cone_lidar            (synthetic)
//   ros2 run mission_vision test_cone_lidar pts.xyz    (file: "x y z" in base_link -> candidates)
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include "mission_vision/cone_lidar.hpp"

namespace
{
const double kFloor = -0.30;                    // base_link z of the floor (standing Go2)
struct Cone {double x, y;};

// Distance along the ray to the first surface, or -1.
double cast(const Eigen::Vector3d & o, const Eigen::Vector3d & d, const std::vector<Cone> & cones,
            const Eigen::Vector2d & pole, double wall_x)
{
  for (double s = 0.3; s < 7.0; s += 0.004) {
    const Eigen::Vector3d p = o + s * d;
    if (p.z() <= kFloor) return s;
    if (p.x() >= wall_x && p.z() < kFloor + 1.0) return s;
    const double h = p.z() - kFloor;
    for (const auto & c : cones) {
      if (h <= 0.23 && std::hypot(p.x() - c.x, p.y() - c.y) <= 0.07 * (1.0 - h / 0.23) + 0.012) return s;
    }
    if (h <= 0.80 && (p.head<2>() - pole).norm() <= 0.02) return s;
  }
  return -1.0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc > 1) {  // file mode: "x y z" per line in base_link -> candidates
    std::vector<Eigen::Vector3d> pts;
    FILE * f = std::fopen(argv[1], "r");
    double x, y, z;
    while (f && std::fscanf(f, "%lf %lf %lf", &x, &y, &z) == 3) pts.emplace_back(x, y, z);
    if (f) std::fclose(f);
    for (const auto & c : cone_lidar::findCones(pts, Eigen::Vector2d(0.25, 0.0), cone_lidar::Config{})) {
      std::printf("%.3f %.3f %d %.3f\n", c.center.x(), c.center.y(), c.points, c.width);
    }
    return 0;
  }
  // XT16 in base_link (bridge_params / camera_extrinsics): 0.25 m forward, 0.1588 m up, yawed 90 deg
  Eigen::Isometry3d base_T_lidar = Eigen::Isometry3d::Identity();
  base_T_lidar.translation() = Eigen::Vector3d(0.25, 0.0, 0.1588);
  base_T_lidar.linear() = Eigen::Quaterniond(0.7070, 0.0024, 0.0063, 0.7071).normalized().toRotationMatrix();
  const std::vector<Cone> cones = {{1.5, 0.20}, {1.5, -0.35}, {3.0, 0.40}, {2.2, 1.2}};
  const Eigen::Vector2d pole(2.0, -1.0);
  const double wall_x = 4.5;
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.015);

  int fails = 0;
  for (int trial = 0; trial < 5; ++trial) {
    std::vector<Eigen::Vector3d> pts;
    const Eigen::Vector3d o = base_T_lidar.translation();
    for (int ring = 0; ring < 16; ++ring) {
      const double el = (-15.0 + 2.0 * ring) * M_PI / 180.0;
      for (double az = -80.0; az <= 80.0; az += 0.36) {
        const double a = az * M_PI / 180.0;
        const Eigen::Vector3d d(std::cos(el) * std::cos(a), std::cos(el) * std::sin(a), std::sin(el));
        const double s = cast(o, d, cones, pole, wall_x);
        if (s > 0) pts.push_back(o + (s + noise(rng)) * d);
      }
    }
    cone_lidar::Config cfg;
    const auto found = cone_lidar::findCones(pts, o.head<2>(), cfg);
    std::printf("trial %d: %zu points, %zu candidates\n", trial, pts.size(), found.size());
    std::vector<bool> hit(cones.size(), false);
    for (const auto & f : found) {
      int best = -1;
      double bd = 0.15;
      for (size_t i = 0; i < cones.size(); ++i) {
        const double dd = std::hypot(f.center.x() - cones[i].x, f.center.y() - cones[i].y);
        if (dd < bd) {bd = dd; best = static_cast<int>(i);}
      }
      std::printf("  (%.3f, %.3f) %2d pts width %.2f -> %s", f.center.x(), f.center.y(), f.points, f.width,
                  best < 0 ? "FALSE CANDIDATE\n" : "");
      if (best < 0) {++fails; continue;}
      hit[best] = true;
      std::printf("cone %d, error %.1f cm\n", best, 100.0 * std::hypot(f.center.x() - cones[best].x, f.center.y() - cones[best].y));
    }
    for (size_t i = 0; i < cones.size(); ++i) {
      if (!hit[i]) {std::printf("  MISSED cone %zu at (%.2f, %.2f)\n", i, cones[i].x, cones[i].y); ++fails;}
    }
  }
  std::printf(fails ? "FAIL (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
