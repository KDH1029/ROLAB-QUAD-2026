// Synthetic check of cone_lidar::findCones: XT16 rays (Go2 mount) against cones, a thin tall pole,
// a wall and the floor, with range noise. Prints what was found; exit code 1 on a miss / false hit.
//   ros2 run mission_vision test_cone_lidar            (synthetic)
//   ros2 run mission_vision test_cone_lidar pts.xyz    (file: "x y z" in base_link -> candidates)
#include <cmath>
#include <cstdlib>
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
    cone_lidar::Config cfg;
    if (argc > 3) {cfg.z_min = std::atof(argv[2]); cfg.z_max = std::atof(argv[3]);}   // [z_min z_max]
    for (const auto & c : cone_lidar::findCones(pts, Eigen::Vector2d(0.25, 0.0), cfg)) {
      std::printf("%.3f %.3f %d %.3f\n", c.center.x(), c.center.y(), c.points, c.width);
    }
    return 0;
  }
  // XT16 in base_link (bridge_params / cone_tracker_params): 0.25 m forward, 0.1588 m up, yawed 90 deg
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
  // --- LiDAR-only persistence: true cones seen in 80 % of scans with 2 cm noise; random floor bits
  // (1.1 per scan, the 10-30 bag rate) and one fixed "threshold" bit flickering in 30 % of scans
  std::printf("persistence (LiDAR-only confirmation, 200 runs):\n");
  double worst_all = 0;
  for (int trial = 0; trial < 200; ++trial) {
    cone_lidar::PersistTracker pt;
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::normal_distribution<double> n2(0.0, 0.02);
    std::poisson_distribution<int> npois(1.1);
    const std::vector<Eigen::Vector2d> truth = {{2.0, 0.3}, {3.0, -0.3}, {4.0, 0.3}};
    const Eigen::Vector2d threshold_bit(2.5, 0.45);
    std::vector<double> confirm_t(truth.size(), -1);
    int false_confirmed = 0;
    for (int k = 0; k < 120; ++k) {               // 6 s at 20 Hz
      const double t = 0.05 * k;
      std::vector<Eigen::Vector2d> c;
      for (const auto & g : truth) if (U(rng) < 0.8) c.push_back(g + Eigen::Vector2d(n2(rng), n2(rng)));
      const int nf = npois(rng);
      for (int i = 0; i < nf; ++i) c.push_back(Eigen::Vector2d(1.0 + 4.0 * U(rng), -0.5 + 1.0 * U(rng)));
      if (U(rng) < 0.3) c.push_back(threshold_bit + Eigen::Vector2d(0.03 * n2(rng) / 0.02, 0.03 * n2(rng) / 0.02));
      for (int id : pt.update(t, c)) {
        for (const auto & tr : pt.tracks()) {
          if (tr.id != id) continue;
          int hit = -1;
          for (size_t g = 0; g < truth.size(); ++g) if ((tr.pos - truth[g]).norm() < 0.08) hit = static_cast<int>(g);
          if (hit < 0) {
            ++false_confirmed;
            std::printf("    false cone at (%.2f, %.2f) t %.2f s, %d hits\n", tr.pos.x(), tr.pos.y(), t, tr.hits);
          } else if (confirm_t[hit] < 0) {
            confirm_t[hit] = t;
          }
        }
      }
    }
    int missed = 0;
    double worst = 0;
    for (double ct : confirm_t) {if (ct < 0) ++missed; else worst = std::max(worst, ct);}
    for (size_t g = 0; g < truth.size(); ++g) {
      if (confirm_t[g] >= 0) continue;
      for (const auto & tr : pt.tracks()) {
        if ((tr.pos - truth[g]).norm() < 0.3) {
          std::printf("    missed cone %zu: track #%d at (%.3f, %.3f) %.3f m off, hits %d, hist %zu, confirmed %d\n", g, tr.id,
                      tr.pos.x(), tr.pos.y(), (tr.pos - truth[g]).norm(), tr.hits, tr.hist.size(), tr.confirmed);
        }
      }
    }
    worst_all = std::max(worst_all, worst);
    if (trial < 3 || missed || false_confirmed) {
      std::printf("  trial %2d: true cones confirmed %zu/3 (slowest after %.2f s), false confirmed %d\n",
                  trial, truth.size() - missed, worst, false_confirmed);
    }
    if (missed) ++fails;
    if (false_confirmed) ++fails;
  }
  std::printf("  slowest confirmation of a true cone over all runs: %.2f s\n", worst_all);
  std::printf(fails ? "FAIL (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
