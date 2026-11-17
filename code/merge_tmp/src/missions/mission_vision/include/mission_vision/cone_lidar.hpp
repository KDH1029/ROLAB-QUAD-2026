// Cone candidates from one LiDAR scan (no ROS). Used by cone_tracker_node and test_cone_lidar.
//
// A 23 cm cone is a small cluster of points between just above the floor and the cone top, with
// nothing right above it, standing clear of the floor around it. Walls and furniture fail on width
// or continue above the cone top (legs, people, boxes); bits of floor that rise into the band
// (thresholds, ramps, body pitch) are flat with the floor next to them. On the 10-30 bag (no cones)
// the floor test took out most of the 1-5 false clusters per scan.
//
// Tall columns go first (tall_cell): a small vertical column holding points both in the cone band and
// above the cone top is something taller than a cone - a wall, a leg, a box side. Those points are
// dropped before clustering, so a cone standing against a wall becomes its own small cluster instead of
// one long cluster with the wall (11-05, 145915 replay: cone 10, 2 cm from the inner wall, was joined to
// the wall in 338 scans and clean in 141). This uses only the scan itself - no map, so a localisation
// error cannot delete a cone or keep a wall.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
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
  double tall_cell{0.04};       // m, column size for the tall-column removal above; 0 = off
  int tall_dilate{1};           // a point is in a tall structure when band AND above points occur within this
                                // many tall_cell cells around it: a wall's foot and top often fall in neighbouring
                                // 4 cm columns (slight lean, ring azimuth offsets) - 0 left cone 10 joined (145915)
  // "something above" is looked for only right over the cluster's points (tall_cell cells, +-1), not
  // over its whole footprint + a cluster_tol cell: a cone 2 cm in front of a wall hides the wall's
  // foot, so the wall top 9-14 cm behind the cone's visible face is not a tall column and used to
  // count as "above" (145915: cone 10). Tables / boxes over a cone still cover its points. 0 = old test.
  bool fine_cover{false};       // off by default (no map / regions: the wide test also drops wall-foot bits)
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
  // columns (tall_cell) with points both in the band and above the cone top: taller than a cone
  std::unordered_map<long long, int> tall;   // bit 1 = band point, bit 2 = above point
  auto tkey = [&](const Eigen::Vector3d & p) {
    return detail::key(static_cast<int>(std::floor(p.x() / c.tall_cell)), static_cast<int>(std::floor(p.y() / c.tall_cell)));
  };
  if (c.tall_cell > 0.0) {
    for (const auto & p : pts) {
      if (p.head<2>().norm() > c.max_range) continue;
      if (p.z() >= c.z_min && p.z() <= c.z_max) tall[tkey(p)] |= 1;
      else if (p.z() > c.z_max + c.clear_gap && p.z() <= c.z_max + c.clear_height) tall[tkey(p)] |= 2;
    }
  }
  auto in_tall = [&](const Eigen::Vector3d & p) {
    if (c.tall_cell <= 0.0) return false;
    const int ix = static_cast<int>(std::floor(p.x() / c.tall_cell)), iy = static_cast<int>(std::floor(p.y() / c.tall_cell));
    const bool is_band = p.z() >= c.z_min && p.z() <= c.z_max;
    const int want = is_band ? 2 : 1;          // a band point needs an above point near it, and vice versa
    auto here = tall.find(detail::key(ix, iy));
    if (here != tall.end() && here->second == 3) return true;
    for (int dx = -c.tall_dilate; dx <= c.tall_dilate; ++dx) {
      for (int dy = -c.tall_dilate; dy <= c.tall_dilate; ++dy) {
        auto it = tall.find(detail::key(ix + dx, iy + dy));
        if (it != tall.end() && (it->second & want)) return true;
      }
    }
    return false;
  };
  std::vector<Eigen::Vector2d> band;
  std::vector<double> band_z;
  std::unordered_map<long long, int> above;  // cells (cluster_tol) with points above the cone top
  std::unordered_map<long long, int> above_f;  // the same in tall_cell cells (fine_cover)
  std::unordered_map<long long, double> low;  // lowest z per cell, band and below (floor level)
  for (const auto & p : pts) {
    if (p.head<2>().norm() > c.max_range) continue;
    if (p.z() <= c.z_max && p.z() > c.z_min - 0.30) {
      const long long k = detail::key(static_cast<int>(std::floor(p.x() / cell)), static_cast<int>(std::floor(p.y() / cell)));
      auto it = low.find(k);
      if (it == low.end() || p.z() < it->second) low[k] = p.z();
    }
    if ((p.z() >= c.z_min && p.z() <= c.z_max) || (p.z() > c.z_max + c.clear_gap && p.z() <= c.z_max + c.clear_height)) {
      if (in_tall(p)) continue;   // wall / leg / box column: neither a cone point nor "something above"
    }
    if (p.z() >= c.z_min && p.z() <= c.z_max) {
      band.push_back(p.head<2>());
      band_z.push_back(p.z());
    } else if (p.z() > c.z_max + c.clear_gap && p.z() <= c.z_max + c.clear_height) {
      above[detail::key(static_cast<int>(std::floor(p.x() / cell)), static_cast<int>(std::floor(p.y() / cell)))]++;
      if (c.tall_cell > 0.0) above_f[tkey(p)]++;
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
    if (c.fine_cover && c.tall_cell > 0.0) {
      for (int i : members) {
        const int fx = static_cast<int>(std::floor(band[i].x() / c.tall_cell)), fy = static_cast<int>(std::floor(band[i].y() / c.tall_cell));
        for (int dx = -1; dx <= 1 && !covered; ++dx) {
          for (int dy = -1; dy <= 1 && !covered; ++dy) covered = above_f.count(detail::key(fx + dx, fy + dy)) > 0;
        }
        if (covered) break;
      }
    } else {
      const int x0 = static_cast<int>(std::floor(lo.x() / cell)) - 1, x1 = static_cast<int>(std::floor(hi.x() / cell)) + 1;
      const int y0 = static_cast<int>(std::floor(lo.y() / cell)) - 1, y1 = static_cast<int>(std::floor(hi.y() / cell)) + 1;
      for (int ix = x0; ix <= x1 && !covered; ++ix) {
        for (int iy = y0; iy <= y1 && !covered; ++iy) {
          covered = above.count(detail::key(ix, iy)) > 0;
        }
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

// ---------------------------------------------------------------------------------------------
// LiDAR-only confirmation (no camera): a cluster becomes a cone only when it is seen again and again
// at the same place on the map. Floor bits that rise into the cone band (thresholds, ramps, body
// pitch) flicker from scan to scan and wander; a cone stays put. 10-30 bag without cones: 1.1 such
// clusters per scan before this filter.
struct PersistConfig
{
  double assoc{0.15};            // m, same track if closer than this
  int confirm_hits{5};           // scans that must see it ...
  double confirm_window_s{1.5};  // ... within this time
  double max_spread{0.08};       // m, sightings within this distance of the centre count (inliers)
  double min_ratio{0.7};         // and be seen in this share of the scans since it first appeared
                                 // (a cone in view: ~80 %; a flickering floor bit at a fixed spot: ~30 %)
  int min_scans{20};             // the share is judged over at least this many scans (1 s at 20 Hz):
                                 // over 5-8 scans a 30 % flicker reaches 60 % now and then (synthetic: 0.6 let 2.5 % of runs confirm it)
  double tentative_timeout_s{1.0};  // s, unconfirmed track dropped when not seen this long
  double alpha{0.2};             // update weight of a new sighting once confirmed
};

struct PersistTrack
{
  int id{0};
  Eigen::Vector2d pos{0, 0};
  std::deque<std::pair<double, Eigen::Vector2d>> hist;  // tentative: recent sightings
  bool confirmed{false};
  double last_seen{0};
  int hits{0};
};

class PersistTracker
{
public:
  explicit PersistTracker(PersistConfig c = {}) : c_(c) {}

  /// One scan's cone candidates in the world frame (already gated). Returns the ids confirmed now.
  std::vector<int> update(double t, const std::vector<Eigen::Vector2d> & pts)
  {
    std::vector<int> fresh;
    scan_t_.push_back(t);
    while (!scan_t_.empty() && scan_t_.front() < t - c_.confirm_window_s - 1.0) scan_t_.pop_front();
    std::vector<bool> used(tracks_.size(), false);
    for (const auto & p : pts) {
      int best = -1, near_used = -1;
      double bd = c_.assoc, bu = c_.assoc;
      for (size_t i = 0; i < tracks_.size(); ++i) {
        const double d = (tracks_[i].pos - p).norm();
        if (!used[i] && d < bd) {bd = d; best = static_cast<int>(i);}
        if (used[i] && d < bu) {bu = d; near_used = static_cast<int>(i);}
      }
      // a second point next to a track that already got one this scan: drop it - starting a new
      // track there split one cone into two half-seen tracks that never confirmed (synthetic: 2-3 %)
      if (best < 0 && near_used >= 0) continue;
      if (best < 0) {
        PersistTrack tr;
        tr.id = next_id_++;
        tr.pos = p;
        tr.hist.push_back({t, p});
        tr.last_seen = t;
        tr.hits = 1;
        tracks_.push_back(tr);
        used.push_back(true);
        continue;
      }
      PersistTrack & tr = tracks_[best];
      used[best] = true;
      tr.last_seen = t;
      ++tr.hits;
      if (tr.confirmed) {
        tr.pos = (1.0 - c_.alpha) * tr.pos + c_.alpha * p;
        continue;
      }
      tr.hist.push_back({t, p});
      while (!tr.hist.empty() && tr.hist.front().first < t - c_.confirm_window_s) tr.hist.pop_front();
      // centre = mean, then mean of the sightings near it: a stray floor bit that happens to land
      // next to a cone must not drag the centre or block the confirmation
      Eigen::Vector2d m = Eigen::Vector2d::Zero();
      for (const auto & h : tr.hist) m += h.second;
      m /= static_cast<double>(tr.hist.size());
      for (int pass = 0; pass < 2; ++pass) {
        Eigen::Vector2d mi = Eigen::Vector2d::Zero();
        int ni = 0;
        for (const auto & h : tr.hist) if ((h.second - m).norm() <= c_.max_spread) {mi += h.second; ++ni;}
        if (ni) m = mi / ni;
      }
      int inliers = 0;
      for (const auto & h : tr.hist) inliers += (h.second - m).norm() <= c_.max_spread ? 1 : 0;
      tr.pos = m;
      const double from = tr.hist.front().first;
      size_t scans = 0;
      for (double st : scan_t_) scans += (st >= from - 1e-9) ? 1 : 0;
      const double ratio = scans ? static_cast<double>(inliers) / scans : 0.0;
      if (inliers >= c_.confirm_hits && static_cast<int>(scans) >= c_.min_scans && ratio >= c_.min_ratio) {
        tr.confirmed = true;
        tr.hist.clear();
        fresh.push_back(tr.id);
      }
    }
    // merge tentative tracks that ended up on the same spot (into the older / confirmed one)
    for (size_t i = 0; i < tracks_.size(); ++i) {
      for (size_t j = i + 1; j < tracks_.size(); ++j) {
        if (tracks_[j].confirmed || (tracks_[i].pos - tracks_[j].pos).norm() > c_.max_spread) continue;
        for (const auto & h : tracks_[j].hist) tracks_[i].hist.push_back(h);
        std::sort(tracks_[i].hist.begin(), tracks_[i].hist.end(),
                  [](const auto & a, const auto & b) {return a.first < b.first;});
        tracks_[i].hits += tracks_[j].hits;
        tracks_[i].last_seen = std::max(tracks_[i].last_seen, tracks_[j].last_seen);
        tracks_[j].last_seen = -1e18;   // dropped below
        tracks_[j].pos = Eigen::Vector2d(1e9, 1e9);
      }
    }
    // unconfirmed tracks not seen for a while go; confirmed ones stay (cones do not move)
    std::vector<PersistTrack> keep;
    for (auto & tr : tracks_) {
      if (tr.confirmed || t - tr.last_seen <= c_.tentative_timeout_s) keep.push_back(tr);
    }
    tracks_.swap(keep);
    return fresh;
  }

  const std::vector<PersistTrack> & tracks() const {return tracks_;}
  size_t confirmedCount() const
  {
    size_t n = 0;
    for (const auto & t : tracks_) n += t.confirmed ? 1 : 0;
    return n;
  }
  void addConfirmed(int id, const Eigen::Vector2d & p)   // reloaded from the save file
  {
    PersistTrack tr;
    tr.id = id;
    tr.pos = p;
    tr.confirmed = true;
    tracks_.push_back(tr);
    next_id_ = std::max(next_id_, id + 1);
  }

private:
  PersistConfig c_;
  std::deque<double> scan_t_;
  std::vector<PersistTrack> tracks_;
  int next_id_{0};
};
}  // namespace cone_lidar
