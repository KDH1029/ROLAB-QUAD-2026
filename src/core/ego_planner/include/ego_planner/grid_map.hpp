#ifndef EGO_PLANNER__GRID_MAP_HPP_
#define EGO_PLANNER__GRID_MAP_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ego_planner
{

// Point filter for building the local obstacle grid (all from ego_planner_params.yaml)
struct CloudFilterConfig
{
  double min_range{0.25};       // ignore returns closer than this (m, horizontal from robot)
  double max_range{10.0};       // ignore returns farther than this (m)
  double body_x_min{-0.35};     // self-filter box in body frame (m)
  double body_x_max{0.33};
  double body_y_abs{0.22};
  double body_z_max{0.20};      // relative to robot base height
  double adaptive_slope{0.045}; // min height grows by this per metre (walking pitch/roll margin)
};

class GridMap
{
public:
  GridMap();
  ~GridMap() = default;

  void init(double resolution, double map_size_x, double map_size_y, double inflate_radius);
  void resetLocal(const Eigen::Vector2d & center);
  void updateStaticOnly(const Eigen::Vector2d & center);
  void updateCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
                   const Eigen::Vector3d & robot_pos,
                   double robot_yaw,
                   double min_h, double max_h);
  // Obstacles from a gravity-levelled 2D scan (e.g. cloud_bridge scan_mode:=vertical, which keeps
  // only vertical surfaces, so ramps and stair treads never become obstacles).
  void updateScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg,
                  const Eigen::Vector3d & robot_pos,
                  double robot_yaw);
  void setFilterConfig(const CloudFilterConfig & cfg) { filter_ = cfg; }
  // Extra obstacles (already in the grid frame), inserted with the static layer on every update
  void setExtraObstacles(const std::vector<Eigen::Vector2d> & pts) { extra_obstacles_ = pts; }

  bool isOccupied(const Eigen::Vector2d & pos) const;
  bool isOccupied(int idx_x, int idx_y) const;
  bool isInflated(const Eigen::Vector2d & pos) const;
  bool isInflated(int idx_x, int idx_y) const;

  bool isInMap(const Eigen::Vector2d & pos) const;
  bool isInMap(int idx_x, int idx_y) const;

  void posToIndex(const Eigen::Vector2d & pos, int & ix, int & iy) const;
  void indexToPos(int ix, int iy, Eigen::Vector2d & pos) const;

  double evaluateCostAndGrad(const Eigen::Vector2d & pos, Eigen::Vector2d & grad, double safe_dist) const;

  bool loadPriorMap(const std::string & bin_path);
  const std::vector<Eigen::Vector2d> & getStaticObstacles() const { return static_obstacles_; }

  std::shared_ptr<GridMap> clone() const {
    return std::make_shared<GridMap>(*this);
  }

  double getResolution() const { return resolution_; }
  const std::vector<uint8_t> & getOccupancy() const { return occupancy_; }
  const std::vector<uint8_t> & getInflated() const { return inflated_; }

  int getSizeX() const { return size_x_; }
  int getSizeY() const { return size_y_; }
  Eigen::Vector2d getOrigin() const { return origin_; }

private:
  double resolution_{0.1};
  double map_size_x_{20.0};
  double map_size_y_{20.0};
  double inflate_radius_{0.35};
  int inflate_cells_{4};

  int size_x_{200};
  int size_y_{200};
  Eigen::Vector2d origin_{-10.0, -10.0};

  std::vector<uint8_t> occupancy_;
  std::vector<uint8_t> inflated_;
  std::vector<Eigen::Vector2d> static_obstacles_;
  CloudFilterConfig filter_;
  std::vector<Eigen::Vector2d> extra_obstacles_;

  void insertStatic(std::vector<std::pair<int, int>> & occupied_indices);
  void markOccupied(const Eigen::Vector2d & pt, std::vector<std::pair<int, int>> & occupied_indices);
  void inflate(const std::vector<std::pair<int, int>> & occupied_indices);

  inline int toIndex(int ix, int iy) const {
    return iy * size_x_ + ix;
  }
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__GRID_MAP_HPP_

