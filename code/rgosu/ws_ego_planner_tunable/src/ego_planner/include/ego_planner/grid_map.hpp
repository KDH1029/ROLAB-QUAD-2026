#ifndef EGO_PLANNER__GRID_MAP_HPP_
#define EGO_PLANNER__GRID_MAP_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ego_planner
{

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

  inline int toIndex(int ix, int iy) const {
    return iy * size_x_ + ix;
  }
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__GRID_MAP_HPP_

