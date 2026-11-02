#ifndef EGO_PLANNER__EGO_PLANNER_NODE_HPP_
#define EGO_PLANNER__EGO_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <future>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ego_planner/grid_map.hpp"
#include "ego_planner/a_star.hpp"
#include "ego_planner/bspline_optimizer.hpp"

namespace ego_planner
{

enum class PlannerState
{
  IDLE,
  PLANNING,
  EXECUTING,
  GOAL_REACHED
};

class EgoPlannerNode : public rclcpp::Node
{
public:
  EgoPlannerNode();
  ~EgoPlannerNode();

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void controlLoop();
  // Runtime changes via `ros2 param set` (merged from the team's tunable fork, extended)
  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> & params);
  void configurePlanners(AStar & astar, BsplineOptimizer & opt, const std::shared_ptr<GridMap> & map) const;
  bool planOn(const std::shared_ptr<GridMap> & map, const Eigen::Vector2d & p0, const Eigen::Vector2d & v0,
              const Eigen::Vector2d & a0, const Eigen::Vector2d & goal, BsplineTrajectory & out) const;

  bool planTrajectory(const Eigen::Vector2d & start_pos, const Eigen::Vector2d & start_vel,
                      const Eigen::Vector2d & goal_pos);
  bool asyncReplanWorker(const std::shared_ptr<GridMap> & map_snapshot,
                         const Eigen::Vector2d & p0,
                         const Eigen::Vector2d & v0,
                         const Eigen::Vector2d & a0,
                         const Eigen::Vector2d & goal);
  bool checkCollisionOnTrajectory(double horizon_s) const;
  void publishVisualizations();
  void publishMapVisualization();
  void stopRobot();

  // Components
  std::shared_ptr<GridMap> grid_map_;
  std::shared_ptr<AStar> a_star_;
  std::shared_ptr<BsplineOptimizer> optimizer_;
  BsplineTrajectory current_traj_;

  // ABPolicy Asynchronous Replanning & C2 Handover
  std::future<bool> async_plan_future_;
  std::atomic<bool> is_replanning_{false};
  std::mutex traj_mutex_;
  BsplineTrajectory next_traj_;
  rclcpp::Time next_traj_start_time_;

  // State
  PlannerState state_{PlannerState::IDLE};
  bool has_odom_{false};
  bool has_goal_{false};
  bool has_cloud_{false};
  Eigen::Vector3d robot_pos_{0, 0, 0};
  Eigen::Vector3d robot_vel_{0, 0, 0};
  double robot_yaw_{0.0};
  Eigen::Vector2d goal_pos_{0, 0};

  rclcpp::Time traj_start_time_;
  rclcpp::Time last_replan_time_;

  // Parameters
  std::string odom_topic_{"/Odometry"};
  std::string goal_topic_{"/goal_pose"};
  std::string cloud_topic_{"/velodyne_points"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string map_frame_{"camera_init"};

  double max_vel_{1.0};
  double max_acc_{1.5};
  double max_wz_{1.5};
  double goal_tol_{0.2};
  double safe_dist_{0.45};
  double min_obs_h_{0.08};
  double max_obs_h_{0.80};
  double replan_gap_{0.5};

  // Everything below comes from ego_planner_params.yaml (see comments there)
  std::string obstacle_source_{"cloud"};
  std::string scan_topic_{"/scan"};
  double inflate_radius_{0.28};
  double map_resolution_{0.1};
  double map_size_{20.0};
  double plan_horizon_{7.5};
  int astar_max_iters_{15000};
  double astar_min_partial_{0.8};
  double lambda_smooth_{1.0};
  double lambda_obs_{30.0};
  double lambda_feas_{2.0};
  double opt_dt_{0.2};
  int opt_iters_{35};
  double opt_step_{0.02};
  double opt_max_step_{0.04};
  double opt_ctrl_spacing_{0.8};
  bool validate_traj_{true};
  double collision_check_horizon_{1.5};
  double stop_horizon_{0.6};
  double replan_min_gap_{0.2};
  double stitch_lead_{0.06};
  double kp_x_{1.0};
  double kp_y_{0.8};
  double kp_yaw_{2.5};
  double max_vy_{0.25};
  double rotate_in_place_rad_{1.20};
  bool estimate_vel_from_pose_{true};
  CloudFilterConfig filter_cfg_;

  // Velocity estimate when the odometry carries no twist (FAST-LIO /Odometry)
  rclcpp::Time last_odom_time_;
  Eigen::Vector3d last_odom_pos_{0, 0, 0};
  bool have_last_odom_{false};
  bool blocked_stop_{false};

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_vis_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__EGO_PLANNER_NODE_HPP_

