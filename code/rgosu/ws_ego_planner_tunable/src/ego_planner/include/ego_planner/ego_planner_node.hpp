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
  void controlLoop();
  // Rgosu: 런타임 파라미터 변경(2026-11-01) -- track_sequencer_node가 구간별로
  // min_obs_h/max_obs_h(장애물 높이필터)/max_vel(속도)을 ros2 param set으로
  // 바꾸면 이 콜백이 불려서 멤버 변수에 실제로 반영된다. 원래 코드는 생성자에서
  // declare_parameter로 한 번만 읽고 끝이라 런타임 변경이 전혀 안 먹혔음.
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

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
  // Rgosu (2026-11-03): A*의 하드 제약(장애물 안쪽은 절대 못 들어감)과 옵티마이저의
  // 소프트 반발거리를 분리. 원래 코드는 safe_dist_ 하나를 둘 다로 썼는데, 팀
  // review 브랜치 실측(누리관 bag)으로 "실제 걸은 경로의 42.9%가 0.45m 팽창 안에
  // 들어간다"는 게 밝혀짐 — 좁은 구간에서 A*가 통째로 실패하는 원인이었음.
  double inflate_radius_{0.28};
  double min_obs_h_{0.08};
  double max_obs_h_{0.80};
  double replan_gap_{0.5};

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_vis_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Rgosu: 런타임 파라미터 변경 콜백 핸들 (등록 해제되지 않게 멤버로 보관해야 함)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__EGO_PLANNER_NODE_HPP_

