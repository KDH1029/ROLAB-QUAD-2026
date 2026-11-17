#include "ego_planner/ego_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ego_planner
{

EgoPlannerNode::EgoPlannerNode()
: Node("ego_planner_node")
{
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_filtered");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  map_frame_ = declare_parameter<std::string>("map_frame", "camera_init");

  max_vel_ = declare_parameter<double>("max_vel", 0.8);
  max_acc_ = declare_parameter<double>("max_acc", 1.2);
  max_wz_ = declare_parameter<double>("max_wz", 1.5);
  goal_tol_ = declare_parameter<double>("goal_tol", 0.25);
  safe_dist_ = declare_parameter<double>("safe_dist", 0.45);
  // Rgosu (2026-11-03): A* 하드 제약은 safe_dist_(소프트 반발)보다 좁게 — 아래 참고.
  inflate_radius_ = declare_parameter<double>("inflate_radius", 0.28);
  min_obs_h_ = declare_parameter<double>("min_obs_h", 0.08);
  max_obs_h_ = declare_parameter<double>("max_obs_h", 0.80);
  replan_gap_ = declare_parameter<double>("replan_gap", 0.5);

  // Rgosu (2026-11-01): min_obs_h/max_obs_h/max_vel을 런타임에 ros2 param set으로
  // 바꿀 수 있게 콜백 등록. 트랙 구간(계단/경사 vs 콘 슬라럼)에 따라
  // track_sequencer_node가 이 값을 바꿔서, 같은 노드를 재시작 안 하고도
  // "장애물 높이필터 완화"/"속도 저하" 같은 구간별 동작을 적용한다.
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&EgoPlannerNode::onParamChange, this, std::placeholders::_1));

  // Initialize algorithmic modules
  grid_map_ = std::make_shared<GridMap>();
  // Rgosu (2026-11-03): 여기 네번째 인자가 A*의 하드 팽창 반경이다. 원래 safe_dist_
  // (0.45)를 그대로 썼는데, 좁은 실제 경기장 구간에서 A*가 "A* search failed to
  // reach target on local grid map!"로 통째로 실패하는 원인이었다(팀 review 브랜치
  // 실측 데이터로 확인). 옵티마이저 쪽 소프트 반발(safe_dist_, 아래 두 곳)은 그대로
  // 둔다 — 부드럽게 더 멀리 돌게 하는 건 유지, A*가 아예 길을 못 찾는 것만 고친다.
  grid_map_->init(0.1, 20.0, 20.0, inflate_radius_);

  std::string static_map = declare_parameter<std::string>("static_map_path", "");
  bool use_prior_map = declare_parameter<bool>("use_prior_map", false);
  if (use_prior_map && !static_map.empty()) {
    std::string resolved_path = static_map;
    if (resolved_path.rfind("~/", 0) == 0) {
      const char * home = getenv("HOME");
      if (home) resolved_path = std::string(home) + resolved_path.substr(1);
    }
    if (resolved_path[0] != '/' && access(resolved_path.c_str(), R_OK) != 0) {
      const char * home = getenv("HOME");
      if (home) {
        std::string ws_path = std::string(home) + "/go2_ws/" + resolved_path;
        if (access(ws_path.c_str(), R_OK) == 0) {
          resolved_path = ws_path;
        }
      }
    }
    if (grid_map_->loadPriorMap(resolved_path)) {
      RCLCPP_INFO(get_logger(), "Loaded clean obstacle avoidance layer (%zu points) from %s",
                  grid_map_->getStaticObstacles().size(), resolved_path.c_str());
      grid_map_->updateStaticOnly(Eigen::Vector2d(0.0, 0.0));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to load prior obstacle map from %s (resolved: %s). Using live LiDAR only.",
                  static_map.c_str(), resolved_path.c_str());
    }
  } else {
    RCLCPP_INFO(get_logger(), "Operating in Pure Real-time 360° LiDAR mode (No ghost walls).");
  }

  a_star_ = std::make_shared<AStar>();
  a_star_->init(grid_map_);

  optimizer_ = std::make_shared<BsplineOptimizer>();
  optimizer_->init(grid_map_, 1.0, 30.0, 2.0, max_vel_, max_acc_, safe_dist_);

  // ROS subscribers & publishers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&EgoPlannerNode::onOdom, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10, std::bind(&EgoPlannerNode::onGoal, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&EgoPlannerNode::onCloud, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_planner/trajectory", 10);
  map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ego_planner/grid_map", 10);

  // 20Hz control and planning loop
  timer_ = create_wall_timer(50ms, std::bind(&EgoPlannerNode::controlLoop, this));

  traj_start_time_ = now();
  last_replan_time_ = now();

  RCLCPP_INFO(get_logger(), "EGO-Planner Ground Node initialized (odom: %s, cloud: %s, cmd: %s)",
              odom_topic_.c_str(), cloud_topic_.c_str(), cmd_vel_topic_.c_str());
}

EgoPlannerNode::~EgoPlannerNode()
{
  if (is_replanning_ && async_plan_future_.valid()) {
    async_plan_future_.wait_for(100ms);
  }
}

// Rgosu (2026-11-01): ros2 param set으로 들어온 변경을 실제 멤버 변수에 반영한다.
// 이름이 일치하는 파라미터만 처리하고, 전부 성공으로 응답한다(값 검증은 최소한만 —
// 음수 속도 등 명백히 잘못된 값만 거른다).
rcl_interfaces::msg::SetParametersResult EgoPlannerNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "min_obs_h") {
      min_obs_h_ = p.as_double();
      RCLCPP_INFO(get_logger(), "[Rgosu] min_obs_h 런타임 변경 -> %.3f m", min_obs_h_);
    } else if (p.get_name() == "max_obs_h") {
      max_obs_h_ = p.as_double();
      RCLCPP_INFO(get_logger(), "[Rgosu] max_obs_h 런타임 변경 -> %.3f m", max_obs_h_);
    } else if (p.get_name() == "max_vel") {
      double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "max_vel must be positive";
        continue;
      }
      max_vel_ = v;
      RCLCPP_INFO(get_logger(), "[Rgosu] max_vel 런타임 변경 -> %.3f m/s", max_vel_);
    }
  }
  return result;
}

void EgoPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  has_odom_ = true;
  robot_pos_.x() = msg->pose.pose.position.x;
  robot_pos_.y() = msg->pose.pose.position.y;
  robot_pos_.z() = msg->pose.pose.position.z;

  robot_vel_.x() = msg->twist.twist.linear.x;
  robot_vel_.y() = msg->twist.twist.linear.y;
  robot_vel_.z() = msg->twist.twist.linear.z;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, robot_yaw_);

  if (!msg->header.frame_id.empty()) {
    map_frame_ = msg->header.frame_id;
  }

  if (!has_cloud_ && !grid_map_->getStaticObstacles().empty()) {
    grid_map_->updateStaticOnly(Eigen::Vector2d(robot_pos_.x(), robot_pos_.y()));
  }
}

void EgoPlannerNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!has_odom_) return;
  has_cloud_ = true;
  grid_map_->updateCloud(msg, robot_pos_, robot_yaw_, min_obs_h_, max_obs_h_);
}

void EgoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_odom_) {
    RCLCPP_WARN(get_logger(), "Received goal pose but no Odometry yet!");
    return;
  }

  // Cancel any running async replan
  if (is_replanning_ && async_plan_future_.valid()) {
    async_plan_future_.wait_for(20ms);
  }
  is_replanning_ = false;

  goal_pos_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
  has_goal_ = true;
  state_ = PlannerState::PLANNING;

  RCLCPP_INFO(get_logger(), "Received new target goal: (%.2f, %.2f)", goal_pos_.x(), goal_pos_.y());

  Eigen::Vector2d start_pos(robot_pos_.x(), robot_pos_.y());
  Eigen::Vector2d start_vel(robot_vel_.x(), robot_vel_.y());

  if (planTrajectory(start_pos, start_vel, goal_pos_)) {
    state_ = PlannerState::EXECUTING;
    traj_start_time_ = now();
    last_replan_time_ = now();
    publishVisualizations();
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to find valid collision-free trajectory to target!");
    state_ = PlannerState::IDLE;
  }
}

bool EgoPlannerNode::planTrajectory(const Eigen::Vector2d & start_pos,
                                   const Eigen::Vector2d & start_vel,
                                   const Eigen::Vector2d & goal_pos)
{
  // Clip distant goal to local planning horizon (7.5m) within 20m x 20m local grid
  Eigen::Vector2d local_subgoal = goal_pos;
  const double dist_to_goal = (goal_pos - start_pos).norm();
  const double max_horizon = 7.5;
  if (dist_to_goal > max_horizon) {
    local_subgoal = start_pos + (goal_pos - start_pos).normalized() * max_horizon;
  }

  std::vector<Eigen::Vector2d> a_star_path;
  if (!a_star_->search(start_pos, local_subgoal, a_star_path)) {
    RCLCPP_WARN(get_logger(), "A* search failed to reach target on local grid map!");
    return false;
  }

  if (!optimizer_->optimizeWithContinuity(a_star_path, start_pos, start_vel, Eigen::Vector2d::Zero(), current_traj_)) {
    RCLCPP_WARN(get_logger(), "B-spline optimization failed!");
    return false;
  }

  return true;
}

bool EgoPlannerNode::asyncReplanWorker(const std::shared_ptr<GridMap> & map_snapshot,
                                      const Eigen::Vector2d & p0,
                                      const Eigen::Vector2d & v0,
                                      const Eigen::Vector2d & a0,
                                      const Eigen::Vector2d & goal)
{
  if (!map_snapshot) return false;

  Eigen::Vector2d local_subgoal = goal;
  const double dist_to_goal = (goal - p0).norm();
  const double max_horizon = 7.5;
  if (dist_to_goal > max_horizon) {
    local_subgoal = p0 + (goal - p0).normalized() * max_horizon;
  }

  AStar astar;
  astar.init(map_snapshot);
  std::vector<Eigen::Vector2d> a_star_path;
  if (!astar.search(p0, local_subgoal, a_star_path)) {
    return false;
  }

  BsplineOptimizer opt;
  opt.init(map_snapshot, 1.0, 30.0, 2.0, max_vel_, max_acc_, safe_dist_);
  BsplineTrajectory traj;
  if (!opt.optimizeWithContinuity(a_star_path, p0, v0, a0, traj)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(traj_mutex_);
  next_traj_ = std::move(traj);
  return true;
}

bool EgoPlannerNode::checkCollisionOnTrajectory(double horizon_s) const
{
  if (current_traj_.getControlPoints().empty()) return true;

  const double t_cur = (now() - traj_start_time_).seconds();
  const double dt_check = 0.08;
  const double t_end = std::min(t_cur + horizon_s, current_traj_.getTotalDuration());

  for (double t = std::max(0.0, t_cur); t <= t_end; t += dt_check) {
    Eigen::Vector2d pt = current_traj_.evaluatePosition(t);
    if (grid_map_->isInflated(pt)) {
      return true;
    }
  }
  return false;
}

void EgoPlannerNode::controlLoop()
{
  // Periodically publish obstacle grid map and planned trajectory visualization (5 Hz)
  static int vis_cnt = 0;
  if (++vis_cnt % 4 == 0) {
    publishMapVisualization();
    if (state_ == PlannerState::EXECUTING) {
      publishVisualizations();
    }
  }

  if (state_ != PlannerState::EXECUTING) {
    return;
  }

  const double dist_to_goal = (robot_pos_.head<2>() - goal_pos_).norm();
  if (dist_to_goal <= goal_tol_) {
    RCLCPP_INFO(get_logger(), "Target goal reached! (distance: %.2f m)", dist_to_goal);
    stopRobot();
    state_ = PlannerState::GOAL_REACHED;
    has_goal_ = false;
    return;
  }

  // 1. ABPolicy: Non-blocking check for completed background trajectory
  if (is_replanning_) {
    if (async_plan_future_.wait_for(0s) == std::future_status::ready) {
      bool success = async_plan_future_.get();
      is_replanning_ = false;
      if (success) {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        current_traj_ = std::move(next_traj_);
        traj_start_time_ = next_traj_start_time_;
        last_replan_time_ = now();
        publishVisualizations();
        RCLCPP_INFO(get_logger(),
          "[ABPolicy] Seamless C2 Handover succeeded! New duration: %.2fs (StitchLag: %.1fms)",
          current_traj_.getTotalDuration(),
          (now() - next_traj_start_time_).seconds() * 1000.0);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[ABPolicy] Async replanning attempt failed. Continuing current trajectory.");
      }
    }
  }

  const double t_cur = (now() - traj_start_time_).seconds();
  const double total_duration = current_traj_.getTotalDuration();

  // 2. Check replanning trigger conditions
  const bool collision_ahead = checkCollisionOnTrajectory(1.5);
  const bool traj_expiring = (t_cur >= total_duration - 0.3);
  const double time_since_replan = (now() - last_replan_time_).seconds();

  // If trajectory fully expired and no background plan is running, bring robot to safe stop
  if (t_cur >= total_duration + 0.2 && !is_replanning_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Trajectory finished, awaiting new commands.");
    stopRobot();
    return;
  }

  const bool need_replan = (collision_ahead && time_since_replan >= 0.2) ||
                           (traj_expiring && time_since_replan >= 0.2) ||
                           (time_since_replan >= replan_gap_);

  if (!is_replanning_ && need_replan) {
    // ABPolicy Asynchronous Planning Decoupling:
    // Estimate computation time dt_est (60ms) and evaluate handover state at t_stitch
    const double dt_est = 0.06;
    double t_stitch = t_cur + dt_est;
    if (t_stitch > total_duration) {
      t_stitch = total_duration;
    }

    Eigen::Vector2d p0 = current_traj_.evaluatePosition(t_stitch);
    Eigen::Vector2d v0 = current_traj_.evaluateVelocity(t_stitch);
    Eigen::Vector2d a0 = current_traj_.evaluateAcceleration(t_stitch);

    if (t_cur > total_duration || v0.norm() < 0.02) {
      p0 = robot_pos_.head<2>();
      v0 = robot_vel_.head<2>();
      a0.setZero();
    }

    rclcpp::Time stitch_wall_time = traj_start_time_ + rclcpp::Duration::from_seconds(t_stitch);
    auto map_snapshot = grid_map_->clone();

    is_replanning_ = true;
    next_traj_start_time_ = stitch_wall_time;
    async_plan_future_ = std::async(
      std::launch::async,
      [this, map_snapshot, p0, v0, a0, goal = goal_pos_]() {
        return asyncReplanWorker(map_snapshot, p0, v0, a0, goal);
      });
  }

  // Evaluate B-spline trajectory at current time
  Eigen::Vector2d des_pos = current_traj_.evaluatePosition(t_cur);
  Eigen::Vector2d des_vel = current_traj_.evaluateVelocity(t_cur);

  // Position error in global frame
  Eigen::Vector2d pos_err = des_pos - robot_pos_.head<2>();

  // Rotate velocity and error into robot body frame
  const double c_yaw = std::cos(robot_yaw_);
  const double s_yaw = std::sin(robot_yaw_);

  // Body frame tracking
  const double err_body_x =  c_yaw * pos_err.x() + s_yaw * pos_err.y();
  const double err_body_y = -s_yaw * pos_err.x() + c_yaw * pos_err.y();

  const double v_ff_x =  c_yaw * des_vel.x() + s_yaw * des_vel.y();
  const double v_ff_y = -s_yaw * des_vel.x() + c_yaw * des_vel.y();

  // Desired heading aligns with trajectory tangent
  double des_yaw = robot_yaw_;
  if (des_vel.norm() > 0.05) {
    des_yaw = std::atan2(des_vel.y(), des_vel.x());
  } else if (dist_to_goal > 0.1) {
    Eigen::Vector2d to_goal = goal_pos_ - robot_pos_.head<2>();
    des_yaw = std::atan2(to_goal.y(), to_goal.x());
  }

  // Heading error wrapped to [-pi, pi]
  double yaw_err = des_yaw - robot_yaw_;
  while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

  // Heading-priority scaling:
  // Smoothly scale forward drive with cos(yaw_err).
  // If heading error is large (> 68 deg = ~1.20 rad), prioritize in-place rotation to align.
  double heading_factor = std::max(0.0, std::cos(yaw_err));
  if (std::abs(yaw_err) > 1.20) {
    heading_factor = 0.0;
  }

  // Controller output (P + Feedforward with heading scaling)
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp((v_ff_x + 1.0 * err_body_x) * heading_factor, -max_vel_, max_vel_);
  cmd.linear.y = std::clamp((v_ff_y + 0.8 * err_body_y) * heading_factor, -0.25, 0.25);
  cmd.angular.z = std::clamp(2.5 * yaw_err, -max_wz_, max_wz_);

  cmd_vel_pub_->publish(cmd);
}

void EgoPlannerNode::stopRobot()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

void EgoPlannerNode::publishVisualizations()
{
  // 1. Publish planned path
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map_frame_;

  const double duration = current_traj_.getTotalDuration();
  const int n_samples = 25;
  for (int i = 0; i <= n_samples; ++i) {
    const double t = duration * (static_cast<double>(i) / n_samples);
    Eigen::Vector2d p = current_traj_.evaluatePosition(t);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = robot_pos_.z();
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }
  path_pub_->publish(path_msg);

  publishMapVisualization();
}

void EgoPlannerNode::publishMapVisualization()
{
  if (!map_vis_pub_ || map_vis_pub_->get_subscription_count() == 0) {
    // Only generate point cloud if someone is listening or for debug
  }

  auto map_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  map_cloud->header.stamp = now();
  map_cloud->header.frame_id = map_frame_;

  const auto & inflated = grid_map_->getInflated();
  const int sx = grid_map_->getSizeX();
  const int sy = grid_map_->getSizeY();

  std::vector<Eigen::Vector2d> obs_pts;
  obs_pts.reserve(1000);
  for (int y = 0; y < sy; y += 2) {
    for (int x = 0; x < sx; x += 2) {
      if (inflated[y * sx + x] > 0) {
        Eigen::Vector2d pos;
        grid_map_->indexToPos(x, y, pos);
        obs_pts.push_back(pos);
      }
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(*map_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(obs_pts.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(*map_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*map_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*map_cloud, "z");

  for (const auto & pt : obs_pts) {
    *it_x = pt.x();
    *it_y = pt.y();
    *it_z = robot_pos_.z();
    ++it_x; ++it_y; ++it_z;
  }

  map_vis_pub_->publish(std::move(map_cloud));
}

}  // namespace ego_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ego_planner::EgoPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

