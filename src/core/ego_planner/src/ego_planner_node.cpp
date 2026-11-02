#include "ego_planner/ego_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

namespace ego_planner
{

EgoPlannerNode::EgoPlannerNode()
: rclcpp_lifecycle::LifecycleNode("ego_planner_node")
{
  // Topics / frames
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  obstacle_source_ = declare_parameter<std::string>("obstacle_source", "cloud");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  map_frame_ = declare_parameter<std::string>("map_frame", "camera_init");

  // Limits and safety distances
  max_vel_ = declare_parameter<double>("max_vel", 0.8);
  max_acc_ = declare_parameter<double>("max_acc", 1.2);
  max_wz_ = declare_parameter<double>("max_wz", 1.5);
  max_vy_ = declare_parameter<double>("max_vy", 0.25);
  goal_tol_ = declare_parameter<double>("goal_tol", 0.25);
  inflate_radius_ = declare_parameter<double>("inflate_radius", 0.28);
  safe_dist_ = declare_parameter<double>("safe_dist", 0.45);
  min_obs_h_ = declare_parameter<double>("min_obs_h", -0.17);
  max_obs_h_ = declare_parameter<double>("max_obs_h", 1.10);
  replan_gap_ = declare_parameter<double>("replan_gap", 0.4);

  // Local grid
  map_resolution_ = declare_parameter<double>("grid.resolution", 0.1);
  map_size_ = declare_parameter<double>("grid.size", 20.0);
  filter_cfg_.min_range = declare_parameter<double>("grid.min_range", 0.25);
  filter_cfg_.max_range = declare_parameter<double>("grid.max_range", 10.0);
  filter_cfg_.body_x_min = declare_parameter<double>("grid.body_x_min", -0.35);
  filter_cfg_.body_x_max = declare_parameter<double>("grid.body_x_max", 0.33);
  filter_cfg_.body_y_abs = declare_parameter<double>("grid.body_y_abs", 0.22);
  filter_cfg_.body_z_max = declare_parameter<double>("grid.body_z_max", 0.20);
  filter_cfg_.adaptive_slope = declare_parameter<double>("grid.adaptive_slope", 0.045);

  // Search / optimizer
  plan_horizon_ = declare_parameter<double>("planner.horizon", 7.5);
  astar_max_iters_ = declare_parameter<int>("planner.astar_max_iters", 15000);
  astar_min_partial_ = declare_parameter<double>("planner.astar_min_partial", 0.8);
  lambda_smooth_ = declare_parameter<double>("optimizer.lambda_smooth", 1.0);
  lambda_obs_ = declare_parameter<double>("optimizer.lambda_obs", 30.0);
  lambda_feas_ = declare_parameter<double>("optimizer.lambda_feas", 2.0);
  opt_dt_ = declare_parameter<double>("optimizer.dt", 0.2);
  opt_iters_ = declare_parameter<int>("optimizer.iterations", 35);
  opt_step_ = declare_parameter<double>("optimizer.step_size", 0.02);
  opt_max_step_ = declare_parameter<double>("optimizer.max_step", 0.04);
  opt_ctrl_spacing_ = declare_parameter<double>("optimizer.ctrl_spacing", 0.8);
  validate_traj_ = declare_parameter<bool>("optimizer.validate", true);

  // Replanning / safety
  collision_check_horizon_ = declare_parameter<double>("replan.collision_horizon", 1.5);
  stop_horizon_ = declare_parameter<double>("replan.stop_horizon", 0.6);
  replan_min_gap_ = declare_parameter<double>("replan.min_gap", 0.2);
  stitch_lead_ = declare_parameter<double>("replan.stitch_lead", 0.06);

  // Tracking controller
  kp_x_ = declare_parameter<double>("control.kp_x", 1.0);
  kp_y_ = declare_parameter<double>("control.kp_y", 0.8);
  kp_yaw_ = declare_parameter<double>("control.kp_yaw", 2.5);
  rotate_in_place_rad_ = declare_parameter<double>("control.rotate_in_place_rad", 1.20);
  estimate_vel_from_pose_ = declare_parameter<bool>("control.estimate_vel_from_pose", true);

  goal_base_frame_ = declare_parameter<std::string>("goal_base_frame", "base_link");
  static_map_path_ = declare_parameter<std::string>("static_map_path", "");
  use_prior_map_ = declare_parameter<bool>("use_prior_map", false);

  // Runtime changes are accepted in any state; they take effect on the next plan.
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&EgoPlannerNode::onParamChange, this, std::placeholders::_1));
}

CallbackReturn EgoPlannerNode::on_configure(const rclcpp_lifecycle::State &)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize algorithmic modules
  grid_map_ = std::make_shared<GridMap>();
  grid_map_->init(map_resolution_, map_size_, map_size_, inflate_radius_);
  grid_map_->setFilterConfig(filter_cfg_);

  if (use_prior_map_ && !static_map_path_.empty()) {
    std::string resolved_path = static_map_path_;
    if (resolved_path.rfind("~/", 0) == 0) {
      const char * home = getenv("HOME");
      if (home) resolved_path = std::string(home) + resolved_path.substr(1);
    }
    if (grid_map_->loadPriorMap(resolved_path)) {
      RCLCPP_INFO(get_logger(), "Loaded prior obstacle layer (%zu points) from %s",
                  grid_map_->getStaticObstacles().size(), resolved_path.c_str());
      grid_map_->updateStaticOnly(Eigen::Vector2d(0.0, 0.0));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to load prior obstacle map from %s. Using live sensing only.",
                  resolved_path.c_str());
    }
  }

  a_star_ = std::make_shared<AStar>();
  optimizer_ = std::make_shared<BsplineOptimizer>();
  configurePlanners(*a_star_, *optimizer_, grid_map_);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/ego_planner/trajectory", 10);
  map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/ego_planner/grid_map", 10);
  return CallbackReturn::SUCCESS;
}

CallbackReturn EgoPlannerNode::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();
  path_pub_->on_activate();
  map_vis_pub_->on_activate();
  state_ = PlannerState::IDLE;
  has_goal_ = false;
  blocked_stop_ = false;

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&EgoPlannerNode::onOdom, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10, std::bind(&EgoPlannerNode::onGoal, this, std::placeholders::_1));
  if (obstacle_source_ == "scan") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EgoPlannerNode::onScan, this, std::placeholders::_1));
  } else {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EgoPlannerNode::onCloud, this, std::placeholders::_1));
  }

  // 20Hz control and planning loop
  timer_ = create_wall_timer(50ms, std::bind(&EgoPlannerNode::controlLoop, this));

  traj_start_time_ = now();
  last_replan_time_ = now();

  RCLCPP_INFO(get_logger(),
    "EGO-Planner ready (odom: %s, obstacles: %s %s, inflate %.2f m, safe_dist %.2f m)",
    odom_topic_.c_str(), obstacle_source_.c_str(),
    obstacle_source_ == "scan" ? scan_topic_.c_str() : cloud_topic_.c_str(),
    inflate_radius_, safe_dist_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn EgoPlannerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivation is a stop: zero velocity first, then drop inputs and the control loop.
  timer_.reset();
  if (is_replanning_ && async_plan_future_.valid()) {
    async_plan_future_.wait_for(100ms);
  }
  is_replanning_ = false;
  stopRobot();
  state_ = PlannerState::IDLE;
  has_goal_ = false;
  odom_sub_.reset();
  goal_sub_.reset();
  scan_sub_.reset();
  cloud_sub_.reset();
  has_odom_ = has_cloud_ = have_last_odom_ = false;
  cmd_vel_pub_->on_deactivate();
  path_pub_->on_deactivate();
  map_vis_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn EgoPlannerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_.reset();
  path_pub_.reset();
  map_vis_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  a_star_.reset();
  optimizer_.reset();
  grid_map_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn EgoPlannerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  if (cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
    stopRobot();
  }
  odom_sub_.reset();
  goal_sub_.reset();
  scan_sub_.reset();
  cloud_sub_.reset();
  return CallbackReturn::SUCCESS;
}

EgoPlannerNode::~EgoPlannerNode()
{
  if (is_replanning_ && async_plan_future_.valid()) {
    async_plan_future_.wait_for(100ms);
  }
}

void EgoPlannerNode::configurePlanners(
  AStar & astar, BsplineOptimizer & opt, const std::shared_ptr<GridMap> & map) const
{
  astar.init(map);
  astar.setSearchParams(astar_max_iters_, astar_min_partial_);
  opt.init(map, lambda_smooth_, lambda_obs_, lambda_feas_, max_vel_, max_acc_, safe_dist_);
  opt.setSolverParams(opt_dt_, opt_iters_, opt_step_, opt_max_step_, opt_ctrl_spacing_);
}

rcl_interfaces::msg::SetParametersResult EgoPlannerNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "min_obs_h") {
      min_obs_h_ = p.as_double();
    } else if (n == "max_obs_h") {
      max_obs_h_ = p.as_double();
    } else if (n == "max_vel" || n == "max_acc" || n == "safe_dist") {
      const double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = n + " must be positive";
        continue;
      }
      (n == "max_vel" ? max_vel_ : n == "max_acc" ? max_acc_ : safe_dist_) = v;
    } else if (n == "max_wz") {
      max_wz_ = p.as_double();
    } else if (n == "max_vy") {
      max_vy_ = p.as_double();
    } else if (n == "goal_tol") {
      goal_tol_ = p.as_double();
    } else {
      continue;  // others need a restart (grid size, inflation, topics)
    }
    RCLCPP_INFO(get_logger(), "runtime parameter %s -> %s", n.c_str(), p.value_to_string().c_str());
  }
  // The original only read limits in the constructor; re-init so the next plan uses new ones.
  if (a_star_ && optimizer_ && grid_map_) {
    configurePlanners(*a_star_, *optimizer_, grid_map_);
  }
  return result;
}

void EgoPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time t(msg->header.stamp);
  const Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Vector3d vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  // FAST-LIO /Odometry carries no twist; without this the replan start velocity is always 0.
  if (estimate_vel_from_pose_ && vel.norm() < 1e-9 && have_last_odom_) {
    const double dt = (t - last_odom_time_).seconds();
    if (dt > 1e-3 && dt < 0.5) {
      vel = 0.7 * robot_vel_ + 0.3 * (pos - last_odom_pos_) / dt;
    } else {
      vel = robot_vel_;
    }
  }
  last_odom_time_ = t;
  last_odom_pos_ = pos;
  have_last_odom_ = true;

  has_odom_ = true;
  robot_pos_ = pos;
  robot_vel_ = vel;

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

void EgoPlannerNode::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!has_odom_) return;
  has_cloud_ = true;
  grid_map_->updateScan(msg, robot_pos_, robot_yaw_);
}

void EgoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_odom_) {
    RCLCPP_WARN(get_logger(), "Received goal pose but no Odometry yet!");
    return;
  }
  if (is_replanning_ && async_plan_future_.valid()) {
    async_plan_future_.wait_for(20ms);
  }
  is_replanning_ = false;

  goal_frame_ = msg->header.frame_id;
  goal_in_frame_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
  if (!resolveGoal(goal_pos_)) {
    RCLCPP_ERROR(get_logger(), "Goal frame '%s' cannot be related to '%s' (no TF to %s)",
                 goal_frame_.c_str(), map_frame_.c_str(), goal_base_frame_.c_str());
    return;
  }
  has_goal_ = true;
  state_ = PlannerState::PLANNING;
  RCLCPP_INFO(get_logger(), "Received new target goal: (%.2f, %.2f) in %s -> (%.2f, %.2f) in %s",
              goal_in_frame_.x(), goal_in_frame_.y(), goal_frame_.empty() ? map_frame_.c_str() : goal_frame_.c_str(),
              goal_pos_.x(), goal_pos_.y(), map_frame_.c_str());

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

bool EgoPlannerNode::planOn(
  const std::shared_ptr<GridMap> & map, const Eigen::Vector2d & p0, const Eigen::Vector2d & v0,
  const Eigen::Vector2d & a0, const Eigen::Vector2d & goal, BsplineTrajectory & out) const
{
  // Clip distant goal to the local planning horizon
  Eigen::Vector2d local_subgoal = goal;
  if ((goal - p0).norm() > plan_horizon_) {
    local_subgoal = p0 + (goal - p0).normalized() * plan_horizon_;
  }
  AStar astar;
  BsplineOptimizer opt;
  configurePlanners(astar, opt, map);
  std::vector<Eigen::Vector2d> a_star_path;
  if (!astar.search(p0, local_subgoal, a_star_path)) {
    return false;
  }
  BsplineTrajectory traj;
  if (!opt.optimizeWithContinuity(a_star_path, p0, v0, a0, traj)) {
    return false;
  }
  // The smoothed curve can cut corners into the inflated zone even when the A* path is free.
  if (validate_traj_ && !opt.isCollisionFree(traj, 0.05)) {
    return false;
  }
  out = std::move(traj);
  return true;
}

bool EgoPlannerNode::planTrajectory(const Eigen::Vector2d & start_pos,
                                   const Eigen::Vector2d & start_vel,
                                   const Eigen::Vector2d & goal_pos)
{
  if (!planOn(grid_map_, start_pos, start_vel, Eigen::Vector2d::Zero(), goal_pos, current_traj_)) {
    RCLCPP_WARN(get_logger(), "No collision-free trajectory on the local grid");
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
  BsplineTrajectory traj;
  if (!planOn(map_snapshot, p0, v0, a0, goal, traj)) {
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
  // Skip the stretch the robot is already standing in (closer to a wall than inflate_radius);
  // otherwise every check fires and the robot holds forever next to the wall.
  bool escaping = true;
  for (double t = std::max(0.0, t_cur); t <= t_end; t += dt_check) {
    const bool hit = grid_map_->isInflated(current_traj_.evaluatePosition(t));
    if (!hit) {
      escaping = false;
    } else if (!escaping) {
      return true;
    }
  }
  return false;
}

void EgoPlannerNode::controlLoop()
{
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

  // 1. Non-blocking check for a completed background trajectory
  if (is_replanning_) {
    if (async_plan_future_.wait_for(0s) == std::future_status::ready) {
      bool success = async_plan_future_.get();
      is_replanning_ = false;
      if (success) {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        current_traj_ = std::move(next_traj_);
        traj_start_time_ = next_traj_start_time_;
        last_replan_time_ = now();
        if (blocked_stop_) {
          RCLCPP_INFO(get_logger(), "Free trajectory found again, resuming");
          blocked_stop_ = false;
        }
        publishVisualizations();
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Async replanning failed. Continuing current trajectory.");
      }
    }
  }

  const double t_cur = (now() - traj_start_time_).seconds();
  const double total_duration = current_traj_.getTotalDuration();

  // 2. Replanning triggers
  const bool collision_ahead = checkCollisionOnTrajectory(collision_check_horizon_);
  const bool traj_expiring = (t_cur >= total_duration - 0.3);
  const double time_since_replan = (now() - last_replan_time_).seconds();

  if (t_cur >= total_duration + 0.2 && !is_replanning_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Trajectory finished, awaiting new commands.");
    stopRobot();
    return;
  }

  const bool need_replan = (collision_ahead && time_since_replan >= replan_min_gap_) ||
                           (traj_expiring && time_since_replan >= replan_min_gap_) ||
                           (time_since_replan >= replan_gap_);

  if (!is_replanning_ && need_replan) {
    resolveGoal(goal_pos_);  // follow map corrections (keeps the last value if TF is missing)
    double t_stitch = std::min(t_cur + stitch_lead_, total_duration);
    Eigen::Vector2d p0 = current_traj_.evaluatePosition(t_stitch);
    Eigen::Vector2d v0 = current_traj_.evaluateVelocity(t_stitch);
    Eigen::Vector2d a0 = current_traj_.evaluateAcceleration(t_stitch);
    if (t_cur > total_duration || v0.norm() < 0.02 || blocked_stop_) {
      p0 = robot_pos_.head<2>();
      v0 = blocked_stop_ ? Eigen::Vector2d::Zero() : Eigen::Vector2d(robot_vel_.head<2>());
      a0.setZero();
      t_stitch = t_cur;
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

  // 3. Safety: the original kept following a trajectory that runs into an obstacle while
  // replans failed. Hold still if the collision is within stop_horizon; replanning continues.
  if (checkCollisionOnTrajectory(stop_horizon_)) {
    if (!blocked_stop_) {
      RCLCPP_WARN(get_logger(), "Obstacle on trajectory within %.1f s and no free replan: holding", stop_horizon_);
    }
    blocked_stop_ = true;
    stopRobot();
    return;
  }

  // 4. Track the B-spline (feedforward + P, body frame, holonomic)
  Eigen::Vector2d des_pos = current_traj_.evaluatePosition(t_cur);
  Eigen::Vector2d des_vel = current_traj_.evaluateVelocity(t_cur);
  Eigen::Vector2d pos_err = des_pos - robot_pos_.head<2>();

  const double c_yaw = std::cos(robot_yaw_);
  const double s_yaw = std::sin(robot_yaw_);
  const double err_body_x =  c_yaw * pos_err.x() + s_yaw * pos_err.y();
  const double err_body_y = -s_yaw * pos_err.x() + c_yaw * pos_err.y();
  const double v_ff_x =  c_yaw * des_vel.x() + s_yaw * des_vel.y();
  const double v_ff_y = -s_yaw * des_vel.x() + c_yaw * des_vel.y();

  double des_yaw = robot_yaw_;
  if (des_vel.norm() > 0.05) {
    des_yaw = std::atan2(des_vel.y(), des_vel.x());
  } else if (dist_to_goal > 0.1) {
    Eigen::Vector2d to_goal = goal_pos_ - robot_pos_.head<2>();
    des_yaw = std::atan2(to_goal.y(), to_goal.x());
  }
  double yaw_err = std::remainder(des_yaw - robot_yaw_, 2.0 * M_PI);

  double heading_factor = std::max(0.0, std::cos(yaw_err));
  if (std::abs(yaw_err) > rotate_in_place_rad_) {
    heading_factor = 0.0;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp((v_ff_x + kp_x_ * err_body_x) * heading_factor, -max_vel_, max_vel_);
  cmd.linear.y = std::clamp((v_ff_y + kp_y_ * err_body_y) * heading_factor, -max_vy_, max_vy_);
  cmd.angular.z = std::clamp(kp_yaw_ * yaw_err, -max_wz_, max_wz_);
  cmd_vel_pub_->publish(cmd);
}

bool EgoPlannerNode::resolveGoal(Eigen::Vector2d & goal_odom)
{
  if (goal_frame_.empty() || goal_frame_ == map_frame_) {
    goal_odom = goal_in_frame_;
    return true;
  }
  geometry_msgs::msg::TransformStamped tr;
  try {
    tr = tf_buffer_->lookupTransform(goal_frame_, goal_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }
  // Robot pose in the goal frame
  tf2::Quaternion q(tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w);
  double r, p, yaw_g;
  tf2::Matrix3x3(q).getRPY(r, p, yaw_g);
  const Eigen::Vector2d pos_g(tr.transform.translation.x, tr.transform.translation.y);
  // Goal relative to the robot, then placed around the robot pose in the odometry frame
  const Eigen::Vector2d d = goal_in_frame_ - pos_g;
  const double cg = std::cos(yaw_g), sg = std::sin(yaw_g);
  const Eigen::Vector2d rel(cg * d.x() + sg * d.y(), -sg * d.x() + cg * d.y());
  const double co = std::cos(robot_yaw_), so = std::sin(robot_yaw_);
  goal_odom = robot_pos_.head<2>() + Eigen::Vector2d(co * rel.x() - so * rel.y(), so * rel.x() + co * rel.y());
  return true;
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
  if (path_pub_->get_subscription_count() == 0) {
    return;
  }
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
  // Building this cloud every 200 ms costs CPU for nothing when nobody is listening
  if (!map_vis_pub_ || map_vis_pub_->get_subscription_count() == 0) {
    return;
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
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

