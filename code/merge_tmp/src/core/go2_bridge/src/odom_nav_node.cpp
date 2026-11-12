/**
 * @file odom_nav_node.cpp
 * @brief Lightweight C++ 20Hz closed-loop odometry waypoint controller for Unitree Go2.
 *
 * Modes:
 *   - "goto"   : Drives to (target_x, target_y, target_yaw_deg) and exits on completion.
 *   - "status" : Prints current 3D odometry pose, base_stab attitude, and front clearance.
 *   - "daemon" : Listens continuously to /odom_goal (geometry_msgs::msg::PoseStamped).
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unitree_api/msg/request.hpp>

namespace
{
volatile std::sig_atomic_t g_stop_requested = 0;
void sigintHandler(int) { g_stop_requested = 1; }

double normalizeAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}
}  // namespace

class Go2OdomNavNode : public rclcpp::Node
{
public:
  Go2OdomNavNode()
  : Node("go2_odom_nav_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    mode_ = declare_parameter<std::string>("mode", "goto");
    ref_frame_ = declare_parameter<std::string>("reference_frame", "odom");
    target_x_ = declare_parameter<double>("target_x", 0.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_yaw_deg_ = declare_parameter<double>("target_yaw_deg", 0.0);
    sit_on_arrival_ = declare_parameter<bool>("sit_on_arrival", false);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 90.0);

    max_vx_ = declare_parameter<double>("max_vx", 0.65);
    max_wz_ = declare_parameter<double>("max_wz", 1.20);
    min_vx_ = declare_parameter<double>("min_vx", 0.08);
    min_wz_ = declare_parameter<double>("min_wz", 0.15);
    xy_tol_m_ = declare_parameter<double>("xy_goal_tolerance_m", 0.18);
    yaw_tol_deg_ = declare_parameter<double>("yaw_goal_tolerance_deg", 6.0);
    align_thresh_deg_ = declare_parameter<double>("heading_align_thresh_deg", 25.0);
    obstacle_stop_m_ = declare_parameter<double>("obstacle_stop_dist_m", 0.35);
    obstacle_fov_deg_ = declare_parameter<double>("obstacle_check_fov_deg", 20.0);
    kp_lin_ = declare_parameter<double>("kp_linear", 0.85);
    kp_ang_ = declare_parameter<double>("kp_angular", 1.60);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { raw_odom_ = msg; });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&Go2OdomNavNode::onScan, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/odom_goal", 10,
      std::bind(&Go2OdomNavNode::onGoalMsg, this, std::placeholders::_1));

    if (mode_ == "goto") {
      has_active_goal_ = true;
      goal_start_time_ = std::chrono::steady_clock::now();
    }
  }

  std::string mode() const { return mode_; }
  bool isFinished() const { return finished_; }

  bool queryPose(std::string & out_frame, double & x, double & y, double & z,
                 double & roll, double & pitch, double & yaw)
  {
    const std::vector<std::string> frames = {ref_frame_, "odom", "map"};
    for (const auto & f : frames) {
      try {
        auto tr = tf_buffer_.lookupTransform(f, "base_link", tf2::TimePointZero);
        x = tr.transform.translation.x;
        y = tr.transform.translation.y;
        z = tr.transform.translation.z;
        tf2::Quaternion q(
          tr.transform.rotation.x,
          tr.transform.rotation.y,
          tr.transform.rotation.z,
          tr.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        out_frame = f;
        return true;
      } catch (const tf2::TransformException &) {
        continue;
      }
    }

    if (raw_odom_) {
      const auto & p = raw_odom_->pose.pose;
      x = p.position.x;
      y = p.position.y;
      z = p.position.z;
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      out_frame = "odom(topic)";
      return true;
    }
    return false;
  }

  void queryStabTiltDeg(double & sroll_deg, double & spitch_deg)
  {
    try {
      auto tr = tf_buffer_.lookupTransform("base_link", "base_stab", tf2::TimePointZero);
      tf2::Quaternion q(
        tr.transform.rotation.x,
        tr.transform.rotation.y,
        tr.transform.rotation.z,
        tr.transform.rotation.w);
      double r, p, y;
      tf2::Matrix3x3(q).getRPY(r, p, y);
      sroll_deg = -r * 180.0 / M_PI;
      spitch_deg = -p * 180.0 / M_PI;
    } catch (const tf2::TransformException &) {
      sroll_deg = 0.0;
      spitch_deg = 0.0;
    }
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(25));
    }
  }

  void printStatus()
  {
    std::string frame;
    double x, y, z, r, p, yaw;
    if (!queryPose(frame, x, y, z, r, p, yaw)) {
      std::cout << "[ERROR] Odometry unavailable.\n";
      return;
    }
    double sr, sp;
    queryStabTiltDeg(sr, sp);
    std::cout << std::fixed << std::setprecision(3)
              << "frame       : " << frame << " -> base_link\n"
              << "position    : x=" << std::showpos << x << "m, y=" << y << "m, z=" << z << "m\n"
              << std::setprecision(2)
              << "yaw         : " << (yaw * 180.0 / M_PI) << " deg\n"
              << "stab_rp     : roll=" << sr << " deg, pitch=" << sp << " deg\n"
              << std::noshowpos
              << "front_clear : " << front_clear_dist_ << "m\n";
  }

  void stepController()
  {
    if (!has_active_goal_) return;

    std::string frame;
    double x, y, z, r, p, yaw;
    if (!queryPose(frame, x, y, z, r, p, yaw)) {
      return;
    }

    if (!printed_header_) {
      std::cout << std::fixed << std::setprecision(3)
                << "[INFO] Starting C++ odometry navigation (frame=" << frame << ")\n"
                << "  start : x=" << std::showpos << x << "m, y=" << y << "m, z=" << z
                << "m, yaw=" << std::setprecision(1) << (yaw * 180.0 / M_PI) << " deg\n"
                << std::setprecision(3)
                << "  target: x=" << target_x_ << "m, y=" << target_y_
                << "m, yaw=" << std::setprecision(1) << target_yaw_deg_ << " deg\n"
                << std::noshowpos;
      printed_header_ = true;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - goal_start_time_).count();
    if (elapsed > timeout_sec_) {
      std::cout << "\n[WARN] Navigation timeout (" << timeout_sec_ << "s). Stopping.\n";
      stopRobot();
      has_active_goal_ = false;
      if (mode_ == "goto") finished_ = true;
      return;
    }

    const double dx = target_x_ - x;
    const double dy = target_y_ - y;
    const double dist_2d = std::hypot(dx, dy);
    const double bearing = std::atan2(dy, dx);
    const double heading_err = normalizeAngle(bearing - yaw);
    const double target_yaw_rad = target_yaw_deg_ * M_PI / 180.0;
    const double final_yaw_err = normalizeAngle(target_yaw_rad - yaw);

    double sr, sp;
    queryStabTiltDeg(sr, sp);

    geometry_msgs::msg::Twist cmd;
    std::string phase = "ALIGN";

    if (dist_2d <= xy_tol_m_) {
      phase = "YAW_LOCK";
      if (std::abs(final_yaw_err * 180.0 / M_PI) <= yaw_tol_deg_) {
        stopRobot();
        std::cout << std::fixed << std::setprecision(1)
                  << "\n[DONE] Target reached: err_xy=" << (dist_2d * 100.0)
                  << "cm, z=" << std::setprecision(2) << std::showpos << z
                  << "m, err_yaw=" << std::setprecision(1) << (final_yaw_err * 180.0 / M_PI)
                  << std::noshowpos << " deg (" << elapsed << "s)\n";
        if (sit_on_arrival_) {
          sendSportCmd(1005);
          rclcpp::sleep_for(std::chrono::milliseconds(400));
        }
        has_active_goal_ = false;
        if (mode_ == "goto") finished_ = true;
        return;
      }
      double wz = std::clamp(kp_ang_ * final_yaw_err, -max_wz_, max_wz_);
      if (std::abs(wz) < min_wz_) {
        wz = std::copysign(min_wz_, final_yaw_err);
      }
      cmd.angular.z = wz;
    } else {
      if (front_clear_dist_ < obstacle_stop_m_ && std::abs(heading_err) < (35.0 * M_PI / 180.0)) {
        stopRobot();
        std::cout << "\r  [HOLD] Front obstacle at " << std::fixed << std::setprecision(2)
                  << front_clear_dist_ << "m   " << std::flush;
        return;
      }

      if (std::abs(heading_err) > (align_thresh_deg_ * M_PI / 180.0)) {
        phase = "ALIGN";
        double wz = std::clamp(kp_ang_ * heading_err, -max_wz_, max_wz_);
        if (std::abs(wz) < min_wz_) {
          wz = std::copysign(min_wz_, heading_err);
        }
        cmd.angular.z = wz;
      } else {
        phase = "CRUISE";
        const double hill_boost = (std::abs(sp) > 3.0) ? 0.04 : 0.0;
        const double vx = std::min(max_vx_, std::max(min_vx_ + 0.04 + hill_boost, kp_lin_ * dist_2d));
        const double wz = std::clamp(kp_ang_ * heading_err, -max_wz_, max_wz_);
        cmd.linear.x = vx;
        cmd.angular.z = wz;
      }
    }

    cmd_pub_->publish(cmd);

    if (++step_count_ % 6 == 0) {
      std::cout << "\r  [" << std::left << std::setw(8) << phase << "] pos=("
                << std::fixed << std::setprecision(2) << std::showpos
                << x << ", " << y << ", z=" << z << "m, "
                << std::setprecision(0) << (yaw * 180.0 / M_PI) << " deg) | "
                << std::noshowpos << std::setprecision(2)
                << "rem=" << dist_2d << "m | stab_rp=("
                << std::showpos << std::setprecision(1) << sr << ", " << sp
                << std::noshowpos << ") deg" << std::flush;
    }
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) return;
    const double fov_rad = obstacle_fov_deg_ * M_PI / 180.0;
    double min_d = 99.9;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (std::abs(ang) <= fov_rad && r < min_d) {
        min_d = r;
      }
    }
    front_clear_dist_ = min_d;
  }

  void onGoalMsg(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    target_x_ = msg->pose.position.x;
    target_y_ = msg->pose.position.y;
    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    target_yaw_deg_ = y * 180.0 / M_PI;
    if (!msg->header.frame_id.empty()) {
      ref_frame_ = msg->header.frame_id;
    }
    printed_header_ = false;
    has_active_goal_ = true;
    goal_start_time_ = std::chrono::steady_clock::now();
  }

  void sendSportCmd(int32_t api_id)
  {
    unitree_api::msg::Request req;
    req.header.identity.id = this->now().nanoseconds();
    req.header.identity.api_id = api_id;
    req.parameter = "{}";
    sport_pub_->publish(req);
  }

  std::string mode_;
  std::string ref_frame_;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_yaw_deg_ = 0.0;
  bool sit_on_arrival_ = false;
  double timeout_sec_ = 90.0;

  double max_vx_ = 0.65;
  double max_wz_ = 1.20;
  double min_vx_ = 0.08;
  double min_wz_ = 0.15;
  double xy_tol_m_ = 0.18;
  double yaw_tol_deg_ = 6.0;
  double align_thresh_deg_ = 25.0;
  double obstacle_stop_m_ = 0.35;
  double obstacle_fov_deg_ = 20.0;
  double kp_lin_ = 0.85;
  double kp_ang_ = 1.60;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  nav_msgs::msg::Odometry::SharedPtr raw_odom_;
  double front_clear_dist_ = 99.9;
  bool has_active_goal_ = false;
  bool printed_header_ = false;
  bool finished_ = false;
  size_t step_count_ = 0;
  std::chrono::steady_clock::time_point goal_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);

  auto node = std::make_shared<Go2OdomNavNode>();
  rclcpp::WallRate rate(20.0);

  if (node->mode() == "status") {
    for (int i = 0; i < 15 && rclcpp::ok(); ++i) {
      rclcpp::spin_some(node);
      rate.sleep();
    }
    node->printStatus();
    rclcpp::shutdown();
    return 0;
  }

  while (rclcpp::ok() && !g_stop_requested && !node->isFinished()) {
    rclcpp::spin_some(node);
    node->stepController();
    rate.sleep();
  }

  if (g_stop_requested) {
    std::cout << "\n[INFO] Interrupted. Stopping robot.\n";
    node->stopRobot();
  }

  rclcpp::shutdown();
  return 0;
}
