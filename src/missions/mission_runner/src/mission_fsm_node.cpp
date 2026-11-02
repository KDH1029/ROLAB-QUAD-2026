/**
 * @file mission_fsm_node.cpp
 * @brief C++ 20Hz Competition Mission State Machine for 2026 RoboWorld (450-pt Full Course).
 *
 * Executes the 7 core competition requirements:
 *   1. IN Start Line departure
 *   2. Central X-intersection Ramp / Stair traversal
 *   3. Stop Line 1: >= 2.0s full stop + wait for GREEN traffic light (+50 pts)
 *   4. Rubber Cone Slalom (23cm cones): lateral repulsion & wall clearance (+100 pts)
 *   5. Stop Line 2: >= 2.0s full stop + wait for GREEN traffic light (+50 pts)
 *   6. Parking Bay ('P' / QR): Sport API 1009 (Sit) for >= 5.0s + 1010 (RiseSit) (+50 pts)
 *   7. OUT Finish Line traversal (+200 pts)
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
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

struct StageWaypoint
{
  std::string name;
  double x;
  double y;
  double yaw_deg;
  std::string action;  // "NONE", "TRAFFIC_WAIT", "SLALOM_AVOID", "PARKING_SIT", "FINISH"
};
}  // namespace

class MissionFsmNode : public rclcpp::Node
{
public:
  MissionFsmNode()
  : Node("mission_fsm_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    ref_frame_ = declare_parameter<std::string>("reference_frame", "map");
    stop_hold_sec_ = declare_parameter<double>("stop_line_hold_sec", 2.3);
    traffic_timeout_sec_ = declare_parameter<double>("traffic_timeout_sec", 12.0);
    parking_sit_sec_ = declare_parameter<double>("parking_sit_hold_sec", 5.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.55);
    max_vy_ = declare_parameter<double>("max_vy", 0.28);
    max_wz_ = declare_parameter<double>("max_wz", 1.10);
    xy_tol_m_ = declare_parameter<double>("xy_goal_tolerance_m", 0.16);
    yaw_tol_deg_ = declare_parameter<double>("yaw_goal_tolerance_deg", 7.0);

    // Relative-to-start option so the FSM can also run in pure odom frame starting at (0,0)
    use_relative_demo_ = declare_parameter<bool>("use_relative_route", false);

    // A GREEN must arrive after the stop began and be this fresh to count
    traffic_fresh_sec_ = declare_parameter<double>("traffic_fresh_sec", 1.0);
    cone_fresh_sec_ = declare_parameter<double>("cone_fresh_sec", 0.5);
    // 1009 Sit / 1010 RiseSit (1005 StandDown + 1004 StandUp leaves the joints locked: Move is ignored)
    sit_api_id_ = declare_parameter<int>("parking_sit_api_id", 1009);
    rise_api_id_ = declare_parameter<int>("parking_rise_api_id", 1010);

    initWaypoints();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { raw_odom_ = msg; });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&MissionFsmNode::onScan, this, std::placeholders::_1));
    traffic_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission/traffic_light", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        traffic_state_ = msg->data;
        traffic_stamp_ = std::chrono::steady_clock::now();
      });
    parking_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mission/parking_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { parking_marker_seen_ = msg->data; });
    cone_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/mission/cone_poses", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        cone_poses_ = *msg;
        cone_stamp_ = std::chrono::steady_clock::now();
      });

    RCLCPP_INFO(
      get_logger(),
      "mission_fsm_node ready: %zu stages loaded (stop_hold=%.1fs, sit_hold=%.1fs)",
      stages_.size(), stop_hold_sec_, parking_sit_sec_);
  }

  bool isFinished() const { return mission_complete_; }

  void stopRobot()
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void step()
  {
    if (mission_complete_ || stage_idx_ >= stages_.size()) {
      mission_complete_ = true;
      return;
    }

    double x, y, z, yaw;
    if (!queryPose(x, y, z, yaw)) {
      // No pose in the reference frame: never drive on a guess
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No pose in '%s' frame (is localization running?). Holding still.", ref_frame_.c_str());
      return;
    }

    const auto & wp = stages_[stage_idx_];

    // Handle active special mission actions at the current waypoint
    if (in_special_action_) {
      executeSpecialAction(wp);
      return;
    }

    const double dx = wp.x - x;
    const double dy = wp.y - y;
    const double dist = std::hypot(dx, dy);
    const double bearing = std::atan2(dy, dx);
    const double heading_err = normalizeAngle(bearing - yaw);
    const double final_yaw_err = normalizeAngle(wp.yaw_deg * M_PI / 180.0 - yaw);

    if (dist <= xy_tol_m_) {
      if (std::abs(final_yaw_err * 180.0 / M_PI) <= yaw_tol_deg_ || wp.action == "SLALOM_AVOID") {
        stopRobot();
        if (wp.action != "NONE" && wp.action != "SLALOM_AVOID") {
          in_special_action_ = true;
          action_start_time_ = std::chrono::steady_clock::now();
          parking_sit_sent_ = false;
          RCLCPP_INFO(
            get_logger(), "[STAGE %zu/%zu] Reached '%s' -> starting action [%s]",
            stage_idx_ + 1, stages_.size(), wp.name.c_str(), wp.action.c_str());
          return;
        }
        advanceStage();
        return;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = std::clamp(1.6 * final_yaw_err, -max_wz_, max_wz_);
      if (std::abs(cmd.angular.z) < 0.15) {
        cmd.angular.z = std::copysign(0.15, final_yaw_err);
      }
      cmd_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    if (std::abs(heading_err) > (28.0 * M_PI / 180.0)) {
      cmd.angular.z = std::clamp(1.6 * heading_err, -max_wz_, max_wz_);
      if (std::abs(cmd.angular.z) < 0.15) {
        cmd.angular.z = std::copysign(0.15, heading_err);
      }
    } else {
      cmd.linear.x = std::min(max_vx_, std::max(0.12, 0.85 * dist));
      cmd.angular.z = std::clamp(1.6 * heading_err, -max_wz_, max_wz_);

      // Lateral repulsion from 23cm traffic cones & side walls to prevent -10pt/s contact penalty
      double vy_repulse = computeLateralAvoidance();
      cmd.linear.y = std::clamp(vy_repulse, -max_vy_, max_vy_);
    }

    cmd_pub_->publish(cmd);

    if (++tick_ % 10 == 0) {
      std::cout << "\r  [STAGE " << (stage_idx_ + 1) << "/" << stages_.size()
                << " " << std::left << std::setw(13) << wp.name << "] "
                << std::fixed << std::setprecision(2)
                << "pos=(" << x << ", " << y << ") rem=" << dist << "m "
                << "tl=" << traffic_state_ << "   " << std::flush;
    }
  }

private:
  void initWaypoints()
  {
    if (use_relative_demo_) {
      stages_ = {
        {"start_in",      0.50,  0.00,   0.0, "NONE"},
        {"stop_line_1",   1.20,  0.00,   0.0, "TRAFFIC_WAIT"},
        {"slalom_mid",    1.80,  0.40,  45.0, "SLALOM_AVOID"},
        {"stop_line_2",   2.20,  0.40,   0.0, "TRAFFIC_WAIT"},
        {"parking_slot",  2.60,  0.00,   0.0, "PARKING_SIT"},
        {"out_finish",    3.10,  0.00,   0.0, "FINISH"}
      };
      return;
    }

    // Official 2026 RoboWorld 6.0m (X) x 8.0m (Y) upright track coordinates
    stages_ = {
      {"start_in",      5.00, 0.80,   90.0, "NONE"},
      {"ramp_ascend",   4.40, 2.50,  135.0, "NONE"},
      {"stop_line_1",   3.00, 4.00,  135.0, "TRAFFIC_WAIT"},
      {"slalom_entry",  1.20, 6.80,   90.0, "NONE"},
      {"slalom_mid",    3.00, 7.20,    0.0, "SLALOM_AVOID"},
      {"slalom_exit",   4.80, 6.80, -135.0, "NONE"},
      {"stop_line_2",   3.00, 4.00, -135.0, "TRAFFIC_WAIT"},
      {"stair_exit",    1.20, 1.80,  -90.0, "NONE"},
      {"bottom_turn",   1.80, 1.50,    0.0, "NONE"},
      {"parking_slot",  3.00, 0.70,  -90.0, "PARKING_SIT"},
      {"out_align",     1.20, 1.20,  -90.0, "NONE"},
      {"out_finish",    1.00, 0.40,  -90.0, "FINISH"}
    };
  }

  void executeSpecialAction(const StageWaypoint & wp)
  {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - action_start_time_).count();

    if (wp.action == "TRAFFIC_WAIT") {
      // Rulebook 5.2: Must remain stopped >= 2.0s inside stop line, then depart on GREEN
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);

      if (elapsed < stop_hold_sec_) {
        return;
      }
      // Only a GREEN received during this stop counts; a GREEN from the previous stop line must not.
      const auto now_tp = std::chrono::steady_clock::now();
      const bool fresh = traffic_stamp_ > action_start_time_ &&
        std::chrono::duration<double>(now_tp - traffic_stamp_).count() <= traffic_fresh_sec_;
      const std::string signal = fresh ? traffic_state_ : std::string("STALE");
      const bool timed_out = elapsed >= traffic_timeout_sec_;
      if (signal == "GREEN" || (timed_out && signal != "RED")) {
        RCLCPP_INFO(
          get_logger(), "[TRAFFIC] Hold=%.1fs complete (signal=%s%s). Proceeding.",
          elapsed, signal.c_str(), timed_out ? ", timeout" : "");
        in_special_action_ = false;
        advanceStage();
      } else if (timed_out) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "[TRAFFIC] Timeout but signal is RED. Still waiting.");
      }
      return;
    }

    if (wp.action == "PARKING_SIT") {
      // Rulebook 5.2: Full sit >= 5.0s inside parking zone (+50 pts)
      if (!parking_sit_sent_) {
        stopRobot();
        sendSportCmd(sit_api_id_);
        parking_sit_sent_ = true;
        parking_rise_sent_ = false;
        RCLCPP_INFO(get_logger(), "[PARKING] Sent Sit(%d). Holding sit for %.1fs...", sit_api_id_, parking_sit_sec_);
        return;
      }
      if (elapsed >= parking_sit_sec_ && elapsed < parking_sit_sec_ + 1.5) {
        if (!parking_rise_sent_) {
          sendSportCmd(rise_api_id_);
          parking_rise_sent_ = true;
        }
        return;
      }
      if (elapsed >= parking_sit_sec_ + 1.5) {
        RCLCPP_INFO(get_logger(), "[PARKING] Sit %.1fs & RiseSit complete. Exiting bay.", parking_sit_sec_);
        in_special_action_ = false;
        advanceStage();
      }
      return;
    }

    if (wp.action == "FINISH") {
      stopRobot();
      RCLCPP_INFO(get_logger(), "[FINISH] Out Line crossed! All missions completed (450/450 pts target).");
      in_special_action_ = false;
      mission_complete_ = true;
    }
  }

  void advanceStage()
  {
    stage_idx_++;
    if (stage_idx_ < stages_.size()) {
      RCLCPP_INFO(
        get_logger(), ">>> Advancing to stage %zu/%zu: %s (%.2f, %.2f)",
        stage_idx_ + 1, stages_.size(),
        stages_[stage_idx_].name.c_str(),
        stages_[stage_idx_].x, stages_[stage_idx_].y);
    } else {
      mission_complete_ = true;
    }
  }

  double computeLateralAvoidance()
  {
    double vy = 0.0;
    // Repulsion from detected 23cm cones within 1.1m ahead (ignore detections the detector stopped refreshing)
    const bool cones_fresh = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - cone_stamp_).count() <= cone_fresh_sec_;
    for (const auto & p : cones_fresh ? cone_poses_.poses : std::vector<geometry_msgs::msg::Pose>{}) {
      if (p.position.x > 0.15 && p.position.x < 1.10 && std::abs(p.position.y) < 0.45) {
        const double sign = (p.position.y >= 0.0) ? -1.0 : 1.0;
        const double mag = 0.18 * (1.10 - p.position.x);
        vy += sign * mag;
      }
    }
    // Wall safety margin repulsion from left/right LaserScan sectors
    if (left_wall_dist_ < 0.38) {
      vy -= 0.20 * (0.38 - left_wall_dist_);
    }
    if (right_wall_dist_ < 0.38) {
      vy += 0.20 * (0.38 - right_wall_dist_);
    }
    return vy;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_l = 99.9, min_r = 99.9;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      const double ang_deg = (msg->angle_min + static_cast<double>(i) * msg->angle_increment) * 180.0 / M_PI;
      if (ang_deg >= 35.0 && ang_deg <= 105.0 && r < min_l) min_l = r;
      if (ang_deg >= -105.0 && ang_deg <= -35.0 && r < min_r) min_r = r;
    }
    left_wall_dist_ = min_l;
    right_wall_dist_ = min_r;
  }

  bool queryPose(double & x, double & y, double & z, double & yaw)
  {
    // Waypoints are in ref_frame_. Falling back to odom would drive map coordinates from wherever
    // the robot was switched on, so only odom-referenced routes may use the raw topic below.
    try {
      auto tr = tf_buffer_.lookupTransform(ref_frame_, "base_link", tf2::TimePointZero);
      x = tr.transform.translation.x;
      y = tr.transform.translation.y;
      z = tr.transform.translation.z;
      tf2::Quaternion q(
        tr.transform.rotation.x,
        tr.transform.rotation.y,
        tr.transform.rotation.z,
        tr.transform.rotation.w);
      double r, p;
      tf2::Matrix3x3(q).getRPY(r, p, yaw);
      return true;
    } catch (const tf2::TransformException &) {
    }
    if (ref_frame_ == "odom" && raw_odom_) {
      const auto & p = raw_odom_->pose.pose;
      x = p.position.x;
      y = p.position.y;
      z = p.position.z;
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      double r, pt;
      tf2::Matrix3x3(q).getRPY(r, pt, yaw);
      return true;
    }
    return false;
  }

  void sendSportCmd(int32_t api_id)
  {
    unitree_api::msg::Request req;
    req.header.identity.id = this->now().nanoseconds();
    req.header.identity.api_id = api_id;
    req.parameter = "{}";
    sport_pub_->publish(req);
  }

  std::string ref_frame_;
  double stop_hold_sec_ = 2.3;
  double traffic_timeout_sec_ = 12.0;
  double parking_sit_sec_ = 5.5;
  double max_vx_ = 0.55;
  double max_vy_ = 0.28;
  double max_wz_ = 1.10;
  double xy_tol_m_ = 0.16;
  double yaw_tol_deg_ = 7.0;
  bool use_relative_demo_ = false;
  double traffic_fresh_sec_ = 1.0;
  double cone_fresh_sec_ = 0.5;
  int sit_api_id_ = 1009;
  int rise_api_id_ = 1010;

  std::vector<StageWaypoint> stages_;
  size_t stage_idx_ = 0;
  bool in_special_action_ = false;
  bool parking_sit_sent_ = false;
  bool parking_rise_sent_ = false;
  bool mission_complete_ = false;
  size_t tick_ = 0;
  std::chrono::steady_clock::time_point action_start_time_;

  std::string traffic_state_ = "UNKNOWN";
  std::chrono::steady_clock::time_point traffic_stamp_{};
  std::chrono::steady_clock::time_point cone_stamp_{};
  bool parking_marker_seen_ = false;
  double left_wall_dist_ = 99.9;
  double right_wall_dist_ = 99.9;
  geometry_msgs::msg::PoseArray cone_poses_;
  nav_msgs::msg::Odometry::SharedPtr raw_odom_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);

  auto node = std::make_shared<MissionFsmNode>();
  rclcpp::WallRate rate(20.0);

  while (rclcpp::ok() && !g_stop_requested && !node->isFinished()) {
    rclcpp::spin_some(node);
    node->step();
    rate.sleep();
  }

  node->stopRobot();
  rclcpp::shutdown();
  return 0;
}
