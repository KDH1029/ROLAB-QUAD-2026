#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <memory>
#include <mutex>

enum class PatrolState {
  IDLE,
  ALIGN_INIT,
  START_SIT,
  START_STAND,
  DRIVE,
  CORNER_ALIGN,
  CORNER_SIT,
  CORNER_STAND,
  TRAFFIC_CHECK,
  CORNER_TURN,
  POST_TURN_DWELL
};

inline const char* stateToString(PatrolState s) {
  switch (s) {
    case PatrolState::IDLE: return "IDLE";
    case PatrolState::ALIGN_INIT: return "ALIGN_INIT";
    case PatrolState::START_SIT: return "START_SIT";
    case PatrolState::START_STAND: return "START_STAND";
    case PatrolState::DRIVE: return "DRIVING";
    case PatrolState::CORNER_ALIGN: return "CORNER_ALIGN";
    case PatrolState::CORNER_SIT: return "CORNER_SIT";
    case PatrolState::CORNER_STAND: return "CORNER_STAND";
    case PatrolState::TRAFFIC_CHECK: return "TRAFFIC_CHECK";
    case PatrolState::CORNER_TURN: return "CORNER_TURN";
    case PatrolState::POST_TURN_DWELL: return "POST_TURN_DWELL";
    default: return "UNKNOWN";
  }
}

struct Point2D {
  float x;
  float y;
};

struct Line2D {
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double distance{0.0};   // Perpendicular distance from origin (0,0)
  double angle{0.0};      // Line normal orientation
  int inlier_count{0};
  bool valid{false};
};

struct PathConfig {
  std::string name;
  double drive_yaw_deg{0.0};
  double drive_yaw_rad{0.0};
  double target_left_dist{0.50};
  double front_trigger_dist{0.85};
  bool has_traffic_light{true};
};

struct CornerConfig {
  std::string name;
  double target_front_dist{0.65};
  double target_left_dist{0.50};
  double target_yaw_deg{0.0};
  double target_yaw_rad{0.0};
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Lifecycle: configure = publishers, activate = inputs + 20 Hz loop (state from auto_start),
// deactivate = zero velocity and stop listening.
class WallRunnerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  WallRunnerNode() : rclcpp_lifecycle::LifecycleNode("wall_runner_node") {
    // 1. Declare and Load Parameters
    declareAndLoadParameters();

    // 2. Setup Dynamic Parameter Update Callback
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WallRunnerNode::parametersCallback, this, std::placeholders::_1)
    );

    // Input for the wall fits:
    //   "scan"  : /scan from cloud_bridge scan_mode:=vertical (gravity-levelled, vertical surfaces
    //             only). Ramps / stair treads never become walls; walls stay visible when the body
    //             pitches on a slope.
    //   "cloud" : original ring slice (rings ring_min..ring_max of /velodyne_points). The slice tilts
    //             with the body: on a 10 deg pitch it hits the floor ~2.4 m ahead or passes over
    //             0.6 m walls beyond ~0.9 m.
    input_source_ = this->declare_parameter<std::string>("input_source", "scan");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol/status", 10);
    posture_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol/posture", 10);
    robot_motion_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/motion_cmd", 10);
    marker_left_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/patrol/viz/left_wall", 10);
    marker_front_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/patrol/viz/front_wall", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    cmd_vel_pub_->on_activate();
    status_pub_->on_activate();
    posture_pub_->on_activate();
    robot_motion_pub_->on_activate();
    marker_left_pub_->on_activate();
    marker_front_pub_->on_activate();

    if (input_source_ == "scan") {
      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallRunnerNode::scanCallback, this, std::placeholders::_1));
    } else {
      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", rclcpp::SensorDataQoS(),
        std::bind(&WallRunnerNode::cloudCallback, this, std::placeholders::_1));
    }

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/body",
      rclcpp::SensorDataQoS(),
      std::bind(&WallRunnerNode::imuCallback, this, std::placeholders::_1)
    );
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/patrol/cmd",
      10,
      std::bind(&WallRunnerNode::cmdCallback, this, std::placeholders::_1)
    );

    traffic_sub_str_ = this->create_subscription<std_msgs::msg::String>(
      "/traffic_light",
      10,
      std::bind(&WallRunnerNode::trafficLightCallback, this, std::placeholders::_1)
    );

    traffic_sub_bool_ = this->create_subscription<std_msgs::msg::Bool>(
      "/traffic_light_state",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        external_traffic_override_ = true;
        external_traffic_green_ = msg->data;
      }
    );

    // 5. Timers
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20Hz
      std::bind(&WallRunnerNode::controlLoop, this)
    );

    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 10Hz
      std::bind(&WallRunnerNode::publishStatus, this)
    );

    start_time_ = this->now();
    state_ = auto_start_ ? PatrolState::ALIGN_INIT : PatrolState::IDLE;

    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), " Go2 Wall Patrol Active (Dynamic Tuning & RViz Enabled)");
    RCLCPP_INFO(this->get_logger(), " Sequence: a -> 1 -> b -> 2 -> c -> 3 -> d -> 4 -> a");
    RCLCPP_INFO(this->get_logger(), " Input: %s | LiDAR Z: [%.2f, %.2f]m | Slowdown: %s",
      input_source_ == "scan" ? scan_topic_.c_str() : "/velodyne_points (ring slice)",
      z_min_, z_max_, enable_slowdown_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), " Obstacle Rejection on Paths 2 & 4: %s", prefer_outer_wall_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    control_timer_.reset();
    status_timer_.reset();
    publishZeroVelocity();
    cloud_sub_.reset();
    scan_sub_.reset();
    imu_sub_.reset();
    cmd_sub_.reset();
    traffic_sub_str_.reset();
    traffic_sub_bool_.reset();
    state_ = PatrolState::IDLE;
    cmd_vel_pub_->on_deactivate();
    status_pub_->on_deactivate();
    posture_pub_->on_deactivate();
    robot_motion_pub_->on_deactivate();
    marker_left_pub_->on_deactivate();
    marker_front_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    cmd_vel_pub_.reset();
    status_pub_.reset();
    posture_pub_.reset();
    robot_motion_pub_.reset();
    marker_left_pub_.reset();
    marker_front_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    control_timer_.reset();
    status_timer_.reset();
    publishZeroVelocity();
    return CallbackReturn::SUCCESS;
  }

  ~WallRunnerNode() override {
    publishZeroVelocity();
  }

private:
  void declareAndLoadParameters() {
    this->declare_parameter<double>("cruise_speed", 0.45);
    this->declare_parameter<bool>("enable_slowdown", false);
    this->declare_parameter<double>("slowdown_dist", 1.50);
    this->declare_parameter<double>("turn_speed", 0.50);
    this->declare_parameter<double>("min_turn_speed", 0.15);
    this->declare_parameter<double>("max_vy", 0.25);
    this->declare_parameter<double>("max_align_vx", 0.20);
    this->declare_parameter<double>("turn_tolerance_deg", 2.5);
    this->declare_parameter<double>("post_turn_dwell_time", 0.5);

    this->declare_parameter<bool>("enable_start_sit", false);
    this->declare_parameter<double>("start_delay_sec", 0.0);
    this->declare_parameter<double>("sit_duration", 3.0);
    this->declare_parameter<double>("stand_duration", 2.0);
    this->declare_parameter<double>("traffic_cycle_sec", 30.0);
    this->declare_parameter<double>("traffic_safety_margin_sec", 5.0);

    this->declare_parameter<double>("lidar_yaw_mount_deg", 90.0);
    this->declare_parameter<double>("lidar_x_offset", 0.25);
    this->declare_parameter<double>("lidar_y_offset", 0.00);
    this->declare_parameter<double>("lidar_z_offset", 0.1588);
    this->declare_parameter<double>("z_min", -0.15);
    this->declare_parameter<double>("z_max", 0.60);
    this->declare_parameter<bool>("enable_ring_filter", true);
    this->declare_parameter<int>("ring_min", 7);
    this->declare_parameter<int>("ring_max", 8);
    this->declare_parameter<double>("filter_alpha", 0.40);
    this->declare_parameter<int>("ransac_iterations", 80);
    this->declare_parameter<double>("ransac_inlier_thresh", 0.035);
    this->declare_parameter<int>("ransac_min_inliers", 20);
    this->declare_parameter<bool>("prefer_outer_wall", true);

    // Paths (1, 2, 3, 4)
    this->declare_parameter<double>("path_1.drive_yaw_deg", 0.0);
    this->declare_parameter<double>("path_1.target_left_dist", 0.50);
    this->declare_parameter<double>("path_1.front_trigger_dist", 0.85);
    this->declare_parameter<bool>("path_1.has_traffic_light", true);

    this->declare_parameter<double>("path_2.drive_yaw_deg", -90.0);
    this->declare_parameter<double>("path_2.target_left_dist", 0.50);
    this->declare_parameter<double>("path_2.front_trigger_dist", 0.85);
    this->declare_parameter<bool>("path_2.has_traffic_light", true);

    this->declare_parameter<double>("path_3.drive_yaw_deg", 180.0);
    this->declare_parameter<double>("path_3.target_left_dist", 0.50);
    this->declare_parameter<double>("path_3.front_trigger_dist", 0.85);
    this->declare_parameter<bool>("path_3.has_traffic_light", false);

    this->declare_parameter<double>("path_4.drive_yaw_deg", 90.0);
    this->declare_parameter<double>("path_4.target_left_dist", 0.50);
    this->declare_parameter<double>("path_4.front_trigger_dist", 0.85);
    this->declare_parameter<bool>("path_4.has_traffic_light", true);

    // Corners (a, b, c, d)
    this->declare_parameter<double>("corner_a.target_front_dist", 0.65);
    this->declare_parameter<double>("corner_a.target_left_dist", 0.50);
    this->declare_parameter<double>("corner_a.target_yaw_deg", 0.0);

    this->declare_parameter<double>("corner_b.target_front_dist", 0.65);
    this->declare_parameter<double>("corner_b.target_left_dist", 0.50);
    this->declare_parameter<double>("corner_b.target_yaw_deg", 0.0);

    this->declare_parameter<double>("corner_c.target_front_dist", 0.65);
    this->declare_parameter<double>("corner_c.target_left_dist", 0.50);
    this->declare_parameter<double>("corner_c.target_yaw_deg", -90.0);

    this->declare_parameter<double>("corner_d.target_front_dist", 0.65);
    this->declare_parameter<double>("corner_d.target_left_dist", 0.50);
    this->declare_parameter<double>("corner_d.target_yaw_deg", 180.0);

    this->declare_parameter<double>("align_tol_x", 0.04);
    this->declare_parameter<double>("align_tol_y", 0.04);
    this->declare_parameter<double>("align_tol_yaw_deg", 2.0);
    this->declare_parameter<double>("align_settle_time", 0.40);

    this->declare_parameter<double>("kp_x", 0.80);
    this->declare_parameter<double>("kp_dist", 0.80);
    this->declare_parameter<double>("kp_yaw", 1.50);
    this->declare_parameter<double>("kp_turn", 1.50);
    this->declare_parameter<bool>("auto_start", false);
    this->declare_parameter<double>("sensor_timeout", 0.60);

    loadAllParameters();
  }

  void loadAllParameters() {
    this->get_parameter("cruise_speed", cruise_speed_);
    this->get_parameter("enable_slowdown", enable_slowdown_);
    this->get_parameter("slowdown_dist", slowdown_dist_);
    this->get_parameter("turn_speed", turn_speed_);
    this->get_parameter("min_turn_speed", min_turn_speed_);
    this->get_parameter("max_vy", max_vy_);
    this->get_parameter("max_align_vx", max_align_vx_);
    this->get_parameter("turn_tolerance_deg", turn_tolerance_deg_);
    this->get_parameter("post_turn_dwell_time", post_turn_dwell_time_);

    this->get_parameter("enable_start_sit", enable_start_sit_);
    this->get_parameter("start_delay_sec", start_delay_sec_);
    this->get_parameter("sit_duration", sit_duration_);
    this->get_parameter("stand_duration", stand_duration_);
    this->get_parameter("traffic_cycle_sec", traffic_cycle_sec_);
    this->get_parameter("traffic_safety_margin_sec", traffic_safety_margin_sec_);

    this->get_parameter("lidar_yaw_mount_deg", lidar_yaw_mount_deg_);
    lidar_yaw_mount_rad_ = lidar_yaw_mount_deg_ * M_PI / 180.0;
    this->get_parameter("lidar_x_offset", lidar_x_offset_);
    this->get_parameter("lidar_y_offset", lidar_y_offset_);
    this->get_parameter("lidar_z_offset", lidar_z_offset_);
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("enable_ring_filter", enable_ring_filter_);
    this->get_parameter("ring_min", ring_min_);
    this->get_parameter("ring_max", ring_max_);
    this->get_parameter("filter_alpha", filter_alpha_);
    this->get_parameter("ransac_iterations", ransac_iterations_);
    this->get_parameter("ransac_inlier_thresh", ransac_inlier_thresh_);
    this->get_parameter("ransac_min_inliers", ransac_min_inliers_);
    this->get_parameter("prefer_outer_wall", prefer_outer_wall_);

    this->get_parameter("align_tol_x", align_tol_x_);
    this->get_parameter("align_tol_y", align_tol_y_);
    this->get_parameter("align_tol_yaw_deg", align_tol_yaw_deg_);
    this->get_parameter("align_settle_time", align_settle_time_);

    this->get_parameter("kp_x", kp_x_);
    this->get_parameter("kp_dist", kp_dist_);
    this->get_parameter("kp_yaw", kp_yaw_);
    this->get_parameter("kp_turn", kp_turn_);
    this->get_parameter("auto_start", auto_start_);
    this->get_parameter("sensor_timeout", sensor_timeout_);

    turn_tolerance_rad_ = turn_tolerance_deg_ * M_PI / 180.0;
    align_tol_yaw_rad_ = align_tol_yaw_deg_ * M_PI / 180.0;

    paths_.resize(4);
    for (int i = 0; i < 4; ++i) {
      const std::string prefix = "path_" + std::to_string(i + 1);
      paths_[i].name = prefix;
      this->get_parameter(prefix + ".drive_yaw_deg", paths_[i].drive_yaw_deg);
      this->get_parameter(prefix + ".target_left_dist", paths_[i].target_left_dist);
      this->get_parameter(prefix + ".front_trigger_dist", paths_[i].front_trigger_dist);
      this->get_parameter(prefix + ".has_traffic_light", paths_[i].has_traffic_light);
      paths_[i].drive_yaw_rad = normalizeAngle(paths_[i].drive_yaw_deg * M_PI / 180.0);
    }

    corners_.resize(4);
    const std::vector<std::string> c_names = {"corner_a", "corner_b", "corner_c", "corner_d"};
    for (int i = 0; i < 4; ++i) {
      corners_[i].name = c_names[i];
      this->get_parameter(c_names[i] + ".target_front_dist", corners_[i].target_front_dist);
      this->get_parameter(c_names[i] + ".target_left_dist", corners_[i].target_left_dist);
      this->get_parameter(c_names[i] + ".target_yaw_deg", corners_[i].target_yaw_deg);
      corners_[i].target_yaw_rad = normalizeAngle(corners_[i].target_yaw_deg * M_PI / 180.0);
    }
  }

  // Dynamic Parameter Update Callback (Tuning without restart)
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& param : params) {
      const std::string& name = param.get_name();
      if (name == "cruise_speed") cruise_speed_ = param.as_double();
      else if (name == "turn_speed") turn_speed_ = param.as_double();
      else if (name == "min_turn_speed") min_turn_speed_ = param.as_double();
      else if (name == "kp_dist") kp_dist_ = param.as_double();
      else if (name == "kp_yaw") kp_yaw_ = param.as_double();
      else if (name == "kp_turn") kp_turn_ = param.as_double();
      else if (name == "turn_tolerance_deg") {
        turn_tolerance_deg_ = param.as_double();
        turn_tolerance_rad_ = turn_tolerance_deg_ * M_PI / 180.0;
      } else if (name == "post_turn_dwell_time") post_turn_dwell_time_ = param.as_double();
      else if (name == "align_tol_yaw_deg") {
        align_tol_yaw_deg_ = param.as_double();
        align_tol_yaw_rad_ = align_tol_yaw_deg_ * M_PI / 180.0;
      } else if (name == "align_settle_time") align_settle_time_ = param.as_double();
      else if (name == "align_tol_x") align_tol_x_ = param.as_double();
      else if (name == "align_tol_y") align_tol_y_ = param.as_double();
      else if (name == "corner_a_front_dist") corners_[0].target_front_dist = param.as_double();
      else if (name == "corner_a_left_dist") corners_[0].target_left_dist = param.as_double();
      else if (name == "corner_a_yaw") {
        corners_[0].target_yaw_deg = param.as_double();
        corners_[0].target_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "corner_b_front_dist") corners_[1].target_front_dist = param.as_double();
      else if (name == "corner_b_left_dist") corners_[1].target_left_dist = param.as_double();
      else if (name == "corner_b_yaw") {
        corners_[1].target_yaw_deg = param.as_double();
        corners_[1].target_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "corner_c_front_dist") corners_[2].target_front_dist = param.as_double();
      else if (name == "corner_c_left_dist") corners_[2].target_left_dist = param.as_double();
      else if (name == "corner_c_yaw") {
        corners_[2].target_yaw_deg = param.as_double();
        corners_[2].target_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "corner_d_front_dist") corners_[3].target_front_dist = param.as_double();
      else if (name == "corner_d_left_dist") corners_[3].target_left_dist = param.as_double();
      else if (name == "corner_d_yaw") {
        corners_[3].target_yaw_deg = param.as_double();
        corners_[3].target_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "slowdown_dist") slowdown_dist_ = param.as_double();
      else if (name == "path_1_left_dist" || name == "path_1.target_left_dist") paths_[0].target_left_dist = param.as_double();
      else if (name == "path_1_front_dist" || name == "path_1.front_trigger_dist") paths_[0].front_trigger_dist = param.as_double();
      else if (name == "path_2_left_dist" || name == "path_2.target_left_dist") paths_[1].target_left_dist = param.as_double();
      else if (name == "path_2_front_dist" || name == "path_2.front_trigger_dist") paths_[1].front_trigger_dist = param.as_double();
      else if (name == "path_3_left_dist" || name == "path_3.target_left_dist") paths_[2].target_left_dist = param.as_double();
      else if (name == "path_3_front_dist" || name == "path_3.front_trigger_dist") paths_[2].front_trigger_dist = param.as_double();
      else if (name == "path_4_left_dist" || name == "path_4.target_left_dist") paths_[3].target_left_dist = param.as_double();
      else if (name == "path_4_front_dist" || name == "path_4.front_trigger_dist") paths_[3].front_trigger_dist = param.as_double();
      else if (name == "imu_yaw_offset_deg") {
        imu_yaw_offset_deg_ = param.as_double();
        imu_yaw_offset_rad_ = imu_yaw_offset_deg_ * M_PI / 180.0;
      } else if (name == "lidar_yaw_mount_deg") {
        lidar_yaw_mount_deg_ = param.as_double();
        lidar_yaw_mount_rad_ = lidar_yaw_mount_deg_ * M_PI / 180.0;
      } else if (name == "lidar_x_offset") lidar_x_offset_ = param.as_double();
      else if (name == "lidar_y_offset") lidar_y_offset_ = param.as_double();
      else if (name == "lidar_z_offset") lidar_z_offset_ = param.as_double();
      else if (name == "z_min") z_min_ = param.as_double();
      else if (name == "z_max") z_max_ = param.as_double();
      else if (name == "enable_ring_filter") enable_ring_filter_ = param.as_bool();
      else if (name == "ring_min") ring_min_ = param.as_int();
      else if (name == "ring_max") ring_max_ = param.as_int();
      else if (name == "filter_alpha") filter_alpha_ = param.as_double();
      else if (name == "enable_start_sit") enable_start_sit_ = param.as_bool();
      else if (name == "start_delay_sec") start_delay_sec_ = param.as_double();
      else if (name == "sit_duration") sit_duration_ = param.as_double();
      else if (name == "stand_duration") stand_duration_ = param.as_double();
      else if (name == "traffic_cycle_sec") traffic_cycle_sec_ = param.as_double();
      else if (name == "traffic_safety_margin_sec") traffic_safety_margin_sec_ = param.as_double();
      else if (name == "prefer_outer_wall") prefer_outer_wall_ = param.as_bool();
      else if (name == "enable_slowdown") enable_slowdown_ = param.as_bool();
      else if (name == "target_left_dist") {
        for (int i = 0; i < 4; ++i) paths_[i].target_left_dist = param.as_double();
        corners_[0].target_left_dist = param.as_double();
      } else if (name == "front_trigger_dist") {
        for (int i = 0; i < 4; ++i) paths_[i].front_trigger_dist = param.as_double();
      } else if (name == "path_1_yaw") {
        paths_[0].drive_yaw_deg = param.as_double();
        paths_[0].drive_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "path_2_yaw") {
        paths_[1].drive_yaw_deg = param.as_double();
        paths_[1].drive_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "path_3_yaw") {
        paths_[2].drive_yaw_deg = param.as_double();
        paths_[2].drive_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      } else if (name == "path_4_yaw") {
        paths_[3].drive_yaw_deg = param.as_double();
        paths_[3].drive_yaw_rad = normalizeAngle(param.as_double() * M_PI / 180.0);
      }

      // Check per-path target_left_dist
      for (int i = 0; i < 4; ++i) {
        if (name == "path_" + std::to_string(i + 1) + ".target_left_dist") {
          paths_[i].target_left_dist = param.as_double();
        } else if (name == "path_" + std::to_string(i + 1) + ".front_trigger_dist") {
          paths_[i].front_trigger_dist = param.as_double();
        }
      }
      RCLCPP_INFO(this->get_logger(), "[DYNAMIC PARAM] Updated '%s'", name.c_str());
    }
    return result;
  }

  // --- IMU Callback ---
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double qx = msg->orientation.x;
    const double qy = msg->orientation.y;
    const double qz = msg->orientation.z;
    const double qw = msg->orientation.w;

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    raw_imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    current_yaw_ = normalizeAngle(raw_imu_yaw_ - imu_yaw_zero_ + imu_yaw_offset_rad_);

    last_imu_time_ = this->now();
    imu_received_ = true;
  }

  // --- Traffic Light Callback ---
  void trafficLightCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::string text = msg->data;
    std::transform(text.begin(), text.end(), text.begin(), ::toupper);
    external_traffic_override_ = true;
    if (text.find("GREEN") != std::string::npos || text.find("GO") != std::string::npos || text.find("ON") != std::string::npos) {
      external_traffic_green_ = true;
    } else {
      external_traffic_green_ = false;
    }
  }

  // --- 2D RANSAC Line Fitting with Obstacle Rejection ---
  Line2D fitRansacLine(const std::vector<Point2D>& points, bool is_front, double expected_dist = 0.50) {
    Line2D best_line;
    if (points.size() < static_cast<size_t>(ransac_min_inliers_)) {
      return best_line;
    }

    std::mt19937 rng(1337);
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    int best_score = 0;

    for (int iter = 0; iter < ransac_iterations_; ++iter) {
      const size_t idx1 = dist(rng);
      size_t idx2 = dist(rng);
      if (idx1 == idx2) continue;

      const auto& p1 = points[idx1];
      const auto& p2 = points[idx2];

      const double dx = p2.x - p1.x;
      const double dy = p2.y - p1.y;
      const double len = std::hypot(dx, dy);
      if (len < 0.20) continue; // Sample baseline at least 20cm to eliminate angular jitter

      double a = -dy / len;
      double b = dx / len;
      double c = -(a * p1.x + b * p1.y);

      if (is_front) {
        if (a < 0.0) { a = -a; b = -b; c = -c; }
        if (a < 0.82) continue; // Wall must be facing robot within ~35 deg of perpendicular
      } else {
        if (b < 0.0) { a = -a; b = -b; c = -c; }
        if (b < 0.70) continue;

        const double candidate_dist = -c;
        if (candidate_dist < 0.12) continue; // Discard points too close to robot body (<12cm)
      }

      int inliers = 0;
      for (const auto& pt : points) {
        const double d = std::abs(a * pt.x + b * pt.y + c);
        if (d <= ransac_inlier_thresh_) {
          inliers++;
        }
      }

      int score = inliers;
      if (!is_front && prefer_outer_wall_) {
        const double line_dist = -c;
        if (line_dist >= (expected_dist - 0.12)) {
          score += 30; // Bonus for outer wall distance match
        }
      }

      if (score > best_score && inliers >= ransac_min_inliers_) {
        best_score = score;
        best_line.a = a;
        best_line.b = b;
        best_line.c = c;
      }
    }

    if (best_score > 0) {
      std::vector<Point2D> inliers;
      inliers.reserve(points.size());
      for (const auto& pt : points) {
        const double d = std::abs(best_line.a * pt.x + best_line.b * pt.y + best_line.c);
        if (d <= ransac_inlier_thresh_) {
          inliers.push_back(pt);
        }
      }

      if (inliers.size() >= static_cast<size_t>(ransac_min_inliers_)) {
        double mean_x = 0.0, mean_y = 0.0;
        for (const auto& pt : inliers) {
          mean_x += pt.x;
          mean_y += pt.y;
        }
        mean_x /= inliers.size();
        mean_y /= inliers.size();

        double c_xx = 0.0, c_xy = 0.0, c_yy = 0.0;
        for (const auto& pt : inliers) {
          const double dx = pt.x - mean_x;
          const double dy = pt.y - mean_y;
          c_xx += dx * dx;
          c_xy += dx * dy;
          c_yy += dy * dy;
        }

        // Direction of maximal variance along the line:
        const double phi = 0.5 * std::atan2(2.0 * c_xy, c_xx - c_yy);
        // Unit normal vector perpendicular to the line:
        double na = -std::sin(phi);
        double nb = std::cos(phi);

        if (is_front) {
          if (na < 0.0) { na = -na; nb = -nb; }
        } else {
          if (nb < 0.0) { na = -na; nb = -nb; }
        }

        const double norm = std::hypot(na, nb);
        best_line.a = na / norm;
        best_line.b = nb / norm;
        best_line.c = -(best_line.a * mean_x + best_line.b * mean_y);
        best_line.distance = -best_line.c;
        best_line.angle = std::atan2(best_line.b, best_line.a);
        best_line.inlier_count = static_cast<int>(inliers.size());

        if (is_front) {
          if (best_line.a < 0.82) {
            best_line.valid = false;
            return best_line;
          }
          double min_y = 1e6, max_y = -1e6;
          for (const auto& pt : inliers) {
            if (pt.y < min_y) min_y = pt.y;
            if (pt.y > max_y) max_y = pt.y;
          }
          if ((max_y - min_y) < 0.15) {
            best_line.valid = false;
            return best_line;
          }
        }
        best_line.valid = true;
      }
    }

    return best_line;
  }

  // --- PointCloud2 Callback ---
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<Point2D> front_pts;
    std::vector<Point2D> left_pts;
    front_pts.reserve(2048);
    left_pts.reserve(2048);

    bool has_ring = false;
    for (const auto& f : msg->fields) {
      if (f.name == "ring") { has_ring = true; break; }
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_ring;
    if (has_ring) {
      iter_ring = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");
    }

    // Check if cloud is already in base_link or if it is from hesai_lidar
    const bool is_base_frame = (msg->header.frame_id == "base_link" || msg->header.frame_id == "base");
    const double cos_yaw = std::cos(lidar_yaw_mount_rad_);
    const double sin_yaw = std::sin(lidar_yaw_mount_rad_);

    // Dynamic corridor width for front sector:
    // Robot width is ~0.31m. Collision corridor is |y| <= 0.25m.
    // If left wall is tracked at d_left, exclude points within 12cm of the wall
    // so side wall points never bleed into the front sector!
    const float max_front_y = frontMaxY();

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      uint16_t ring = 0;
      if (has_ring && iter_ring) {
        ring = **iter_ring;
        ++(*iter_ring);
      }

      const float raw_x = *iter_x;
      const float raw_y = *iter_y;
      const float raw_z = *iter_z;

      if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_z)) continue;

      // Layer/Ring Slicing (2D virtual lidar): default rings 7-8 for near-horizontal plane
      if (enable_ring_filter_) {
        if (has_ring) {
          if (ring < ring_min_ || ring > ring_max_) continue;
        } else {
          // Fallback if no ring field: thin vertical slice around LiDAR plane
          if (std::abs(raw_z) > 0.06f) continue;
        }
      }

      float x, y, z;
      if (is_base_frame) {
        x = raw_x;
        y = raw_y;
        z = raw_z;
      } else {
        // Transform from Hesai XT16 sensor frame to robot base_link frame
        // Hesai mounted on Go2: x_base = x_off + cos*x_l - sin*y_l
        // With yaw=+90 deg (cos=0, sin=1, x_off=0.25): x_base = 0.25 - y_l
        // y_base = y_off + sin*x_l + cos*y_l = x_l
        // z_base = z_l + z_off = z_l + 0.1588
        x = static_cast<float>(lidar_x_offset_ + cos_yaw * raw_x - sin_yaw * raw_y);
        y = static_cast<float>(lidar_y_offset_ + sin_yaw * raw_x + cos_yaw * raw_y);
        z = static_cast<float>(raw_z + lidar_z_offset_);
      }

      // Height filter in base_link frame (ground floor is at z ~= -0.33m)
      if (z < z_min_ || z > z_max_) continue;

      addSectorPoint(x, y, max_front_y, front_pts, left_pts);
    }

    updateWalls(front_pts, left_pts);
  }

  // Front corridor half-width: exclude points within 12 cm of the tracked left wall so side-wall
  // points never bleed into the front sector.
  float frontMaxY() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    float max_front_y = 0.25f;
    if (left_line_.valid && d_left_ > 0.15f && d_left_ < 3.0f) {
      max_front_y = std::min(0.25f, static_cast<float>(d_left_ - 0.12f));
    }
    return max_front_y;
  }

  void addSectorPoint(float x, float y, float max_front_y,
                      std::vector<Point2D> & front_pts, std::vector<Point2D> & left_pts) {
    // Front Sector: x 0.25..6.0 m ahead, y -0.25..max_front_y
    if (x > 0.25f && x <= 6.00f && y >= -0.25f && y <= max_front_y) {
      front_pts.push_back({x, y});
    }
    // Left Wall Sector: y 0.10..3.0 m, along the robot x -0.80..1.20 m
    if (y > 0.10f && y <= 3.00f && x >= -0.80f && x <= 1.20f) {
      left_pts.push_back({x, y});
    }
  }

  // /scan in base_stab: gravity-levelled at the robot base, x forward, y left (same as base_link
  // horizontally). With cloud_bridge scan_mode:=vertical every return is a vertical surface.
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<Point2D> front_pts;
    std::vector<Point2D> left_pts;
    front_pts.reserve(msg->ranges.size());
    left_pts.reserve(msg->ranges.size());
    const float max_front_y = frontMaxY();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      addSectorPoint(static_cast<float>(r * std::cos(ang)), static_cast<float>(r * std::sin(ang)),
                     max_front_y, front_pts, left_pts);
    }
    updateWalls(front_pts, left_pts);
  }

  void updateWalls(const std::vector<Point2D> & front_pts, const std::vector<Point2D> & left_pts) {
    const double expected_left = paths_[current_path_idx_].target_left_dist;
    Line2D front_line = fitRansacLine(front_pts, true, 0.65);
    Line2D left_line = fitRansacLine(left_pts, false, expected_left);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      front_line_ = front_line;
      left_line_ = left_line;

      // Adaptive low-pass filter: fast snap on sudden changes (>0.12m) to eliminate visual/control lag,
      // and gentle EMA on small changes to suppress lidar jitter.
      if (front_line_.valid) {
        const float raw_df = static_cast<float>(front_line_.distance);
        if (d_front_ > 50.0f || std::abs(raw_df - d_front_) > 0.12f) {
          d_front_ = raw_df;
        } else {
          d_front_ = static_cast<float>(filter_alpha_ * raw_df + (1.0 - filter_alpha_) * d_front_);
        }
        front_lost_count_ = 0;
      } else {
        front_lost_count_++;
        if (front_lost_count_ > 4) {
          d_front_ = 99.0f;
        }
      }

      if (left_line_.valid) {
        const float raw_dl = static_cast<float>(left_line_.distance);
        if (d_left_ < 0.0f || std::abs(raw_dl - d_left_) > 0.12f) {
          d_left_ = raw_dl;
        } else {
          d_left_ = static_cast<float>(filter_alpha_ * raw_dl + (1.0 - filter_alpha_) * d_left_);
        }
        left_lost_count_ = 0;
      } else {
        left_lost_count_++;
        if (left_lost_count_ > 4) {
          d_left_ = -1.0f;
        }
      }

      last_cloud_time_ = this->now();
      cloud_received_ = true;
    }

    // Publish RViz Markers (expressed in base_link)
    publishMarkers("base_link");

    // Immediately publish status for zero-latency telemetry response
    publishStatus();
  }

  // --- Publish RViz Visualization Markers ---
  void publishMarkers(const std::string& frame_id) {
    const std::string fid = frame_id.empty() ? "base_link" : frame_id;

    // 1. Left Wall Marker (Green line strip)
    if (left_line_.valid && std::abs(left_line_.b) > 1e-4) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = fid;
      m.header.stamp = this->now();
      m.ns = "ransac_walls";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.04; // line thickness
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 0.9f;

      geometry_msgs::msg::Point p1, p2;
      // ax + by + c = 0 => y = (-c - ax)/b
      p1.x = -1.2; p1.y = (-left_line_.c - left_line_.a * p1.x) / left_line_.b; p1.z = 0.0;
      p2.x = 1.2;  p2.y = (-left_line_.c - left_line_.a * p2.x) / left_line_.b; p2.z = 0.0;
      m.points.push_back(p1);
      m.points.push_back(p2);
      marker_left_pub_->publish(m);
    }

    // 2. Front Wall Marker (Red line strip)
    if (front_line_.valid && std::abs(front_line_.a) > 1e-4) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = fid;
      m.header.stamp = this->now();
      m.ns = "ransac_walls";
      m.id = 1;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.04;
      m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 0.9f;

      geometry_msgs::msg::Point p1, p2;
      // ax + by + c = 0 => x = (-c - by)/a
      p1.y = -0.6; p1.x = (-front_line_.c - front_line_.b * p1.y) / front_line_.a; p1.z = 0.0;
      p2.y = 0.6;  p2.x = (-front_line_.c - front_line_.b * p2.y) / front_line_.a; p2.z = 0.0;
      m.points.push_back(p1);
      m.points.push_back(p2);
      marker_front_pub_->publish(m);
    }
  }

  // --- Command Callback ---
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::string raw = msg->data;

    // Check if it is a SET_PARAM command: "SET_PARAM:key=val,key2=val2"
    if (raw.rfind("SET_PARAM", 0) == 0) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      size_t colon_pos = raw.find(':');
      if (colon_pos != std::string::npos) {
        std::string payload = raw.substr(colon_pos + 1);
        std::stringstream ss(payload);
        std::string item;
        while (std::getline(ss, item, ',')) {
          size_t eq_pos = item.find('=');
          if (eq_pos != std::string::npos) {
            std::string key = item.substr(0, eq_pos);
            std::string val_str = item.substr(eq_pos + 1);
            key.erase(0, key.find_first_not_of(" \t\r\n{\""));
            key.erase(key.find_last_not_of(" \t\r\n}\"") + 1);
            val_str.erase(0, val_str.find_first_not_of(" \t\r\n{\""));
            val_str.erase(val_str.find_last_not_of(" \t\r\n}\"") + 1);
            try {
              double v = std::stod(val_str);
              if (key == "cruise_speed") {
                cruise_speed_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] cruise_speed -> %.2f", v);
              } else if (key == "turn_speed") {
                turn_speed_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] turn_speed -> %.2f", v);
              } else if (key == "min_turn_speed") {
                min_turn_speed_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] min_turn_speed -> %.2f", v);
              } else if (key == "kp_dist") {
                kp_dist_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] kp_dist -> %.2f", v);
              } else if (key == "kp_yaw") {
                kp_yaw_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] kp_yaw -> %.2f", v);
              } else if (key == "kp_turn") {
                kp_turn_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] kp_turn -> %.2f", v);
              } else if (key == "turn_tolerance_deg") {
                turn_tolerance_deg_ = v;
                turn_tolerance_rad_ = v * M_PI / 180.0;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] turn_tolerance_deg -> %.2f", v);
              } else if (key == "post_turn_dwell_time") {
                post_turn_dwell_time_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] post_turn_dwell_time -> %.2f", v);
              } else if (key == "align_tol_yaw_deg") {
                align_tol_yaw_deg_ = v;
                align_tol_yaw_rad_ = v * M_PI / 180.0;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] align_tol_yaw_deg -> %.2f", v);
              } else if (key == "align_settle_time") {
                align_settle_time_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] align_settle_time -> %.2f", v);
              } else if (key == "align_tol_x") {
                align_tol_x_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] align_tol_x -> %.2f", v);
              } else if (key == "align_tol_y") {
                align_tol_y_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] align_tol_y -> %.2f", v);
              } else if (key == "corner_a_front_dist") {
                corners_[0].target_front_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_a_front_dist -> %.2f", v);
              } else if (key == "corner_a_left_dist") {
                corners_[0].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_a_left_dist -> %.2f", v);
              } else if (key == "corner_a_yaw") {
                corners_[0].target_yaw_deg = v;
                corners_[0].target_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_a_yaw -> %.2f", v);
              } else if (key == "corner_b_front_dist") {
                corners_[1].target_front_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_b_front_dist -> %.2f", v);
              } else if (key == "corner_b_left_dist") {
                corners_[1].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_b_left_dist -> %.2f", v);
              } else if (key == "corner_b_yaw") {
                corners_[1].target_yaw_deg = v;
                corners_[1].target_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_b_yaw -> %.2f", v);
              } else if (key == "corner_c_front_dist") {
                corners_[2].target_front_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_c_front_dist -> %.2f", v);
              } else if (key == "corner_c_left_dist") {
                corners_[2].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_c_left_dist -> %.2f", v);
              } else if (key == "corner_c_yaw") {
                corners_[2].target_yaw_deg = v;
                corners_[2].target_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_c_yaw -> %.2f", v);
              } else if (key == "corner_d_front_dist") {
                corners_[3].target_front_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_d_front_dist -> %.2f", v);
              } else if (key == "corner_d_left_dist") {
                corners_[3].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_d_left_dist -> %.2f", v);
              } else if (key == "corner_d_yaw") {
                corners_[3].target_yaw_deg = v;
                corners_[3].target_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] corner_d_yaw -> %.2f", v);
              } else if (key == "slowdown_dist") {
                slowdown_dist_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] slowdown_dist -> %.2f", v);
              } else if (key == "path_1_left_dist") {
                paths_[0].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_1_left_dist -> %.2f", v);
              } else if (key == "path_1_front_dist") {
                paths_[0].front_trigger_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_1_front_dist -> %.2f", v);
              } else if (key == "path_2_left_dist") {
                paths_[1].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_2_left_dist -> %.2f", v);
              } else if (key == "path_2_front_dist") {
                paths_[1].front_trigger_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_2_front_dist -> %.2f", v);
              } else if (key == "path_3_left_dist") {
                paths_[2].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_3_left_dist -> %.2f", v);
              } else if (key == "path_3_front_dist") {
                paths_[2].front_trigger_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_3_front_dist -> %.2f", v);
              } else if (key == "path_4_left_dist") {
                paths_[3].target_left_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_4_left_dist -> %.2f", v);
              } else if (key == "path_4_front_dist") {
                paths_[3].front_trigger_dist = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_4_front_dist -> %.2f", v);
              } else if (key == "imu_yaw_offset_deg") {
                imu_yaw_offset_deg_ = v;
                imu_yaw_offset_rad_ = v * M_PI / 180.0;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] imu_yaw_offset_deg -> %.2f", v);
              } else if (key == "z_min") {
                z_min_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] z_min -> %.2f", v);
              } else if (key == "z_max") {
                z_max_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] z_max -> %.2f", v);
              } else if (key == "enable_ring_filter") {
                enable_ring_filter_ = (v > 0.5);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] enable_ring_filter -> %s", enable_ring_filter_ ? "ON" : "OFF");
              } else if (key == "ring_min") {
                ring_min_ = static_cast<int>(std::round(v));
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] ring_min -> %d", ring_min_);
              } else if (key == "ring_max") {
                ring_max_ = static_cast<int>(std::round(v));
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] ring_max -> %d", ring_max_);
              } else if (key == "filter_alpha") {
                filter_alpha_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] filter_alpha -> %.2f", filter_alpha_);
              } else if (key == "lidar_yaw_mount_deg") {
                lidar_yaw_mount_deg_ = v;
                lidar_yaw_mount_rad_ = v * M_PI / 180.0;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] lidar_yaw_mount_deg -> %.2f", v);
              } else if (key == "enable_start_sit") {
                enable_start_sit_ = (v > 0.5);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] enable_start_sit -> %s", enable_start_sit_ ? "ON" : "OFF");
              } else if (key == "start_delay_sec") {
                start_delay_sec_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] start_delay_sec -> %.2f", v);
              } else if (key == "sit_duration") {
                sit_duration_ = v;
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] sit_duration -> %.2f", v);
              } else if (key == "target_left_dist") {
                // Legacy default: do not overwrite individual path_1..path_4 settings
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] target_left_dist -> %.2f", v);
              } else if (key == "front_trigger_dist") {
                // Legacy default: do not overwrite individual path_1..path_4 settings
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] front_trigger_dist -> %.2f", v);
              } else if (key == "path_1_yaw") {
                paths_[0].drive_yaw_deg = v;
                paths_[0].drive_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_1_yaw -> %.2f", v);
              } else if (key == "path_2_yaw") {
                paths_[1].drive_yaw_deg = v;
                paths_[1].drive_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_2_yaw -> %.2f", v);
              } else if (key == "path_3_yaw") {
                paths_[2].drive_yaw_deg = v;
                paths_[2].drive_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_3_yaw -> %.2f", v);
              } else if (key == "path_4_yaw") {
                paths_[3].drive_yaw_deg = v;
                paths_[3].drive_yaw_rad = normalizeAngle(v * M_PI / 180.0);
                RCLCPP_INFO(this->get_logger(), "[PARAM SET] path_4_yaw -> %.2f", v);
              }
            } catch (...) {
              RCLCPP_WARN(this->get_logger(), "[PARAM SET] Parse error for key='%s', val='%s'", key.c_str(), val_str.c_str());
            }
          }
        }
      }
      return;
    }

    std::string cmd = raw;
    cmd.erase(cmd.find_last_not_of(" \n\r\t") + 1);
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);

    RCLCPP_INFO(this->get_logger(), "Command received: '%s'", cmd.c_str());

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cmd == "START") {
      align_only_ = false;
      start_time_ = this->now();
      global_timer_started_ = true;
      RCLCPP_INFO(this->get_logger(), "🚦 [GLOBAL TIMER SYNC] Traffic light global timer synchronized to 0.0s!");
      if (state_ == PatrolState::IDLE) {
        if (imu_zeroed_) {
          // Already aligned at Corner a! Directly transition to START_SIT or DRIVE
          current_path_idx_ = 0;
          current_yaw_ = normalizeAngle(raw_imu_yaw_ - imu_yaw_zero_);
          target_yaw_ = paths_[0].drive_yaw_rad;
          if (enable_start_sit_ && (start_delay_sec_ > 0.0 || sit_duration_ > 0.0)) {
            sit_start_time_ = start_time_;
            state_ = PatrolState::START_SIT;
            sendPostureCommand("DOWN");
            mission_log_ = "⏳ [시작 대기] 엎드려서 첫 미션(20초) 대기 중...";
            RCLCPP_INFO(this->get_logger(), "Patrol STARTED -> Pre-aligned! Starting START_SIT.");
          } else {
            state_ = PatrolState::DRIVE;
            mission_log_ = "🚀 [1번길 순찰 주행 시작] 벽 추종 직진 주행을 시작합니다.";
            RCLCPP_INFO(this->get_logger(), "Patrol STARTED -> Pre-aligned! Starting DRIVE.");
          }
        } else {
          // Not aligned yet: perform ALIGN_INIT first
          current_path_idx_ = 0;
          state_ = PatrolState::ALIGN_INIT;
          align_settling_ = false;
          start_time_ = this->now();
          mission_log_ = "📐 [출발 준비] 코너 a 정렬 및 IMU 영점 설정을 시작합니다.";
          RCLCPP_INFO(this->get_logger(), "Patrol STARTED -> Starting with ALIGN_INIT at corner_a");
        }
      } else if (state_ == PatrolState::ALIGN_INIT) {
        align_only_ = false;
        RCLCPP_INFO(this->get_logger(), "Patrol START received during ALIGN_INIT -> Will proceed upon alignment.");
      }
    } else if (cmd == "STOP") {
      state_ = PatrolState::IDLE;
      align_only_ = false;
      publishZeroVelocity();
      mission_log_ = "🛠️ [정지 / 튜닝 모드] 로봇이 안전하게 정지했습니다. 센서 계측 및 파라미터 조절 가능";
      RCLCPP_INFO(this->get_logger(), "Patrol STOPPED -> IDLE");
    } else if (cmd == "ZERO_IMU") {
      imu_yaw_zero_ = raw_imu_yaw_;
      imu_zeroed_ = true;
      current_yaw_ = normalizeAngle(imu_yaw_offset_rad_);
      mission_log_ = "🧭 [IMU 영점 재설정] 현재 로봇 헤딩을 기준 0.0°로 설정 완료";
      RCLCPP_INFO(this->get_logger(), "IMU Zeroed directly to current heading: %.2f deg", raw_imu_yaw_ * 180.0 / M_PI);
    } else if (cmd == "ALIGN" || cmd == "ALIGN A" || cmd == "ALIGN_A") {
      align_only_ = true;
      state_ = PatrolState::ALIGN_INIT;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "📐 [코너 a 정렬 시작] 전방/좌측 벽면 RANSAC 3축 정밀 정렬 및 IMU 영점 설정을 수행합니다.";
      RCLCPP_INFO(this->get_logger(), "Patrol ALIGN command -> ALIGN_INIT at corner_a (align_only=true)");
    } else if (cmd == "ALIGN B" || cmd == "ALIGN_B") {
      current_path_idx_ = 0; // target_corner_idx = 1 (corner_b)
      state_ = PatrolState::CORNER_ALIGN;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "📐 [수동 전환] 코너 b 정밀 도킹(CORNER_ALIGN corner_b)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to CORNER_ALIGN at corner_b");
    } else if (cmd == "ALIGN C" || cmd == "ALIGN_C") {
      current_path_idx_ = 1; // target_corner_idx = 2 (corner_c)
      state_ = PatrolState::CORNER_ALIGN;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "📐 [수동 전환] 코너 c 정밀 도킹(CORNER_ALIGN corner_c)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to CORNER_ALIGN at corner_c");
    } else if (cmd == "ALIGN D" || cmd == "ALIGN_D") {
      current_path_idx_ = 2; // target_corner_idx = 3 (corner_d)
      state_ = PatrolState::CORNER_ALIGN;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "📐 [수동 전환] 코너 d 정밀 도킹(CORNER_ALIGN corner_d)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to CORNER_ALIGN at corner_d");
    } else if (cmd == "GOTO 1" || cmd == "PATH 1" || cmd == "DRIVE 1" || cmd == "GOTO_1" || cmd == "PATH_1") {
      current_path_idx_ = 0;
      target_yaw_ = paths_[0].drive_yaw_rad;
      state_ = PatrolState::DRIVE;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "🚀 [수동 전환] 1번길 벽 추종 순항(DRIVE path_1)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to DRIVE path_1 (heading: %.1f deg)", paths_[0].drive_yaw_deg);
    } else if (cmd == "GOTO 2" || cmd == "PATH 2" || cmd == "DRIVE 2" || cmd == "GOTO_2" || cmd == "PATH_2") {
      current_path_idx_ = 1;
      target_yaw_ = paths_[1].drive_yaw_rad;
      state_ = PatrolState::DRIVE;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "🚀 [수동 전환] 2번길 벽 추종 순항(DRIVE path_2)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to DRIVE path_2 (heading: %.1f deg)", paths_[1].drive_yaw_deg);
    } else if (cmd == "GOTO 3" || cmd == "PATH 3" || cmd == "DRIVE 3" || cmd == "GOTO_3" || cmd == "PATH_3") {
      current_path_idx_ = 2;
      target_yaw_ = paths_[2].drive_yaw_rad;
      state_ = PatrolState::DRIVE;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "🚀 [수동 전환] 3번길 벽 추종 순항(DRIVE path_3)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to DRIVE path_3 (heading: %.1f deg)", paths_[2].drive_yaw_deg);
    } else if (cmd == "GOTO 4" || cmd == "PATH 4" || cmd == "DRIVE 4" || cmd == "GOTO_4" || cmd == "PATH_4") {
      current_path_idx_ = 3;
      target_yaw_ = paths_[3].drive_yaw_rad;
      state_ = PatrolState::DRIVE;
      align_settling_ = false;
      sendPostureCommand("STAND");
      mission_log_ = "🚀 [수동 전환] 4번길 벽 추종 순항(DRIVE path_4)으로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to DRIVE path_4 (heading: %.1f deg)", paths_[3].drive_yaw_deg);
    } else if (cmd == "GOTO TRAFFIC" || cmd == "TRAFFIC") {
      state_ = PatrolState::TRAFFIC_CHECK;
      align_settling_ = false;
      mission_log_ = "🚦 [수동 전환] 신호등 대기 상태(TRAFFIC_CHECK)로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to TRAFFIC_CHECK");
    } else if (cmd == "GOTO TURN" || cmd == "TURN") {
      state_ = PatrolState::CORNER_TURN;
      align_settling_ = false;
      mission_log_ = "🔄 [수동 전환] 코너 회전 상태(CORNER_TURN)로 강제 전환되었습니다.";
      RCLCPP_INFO(this->get_logger(), "State FORCED to CORNER_TURN");
    } else if (cmd == "SYNC_TIME" || cmd == "RESET_TIME" || cmd == "SYNC_TL" || cmd == "SYNC" || cmd == "ZERO_TIME") {
      start_time_ = this->now();
      global_timer_started_ = true;
      mission_log_ = "🚦 [글로벌 타임 동기화] 신호등 타이머가 0.0s(초록불 시작)로 재설정되었습니다.";
      RCLCPP_INFO(this->get_logger(), "🚦 [GLOBAL TIMER SYNC] Traffic light global timer resynchronized to 0.0s!");
    } else if (cmd.rfind("SET_TIME", 0) == 0) {
      std::string val_str = cmd.substr(8);
      val_str.erase(0, val_str.find_first_not_of(" :="));
      try {
        double offset_sec = std::stod(val_str);
        start_time_ = this->now() - rclcpp::Duration::from_seconds(offset_sec);
        global_timer_started_ = true;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "🚦 [글로벌 타임 설정] 신호등 경과시간이 " << offset_sec << "s로 재설정되었습니다.";
        mission_log_ = ss.str();
        RCLCPP_INFO(this->get_logger(), "🚦 [GLOBAL TIMER SET] Elapsed time set to %.1fs", offset_sec);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse SET_TIME value: '%s'", val_str.c_str());
      }
    } else if (cmd == "TL GREEN" || cmd == "TL_GREEN" || cmd == "GREEN") {
      external_traffic_override_ = true;
      external_traffic_green_ = true;
      mission_log_ = "🚦 [신호 강제] 초록불(GREEN) 강제 통과 모드 활성화";
      RCLCPP_INFO(this->get_logger(), "Traffic light FORCED to GREEN (external override)");
    } else if (cmd == "TL RED" || cmd == "TL_RED" || cmd == "RED") {
      external_traffic_override_ = true;
      external_traffic_green_ = false;
      mission_log_ = "🚦 [신호 강제] 빨간불(RED) 대기 모드 활성화";
      RCLCPP_INFO(this->get_logger(), "Traffic light FORCED to RED (external override)");
    } else if (cmd == "TL AUTO" || cmd == "TL_AUTO" || cmd == "AUTO") {
      external_traffic_override_ = false;
      mission_log_ = "🚦 [신호 복구] 30초 글로벌 자동 타이머 모드로 복구되었습니다.";
      RCLCPP_INFO(this->get_logger(), "Traffic light restored to AUTO (30s cycle timer)");
    } else if (cmd == "RESET") {
      state_ = PatrolState::IDLE;
      align_only_ = false;
      imu_zeroed_ = false;
      current_path_idx_ = 0;
      publishZeroVelocity();
      mission_log_ = "🔄 [코스 초기화] 1번길(코너 a) 시작 상태로 리셋되었습니다.";
      RCLCPP_INFO(this->get_logger(), "Patrol RESET to IDLE (corner_a / path_1)");
    } else if (cmd == "SIT" || cmd == "DOWN" || cmd == "STAND_DOWN") {
      sendPostureCommand("DOWN");
      mission_log_ = "🧎 [엎드리기] 바닥에 엎드립니다 (StandDown).";
      RCLCPP_INFO(this->get_logger(), "Posture command: STAND_DOWN");
    } else if (cmd == "STAND" || cmd == "UP" || cmd == "STAND_UP") {
      sendPostureCommand("STAND");
      mission_log_ = "🐕 [일어서기] 정상 기립합니다 (StandUp -> BalanceStand).";
      RCLCPP_INFO(this->get_logger(), "Posture command: STAND_UP");
    }
  }

  // --- Main Control Loop (20Hz) ---
  void controlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    const rclcpp::Time now = this->now();

    if (!imu_received_ || !cloud_received_) {
      if (state_ != PatrolState::IDLE) {
        publishZeroVelocity();
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for sensors... (IMU: %s, Cloud: %s)",
        imu_received_ ? "OK" : "WAITING",
        cloud_received_ ? "OK" : "WAITING");
      return;
    }

    const double imu_age = (now - last_imu_time_).seconds();
    const double cloud_age = (now - last_cloud_time_).seconds();
    if (imu_age > sensor_timeout_ || cloud_age > sensor_timeout_) {
      if (state_ != PatrolState::IDLE) {
        publishZeroVelocity();
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "SENSOR TIMEOUT! IMU age: %.3fs, Cloud age: %.3fs. Safety stop active.",
        imu_age, cloud_age);
      return;
    }

    const int path_idx = current_path_idx_;
    const int target_corner_idx = (path_idx + 1) % 4;

    switch (state_) {
      case PatrolState::IDLE: {
        // Do not publish to /cmd_vel in IDLE so external test tools or teleop can control
        break;
      }

      case PatrolState::ALIGN_INIT: {
        const auto& corner_a = corners_[0];
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;

        double err_x = 0.0;
        double err_y = 0.0;
        double err_yaw = 0.0;

        // 1. Front wall distance alignment (x)
        if (front_line_.valid) {
          err_x = d_front_ - corner_a.target_front_dist;
          vx = kp_x_ * err_x;
          if (std::abs(err_x) > align_tol_x_) {
            const double min_align_vx = 0.22;
            if (std::abs(vx) < min_align_vx) {
              vx = std::copysign(min_align_vx, vx);
            }
          }
          vx = std::clamp(vx, -max_align_vx_, max_align_vx_);
        } else {
          // If front wall is not reached yet, move forward at stepping speed
          vx = 0.25;
        }

        // 2. Left wall distance and heading alignment (y and yaw)
        if (left_line_.valid) {
          err_y = d_left_ - corner_a.target_left_dist;
          vy = kp_dist_ * err_y;
          if (std::abs(err_y) > align_tol_y_) {
            const double min_align_vy = 0.18;
            if (std::abs(vy) < min_align_vy) {
              vy = std::copysign(min_align_vy, vy);
            }
          }
          vy = std::clamp(vy, -max_vy_, max_vy_);

          // Normal of left wall in robot frame: angle = atan2(b, a).
          // Perfectly parallel to left wall implies normal is at 90 deg (pi/2).
          err_yaw = normalizeAngle(left_line_.angle - (M_PI / 2.0));
          wz = kp_yaw_ * err_yaw;
          if (std::abs(err_yaw) > align_tol_yaw_rad_) {
            if (std::abs(wz) < min_turn_speed_) {
              wz = std::copysign(min_turn_speed_, wz);
            }
          }
          wz = std::clamp(wz, -turn_speed_, turn_speed_);
        }

        const bool x_ok = front_line_.valid && (std::abs(err_x) <= align_tol_x_);
        const bool y_ok = left_line_.valid && (std::abs(err_y) <= align_tol_y_);
        const bool yaw_ok = left_line_.valid && (std::abs(err_yaw) <= align_tol_yaw_rad_);

        std::ostringstream ss_align;
        ss_align << std::fixed << std::setprecision(2);
        ss_align << "📐 [코너 a 정렬 중] 앞벽오차: " << err_x << "m, 좌측오차: " << err_y
                 << "m, 각도오차: " << std::setprecision(1) << (err_yaw * 180.0 / M_PI) << "°";
        mission_log_ = ss_align.str();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[ALIGN_INIT at corner_a] err_x=%.2fm, err_y=%.2fm, err_yaw=%.1f deg (x:%d, y:%d, yaw:%d)",
          err_x, err_y, err_yaw * 180.0 / M_PI, x_ok, y_ok, yaw_ok);

        if (x_ok && y_ok && yaw_ok) {
          if (!align_settling_) {
            align_settling_ = true;
            align_settle_start_ = now;
            mission_log_ = "📐 [코너 a 정렬 완료 중] 잔여 안정화 대기 중...";
          } else if ((now - align_settle_start_).seconds() >= align_settle_time_) {
            publishZeroVelocity();
            align_settling_ = false;

            // Zero the IMU reference heading to the current aligned physical heading!
            imu_yaw_zero_ = raw_imu_yaw_;
            current_yaw_ = 0.0;
            imu_zeroed_ = true;

            current_path_idx_ = 0;
            target_yaw_ = paths_[0].drive_yaw_rad;

            RCLCPP_INFO(this->get_logger(),
              "============================================================");
            RCLCPP_INFO(this->get_logger(),
              "🎯 [ALIGN_INIT SUCCESS] Corner 'a' aligned with RANSAC!");
            RCLCPP_INFO(this->get_logger(),
              "   Front Dist: %.2fm, Left Dist: %.2fm, Wall Angle: %.1f deg",
              d_front_, d_left_, left_line_.angle * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(),
              "   IMU Zero Offset set to: %.2f deg -> Current Yaw calibrated to: 0.0 deg",
              imu_yaw_zero_ * 180.0 / M_PI);

            if (align_only_) {
              state_ = PatrolState::IDLE;
              publishZeroVelocity();
              mission_log_ = "🎯 [정렬 완료] 코너 a 정렬 & IMU 영점(0.0°) 설정 완료! 로봇 정지 대기 중 (사용자 START 대기)";
              RCLCPP_INFO(this->get_logger(),
                "🎯 [ALIGN ONLY SUCCESS] Corner 'a' aligned & IMU zeroed! Standing ready in IDLE. Waiting for user START command.");
              RCLCPP_INFO(this->get_logger(),
                "============================================================");
              break;
            }

            if (enable_start_sit_ && (start_delay_sec_ > 0.0 || sit_duration_ > 0.0)) {
              sit_start_time_ = now;
              state_ = PatrolState::START_SIT;
              sendPostureCommand("DOWN");
              mission_log_ = "⏳ [시작 대기] 엎드려서 첫 미션(20초) 대기 중...";
              RCLCPP_INFO(this->get_logger(),
                "   Standing down (StandDown) for start delay (%.1fs) + mission (%.1fs) = total %.1fs.",
                start_delay_sec_, sit_duration_, start_delay_sec_ + sit_duration_);
            } else {
              state_ = PatrolState::DRIVE;
              mission_log_ = "🚀 [1번길 순찰 주행 시작] 벽 추종 직진 주행을 시작합니다.";
              RCLCPP_INFO(this->get_logger(), "   Starting DRIVE on path_1!");
            }
            RCLCPP_INFO(this->get_logger(),
              "============================================================");
            break;
          }
        } else {
          align_settling_ = false;
        }

        publishVelocity(vx, vy, wz);
        break;
      }

      case PatrolState::START_SIT: {
        publishZeroVelocity();
        const double total_sit_target = start_delay_sec_ + sit_duration_;
        const double sit_elapsed = (now - sit_start_time_).seconds();

        std::ostringstream ss_wait;
        ss_wait << std::fixed << std::setprecision(1);
        if (sit_elapsed < start_delay_sec_) {
          ss_wait << "⏳ [첫 미션 대기] 20초 생성 대기 중 (" << sit_elapsed << "s / " << start_delay_sec_ << "s)";
        } else {
          const double mission_elapsed = sit_elapsed - start_delay_sec_;
          ss_wait << "🧎 [시작 미션 수행] 미션 처리 중 (" << mission_elapsed << "s / " << sit_duration_ << "s)";
        }
        mission_log_ = ss_wait.str();

        if (sit_elapsed >= total_sit_target) {
          stand_start_time_ = now;
          state_ = PatrolState::START_STAND;
          sendPostureCommand("STAND");
          mission_log_ = "🧍 [미션 완료 기립] 일어서는 중...";
          RCLCPP_INFO(this->get_logger(),
            "[START SIT COMPLETE] Start delay (%.1fs) + mission (%.1fs) = total %.1fs complete. Standing up to cruise...",
            start_delay_sec_, sit_duration_, total_sit_target);
        }
        break;
      }

      case PatrolState::START_STAND: {
        publishZeroVelocity();
        const double stand_elapsed = (now - stand_start_time_).seconds();
        std::ostringstream ss_std;
        ss_std << std::fixed << std::setprecision(1);
        ss_std << "🧍 [시작 기립] 일어서는 중 (" << stand_elapsed << "s / " << stand_duration_ << "s)";
        mission_log_ = ss_std.str();

        if (stand_elapsed >= stand_duration_) {
          state_ = PatrolState::DRIVE;
          mission_log_ = "🚀 [1번길 순찰 주행 시작] 벽 추종 직진 주행을 시작합니다.";
          RCLCPP_INFO(this->get_logger(),
            "[START STAND COMPLETE] Robot standing ready. Starting Path 1 cruise!");
        }
        break;
      }

      case PatrolState::DRIVE: {
        const auto& path = paths_[path_idx];
        const auto& target_corner = corners_[target_corner_idx];

        std::ostringstream ss_drive;
        ss_drive << std::fixed << std::setprecision(2);
        ss_drive << "🚀 [" << path.name << " 순항 주행] 목표좌측: " << path.target_left_dist
                 << "m (현재: " << d_left_ << "m) | 앞벽: " << d_front_
                 << "m (도착트리거: " << path.front_trigger_dist << "m)";
        mission_log_ = ss_drive.str();

        // 1. Check Corner Arrival condition
        if (front_line_.valid && d_front_ <= path.front_trigger_dist) {
          publishZeroVelocity();
          align_settling_ = false;
          state_ = PatrolState::CORNER_ALIGN;
          mission_log_ = "🎯 [" + target_corner.name + " 도착] 정밀 도킹 위치 조정을 시작합니다.";
          RCLCPP_INFO(this->get_logger(),
            "[CORNER REACHED] %s: Front RANSAC d_front=%.2fm <= trigger=%.2fm. Arrived at %s! Aligning...",
            path.name.c_str(), d_front_, path.front_trigger_dist, target_corner.name.c_str());
          break;
        }

        // 2. Forward Velocity (vx): Full cruise speed
        double vx = cruise_speed_;
        if (enable_slowdown_ && front_line_.valid && d_front_ < slowdown_dist_) {
          const double span = std::max(0.05, slowdown_dist_ - path.front_trigger_dist);
          const double ratio = std::clamp((d_front_ - path.front_trigger_dist) / span, 0.0, 1.0);
          vx = 0.10 + (cruise_speed_ - 0.10) * ratio;
        }

        // 3. Lateral Velocity (vy): Left Wall RANSAC distance tracking
        double vy = 0.0;
        if (left_line_.valid) {
          const double e_dist = d_left_ - path.target_left_dist;
          vy = kp_dist_ * e_dist;
          vy = std::clamp(vy, -max_vy_, max_vy_);
        } else {
          vy = 0.0;
        }

        // 4. Heading Velocity (wz): Lock corridor orientation
        const double e_yaw = normalizeAngle(path.drive_yaw_rad - current_yaw_);
        double wz = kp_yaw_ * e_yaw;
        wz = std::clamp(wz, -turn_speed_, turn_speed_);

        publishVelocity(vx, vy, wz);
        break;
      }

      case PatrolState::CORNER_ALIGN: {
        const auto& corner = corners_[target_corner_idx];
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;

        double err_x = 0.0;
        double err_y = 0.0;
        // The robot must maintain the heading of the path it just drove while docking at the corner
        const double path_target_yaw = paths_[path_idx].drive_yaw_rad;
        double err_yaw = normalizeAngle(path_target_yaw - current_yaw_);

        // Front alignment
        if (front_line_.valid) {
          err_x = d_front_ - corner.target_front_dist;
          vx = kp_x_ * err_x;
          if (std::abs(err_x) > align_tol_x_) {
            const double min_align_vx = 0.22;
            if (std::abs(vx) < min_align_vx) {
              vx = std::copysign(min_align_vx, vx);
            }
          }
          vx = std::clamp(vx, -max_align_vx_, max_align_vx_);
        }

        // Left alignment
        if (left_line_.valid) {
          err_y = d_left_ - corner.target_left_dist;
          vy = kp_dist_ * err_y;
          if (std::abs(err_y) > align_tol_y_) {
            const double min_align_vy = 0.18;
            if (std::abs(vy) < min_align_vy) {
              vy = std::copysign(min_align_vy, vy);
            }
          }
          vy = std::clamp(vy, -max_vy_, max_vy_);
        }

        // Heading alignment
        wz = kp_yaw_ * err_yaw;
        if (std::abs(err_yaw) > align_tol_yaw_rad_) {
          if (std::abs(wz) < min_turn_speed_) {
            wz = std::copysign(min_turn_speed_, wz);
          }
        }
        wz = std::clamp(wz, -turn_speed_, turn_speed_);

        const bool x_ok = front_line_.valid && (std::abs(err_x) <= align_tol_x_);
        const bool y_ok = left_line_.valid && (std::abs(err_y) <= align_tol_y_);
        const bool yaw_ok = (std::abs(err_yaw) <= align_tol_yaw_rad_);

        std::ostringstream ss_ca;
        ss_ca << std::fixed << std::setprecision(2);
        ss_ca << "🎯 [" << corner.name << " 정밀 도킹] 앞벽(" << corner.target_front_dist << "m) 오차: " << err_x
              << "m | 좌측(" << corner.target_left_dist << "m) 오차: " << err_y
              << "m | 각도오차: " << std::setprecision(1) << (err_yaw * 180.0 / M_PI) << "°";
        mission_log_ = ss_ca.str();

        if (x_ok && y_ok && yaw_ok) {
          if (!align_settling_) {
            align_settling_ = true;
            align_settle_start_ = now;
          } else if ((now - align_settle_start_).seconds() >= align_settle_time_) {
            publishZeroVelocity();
            align_settling_ = false;
            sit_start_time_ = now;
            state_ = PatrolState::CORNER_SIT;
            sendPostureCommand("DOWN");
            mission_log_ = "🧎 [" + corner.name + " 도킹 완료] 엎드리기 미션을 수행합니다.";
            RCLCPP_INFO(this->get_logger(),
              "[%s ALIGNED] (ex=%.3fm, ey=%.3fm, eyaw=%.1f deg). Standing down (엎드리기) for %.1fs.",
              corner.name.c_str(), err_x, err_y, err_yaw * 180.0 / M_PI, sit_duration_);
            break;
          }
        } else {
          align_settling_ = false;
        }

        publishVelocity(vx, vy, wz);
        break;
      }

      case PatrolState::CORNER_SIT: {
        publishZeroVelocity();
        const double sit_elapsed = (now - sit_start_time_).seconds();
        std::ostringstream ss_sit;
        ss_sit << std::fixed << std::setprecision(1);
        ss_sit << "🧎 [" << corners_[target_corner_idx].name << " 엎드림] 바닥에 엎드린 상태 유지 중 (" << sit_elapsed << "s / " << sit_duration_ << "s)";
        mission_log_ = ss_sit.str();

        if (sit_elapsed >= sit_duration_) {
          stand_start_time_ = now;
          state_ = PatrolState::CORNER_STAND;
          sendPostureCommand("STAND");
          mission_log_ = "🧍 [" + corners_[target_corner_idx].name + " 기립] 일어서는 중...";
          RCLCPP_INFO(this->get_logger(),
            "[%s] StandDown duration complete. Standing up...", corners_[target_corner_idx].name.c_str());
        }
        break;
      }

      case PatrolState::CORNER_STAND: {
        publishZeroVelocity();
        const double stand_elapsed = (now - stand_start_time_).seconds();
        std::ostringstream ss_std;
        ss_std << std::fixed << std::setprecision(1);
        ss_std << "🧍 [" << corners_[target_corner_idx].name << " 기립] 일어서는 중 (" << stand_elapsed << "s / " << stand_duration_ << "s)";
        mission_log_ = ss_std.str();

        if (stand_elapsed >= stand_duration_) {
          state_ = PatrolState::TRAFFIC_CHECK;
          mission_log_ = "🚦 다음 구간 신호등 상태 확인 중...";
          RCLCPP_INFO(this->get_logger(),
            "[%s] Standing complete. Checking traffic light for next path.",
            corners_[target_corner_idx].name.c_str());
        }
        break;
      }

      case PatrolState::TRAFFIC_CHECK: {
        publishZeroVelocity();
        const int next_path_idx = (current_path_idx_ + 1) % 4;
        const auto& next_path = paths_[next_path_idx];

        bool can_proceed = false;

        if (!next_path.has_traffic_light) {
          can_proceed = true;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[TRAFFIC] %s (건너편) has NO traffic light. Proceeding immediately!", next_path.name.c_str());
        } else {
          if (external_traffic_override_) {
            can_proceed = external_traffic_green_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
              "[TRAFFIC] %s waiting on external topic: %s",
              next_path.name.c_str(), can_proceed ? "GREEN (GO)" : "RED (WAIT)");
          } else {
            const double elapsed = (now - start_time_).seconds();
            const int cycle_idx = static_cast<int>(elapsed / traffic_cycle_sec_);
            const double rem = traffic_cycle_sec_ - std::fmod(elapsed, traffic_cycle_sec_);
            const bool is_green = (cycle_idx % 2 == 0);
            can_proceed = is_green && (rem >= traffic_safety_margin_sec_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
              "[TRAFFIC] %s 30s timer: %s (Elapsed: %.1fs, Rem: %.1fs, SafeMargin: %.1fs, Cycle #%d)",
              next_path.name.c_str(), can_proceed ? "GREEN (GO)" : (is_green ? "YELLOW (TOO CLOSE TO RED)" : "RED (WAIT)"),
              elapsed, rem, traffic_safety_margin_sec_, cycle_idx);
          }
        }

        std::ostringstream ss_tf;
        const double elapsed_tf = (now - start_time_).seconds();
        const double rem_tf = traffic_cycle_sec_ - std::fmod(elapsed_tf, traffic_cycle_sec_);
        const bool is_green_tf = (static_cast<int>(elapsed_tf / traffic_cycle_sec_) % 2 == 0);
        if (can_proceed) {
          ss_tf << "🚦 [" << next_path.name << " 신호] 🟢 초록불 (" << std::fixed << std::setprecision(1) << rem_tf << "s 남음, 출발 가능)";
        } else if (is_green_tf) {
          ss_tf << "🚦 [" << next_path.name << " 신호] ⚠️ 잔여시간 부족 (" << std::fixed << std::setprecision(1) << rem_tf << "s < " << traffic_safety_margin_sec_ << "s, 다음 신호 대기)";
        } else {
          ss_tf << "🚦 [" << next_path.name << " 신호] 🔴 빨간불 (대기 중, 잔여: " << std::fixed << std::setprecision(1) << rem_tf << "s)";
        }
        mission_log_ = ss_tf.str();

        if (can_proceed) {
          state_ = PatrolState::CORNER_TURN;
          mission_log_ = "🔄 [" + next_path.name + " 진입] 90° 우회전을 시작합니다.";
          RCLCPP_INFO(this->get_logger(),
            "[TRAFFIC PASS] Turning 90 deg CW onto %s (Target: %.1f deg)",
            next_path.name.c_str(), next_path.drive_yaw_deg);
        }
        break;
      }

      case PatrolState::CORNER_TURN: {
        const int next_path_idx = (current_path_idx_ + 1) % 4;
        const double target_yaw = paths_[next_path_idx].drive_yaw_rad;
        const double e_yaw = normalizeAngle(target_yaw - current_yaw_);

        std::ostringstream ss_turn;
        ss_turn << std::fixed << std::setprecision(1);
        ss_turn << "🔄 [" << paths_[next_path_idx].name << " 방향 90° 회전] 목표각: " << paths_[next_path_idx].drive_yaw_deg
                << "° | 잔여각 오차: " << (e_yaw * 180.0 / M_PI) << "°";
        mission_log_ = ss_turn.str();

        if (std::abs(e_yaw) <= turn_tolerance_rad_) {
          publishZeroVelocity();
          dwell_start_time_ = now;
          state_ = PatrolState::POST_TURN_DWELL;
          mission_log_ = "⏸️ [회전 완료] 직진 주행 전 자세 안정화 대기 중...";
          RCLCPP_INFO(this->get_logger(),
            "[TURN COMPLETE] Turned onto %s (|e_yaw|=%.2f deg). Dwell for stabilization.",
            paths_[next_path_idx].name.c_str(), e_yaw * 180.0 / M_PI);
          break;
        }

        double wz = kp_turn_ * e_yaw;
        if (std::abs(wz) < min_turn_speed_) {
          wz = std::copysign(min_turn_speed_, e_yaw);
        }
        wz = std::clamp(wz, -turn_speed_, turn_speed_);

        publishVelocity(0.0, 0.0, wz);
        break;
      }

      case PatrolState::POST_TURN_DWELL: {
        publishZeroVelocity();
        mission_log_ = "⏸️ [회전 완료] 직진 주행 전 자세 안정화 대기 중...";
        if ((now - dwell_start_time_).seconds() >= post_turn_dwell_time_) {
          current_path_idx_ = (current_path_idx_ + 1) % 4;
          total_laps_completed_ += (current_path_idx_ == 0) ? 1 : 0;
          state_ = PatrolState::DRIVE;
          mission_log_ = "🚀 [" + paths_[current_path_idx_].name + " 진입] 순항 주행을 시작합니다.";
          RCLCPP_INFO(this->get_logger(),
            "============================================================");
          RCLCPP_INFO(this->get_logger(),
            " [ENTER PATH] Now cruising %s! Target Left: %.2fm, Target Yaw: %.1f deg",
            paths_[current_path_idx_].name.c_str(),
            paths_[current_path_idx_].target_left_dist,
            paths_[current_path_idx_].drive_yaw_deg);
          RCLCPP_INFO(this->get_logger(),
            "============================================================");
        }
        break;
      }
    }
  }

  // --- Helper: Angle Normalization to [-pi, pi] ---
  static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // --- Helper: Publish Posture Commands ---
  void sendPostureCommand(const std::string& posture) {
    std_msgs::msg::String msg;
    msg.data = posture;
    posture_pub_->publish(msg);

    std_msgs::msg::String motion_msg;
    if (posture == "DOWN" || posture == "SIT" || posture == "STAND_DOWN") {
      motion_msg.data = "STAND_DOWN";
    } else {
      motion_msg.data = "STAND_UP";
    }
    robot_motion_pub_->publish(motion_msg);
  }

  // --- Helper: Publish Twist Velocity ---
  void publishVelocity(double vx, double vy, double wz) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = wz;
    cmd_vel_pub_->publish(cmd);

    last_vx_ = vx;
    last_vy_ = vy;
    last_wz_ = wz;
  }

  void publishZeroVelocity() {
    if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) return;
    publishVelocity(0.0, 0.0, 0.0);
  }

  // --- Status Publisher (10Hz) ---
  void publishStatus() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    const int next_path_idx = (current_path_idx_ + 1) % 4;
    std::string tl_status = "NO_LIGHT";
    double elapsed = 0.0;
    double cycle_rem = 0.0;
    int cycle_idx = 0;

    if (global_timer_started_) {
      elapsed = (this->now() - start_time_).seconds();
      cycle_idx = static_cast<int>(elapsed / traffic_cycle_sec_);
      cycle_rem = traffic_cycle_sec_ - std::fmod(elapsed, traffic_cycle_sec_);
    }

    if (paths_[next_path_idx].has_traffic_light) {
      if (external_traffic_override_) {
        tl_status = external_traffic_green_ ? "GREEN" : "RED";
      } else {
        tl_status = (cycle_idx % 2 == 0) ? "GREEN" : "RED";
      }
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{"
       << "\"mission_log\":\"" << mission_log_ << "\","
       << "\"state\":\"" << stateToString(state_) << "\","
       << "\"current_path\":\"" << paths_[current_path_idx_].name << "\","
       << "\"target_corner\":\"" << corners_[(current_path_idx_ + 1) % 4].name << "\","
       << "\"d_left\":" << d_left_ << ","
       << "\"d_front\":" << d_front_ << ","
       << "\"yaw_deg\":" << (current_yaw_ * 180.0 / M_PI) << ","
       << "\"target_yaw_deg\":" << paths_[current_path_idx_].drive_yaw_deg << ","
       << "\"vx\":" << last_vx_ << ","
       << "\"vy\":" << last_vy_ << ","
       << "\"wz\":" << last_wz_ << ","
       << "\"traffic_light\":\"" << tl_status << "\","
       << "\"elapsed_sec\":" << elapsed << ","
       << "\"traffic_cycle_rem\":" << cycle_rem << ","
       << "\"left_inliers\":" << left_line_.inlier_count << ","
       << "\"front_inliers\":" << front_line_.inlier_count << ","
       << "\"laps\":" << total_laps_completed_ << ","
       << "\"params\":{"
       << "\"cruise_speed\":" << cruise_speed_ << ","
       << "\"turn_speed\":" << turn_speed_ << ","
       << "\"min_turn_speed\":" << min_turn_speed_ << ","
       << "\"target_left_dist\":" << paths_[0].target_left_dist << ","
       << "\"front_trigger_dist\":" << paths_[0].front_trigger_dist << ","
       << "\"kp_yaw\":" << kp_yaw_ << ","
       << "\"kp_dist\":" << kp_dist_ << ","
       << "\"kp_turn\":" << kp_turn_ << ","
       << "\"turn_tolerance_deg\":" << turn_tolerance_deg_ << ","
       << "\"post_turn_dwell_time\":" << post_turn_dwell_time_ << ","
       << "\"align_tol_yaw_deg\":" << align_tol_yaw_deg_ << ","
       << "\"align_settle_time\":" << align_settle_time_ << ","
       << "\"align_tol_x\":" << align_tol_x_ << ","
       << "\"align_tol_y\":" << align_tol_y_ << ","
       << "\"corner_a_front_dist\":" << corners_[0].target_front_dist << ","
       << "\"corner_a_left_dist\":" << corners_[0].target_left_dist << ","
       << "\"corner_a_yaw\":" << corners_[0].target_yaw_deg << ","
       << "\"corner_b_front_dist\":" << corners_[1].target_front_dist << ","
       << "\"corner_b_left_dist\":" << corners_[1].target_left_dist << ","
       << "\"corner_b_yaw\":" << corners_[1].target_yaw_deg << ","
       << "\"corner_c_front_dist\":" << corners_[2].target_front_dist << ","
       << "\"corner_c_left_dist\":" << corners_[2].target_left_dist << ","
       << "\"corner_c_yaw\":" << corners_[2].target_yaw_deg << ","
       << "\"corner_d_front_dist\":" << corners_[3].target_front_dist << ","
       << "\"corner_d_left_dist\":" << corners_[3].target_left_dist << ","
       << "\"corner_d_yaw\":" << corners_[3].target_yaw_deg << ","
       << "\"slowdown_dist\":" << slowdown_dist_ << ","
       << "\"path_1_left_dist\":" << paths_[0].target_left_dist << ","
       << "\"path_1_front_dist\":" << paths_[0].front_trigger_dist << ","
       << "\"path_2_left_dist\":" << paths_[1].target_left_dist << ","
       << "\"path_2_front_dist\":" << paths_[1].front_trigger_dist << ","
       << "\"path_3_left_dist\":" << paths_[2].target_left_dist << ","
       << "\"path_3_front_dist\":" << paths_[2].front_trigger_dist << ","
       << "\"path_4_left_dist\":" << paths_[3].target_left_dist << ","
       << "\"path_4_front_dist\":" << paths_[3].front_trigger_dist << ","
       << "\"imu_yaw_offset_deg\":" << imu_yaw_offset_deg_ << ","
       << "\"path_1_yaw\":" << paths_[0].drive_yaw_deg << ","
       << "\"path_2_yaw\":" << paths_[1].drive_yaw_deg << ","
       << "\"path_3_yaw\":" << paths_[2].drive_yaw_deg << ","
       << "\"path_4_yaw\":" << paths_[3].drive_yaw_deg << ","
       << "\"enable_start_sit\":" << (enable_start_sit_ ? 1 : 0) << ","
       << "\"start_delay_sec\":" << start_delay_sec_ << ","
       << "\"sit_duration\":" << sit_duration_ << ","
       << "\"z_min\":" << z_min_ << ","
       << "\"z_max\":" << z_max_ << ","
       << "\"lidar_yaw_mount_deg\":" << lidar_yaw_mount_deg_ << ","
       << "\"enable_ring_filter\":" << (enable_ring_filter_ ? 1 : 0) << ","
       << "\"ring_min\":" << ring_min_ << ","
       << "\"ring_max\":" << ring_max_ << ","
       << "\"filter_alpha\":" << filter_alpha_
       << "}"
       << "}";

    std_msgs::msg::String status_msg;
    status_msg.data = ss.str();
    status_pub_->publish(status_msg);
  }

  // --- Member Variables ---
  std::mutex data_mutex_;

  double cruise_speed_{0.45};
  bool enable_slowdown_{false};
  double slowdown_dist_{1.50};
  double turn_speed_{0.50};
  double min_turn_speed_{0.15};
  double max_vy_{0.25};
  double max_align_vx_{0.20};
  double turn_tolerance_deg_{2.5};
  double turn_tolerance_rad_{0.043};
  double post_turn_dwell_time_{0.5};

  bool enable_start_sit_{false};
  double start_delay_sec_{0.0};
  double sit_duration_{3.0};
  double stand_duration_{2.0};
  double traffic_cycle_sec_{30.0};
  double traffic_safety_margin_sec_{5.0};

  double lidar_yaw_mount_deg_{90.0};
  double lidar_yaw_mount_rad_{M_PI / 2.0};
  double lidar_x_offset_{0.25};
  double lidar_y_offset_{0.00};
  double lidar_z_offset_{0.1588};

  double z_min_{-0.15};
  double z_max_{0.60};
  bool enable_ring_filter_{true};
  int ring_min_{7};
  int ring_max_{8};
  double filter_alpha_{0.40};
  int front_lost_count_{0};
  int left_lost_count_{0};
  int ransac_iterations_{80};
  double ransac_inlier_thresh_{0.035};
  int ransac_min_inliers_{20};
  bool prefer_outer_wall_{true};

  std::vector<PathConfig> paths_;
  std::vector<CornerConfig> corners_;

  double align_tol_x_{0.04};
  double align_tol_y_{0.04};
  double align_tol_yaw_deg_{2.0};
  double align_tol_yaw_rad_{0.035};
  double align_settle_time_{0.40};

  double kp_x_{0.80};
  double kp_dist_{0.80};
  double kp_yaw_{1.50};
  double kp_turn_{1.50};
  bool auto_start_{true};
  double sensor_timeout_{0.60};

  PatrolState state_{PatrolState::IDLE};
  std::string mission_log_{"🛠️ [대기/튜닝] 로봇 정지. 센서 계측 및 튜닝 대기 중"};
  int current_path_idx_{0};
  int total_laps_completed_{0};
  double raw_imu_yaw_{0.0};
  double imu_yaw_zero_{0.0};
  bool imu_zeroed_{false};
  double imu_yaw_offset_deg_{0.0};
  double imu_yaw_offset_rad_{0.0};
  double current_yaw_{0.0};
  double target_yaw_{0.0};

  Line2D front_line_;
  Line2D left_line_;
  float d_front_{99.0f};
  float d_left_{-1.0f};

  double last_vx_{0.0};
  double last_vy_{0.0};
  double last_wz_{0.0};

  bool imu_received_{false};
  bool cloud_received_{false};
  bool align_settling_{false};
  bool align_only_{false};
  bool global_timer_started_{false};
  bool external_traffic_override_{false};
  bool external_traffic_green_{false};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dwell_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_settle_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sit_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stand_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr posture_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr robot_motion_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_front_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::string input_source_{"scan"};
  std::string scan_topic_{"/scan"};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_sub_str_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr traffic_sub_bool_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WallRunnerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
