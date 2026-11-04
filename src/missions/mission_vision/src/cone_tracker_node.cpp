/**
 * @file cone_tracker_node.cpp
 * @brief Cone localisation: YOLO boxes + LiDAR (preferred) / depth (fallback) -> world-frame tracks.
 *
 * Per detection (from the teammate's YOLO node: std_msgs/String JSON by default - the robot PC runs Foxy,
 * whose vision_msgs differs from Jazzy's - or vision_msgs/Detection2DArray, detections_format):
 *   1. Coarse: depth pixels re-projected into the colour image (no aligned depth needed), nearest
 *      fraction inside the box -> rough cone position in base_link. Without depth: the ray through
 *      the box bottom-centre pixel intersected with the floor plane.
 *   2. Precise: every LiDAR scan is searched for cone-shaped clusters on its own (cone_lidar.hpp:
 *      small, in the cone height band, nothing above). The camera only says WHICH cluster is a
 *      cone: the nearest one within search_radius (+ search_radius_per_m * range) of the depth
 *      estimate, or - floor-ray estimate, its range depends on the camera pitch - the one closest
 *      in bearing (bearing_tol_deg). Camera pitch changes / jitter therefore do not move the
 *      result, and cones close to each other stay apart (clusters, not a centroid of a radius).
 *      No cluster -> the coarse position is used, with a lower tracking weight.
 *   3. base_link -> world_frame (AMCL map) through TF at the image stamp.
 *   4. Tracking: nearest track within assoc_dist. LiDAR-refined measurements dominate the position;
 *      a cone is confirmed after confirm_hits detections of which >= confirm_lidar_hits are refined.
 *   5. Confirmed cones keep being updated by LiDAR clusters alone (the camera sees the floor only
 *      to ~2.2 m, the XT16 sees cones to ~4 m), and are saved to output.save_path.
 * Outputs (world frame): /mission/cones (confirmed, PoseArray), /mission/cone_obstacles
 * (PointCloud2 ring per cone for the planner grid), /mission/cone_markers.
 *
 * Sensor sync (the camera runs on the robot PC whose clock is not synced - stamps in 1970 - while
 * the LiDAR and TF are on the NUC clock):
 *   - every stream's stamps are mapped to this node's clock by the smallest (receive - stamp) seen
 *     (StampMap); a stream already on this clock (|offset| < 0.5 s) is left alone. Detections carry
 *     the colour image stamp, so they use the colour stream's offset (camera_info), not their own
 *     receive time (which includes the YOLO delay).
 *   - depth frames and LiDAR scans are buffered (sync.buffer_s) and the ones closest to the image
 *     time are used (at most sync.max_dt apart), then moved to the image time with the odometry TF
 *     (odom_frame) - the robot keeps turning while YOLO runs.
 *   - the world TF is looked up at the image time; a fallback to the latest TF is allowed only if it
 *     is younger than sync.max_tf_age.
 *
 * mode (cones = camera-confirmed  U  LiDAR-only-confirmed, duplicates within 0.3 m merged):
 *   fusion      camera required: only the camera says what is a cone (no LiDAR-only cones)
 *   auto        (default) fusion while YOLO messages arrive; LiDAR-only cones inside lidar_only.regions
 *               always, and everywhere (or still only the regions, if set) when no YOLO message came
 *               for vision_timeout_s - the camera / Jetson can die without losing the cones
 *   lidar_only  no camera at all
 * LiDAR-only: a LiDAR cone cluster becomes a cone when it is seen at the same map
 * spot in >= 70 % of >= 20 scans (cone_lidar::PersistTracker), inside the map's free space and away
 * from its walls (lidar_only.wall_clearance), optionally only inside lidar_only.regions (e.g. the
 * slalom straights). Synthetic: 200 runs, 0 floor bits confirmed, 0 cones missed, ~1 s to confirm.
 *
 * Extrinsics come from config/camera_extrinsics.yaml (CAD mount + RealSense factory depth->colour).
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include "mission_vision/cone_lidar.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{
struct Pinhole
{
  double fx{0}, fy{0}, cx{0}, cy{0};
  int w{0}, h{0};
  bool valid() const {return fx > 0 && fy > 0;}
};

/// One camera box (pixels) with its score.
struct Box
{
  double cu, cv, sx, sy, score;
};

/// {"stamp":[sec,nsec],...,"cones":[[x1,y1,x2,y2,conf],...]} from the YOLO node. Only these two
/// keys are read; false when the stamp is missing.
bool parseConeJson(const std::string & s, int64_t & sec, int64_t & nsec, std::vector<Box> & boxes)
{
  auto numbers = [&](size_t from, size_t to) {
      std::vector<double> v;
      const char * p = s.c_str() + from;
      const char * end = s.c_str() + to;
      while (p < end) {
        if ((*p >= '0' && *p <= '9') || *p == '-' || *p == '.') {
          char * q;
          v.push_back(std::strtod(p, &q));
          p = q;
        } else {
          ++p;
        }
      }
      return v;
    };
  const size_t ks = s.find("\"stamp\"");
  if (ks == std::string::npos) return false;
  const size_t a = s.find('[', ks), b = s.find(']', ks);
  if (a == std::string::npos || b == std::string::npos) return false;
  const auto st = numbers(a, b);
  if (st.size() < 2) return false;
  sec = static_cast<int64_t>(st[0]);
  nsec = static_cast<int64_t>(st[1]);
  boxes.clear();
  const size_t kc = s.find("\"cones\"");
  if (kc == std::string::npos) return true;
  size_t p = s.find('[', kc) + 1;          // outer list
  while (true) {
    const size_t o = s.find('[', p), c = s.find(']', p);
    if (o == std::string::npos || c == std::string::npos || c < o) break;   // the outer ']' comes first
    const size_t ce = s.find(']', o);
    const auto v = numbers(o + 1, ce);
    if (v.size() >= 5) boxes.push_back({0.5 * (v[0] + v[2]), 0.5 * (v[1] + v[3]), v[2] - v[0], v[3] - v[1], v[4]});
    p = ce + 1;
  }
  return true;
}

struct Track
{
  Eigen::Vector2d pos;
  int hits{0};
  int lidar_hits{0};
  rclcpp::Time last_seen;
  bool confirmed{false};
  int id{0};
};

/// Maps a sensor's header stamps to this node's clock: offset = 5th percentile of (receive - stamp)
/// over the last 300 messages, i.e. the stamp plus a small transport delay. A percentile, not the
/// minimum: one sample taken before the clock started (sim time 0) or across a clock step would
/// otherwise poison the offset for the whole window.
struct StampMap
{
  std::deque<double> d;
  double offset{0.0};
  bool same_clock{true};
  int n{0};
  void add(double stamp, double recv)
  {
    if (recv <= 0.0 || stamp <= 0.0) return;            // clock not running yet
    d.push_back(recv - stamp);
    if (d.size() > 300) d.pop_front();
    std::vector<double> v(d.begin(), d.end());
    const size_t k = v.size() / 20;
    std::nth_element(v.begin(), v.begin() + k, v.end());
    offset = v[k];
    same_clock = std::abs(offset) < 0.5;
    ++n;
  }
  bool ready() const {return n > 0;}
  double map(double stamp) const {return same_clock ? stamp : stamp + offset;}
};

struct DepthFrame
{
  double t;
  sensor_msgs::msg::Image::SharedPtr img;
};

struct Scan
{
  double t;
  std::vector<cone_lidar::Candidate> cones;  // base_link at scan time
};

Eigen::Isometry3d poseFromParams(const std::vector<double> & xyz, const std::vector<double> & q_xyzw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (xyz.size() == 3) {T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);}
  if (q_xyzw.size() == 4) {
    T.linear() = Eigen::Quaterniond(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]).normalized().toRotationMatrix();
  }
  return T;
}
}  // namespace

class ConeTrackerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ConeTrackerNode()
  : rclcpp_lifecycle::LifecycleNode("cone_tracker_node")
  {
    det_topic_ = declare_parameter<std::string>("detections_topic", "/mission/cone_detections");
    det_json_topic_ = declare_parameter<std::string>("detections_json_topic", "/mission/cone_detections_json");
    det_format_ = declare_parameter<std::string>("detections_format", "string");   // string | vision_msgs
    color_info_topic_ = declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
    depth_info_topic_ = declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    sync_buffer_s_ = declare_parameter<double>("sync.buffer_s", 1.0);
    sync_max_dt_ = declare_parameter<double>("sync.max_dt", 0.05);
    sync_max_tf_age_ = declare_parameter<double>("sync.max_tf_age", 0.2);
    sync_motion_comp_ = declare_parameter<bool>("sync.motion_comp", true);

    // Extrinsics (camera_extrinsics.yaml overrides these)
    base_T_color_ = poseFromParams(
      declare_parameter<std::vector<double>>("base_to_color_optical.xyz", {0.30, 0.0, 0.117}),
      declare_parameter<std::vector<double>>("base_to_color_optical.q_xyzw", {-0.6187, 0.6187, -0.3423, 0.3423}));
    base_T_lidar_ = poseFromParams(
      declare_parameter<std::vector<double>>("base_to_lidar.xyz", {0.25, 0.0, 0.1588}),
      declare_parameter<std::vector<double>>("base_to_lidar.q_xyzw", {0.0024, 0.0063, 0.7071, 0.7070}));
    color_T_depth_ = poseFromParams(
      declare_parameter<std::vector<double>>("color_to_depth.xyz", {-0.0148, 0.0, 0.0}),
      declare_parameter<std::vector<double>>("color_to_depth.q_xyzw", {0.0, 0.0, 0.0, 1.0}));

    box_shrink_ = declare_parameter<double>("fusion.box_shrink", 0.2);
    near_fraction_ = declare_parameter<double>("fusion.near_fraction", 0.3);
    depth_stride_ = std::max<int>(1, declare_parameter<int>("fusion.depth_stride", 4));
    depth_min_ = declare_parameter<double>("fusion.depth_min", 0.25);
    depth_max_ = declare_parameter<double>("fusion.depth_max", 4.0);
    max_range_ = declare_parameter<double>("fusion.max_range", 4.0);
    min_score_ = declare_parameter<double>("fusion.min_score", 0.35);
    surface_to_center_ = declare_parameter<double>("fusion.surface_to_center", 0.05);
    lcfg_.surface_to_center = surface_to_center_;
    floor_z_ = declare_parameter<double>("fusion.floor_z", -0.30);
    refine_radius_ = declare_parameter<double>("refine.search_radius", 0.30);
    refine_z_min_ = declare_parameter<double>("refine.z_min", -0.27);
    refine_z_max_ = declare_parameter<double>("refine.z_max", -0.05);
    refine_min_pts_ = declare_parameter<int>("refine.min_points", 3);
    refine_radius_per_m_ = declare_parameter<double>("refine.search_radius_per_m", 0.05);
    bearing_tol_deg_ = declare_parameter<double>("refine.bearing_tol_deg", 6.0);
    lcfg_.z_min = refine_z_min_;
    lcfg_.z_max = refine_z_max_;
    lcfg_.min_points = refine_min_pts_;
    lcfg_.clear_gap = declare_parameter<double>("lidar_cone.clear_gap", 0.08);
    lcfg_.clear_height = declare_parameter<double>("lidar_cone.clear_height", 0.60);
    lcfg_.cluster_tol = declare_parameter<double>("lidar_cone.cluster_tol", 0.10);
    lcfg_.max_width = declare_parameter<double>("lidar_cone.max_width", 0.30);
    lcfg_.max_range = declare_parameter<double>("lidar_cone.max_range", 6.0);
    lcfg_.floor_gap = declare_parameter<double>("lidar_cone.floor_gap", 0.03);
    lcfg_.floor_radius = declare_parameter<double>("lidar_cone.floor_radius", 0.40);
    lidar_only_assoc_ = declare_parameter<double>("tracking.lidar_only_assoc", 0.15);
    alpha_lidar_only_ = declare_parameter<double>("tracking.alpha_lidar_only", 0.2);
    mode_ = declare_parameter<std::string>("mode", "auto");            // auto | fusion | lidar_only
    vision_timeout_s_ = declare_parameter<double>("vision_timeout_s", 3.0);
    if (mode_ != "auto" && mode_ != "fusion" && mode_ != "lidar_only") {
      RCLCPP_ERROR(get_logger(), "unknown mode '%s' - using auto", mode_.c_str());
      mode_ = "auto";
    }
    pcfg_.assoc = declare_parameter<double>("lidar_only.assoc", pcfg_.assoc);
    pcfg_.confirm_hits = declare_parameter<int>("lidar_only.confirm_hits", pcfg_.confirm_hits);
    pcfg_.confirm_window_s = declare_parameter<double>("lidar_only.confirm_window_s", pcfg_.confirm_window_s);
    pcfg_.max_spread = declare_parameter<double>("lidar_only.max_spread", pcfg_.max_spread);
    pcfg_.min_ratio = declare_parameter<double>("lidar_only.min_ratio", pcfg_.min_ratio);
    pcfg_.min_scans = declare_parameter<int>("lidar_only.min_scans", pcfg_.min_scans);
    pcfg_.tentative_timeout_s = declare_parameter<double>("lidar_only.tentative_timeout_s", pcfg_.tentative_timeout_s);
    pcfg_.alpha = alpha_lidar_only_;
    wall_clear_ = declare_parameter<double>("lidar_only.wall_clearance", 0.15);
    map_topic_ = declare_parameter<std::string>("lidar_only.map_topic", "/map");
    regions_ = declare_parameter<std::vector<double>>("lidar_only.regions", std::vector<double>{});
    if (regions_.size() % 4) {
      RCLCPP_ERROR(get_logger(), "lidar_only.regions must be x0 y0 x1 y1 per rectangle - ignored");
      regions_.clear();
    }
    ptracker_ = cone_lidar::PersistTracker(pcfg_);
    confirm_lidar_hits_ = declare_parameter<int>("tracking.confirm_lidar_hits", 2);
    alpha_lidar_ = declare_parameter<double>("tracking.alpha_lidar", 0.5);
    alpha_coarse_ = declare_parameter<double>("tracking.alpha_coarse", 0.1);
    save_path_ = declare_parameter<std::string>("output.save_path", "");
    load_saved_ = declare_parameter<bool>("output.load_saved", false);
    if (!save_path_.empty() && save_path_[0] == '~') {
      const char * home = getenv("HOME");
      if (home) save_path_ = std::string(home) + save_path_.substr(1);
    }

    assoc_dist_ = declare_parameter<double>("tracking.assoc_dist", 0.30);
    confirm_hits_ = declare_parameter<int>("tracking.confirm_hits", 3);
    tentative_timeout_ = declare_parameter<double>("tracking.tentative_timeout", 2.0);
    confirmed_timeout_ = declare_parameter<double>("tracking.confirmed_timeout", 30.0);
    obstacle_radius_ = declare_parameter<double>("output.obstacle_radius", 0.08);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cones_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/mission/cones", 10);
    obs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/mission/cone_obstacles", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mission/cone_markers", 10);
    if (load_saved_) loadSaved();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cones_pub_->on_activate();
    obs_pub_->on_activate();
    marker_pub_->on_activate();
    auto qos = rclcpp::SensorDataQoS();
    if (mode_ != "fusion") {
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {std::lock_guard<std::mutex> lk(mtx_); map_ = m;});
      if (regions_.empty()) {
        RCLCPP_WARN(get_logger(), "mode %s without lidar_only.regions: LiDAR-only cones %s - stair edges and low "
                    "objects can become cones; set the slalom rectangles", mode_.c_str(),
                    mode_ == "auto" ? "everywhere once YOLO stops" : "everywhere");
      }
    }
    if (mode_ == "lidar_only") {
      // no camera subscriptions
    } else if (det_format_ == "vision_msgs") {
      det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        det_topic_, 10, [this](const vision_msgs::msg::Detection2DArray::SharedPtr m) {
          std::vector<Box> boxes;
          for (const auto & d : m->detections) {
            boxes.push_back({d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y,
                             d.results.empty() ? 1.0 : d.results[0].hypothesis.score});
          }
          last_det_s_ = now().seconds();
          onDetections(m->header.stamp, boxes);
        });
    } else {
      det_json_sub_ = create_subscription<std_msgs::msg::String>(
        det_json_topic_, 10, [this](const std_msgs::msg::String::SharedPtr m) {
          int64_t sec, nsec;
          std::vector<Box> boxes;
          if (!parseConeJson(m->data, sec, nsec, boxes)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "bad cone JSON on %s", det_json_topic_.c_str());
            return;
          }
          builtin_interfaces::msg::Time st;
          st.sec = static_cast<int32_t>(sec);
          st.nanosec = static_cast<uint32_t>(nsec);
          last_det_s_ = now().seconds();
          onDetections(st, boxes);
        });
    }
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic_, qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
        color_K_ = toPinhole(*m);
        std::lock_guard<std::mutex> lk(mtx_);
        color_map_.add(rclcpp::Time(m->header.stamp).seconds(), now().seconds());
      });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic_, qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {depth_K_ = toPinhole(*m);});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos, [this](const sensor_msgs::msg::Image::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        depth_map_.add(rclcpp::Time(m->header.stamp).seconds(), now().seconds());
        const double t = depth_map_.map(rclcpp::Time(m->header.stamp).seconds());
        depth_buf_.push_back({t, m});
        while (!depth_buf_.empty() && depth_buf_.front().t < t - sync_buffer_s_) depth_buf_.pop_front();
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos, std::bind(&ConeTrackerNode::onCloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&ConeTrackerNode::publishTracks, this));
    RCLCPP_INFO(get_logger(), "cone_tracker active: %s + %s/%s -> %s frame",
                (det_format_ == "vision_msgs" ? det_topic_ : det_json_topic_).c_str(), cloud_topic_.c_str(),
                depth_topic_.c_str(), world_frame_.c_str());
    const Eigen::Quaterniond qc(base_T_color_.linear());
    const Eigen::Vector3d tc = base_T_color_.translation();
    RCLCPP_INFO(get_logger(), "camera extrinsic base->colour optical: xyz (%.3f, %.3f, %.3f) q_xyzw (%.4f, %.4f, %.4f, %.4f)",
                tc.x(), tc.y(), tc.z(), qc.x(), qc.y(), qc.z(), qc.w());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    saveConfirmed();
    det_sub_.reset(); det_json_sub_.reset(); map_sub_.reset(); color_info_sub_.reset(); depth_info_sub_.reset(); depth_sub_.reset(); cloud_sub_.reset();
    timer_.reset();
    std::lock_guard<std::mutex> lk(mtx_);
    depth_buf_.clear(); scan_buf_.clear();
    cones_pub_->on_deactivate(); obs_pub_->on_deactivate(); marker_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    cones_pub_.reset(); obs_pub_.reset(); marker_pub_.reset();
    tf_listener_.reset(); tf_buffer_.reset();
    tracks_.clear();
    return CallbackReturn::SUCCESS;
  }

private:
  static Pinhole toPinhole(const sensor_msgs::msg::CameraInfo & m)
  {
    Pinhole k;
    k.fx = m.k[0]; k.fy = m.k[4]; k.cx = m.k[2]; k.cy = m.k[5]; k.w = m.width; k.h = m.height;
    return k;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Keep LiDAR points in base_link (only those in front, within range)
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->width * msg->height / 4);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
      const Eigen::Vector3d p = base_T_lidar_ * Eigen::Vector3d(*ix, *iy, *iz);
      if (p.x() < 0.1 || p.head<2>().norm() > lcfg_.max_range + 0.5) continue;
      pts.push_back(p);
    }
    auto cones = cone_lidar::findCones(pts, base_T_lidar_.translation().head<2>(), lcfg_);
    std::lock_guard<std::mutex> lk(mtx_);
    ++stat_scans_;
    stat_cands_ += static_cast<int>(cones.size());
    stat_cands_max_ = std::max(stat_cands_max_, static_cast<int>(cones.size()));
    cloud_map_.add(rclcpp::Time(msg->header.stamp).seconds(), now().seconds());
    const double t = cloud_map_.map(rclcpp::Time(msg->header.stamp).seconds());
    scan_buf_.push_back({t, std::move(cones)});
    while (!scan_buf_.empty() && scan_buf_.front().t < t - sync_buffer_s_) scan_buf_.pop_front();
  }

  /// base_link(at t_img) <- base_link(at t): how the robot moved between t and the image time,
  /// from the odometry TF. Identity when disabled or unavailable (counted in the sync log).
  Eigen::Isometry3d motionComp(double t, double t_img)
  {
    if (!sync_motion_comp_ || std::abs(t - t_img) < 1e-4) return Eigen::Isometry3d::Identity();
    try {
      const auto a = tf_buffer_->lookupTransform(odom_frame_, base_frame_, rclcpp::Time(static_cast<int64_t>(t_img * 1e9), get_clock()->get_clock_type()));
      const auto b = tf_buffer_->lookupTransform(odom_frame_, base_frame_, rclcpp::Time(static_cast<int64_t>(t * 1e9), get_clock()->get_clock_type()));
      return toIso(a).inverse() * toIso(b);
    } catch (const tf2::TransformException &) {
      ++stat_comp_fail_;
      return Eigen::Isometry3d::Identity();
    }
  }

  static Eigen::Isometry3d toIso(const geometry_msgs::msg::TransformStamped & tf)
  {
    const auto & q = tf.transform.rotation;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    T.translation() = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    return T;
  }

  bool inBox(double u, double v, double u0, double v0, double u1, double v1) const
  {
    return u >= u0 && u <= u1 && v >= v0 && v <= v1;
  }

  /// Precise cone centre: the LiDAR cone cluster the camera points at (base_link, image time).
  /// by_bearing: the coarse range is unreliable (floor ray), match on direction from the camera.
  bool selectCluster(const Eigen::Vector3d & coarse, bool by_bearing, const std::vector<cone_lidar::Candidate> & cands,
                     const Eigen::Isometry3d & comp, Eigen::Vector3d & out) const
  {
    const Eigen::Vector2d cam = base_T_color_.translation().head<2>();
    const Eigen::Vector2d rc = coarse.head<2>() - cam;
    const double radius = refine_radius_ + refine_radius_per_m_ * coarse.head<2>().norm();
    double best = by_bearing ? bearing_tol_deg_ * M_PI / 180.0 : radius;
    bool found = false;
    for (const auto & cd : cands) {
      const Eigen::Vector2d c = (comp * Eigen::Vector3d(cd.center.x(), cd.center.y(), 0.0)).head<2>();
      double score;
      if (by_bearing) {
        const Eigen::Vector2d r = c - cam;
        const double ratio = r.norm() / std::max(rc.norm(), 1e-3);
        if (ratio < 0.5 || ratio > 2.0) continue;
        score = std::abs(std::atan2(rc.x() * r.y() - rc.y() * r.x(), rc.dot(r)));
      } else {
        score = (c - coarse.head<2>()).norm();
      }
      if (score <= best) {
        best = score;
        out = Eigen::Vector3d(c.x(), c.y(), floor_z_ + 0.115);
        found = true;
      }
    }
    return found;
  }

  // Coarse position without depth: ray through the box bottom-centre pixel meets the floor
  bool fromGroundRay(double u, double v, Eigen::Vector3d & out) const
  {
    const Eigen::Vector3d dir_c((u - color_K_.cx) / color_K_.fx, (v - color_K_.cy) / color_K_.fy, 1.0);
    const Eigen::Vector3d o = base_T_color_.translation();
    const Eigen::Vector3d d = base_T_color_.linear() * dir_c;
    if (d.z() > -1e-3) return false;  // ray does not go down to the floor
    const double s = (floor_z_ - o.z()) / d.z();
    out = o + s * d;
    return true;
  }

  // Cone centre from depth pixels re-projected into the colour image (no aligned depth needed)
  /// comp: base(image time) <- base(depth time); depth points are moved to the image time before
  /// they are projected into the colour image.
  bool fromDepth(double u0, double v0, double u1, double v1, const sensor_msgs::msg::Image::SharedPtr & d,
                 const Eigen::Isometry3d & comp, Eigen::Vector3d & out)
  {
    if (!d || !depth_K_.valid()) return false;
    const Eigen::Isometry3d color_T_depth_img = base_T_color_.inverse() * comp * base_T_color_ * color_T_depth_;
    const bool u16 = (d->encoding == "16UC1" || d->encoding == "mono16");
    const bool f32 = (d->encoding == "32FC1");
    if (!u16 && !f32) return false;
    std::vector<double> xs, ys, zs;
    for (uint32_t r = 0; r < d->height; r += depth_stride_) {
      for (uint32_t c = 0; c < d->width; c += depth_stride_) {
        double z;
        if (u16) {
          uint16_t mm;
          std::memcpy(&mm, &d->data[r * d->step + c * 2], 2);
          z = mm * 0.001;
        } else {
          float f;
          std::memcpy(&f, &d->data[r * d->step + c * 4], 4);
          z = f;
        }
        if (!(z >= depth_min_ && z <= depth_max_)) continue;
        const Eigen::Vector3d pd((c - depth_K_.cx) * z / depth_K_.fx, (r - depth_K_.cy) * z / depth_K_.fy, z);
        const Eigen::Vector3d pc = color_T_depth_img * pd;
        if (pc.z() < 0.1) continue;
        const double u = color_K_.fx * pc.x() / pc.z() + color_K_.cx;
        const double v = color_K_.fy * pc.y() / pc.z() + color_K_.cy;
        if (!inBox(u, v, u0, v0, u1, v1)) continue;
        xs.push_back(pc.x()); ys.push_back(pc.y()); zs.push_back(pc.z());
      }
    }
    if (zs.size() < 5) return false;
    // Nearest fraction by depth (cone in front of whatever is behind it), then median
    std::vector<size_t> idx(zs.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {return zs[a] < zs[b];});
    const size_t n = std::max<size_t>(5, static_cast<size_t>(near_fraction_ * idx.size()));
    std::vector<double> sx, sy, sz;
    for (size_t i = 0; i < n && i < idx.size(); ++i) {sx.push_back(xs[idx[i]]); sy.push_back(ys[idx[i]]); sz.push_back(zs[idx[i]]);}
    auto med = [](std::vector<double> v) {std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end()); return v[v.size() / 2];};
    out = base_T_color_ * Eigen::Vector3d(med(sx), med(sy), med(sz));
    return true;
  }

  void onDetections(const builtin_interfaces::msg::Time & img_stamp, const std::vector<Box> & boxes)
  {
    if (!color_K_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", color_info_topic_.c_str());
      return;
    }
    // 1. image time on this node's clock (detections carry the colour image stamp)
    double t_img;
    sensor_msgs::msg::Image::SharedPtr depth;
    double t_depth = 0.0, t_scan = 0.0, t_latest_scan = NAN;
    const std::vector<cone_lidar::Candidate> * scan = nullptr;
    std::vector<cone_lidar::Candidate> scan_copy;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!color_map_.ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no %s yet: cannot map image stamps", color_info_topic_.c_str());
        return;
      }
      // colour and depth share the camera clock; the depth stream (30 Hz, small messages) gives the
      // steadier offset - the colour stream queues behind the saturated link
      const StampMap & cam = (depth_map_.ready() && !depth_map_.same_clock && !color_map_.same_clock &&
                              std::abs(depth_map_.offset - color_map_.offset) < 1.0) ? depth_map_ : color_map_;
      t_img = cam.map(rclcpp::Time(img_stamp).seconds());
      if (!scan_buf_.empty()) t_latest_scan = scan_buf_.back().t;
      // 2. depth frame and LiDAR scan closest to the image time
      double best = sync_max_dt_;
      for (const auto & f : depth_buf_) {
        if (std::abs(f.t - t_img) <= best) {best = std::abs(f.t - t_img); depth = f.img; t_depth = f.t;}
      }
      best = sync_max_dt_;
      for (const auto & sc : scan_buf_) {
        if (std::abs(sc.t - t_img) <= best) {best = std::abs(sc.t - t_img); scan = &sc.cones; t_scan = sc.t;}
      }
      if (scan) scan_copy = *scan;
    }
    // 3. world pose of the robot at the image time
    geometry_msgs::msg::TransformStamped tf;
    const rclcpp::Time stamp(static_cast<int64_t>(t_img * 1e9), get_clock()->get_clock_type());
    try {
      tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s->%s: %s",
                             world_frame_.c_str(), base_frame_.c_str(), e.what());
        return;
      }
      const double age = t_img - rclcpp::Time(tf.header.stamp).seconds();
      if (std::abs(age) > sync_max_tf_age_) {
        ++stat_tf_old_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TF %s->%s at the image time missing, latest is %.2f s off: detections skipped",
                             world_frame_.c_str(), base_frame_.c_str(), age);
        return;
      }
    }
    const Eigen::Isometry3d world_T_base = toIso(tf);
    const Eigen::Isometry3d comp_depth = depth ? motionComp(t_depth, t_img) : Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d comp_scan = scan ? motionComp(t_scan, t_img) : Eigen::Isometry3d::Identity();
    // what "the latest scan" (the old behaviour) would have been off by, for the sync log
    const Eigen::Isometry3d comp_latest = std::isnan(t_latest_scan) ? Eigen::Isometry3d::Identity()
                                                                   : motionComp(t_latest_scan, t_img);
    noteSync(t_img, depth ? t_depth - t_img : NAN, scan ? t_scan - t_img : NAN, comp_scan, comp_latest);

    for (const auto & det : boxes) {
      if (det.score < min_score_) continue;
      const double cu = det.cu, cv = det.cv;
      const double hw = 0.5 * det.sx * (1.0 - box_shrink_), hh = 0.5 * det.sy * (1.0 - box_shrink_);
      // 1. Coarse: YOLO + depth (or the floor ray through the box bottom)
      Eigen::Vector3d coarse;
      const char * src = "depth";
      bool by_bearing = false;
      if (fromDepth(cu - hw, cv - hh, cu + hw, cv + hh, depth, comp_depth, coarse)) {
        const Eigen::Vector2d ray = coarse.head<2>() - base_T_color_.translation().head<2>();
        if (ray.norm() > 1e-3) coarse.head<2>() += surface_to_center_ * ray.normalized();
      } else {
        src = "floor-ray";
        by_bearing = true;
        if (!fromGroundRay(cu, cv + 0.5 * det.sy, coarse)) continue;
      }
      if (coarse.head<2>().norm() > max_range_) continue;
      // 2. Precise: LiDAR around the coarse position
      Eigen::Vector3d pb = coarse;
      const bool refined = scan && selectCluster(coarse, by_bearing, scan_copy, comp_scan, pb);
      const Eigen::Vector3d pw = world_T_base * pb;
      update(Eigen::Vector2d(pw.x(), pw.y()), stamp, refined);
      RCLCPP_DEBUG(get_logger(), "cone coarse(%s) (%.2f, %.2f) -> %s (%.2f, %.2f) base, world (%.2f, %.2f)",
                   src, coarse.x(), coarse.y(), refined ? "lidar" : "coarse", pb.x(), pb.y(), pw.x(), pw.y());
    }
  }

  /// Sync statistics, logged every 10 s: offsets, matched time differences, compensation sizes.
  void noteSync(double t_img, double dt_depth, double dt_scan, const Eigen::Isometry3d & comp,
                const Eigen::Isometry3d & comp_latest)
  {
    stat_latest_deg_ = std::max(stat_latest_deg_, Eigen::AngleAxisd(comp_latest.linear()).angle() * 180.0 / M_PI);
    stat_latest_cm_ = std::max(stat_latest_cm_, comp_latest.translation().norm() * 100.0);
    ++stat_n_;
    if (std::isnan(dt_depth)) ++stat_no_depth_; else stat_dt_depth_ = std::max(stat_dt_depth_, std::abs(dt_depth));
    if (std::isnan(dt_scan)) ++stat_no_scan_; else stat_dt_scan_ = std::max(stat_dt_scan_, std::abs(dt_scan));
    stat_comp_deg_ = std::max(stat_comp_deg_, Eigen::AngleAxisd(comp.linear()).angle() * 180.0 / M_PI);
    stat_comp_cm_ = std::max(stat_comp_cm_, comp.translation().norm() * 100.0);
    stat_lag_ = std::max(stat_lag_, now().seconds() - t_img);
    if (now().seconds() - stat_t0_ < 10.0) return;
    RCLCPP_INFO(get_logger(),
      "sync %d dets: clock offset colour %+.3f%s depth %+.3f%s lidar %+.3f%s s | max |dt| depth %.0f ms lidar %.0f ms, "
      "none: depth %d lidar %d | YOLO lag max %.0f ms, motion comp max %.2f deg %.1f cm (fail %d), old TF %d | "
      "latest scan instead would be off by max %.2f deg %.1f cm | LiDAR-only cone updates %d | "
      "LiDAR cone clusters per scan mean %.2f max %d (%d scans)",
      stat_n_, color_map_.offset, color_map_.same_clock ? "(same)" : "", depth_map_.offset,
      depth_map_.same_clock ? "(same)" : "", cloud_map_.offset, cloud_map_.same_clock ? "(same)" : "",
      stat_dt_depth_ * 1e3, stat_dt_scan_ * 1e3, stat_no_depth_, stat_no_scan_, stat_lag_ * 1e3,
      stat_comp_deg_, stat_comp_cm_, stat_comp_fail_, stat_tf_old_, stat_latest_deg_, stat_latest_cm_,
      stat_lidar_only_, stat_scans_ ? static_cast<double>(stat_cands_) / stat_scans_ : 0.0, stat_cands_max_, stat_scans_);
    stat_lidar_only_ = stat_scans_ = stat_cands_ = stat_cands_max_ = 0;
    stat_t0_ = now().seconds();
    stat_n_ = stat_no_depth_ = stat_no_scan_ = stat_comp_fail_ = stat_tf_old_ = 0;
    stat_dt_depth_ = stat_dt_scan_ = stat_comp_deg_ = stat_comp_cm_ = stat_lag_ = 0.0;
    stat_latest_deg_ = stat_latest_cm_ = 0.0;
    (void)t_img;
  }

  void update(const Eigen::Vector2d & p, const rclcpp::Time & t, bool refined)
  {
    Track * best = nullptr;
    double best_d = assoc_dist_;
    for (auto & tr : tracks_) {
      const double d = (tr.pos - p).norm();
      if (d < best_d) {best_d = d; best = &tr;}
    }
    if (!best) {
      tracks_.push_back(Track{p, 1, refined ? 1 : 0, t, false, next_id_++});
      best = &tracks_.back();
    } else {
      // LiDAR-refined measurements set the position; coarse ones only nudge it until a LiDAR
      // measurement exists, then they no longer move it.
      if (refined) {
        best->pos = best->lidar_hits == 0 ? p : (1.0 - alpha_lidar_) * best->pos + alpha_lidar_ * p;
        best->lidar_hits++;
      } else if (best->lidar_hits == 0) {
        best->pos = (1.0 - alpha_coarse_) * best->pos + alpha_coarse_ * p;
      }
      best->hits++;
      best->last_seen = t;
    }
    if (!best->confirmed && best->hits >= confirm_hits_ && best->lidar_hits >= confirm_lidar_hits_) {
      best->confirmed = true;
      RCLCPP_INFO(get_logger(), "cone #%d confirmed at (%.3f, %.3f) in %s (%d detections, %d LiDAR)",
                  best->id, best->pos.x(), best->pos.y(), world_frame_.c_str(), best->hits, best->lidar_hits);
      saveConfirmed();
    }
  }

  void saveConfirmed()
  {
    if (save_path_.empty()) return;
    std::ofstream f(save_path_);
    if (!f) return;
    f << "# cone_tracker_node confirmed cones (" << world_frame_ << " frame; id >= 10000: LiDAR-only)\nframe: "
      << world_frame_ << "\ncones:\n";
    for (const auto & tr : outputCones()) {
      f << "  - {id: " << tr.id << ", x: " << tr.pos.x() << ", y: " << tr.pos.y()
        << ", detections: " << tr.hits << ", lidar: " << tr.lidar_hits << "}\n";
    }
  }

  void loadSaved()
  {
    std::ifstream f(save_path_);
    std::string line;
    int n = 0;
    while (std::getline(f, line)) {
      int id, hits, lh;
      double x, y;
      if (std::sscanf(line.c_str(), "  - {id: %d, x: %lf, y: %lf, detections: %d, lidar: %d}", &id, &x, &y, &hits, &lh) == 5) {
        Track tr{Eigen::Vector2d(x, y), hits, lh, now(), true, id};
        if (id >= 10000) {
          ptracker_.addConfirmed(id - 10000, Eigen::Vector2d(x, y));
        } else {
          tracks_.push_back(tr);
          next_id_ = std::max(next_id_, id + 1);
        }
        ++n;
      }
    }
    if (n) RCLCPP_INFO(get_logger(), "loaded %d saved cones from %s", n, save_path_.c_str());
  }

  /// Confirmed cones are kept up to date by LiDAR clusters alone (the newest scan that is at least
  /// 0.1 s old, so the TF for its time has arrived).
  void lidarOnlyUpdate()
  {
    Scan sc;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const double t_now = now().seconds();
      for (auto it = scan_buf_.rbegin(); it != scan_buf_.rend(); ++it) {
        if (t_now - it->t >= 0.1) {sc = *it; break;}
      }
    }
    if (sc.cones.empty() || sc.t <= last_lidar_only_t_) return;
    last_lidar_only_t_ = sc.t;
    Eigen::Isometry3d world_T_base;
    try {
      world_T_base = toIso(tf_buffer_->lookupTransform(world_frame_, base_frame_,
        rclcpp::Time(static_cast<int64_t>(sc.t * 1e9), get_clock()->get_clock_type())));
    } catch (const tf2::TransformException &) {
      return;
    }
    const rclcpp::Time stamp(static_cast<int64_t>(sc.t * 1e9), get_clock()->get_clock_type());
    for (const auto & cd : sc.cones) {
      const Eigen::Vector2d w = (world_T_base * Eigen::Vector3d(cd.center.x(), cd.center.y(), 0.0)).head<2>();
      Track * best = nullptr;
      double best_d = lidar_only_assoc_;   // tight: other clusters must not drag a cone around
      for (auto & tr : tracks_) {
        const double d = (tr.pos - w).norm();
        if (tr.confirmed && d < best_d) {best_d = d; best = &tr;}
      }
      if (!best) continue;
      best->pos = (1.0 - alpha_lidar_only_) * best->pos + alpha_lidar_only_ * w;
      best->last_seen = stamp;
      ++stat_lidar_only_;
    }
  }

  /// mode lidar_only: every buffered scan at least 0.1 s old (its TF has arrived) -> world frame ->
  /// gates (regions, map free space + wall clearance) -> persistence tracker -> tracks_.
  bool gate(const Eigen::Vector2d & w, const nav_msgs::msg::OccupancyGrid::SharedPtr & m, int & why)
  {
    if (!regions_.empty()) {
      bool in = false;
      for (size_t i = 0; i + 3 < regions_.size(); i += 4) {
        in |= w.x() >= std::min(regions_[i], regions_[i + 2]) && w.x() <= std::max(regions_[i], regions_[i + 2]) &&
              w.y() >= std::min(regions_[i + 1], regions_[i + 3]) && w.y() <= std::max(regions_[i + 1], regions_[i + 3]);
      }
      if (!in) {why = 0; return false;}
    }
    if (!m) return true;
    const double res = m->info.resolution;
    const int W = static_cast<int>(m->info.width), H = static_cast<int>(m->info.height);
    const int cx = static_cast<int>(std::floor((w.x() - m->info.origin.position.x) / res));
    const int cy = static_cast<int>(std::floor((w.y() - m->info.origin.position.y) / res));
    if (cx < 0 || cy < 0 || cx >= W || cy >= H) {why = 1; return false;}
    const int8_t v = m->data[cy * W + cx];
    if (v < 0 || v > 25) {why = 1; return false;}                 // unknown or a wall: not the track
    const int r = static_cast<int>(std::ceil(wall_clear_ / res));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int x = cx + dx, y = cy + dy;
        if (x < 0 || y < 0 || x >= W || y >= H || (dx * dx + dy * dy) * res * res > wall_clear_ * wall_clear_) continue;
        if (m->data[y * W + x] > 65) {why = 2; return false;}      // wall foot / baseboard edge
      }
    }
    return true;
  }

  /// Is YOLO alive (a detection message - empty or not - within vision_timeout_s)?
  bool visionAlive()
  {
    return mode_ != "lidar_only" && last_det_s_ > 0 && now().seconds() - last_det_s_ < vision_timeout_s_;
  }

  void lidarOnlyStep()
  {
    // auto with YOLO alive: LiDAR-only cones only inside the regions (none if no regions)
    const bool alive = visionAlive();
    if (mode_ == "auto" && alive != vision_was_alive_) {
      if (alive) {
        RCLCPP_INFO(get_logger(), "vision: YOLO messages back - camera picks the cones%s",
                    regions_.empty() ? "" : " (LiDAR-only stays on inside the regions)");
      } else {
        RCLCPP_WARN(get_logger(), "vision: no YOLO message for %.1f s - LiDAR-only cones %s", vision_timeout_s_,
                    regions_.empty() ? "everywhere" : "inside the regions");
      }
      vision_was_alive_ = alive;
    }
    const bool restrict_to_regions = (mode_ == "auto" && alive) || !regions_.empty();
    if (restrict_to_regions && regions_.empty()) {   // auto, YOLO alive, no regions: nothing to do
      std::lock_guard<std::mutex> lk(mtx_);
      if (!scan_buf_.empty()) last_lidar_only_t_ = scan_buf_.back().t;
      return;
    }
    std::vector<Scan> todo;
    nav_msgs::msg::OccupancyGrid::SharedPtr m;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const double t_now = now().seconds();
      for (const auto & sc : scan_buf_) {
        if (sc.t > last_lidar_only_t_ && t_now - sc.t >= 0.1) todo.push_back(sc);
      }
      m = map_;
    }
    if (!m && !warned_no_map_) {
      RCLCPP_WARN(get_logger(), "lidar_only: no %s yet - map gating off until it arrives", map_topic_.c_str());
      warned_no_map_ = true;
    }
    for (const auto & sc : todo) {
      last_lidar_only_t_ = sc.t;
      Eigen::Isometry3d world_T_base;
      try {
        world_T_base = toIso(tf_buffer_->lookupTransform(world_frame_, base_frame_,
          rclcpp::Time(static_cast<int64_t>(sc.t * 1e9), get_clock()->get_clock_type())));
      } catch (const tf2::TransformException &) {
        ++lo_no_tf_;
        continue;
      }
      std::vector<Eigen::Vector2d> pts;
      for (const auto & cd : sc.cones) {
        const Eigen::Vector2d w = (world_T_base * Eigen::Vector3d(cd.center.x(), cd.center.y(), 0.0)).head<2>();
        int why = -1;
        if (gate(w, m, why)) pts.push_back(w); else ++lo_gated_[why];
      }
      lo_cands_ += sc.cones.size();
      ++lo_scans_;
      for (int id : ptracker_.update(sc.t, pts)) {
        for (const auto & tr : ptracker_.tracks()) {
          if (tr.id == id) {
            RCLCPP_INFO(get_logger(), "cone #%d confirmed (LiDAR only) at (%.3f, %.3f) in %s", id, tr.pos.x(), tr.pos.y(),
                        world_frame_.c_str());
          }
        }
      }
    }
    if (ptracker_.confirmedCount() != lo_saved_count_) {
      lo_saved_count_ = ptracker_.confirmedCount();
      saveConfirmed();
    }
    if (now().seconds() - lo_t0_ >= 10.0) {
      size_t tentative = ptracker_.tracks().size() - ptracker_.confirmedCount();
      RCLCPP_INFO(get_logger(), "lidar_only %d scans: %.2f clusters/scan, gated out: region %d map %d wall %d, no TF %d | "
                  "tentative %zu, confirmed %zu | vision %s", lo_scans_, lo_scans_ ? double(lo_cands_) / lo_scans_ : 0.0,
                  lo_gated_[0], lo_gated_[1], lo_gated_[2], lo_no_tf_, tentative, ptracker_.confirmedCount(),
                  mode_ == "lidar_only" ? "off (lidar_only)" : (alive ? "alive" : "LOST"));
      lo_t0_ = now().seconds();
      lo_scans_ = lo_no_tf_ = 0;
      lo_cands_ = 0;
      lo_gated_[0] = lo_gated_[1] = lo_gated_[2] = 0;
    }
  }

  /// Published / saved cones: camera-confirmed tracks + LiDAR-only ones not within 0.3 m of those.
  std::vector<Track> outputCones()
  {
    std::vector<Track> out;
    for (const auto & tr : tracks_) if (tr.confirmed) out.push_back(tr);
    const size_t n_cam = out.size();
    for (const auto & pt : ptracker_.tracks()) {
      if (!pt.confirmed) continue;
      bool dup = false;
      for (size_t i = 0; i < n_cam; ++i) dup |= (out[i].pos - pt.pos).norm() < 0.3;
      if (!dup) out.push_back(Track{pt.pos, pt.hits, pt.hits, now(), true, 10000 + pt.id});
    }
    return out;
  }

  void publishTracks()
  {
    if (mode_ != "lidar_only") lidarOnlyUpdate();   // camera-confirmed cones keep following the LiDAR
    if (mode_ != "fusion") lidarOnlyStep();
    const rclcpp::Time now_t = now();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
      const double age = (now_t - tr.last_seen).seconds();
      return tr.confirmed ? (confirmed_timeout_ > 0.0 && age > confirmed_timeout_) : age > tentative_timeout_;
    }), tracks_.end());

    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = now_t;
    pa.header.frame_id = world_frame_;
    visualization_msgs::msg::MarkerArray ma;
    std::vector<Eigen::Vector2d> ring;
    for (const auto & tr : outputCones()) {
      geometry_msgs::msg::Pose p;
      p.position.x = tr.pos.x(); p.position.y = tr.pos.y(); p.orientation.w = 1.0;
      pa.poses.push_back(p);
      for (int k = 0; k < 12; ++k) {
        const double a = k * M_PI / 6.0;
        ring.emplace_back(tr.pos.x() + obstacle_radius_ * std::cos(a), tr.pos.y() + obstacle_radius_ * std::sin(a));
      }
      visualization_msgs::msg::Marker m;
      m.header = pa.header; m.ns = "cones"; m.id = tr.id; m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.pose = p; m.pose.position.z = 0.115; m.scale.x = m.scale.y = 0.14; m.scale.z = 0.23;
      m.color.r = 1.0; m.color.g = 0.5; m.color.a = 0.9; m.lifetime = rclcpp::Duration::from_seconds(0.5);
      ma.markers.push_back(m);
    }
    cones_pub_->publish(pa);
    if (!ma.markers.empty()) marker_pub_->publish(ma);

    sensor_msgs::msg::PointCloud2 pc;
    pc.header = pa.header;
    sensor_msgs::PointCloud2Modifier mod(pc);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(ring.size());
    sensor_msgs::PointCloud2Iterator<float> ox(pc, "x"), oy(pc, "y"), oz(pc, "z");
    for (const auto & r : ring) {*ox = r.x(); *oy = r.y(); *oz = 0.1f; ++ox; ++oy; ++oz;}
    obs_pub_->publish(pc);
  }

  // Parameters
  std::string det_json_topic_, det_format_;
  std::string det_topic_, color_info_topic_, depth_topic_, depth_info_topic_, cloud_topic_, base_frame_, world_frame_;
  std::string odom_frame_;
  double sync_buffer_s_, sync_max_dt_, sync_max_tf_age_;
  bool sync_motion_comp_;
  Eigen::Isometry3d base_T_color_, base_T_lidar_, color_T_depth_;
  double box_shrink_, near_fraction_, depth_min_, depth_max_, max_range_, min_score_, surface_to_center_;
  double floor_z_, refine_radius_, refine_z_min_, refine_z_max_, alpha_lidar_, alpha_coarse_;
  int refine_min_pts_, confirm_lidar_hits_;
  double refine_radius_per_m_, bearing_tol_deg_, alpha_lidar_only_, lidar_only_assoc_;
  std::string mode_, map_topic_;
  cone_lidar::PersistConfig pcfg_;
  cone_lidar::PersistTracker ptracker_;
  double wall_clear_{0.15};
  std::vector<double> regions_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  bool warned_no_map_{false};
  double vision_timeout_s_{3.0}, last_det_s_{-1};
  bool vision_was_alive_{false};
  size_t lo_saved_count_{0};
  int lo_scans_{0}, lo_no_tf_{0}, lo_gated_[3]{0, 0, 0};
  size_t lo_cands_{0};
  double lo_t0_{0};
  cone_lidar::Config lcfg_;
  std::string save_path_;
  bool load_saved_;
  int depth_stride_;
  double assoc_dist_, tentative_timeout_, confirmed_timeout_, obstacle_radius_;
  int confirm_hits_;

  // State
  std::mutex mtx_;
  Pinhole color_K_, depth_K_;
  StampMap color_map_, depth_map_, cloud_map_;
  std::deque<DepthFrame> depth_buf_;
  std::deque<Scan> scan_buf_;
  int stat_n_{0}, stat_no_depth_{0}, stat_no_scan_{0}, stat_comp_fail_{0}, stat_tf_old_{0};
  double stat_latest_deg_{0}, stat_latest_cm_{0}, stat_dt_depth_{0}, stat_dt_scan_{0}, stat_comp_deg_{0}, stat_comp_cm_{0}, stat_lag_{0}, stat_t0_{0};
  std::vector<Track> tracks_;
  int next_id_{0};
  double last_lidar_only_t_{0.0};
  int stat_lidar_only_{0}, stat_scans_{0}, stat_cands_{0}, stat_cands_max_{0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr det_json_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_, depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr cones_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeTrackerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
