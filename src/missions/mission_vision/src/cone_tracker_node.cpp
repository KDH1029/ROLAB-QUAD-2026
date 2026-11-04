/**
 * @file cone_tracker_node.cpp
 * @brief Cone localisation from the LiDAR alone -> world-frame (AMCL map) cone positions.
 *
 *   1. Every XT16 scan is searched for cone-shaped clusters (cone_lidar::findCones: tall columns of walls /
 *      legs / boxes dropped first, then small clusters in the cone height band with nothing right above,
 *      standing clear of the floor).
 *   2. Each candidate goes to the world frame through TF at the scan time, then the gates: inside
 *      lidar_only.regions (the slalom straights), on free map space and lidar_only.wall_clearance away from
 *      map walls.
 *   3. cone_lidar::PersistTracker: a candidate becomes a cone when it is seen at the same map spot in
 *      >= min_ratio of >= min_scans scans. Confirmed cones are kept for the whole run (cones are static; a cone
 *      the robot is too close to - under the lowest ring within ~0.8 m - or that is hidden stays in the list)
 *      and saved to output.save_path.
 * Outputs (world frame, 5 Hz): /mission/cones (PoseArray, ego_planner cones.*), /mission/cone_obstacles
 * (PointCloud2 ring per cone), /mission/cone_markers.
 *
 * Camera / YOLO fusion was removed on 11-05 (vision is the teammate's). 145915 replay: 10/10 cones within
 * 4-14 cm, no fake cone; arena4 (cones moved): the 10 placed.
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{
/// Maps a sensor's header stamps to this node's clock: offset = 5th percentile of (receive - stamp)
/// over the last 300 messages. A stream already on this clock (|offset| < 0.5 s) is left alone.
struct StampMap
{
  std::deque<double> d;
  double offset{0.0};
  bool same_clock{true};
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
  }
  double map(double stamp) const {return same_clock ? stamp : stamp + offset;}
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

Eigen::Isometry3d toIso(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & q = tf.transform.rotation;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  T.translation() = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  return T;
}
}  // namespace

class ConeTrackerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ConeTrackerNode()
  : rclcpp_lifecycle::LifecycleNode("cone_tracker_node")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    buffer_s_ = declare_parameter<double>("sync.buffer_s", 1.0);
    base_T_lidar_ = poseFromParams(
      declare_parameter<std::vector<double>>("base_to_lidar.xyz", {0.25, 0.0, 0.1588}),
      declare_parameter<std::vector<double>>("base_to_lidar.q_xyzw", {0.0024, 0.0063, 0.7071, 0.7070}));

    lcfg_.z_min = declare_parameter<double>("lidar_cone.z_min", -0.306);
    lcfg_.z_max = declare_parameter<double>("lidar_cone.z_max", -0.086);
    lcfg_.min_points = declare_parameter<int>("lidar_cone.min_points", 3);
    lcfg_.surface_to_center = declare_parameter<double>("lidar_cone.surface_to_center", 0.05);
    lcfg_.clear_gap = declare_parameter<double>("lidar_cone.clear_gap", 0.08);
    lcfg_.clear_height = declare_parameter<double>("lidar_cone.clear_height", 0.60);
    lcfg_.cluster_tol = declare_parameter<double>("lidar_cone.cluster_tol", 0.10);
    lcfg_.max_width = declare_parameter<double>("lidar_cone.max_width", 0.30);
    lcfg_.max_range = declare_parameter<double>("lidar_cone.max_range", 6.0);
    lcfg_.floor_gap = declare_parameter<double>("lidar_cone.floor_gap", 0.03);
    lcfg_.floor_radius = declare_parameter<double>("lidar_cone.floor_radius", 0.40);
    lcfg_.tall_cell = declare_parameter<double>("lidar_cone.tall_cell", 0.04);
    lcfg_.tall_dilate = declare_parameter<int>("lidar_cone.tall_dilate", 1);
    lcfg_.fine_cover = declare_parameter<bool>("lidar_cone.fine_cover", false);

    pcfg_.assoc = declare_parameter<double>("lidar_only.assoc", pcfg_.assoc);
    pcfg_.confirm_hits = declare_parameter<int>("lidar_only.confirm_hits", pcfg_.confirm_hits);
    pcfg_.confirm_window_s = declare_parameter<double>("lidar_only.confirm_window_s", pcfg_.confirm_window_s);
    pcfg_.max_spread = declare_parameter<double>("lidar_only.max_spread", pcfg_.max_spread);
    pcfg_.min_ratio = declare_parameter<double>("lidar_only.min_ratio", pcfg_.min_ratio);
    pcfg_.min_scans = declare_parameter<int>("lidar_only.min_scans", pcfg_.min_scans);
    pcfg_.tentative_timeout_s = declare_parameter<double>("lidar_only.tentative_timeout_s", pcfg_.tentative_timeout_s);
    pcfg_.alpha = declare_parameter<double>("tracking.alpha_lidar_only", 0.2);
    ptracker_ = cone_lidar::PersistTracker(pcfg_);
    wall_clear_ = declare_parameter<double>("lidar_only.wall_clearance", 0.0);
    map_topic_ = declare_parameter<std::string>("lidar_only.map_topic", "/map");
    regions_ = declare_parameter<std::vector<double>>("lidar_only.regions", std::vector<double>{});
    if (regions_.size() % 4) {
      RCLCPP_ERROR(get_logger(), "lidar_only.regions must be x0 y0 x1 y1 per rectangle - ignored");
      regions_.clear();
    }

    obstacle_radius_ = declare_parameter<double>("output.obstacle_radius", 0.08);
    save_path_ = declare_parameter<std::string>("output.save_path", "");
    load_saved_ = declare_parameter<bool>("output.load_saved", false);
    if (!save_path_.empty() && save_path_[0] == '~') {
      const char * home = getenv("HOME");
      if (home) save_path_ = std::string(home) + save_path_.substr(1);
    }
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
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {std::lock_guard<std::mutex> lk(mtx_); map_ = m;});
    if (regions_.empty()) {
      RCLCPP_WARN(get_logger(), "no lidar_only.regions: LiDAR cones everywhere - stair edges and low objects can "
                  "become cones; set the slalom rectangles");
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(), std::bind(&ConeTrackerNode::onCloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&ConeTrackerNode::publishTracks, this));
    RCLCPP_INFO(get_logger(), "cone_tracker active (LiDAR only): %s -> %s frame, %zu region(s)",
                cloud_topic_.c_str(), world_frame_.c_str(), regions_.size() / 4);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    saveConfirmed();
    map_sub_.reset(); cloud_sub_.reset(); timer_.reset();
    std::lock_guard<std::mutex> lk(mtx_);
    scan_buf_.clear();
    cones_pub_->on_deactivate(); obs_pub_->on_deactivate(); marker_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    cones_pub_.reset(); obs_pub_.reset(); marker_pub_.reset();
    tf_listener_.reset(); tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // LiDAR points in base_link (only those in front, within range)
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
    cloud_map_.add(rclcpp::Time(msg->header.stamp).seconds(), now().seconds());
    const double t = cloud_map_.map(rclcpp::Time(msg->header.stamp).seconds());
    scan_buf_.push_back({t, std::move(cones)});
    while (!scan_buf_.empty() && scan_buf_.front().t < t - buffer_s_) scan_buf_.pop_front();
  }

  /// regions, then map free space + wall clearance. why: 0 region, 1 map (unknown / wall cell), 2 wall.
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

  /// every buffered scan at least 0.1 s old (its TF has arrived) -> world frame -> gates -> persistence tracker
  void step()
  {
    std::vector<Scan> todo;
    nav_msgs::msg::OccupancyGrid::SharedPtr m;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const double t_now = now().seconds();
      for (const auto & sc : scan_buf_) {
        if (sc.t > last_t_ && t_now - sc.t >= 0.1) todo.push_back(sc);
      }
      m = map_;
    }
    if (!m && !warned_no_map_) {
      RCLCPP_WARN(get_logger(), "no %s yet - map gating off until it arrives", map_topic_.c_str());
      warned_no_map_ = true;
    }
    for (const auto & sc : todo) {
      last_t_ = sc.t;
      Eigen::Isometry3d world_T_base;
      try {
        world_T_base = toIso(tf_buffer_->lookupTransform(world_frame_, base_frame_,
          rclcpp::Time(static_cast<int64_t>(sc.t * 1e9), get_clock()->get_clock_type())));
      } catch (const tf2::TransformException &) {
        ++st_no_tf_;
        continue;
      }
      std::vector<Eigen::Vector2d> pts;
      for (const auto & cd : sc.cones) {
        const Eigen::Vector2d w = (world_T_base * Eigen::Vector3d(cd.center.x(), cd.center.y(), 0.0)).head<2>();
        int why = -1;
        if (gate(w, m, why)) pts.push_back(w); else ++st_gated_[why];
      }
      st_cands_ += sc.cones.size();
      ++st_scans_;
      for (int id : ptracker_.update(sc.t, pts)) {
        for (const auto & tr : ptracker_.tracks()) {
          if (tr.id == id) {
            RCLCPP_INFO(get_logger(), "cone #%d confirmed at (%.3f, %.3f) in %s", id, tr.pos.x(), tr.pos.y(),
                        world_frame_.c_str());
          }
        }
      }
    }
    if (ptracker_.confirmedCount() != saved_count_) {
      saved_count_ = ptracker_.confirmedCount();
      saveConfirmed();
    }
    if (now().seconds() - st_t0_ >= 10.0) {
      const size_t tentative = ptracker_.tracks().size() - ptracker_.confirmedCount();
      RCLCPP_INFO(get_logger(), "%d scans: %.2f clusters/scan, gated out: region %d map %d wall %d, no TF %d | "
                  "tentative %zu, confirmed %zu", st_scans_, st_scans_ ? double(st_cands_) / st_scans_ : 0.0,
                  st_gated_[0], st_gated_[1], st_gated_[2], st_no_tf_, tentative, ptracker_.confirmedCount());
      st_t0_ = now().seconds();
      st_scans_ = st_no_tf_ = 0;
      st_cands_ = 0;
      st_gated_[0] = st_gated_[1] = st_gated_[2] = 0;
    }
  }

  void saveConfirmed()
  {
    if (save_path_.empty()) return;
    std::ofstream f(save_path_);
    if (!f) return;
    f << "# cone_tracker_node confirmed cones (" << world_frame_ << " frame, LiDAR only)\nframe: "
      << world_frame_ << "\ncones:\n";
    for (const auto & tr : ptracker_.tracks()) {
      if (!tr.confirmed) continue;
      f << "  - {id: " << 10000 + tr.id << ", x: " << tr.pos.x() << ", y: " << tr.pos.y()
        << ", detections: " << tr.hits << ", lidar: " << tr.hits << "}\n";
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
        ptracker_.addConfirmed(id >= 10000 ? id - 10000 : id, Eigen::Vector2d(x, y));
        ++n;
      }
    }
    if (n) RCLCPP_INFO(get_logger(), "loaded %d saved cones from %s", n, save_path_.c_str());
  }

  void publishTracks()
  {
    step();
    const rclcpp::Time now_t = now();
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = now_t;
    pa.header.frame_id = world_frame_;
    visualization_msgs::msg::MarkerArray ma;
    std::vector<Eigen::Vector2d> ring;
    for (const auto & tr : ptracker_.tracks()) {
      if (!tr.confirmed) continue;
      geometry_msgs::msg::Pose p;
      p.position.x = tr.pos.x(); p.position.y = tr.pos.y(); p.orientation.w = 1.0;
      pa.poses.push_back(p);
      for (int k = 0; k < 12; ++k) {
        const double a = k * M_PI / 6.0;
        ring.emplace_back(tr.pos.x() + obstacle_radius_ * std::cos(a), tr.pos.y() + obstacle_radius_ * std::sin(a));
      }
      visualization_msgs::msg::Marker mk;
      mk.header = pa.header; mk.ns = "cones"; mk.id = tr.id; mk.type = visualization_msgs::msg::Marker::CYLINDER;
      mk.pose = p; mk.pose.position.z = 0.115; mk.scale.x = mk.scale.y = 0.14; mk.scale.z = 0.23;
      mk.color.r = 1.0; mk.color.g = 0.5; mk.color.a = 0.9; mk.lifetime = rclcpp::Duration::from_seconds(0.5);
      ma.markers.push_back(mk);
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
  std::string cloud_topic_, base_frame_, world_frame_, map_topic_, save_path_;
  double buffer_s_{1.0}, wall_clear_{0.0}, obstacle_radius_{0.08};
  bool load_saved_{false};
  Eigen::Isometry3d base_T_lidar_;
  std::vector<double> regions_;
  cone_lidar::Config lcfg_;
  cone_lidar::PersistConfig pcfg_;

  // State
  std::mutex mtx_;
  cone_lidar::PersistTracker ptracker_;
  StampMap cloud_map_;
  std::deque<Scan> scan_buf_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool warned_no_map_{false};
  double last_t_{0.0}, st_t0_{0.0};
  size_t saved_count_{0}, st_cands_{0};
  int st_scans_{0}, st_no_tf_{0}, st_gated_[3]{0, 0, 0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
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
