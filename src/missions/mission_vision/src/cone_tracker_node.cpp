/**
 * @file cone_tracker_node.cpp
 * @brief Cone localisation: YOLO boxes + LiDAR (preferred) / depth (fallback) -> world-frame tracks.
 *
 * Per detection (vision_msgs/Detection2DArray from yolo_detector_node):
 *   1. LiDAR points projected into the colour image; points inside the (shrunk) box, nearest
 *      fraction by range -> cone centre in base_link.
 *   2. If too few LiDAR points: depth pixels re-projected into the colour image (no aligned depth
 *      needed), median of those inside the box.
 *   3. base_link -> world_frame (AMCL map) through TF at the image stamp.
 *   4. Tracking: nearest track within assoc_dist, EMA update; confirmed after confirm_hits;
 *      tentative tracks expire after tentative_timeout, confirmed ones after confirmed_timeout.
 * Outputs (world frame): /mission/cones (confirmed, PoseArray), /mission/cone_obstacles
 * (PointCloud2 ring per cone for the planner grid), /mission/cone_markers.
 *
 * Extrinsics come from config/camera_extrinsics.yaml (written by tools/calib_capture.py).
 */
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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

struct Track
{
  Eigen::Vector2d pos;
  int hits{0};
  rclcpp::Time last_seen;
  bool confirmed{false};
  int id{0};
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
    color_info_topic_ = declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
    depth_info_topic_ = declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");

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
    min_lidar_pts_ = declare_parameter<int>("fusion.min_lidar_points", 3);
    near_fraction_ = declare_parameter<double>("fusion.near_fraction", 0.3);
    depth_stride_ = std::max<int>(1, declare_parameter<int>("fusion.depth_stride", 4));
    depth_min_ = declare_parameter<double>("fusion.depth_min", 0.25);
    depth_max_ = declare_parameter<double>("fusion.depth_max", 4.0);
    max_range_ = declare_parameter<double>("fusion.max_range", 4.0);
    min_score_ = declare_parameter<double>("fusion.min_score", 0.35);
    surface_to_center_ = declare_parameter<double>("fusion.surface_to_center", 0.05);

    assoc_dist_ = declare_parameter<double>("tracking.assoc_dist", 0.30);
    confirm_hits_ = declare_parameter<int>("tracking.confirm_hits", 3);
    tentative_timeout_ = declare_parameter<double>("tracking.tentative_timeout", 2.0);
    confirmed_timeout_ = declare_parameter<double>("tracking.confirmed_timeout", 30.0);
    ema_alpha_ = declare_parameter<double>("tracking.ema_alpha", 0.3);
    obstacle_radius_ = declare_parameter<double>("output.obstacle_radius", 0.08);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cones_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/mission/cones", 10);
    obs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/mission/cone_obstacles", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mission/cone_markers", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cones_pub_->on_activate();
    obs_pub_->on_activate();
    marker_pub_->on_activate();
    auto qos = rclcpp::SensorDataQoS();
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      det_topic_, 10, std::bind(&ConeTrackerNode::onDetections, this, std::placeholders::_1));
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic_, qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {color_K_ = toPinhole(*m);});
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic_, qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {depth_K_ = toPinhole(*m);});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos, [this](const sensor_msgs::msg::Image::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_depth_ = m;
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos, std::bind(&ConeTrackerNode::onCloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&ConeTrackerNode::publishTracks, this));
    RCLCPP_INFO(get_logger(), "cone_tracker active: %s + %s/%s -> %s frame",
                det_topic_.c_str(), cloud_topic_.c_str(), depth_topic_.c_str(), world_frame_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    det_sub_.reset(); color_info_sub_.reset(); depth_info_sub_.reset(); depth_sub_.reset(); cloud_sub_.reset();
    timer_.reset();
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
      if (p.x() < 0.1 || p.head<2>().norm() > max_range_ + 0.5) continue;
      pts.push_back(p);
    }
    std::lock_guard<std::mutex> lk(mtx_);
    lidar_base_ = std::move(pts);
  }

  bool inBox(double u, double v, double u0, double v0, double u1, double v1) const
  {
    return u >= u0 && u <= u1 && v >= v0 && v <= v1;
  }

  // Cone centre in base_link from LiDAR points that project into the box
  bool fromLidar(double u0, double v0, double u1, double v1, Eigen::Vector3d & out)
  {
    std::vector<std::pair<double, Eigen::Vector3d>> hits;
    const Eigen::Isometry3d color_T_base = base_T_color_.inverse();
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & pb : lidar_base_) {
      const Eigen::Vector3d pc = color_T_base * pb;
      if (pc.z() < 0.1) continue;
      const double u = color_K_.fx * pc.x() / pc.z() + color_K_.cx;
      const double v = color_K_.fy * pc.y() / pc.z() + color_K_.cy;
      if (inBox(u, v, u0, v0, u1, v1)) hits.emplace_back(pc.norm(), pb);
    }
    if (static_cast<int>(hits.size()) < min_lidar_pts_) return false;
    // The box also covers floor and wall behind the cone: keep the nearest fraction
    std::sort(hits.begin(), hits.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
    const size_t n = std::max<size_t>(static_cast<size_t>(min_lidar_pts_), static_cast<size_t>(near_fraction_ * hits.size()));
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < n && i < hits.size(); ++i) c += hits[i].second;
    out = c / static_cast<double>(std::min(n, hits.size()));
    return true;
  }

  // Cone centre from depth pixels re-projected into the colour image (no aligned depth needed)
  bool fromDepth(double u0, double v0, double u1, double v1, Eigen::Vector3d & out)
  {
    sensor_msgs::msg::Image::SharedPtr d;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      d = last_depth_;
    }
    if (!d || !depth_K_.valid()) return false;
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
        const Eigen::Vector3d pc = color_T_depth_ * pd;
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

  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (!color_K_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", color_info_topic_.c_str());
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s->%s: %s",
                             world_frame_.c_str(), base_frame_.c_str(), e.what());
        return;
      }
    }
    const auto & tq = tf.transform.rotation;
    Eigen::Isometry3d world_T_base = Eigen::Isometry3d::Identity();
    world_T_base.linear() = Eigen::Quaterniond(tq.w, tq.x, tq.y, tq.z).toRotationMatrix();
    world_T_base.translation() = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

    const rclcpp::Time stamp(msg->header.stamp);
    for (const auto & det : msg->detections) {
      if (!det.results.empty() && det.results[0].hypothesis.score < min_score_) continue;
      const double cu = det.bbox.center.position.x, cv = det.bbox.center.position.y;
      const double hw = 0.5 * det.bbox.size_x * (1.0 - box_shrink_), hh = 0.5 * det.bbox.size_y * (1.0 - box_shrink_);
      Eigen::Vector3d pb;
      const char * src = "lidar";
      if (!fromLidar(cu - hw, cv - hh, cu + hw, cv + hh, pb)) {
        src = "depth";
        if (!fromDepth(cu - hw, cv - hh, cu + hw, cv + hh, pb)) continue;
      }
      if (pb.head<2>().norm() > max_range_) continue;
      // Both sources measure the cone's near surface: push the estimate back along the
      // horizontal viewing ray towards the cone axis.
      Eigen::Vector2d ray = pb.head<2>() - base_T_color_.translation().head<2>();
      if (ray.norm() > 1e-3) pb.head<2>() += surface_to_center_ * ray.normalized();
      const Eigen::Vector3d pw = world_T_base * pb;
      update(Eigen::Vector2d(pw.x(), pw.y()), stamp);
      RCLCPP_DEBUG(get_logger(), "cone (%s) base (%.2f, %.2f) world (%.2f, %.2f)", src, pb.x(), pb.y(), pw.x(), pw.y());
    }
  }

  void update(const Eigen::Vector2d & p, const rclcpp::Time & t)
  {
    Track * best = nullptr;
    double best_d = assoc_dist_;
    for (auto & tr : tracks_) {
      const double d = (tr.pos - p).norm();
      if (d < best_d) {best_d = d; best = &tr;}
    }
    if (best) {
      best->pos = (1.0 - ema_alpha_) * best->pos + ema_alpha_ * p;
      best->hits++;
      best->last_seen = t;
      if (!best->confirmed && best->hits >= confirm_hits_) {
        best->confirmed = true;
        RCLCPP_INFO(get_logger(), "cone #%d confirmed at (%.2f, %.2f) in %s", best->id, best->pos.x(), best->pos.y(), world_frame_.c_str());
      }
    } else {
      tracks_.push_back(Track{p, 1, t, confirm_hits_ <= 1, next_id_++});
    }
  }

  void publishTracks()
  {
    const rclcpp::Time now_t = now();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
      const double age = (now_t - tr.last_seen).seconds();
      return age > (tr.confirmed ? confirmed_timeout_ : tentative_timeout_);
    }), tracks_.end());

    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = now_t;
    pa.header.frame_id = world_frame_;
    visualization_msgs::msg::MarkerArray ma;
    std::vector<Eigen::Vector2d> ring;
    for (const auto & tr : tracks_) {
      if (!tr.confirmed) continue;
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
  std::string det_topic_, color_info_topic_, depth_topic_, depth_info_topic_, cloud_topic_, base_frame_, world_frame_;
  Eigen::Isometry3d base_T_color_, base_T_lidar_, color_T_depth_;
  double box_shrink_, near_fraction_, depth_min_, depth_max_, max_range_, min_score_, surface_to_center_;
  int min_lidar_pts_, depth_stride_;
  double assoc_dist_, tentative_timeout_, confirmed_timeout_, ema_alpha_, obstacle_radius_;
  int confirm_hits_;

  // State
  std::mutex mtx_;
  Pinhole color_K_, depth_K_;
  sensor_msgs::msg::Image::SharedPtr last_depth_;
  std::vector<Eigen::Vector3d> lidar_base_;
  std::vector<Track> tracks_;
  int next_id_{0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
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
