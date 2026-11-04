// Hesai XT16 PointCloud2 converter and terrain-aware 2D LaserScan generator.
//
// 1. Converts Hesai PointCloud2 (/lidar_points) to Velodyne format (/velodyne_points)
//    with ClockAligner timestamp synchronization.
// 2. Extracts a 2D LaserScan (/scan) in the gravity-stabilized frame (base_stab).
//    To handle sloped intersections (hill terrain) without false positive obstacles,
//    each azimuth sector tracks a radial ground profile: gradual slopes below
//    max_walkable_slope_deg are treated as traversable ground, while sharp vertical
//    steps (>= obstacle_min_dz) are marked as obstacles.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "go2_bridge/clock_aligner.hpp"

namespace go2_bridge
{

namespace
{
constexpr int OUT_OFF_X = 0;
constexpr int OUT_OFF_Y = 4;
constexpr int OUT_OFF_Z = 8;
constexpr int OUT_OFF_INTENSITY = 12;
constexpr int OUT_OFF_TIME = 16;
constexpr int OUT_OFF_RING = 20;
constexpr int OUT_POINT_STEP = 22;
}  // namespace

struct Mat3
{
  double m[9];
};

inline Mat3 mat3Identity()
{
  return Mat3{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
}

inline Mat3 quatToMat(double w, double x, double y, double z)
{
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n < 1e-12) {return mat3Identity();}
  w /= n; x /= n; y /= n; z /= n;
  Mat3 r;
  r.m[0] = 1 - 2 * (y * y + z * z); r.m[1] = 2 * (x * y - z * w); r.m[2] = 2 * (x * z + y * w);
  r.m[3] = 2 * (x * y + z * w);     r.m[4] = 1 - 2 * (x * x + z * z); r.m[5] = 2 * (y * z - x * w);
  r.m[6] = 2 * (x * z - y * w);     r.m[7] = 2 * (y * z + x * w);     r.m[8] = 1 - 2 * (x * x + y * y);
  return r;
}

struct InputLayout
{
  int x{-1}, y{-1}, z{-1}, intensity{-1}, ring{-1}, timestamp{-1};
  uint8_t ring_type{0};
  uint8_t timestamp_type{0};

  bool valid() const
  {
    return x >= 0 && y >= 0 && z >= 0;
  }
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CloudBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  CloudBridgeNode()
  : rclcpp_lifecycle::LifecycleNode("go2_cloud_bridge")
  {
    in_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    out_topic_ = declare_parameter<std::string>("output_topic", "/velodyne_points");
    frame_id_ = declare_parameter<std::string>("frame_id", "hesai_lidar");
    stride_ = std::max<int>(1, declare_parameter<int>("stride", 1));

    publish_terrain_scan_ = declare_parameter<bool>("publish_terrain_scan", true);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_stab");
    min_range_ = declare_parameter<double>("min_range", 0.40);
    max_range_ = declare_parameter<double>("max_range", 15.0);
    enable_slope_filter_ = declare_parameter<bool>("enable_slope_filter", true);
    max_walkable_slope_deg_ = declare_parameter<double>("max_walkable_slope_deg", 18.0);
    obstacle_min_dz_ = declare_parameter<double>("obstacle_min_dz", 0.18);
    obstacle_max_dz_ = declare_parameter<double>("obstacle_max_dz", 1.20);
    radial_bin_m_ = declare_parameter<double>("radial_bin_m", 0.35);
    azimuth_bins_ = std::max<int>(90, declare_parameter<int>("azimuth_bins", 360));
    // "slope": radial ground profile (original). "vertical": per azimuth, a wall is >= vertical_min_rings
    // adjacent rings hitting the same horizontal distance (+-vertical_tol_m). Floors, ramps and stair
    // treads spread out with elevation and never qualify; needs no ground or robot-height estimate.
    scan_mode_ = declare_parameter<std::string>("scan_mode", "slope");
    vertical_tol_m_ = declare_parameter<double>("vertical_tol_m", 0.03);
    vertical_min_rings_ = std::max<int>(2, declare_parameter<int>("vertical_min_rings", 3));
    vertical_min_span_m_ = declare_parameter<double>("vertical_min_span_m", 0.08);
    // Optional second vertical scan for AMCL only (amcl_scan_topic "" = off): just the returns between
    // amcl_min_z and amcl_max_z in target_frame (base_stab, origin at the body ~0.31 m over the feet).
    // Default band = 0.30-0.57 m above the floor: over the 0.23-0.25 m cones (they never reach AMCL, so
    // moved cones cannot pull it) and under the 0.6 m arena walls (hall clutter behind them is cut too).
    // Floors / ramps / steps cannot enter the band, so fewer stacked rings are needed than for /scan,
    // and the band is only ~0.27 m tall: 3 rings would drop walls beyond ~3 m, 2 rings keep them to ~4-5 m.
    amcl_scan_topic_ = declare_parameter<std::string>("amcl_scan_topic", "");
    amcl_min_z_ = declare_parameter<double>("amcl_min_z", -0.02);
    amcl_max_z_ = declare_parameter<double>("amcl_max_z", 0.26);
    amcl_min_rings_ = std::max<int>(1, declare_parameter<int>("amcl_min_rings", 2));
    amcl_min_span_m_ = declare_parameter<double>("amcl_min_span_m", 0.03);

  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      out_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    if (publish_terrain_scan_) {
      scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS());
      if (!amcl_scan_topic_.empty()) {
        amcl_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(amcl_scan_topic_, rclcpp::SensorDataQoS());
      }
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_activate();
    if (scan_pub_) {scan_pub_->on_activate();}
    if (amcl_scan_pub_) {amcl_scan_pub_->on_activate();}
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudBridgeNode::onCloud, this, std::placeholders::_1));
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&CloudBridgeNode::report, this));
    RCLCPP_INFO(
      get_logger(),
      "cloud_bridge active: %s -> %s (scan=%s, mode=%s)",
      in_topic_.c_str(), out_topic_.c_str(),
      publish_terrain_scan_ ? scan_topic_.c_str() : "off", scan_mode_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    sub_.reset();
    report_timer_.reset();
    pub_->on_deactivate();
    if (scan_pub_) {scan_pub_->on_deactivate();}
    if (amcl_scan_pub_) {amcl_scan_pub_->on_deactivate();}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    pub_.reset();
    scan_pub_.reset();
    amcl_scan_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    layout_ = InputLayout();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    sub_.reset();
    report_timer_.reset();
    pub_.reset();
    scan_pub_.reset();
    amcl_scan_pub_.reset();
    tf_listener_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  static InputLayout resolveLayout(const sensor_msgs::msg::PointCloud2 & msg)
  {
    InputLayout l;
    for (const auto & f : msg.fields) {
      if (f.name == "x") {l.x = static_cast<int>(f.offset);}
      else if (f.name == "y") {l.y = static_cast<int>(f.offset);}
      else if (f.name == "z") {l.z = static_cast<int>(f.offset);}
      else if (f.name == "intensity") {l.intensity = static_cast<int>(f.offset);}
      else if (f.name == "ring") {
        l.ring = static_cast<int>(f.offset);
        l.ring_type = f.datatype;
      } else if (f.name == "timestamp" || f.name == "time" || f.name == "t") {
        l.timestamp = static_cast<int>(f.offset);
        l.timestamp_type = f.datatype;
      }
    }
    return l;
  }

  static double readTimestamp(const uint8_t * p, uint8_t datatype)
  {
    if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
      double v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    float v;
    std::memcpy(&v, p, sizeof(v));
    return static_cast<double>(v);
  }

  static uint16_t readRing(const uint8_t * p, uint8_t datatype)
  {
    if (datatype == sensor_msgs::msg::PointField::UINT16 ||
      datatype == sensor_msgs::msg::PointField::INT16)
    {
      uint16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    if (datatype == sensor_msgs::msg::PointField::UINT8 ||
      datatype == sensor_msgs::msg::PointField::INT8)
    {
      return static_cast<uint16_t>(*p);
    }
    uint32_t v;
    std::memcpy(&v, p, sizeof(v));
    return static_cast<uint16_t>(v);
  }

  static std::vector<sensor_msgs::msg::PointField> makeVelodyneFields()
  {
    using PF = sensor_msgs::msg::PointField;
    std::vector<PF> f(6);
    auto set = [](PF & p, const char * name, uint32_t off, uint8_t type) {
        p.name = name; p.offset = off; p.datatype = type; p.count = 1;
      };
    set(f[0], "x", OUT_OFF_X, PF::FLOAT32);
    set(f[1], "y", OUT_OFF_Y, PF::FLOAT32);
    set(f[2], "z", OUT_OFF_Z, PF::FLOAT32);
    set(f[3], "intensity", OUT_OFF_INTENSITY, PF::FLOAT32);
    set(f[4], "time", OUT_OFF_TIME, PF::FLOAT32);
    set(f[5], "ring", OUT_OFF_RING, PF::UINT16);
    return f;
  }

  void publishVerticalScan(
    const std::vector<float> & bx, const std::vector<float> & by, const std::vector<float> & bz,
    const std::vector<uint16_t> & ring, const builtin_interfaces::msg::Time & stamp,
    const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr & pub,
    float min_z, float max_z, int min_rings, float min_span)
  {
    if (!pub || bx.empty()) {return;}
    const int n_az = azimuth_bins_;
    const double d_az = (2.0 * M_PI) / static_cast<double>(n_az);
    int n_ring = 0;
    for (auto r : ring) {n_ring = std::max<int>(n_ring, r + 1);}
    n_ring = std::min(n_ring, 128);
    const float INF = std::numeric_limits<float>::infinity();

    // Nearest return of each ring in each azimuth bin (horizontal distance from base_stab origin)
    std::vector<float> rh(static_cast<size_t>(n_az * n_ring), INF), zz(static_cast<size_t>(n_az * n_ring), 0.0f);
    for (size_t i = 0; i < bx.size(); ++i) {
      const float r = std::hypot(bx[i], by[i]);
      if (r < min_range_ || r >= max_range_ || ring[i] >= n_ring || bz[i] < min_z || bz[i] > max_z) {continue;}
      int a = static_cast<int>((std::atan2(by[i], bx[i]) + M_PI) / d_az);
      a = std::clamp(a, 0, n_az - 1);
      const size_t k = static_cast<size_t>(a * n_ring + ring[i]);
      if (r < rh[k]) {rh[k] = r; zz[k] = bz[i];}
    }

    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = stamp;
    scan->header.frame_id = target_frame_;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = static_cast<float>(d_az);
    scan->scan_time = 0.05f;
    scan->range_min = static_cast<float>(min_range_);
    scan->range_max = static_cast<float>(max_range_);
    scan->ranges.assign(n_az, INF);

    const float tol = static_cast<float>(vertical_tol_m_);
    for (int a = 0; a < n_az; ++a) {
      const float * r = &rh[static_cast<size_t>(a * n_ring)];
      const float * z = &zz[static_cast<size_t>(a * n_ring)];
      float best = INF;
      // Runs of adjacent rings whose horizontal distances stay within tol of each other
      for (int i = 0; i < n_ring; ++i) {
        if (!std::isfinite(r[i])) {continue;}
        float lo = r[i], hi = r[i], zlo = z[i], zhi = z[i];
        int j = i + 1;
        for (; j < n_ring && std::isfinite(r[j]); ++j) {
          const float nlo = std::min(lo, r[j]), nhi = std::max(hi, r[j]);
          if (nhi - nlo > tol) {break;}
          lo = nlo; hi = nhi; zlo = std::min(zlo, z[j]); zhi = std::max(zhi, z[j]);
        }
        if (j - i >= min_rings && zhi - zlo >= min_span) {
          best = std::min(best, 0.5f * (lo + hi));
        }
      }
      scan->ranges[a] = best;
    }
    pub->publish(std::move(scan));
  }

  void publishTerrainAwareScan(
    const std::vector<float> & bx,
    const std::vector<float> & by,
    const std::vector<float> & bz,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!scan_pub_ || bx.empty()) {return;}

    const int n_az = azimuth_bins_;
    const int n_rad = std::max<int>(1, static_cast<int>(std::ceil(max_range_ / radial_bin_m_)));
    const double d_az = (2.0 * M_PI) / static_cast<double>(n_az);

    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> bin_min_z(n_az * n_rad, INF);
    std::vector<float> bin_max_z(n_az * n_rad, -INF);
    std::vector<float> bin_min_r(n_az * n_rad, INF);

    for (size_t i = 0; i < bx.size(); ++i) {
      const float x = bx[i];
      const float y = by[i];
      const float z = bz[i];
      const float r = std::hypot(x, y);
      if (r < min_range_ || r >= max_range_) {continue;}

      double ang = std::atan2(y, x);
      int az_idx = static_cast<int>((ang + M_PI) / d_az);
      if (az_idx < 0) {az_idx = 0;}
      if (az_idx >= n_az) {az_idx = n_az - 1;}

      int rad_idx = static_cast<int>(r / radial_bin_m_);
      if (rad_idx < 0) {rad_idx = 0;}
      if (rad_idx >= n_rad) {rad_idx = n_rad - 1;}

      const int idx = az_idx * n_rad + rad_idx;
      if (z < bin_min_z[idx]) {bin_min_z[idx] = z;}
      if (z > bin_max_z[idx]) {bin_max_z[idx] = z;}
      if (r < bin_min_r[idx]) {bin_min_r[idx] = r;}
    }

    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = stamp;
    scan->header.frame_id = target_frame_;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = static_cast<float>(d_az);
    scan->time_increment = 0.0f;
    scan->scan_time = 0.05f;
    scan->range_min = static_cast<float>(min_range_);
    scan->range_max = static_cast<float>(max_range_);
    scan->ranges.assign(n_az, INF);

    const double max_slope_tan = std::tan(max_walkable_slope_deg_ * M_PI / 180.0);

    for (int a = 0; a < n_az; ++a) {
      double ground_z = -0.30;
      double prev_r = 0.0;

      for (int r_bin = 0; r_bin < n_rad; ++r_bin) {
        const int idx = a * n_rad + r_bin;
        const float min_z = bin_min_z[idx];
        const float max_z = bin_max_z[idx];
        const float r_val = bin_min_r[idx];
        if (!std::isfinite(min_z)) {continue;}

        const double dr = std::max(0.1, static_cast<double>(r_val) - prev_r);

        if (enable_slope_filter_) {
          const double vertical_span = static_cast<double>(max_z - min_z);
          const double dz_from_ground = static_cast<double>(min_z) - ground_z;
          const double slope_tan = std::abs(dz_from_ground) / dr;

          if (vertical_span >= obstacle_min_dz_ && (max_z - ground_z) <= obstacle_max_dz_) {
            scan->ranges[a] = r_val;
            break;
          } else if (dz_from_ground >= obstacle_min_dz_ && slope_tan > max_slope_tan &&
                     dz_from_ground <= obstacle_max_dz_)
          {
            scan->ranges[a] = r_val;
            break;
          } else if (slope_tan <= max_slope_tan) {
            // Track the last accepted ground point itself. An EMA lags behind a ramp, so the next
            // bin's rise is measured against a stale height and a 10 deg ramp reads as a wall.
            ground_z = static_cast<double>(min_z);
            prev_r = static_cast<double>(r_val);
          }
        } else {
          if (max_z >= -0.04f && max_z <= 0.70f) {
            scan->ranges[a] = r_val;
            break;
          }
        }
      }
    }

    scan_pub_->publish(std::move(scan));
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (n == 0) {return;}

    if (!layout_.valid()) {
      layout_ = resolveLayout(*msg);
      if (!layout_.valid()) {return;}
    }

    const uint8_t * src = msg->data.data();
    const size_t in_step = msg->point_step;

    double t_min = std::numeric_limits<double>::max();
    double t_max = std::numeric_limits<double>::lowest();
    if (layout_.timestamp >= 0) {
      for (size_t i = 0; i < n; ++i) {
        const double t = readTimestamp(src + i * in_step + layout_.timestamp, layout_.timestamp_type);
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
      }
    } else {
      t_min = timeToSec(msg->header.stamp);
      t_max = t_min;
    }

    const double now_s = now().seconds();
    double stamp_s = now_s;
    if (t_max > 1.0) {
      const double off = clock_.update(t_max, now_s);
      stamp_s = mono_.next(t_min + off);
    } else {
      stamp_s = mono_.next(now_s);
    }
    const auto out_stamp = secToTime(stamp_s);

    Mat3 R_to_stab = mat3Identity();
    double tx_stab = 0.0, ty_stab = 0.0, tz_stab = 0.0;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id,
        tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      R_to_stab = quatToMat(q.w, q.x, q.y, q.z);
      tx_stab = tf.transform.translation.x;
      ty_stab = tf.transform.translation.y;
      tz_stab = tf.transform.translation.z;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF to %s unavailable (%s); /scan uses fixed untilted extrinsic", target_frame_.c_str(), e.what());
      R_to_stab = Mat3{{0, -1, 0, 1, 0, 0, 0, 0, 1}};
      tx_stab = 0.25;
      tz_stab = 0.1588;
    }

    const size_t m = (n + stride_ - 1) / static_cast<size_t>(stride_);
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header.stamp = out_stamp;
    out->header.frame_id = frame_id_;
    out->height = 1;
    out->width = static_cast<uint32_t>(m);
    out->is_bigendian = false;
    out->is_dense = true;
    out->point_step = OUT_POINT_STEP;
    out->row_step = OUT_POINT_STEP * static_cast<uint32_t>(m);
    out->fields = makeVelodyneFields();
    out->data.resize(static_cast<size_t>(OUT_POINT_STEP) * m);

    std::vector<float> stab_x, stab_y, stab_z;
    std::vector<uint16_t> stab_ring;
    if (publish_terrain_scan_) {
      stab_x.reserve(m);
      stab_y.reserve(m);
      stab_z.reserve(m);
      stab_ring.reserve(m);
    }

    uint8_t * dst = out->data.data();
    size_t k = 0;
    for (size_t i = 0; i < n; i += static_cast<size_t>(stride_), ++k) {
      const uint8_t * sp = src + i * in_step;
      uint8_t * dp = dst + k * OUT_POINT_STEP;

      float px, py, pz;
      std::memcpy(&px, sp + layout_.x, 4);
      std::memcpy(&py, sp + layout_.y, 4);
      std::memcpy(&pz, sp + layout_.z, 4);

      std::memcpy(dp + OUT_OFF_X, &px, 4);
      std::memcpy(dp + OUT_OFF_Y, &py, 4);
      std::memcpy(dp + OUT_OFF_Z, &pz, 4);

      if (layout_.intensity >= 0) {
        std::memcpy(dp + OUT_OFF_INTENSITY, sp + layout_.intensity, 4);
      } else {
        const float zero = 0.0f;
        std::memcpy(dp + OUT_OFF_INTENSITY, &zero, 4);
      }

      const float rel = (layout_.timestamp >= 0)
        ? static_cast<float>(readTimestamp(sp + layout_.timestamp, layout_.timestamp_type) - t_min)
        : 0.0f;
      std::memcpy(dp + OUT_OFF_TIME, &rel, 4);

      const uint16_t ring = (layout_.ring >= 0) ? readRing(sp + layout_.ring, layout_.ring_type) : 0;
      std::memcpy(dp + OUT_OFF_RING, &ring, 2);

      if (publish_terrain_scan_ && std::isfinite(px) && std::isfinite(py) && std::isfinite(pz)) {
        const float sx = static_cast<float>(
          R_to_stab.m[0] * px + R_to_stab.m[1] * py + R_to_stab.m[2] * pz + tx_stab);
        const float sy = static_cast<float>(
          R_to_stab.m[3] * px + R_to_stab.m[4] * py + R_to_stab.m[5] * pz + ty_stab);
        const float sz = static_cast<float>(
          R_to_stab.m[6] * px + R_to_stab.m[7] * py + R_to_stab.m[8] * pz + tz_stab);
        stab_x.push_back(sx);
        stab_y.push_back(sy);
        stab_z.push_back(sz);
        stab_ring.push_back(ring);
      }
    }

    pub_->publish(std::move(out));
    if (publish_terrain_scan_) {
      if (scan_mode_ == "vertical" && layout_.ring >= 0) {
        const float inf = std::numeric_limits<float>::infinity();
        publishVerticalScan(stab_x, stab_y, stab_z, stab_ring, out_stamp, scan_pub_, -inf, inf,
                            vertical_min_rings_, static_cast<float>(vertical_min_span_m_));
        if (amcl_scan_pub_) {
          publishVerticalScan(stab_x, stab_y, stab_z, stab_ring, out_stamp, amcl_scan_pub_,
                              static_cast<float>(amcl_min_z_), static_cast<float>(amcl_max_z_),
                              amcl_min_rings_, static_cast<float>(amcl_min_span_m_));
        }
      } else {
        publishTerrainAwareScan(stab_x, stab_y, stab_z, out_stamp);
      }
    }
    ++count_;
  }

  void report()
  {
    if (count_ > 0) {
      RCLCPP_INFO(
        get_logger(),
        "cloud_bridge: %.1f Hz (clk_offset=%+.3fs)",
        static_cast<double>(count_) / 5.0,
        clock_.initialized() ? clock_.offset() : 0.0);
    }
    count_ = 0;
  }

  std::string in_topic_, out_topic_, frame_id_;
  int stride_{1};
  bool publish_terrain_scan_{true};
  std::string scan_topic_, target_frame_;
  double min_range_{0.40}, max_range_{15.0};
  bool enable_slope_filter_{true};
  double max_walkable_slope_deg_{18.0};
  double obstacle_min_dz_{0.18};
  double obstacle_max_dz_{1.20};
  double radial_bin_m_{0.35};
  int azimuth_bins_{360};
  std::string scan_mode_{"slope"};
  double vertical_tol_m_{0.03};
  int vertical_min_rings_{3};
  std::string amcl_scan_topic_;
  double amcl_min_z_{-0.02}, amcl_max_z_{0.26}, amcl_min_span_m_{0.03};
  int amcl_min_rings_{2};
  double vertical_min_span_m_{0.08};

  InputLayout layout_;
  ClockAligner clock_;
  MonotonicStamp mono_;
  size_t count_{0};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_, amcl_scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace go2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_bridge::CloudBridgeNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
