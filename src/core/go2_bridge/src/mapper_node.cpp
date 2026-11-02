/**
 * @file mapper_node.cpp
 * @brief High-performance C++ 3D point cloud mapper & local-floor-leveled 2D grid converter.
 *
 * Modes:
 *   - "live"    : Subscribes to /scan and /velodyne_points, accumulates keyframes in odom frame,
 *                 and exports .pcd, .pgm, and .yaml on shutdown or duration timeout.
 *   - "pcd2map" : Reads an existing 3D .pcd file, removes hill slopes via cell-wise nth_element
 *                 floor estimation, and writes 2D occupancy grid (.pgm/.yaml).
 */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fs = std::filesystem;

namespace
{
volatile std::sig_atomic_t g_stop_requested = 0;
void sigintHandler(int) { g_stop_requested = 1; }

struct Point3f
{
  float x;
  float y;
  float z;
};

struct MapperConfig
{
  std::string mode = "live";
  std::string pcd_path = "";
  std::string maps_dir = "maps";
  std::string map_name = "map_live";
  std::string cloud_topic = "/velodyne_points";
  double resolution = 0.03;
  double local_floor_cell_m = 0.50;
  double local_floor_percentile = 5.0;
  double wall_z_min = 0.22;
  double wall_z_max = 1.15;
  int min_pts_per_cell = 2;
  double free_pad_m = 0.80;
  double keyframe_dist_m = 0.10;
  double keyframe_angle_deg = 5.0;
  int cloud_stride = 4;
  double min_range_m = 0.5;
  double max_range_m = 12.0;
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  double duration_sec = 0.0;
  bool local_level = true;
  bool set_default = true;
};

bool readPcdFile(const std::string & path, std::vector<Point3f> & out_pts)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    std::cerr << "[ERROR] Cannot open PCD file: " << path << "\n";
    return false;
  }

  std::string line;
  size_t num_points = 0;
  bool is_binary = false;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<std::string> fields;

  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;
    if (tag == "FIELDS") {
      std::string f;
      while (iss >> f) fields.push_back(f);
    } else if (tag == "SIZE") {
      int s;
      while (iss >> s) sizes.push_back(s);
    } else if (tag == "TYPE") {
      char t;
      while (iss >> t) types.push_back(t);
    } else if (tag == "POINTS") {
      iss >> num_points;
    } else if (tag == "DATA") {
      std::string fmt;
      iss >> fmt;
      is_binary = (fmt == "binary");
      break;
    }
  }

  if (num_points == 0) {
    return false;
  }
  out_pts.clear();
  out_pts.reserve(num_points);

  if (is_binary) {
    size_t stride = 0;
    int x_off = 0, y_off = 4, z_off = 8;
    for (size_t i = 0; i < sizes.size(); ++i) {
      if (i < fields.size()) {
        if (fields[i] == "x") x_off = static_cast<int>(stride);
        else if (fields[i] == "y") y_off = static_cast<int>(stride);
        else if (fields[i] == "z") z_off = static_cast<int>(stride);
      }
      stride += static_cast<size_t>(sizes[i]);
    }
    if (stride < 12) stride = 12;

    std::vector<uint8_t> buf(num_points * stride);
    ifs.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
    const size_t read_bytes = static_cast<size_t>(ifs.gcount());
    const size_t actual_pts = read_bytes / stride;

    for (size_t i = 0; i < actual_pts; ++i) {
      const uint8_t * ptr = buf.data() + i * stride;
      Point3f p;
      std::memcpy(&p.x, ptr + x_off, sizeof(float));
      std::memcpy(&p.y, ptr + y_off, sizeof(float));
      std::memcpy(&p.z, ptr + z_off, sizeof(float));
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        out_pts.push_back(p);
      }
    }
  } else {
    std::string dline;
    while (std::getline(ifs, dline)) {
      std::istringstream diss(dline);
      Point3f p;
      if (diss >> p.x >> p.y >> p.z) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
          out_pts.push_back(p);
        }
      }
    }
  }
  return !out_pts.empty();
}

bool writePcdFile(const std::string & path, const std::vector<Point3f> & pts)
{
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH " << pts.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << pts.size() << "\n"
      << "DATA binary\n";
  ofs.write(reinterpret_cast<const char *>(pts.data()),
            static_cast<std::streamsize>(pts.size() * sizeof(Point3f)));
  return true;
}

bool exportOccupancyGrid(const std::vector<Point3f> & raw_pts, const MapperConfig & cfg)
{
  if (raw_pts.empty()) {
    std::cerr << "[WARN] Point cloud is empty. Nothing to export.\n";
    return false;
  }

  std::vector<Point3f> pts = raw_pts;
  float x_min = pts[0].x, x_max = pts[0].x;
  float y_min = pts[0].y, y_max = pts[0].y;
  for (const auto & p : pts) {
    x_min = std::min(x_min, p.x);
    x_max = std::max(x_max, p.x);
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }

  float hill_relief = 0.0f;
  if (cfg.local_level) {
    const float cell = static_cast<float>(cfg.local_floor_cell_m);
    const int nx = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / cell)) + 1);
    const int ny = std::max(1, static_cast<int>(std::ceil((y_max - y_min) / cell)) + 1);

    std::unordered_map<int, std::vector<size_t>> cell_indices;
    cell_indices.reserve(static_cast<size_t>(nx * ny / 2 + 1));

    for (size_t i = 0; i < pts.size(); ++i) {
      int ix = std::clamp(static_cast<int>((pts[i].x - x_min) / cell), 0, nx - 1);
      int iy = std::clamp(static_cast<int>((pts[i].y - y_min) / cell), 0, ny - 1);
      cell_indices[iy * nx + ix].push_back(i);
    }

    float min_floor = 1e9f, max_floor = -1e9f;
    std::vector<float> z_buf;
    for (auto & kv : cell_indices) {
      const auto & idxs = kv.second;
      z_buf.resize(idxs.size());
      for (size_t k = 0; k < idxs.size(); ++k) {
        z_buf[k] = pts[idxs[k]].z;
      }

      float floor_z = 0.0f;
      if (z_buf.size() >= 5) {
        size_t kth = static_cast<size_t>(
          std::clamp(cfg.local_floor_percentile / 100.0, 0.0, 0.99) * z_buf.size());
        std::nth_element(z_buf.begin(), z_buf.begin() + kth, z_buf.end());
        floor_z = z_buf[kth];
      } else {
        floor_z = *std::min_element(z_buf.begin(), z_buf.end());
      }

      min_floor = std::min(min_floor, floor_z);
      max_floor = std::max(max_floor, floor_z);
      for (size_t idx : idxs) {
        pts[idx].z -= floor_z;
      }
    }
    if (max_floor >= min_floor) {
      hill_relief = max_floor - min_floor;
    }
  } else {
    std::vector<float> z_all(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) z_all[i] = pts[i].z;
    size_t kth = z_all.size() / 50;
    std::nth_element(z_all.begin(), z_all.begin() + kth, z_all.end());
    float floor_z = z_all[kth];
    for (auto & p : pts) p.z -= floor_z;
  }

  std::vector<Point3f> wall_pts;
  wall_pts.reserve(pts.size() / 2);
  const float zmin = static_cast<float>(cfg.wall_z_min);
  const float zmax = static_cast<float>(cfg.wall_z_max);
  for (const auto & p : pts) {
    if (p.z >= zmin && p.z <= zmax) {
      wall_pts.push_back(p);
    }
  }
  if (wall_pts.size() < 10) {
    wall_pts = pts;
  }

  float wx_min = wall_pts[0].x, wx_max = wall_pts[0].x;
  float wy_min = wall_pts[0].y, wy_max = wall_pts[0].y;
  for (const auto & p : wall_pts) {
    wx_min = std::min(wx_min, p.x);
    wx_max = std::max(wx_max, p.x);
    wy_min = std::min(wy_min, p.y);
    wy_max = std::max(wy_max, p.y);
  }

  const float pad = 1.0f;
  const float res = static_cast<float>(cfg.resolution);
  const float x0 = wx_min - pad;
  const float y0 = wy_min - pad;
  const int width = std::max(1, static_cast<int>(std::ceil((wx_max + pad - x0) / res)));
  const int height = std::max(1, static_cast<int>(std::ceil((wy_max + pad - y0) / res)));

  std::vector<uint8_t> img(static_cast<size_t>(width * height), 205);
  std::vector<uint8_t> seed_free(static_cast<size_t>(width * height), 0);

  for (const auto & p : pts) {
    int gx = static_cast<int>((p.x - x0) / res);
    int gy = static_cast<int>((p.y - y0) / res);
    if (gx >= 0 && gy >= 0 && gx < width && gy < height) {
      seed_free[static_cast<size_t>(gy * width + gx)] = 1;
      img[static_cast<size_t>(gy * width + gx)] = 254;
    }
  }

  if (cfg.free_pad_m > 0.0) {
    const int rad = std::min(8, std::max(1, static_cast<int>(std::round(cfg.free_pad_m / res))));
    const int rad2 = rad * rad;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!seed_free[static_cast<size_t>(y * width + x)]) continue;
        for (int dy = -rad; dy <= rad; ++dy) {
          int ny_i = y + dy;
          if (ny_i < 0 || ny_i >= height) continue;
          for (int dx = -rad; dx <= rad; ++dx) {
            if (dx * dx + dy * dy > rad2) continue;
            int nx_i = x + dx;
            if (nx_i >= 0 && nx_i < width) {
              img[static_cast<size_t>(ny_i * width + nx_i)] = 254;
            }
          }
        }
      }
    }
  }

  std::vector<int> counts(static_cast<size_t>(width * height), 0);
  for (const auto & p : wall_pts) {
    int gx = static_cast<int>((p.x - x0) / res);
    int gy = static_cast<int>((p.y - y0) / res);
    if (gx >= 0 && gy >= 0 && gx < width && gy < height) {
      counts[static_cast<size_t>(gy * width + gx)]++;
    }
  }

  size_t occupied_cells = 0;
  for (size_t i = 0; i < counts.size(); ++i) {
    if (counts[i] >= cfg.min_pts_per_cell) {
      img[i] = 0;
      occupied_cells++;
    }
  }

  fs::path maps_dir(cfg.maps_dir);
  fs::create_directories(maps_dir);
  fs::path pgm_path = maps_dir / (cfg.map_name + ".pgm");
  fs::path yaml_path = maps_dir / (cfg.map_name + ".yaml");

  {
    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y) {
      pgm.write(reinterpret_cast<const char *>(&img[static_cast<size_t>(y * width)]), width);
    }
  }

  std::ostringstream yaml_ss;
  yaml_ss << std::fixed << std::setprecision(3)
          << "image: " << cfg.map_name << ".pgm\n"
          << "mode: trinary\n"
          << "resolution: " << cfg.resolution << "\n"
          << "origin: [" << x0 << ", " << y0 << ", 0.0]\n"
          << "negate: 0\n"
          << "occupied_thresh: 0.65\n"
          << "free_thresh: 0.196\n";
  {
    std::ofstream yofs(yaml_path);
    yofs << yaml_ss.str();
  }

  std::cout << "[DONE] Map exported (C++ mapper_node):\n"
            << "  - PGM      : " << pgm_path.string() << " (" << width << "x" << height
            << " px, res=" << cfg.resolution << "m)\n"
            << "  - YAML     : " << yaml_path.string() << " (origin=[" << x0 << ", " << y0 << "])\n"
            << "  - Relief   : " << std::fixed << std::setprecision(2) << hill_relief << "m\n"
            << "  - Occupied : " << occupied_cells << " cells (" << wall_pts.size() << " wall pts)\n";

  if (cfg.set_default && cfg.map_name != "map_default") {
    fs::copy_file(pgm_path, maps_dir / "map_default.pgm", fs::copy_options::overwrite_existing);
    std::ofstream def_yaml(maps_dir / "map_default.yaml");
    std::string def_str = yaml_ss.str();
    const std::string target = "image: " + cfg.map_name + ".pgm";
    const size_t pos = def_str.find(target);
    if (pos != std::string::npos) {
      def_str.replace(pos, target.size(), "image: map_default.pgm");
    }
    def_yaml << def_str;
    std::cout << "  - Default  : updated map_default.yaml / map_default.pgm\n";
  }
  return true;
}
}  // namespace

class Go2MapperNode : public rclcpp::Node
{
public:
  Go2MapperNode()
  : Node("go2_mapper_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cfg_.mode = declare_parameter<std::string>("mode", "live");
    cfg_.pcd_path = declare_parameter<std::string>("pcd_path", "");
    cfg_.maps_dir = declare_parameter<std::string>("maps_dir", "maps");
    cfg_.map_name = declare_parameter<std::string>("map_name", "map_live");
    cfg_.cloud_topic = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    cfg_.resolution = declare_parameter<double>("resolution", 0.03);
    cfg_.local_floor_cell_m = declare_parameter<double>("local_floor_cell_m", 0.50);
    cfg_.local_floor_percentile = declare_parameter<double>("local_floor_percentile", 5.0);
    cfg_.wall_z_min = declare_parameter<double>("wall_z_min", 0.22);
    cfg_.wall_z_max = declare_parameter<double>("wall_z_max", 1.15);
    cfg_.min_pts_per_cell = declare_parameter<int>("min_pts_per_cell", 2);
    cfg_.free_pad_m = declare_parameter<double>("free_pad_m", 0.80);
    cfg_.keyframe_dist_m = declare_parameter<double>("keyframe_dist_m", 0.10);
    cfg_.keyframe_angle_deg = declare_parameter<double>("keyframe_angle_deg", 5.0);
    cfg_.cloud_stride = std::max(1, static_cast<int>(declare_parameter<int>("cloud_stride", 4)));
    // Sensor-range gate: inside min_range are the robot's own body/legs, which otherwise paint the
    // whole driven path as occupied.
    cfg_.min_range_m = declare_parameter<double>("min_range_m", 0.5);
    cfg_.max_range_m = declare_parameter<double>("max_range_m", 12.0);
    cfg_.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    cfg_.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    cfg_.duration_sec = declare_parameter<double>("duration_sec", 0.0);
    cfg_.local_level = declare_parameter<bool>("local_level", true);
    cfg_.set_default = declare_parameter<bool>("set_default", true);

    if (cfg_.mode == "live") {
      accumulated_pts_.reserve(500000);
      auto qos = rclcpp::SensorDataQoS();
      // /scan is not accumulated: its 2D returns have no height, and placing them at a fixed z
      // put every return inside the wall band.
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic, qos,
        std::bind(&Go2MapperNode::onCloud, this, std::placeholders::_1));

      RCLCPP_INFO(
        get_logger(), "mapper_node (live): map=%s, cloud=%s, stride=%d",
        cfg_.map_name.c_str(), cfg_.cloud_topic.c_str(), cfg_.cloud_stride);
    }
  }

  const MapperConfig & config() const { return cfg_; }

  void finalizeLiveMap()
  {
    if (accumulated_pts_.empty()) {
      RCLCPP_WARN(get_logger(), "No scan keyframes accumulated.");
      return;
    }
    fs::path pcd_out = fs::path(cfg_.maps_dir) / (cfg_.map_name + ".pcd");
    writePcdFile(pcd_out.string(), accumulated_pts_);
    std::cout << "\n[DONE] 3D PCD saved: " << pcd_out.string()
              << " (" << accumulated_pts_.size() << " pts)\n";

    exportOccupancyGrid(accumulated_pts_, cfg_);
  }

private:
  // Keyframes are decided on the robot base pose only. Deciding on the pose of whichever sensor
  // frame arrived made poses alternate between frames, so every message became a keyframe.
  bool checkKeyframe(double tx, double ty, double yaw)
  {
    if (!has_last_pose_) {
      last_x_ = tx;
      last_y_ = ty;
      last_yaw_ = yaw;
      has_last_pose_ = true;
      return true;
    }
    const double dist = std::hypot(tx - last_x_, ty - last_y_);
    const double dyaw = std::abs(std::remainder(yaw - last_yaw_, 2.0 * M_PI));
    if (dist >= cfg_.keyframe_dist_m || dyaw >= cfg_.keyframe_angle_deg * M_PI / 180.0) {
      last_x_ = tx;
      last_y_ = ty;
      last_yaw_ = yaw;
      return true;
    }
    return false;
  }

  // Transform at the cloud stamp; fall back to the latest one if the buffer does not reach it.
  bool lookup(const std::string & child, const rclcpp::Time & stamp, geometry_msgs::msg::TransformStamped & out)
  {
    try {
      out = tf_buffer_.lookupTransform(cfg_.odom_frame, child, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException &) {
    }
    try {
      out = tf_buffer_.lookupTransform(cfg_.odom_frame, child, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    geometry_msgs::msg::TransformStamped base_tf, sensor_tf;
    if (!lookup(cfg_.base_frame, stamp, base_tf)) return;
    if (!lookup(msg->header.frame_id, stamp, sensor_tf)) return;

    const auto & bq = base_tf.transform.rotation;
    double br, bp, byaw;
    tf2::Matrix3x3(tf2::Quaternion(bq.x, bq.y, bq.z, bq.w)).getRPY(br, bp, byaw);
    if (!checkKeyframe(base_tf.transform.translation.x, base_tf.transform.translation.y, byaw)) return;

    const size_t n_pts = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const size_t step = msg->point_step;
    if (n_pts == 0 || step < 12) return;

    // Full 3D rotation (roll/pitch included) so sloped walking does not smear the walls.
    const auto & q = sensor_tf.transform.rotation;
    const tf2::Matrix3x3 R(tf2::Quaternion(q.x, q.y, q.z, q.w));
    const auto & t = sensor_tf.transform.translation;
    const uint8_t * raw = msg->data.data();
    const double min_r2 = cfg_.min_range_m * cfg_.min_range_m;
    const double max_r2 = cfg_.max_range_m * cfg_.max_range_m;

    for (size_t i = 0; i < n_pts; i += static_cast<size_t>(cfg_.cloud_stride)) {
      float lx, ly, lz;
      std::memcpy(&lx, raw + i * step + 0, sizeof(float));
      std::memcpy(&ly, raw + i * step + 4, sizeof(float));
      std::memcpy(&lz, raw + i * step + 8, sizeof(float));
      if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) continue;
      const double r2 = static_cast<double>(lx) * lx + static_cast<double>(ly) * ly + static_cast<double>(lz) * lz;
      if (r2 < min_r2 || r2 > max_r2) continue;
      const tf2::Vector3 w = R * tf2::Vector3(lx, ly, lz);
      accumulated_pts_.push_back({
        static_cast<float>(t.x + w.x()),
        static_cast<float>(t.y + w.y()),
        static_cast<float>(t.z + w.z())
      });
    }
    keyframes_++;
    if (keyframes_ % 20 == 0) {
      std::cout << "\r  keyframes=" << keyframes_ << " | pts=" << accumulated_pts_.size()
                << " | pose=(" << std::fixed << std::setprecision(2)
                << base_tf.transform.translation.x << ", " << base_tf.transform.translation.y << ")" << std::flush;
    }
  }

  MapperConfig cfg_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::vector<Point3f> accumulated_pts_;
  size_t keyframes_ = 0;
  bool has_last_pose_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_yaw_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigintHandler);

  auto node = std::make_shared<Go2MapperNode>();
  const auto & cfg = node->config();

  if (cfg.mode == "pcd2map") {
    std::vector<Point3f> pts;
    std::cout << "[INFO] Reading PCD (C++ mapper_node): " << cfg.pcd_path << "\n";
    if (!readPcdFile(cfg.pcd_path, pts)) {
      rclcpp::shutdown();
      return 1;
    }
    std::cout << "  loaded " << pts.size() << " valid 3D points\n";
    bool ok = exportOccupancyGrid(pts, cfg);
    rclcpp::shutdown();
    return ok ? 0 : 1;
  }

  const auto t_start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok() && !g_stop_requested) {
    rclcpp::spin_some(node);
    if (cfg.duration_sec > 0.0) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
      if (elapsed >= cfg.duration_sec) {
        std::cout << "\n[INFO] Duration (" << cfg.duration_sec << "s) reached. Exporting map...\n";
        break;
      }
    }
    rate.sleep();
  }

  node->finalizeLiveMap();
  rclcpp::shutdown();
  return 0;
}
