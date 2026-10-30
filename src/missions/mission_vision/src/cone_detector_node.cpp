/**
 * @file cone_detector_node.cpp
 * @brief C++ Depth-Camera & 2D LaserScan Fusion Detector for 23cm Traffic Cones.
 *
 * Physical specifications (Q&A 01):
 *   - Height: 23 cm, Base: 14 x 14 cm (Orange traffic cone)
 *   - Combines compact LaserScan clusters (width <= 0.25m) with RGB-D orange ROI
 *     back-projection to publish 3D cone centroids on /mission/cone_poses.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ConeDetectorNode : public rclcpp::Node
{
public:
  ConeDetectorNode()
  : Node("cone_detector_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mission/cone_poses");
    max_cone_width_m_ = declare_parameter<double>("max_cone_width_m", 0.26);
    max_detect_range_m_ = declare_parameter<double>("max_detect_range_m", 3.50);
    fx_ = declare_parameter<double>("camera_fx", 605.0);
    fy_ = declare_parameter<double>("camera_fy", 605.0);
    cx_ = declare_parameter<double>("camera_cx", 320.0);
    cy_ = declare_parameter<double>("camera_cy", 240.0);

    cone_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);

    auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos,
      std::bind(&ConeDetectorNode::onScan, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { last_depth_ = msg; });
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, qos,
      std::bind(&ConeDetectorNode::onColor, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "cone_detector_node: fusing %s + %s -> %s",
      scan_topic_.c_str(), color_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onColor(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!last_depth_ || msg->width == 0 || msg->height == 0) return;
    if (last_depth_->width != msg->width || last_depth_->height != msg->height) return;

    const bool is_bgr = (msg->encoding == "bgr8" || msg->encoding == "bgra8");
    const int channels = (msg->encoding.find("a") != std::string::npos) ? 4 : 3;
    const bool is_16u = (last_depth_->encoding == "16UC1" || last_depth_->encoding == "mono16");

    double sum_u = 0.0, sum_v = 0.0, sum_z = 0.0;
    int count = 0;

    for (uint32_t v = msg->height / 3; v < msg->height; v += 4) {
      const uint8_t * crow = msg->data.data() + static_cast<size_t>(v) * msg->step;
      const uint8_t * drow = last_depth_->data.data() + static_cast<size_t>(v) * last_depth_->step;
      for (uint32_t u = 0; u < msg->width; u += 4) {
        const uint8_t c0 = crow[u * channels + 0];
        const uint8_t c1 = crow[u * channels + 1];
        const uint8_t c2 = crow[u * channels + 2];
        const uint8_t r = is_bgr ? c2 : c0;
        const uint8_t g = c1;
        const uint8_t b = is_bgr ? c0 : c2;

        // Orange cone heuristic: strong red, moderate green, low blue
        if (r > 160 && g > 55 && g < 150 && b < 70 && (r - b) > 100) {
          float depth_m = 0.0f;
          if (is_16u) {
            uint16_t d_mm = 0;
            std::memcpy(&d_mm, drow + u * sizeof(uint16_t), sizeof(uint16_t));
            depth_m = static_cast<float>(d_mm) * 0.001f;
          } else {
            std::memcpy(&depth_m, drow + u * sizeof(float), sizeof(float));
          }
          if (depth_m >= 0.30f && depth_m <= static_cast<float>(max_detect_range_m_)) {
            sum_u += u;
            sum_v += v;
            sum_z += depth_m;
            count++;
          }
        }
      }
    }

    if (count >= 12) {
      const double u_bar = sum_u / count;
      const double z_bar = sum_z / count;
      // Camera optical (Z forward, X right) -> Robot base_stab (X forward, Y left)
      rgbd_cone_x_ = z_bar + 0.25;
      rgbd_cone_y_ = -((u_bar - cx_) * z_bar / fx_);
      has_rgbd_cone_ = true;
    } else {
      has_rgbd_cone_ = false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::PoseArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = msg->header.frame_id.empty() ? "base_stab" : msg->header.frame_id;

    // Extract compact cylindrical clusters matching the 14x14cm base of 23cm cones
    std::vector<std::pair<double, double>> cluster;
    auto flush_cluster = [&]() {
      if (cluster.empty()) return;
      const double dx = cluster.back().first - cluster.front().first;
      const double dy = cluster.back().second - cluster.front().second;
      const double span = std::hypot(dx, dy);
      if (span <= max_cone_width_m_) {
        double cx = 0.0, cy = 0.0;
        for (const auto & pt : cluster) {
          cx += pt.first;
          cy += pt.second;
        }
        cx /= static_cast<double>(cluster.size());
        cy /= static_cast<double>(cluster.size());
        if (cx > 0.20 && cx <= max_detect_range_m_ && std::abs(cy) <= 1.20) {
          geometry_msgs::msg::Pose p;
          p.position.x = cx;
          p.position.y = cy;
          p.position.z = 0.115;  // half of 23cm cone height
          p.orientation.w = 1.0;
          out.poses.push_back(p);
        }
      }
      cluster.clear();
    };

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > max_detect_range_m_) {
        flush_cluster();
        continue;
      }
      const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double px = r * std::cos(ang);
      const double py = r * std::sin(ang);
      if (!cluster.empty()) {
        const double jump = std::hypot(px - cluster.back().first, py - cluster.back().second);
        if (jump > 0.18) {
          flush_cluster();
        }
      }
      cluster.emplace_back(px, py);
    }
    flush_cluster();

    if (has_rgbd_cone_) {
      bool merged = false;
      for (auto & p : out.poses) {
        if (std::hypot(p.position.x - rgbd_cone_x_, p.position.y - rgbd_cone_y_) < 0.30) {
          p.position.x = 0.5 * (p.position.x + rgbd_cone_x_);
          p.position.y = 0.5 * (p.position.y + rgbd_cone_y_);
          merged = true;
          break;
        }
      }
      if (!merged) {
        geometry_msgs::msg::Pose p;
        p.position.x = rgbd_cone_x_;
        p.position.y = rgbd_cone_y_;
        p.position.z = 0.115;
        p.orientation.w = 1.0;
        out.poses.push_back(p);
      }
    }

    cone_pub_->publish(out);
  }

  std::string scan_topic_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string output_topic_;
  double max_cone_width_m_ = 0.26;
  double max_detect_range_m_ = 3.50;
  double fx_ = 605.0;
  double fy_ = 605.0;
  double cx_ = 320.0;
  double cy_ = 240.0;

  bool has_rgbd_cone_ = false;
  double rgbd_cone_x_ = 0.0;
  double rgbd_cone_y_ = 0.0;
  sensor_msgs::msg::Image::SharedPtr last_depth_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
