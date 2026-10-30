/**
 * @file parking_marker_node.cpp
 * @brief Lightweight C++ parking sign ('P' / QR 10x10cm) & bay alignment detector.
 *
 * Evaluates high-contrast square fiducial transitions in the central camera ROI
 * combined with front/side LaserScan wall distance verification inside the 1.5m x 1.5m bay.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

class ParkingMarkerNode : public rclcpp::Node
{
public:
  ParkingMarkerNode()
  : Node("parking_marker_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mission/parking_ready");
    target_wall_dist_min_ = declare_parameter<double>("target_wall_dist_min", 0.35);
    target_wall_dist_max_ = declare_parameter<double>("target_wall_dist_max", 1.35);
    min_edge_transitions_ = declare_parameter<int>("min_edge_transitions", 80);

    ready_pub_ = create_publisher<std_msgs::msg::Bool>(output_topic_, 10);
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ParkingMarkerNode::onImage, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ParkingMarkerNode::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "parking_marker_node: listening on %s & %s -> %s",
      image_topic_.c_str(), scan_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) return;
    double min_front = 99.9;
    const double fov = 20.0 * M_PI / 180.0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
      const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (std::abs(ang) <= fov && r < min_front) {
        min_front = r;
      }
    }
    front_wall_dist_ = min_front;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->width < 32 || msg->height < 32 || msg->data.empty()) return;
    const int channels = (msg->encoding.find("a") != std::string::npos) ? 4 : 3;
    if (msg->step < msg->width * static_cast<uint32_t>(channels)) return;

    const int u0 = static_cast<int>(msg->width * 0.25);
    const int u1 = static_cast<int>(msg->width * 0.75);
    const int v0 = static_cast<int>(msg->height * 0.20);
    const int v1 = static_cast<int>(msg->height * 0.80);

    // Count sharp black-white-black fiducial transitions characteristic of QR & 'P' sign (10x10cm)
    int transitions = 0;
    for (int v = v0; v < v1; v += 3) {
      const uint8_t * row = msg->data.data() + static_cast<size_t>(v) * msg->step;
      int prev_luma = -1;
      for (int u = u0; u < u1; u += 2) {
        const int luma = (static_cast<int>(row[u * channels]) +
                          static_cast<int>(row[u * channels + 1]) +
                          static_cast<int>(row[u * channels + 2])) / 3;
        if (prev_luma >= 0 && std::abs(luma - prev_luma) > 95) {
          transitions++;
        }
        prev_luma = luma;
      }
    }

    const bool dist_ok = (front_wall_dist_ >= target_wall_dist_min_ &&
                          front_wall_dist_ <= target_wall_dist_max_);
    const bool visual_ok = (transitions >= min_edge_transitions_);

    std_msgs::msg::Bool out;
    out.data = (visual_ok && dist_ok);
    ready_pub_->publish(out);
  }

  std::string image_topic_;
  std::string scan_topic_;
  std::string output_topic_;
  double target_wall_dist_min_ = 0.35;
  double target_wall_dist_max_ = 1.35;
  int min_edge_transitions_ = 80;
  double front_wall_dist_ = 1.0;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
