/**
 * @file traffic_light_node.cpp
 * @brief Lightweight C++ traffic light state detector (RED / GREEN) for 2026 RoboWorld.
 *
 * Physical specifications (Rulebook & Q&A 02/03):
 *   - Height: 130 cm above floor, body size: 26 x 30 cm
 *   - Independent asynchronous switching periods between the 2 traffic lights
 *   - Evaluates the upper-center ROI in HSV color space with temporal hysteresis.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
inline void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, float & h_deg, float & s, float & v)
{
  const float rf = r / 255.0f;
  const float gf = g / 255.0f;
  const float bf = b / 255.0f;
  const float cmax = std::max({rf, gf, bf});
  const float cmin = std::min({rf, gf, bf});
  const float delta = cmax - cmin;

  v = cmax;
  s = (cmax > 1e-5f) ? (delta / cmax) : 0.0f;
  if (delta < 1e-5f) {
    h_deg = 0.0f;
    return;
  }
  if (cmax == rf) {
    h_deg = 60.0f * std::fmod(((gf - bf) / delta) + 6.0f, 6.0f);
  } else if (cmax == gf) {
    h_deg = 60.0f * (((bf - rf) / delta) + 2.0f);
  } else {
    h_deg = 60.0f * (((rf - gf) / delta) + 4.0f);
  }
}
}  // namespace

class TrafficLightNode : public rclcpp::Node
{
public:
  TrafficLightNode()
  : Node("traffic_light_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mission/traffic_light");
    roi_y_min_ = declare_parameter<double>("roi_y_min", 0.05);
    roi_y_max_ = declare_parameter<double>("roi_y_max", 0.65);
    roi_x_min_ = declare_parameter<double>("roi_x_min", 0.20);
    roi_x_max_ = declare_parameter<double>("roi_x_max", 0.80);
    min_pixel_count_ = declare_parameter<int>("min_pixel_count", 45);
    min_saturation_ = declare_parameter<double>("min_saturation", 0.45);
    min_value_ = declare_parameter<double>("min_value", 0.45);

    state_pub_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrafficLightNode::onImage, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "traffic_light_node: listening on %s -> %s",
      image_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->width == 0 || msg->height == 0 || msg->data.empty()) return;

    const bool is_bgr = (msg->encoding == "bgr8" || msg->encoding == "bgra8");
    const int channels = (msg->encoding.find("a") != std::string::npos) ? 4 : 3;
    if (msg->step < msg->width * static_cast<uint32_t>(channels)) return;

    const int u0 = std::clamp(static_cast<int>(msg->width * roi_x_min_), 0, static_cast<int>(msg->width) - 1);
    const int u1 = std::clamp(static_cast<int>(msg->width * roi_x_max_), u0 + 1, static_cast<int>(msg->width));
    const int v0 = std::clamp(static_cast<int>(msg->height * roi_y_min_), 0, static_cast<int>(msg->height) - 1);
    const int v1 = std::clamp(static_cast<int>(msg->height * roi_y_max_), v0 + 1, static_cast<int>(msg->height));

    int red_count = 0;
    int green_count = 0;
    const float s_thresh = static_cast<float>(min_saturation_);
    const float v_thresh = static_cast<float>(min_value_);

    for (int v = v0; v < v1; v += 2) {
      const uint8_t * row = msg->data.data() + static_cast<size_t>(v) * msg->step;
      for (int u = u0; u < u1; u += 2) {
        const uint8_t c0 = row[u * channels + 0];
        const uint8_t c1 = row[u * channels + 1];
        const uint8_t c2 = row[u * channels + 2];
        const uint8_t r = is_bgr ? c2 : c0;
        const uint8_t g = c1;
        const uint8_t b = is_bgr ? c0 : c2;

        float h_deg, s, val;
        rgbToHsv(r, g, b, h_deg, s, val);
        if (s < s_thresh || val < v_thresh) continue;

        if (h_deg <= 18.0f || h_deg >= 342.0f) {
          red_count++;
        } else if (h_deg >= 85.0f && h_deg <= 165.0f) {
          green_count++;
        }
      }
    }

    std::string raw_state = "UNKNOWN";
    if (green_count >= min_pixel_count_ && green_count > red_count * 1.3) {
      raw_state = "GREEN";
    } else if (red_count >= min_pixel_count_ && red_count > green_count * 1.3) {
      raw_state = "RED";
    }

    history_.push_back(raw_state);
    if (history_.size() > 5) history_.pop_front();

    int g_votes = 0, r_votes = 0;
    for (const auto & st : history_) {
      if (st == "GREEN") g_votes++;
      else if (st == "RED") r_votes++;
    }

    std_msgs::msg::String out;
    if (g_votes >= 3) out.data = "GREEN";
    else if (r_votes >= 3) out.data = "RED";
    else out.data = "UNKNOWN";

    state_pub_->publish(out);
  }

  std::string image_topic_;
  std::string output_topic_;
  double roi_y_min_ = 0.05;
  double roi_y_max_ = 0.65;
  double roi_x_min_ = 0.20;
  double roi_x_max_ = 0.80;
  int min_pixel_count_ = 45;
  double min_saturation_ = 0.45;
  double min_value_ = 0.45;

  std::deque<std::string> history_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLightNode>());
  rclcpp::shutdown();
  return 0;
}
