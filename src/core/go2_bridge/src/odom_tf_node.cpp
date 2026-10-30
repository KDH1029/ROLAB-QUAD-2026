#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "go2_bridge/clock_aligner.hpp"

namespace go2_bridge
{

class OdomTfNode : public rclcpp::Node
{
public:
  OdomTfNode()
  : Node("go2_odom_tf")
  {
    topic_ = declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
    parent_ = declare_parameter<std::string>("dst_parent", "odom");
    child_ = declare_parameter<std::string>("dst_child", "base_link");
    z_lift_ = declare_parameter<double>("z_lift", 0.0);
    stamp_mode_ = declare_parameter<std::string>("stamp_mode", "align");
    max_rate_ = declare_parameter<double>("max_rate", 50.0);
    min_dt_ = (max_rate_ > 0.0) ? (1.0 / max_rate_) : 0.0;

    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
      std::bind(&OdomTfNode::onOdom, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&OdomTfNode::report, this));

    RCLCPP_INFO(
      get_logger(), "odom_tf started: %s -> %s (topic=%s, mode=%s, max_rate=%.1fHz)",
      parent_.c_str(), child_.c_str(), topic_.c_str(), stamp_mode_.c_str(), max_rate_);
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double now_s = now().seconds();
    if (min_dt_ > 0.0) {
      if (now_s - last_pub_ < min_dt_) {
        ++skipped_;
        return;
      }
      last_pub_ = now_s;
    }

    geometry_msgs::msg::TransformStamped out;
    if (stamp_mode_ == "align") {
      const double src = timeToSec(msg->header.stamp);
      if (src > 1.0) {
        const double off = clock_.update(src, now_s);
        const double aligned_s = mono_.next(src + off);
        out.header.stamp = secToTime(aligned_s);
      } else {
        out.header.stamp = secToTime(mono_.next(now_s));
      }
    } else if (stamp_mode_ == "now") {
      out.header.stamp = secToTime(mono_.next(now_s));
    } else {
      out.header.stamp = msg->header.stamp;
    }

    out.header.frame_id = parent_;
    out.child_frame_id = child_;
    const auto & p = msg->pose.pose;
    out.transform.translation.x = p.position.x;
    out.transform.translation.y = p.position.y;
    out.transform.translation.z = p.position.z + z_lift_;
    out.transform.rotation = p.orientation;

    br_->sendTransform(out);
    ++count_;
  }

  void report()
  {
    if (count_ == 0) {
      RCLCPP_WARN(get_logger(), "waiting for odometry on %s", topic_.c_str());
    } else {
      const double hz = static_cast<double>(count_) / 5.0;
      RCLCPP_INFO(
        get_logger(), "odom_tf: %.1f Hz (dropped=%zu, clk_offset=%+.3fs)",
        hz, skipped_ / 5, clock_.offset());
    }
    count_ = 0;
    skipped_ = 0;
  }

  std::string topic_, parent_, child_, stamp_mode_;
  double z_lift_{0.0}, max_rate_{50.0}, min_dt_{0.02}, last_pub_{0.0};
  size_t count_{0}, skipped_{0};
  ClockAligner clock_{10.0};
  MonotonicStamp mono_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_bridge::OdomTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
