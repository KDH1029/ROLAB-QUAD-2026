#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

namespace go2_bridge
{

constexpr int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;

class CmdVelToSportNode : public rclcpp::Node
{
public:
  CmdVelToSportNode()
  : Node("go2_cmd_vel_to_sport")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    sport_req_topic_ = declare_parameter<std::string>("sport_req_topic", "/api/sport/request");
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 0.4);
    max_wz_ = declare_parameter<double>("max_wz", 1.8);
    min_vx_ = declare_parameter<double>("min_vx", 0.08);
    min_vy_ = declare_parameter<double>("min_vy", 0.08);
    min_wz_ = declare_parameter<double>("min_wz", 0.15);
    max_ax_ = declare_parameter<double>("max_ax", 1.0);
    max_alphaz_ = declare_parameter<double>("max_alphaz", 2.0);
    deadman_s_ = declare_parameter<double>("deadman_s", 0.5);

    pub_ = create_publisher<unitree_api::msg::Request>(sport_req_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&CmdVelToSportNode::onCmdVel, this, std::placeholders::_1));

    timer_ = create_wall_timer(50ms, std::bind(&CmdVelToSportNode::onTimer, this));

    last_cmd_time_ = now();
    last_timer_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_to_sport started: %s -> %s (max_vx=%.2f, min_vx=%.2f, max_wz=%.2f)",
      cmd_vel_topic_.c_str(), sport_req_topic_.c_str(), max_vx_, min_vx_, max_wz_);
  }

  ~CmdVelToSportNode() override
  {
    sendStop();
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();

    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    // Deadband compensation to prevent in-place body swaying at very low speeds
    if (std::abs(vx) > 1e-4 && std::abs(vx) < min_vx_) {
      vx = (vx > 0.0) ? min_vx_ : -min_vx_;
    }
    if (std::abs(vy) > 1e-4 && std::abs(vy) < min_vy_) {
      vy = (vy > 0.0) ? min_vy_ : -min_vy_;
    }
    if (std::abs(wz) > 1e-4 && std::abs(wz) < min_wz_) {
      wz = (wz > 0.0) ? min_wz_ : -min_wz_;
    }

    target_vx_ = std::clamp(vx, -max_vx_, max_vx_);
    target_vy_ = std::clamp(vy, -max_vy_, max_vy_);
    target_wz_ = std::clamp(wz, -max_wz_, max_wz_);
  }

  void onTimer()
  {
    const rclcpp::Time current_time = now();
    const double elapsed = (current_time - last_cmd_time_).seconds();

    double dt = 0.05;
    if (last_timer_time_.nanoseconds() > 0) {
      dt = std::clamp((current_time - last_timer_time_).seconds(), 0.02, 0.2);
    }
    last_timer_time_ = current_time;

    double des_vx = 0.0, des_vy = 0.0, des_wz = 0.0;
    if (elapsed <= deadman_s_) {
      des_vx = target_vx_;
      des_vy = target_vy_;
      des_wz = target_wz_;
    }

    current_vx_ += std::clamp(des_vx - current_vx_, -max_ax_ * dt, max_ax_ * dt);
    current_vy_ += std::clamp(des_vy - current_vy_, -max_ax_ * dt, max_ax_ * dt);
    current_wz_ += std::clamp(des_wz - current_wz_, -max_alphaz_ * dt, max_alphaz_ * dt);

    if (std::abs(current_vx_) < 1e-3 && std::abs(current_vy_) < 1e-3 && std::abs(current_wz_) < 1e-3) {
      current_vx_ = 0.0;
      current_vy_ = 0.0;
      current_wz_ = 0.0;
      if (is_moving_) {
        sendStop();
        is_moving_ = false;
      }
      return;
    }

    sendMove(current_vx_, current_vy_, current_wz_);
    is_moving_ = true;
  }

  unitree_api::msg::Request createRequest(int32_t api_id, const std::string & parameter)
  {
    unitree_api::msg::Request req;
    const auto t = now();
    req.header.identity.id = t.nanoseconds();
    req.header.identity.api_id = api_id;
    req.header.lease.id = 0;
    req.header.policy.priority = 0;
    req.header.policy.noreply = false;
    req.parameter = parameter;
    return req;
  }

  void sendMove(double vx, double vy, double wz)
  {
    char param_buf[128];
    std::snprintf(
      param_buf, sizeof(param_buf),
      "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}",
      vx, vy, wz);
    pub_->publish(createRequest(ROBOT_SPORT_API_ID_MOVE, param_buf));
  }

  void sendStop()
  {
    pub_->publish(createRequest(ROBOT_SPORT_API_ID_STOPMOVE, "{}"));
  }

  std::string cmd_vel_topic_;
  std::string sport_req_topic_;
  double max_vx_{1.0};
  double max_vy_{0.4};
  double max_wz_{1.8};
  double min_vx_{0.08};
  double min_vy_{0.08};
  double min_wz_{0.15};
  double max_ax_{1.0};
  double max_alphaz_{2.0};
  double deadman_s_{0.5};

  double target_vx_{0.0};
  double target_vy_{0.0};
  double target_wz_{0.0};
  double current_vx_{0.0};
  double current_vy_{0.0};
  double current_wz_{0.0};
  bool is_moving_{false};

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_timer_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_bridge::CmdVelToSportNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
