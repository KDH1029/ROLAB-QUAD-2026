#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "go2_bridge/clock_aligner.hpp"
#include "go2_bridge/go2_kinematics.hpp"

namespace go2_bridge
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ImuBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ImuBridgeNode()
  : rclcpp_lifecycle::LifecycleNode("go2_imu_bridge")
  {
    in_topic_ = declare_parameter<std::string>("input_topic", "/lowstate");
    out_topic_ = declare_parameter<std::string>("output_topic", "/imu/body");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    publish_static_tf_ = declare_parameter<bool>("publish_static_tf", true);
    publish_stab_ = declare_parameter<bool>("publish_stab", true);
    stab_parent_ = declare_parameter<std::string>("stab_parent", "base_link");
    stab_child_ = declare_parameter<std::string>("stab_child", "base_stab");
    stab_rate_ = declare_parameter<double>("stab_rate", 50.0);
    stab_min_dt_ = (stab_rate_ > 0.0) ? (1.0 / stab_rate_) : 0.0;

    // Extrinsic calibration (base_link -> hesai_lidar)
    lidar_x_ = declare_parameter<double>("lidar_x", 0.2500);
    lidar_y_ = declare_parameter<double>("lidar_y", 0.0000);
    lidar_z_ = declare_parameter<double>("lidar_z", 0.1588);
    lidar_roll_ = declare_parameter<double>("lidar_roll", 0.012330);
    lidar_pitch_ = declare_parameter<double>("lidar_pitch", 0.005579);
    lidar_yaw_ = declare_parameter<double>("lidar_yaw", 1.57079632679);

    // Kinematic attitude filter & slope adaptation
    stab_mode_ = declare_parameter<std::string>("stab_mode", "gravity");
    use_kinematics_filter_ = declare_parameter<bool>("use_kinematics_filter", false);  // see bridge_params.yaml
    filter_alpha_ = declare_parameter<double>("filter_alpha", 0.96);
    contact_force_thresh_ = declare_parameter<int>("contact_force_thresh", 15);
    max_kin_imu_diff_deg_ = declare_parameter<double>("max_kin_imu_diff_deg", 6.0);

  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, pub_qos);
    if (publish_stab_) {
      tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    if (publish_static_tf_) {
      static_tf_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_activate();
    if (static_tf_br_) {
      publishStaticTransforms();
    }
    initialized_filter_ = false;
    sub_ = create_subscription<unitree_go::msg::LowState>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuBridgeNode::onLowState, this, std::placeholders::_1));
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&ImuBridgeNode::report, this));
    RCLCPP_INFO(
      get_logger(),
      "imu_bridge active: mode=%s, fk_filter=%s (alpha=%.2f, max_diff=%.1f deg)",
      stab_mode_.c_str(), use_kinematics_filter_ ? "on" : "off",
      filter_alpha_, max_kin_imu_diff_deg_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    sub_.reset();
    report_timer_.reset();
    pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    pub_.reset();
    tf_br_.reset();
    static_tf_br_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    sub_.reset();
    report_timer_.reset();
    pub_.reset();
    tf_br_.reset();
    static_tf_br_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void publishStaticTransforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    geometry_msgs::msg::TransformStamped t_lidar;
    t_lidar.header.stamp = now();
    t_lidar.header.frame_id = "base_link";
    t_lidar.child_frame_id = "hesai_lidar";
    t_lidar.transform.translation.x = lidar_x_;
    t_lidar.transform.translation.y = lidar_y_;
    t_lidar.transform.translation.z = lidar_z_;

    tf2::Quaternion q_lidar;
    q_lidar.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    t_lidar.transform.rotation.x = q_lidar.x();
    t_lidar.transform.rotation.y = q_lidar.y();
    t_lidar.transform.rotation.z = q_lidar.z();
    t_lidar.transform.rotation.w = q_lidar.w();
    transforms.push_back(t_lidar);

    geometry_msgs::msg::TransformStamped t_imu;
    t_imu.header.stamp = now();
    t_imu.header.frame_id = "base_link";
    t_imu.child_frame_id = "imu_link";
    t_imu.transform.rotation.w = 1.0;
    transforms.push_back(t_imu);

    geometry_msgs::msg::TransformStamped t_body;
    t_body.header.stamp = now();
    t_body.header.frame_id = "base_link";
    t_body.child_frame_id = "body";
    t_body.transform.rotation.w = 1.0;
    transforms.push_back(t_body);

    static_tf_br_->sendTransform(transforms);
  }

  void onLowState(const unitree_go::msg::LowState::SharedPtr msg)
  {
    rclcpp::Time current_time = now();
    const double now_s = current_time.seconds();

    auto imu = std::make_unique<sensor_msgs::msg::Imu>();
    imu->header.frame_id = frame_id_;
    imu->header.stamp = current_time;

    const auto & s = msg->imu_state;
    imu->orientation.w = s.quaternion[0];
    imu->orientation.x = s.quaternion[1];
    imu->orientation.y = s.quaternion[2];
    imu->orientation.z = s.quaternion[3];
    imu->angular_velocity.x = s.gyroscope[0];
    imu->angular_velocity.y = s.gyroscope[1];
    imu->angular_velocity.z = s.gyroscope[2];
    imu->linear_acceleration.x = s.accelerometer[0];
    imu->linear_acceleration.y = s.accelerometer[1];
    imu->linear_acceleration.z = s.accelerometer[2];

    pub_->publish(std::move(imu));
    ++count_;

    if (!publish_stab_) {
      return;
    }

    const double imu_roll = static_cast<double>(s.rpy[0]);
    const double imu_pitch = static_cast<double>(s.rpy[1]);
    double roll_out = imu_roll;
    double pitch_out = imu_pitch;

    if (use_kinematics_filter_) {
      std::array<double, 12> motor_q;
      for (size_t i = 0; i < 12; ++i) {
        motor_q[i] = msg->motor_state[i].q;
      }

      std::array<int16_t, 4> foot_force;
      for (size_t i = 0; i < 4; ++i) {
        foot_force[i] = msg->foot_force[i];
      }

      auto kin_res = kinematics_.compute(
        motor_q, foot_force, static_cast<int16_t>(contact_force_thresh_));

      double ref_roll = imu_roll;
      double ref_pitch = imu_pitch;

      if (kin_res.valid) {
        latest_height_ = kin_res.height_kin;
        latest_stance_ = kin_res.stance_count;
        latest_hill_pitch_deg_ = (imu_pitch - kin_res.pitch_kin) * 180.0 / M_PI;

        const double max_diff_rad = max_kin_imu_diff_deg_ * M_PI / 180.0;
        if (stab_mode_ == "terrain") {
          ref_roll = kin_res.roll_kin;
          ref_pitch = kin_res.pitch_kin;
        } else {
          // Suppress gait impact jitter via FK while clamping divergence on sloped terrain
          ref_roll = std::clamp(kin_res.roll_kin, imu_roll - max_diff_rad, imu_roll + max_diff_rad);
          ref_pitch = std::clamp(kin_res.pitch_kin, imu_pitch - max_diff_rad, imu_pitch + max_diff_rad);
        }
      }

      if (!initialized_filter_) {
        filtered_roll_ = ref_roll;
        filtered_pitch_ = ref_pitch;
        last_filter_time_s_ = now_s;
        initialized_filter_ = true;
      } else {
        const double dt = now_s - last_filter_time_s_;
        if (dt > 0.0 && dt < 0.1) {
          const double gyro_roll = filtered_roll_ + s.gyroscope[0] * dt;
          const double gyro_pitch = filtered_pitch_ + s.gyroscope[1] * dt;

          filtered_roll_ = filter_alpha_ * gyro_roll + (1.0 - filter_alpha_) * ref_roll;
          filtered_pitch_ = filter_alpha_ * gyro_pitch + (1.0 - filter_alpha_) * ref_pitch;
        } else {
          filtered_roll_ = ref_roll;
          filtered_pitch_ = ref_pitch;
        }
        last_filter_time_s_ = now_s;
      }

      roll_out = filtered_roll_;
      pitch_out = filtered_pitch_;
    }

    publishStab(roll_out, pitch_out, now_s);
  }

  void publishStab(double roll, double pitch, double now_s)
  {
    if (stab_min_dt_ > 0.0 && (now_s - last_stab_s_) < stab_min_dt_) {
      return;
    }
    last_stab_s_ = now_s;

    const double a = -roll * 0.5;
    const double b = -pitch * 0.5;
    const double ca = std::cos(a);
    const double sa = std::sin(a);
    const double cb = std::cos(b);
    const double sb = std::sin(b);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = secToTime(mono_stab_.next(now_s));
    tf_msg.header.frame_id = stab_parent_;
    tf_msg.child_frame_id = stab_child_;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.w = ca * cb;
    tf_msg.transform.rotation.x = sa * cb;
    tf_msg.transform.rotation.y = ca * sb;
    tf_msg.transform.rotation.z = sa * sb;

    tf_br_->sendTransform(tf_msg);
  }

  void report()
  {
    if (count_ == 0) {
      RCLCPP_WARN(get_logger(), "waiting for /lowstate on %s", in_topic_.c_str());
    } else {
      const double hz = static_cast<double>(count_) / 5.0;
      RCLCPP_INFO(
        get_logger(),
        "imu_bridge: %.1f Hz | stab_rp=(%+.2f, %+.2f) deg | slope=%+.1f deg | h=%.3fm (stance=%d/4)",
        hz, filtered_roll_ * 180.0 / M_PI, filtered_pitch_ * 180.0 / M_PI,
        latest_hill_pitch_deg_, latest_height_, latest_stance_);
    }
    count_ = 0;
  }

  std::string in_topic_;
  std::string out_topic_;
  std::string frame_id_;
  bool publish_static_tf_{true};
  bool publish_stab_{true};
  std::string stab_parent_;
  std::string stab_child_;
  double stab_rate_{50.0};
  double stab_min_dt_{0.02};
  double last_stab_s_{0.0};
  MonotonicStamp mono_stab_;

  double lidar_x_{0.2500};
  double lidar_y_{0.0000};
  double lidar_z_{0.1588};
  double lidar_roll_{0.012330};
  double lidar_pitch_{0.005579};
  double lidar_yaw_{1.57079632679};

  std::string stab_mode_{"gravity"};
  bool use_kinematics_filter_{true};
  double filter_alpha_{0.96};
  int contact_force_thresh_{15};
  double max_kin_imu_diff_deg_{6.0};
  Go2Kinematics kinematics_;
  bool initialized_filter_{false};
  double filtered_roll_{0.0};
  double filtered_pitch_{0.0};
  double last_filter_time_s_{0.0};
  double latest_height_{0.28};
  double latest_hill_pitch_deg_{0.0};
  int latest_stance_{4};

  size_t count_{0};
  rclcpp::TimerBase::SharedPtr report_timer_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_;
};

}  // namespace go2_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2_bridge::ImuBridgeNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
