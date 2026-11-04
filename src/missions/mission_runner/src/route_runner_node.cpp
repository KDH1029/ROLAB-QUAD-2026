/**
 * @file route_runner_node.cpp
 * @brief Drives a saved route with EGO-Planner: one /goal_pose (map frame) per step, the next one
 *        once the step is done, stop-line and parking actions, per-step EGO goal_tol / max_vel.
 *
 * Route = a ROS parameter file (maps/route_<name>.yaml, points saved on site by
 * scripts/go2_route.py from the AMCL pose, never FAST-LIO):
 *   route_runner_node:
 *     ros__parameters:
 *       points:   {top: [x, y, yaw_deg], stop1: [...], ...}
 *       sequence: ["top", "stop1 stopline tol=0.10 vmax=0.30", "slalom", "top", "park parking", "out finish"]
 * A point may appear in several steps (revisits reuse the same coordinates).
 *
 * Decisions live in route_logic.hpp (tested by test_route_logic); this node feeds it the pose
 * (TF map -> base_link) and operator commands
 * (/route/cmd: "START [step]", "STOP", "RESUME"), and carries out its outputs:
 *   goal      -> set EGO goal_tol / max_vel first (parameter service), then /goal_pose
 *   sport     -> /api/sport/request (1009 Sit, 1010 RiseSit)
 *   stop      -> deactivate the motor bridge (go2_cmd_vel_to_sport: StopMove), resume -> activate
 * FAST-LIO watch: EGO steers with /Odometry; a jump (> lio.max_jump_m in one message, or faster
 * than lio.max_speed for lio.speed_hold_s) stops the route when lio.mode = "stop" ("log": warn only).
 * The thresholds are first guesses - stair impacts make FAST-LIO jump too; set them on the robot.
 * Nothing moves until START. Status: /route/status (2 Hz).
 */
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unitree_api/msg/request.hpp>

#include "mission_runner/route_logic.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RouteRunnerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RouteRunnerNode()
  : rclcpp_lifecycle::LifecycleNode("route_runner_node",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
  {
    auto p = [this](const std::string & n, auto def) {
        using T = decltype(def);
        if (!has_parameter(n)) declare_parameter<T>(n, def);
        return get_parameter(n).get_value<T>();
      };
    map_frame_ = p("map_frame", std::string("map"));
    base_frame_ = p("base_frame", std::string("base_link"));
    goal_topic_ = p("goal_topic", std::string("/goal_pose"));
    ego_node_ = p("ego_node", std::string("/ego_planner_node"));
    bridge_node_ = p("motor_bridge_node", std::string("/go2_cmd_vel_to_sport"));
    sport_topic_ = p("sport_topic", std::string("/api/sport/request"));
    default_tol_ = p("default_goal_tol", 0.25);
    default_vmax_ = p("default_max_vel", 0.6);
    lio_topic_ = p("lio.topic", std::string("/Odometry"));
    lio_mode_ = p("lio.mode", std::string("stop"));
    lio_max_jump_ = p("lio.max_jump_m", 0.5);
    lio_max_speed_ = p("lio.max_speed", 3.0);
    lio_speed_hold_ = p("lio.speed_hold_s", 0.3);
    cfg_.pass_radius = p("pass_radius", cfg_.pass_radius);
    cfg_.arrive_margin = p("arrive_margin", cfg_.arrive_margin);
    cfg_.still_speed = p("still_speed", cfg_.still_speed);
    cfg_.still_time_s = p("still_time_s", cfg_.still_time_s);
    cfg_.stop_hold_s = p("stop_hold_s", cfg_.stop_hold_s);
    cfg_.sit_hold_s = p("sit_hold_s", cfg_.sit_hold_s);
    cfg_.rise_s = p("rise_s", cfg_.rise_s);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::string err;
    auto steps = loadRoute(err);
    if (!err.empty()) {
      RCLCPP_ERROR(get_logger(), "route not loaded: %s", err.c_str());
      return CallbackReturn::FAILURE;
    }
    logic_ = std::make_unique<route::Logic>(steps, cfg_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10).reliable());
    sport_pub_ = create_publisher<unitree_api::msg::Request>(sport_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/route/status", 10);
    ego_params_ = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node_base_interface(), get_node_topics_interface(), get_node_graph_interface(),
      get_node_services_interface(), ego_node_);
    bridge_client_ = create_client<lifecycle_msgs::srv::ChangeState>(bridge_node_ + "/change_state");
    std::ostringstream s;
    for (size_t i = 0; i < steps.size(); ++i) {
      s << "\n  " << i << ": " << steps[i].name << " (" << steps[i].x << ", " << steps[i].y << ") "
        << route::actionName(steps[i].action) << " tol " << steps[i].goal_tol << " vmax " << steps[i].max_vel;
    }
    RCLCPP_INFO(get_logger(), "route: %zu steps%s", steps.size(), s.str().c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    goal_pub_->on_activate();
    sport_pub_->on_activate();
    status_pub_->on_activate();
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/route/cmd", 10, std::bind(&RouteRunnerNode::onCmd, this, std::placeholders::_1));
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_topic_, rclcpp::SensorDataQoS(), std::bind(&RouteRunnerNode::onLio, this, std::placeholders::_1));
    timer_ = create_wall_timer(50ms, std::bind(&RouteRunnerNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "route_runner active - waiting for START on /route/cmd (lio.mode %s)", lio_mode_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (logic_ && logic_->state() != route::State::IDLE && logic_->state() != route::State::DONE) {
      carryOut(logic_->stop("route_runner deactivated"));
    }
    timer_.reset(); cmd_sub_.reset(); lio_sub_.reset();
    goal_pub_->on_deactivate(); sport_pub_->on_deactivate(); status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    goal_pub_.reset(); sport_pub_.reset(); status_pub_.reset();
    tf_listener_.reset(); tf_buffer_.reset(); logic_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  double nowS() {return now().seconds();}

  // "stop1 stopline tol=0.10 vmax=0.30" -> Step (point coordinates from points.<name>)
  std::vector<route::Step> loadRoute(std::string & err)
  {
    std::vector<route::Step> steps;
    std::vector<std::string> seq;
    if (!has_parameter("sequence") || (seq = get_parameter("sequence").as_string_array()).empty()) {
      err = "no 'sequence' (pass the route file with --params-file)";
      return steps;
    }
    for (const auto & line : seq) {
      std::istringstream in(line);
      route::Step s;
      in >> s.name;
      s.goal_tol = default_tol_;
      s.max_vel = default_vmax_;
      std::string tok;
      while (in >> tok) {
        if (tok.rfind("tol=", 0) == 0) s.goal_tol = std::stod(tok.substr(4));
        else if (tok.rfind("vmax=", 0) == 0) s.max_vel = std::stod(tok.substr(5));
        else if (tok == "stopline" || tok == "parking" || tok == "finish" || tok == "none") s.action = route::parseAction(tok);
        else {err = "step '" + line + "': unknown '" + tok + "'"; return steps;}
      }
      const std::string key = "points." + s.name;
      if (!has_parameter(key)) {err = "step '" + line + "': no point '" + s.name + "' (save it with go2_route.py)"; return steps;}
      // hand-edited files may hold whole numbers ([2, 0, 90] is an integer array to ROS)
      const auto prm = get_parameter(key);
      std::vector<double> v;
      if (prm.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        for (auto i : prm.as_integer_array()) v.push_back(static_cast<double>(i));
      } else if (prm.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        v = prm.as_double_array();
      } else {
        err = key + " must be [x, y, yaw_deg]";
        return steps;
      }
      if (v.size() < 2) {err = key + " needs [x, y, yaw_deg]"; return steps;}
      s.x = v[0];
      s.y = v[1];
      s.yaw = v.size() > 2 ? v[2] * M_PI / 180.0 : 0.0;
      steps.push_back(s);
    }
    if (steps.back().action != route::Action::FINISH) {
      RCLCPP_WARN(get_logger(), "last step '%s' is not 'finish'", steps.back().name.c_str());
    }
    return steps;
  }

  void onCmd(const std_msgs::msg::String::SharedPtr m)
  {
    std::istringstream in(m->data);
    std::string c;
    in >> c;
    if (c == "START") {
      size_t from = 0;
      in >> from;
      carryOut(logic_->start(nowS(), from));
    } else if (c == "STOP") {
      carryOut(logic_->stop("operator"));
    } else if (c == "RESUME") {
      carryOut(logic_->resume(nowS()));
    } else {
      RCLCPP_WARN(get_logger(), "unknown /route/cmd '%s' (START [step] | STOP | RESUME)", m->data.c_str());
    }
  }

  void onLio(const nav_msgs::msg::Odometry::SharedPtr m)
  {
    const double t = rclcpp::Time(m->header.stamp).seconds();
    const auto & q = m->pose.pose.position;
    if (have_lio_) {
      const double dt = t - lio_t_;
      const double jump = std::hypot(q.x - lio_x_, q.y - lio_y_);
      std::string why;
      if (jump > lio_max_jump_) {
        why = "FAST-LIO jumped " + std::to_string(jump).substr(0, 4) + " m in one message";
      } else if (dt > 1e-3 && jump / dt > lio_max_speed_) {
        if (fast_since_ < 0) fast_since_ = t;
        if (t - fast_since_ >= lio_speed_hold_) why = "FAST-LIO speed " + std::to_string(jump / dt).substr(0, 4) + " m/s";
      } else {
        fast_since_ = -1;
      }
      if (!why.empty()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", why.c_str());
        const auto st = logic_->state();
        if (lio_mode_ == "stop" && st != route::State::IDLE && st != route::State::DONE && st != route::State::STOPPED) {
          carryOut(logic_->stop(why));
        }
      }
    }
    lio_x_ = q.x; lio_y_ = q.y; lio_t_ = t; have_lio_ = true;
  }

  void onTimer()
  {
    route::Input in;
    in.t = nowS();
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      in.pose_ok = true;
      in.x = tf.transform.translation.x;
      in.y = tf.transform.translation.y;
      // speed from successive poses over >= 0.2 s (TF can repeat the same stamp)
      const double ts = rclcpp::Time(tf.header.stamp).seconds();
      if (have_pose_ && ts - pose_t_ >= 0.2) {
        speed_ = std::hypot(in.x - pose_x_, in.y - pose_y_) / (ts - pose_t_);
        pose_x_ = in.x; pose_y_ = in.y; pose_t_ = ts;
      } else if (!have_pose_) {
        pose_x_ = in.x; pose_y_ = in.y; pose_t_ = ts; have_pose_ = true;
      }
      if (in.t - ts > 1.0) speed_ = 0.0;  // no fresh TF: nothing moves it either
      in.speed = speed_;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s -> %s: %s", map_frame_.c_str(),
                           base_frame_.c_str(), e.what());
    }
    carryOut(logic_->tick(in));
    if (++ticks_ % 10 == 0) {
      std_msgs::msg::String s;
      s.data = std::string(route::stateName(logic_->state())) + " step " + std::to_string(logic_->index()) + "/" +
        std::to_string(logic_->steps().size()) + " " + logic_->stepName() + " | speed " +
        std::to_string(in.speed).substr(0, 4);
      status_pub_->publish(s);
    }
  }

  void carryOut(const route::Output & o)
  {
    if (!o.event.empty()) RCLCPP_INFO(get_logger(), "%s", o.event.c_str());
    if (o.resume_motion) changeBridge(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (o.stop_motion) changeBridge(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    if (o.sport_api) {
      unitree_api::msg::Request r;
      r.header.identity.id = now().nanoseconds();
      r.header.identity.api_id = o.sport_api;
      r.parameter = "{}";
      sport_pub_->publish(r);
    }
    if (o.send_goal) {
      const route::Step s = logic_->current();
      auto publish_goal = [this, s]() {
          geometry_msgs::msg::PoseStamped g;
          g.header.frame_id = map_frame_;
          g.header.stamp = now();
          g.pose.position.x = s.x;
          g.pose.position.y = s.y;
          g.pose.orientation.z = std::sin(s.yaw / 2);
          g.pose.orientation.w = std::cos(s.yaw / 2);
          goal_pub_->publish(g);
        };
      if (o.set_params && ego_params_->service_is_ready()) {
        // EGO plans with the limits it has when the goal arrives: set them first, then send the goal
        ego_params_->set_parameters(
          {rclcpp::Parameter("goal_tol", s.goal_tol), rclcpp::Parameter("max_vel", s.max_vel)},
          [this, publish_goal](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
            for (const auto & r : f.get()) {
              if (!r.successful) RCLCPP_WARN(get_logger(), "EGO parameter refused: %s", r.reason.c_str());
            }
            publish_goal();
          });
      } else {
        if (o.set_params) RCLCPP_WARN(get_logger(), "%s parameters not reachable: goal sent with its current limits", ego_node_.c_str());
        publish_goal();
      }
    }
  }

  void changeBridge(uint8_t transition)
  {
    if (!bridge_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "%s/change_state not available - %s it by hand", bridge_node_.c_str(),
                   transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE ? "activate" : "STOP");
      return;
    }
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = transition;
    bridge_client_->async_send_request(req, [this, transition](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture f) {
        // activating an already active bridge is refused - harmless
        if (!f.get()->success && transition == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
          RCLCPP_ERROR(get_logger(), "motor bridge did not deactivate - check it");
        }
      });
  }

  // parameters
  std::string map_frame_, base_frame_, goal_topic_, ego_node_, bridge_node_, sport_topic_;
  std::string lio_topic_, lio_mode_;
  double default_tol_, default_vmax_, lio_max_jump_, lio_max_speed_, lio_speed_hold_;
  route::Config cfg_;
  // state
  std::unique_ptr<route::Logic> logic_;
  bool have_pose_{false};
  double pose_x_{0}, pose_y_{0}, pose_t_{0}, speed_{0};
  bool have_lio_{false};
  double lio_x_{0}, lio_y_{0}, lio_t_{0}, fast_since_{-1};
  size_t ticks_{0};
  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::AsyncParametersClient::SharedPtr ego_params_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr bridge_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RouteRunnerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
