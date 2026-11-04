// Route sequencing logic (no ROS): which goal EGO-Planner gets next, when a step counts as done,
// and the stop-line / parking actions. route_runner_node feeds it the pose and commands every tick
// and carries out what it returns; test_route_logic drives it with a simulated robot.
// The traffic light is not handled here (vision is the teammate's, removed 11-05).
//
// A step = a saved map point + what to do there:
//   none      drive through: the next goal is sent once the robot is within pass_radius
//   stopline  arrive, stand still >= stop_hold_s (rulebook 2 s + margin), then go on
//   parking   arrive, stand still, Sit (1009) for sit_hold_s (rulebook 5 s + margin), RiseSit (1010)
//   finish    arrive -> done
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace route
{
enum class Action { NONE, STOPLINE, PARKING, FINISH };

inline Action parseAction(const std::string & s)
{
  if (s == "stopline") return Action::STOPLINE;
  if (s == "parking") return Action::PARKING;
  if (s == "finish") return Action::FINISH;
  return Action::NONE;
}

inline const char * actionName(Action a)
{
  switch (a) {
    case Action::STOPLINE: return "stopline";
    case Action::PARKING: return "parking";
    case Action::FINISH: return "finish";
    default: return "none";
  }
}

struct Step
{
  std::string name;          // point name (a point can appear in several steps: revisits)
  double x{0}, y{0}, yaw{0}; // map frame, yaw in rad
  Action action{Action::NONE};
  double goal_tol{0.25};     // EGO goal_tol for this step (m)
  double max_vel{0.6};       // EGO max_vel for this step (m/s)
};

struct Config
{
  double pass_radius{0.50};      // drive-through: next goal once this close (m)
  double arrive_margin{0.05};    // stop steps: done when within goal_tol + this (m)
  double still_speed{0.05};      // m/s below which the robot counts as standing still
  double still_time_s{0.5};      // stop steps: still this long before the action starts
  double stop_hold_s{2.3};       // stop line: standing still (rule: >= 2 s)
  double sit_hold_s{5.5};        // parking: sitting (rule: >= 5 s)
  double rise_s{3.0};            // parking: time for RiseSit before driving on
};

enum class State { IDLE, DRIVE, SETTLE, HOLD, SIT, RISE, DONE, STOPPED };

inline const char * stateName(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::DRIVE: return "DRIVE";
    case State::SETTLE: return "SETTLE";
    case State::HOLD: return "HOLD";
    case State::SIT: return "SIT";
    case State::RISE: return "RISE";
    case State::DONE: return "DONE";
    default: return "STOPPED";
  }
}

struct Input
{
  double t{0};                   // s, monotonic
  bool pose_ok{false};
  double x{0}, y{0};             // map frame
  double speed{0};               // m/s (from successive poses)
};

struct Output
{
  bool send_goal{false};         // publish /goal_pose = steps[step]
  bool set_params{false};        // set EGO goal_tol / max_vel of steps[step] (before the goal)
  int sport_api{0};              // 1009 Sit / 1010 RiseSit, 0 = none
  bool stop_motion{false};       // software stop (deactivate the motor bridge)
  bool resume_motion{false};     // re-activate the motor bridge
  std::string event;             // one log line when something happened
};

class Logic
{
public:
  Logic(std::vector<Step> steps, Config cfg) : steps_(std::move(steps)), cfg_(cfg) {}

  // Commands from the operator: START [i] (from step i), STOP (software stop, keeps the step),
  // RESUME (drive on from the current step, e.g. after a maintenance pause - rule 2.5).
  Output start(double t, size_t from = 0)
  {
    Output o;
    if (from >= steps_.size()) {o.event = "START refused: no step " + std::to_string(from); return o;}
    idx_ = from;
    o = beginStep(t);
    o.resume_motion = true;
    o.event = "START at step " + std::to_string(idx_) + " " + steps_[idx_].name + " | " + o.event;
    return o;
  }

  Output stop(const std::string & why)
  {
    Output o;
    if (state_ == State::STOPPED) return o;
    resume_state_ = state_;
    state_ = State::STOPPED;
    o.stop_motion = true;
    o.event = "STOP (" + why + ") at step " + std::to_string(idx_) + " " + stepName();
    return o;
  }

  Output resume(double t)
  {
    Output o;
    if (state_ != State::STOPPED) {o.event = "RESUME ignored: not stopped"; return o;}
    if (resume_state_ == State::IDLE || resume_state_ == State::DONE) {
      state_ = resume_state_;
      o.event = "RESUME: nothing to do (" + std::string(stateName(state_)) + ")";
      return o;
    }
    // Re-run the current step from the start: a stop line holds again, parking sits again.
    o = beginStep(t);
    o.resume_motion = true;
    o.event = "RESUME step " + std::to_string(idx_) + " " + stepName() + " | " + o.event;
    return o;
  }

  Output tick(const Input & in)
  {
    Output o;
    if (state_ == State::IDLE || state_ == State::DONE || state_ == State::STOPPED || !in.pose_ok) return o;
    const Step & s = steps_[idx_];
    const double dist = std::hypot(s.x - in.x, s.y - in.y);
    const bool still = in.speed < cfg_.still_speed;

    switch (state_) {
      case State::DRIVE:
        if (s.action == Action::NONE) {
          if (dist <= std::max(cfg_.pass_radius, s.goal_tol)) return next(in.t, "passed " + s.name);
        } else if (dist <= s.goal_tol + cfg_.arrive_margin) {
          state_ = State::SETTLE;
          t0_ = in.t;
          o.event = "arrived " + s.name + " (" + fmt(dist) + " m)";
        }
        return o;

      case State::SETTLE:
        if (!still) {t0_ = in.t; return o;}
        if (in.t - t0_ < cfg_.still_time_s) return o;
        if (s.action == Action::FINISH) {
          state_ = State::DONE;
          o.event = "FINISH at " + s.name;
          return o;
        }
        if (s.action == Action::STOPLINE) {
          state_ = State::HOLD;
          t0_ = in.t - cfg_.still_time_s;   // still since the start of SETTLE
          o.event = "stop line " + s.name + ": holding";
          return o;
        }
        state_ = State::SIT;               // PARKING
        t0_ = in.t;
        o.sport_api = 1009;
        o.event = "parking " + s.name + ": Sit (1009)";
        return o;

      case State::HOLD:
        if (!still) {t0_ = in.t; o.event = "stop line: moved, hold restarts"; return o;}
        if (in.t - t0_ < cfg_.stop_hold_s) return o;
        return next(in.t, "stop line " + s.name + ": held " + fmt(in.t - t0_) + " s, going on");

      case State::SIT:
        if (in.t - t0_ < cfg_.sit_hold_s) return o;
        state_ = State::RISE;
        t0_ = in.t;
        o.sport_api = 1010;
        o.event = "parking: sat " + fmt(cfg_.sit_hold_s) + " s, RiseSit (1010)";
        return o;

      case State::RISE:
        if (in.t - t0_ < cfg_.rise_s) return o;
        return next(in.t, "parking done");

      default:
        return o;
    }
  }

  State state() const {return state_;}
  size_t index() const {return idx_;}
  const Step & current() const {return steps_[idx_];}
  const std::vector<Step> & steps() const {return steps_;}
  std::string stepName() const {return idx_ < steps_.size() ? steps_[idx_].name : "-";}

private:
  Output beginStep(double t)
  {
    Output o;
    state_ = State::DRIVE;
    t0_ = t;
    o.set_params = true;
    o.send_goal = true;
    const Step & s = steps_[idx_];
    o.event = "goal " + s.name + " (" + fmt(s.x) + ", " + fmt(s.y) + ") " + actionName(s.action) +
      " tol " + fmt(s.goal_tol) + " vmax " + fmt(s.max_vel);
    return o;
  }

  Output next(double t, const std::string & why)
  {
    if (idx_ + 1 >= steps_.size()) {
      state_ = State::DONE;
      Output o;
      o.event = why + " | route complete";
      return o;
    }
    ++idx_;
    Output o = beginStep(t);
    o.event = why + " | " + o.event;
    return o;
  }

  static std::string fmt(double v)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.2f", v);
    return b;
  }

  std::vector<Step> steps_;
  Config cfg_;
  State state_{State::IDLE}, resume_state_{State::IDLE};
  size_t idx_{0};
  double t0_{0};
};
}  // namespace route
