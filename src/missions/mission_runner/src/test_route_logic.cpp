// Simulated run of route::Logic: a robot that drives straight to the current goal (EGO stand-in,
// stops at goal_tol), a traffic light that alternates every 5 s and is seen 0.3 s late, a revisited
// point, a STOP / RESUME, and START from a later step. Checks the rule timings; exit 1 on a failure.
//   ros2 run mission_runner test_route_logic
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "mission_runner/route_logic.hpp"

using namespace route;

namespace
{
int fails = 0;
void check(bool ok, const std::string & what)
{
  std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what.c_str());
  if (!ok) ++fails;
}

struct Sim
{
  double x{0}, y{0}, gx{0}, gy{0}, tol{0.25}, vmax{0.6};
  bool has_goal{false}, motors{true}, sitting{false};
  double speed{0};
  void step(double dt)
  {
    speed = 0;
    if (!has_goal || !motors || sitting) return;
    const double dx = gx - x, dy = gy - y, d = std::hypot(dx, dy);
    if (d <= tol) {has_goal = false; return;}      // EGO: goal reached -> stop
    const double s = std::min(vmax * dt, d);
    x += s * dx / d;
    y += s * dy / d;
    speed = s / dt;
  }
};

struct Log
{
  double t;
  std::string ev;
};

std::vector<Log> run(Logic & L, Sim & sim, double t_end, double light_phase, std::vector<double> stop_at = {},
                     double resume_after = 0, std::vector<int> * sport = nullptr, std::vector<double> * sport_t = nullptr)
{
  std::vector<Log> log;
  const double dt = 0.05;
  double last_seen_change = -1e9;
  bool seen_green = false;
  auto apply = [&](const Output & o, double t) {
      if (o.set_params) {sim.tol = L.current().goal_tol; sim.vmax = L.current().max_vel;}
      if (o.send_goal) {sim.gx = L.current().x; sim.gy = L.current().y; sim.has_goal = true;}
      if (o.stop_motion) sim.motors = false;
      if (o.resume_motion) sim.motors = true;
      if (o.sport_api == 1009) sim.sitting = true;
      if (o.sport_api == 1010) sim.sitting = false;
      if (o.sport_api && sport) {sport->push_back(o.sport_api); sport_t->push_back(t);}
      if (!o.event.empty()) log.push_back({t, o.event});
    };
  apply(L.start(0.0), 0.0);
  double stopped_t = -1;
  for (double t = dt; t < t_end && L.state() != State::DONE; t += dt) {
    sim.step(dt);
    // traffic light: GREEN for 5 s, RED for 5 s; the camera reports it 0.3 s late, at 5 Hz
    const bool green_now = std::fmod(t - 0.3 + light_phase, 10.0) < 5.0;
    if (std::fmod(t, 0.2) < dt) {seen_green = green_now; last_seen_change = t;}
    for (double s : stop_at) {
      if (std::abs(t - s) < dt / 2) {apply(L.stop("test"), t); stopped_t = t;}
    }
    if (stopped_t > 0 && resume_after > 0 && t - stopped_t >= resume_after && L.state() == State::STOPPED) {
      apply(L.resume(t), t);
      stopped_t = -1;
    }
    Input in;
    in.t = t; in.pose_ok = true; in.x = sim.x; in.y = sim.y; in.speed = sim.speed;
    in.green = seen_green; in.signal_age_s = t - last_seen_change;
    apply(L.tick(in), t);
  }
  return log;
}

std::vector<Step> course()
{
  // start -> top (pass) -> stop1 (stopline) -> slalom (pass) -> top (revisit, pass) -> park -> out
  auto S = [](const char * n, double x, double y, Action a, double tol = 0.25, double v = 0.6) {
      Step s; s.name = n; s.x = x; s.y = y; s.action = a; s.goal_tol = tol; s.max_vel = v; return s;
    };
  return {S("top", 2.0, 0.0, Action::NONE), S("stop1", 3.0, 1.0, Action::STOPLINE, 0.10, 0.3),
          S("slalom", 5.0, 1.0, Action::NONE), S("top", 2.0, 0.0, Action::NONE),
          S("park", 1.0, -1.5, Action::PARKING, 0.10, 0.3), S("out", 0.0, -2.0, Action::FINISH)};
}

double eventTime(const std::vector<Log> & log, const std::string & needle, int nth = 0)
{
  for (const auto & l : log) {
    if (l.ev.find(needle) != std::string::npos && nth-- == 0) return l.t;
  }
  return -1;
}
}  // namespace

int main()
{
  Config cfg;
  std::printf("1) full course at three light phases (arriving on GREEN, on RED, just before a change)\n");
  for (double phase : {0.0, 2.5, 7.0}) {
    Logic L(course(), cfg);
    Sim sim;
    std::vector<int> api;
    std::vector<double> api_t;
    auto log = run(L, sim, 200, phase, {}, 0, &api, &api_t);
    std::printf(" phase %.1f s:\n", phase);
    for (const auto & l : log) std::printf("    %6.2f  %s\n", l.t, l.ev.c_str());
    check(L.state() == State::DONE, "route completes");
    // the hold counts from when the robot came to rest: still_time_s before the "holding" line
    const double rest = eventTime(log, "stop line stop1: holding") - cfg.still_time_s;
    const double leave = eventTime(log, "GREEN at stop1");
    check(rest > 0 && leave - rest >= cfg.stop_hold_s - 1e-6, "stop line: still >= 2.3 s before leaving");
    // the camera reports the light 0.3 s late: the GREEN it acted on was the light at leave - 0.3 s
    check(std::fmod(leave - 0.3 + phase, 10.0) < 5.0, "stop line: left on a real GREEN");
    check(api.size() == 2 && api[0] == 1009 && api[1] == 1010 && api_t[1] - api_t[0] >= cfg.sit_hold_s - 1e-6,
          "parking: Sit 1009, then RiseSit 1010 after >= 5.5 s");
    check(eventTime(log, "passed top", 1) > 0, "revisited point 'top' passed twice");
  }

  std::printf("2) light never GREEN -> proceed after the timeout\n");
  {
    Sim sim;
    std::vector<Log> log;
    Config c2 = cfg;
    c2.signal_max_age_s = -1;  // nothing counts as fresh
    Logic L2(course(), c2);
    log = run(L2, sim, 200, 0);
    const double rest = eventTime(log, "stop line stop1: holding") - c2.still_time_s, go = eventTime(log, "no GREEN");
    check(go > 0 && std::abs(go - rest - c2.traffic_timeout_s) < 0.1, "proceeds 15 s after the robot stopped");
    check(L2.state() == State::DONE, "route still completes");
  }

  std::printf("2b) no traffic-light messages at all (camera off) -> proceed right after the stop\n");
  {
    Logic L(course(), cfg);
    Sim sim;
    // run() always feeds a light; build the no-signal case by hand: signal age grows forever
    std::vector<Log> log;
    auto apply = [&](const Output & o, double t) {
        if (o.set_params) {sim.tol = L.current().goal_tol; sim.vmax = L.current().max_vel;}
        if (o.send_goal) {sim.gx = L.current().x; sim.gy = L.current().y; sim.has_goal = true;}
        if (o.sport_api == 1009) sim.sitting = true;
        if (o.sport_api == 1010) sim.sitting = false;
        if (!o.event.empty()) log.push_back({t, o.event});
      };
    apply(L.start(0.0), 0.0);
    for (double t = 0.05; t < 200 && L.state() != State::DONE; t += 0.05) {
      sim.step(0.05);
      Input in;
      in.t = t; in.pose_ok = true; in.x = sim.x; in.y = sim.y; in.speed = sim.speed;
      in.green = false; in.signal_age_s = 1e9;
      apply(L.tick(in), t);
    }
    const double rest = eventTime(log, "stop line stop1: holding") - cfg.still_time_s;
    const double go = eventTime(log, "no traffic-light signal");
    check(go > 0 && go - rest >= cfg.stop_hold_s && std::abs(go - rest - cfg.no_signal_wait_s) < 0.1,
          "no signal: still >= 2.3 s, proceeds 2.8 s after stopping (not 15 s)");
    check(L.state() == State::DONE, "route completes without a camera");
  }

  std::printf("3) STOP while driving, RESUME 3 s later\n");
  {
    Logic L(course(), cfg);
    Sim sim;
    auto log = run(L, sim, 200, 2.5, {6.0}, 3.0);
    for (const auto & l : log) if (l.t > 5.5 && l.t < 10) std::printf("    %6.2f  %s\n", l.t, l.ev.c_str());
    check(eventTime(log, "STOP (test)") > 0 && eventTime(log, "RESUME step") > 0, "stopped and resumed");
    check(L.state() == State::DONE, "route completes after the resume");
  }

  std::printf("4) START from step 4 (after a maintenance pause near the parking)\n");
  {
    Logic L(course(), cfg);
    auto o = L.start(0.0, 4);
    check(o.send_goal && L.current().name == "park", "first goal is 'park'");
    auto bad = L.start(0.0, 9);
    check(!bad.send_goal, "START past the last step is refused");
  }
  std::printf(fails ? "FAIL (%d)\n" : "OK\n", fails);
  return fails ? 1 : 0;
}
