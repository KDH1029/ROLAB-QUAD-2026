#ifndef GO2_BRIDGE__CLOCK_ALIGNER_HPP_
#define GO2_BRIDGE__CLOCK_ALIGNER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"

namespace go2_bridge
{

/**
 * @brief Convert builtin_interfaces::msg::Time to double seconds.
 */
inline double timeToSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

/**
 * @brief Convert double seconds safely to builtin_interfaces::msg::Time
 *        handling fractional nanosecond rounding and carry-over.
 */
inline builtin_interfaces::msg::Time secToTime(double s)
{
  builtin_interfaces::msg::Time t;
  if (s < 0.0) {
    s = 0.0;
  }
  double sec_part = std::floor(s);
  int64_t sec = static_cast<int64_t>(sec_part);
  int64_t nsec = static_cast<int64_t>(std::llround((s - sec_part) * 1e9));
  if (nsec >= 1000000000LL) {
    sec += 1;
    nsec -= 1000000000LL;
  } else if (nsec < 0) {
    nsec = 0;
  }
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(nsec);
  return t;
}

/**
 * @brief Sliding-window minimum offset estimator (Cristian's algorithm variant).
 *        Aligns Unitree MCU clock timestamps with host ROS clock without network jitter.
 */
class ClockAligner
{
public:
  explicit ClockAligner(double window_s = 10.0)
  : window_s_(window_s) {}

  double update(double src_time_s, double arrival_s)
  {
    const double inst = arrival_s - src_time_s;
    samples_.emplace_back(arrival_s, inst);

    while (!samples_.empty() && arrival_s - samples_.front().first > window_s_) {
      samples_.pop_front();
    }

    double m = samples_.front().second;
    for (const auto & s : samples_) {
      m = std::min(m, s.second);
    }
    offset_ = m;
    initialized_ = true;
    return offset_;
  }

  bool initialized() const { return initialized_; }
  double offset() const { return offset_; }
  size_t sample_count() const { return samples_.size(); }

private:
  double window_s_;
  std::deque<std::pair<double, double>> samples_;   // (arrival, inst_offset)
  double offset_{0.0};
  bool initialized_{false};
};

/**
 * @brief Ensures strictly monotonic timestamps to prevent TF_OLD_DATA / duplicate stamp warnings.
 */
class MonotonicStamp
{
public:
  explicit MonotonicStamp(double min_step_s = 1e-6)
  : min_step_(min_step_s) {}

  double next(double t)
  {
    if (have_last_ && t <= last_) {
      ++corrections_;
      t = last_ + min_step_;
    }
    last_ = t;
    have_last_ = true;
    return t;
  }

  size_t corrections() const { return corrections_; }
  void reset_corrections() { corrections_ = 0; }

private:
  double min_step_;
  double last_{0.0};
  bool have_last_{false};
  size_t corrections_{0};
};

}  // namespace go2_bridge

#endif  // GO2_BRIDGE__CLOCK_ALIGNER_HPP_
