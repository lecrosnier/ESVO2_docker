#ifndef ESVO2_CORE_TOOLS_STILLNESS_H
#define ESVO2_CORE_TOOLS_STILLNESS_H

#include <algorithm>
#include <deque>
#include <Eigen/Core>

namespace esvo2_core
{
namespace tools
{
struct ImuSample
{
  double t;          // s, IMU clock
  Eigen::Vector3d w; // rad/s
  Eigen::Vector3d a; // m/s^2
};

enum class Motion
{
  STILL,
  MOVING,
  UNKNOWN // no usable IMU data: never treat as still
};

inline const char *motionName(Motion m)
{
  return m == Motion::STILL ? "STILL" : (m == Motion::MOVING ? "MOVING" : "UNKNOWN");
}

// Classifies the rig as still or moving from the last window_s seconds of IMU samples.
// STILL needs every gyro axis' std below gyro_max_std, every gyro axis' mean below
// gyro_max_abs_mean in magnitude (a steady slow turn has almost no std), and every
// accelerometer axis' std below acc_max_std (rolling a cart straight barely rotates it, but
// shakes it). The answer is UNKNOWN, never STILL, when the data cannot support a decision:
// no sample within max_age of the query time (IMU unplugged, driver dead, stream stalled),
// less than 90% of a window buffered, two consecutive samples more than max_gap apart, or
// a window with no noise at all on either sensor (a driver repeating its last value: a real
// MEMS IMU at rest still shows ~1e-4 rad/s and ~1e-3 m/s^2 of noise).
// Samples must arrive in time order; an older one resets the buffer (clock jump).
class StillnessDetector
{
public:
  StillnessDetector(double window_s = 0.5, double gyro_max_std = 0.0035, double acc_max_std = 0.03,
                    double gyro_max_abs_mean = 0.02, double max_gap = 0.02, double max_age = 0.1)
    : window_s_(window_s), gyro_max_std_(gyro_max_std), acc_max_std_(acc_max_std),
      gyro_max_abs_mean_(gyro_max_abs_mean), max_gap_(max_gap), max_age_(max_age)
  {}

  void add(const ImuSample &s)
  {
    if (!buf_.empty() && s.t <= buf_.back().t)
    {
      if (s.t < buf_.back().t - window_s_)
        buf_.clear(); // clock jumped backwards
      else
        return; // duplicate or slightly out of order: drop
    }
    buf_.push_back(s);
    while (buf_.size() > 1 && buf_.front().t < s.t - window_s_)
      buf_.pop_front();
  }

  // t_now in the IMU clock.
  Motion classify(double t_now)
  {
    gyro_std_ = acc_std_ = gyro_abs_mean_ = -1.0;
    if (buf_.empty() || t_now - buf_.back().t > max_age_ || buf_.back().t - t_now > max_age_)
      return Motion::UNKNOWN;
    if (buf_.size() < 2 || buf_.back().t - buf_.front().t < 0.9 * window_s_)
      return Motion::UNKNOWN;
    // Two passes: a one-pass sum of squares loses the noise to rounding next to 9.8 m/s^2.
    Eigen::Vector3d mw = Eigen::Vector3d::Zero(), ma = mw;
    for (size_t i = 0; i < buf_.size(); i++)
    {
      if (i > 0 && buf_[i].t - buf_[i - 1].t > max_gap_)
        return Motion::UNKNOWN;
      mw += buf_[i].w;
      ma += buf_[i].a;
    }
    const double n = static_cast<double>(buf_.size());
    mw /= n;
    ma /= n;
    Eigen::Vector3d vw = Eigen::Vector3d::Zero(), va = vw;
    for (const auto &s : buf_)
    {
      vw += (s.w - mw).cwiseAbs2();
      va += (s.a - ma).cwiseAbs2();
    }
    gyro_std_ = (vw / n).cwiseSqrt().maxCoeff();
    acc_std_ = (va / n).cwiseSqrt().maxCoeff();
    gyro_abs_mean_ = mw.cwiseAbs().maxCoeff();
    if (gyro_std_ < kFrozenStd && acc_std_ < kFrozenStd)
      return Motion::UNKNOWN;
    const bool still = gyro_std_ < gyro_max_std_ && acc_std_ < acc_max_std_ && gyro_abs_mean_ < gyro_max_abs_mean_;
    return still ? Motion::STILL : Motion::MOVING;
  }

  // Statistics of the last classify() call; -1 when it returned UNKNOWN before computing them.
  double gyroStd() const { return gyro_std_; }
  double accStd() const { return acc_std_; }
  double gyroAbsMean() const { return gyro_abs_mean_; }
  // Age of the newest sample at t_now (IMU clock); infinity when there is none.
  double age(double t_now) const { return buf_.empty() ? 1e9 : t_now - buf_.back().t; }

private:
  static constexpr double kFrozenStd = 1e-7;
  double window_s_, gyro_max_std_, acc_max_std_, gyro_max_abs_mean_, max_gap_, max_age_;
  std::deque<ImuSample> buf_;
  double gyro_std_ = -1.0, acc_std_ = -1.0, gyro_abs_mean_ = -1.0;
};
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_STILLNESS_H
