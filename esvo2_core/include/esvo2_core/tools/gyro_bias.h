#ifndef ESVO2_CORE_TOOLS_GYRO_BIAS_H
#define ESVO2_CORE_TOOLS_GYRO_BIAS_H

#include <vector>
#include <Eigen/Core>
#include <esvo2_core/tools/gyro_prediction.h>

namespace esvo2_core
{
namespace tools
{
// Estimates a constant gyro bias from the first window in which the rig is still.
// Samples are grouped into consecutive, non-overlapping windows of window_s seconds
// (a window closes on the first sample at least window_s after its first sample).
// A window is accepted when every axis' standard deviation is below max_std and no two
// consecutive samples in it are more than max_gap apart. The first accepted window's mean
// is the bias; samples after that are ignored. A rejected window is discarded and the next
// one starts at the next sample.
class GyroBiasEstimator
{
public:
  GyroBiasEstimator(double window_s, double max_std, double max_gap = 0.02)
    : window_s_(window_s), max_std_(max_std), max_gap_(max_gap)
  {
    resetWindow();
  }

  // Returns true on the sample that completes the first accepted window.
  bool add(const GyroSample &s)
  {
    if (has_bias_)
      return false;
    if (n_ == 0)
      t_start_ = s.t;
    else if (s.t - t_last_ > max_gap_)
      gap_ = true;
    sum_ += s.w;
    sum_sq_ += s.w.cwiseProduct(s.w);
    n_++;
    t_last_ = s.t;
    if (s.t - t_start_ < window_s_)
      return false;

    const Eigen::Vector3d mean = sum_ / static_cast<double>(n_);
    const Eigen::Vector3d var = (sum_sq_ / static_cast<double>(n_) - mean.cwiseProduct(mean)).cwiseMax(0.0);
    const Eigen::Vector3d sd = var.cwiseSqrt();
    const bool still = !gap_ && n_ >= 2 && sd.maxCoeff() < max_std_;
    if (still)
    {
      bias_ = mean;
      bias_std_ = sd;
      has_bias_ = true;
    }
    resetWindow();
    return still;
  }

  bool hasBias() const { return has_bias_; }
  Eigen::Vector3d bias() const { return bias_; }
  Eigen::Vector3d biasStd() const { return bias_std_; }
  double window() const { return window_s_; }

private:
  void resetWindow()
  {
    sum_.setZero();
    sum_sq_.setZero();
    n_ = 0;
    gap_ = false;
  }

  double window_s_, max_std_, max_gap_;
  Eigen::Vector3d sum_, sum_sq_;
  size_t n_ = 0;
  double t_start_ = 0.0, t_last_ = 0.0;
  bool gap_ = false;
  bool has_bias_ = false;
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero(), bias_std_ = Eigen::Vector3d::Zero();
};

// Removes a constant bias from every sample (rates in the same frame as the bias).
inline void subtractBias(std::vector<GyroSample> &samples, const Eigen::Vector3d &bias)
{
  for (auto &s : samples)
    s.w -= bias;
}
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_GYRO_BIAS_H
