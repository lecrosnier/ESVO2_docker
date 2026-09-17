#ifndef ESVO2_CORE_TOOLS_GYRO_PREDICTION_H
#define ESVO2_CORE_TOOLS_GYRO_PREDICTION_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace esvo2_core
{
namespace tools
{
// One gyro sample: t = /imu/data_synced stamp (s), w = angular velocity in the IMU frame (rad/s).
struct GyroSample
{
  double t;
  Eigen::Vector3d w;
};

inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &v)
{
  double th = v.norm();
  if (th < 1e-12)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, v / th).toRotationMatrix();
}

// Integrates gyro samples over camera-time interval (t_from, t_to].
// A sample at IMU time s belongs to camera time s + t_d and is held until the
// next sample (zero-order hold); the sample preceding t_from covers the start.
// R_imu = IMU orientation at t_to expressed in the IMU frame at t_from.
// Returns false if the samples do not reach within 20 ms of both ends, the
// interval is empty, or it is longer than 0.2 s.
inline bool gyroDeltaRotation(const std::vector<GyroSample> &samples, double t_from, double t_to, double t_d,
                              Eigen::Matrix3d &R_imu)
{
  const double kMaxGap = 0.02, kMaxInterval = 0.2;
  if (samples.empty() || !(t_to > t_from) || t_to - t_from > kMaxInterval)
    return false;
  if (samples.front().t + t_d > t_from + kMaxGap || samples.back().t + t_d < t_to - kMaxGap)
    return false;

  // first sample whose hold interval overlaps t_from: last sample with camera time <= t_from, else the first one
  size_t i = 0;
  while (i + 1 < samples.size() && samples[i + 1].t + t_d <= t_from)
    i++;

  R_imu.setIdentity();
  double t = t_from;
  for (; i < samples.size() && t < t_to; i++)
  {
    double seg_end = (i + 1 < samples.size()) ? samples[i + 1].t + t_d : t_to;
    seg_end = std::min(seg_end, t_to);
    if (seg_end > t)
    {
      R_imu = R_imu * so3Exp(samples[i].w * (seg_end - t));
      t = seg_end;
    }
  }
  return true;
}

// Camera rotation from IMU rotation with p_imu = R_b_c * p_cam.
inline Eigen::Matrix3d imuToCameraRotation(const Eigen::Matrix3d &R_imu, const Eigen::Matrix3d &R_b_c)
{
  return R_b_c.transpose() * R_imu * R_b_c;
}
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_GYRO_PREDICTION_H
