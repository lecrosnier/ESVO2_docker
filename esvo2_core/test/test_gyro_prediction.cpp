#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <esvo2_core/tools/gyro_prediction.h>

using esvo2_core::tools::GyroSample;
using esvo2_core::tools::gyroDeltaRotation;
using esvo2_core::tools::imuToCameraRotation;
using esvo2_core::tools::so3Exp;

static std::vector<GyroSample> constantRate(double t0, double t1, double dt, const Eigen::Vector3d &w)
{
  std::vector<GyroSample> s;
  for (double t = t0; t <= t1 + 1e-12; t += dt)
    s.push_back({t, w});
  return s;
}

static double angleBetween(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
{
  return Eigen::AngleAxisd(A.transpose() * B).angle();
}

TEST(GyroPrediction, ConstantRateGivesExpectedAngle)
{
  Eigen::Vector3d w(0.1, -0.4, 0.2);
  auto s = constantRate(0.0, 1.0, 0.005, w);
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.4023, 0.4431, 0.0, R));
  EXPECT_LT(angleBetween(R, so3Exp(w * (0.4431 - 0.4023))), 1e-9);
}

TEST(GyroPrediction, ClipsPartialIntervalsAtBothEnds)
{
  std::vector<GyroSample> s = {{0.000, Eigen::Vector3d(0, 0, 1)}, {0.010, Eigen::Vector3d(0, 0, 2)}, {0.020, Eigen::Vector3d(0, 0, 4)}};
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.005, 0.015, 0.0, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 1.0 * 0.005 + 2.0 * 0.005, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(R).axis().z(), 1.0, 1e-9);
}

TEST(GyroPrediction, TimeOffsetShiftsSamples)
{
  // IMU rotates only during IMU time [1.000, 1.050); with t_d = 0.2 that is camera time [1.200, 1.250)
  std::vector<GyroSample> s;
  for (int i = 0; i <= 400; i++)
  {
    double t = 0.5 + 0.005 * i;
    s.push_back({t, (t >= 1.0 - 1e-9 && t < 1.05 - 1e-9) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d::Zero()});
  }
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 1.2, 1.25, 0.2, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 0.05, 1e-9);
  ASSERT_TRUE(gyroDeltaRotation(s, 1.0, 1.05, 0.2, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 0.0, 1e-12);
}

TEST(GyroPrediction, RejectsMissingCoverageAndLongIntervals)
{
  auto s = constantRate(1.0, 2.0, 0.005, Eigen::Vector3d(0, 1, 0));
  Eigen::Matrix3d R;
  EXPECT_FALSE(gyroDeltaRotation(s, 0.95, 1.05, 0.0, R));   // first sample 50 ms after t_from
  EXPECT_FALSE(gyroDeltaRotation(s, 1.95, 2.05, 0.0, R));   // last sample 50 ms before t_to
  EXPECT_TRUE(gyroDeltaRotation(s, 0.99, 1.05, 0.0, R));    // 10 ms gap is tolerated
  EXPECT_FALSE(gyroDeltaRotation(s, 1.1, 1.35, 0.0, R));    // longer than 0.2 s
  EXPECT_FALSE(gyroDeltaRotation(s, 1.2, 1.2, 0.0, R));     // empty interval
  EXPECT_FALSE(gyroDeltaRotation({}, 1.0, 1.04, 0.0, R));   // no samples
}

TEST(GyroPrediction, ImuToCameraRotationConvention)
{
  // IMU z-up, camera y-down: p_imu = R_b_c p_cam with camera y = -IMU z, camera x = IMU x, camera z = IMU y
  Eigen::Matrix3d R_b_c;
  R_b_c << 1, 0, 0,
           0, 0, 1,
           0, -1, 0;
  Eigen::Vector3d w_cam(0, 0.3, 0);          // rotation about camera y
  Eigen::Vector3d w_imu = R_b_c * w_cam;     // = about IMU -z
  Eigen::Matrix3d R_c = imuToCameraRotation(so3Exp(w_imu), R_b_c);
  EXPECT_LT(angleBetween(R_c, so3Exp(w_cam)), 1e-12);
}
