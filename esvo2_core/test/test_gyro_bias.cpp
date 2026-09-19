#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Geometry>
#include <esvo2_core/tools/gyro_bias.h>

using esvo2_core::tools::GyroBiasEstimator;
using esvo2_core::tools::GyroSample;
using esvo2_core::tools::gyroDeltaRotation;
using esvo2_core::tools::so3Exp;
using esvo2_core::tools::subtractBias;

static const Eigen::Vector3d kBias(0.003425, -0.004210, -0.003853);

// Feeds samples every dt from t0 to t1 (inclusive); returns the time of the sample on
// which add() returned true, or -1 if it never did.
template <typename RateFn>
static double feed(GyroBiasEstimator &e, double t0, double t1, double dt, RateFn rate)
{
  double accepted_at = -1.0;
  for (double t = t0; t <= t1 + 1e-12; t += dt)
    if (e.add({t, rate(t)}) && accepted_at < 0)
      accepted_at = t;
  return accepted_at;
}

// Still: bias plus +/-0.0005 rad/s alternating noise (std 0.0005, mean exactly 0 over pairs).
static Eigen::Vector3d stillRate(double t)
{
  const double n = (static_cast<long>(std::llround(t / 0.005)) % 2 == 0) ? 0.0005 : -0.0005;
  return kBias + Eigen::Vector3d(n, -n, n);
}

TEST(GyroBias, StillWindowGivesItsMean)
{
  GyroBiasEstimator e(2.0, 0.0035);
  const double at = feed(e, 0.0, 2.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_NEAR(at, 2.0, 0.006);
  EXPECT_LT((e.bias() - kBias).norm(), 1e-5);
  EXPECT_NEAR(e.biasStd().x(), 0.0005, 1e-5);
}

TEST(GyroBias, MovingWindowRejectedThenStillAccepted)
{
  GyroBiasEstimator e(2.0, 0.0035);
  // hand-held motion: ~0.07 rad/s swings for the first 2 s
  feed(e, 0.0, 2.0, 0.005, [](double t) { return Eigen::Vector3d(kBias + 0.1 * std::sin(2 * M_PI * t) * Eigen::Vector3d::Ones()); });
  EXPECT_FALSE(e.hasBias());
  const double at = feed(e, 2.005, 4.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  // The motion feed ends at t≈1.99999 (float accumulation), so the mixed window closes on the
  // first still sample (2.005) and is rejected; the first still-only window then closes ~2 s later.
  EXPECT_GT(at, 4.0);
  EXPECT_LT(at, 4.03);
  EXPECT_LT((e.bias() - kBias).norm(), 2e-5);
}

TEST(GyroBias, WindowWithGapRejected)
{
  GyroBiasEstimator e(2.0, 0.0035);
  feed(e, 0.0, 1.0, 0.005, stillRate);
  feed(e, 1.03, 2.0, 0.005, stillRate);  // 30 ms gap inside the first window
  EXPECT_FALSE(e.hasBias());
  const double at = feed(e, 2.005, 4.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_GT(at, 4.0);
}

TEST(GyroBias, IgnoresSamplesAfterAcceptance)
{
  GyroBiasEstimator e(2.0, 0.0035);
  feed(e, 0.0, 2.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_FALSE(e.add({2.6, Eigen::Vector3d(1, 1, 1)}));
  EXPECT_LT((e.bias() - kBias).norm(), 1e-5);
}

TEST(GyroBias, SteadyRotationRejected)
{
  GyroBiasEstimator e(2.0, 0.0035);
  // Steady slow rotation: same small alternating noise as stillRate, but with a
  // non-zero mean on the x axis (0.05 rad/s) that variance alone would not catch.
  feed(e, 0.0, 2.0, 0.005, [](double t) -> Eigen::Vector3d { return stillRate(t) + Eigen::Vector3d(0.05, 0, 0); });
  EXPECT_FALSE(e.hasBias());
  const double at = feed(e, 2.005, 4.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_GT(at, 4.0);
  EXPECT_LT(at, 4.03);
  EXPECT_LT((e.bias() - kBias).norm(), 2e-5);
}

TEST(GyroBias, SubtractedBiasLeavesTrueRotation)
{
  const Eigen::Vector3d w(0.1, -0.4, 0.2);
  std::vector<GyroSample> s;
  for (double t = 0.0; t <= 1.0 + 1e-12; t += 0.005)
    s.push_back({t, w + kBias});
  subtractBias(s, kBias);
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.40, 0.44, 0.0, R));
  EXPECT_LT(Eigen::AngleAxisd(R.transpose() * so3Exp(w * 0.04)).angle(), 1e-9);
}
