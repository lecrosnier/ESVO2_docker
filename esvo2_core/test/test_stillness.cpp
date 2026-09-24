#include <gtest/gtest.h>
#include <cmath>
#include <esvo2_core/tools/stillness.h>

using esvo2_core::tools::ImuSample;
using esvo2_core::tools::Motion;
using esvo2_core::tools::StillnessDetector;

static const Eigen::Vector3d kBias(0.0034, -0.0042, -0.0039);
static const Eigen::Vector3d kGravity(0.1, 9.80, 0.2);

// Alternating +/-amp: std exactly amp, mean 0 over pairs.
static double wobble(double t, double amp)
{
  return (std::llround(t / 0.005) % 2 == 0) ? amp : -amp;
}

// Feeds 200 Hz samples on [t0, t1].
template <typename Fn>
static void feed(StillnessDetector &d, double t0, double t1, Fn sample)
{
  for (double t = t0; t <= t1 + 1e-12; t += 0.005)
    d.add(sample(t));
}

static ImuSample still(double t)
{
  const double n = wobble(t, 0.0005);
  return {t, kBias + Eigen::Vector3d(n, -n, n), kGravity + Eigen::Vector3d(4 * n, 4 * n, -4 * n)};
}

TEST(Stillness, StillRigIsStill)
{
  StillnessDetector d;
  feed(d, 0.0, 1.0, still);
  EXPECT_EQ(d.classify(1.0), Motion::STILL);
  EXPECT_NEAR(d.gyroStd(), 0.0005, 1e-6);
  EXPECT_NEAR(d.accStd(), 0.002, 1e-6);
}

TEST(Stillness, RollingCartIsMovingThroughTheAccelerometer)
{
  // Gyro as quiet as at rest, but the wheels shake the accelerometer.
  StillnessDetector d;
  feed(d, 0.0, 1.0, [](double t) {
    ImuSample s = still(t);
    s.a += Eigen::Vector3d(0.0, 0.0, wobble(t, 0.1));
    return s;
  });
  EXPECT_EQ(d.classify(1.0), Motion::MOVING);
  EXPECT_LT(d.gyroStd(), 0.0035);
}

TEST(Stillness, SteadySlowTurnIsMovingThroughTheGyroMean)
{
  StillnessDetector d;
  feed(d, 0.0, 1.0, [](double t) {
    ImuSample s = still(t);
    s.w.z() += 0.05;
    return s;
  });
  EXPECT_EQ(d.classify(1.0), Motion::MOVING);
}

TEST(Stillness, ShakingGyroIsMoving)
{
  StillnessDetector d;
  feed(d, 0.0, 1.0, [](double t) {
    ImuSample s = still(t);
    s.w.x() += wobble(t, 0.01);
    return s;
  });
  EXPECT_EQ(d.classify(1.0), Motion::MOVING);
}

TEST(Stillness, StaleImuIsUnknownNotStill)
{
  // The IMU dies at t = 1 s; a still rig must not stay "still" on old data.
  StillnessDetector d;
  feed(d, 0.0, 1.0, still);
  EXPECT_EQ(d.classify(1.05), Motion::STILL);
  EXPECT_EQ(d.classify(1.2), Motion::UNKNOWN);
  EXPECT_GT(d.age(1.2), 0.1);
}

TEST(Stillness, NoDataIsUnknown)
{
  StillnessDetector d;
  EXPECT_EQ(d.classify(0.0), Motion::UNKNOWN);
}

TEST(Stillness, PartialWindowIsUnknown)
{
  StillnessDetector d;
  feed(d, 0.0, 0.3, still);
  EXPECT_EQ(d.classify(0.3), Motion::UNKNOWN);
}

TEST(Stillness, GapInsideWindowIsUnknown)
{
  StillnessDetector d;
  feed(d, 0.0, 0.7, still);
  feed(d, 0.75, 1.0, still); // 50 ms hole mid-window
  EXPECT_EQ(d.classify(1.0), Motion::UNKNOWN);
  feed(d, 1.005, 1.6, still); // hole leaves the window
  EXPECT_EQ(d.classify(1.6), Motion::STILL);
}

TEST(Stillness, BackwardClockJumpRestarts)
{
  StillnessDetector d;
  feed(d, 10.0, 11.0, still);
  feed(d, 0.0, 1.0, still);
  EXPECT_EQ(d.classify(1.0), Motion::STILL);
}

TEST(Stillness, RecoversAfterMotionStops)
{
  StillnessDetector d;
  feed(d, 0.0, 1.0, [](double t) {
    ImuSample s = still(t);
    s.a.x() += wobble(t, 0.2);
    return s;
  });
  EXPECT_EQ(d.classify(1.0), Motion::MOVING);
  feed(d, 1.005, 1.6, still);
  EXPECT_EQ(d.classify(1.6), Motion::STILL);
}

TEST(Stillness, FrozenStreamIsUnknownNotStill)
{
  // A dead driver republishing its last message: fresh stamps, identical values.
  StillnessDetector d;
  feed(d, 0.0, 1.0, [](double t) { return ImuSample{t, kBias, kGravity}; });
  EXPECT_EQ(d.classify(1.0), Motion::UNKNOWN);
}
