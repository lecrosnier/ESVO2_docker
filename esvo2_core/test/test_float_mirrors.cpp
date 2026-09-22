#include <gtest/gtest.h>
#include <esvo2_core/container/TimeSurfaceObservation.h>

using esvo2_core::container::TimeSurfaceObservation;

static cv_bridge::CvImagePtr randomMono8(int rows, int cols, int seed)
{
  cv_bridge::CvImagePtr p(new cv_bridge::CvImage);
  p->encoding = "mono8";
  p->image = cv::Mat(rows, cols, CV_8UC1);
  cv::RNG rng(seed);
  rng.fill(p->image, cv::RNG::UNIFORM, 0, 256);
  return p;
}

TEST(FloatMirrors, EqualTheDoubleSurfaces)
{
  cv_bridge::CvImagePtr l = randomMono8(72, 128, 1), r = randomMono8(72, 128, 2);
  TimeSurfaceObservation obs(l, r, 0, false);
  obs.refreshFloatMirrors();
  ASSERT_EQ(obs.TS_left_f_.rows(), 72);
  ASSERT_EQ(obs.TS_left_f_.cols(), 128);
  EXPECT_TRUE((obs.TS_left_f_.cast<double>().array() == obs.TS_left_.array()).all());
  EXPECT_TRUE((obs.TS_right_f_.cast<double>().array() == obs.TS_right_.array()).all());
}

TEST(FloatMirrors, FollowTheBlur)
{
  cv_bridge::CvImagePtr l = randomMono8(72, 128, 3), r = randomMono8(72, 128, 4);
  TimeSurfaceObservation obs(l, r, 0, false);
  obs.GaussianBlurTS(5);
  obs.refreshFloatMirrors();
  EXPECT_TRUE((obs.TS_left_f_.cast<double>().array() == obs.TS_left_.array()).all());
  EXPECT_TRUE((obs.TS_right_f_.cast<double>().array() == obs.TS_right_.array()).all());
}
