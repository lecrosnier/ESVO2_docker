#include <gtest/gtest.h>
#include <cstdlib>
#include <esvo2_core/tools/golden_capture.h>

using namespace esvo2_core::tools;

static GoldenConfig sampleConfig()
{
  GoldenConfig c;
  c.patch_size_X = 15; c.patch_size_Y = 7;
  c.BM_min_disparity = 0; c.BM_max_disparity = 320;
  c.invDepth_min_range = 0.1; c.invDepth_max_range = 1.0;
  c.BM_step = 3; c.BM_ZNCC_Threshold = 0.2;
  c.PROCESS_EVENT_NUM = 4000; c.num_threads = 4;
  c.LSnorm = "Tdist"; c.Tdist_nu = 2.182; c.Tdist_scale = 17.277;
  c.calibInfoDir = "/some/calib/evk4_stereo";
  return c;
}

static GoldenCycle sampleCycle()
{
  GoldenCycle c;
  c.cycle = 40;
  c.TS_left = Eigen::MatrixXd(6, 8);
  for (int r = 0; r < 6; r++)
    for (int k = 0; k < 8; k++)
      c.TS_left(r, k) = (r * 8 + k) * 5 % 256;
  c.TS_right = Eigen::MatrixXd::Constant(6, 8, 255.0);
  c.q_wxyz = {0.9238795325112867, 0.0, 0.3826834323650898, 0.0};
  c.position = {0.5, -0.25, 1.0 / 3.0};
  c.events = {{10, 20, 1600000000u, 123456789u}, {11, 21, 1600000001u, 5u}};
  GoldenMatch m;
  m.x_left_raw = {10, 20}; m.x_left = {10.123456789012345, 20.5}; m.x_right = {3, 20};
  m.invDepth = 0.4567; m.cost = 0.1234; m.disp = 7;
  c.matches = {m};
  GoldenDepth d;
  d.x = {10.123456789012345, 20.5}; d.invDepth = 0.4567; d.variance = 1e-3 / 3; d.residual = 123.456;
  c.depths = {d};
  return c;
}

static std::string tempDir()
{
  char tmpl[] = "/tmp/golden_test_XXXXXX";
  return std::string(mkdtemp(tmpl));
}

TEST(GoldenCapture, RoundTripsEveryField)
{
  const std::string dir = tempDir();
  const GoldenConfig cfg = sampleConfig();
  const GoldenCycle c = sampleCycle();
  std::string err;
  ASSERT_TRUE(writeGoldenCycle(dir, cfg, c, &err)) << err;

  const std::vector<std::string> files = listGoldenCycles(dir);
  ASSERT_EQ(files.size(), 1u);
  GoldenConfig cfg2;
  GoldenCycle c2;
  ASSERT_TRUE(readGoldenCycle(files[0], cfg2, c2, &err)) << err;

  EXPECT_TRUE(configDifferences(cfg, cfg2).empty());
  EXPECT_EQ(c2.cycle, 40);
  EXPECT_TRUE(c2.TS_left == c.TS_left);
  EXPECT_TRUE(c2.TS_right == c.TS_right);
  EXPECT_EQ(c2.q_wxyz, c.q_wxyz);
  EXPECT_EQ(c2.position, c.position);
  ASSERT_EQ(c2.events.size(), 2u);
  EXPECT_EQ(c2.events[0].x, 10);
  EXPECT_EQ(c2.events[0].nsec, 123456789u);
  EXPECT_EQ(c2.events[1].sec, 1600000001u);
  ASSERT_EQ(c2.matches.size(), 1u);
  EXPECT_EQ(c2.matches[0].x_left, c.matches[0].x_left);
  EXPECT_EQ(c2.matches[0].x_right, c.matches[0].x_right);
  EXPECT_EQ(c2.matches[0].invDepth, c.matches[0].invDepth);
  EXPECT_EQ(c2.matches[0].disp, c.matches[0].disp);
  ASSERT_EQ(c2.depths.size(), 1u);
  EXPECT_EQ(c2.depths[0].variance, c.depths[0].variance);
  EXPECT_EQ(c2.depths[0].residual, c.depths[0].residual);
  std::system(("rm -rf " + dir).c_str());
}

TEST(GoldenCapture, RefusesANonIntegerSurface)
{
  const std::string dir = tempDir();
  GoldenCycle c = sampleCycle();
  c.TS_left(0, 0) = 1.5;
  std::string err;
  EXPECT_FALSE(writeGoldenCycle(dir, sampleConfig(), c, &err));
  EXPECT_FALSE(err.empty());
  std::system(("rm -rf " + dir).c_str());
}

TEST(GoldenCapture, NamesTheConfigDifferences)
{
  GoldenConfig a = sampleConfig(), b = sampleConfig();
  b.BM_step = 2;
  b.LSnorm = "l2";
  const std::vector<std::string> d = configDifferences(a, b);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0], "BM_step");
  EXPECT_EQ(d[1], "LSnorm");
}

TEST(GoldenCapture, ListsNothingForAMissingDirectory)
{
  EXPECT_TRUE(listGoldenCycles("/nonexistent/golden").empty());
}
