#include <gtest/gtest.h>
#include <cstdio>
#include "golden_harness.h"

using namespace golden;

// Reads one cycle and checks it was captured under the committed config.
// Records a test failure and returns false otherwise.
static bool loadCycle(const std::string &file, tools::GoldenConfig &cfg, tools::GoldenCycle &c)
{
  std::string err;
  if (!tools::readGoldenCycle(file, cfg, c, &err))
  {
    ADD_FAILURE() << err;
    return false;
  }
  const std::vector<std::string> diff = tools::configDifferences(cfg, repoConfig());
  if (!diff.empty())
  {
    std::string keys;
    for (const std::string &k : diff)
      keys += " " + k;
    ADD_FAILURE() << file << " was captured under a different config (" << keys
                  << " ); re-capture (plan Task 3) instead of comparing.";
    return false;
  }
  return true;
}

static std::vector<std::string> goldenFiles()
{
  return tools::listGoldenCycles(goldenDir());
}

// The harness itself: the offline double path must reproduce what the node
// computed, or no comparison built on it means anything.
TEST(Golden, OfflineDoubleReproducesTheNode)
{
  const std::vector<std::string> files = goldenFiles();
  if (files.empty())
    GTEST_SKIP() << "no golden capture in " << goldenDir() << " (see plan Task 3)";
  size_t matches = 0, depths = 0;
  for (const std::string &file : files)
  {
    tools::GoldenConfig cfg;
    tools::GoldenCycle c;
    if (!loadCycle(file, cfg, c))
      continue;
    Offline off(cfg);
    std::unique_ptr<constStampedTimeSurfaceObs> obs = makeObservation(c);
    std::vector<dvs_msgs::Event> evs = makeEvents(c);
    std::vector<dvs_msgs::Event *> ptrs = pointersTo(evs);

    std::unique_ptr<core::EventBM> bm = off.makeBM();
    bm->createMatchProblem(obs.get(), nullptr, &ptrs);
    std::vector<core::EventMatchPair> vEMP;
    bm->match_all_HyperThread(vEMP);
    ASSERT_EQ(vEMP.size(), c.matches.size()) << file;
    for (size_t i = 0; i < vEMP.size(); i++)
    {
      EXPECT_EQ(vEMP[i].x_left_raw_(0), c.matches[i].x_left_raw[0]) << file << " match " << i;
      EXPECT_EQ(vEMP[i].x_left_raw_(1), c.matches[i].x_left_raw[1]) << file << " match " << i;
      EXPECT_EQ(vEMP[i].disp_, c.matches[i].disp) << file << " match " << i;
      EXPECT_EQ(vEMP[i].cost_, c.matches[i].cost) << file << " match " << i;
      EXPECT_EQ(vEMP[i].invDepth_, c.matches[i].invDepth) << file << " match " << i;
    }
    matches += vEMP.size();

    std::unique_ptr<core::DepthProblemSolver> solver = off.makeSolver();
    std::vector<DepthPoint> vdp;
    solver->solve(&vEMP, obs.get(), vdp);
    ASSERT_EQ(vdp.size(), c.depths.size()) << file;
    for (size_t i = 0; i < vdp.size(); i++)
    {
      EXPECT_EQ(vdp[i].invDepth(), c.depths[i].invDepth) << file << " point " << i;
      EXPECT_EQ(vdp[i].variance(), c.depths[i].variance) << file << " point " << i;
      EXPECT_EQ(vdp[i].residual(), c.depths[i].residual) << file << " point " << i;
    }
    depths += vdp.size();
  }
  std::printf("[golden] %zu cycles, %zu matches and %zu depth points reproduced\n",
              files.size(), matches, depths);
}
