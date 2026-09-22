#include <gtest/gtest.h>
#include <esvo2_core/core/EventBM.h>
#include <esvo2_core/core/DepthProblemSolver.h>
#include "synthetic_scene.h"

using namespace esvo2_core;

TEST(DepthFloat, MatchesTheDoubleSolveOnTheSyntheticScene)
{
  SyntheticScene sc;
  core::EventBM bm(sc.cam, 4, false);
  bm.resetParameters(15, 7, 0, 320, 3, 0.2, false);
  bm.setUseFloat(true); // createMatchProblem then refreshes the mirrors the float solve reads
  bm.createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  std::vector<core::EventMatchPair> vEMP;
  bm.match_all_HyperThread(vEMP);
  ASSERT_GT(vEMP.size(), 100u);

  // One config per solver: DepthProblemSolver halves the patch size in place.
  std::shared_ptr<core::DepthProblemConfig> cfgD = std::make_shared<core::DepthProblemConfig>(
    15, 7, "Tdist", 2.182, 17.277, 1, 5, 8, 8);
  std::shared_ptr<core::DepthProblemConfig> cfgF = std::make_shared<core::DepthProblemConfig>(
    15, 7, "Tdist", 2.182, 17.277, 1, 5, 8, 8);
  core::DepthProblemSolver sd(sc.cam, cfgD, core::NUMERICAL, 4, true);
  core::DepthProblemSolver sf(sc.cam, cfgF, core::NUMERICAL, 4, true);
  sf.setUseFloat(true);
  std::vector<DepthPoint> vd, vf;
  sd.solve(&vEMP, sc.obs.get(), vd);
  sf.solve(&vEMP, sc.obs.get(), vf);

  ASSERT_EQ(vd.size(), vf.size());
  for (size_t i = 0; i < vd.size(); i++)
  {
    EXPECT_EQ(vd[i].x(), vf[i].x()) << i;
    EXPECT_EQ(vd[i].invDepth(), vf[i].invDepth()) << i;
    EXPECT_NEAR(vf[i].variance(), vd[i].variance(), 1e-6 * std::abs(vd[i].variance())) << i;
    EXPECT_NEAR(vf[i].residual(), vd[i].residual(), 1e-6 * std::abs(vd[i].residual()) + 1e-9) << i;
  }
}

TEST(DepthFloat, L2NormAlsoMatches)
{
  SyntheticScene sc;
  core::EventBM bm(sc.cam, 4, false);
  bm.resetParameters(15, 7, 0, 320, 3, 0.2, false);
  bm.setUseFloat(true);
  bm.createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  std::vector<core::EventMatchPair> vEMP;
  bm.match_all_HyperThread(vEMP);
  std::shared_ptr<core::DepthProblemConfig> cfgD = std::make_shared<core::DepthProblemConfig>(
    15, 7, "l2", 2.182, 17.277, 1, 5, 8, 8);
  std::shared_ptr<core::DepthProblemConfig> cfgF = std::make_shared<core::DepthProblemConfig>(
    15, 7, "l2", 2.182, 17.277, 1, 5, 8, 8);
  core::DepthProblemSolver sd(sc.cam, cfgD, core::NUMERICAL, 4, true);
  core::DepthProblemSolver sf(sc.cam, cfgF, core::NUMERICAL, 4, true);
  sf.setUseFloat(true);
  std::vector<DepthPoint> vd, vf;
  sd.solve(&vEMP, sc.obs.get(), vd);
  sf.solve(&vEMP, sc.obs.get(), vf);
  ASSERT_EQ(vd.size(), vf.size());
  for (size_t i = 0; i < vd.size(); i++)
    EXPECT_NEAR(vf[i].variance(), vd[i].variance(), 1e-6 * std::abs(vd[i].variance())) << i;
}
