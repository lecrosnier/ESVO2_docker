#include <gtest/gtest.h>
#include <esvo2_core/core/EventBM.h>
#include "synthetic_scene.h"

using namespace esvo2_core;

static std::unique_ptr<core::EventBM> makeBM(CameraSystem::Ptr cam, bool use_float)
{
  std::unique_ptr<core::EventBM> bm(new core::EventBM(cam, 4, false));
  bm->resetParameters(15, 7, 0, 320, 3, 0.2, false);
  bm->setUseFloat(use_float);
  return bm;
}

TEST(BmFloat, FloatSumsAreExactUpTo258Pixels)
{
  EXPECT_TRUE(core::EventBM::floatSumsAreExact(15, 7));
  EXPECT_TRUE(core::EventBM::floatSumsAreExact(258, 1));
  EXPECT_FALSE(core::EventBM::floatSumsAreExact(259, 1));
  EXPECT_FALSE(core::EventBM::floatSumsAreExact(25, 25));
}

// Per event: the float path must reach the same decision and the same match.
// Sums of 8-bit values are exact in float here, so this is expected to be bit-identical.
TEST(BmFloat, SameDecisionAndMatchPerEvent)
{
  SyntheticScene sc;
  std::unique_ptr<core::EventBM> bd = makeBM(sc.cam, false), bf = makeBM(sc.cam, true);
  bd->createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  bf->createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  core::EventBM::BmScratch s;
  bf->prepareScratch(s);
  int accepted = 0, near_truth = 0;
  for (size_t i = 0; i < sc.ptrs.size(); i++)
  {
    std::pair<size_t, size_t> bound(0, 320);
    core::EventMatchPair md, mf;
    const bool ad = bd->match_an_event2(sc.ptrs[i], bound, md);
    const bool af = bf->match_an_event2_f(sc.ptrs[i], bound, mf, s);
    ASSERT_EQ(ad, af) << "event " << i;
    if (!ad)
      continue;
    accepted++;
    near_truth += std::abs(md.disp_ - SyntheticScene::kDisparity) <= 1;
    EXPECT_EQ(md.disp_, mf.disp_) << "event " << i;
    EXPECT_EQ(md.cost_, mf.cost_) << "event " << i;
    EXPECT_EQ(md.invDepth_, mf.invDepth_) << "event " << i;
    EXPECT_EQ(md.x_right_, mf.x_right_) << "event " << i;
  }
  // The scene must actually exercise matching, and the double path must find the truth.
  EXPECT_GT(accepted, 100);
  EXPECT_GT(near_truth, accepted * 9 / 10);
}

TEST(BmFloat, SameOutputThroughTheThreadedEntryPoint)
{
  SyntheticScene sc;
  std::unique_ptr<core::EventBM> bd = makeBM(sc.cam, false), bf = makeBM(sc.cam, true);
  std::vector<core::EventMatchPair> vd, vf;
  bd->createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  bd->match_all_HyperThread(vd);
  bf->createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  bf->match_all_HyperThread(vf);
  ASSERT_EQ(vd.size(), vf.size());
  for (size_t i = 0; i < vd.size(); i++)
  {
    EXPECT_EQ(vd[i].x_left_raw_, vf[i].x_left_raw_) << i;
    EXPECT_EQ(vd[i].disp_, vf[i].disp_) << i;
    EXPECT_EQ(vd[i].cost_, vf[i].cost_) << i;
  }
}

TEST(BmFloat, CreateMatchProblemRefreshesTheMirrors)
{
  SyntheticScene sc;
  std::unique_ptr<core::EventBM> bf = makeBM(sc.cam, true);
  bf->createMatchProblem(sc.obs.get(), nullptr, &sc.ptrs);
  ASSERT_EQ(sc.obs->second.TS_left_f_.rows(), sc.obs->second.TS_left_.rows());
  EXPECT_TRUE((sc.obs->second.TS_right_f_.cast<double>().array() == sc.obs->second.TS_right_.array()).all());
}
