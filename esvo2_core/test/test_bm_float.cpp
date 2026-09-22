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

static cv_bridge::CvImagePtr randomMono8(int rows, int cols, int seed)
{
  cv_bridge::CvImagePtr p(new cv_bridge::CvImage);
  p->encoding = "mono8";
  p->image = cv::Mat(rows, cols, CV_8UC1);
  cv::RNG rng(seed);
  rng.fill(p->image, cv::RNG::UNIFORM, 0, 256);
  return p;
}

// Regression test (A1 fix round 2, finding 2): createMatchProblem must blur
// the observation's cvImagePtr_left_/right_ (when bSmoothTS_) BEFORE it
// refreshes the float mirrors, so the float path never reads stale
// (pre-blur) TS_left_f_/TS_right_f_. Uses the (left, right, id,
// bCalcTsGradient) constructor, which sets cvImagePtr_left_/right_, on
// random 8-bit texture so the blur visibly changes the surface.
TEST(BmFloat, CreateMatchProblemRefreshesTheMirrorsAfterTheBlur)
{
  using namespace esvo2_core::container;
  cv_bridge::CvImagePtr l = randomMono8(72, 128, 31), r = randomMono8(72, 128, 32);
  Eigen::MatrixXd unblurred_left;
  cv::cv2eigen(l->image, unblurred_left);

  std::unique_ptr<constStampedTimeSurfaceObs> obs(
    new constStampedTimeSurfaceObs(ros::Time(0), TimeSurfaceObservation(l, r, 0, false)));
  obs->second.tr_.setIdentity();

  CameraSystem::Ptr cam = std::make_shared<CameraSystem>(
    std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
  std::unique_ptr<core::EventBM> bf(new core::EventBM(cam, 4, /*bSmoothTS=*/true));
  bf->resetParameters(15, 7, 0, 320, 3, 0.2, false);
  bf->setUseFloat(true);

  std::vector<dvs_msgs::Event *> ptrs; // empty: only the mirror refresh is under test
  bf->createMatchProblem(obs.get(), nullptr, &ptrs);

  ASSERT_EQ(obs->second.TS_left_f_.rows(), obs->second.TS_left_.rows());
  ASSERT_EQ(obs->second.TS_left_f_.cols(), obs->second.TS_left_.cols());
  // The mirror must equal the (post-blur) TS_left_ ...
  EXPECT_TRUE((obs->second.TS_left_f_.cast<double>().array() == obs->second.TS_left_.array()).all());
  // ... and that TS_left_ must actually be the blurred surface, not the raw
  // image, otherwise the equality above would be true trivially.
  EXPECT_FALSE((obs->second.TS_left_.array() == unblurred_left.array()).all());
}

// Regression test (A1 fix round 1): epipolarSearchingCoarse (double) skips
// marking neighbours for fine search whenever the very first coarse
// candidate (disparity == lowDisparity == 1) already passes the preliminary
// ZNCC threshold -- its window-bound loop mixes int and size_t, and the
// negative lower bound that case needs makes the loop condition compare a
// huge unsigned value against the (unsigned) upper bound, so the loop body
// never runs. The float path must reproduce this exactly, not fix it: A1 is
// same-algorithm, bit-identical static BM, not a behaviour change, and any
// fix to the quirk is a separate decision affecting both paths at once. A
// scene with a true disparity of 2 px makes disparity 1 (the first coarse
// candidate) a near-miss with a good enough ZNCC cost to hit this case.
TEST(BmFloat, FirstCoarseCandidateQuirkMatchesDoublePath)
{
  using namespace esvo2_core::container;
  CameraSystem::Ptr cam = std::make_shared<CameraSystem>(
    std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
  const int W = cam->cam_left_ptr_->width_, H = cam->cam_left_ptr_->height_;
  const int kDisparity = 2;
  cv::Mat noise(H, W + kDisparity, CV_8UC1), tex;
  cv::RNG rng(11);
  rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(noise, tex, cv::Size(5, 5), 0.0);
  const cv::Mat left = tex(cv::Rect(0, 0, W, H)).clone();
  const cv::Mat right = tex(cv::Rect(kDisparity, 0, W, H)).clone(); // right(y,x) = left(y,x+2)

  std::unique_ptr<constStampedTimeSurfaceObs> obs(
    new constStampedTimeSurfaceObs(ros::Time(0), TimeSurfaceObservation()));
  cv::cv2eigen(left, obs->second.TS_left_);
  cv::cv2eigen(right, obs->second.TS_right_);
  obs->second.tr_.setIdentity();

  std::vector<dvs_msgs::Event> events;
  for (int y = 60; y < H - 60; y += 23)
    for (int x = 100; x < W - 100; x += 29)
    {
      dvs_msgs::Event e;
      e.x = x;
      e.y = y;
      e.ts = ros::Time(1.0);
      e.polarity = true;
      events.push_back(e);
    }
  std::vector<dvs_msgs::Event *> ptrs;
  for (dvs_msgs::Event &e : events)
    ptrs.push_back(&e);

  std::unique_ptr<core::EventBM> bd = makeBM(cam, false), bf = makeBM(cam, true);
  bd->createMatchProblem(obs.get(), nullptr, &ptrs);
  bf->createMatchProblem(obs.get(), nullptr, &ptrs);
  core::EventBM::BmScratch s;
  bf->prepareScratch(s);

  int accepted = 0;
  for (size_t i = 0; i < ptrs.size(); i++)
  {
    std::pair<size_t, size_t> bound(0, 320);
    core::EventMatchPair md, mf;
    const bool ad = bd->match_an_event2(ptrs[i], bound, md);
    const bool af = bf->match_an_event2_f(ptrs[i], bound, mf, s);
    ASSERT_EQ(ad, af) << "event " << i;
    if (!ad)
      continue;
    accepted++;
    EXPECT_EQ(md.disp_, mf.disp_) << "event " << i;
    EXPECT_EQ(md.cost_, mf.cost_) << "event " << i;
    EXPECT_EQ(md.invDepth_, mf.invDepth_) << "event " << i;
    EXPECT_EQ(md.x_right_, mf.x_right_) << "event " << i;
  }
  // The scene must actually exercise matching for this to be a meaningful check.
  EXPECT_GT(accepted, 50);
}
