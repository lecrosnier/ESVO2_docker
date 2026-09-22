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

// Spec acceptance for static BM: >= 99% same decisions; every event matched by
// both within 1 px disparity and 0.1% inverse depth.
TEST(Golden, FloatBlockMatchingMeetsSpec)
{
  const std::vector<std::string> files = goldenFiles();
  if (files.empty())
    GTEST_SKIP() << "no golden capture in " << goldenDir() << " (see plan Task 3)";
  size_t events = 0, same = 0, both = 0, disp_ok = 0, inv_ok = 0;
  std::vector<double> inv_rel, disp_abs;
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
    std::unique_ptr<core::EventBM> bd = off.makeBM(false), bf = off.makeBM(true);
    bd->createMatchProblem(obs.get(), nullptr, &ptrs);
    bf->createMatchProblem(obs.get(), nullptr, &ptrs);
    core::EventBM::BmScratch s;
    bf->prepareScratch(s);
    for (dvs_msgs::Event *e : ptrs)
    {
      std::pair<size_t, size_t> bound = off.disp;
      core::EventMatchPair md, mf;
      const bool ad = bd->match_an_event2(e, bound, md);
      const bool af = bf->match_an_event2_f(e, bound, mf, s);
      events++;
      same += (ad == af);
      if (!(ad && af))
        continue;
      both++;
      const double dd = std::abs(md.disp_ - mf.disp_);
      const double dr = std::abs(md.invDepth_ - mf.invDepth_) / std::abs(md.invDepth_);
      disp_abs.push_back(dd);
      inv_rel.push_back(dr);
      disp_ok += dd <= 1.0;
      inv_ok += dr <= 1e-3;
    }
  }
  std::printf("[golden BM] %zu events, same decision %zu (%.4f%%), matched by both %zu\n",
              events, same, 100.0 * same / std::max<size_t>(events, 1), both);
  std::printf("[golden BM] |d disparity| px: p50 %.3g p99 %.3g max %.3g\n",
              percentile(disp_abs, 0.5), percentile(disp_abs, 0.99), percentile(disp_abs, 1.0));
  std::printf("[golden BM] rel d invDepth: p50 %.3g p99 %.3g max %.3g\n",
              percentile(inv_rel, 0.5), percentile(inv_rel, 0.99), percentile(inv_rel, 1.0));
  EXPECT_GE(same, 0.99 * events);
  EXPECT_EQ(disp_ok, both);
  EXPECT_EQ(inv_ok, both);
}

static double relDiff(double a, double b)
{
  const double scale = std::max(std::abs(a), std::abs(b));
  return scale == 0 ? 0 : std::abs(a - b) / scale;
}

// Spec acceptance for the static depth solve, on the same (captured) match
// pairs: variance and residual within 0.1% for >= 99.9% of points.
TEST(Golden, FloatDepthSolveMeetsSpec)
{
  const std::vector<std::string> files = goldenFiles();
  if (files.empty())
    GTEST_SKIP() << "no golden capture in " << goldenDir() << " (see plan Task 3)";
  size_t points = 0, ok = 0;
  std::vector<double> var_rel, res_rel;
  for (const std::string &file : files)
  {
    tools::GoldenConfig cfg;
    tools::GoldenCycle c;
    if (!loadCycle(file, cfg, c))
      continue;
    Offline off(cfg);
    std::unique_ptr<constStampedTimeSurfaceObs> obs = makeObservation(c);
    obs->second.refreshFloatMirrors();
    std::vector<core::EventMatchPair> vEMP = makeMatches(c, *obs);
    std::vector<DepthPoint> vd, vf;
    off.makeSolver(false)->solve(&vEMP, obs.get(), vd);
    off.makeSolver(true)->solve(&vEMP, obs.get(), vf);
    ASSERT_EQ(vd.size(), vf.size()) << file;
    for (size_t i = 0; i < vd.size(); i++)
    {
      const double rv = relDiff(vd[i].variance(), vf[i].variance());
      const double rr = relDiff(vd[i].residual(), vf[i].residual());
      var_rel.push_back(rv);
      res_rel.push_back(rr);
      points++;
      ok += (rv <= 1e-3 && rr <= 1e-3);
    }
  }
  std::printf("[golden depth] %zu points, within 0.1%%: %zu (%.4f%%)\n",
              points, ok, 100.0 * ok / std::max<size_t>(points, 1));
  std::printf("[golden depth] rel d variance: p50 %.3g p99 %.3g max %.3g\n",
              percentile(var_rel, 0.5), percentile(var_rel, 0.99), percentile(var_rel, 1.0));
  std::printf("[golden depth] rel d residual: p50 %.3g p99 %.3g max %.3g\n",
              percentile(res_rel, 0.5), percentile(res_rel, 0.99), percentile(res_rel, 1.0));
  EXPECT_GE(ok, 0.999 * points);
}
