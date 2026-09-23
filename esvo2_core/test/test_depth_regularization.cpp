#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <random>
#include <esvo2_core/core/DepthRegularization.h>

using namespace esvo2_core;
using namespace esvo2_core::core;

namespace
{
// The regularizer as it was before it was parallelised (upstream ESVO2),
// kept verbatim as the reference the optimised version must reproduce.
void referenceApply(DepthMap &dm, size_t radius, size_t minNeighbours, size_t minClose, const std::string &norm)
{
  DepthMap dmTmp(dm.rows(), dm.cols());
  DepthMap::iterator it = dm.begin();
  while (it != dm.end())
  {
    dmTmp.set(it->row(), it->col(), *it);
    DepthPoint &newDp = dmTmp.get(it->row(), it->col());
    if (it->valid())
    {
      std::vector<DepthPoint *> neighbours;
      dm.getNeighbourhood(it->row(), it->col(), radius, neighbours);
      bool isSet = false;
      if (neighbours.size() > minNeighbours)
      {
        std::vector<DepthPoint *> closeNeighbours;
        for (size_t i = 0; i < neighbours.size(); i++)
        {
          if (neighbours[i]->valid())
          {
            double diff = fabs(it->invDepth() - neighbours[i]->invDepth());
            if (diff < 2.0 * sqrt(it->variance()) ||
                diff < 2.0 * sqrt(neighbours[i]->variance()))
              closeNeighbours.push_back(neighbours[i]);
          }
        }
        if (closeNeighbours.size() > minClose)
        {
          double statisticalMean = 0.0;
          if (norm == "l2")
          {
            double totalInvVariances = 0.0;
            for (size_t i = 0; i < closeNeighbours.size(); i++)
              totalInvVariances += 1.0 / closeNeighbours[i]->variance();
            for (size_t i = 0; i < closeNeighbours.size(); i++)
              statisticalMean += closeNeighbours[i]->invDepth() *
                                 (1.0 / closeNeighbours[i]->variance()) / totalInvVariances;
          }
          else
          {
            double nu_post = closeNeighbours[0]->nu();
            double invDepth_post = closeNeighbours[0]->invDepth();
            double scale2_post = closeNeighbours[0]->scaleSquared();
            for (size_t i = 1; i < closeNeighbours.size(); i++)
            {
              double nu_prior = nu_post;
              double invDepth_prior = invDepth_post;
              double scale2_prior = scale2_post;
              double nu_obs = closeNeighbours[i]->nu();
              double invDepth_obs = closeNeighbours[i]->invDepth();
              double scale2_obs = closeNeighbours[i]->scaleSquared();
              nu_post = std::min(nu_prior, nu_obs);
              invDepth_post = (scale2_obs * invDepth_prior + scale2_prior * invDepth_obs) / (scale2_obs + scale2_prior);
              scale2_post = (nu_post + pow(invDepth_prior - invDepth_obs, 2) / (scale2_prior + scale2_obs)) /
                            (nu_post + 1) * (scale2_prior * scale2_obs) / (scale2_prior + scale2_obs);
            }
            statisticalMean = invDepth_post;
          }
          newDp.invDepth() = statisticalMean;
          isSet = true;
        }
      }
      if (!isSet)
        newDp.invDepth() = -1.0;
    }
    it++;
  }
  dm = dmTmp;
}

// A VECtor-sized map: 480x640, a smooth inverse-depth field with noise, so
// windows hold both close and distant neighbours, plus some invalid points.
DepthMap::Ptr makeMap(size_t numPoints, unsigned seed)
{
  DepthMap::Ptr dm = std::make_shared<DepthMap>(480, 640);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> row(0, 479), col(0, 639);
  std::normal_distribution<double> noise(0.0, 0.02);
  std::uniform_real_distribution<double> var(1e-5, 4e-3), nu(2.0, 10.0), unit(0.0, 1.0);
  for (size_t i = 0; i < numPoints; i++)
  {
    const int r = row(rng), c = col(rng);
    DepthPoint dp(r, c);
    dp.invDepth() = unit(rng) < 0.05 ? -1.0 : 0.3 + 0.4 * sin(r / 60.0) * cos(c / 80.0) + noise(rng);
    dp.variance() = var(rng);
    dp.scaleSquared() = var(rng);
    dp.nu() = nu(rng);
    dm->set(r, c, dp);
  }
  return dm;
}

std::shared_ptr<DepthProblemConfig> makeConfig(const std::string &norm, int radius, int minNb, int minClose)
{
  return std::make_shared<DepthProblemConfig>(15, 7, norm, 2.182, 17.277, 1, radius, minNb, minClose);
}

// Counts points whose regularized inverse depth differs from the reference.
size_t countMismatches(DepthMap &a, DepthMap &b, double &maxAbsDiff)
{
  size_t n = 0;
  maxAbsDiff = 0.0;
  for (DepthMap::iterator it = a.begin(); it != a.end(); it++)
  {
    const double other = b.get(it->row(), it->col()).invDepth();
    if (it->invDepth() != other)
    {
      n++;
      maxAbsDiff = std::max(maxAbsDiff, std::fabs(it->invDepth() - other));
    }
  }
  return n;
}

void checkAgainstReference(const std::string &norm, int radius, int minNb, int minClose, size_t numThread)
{
  for (unsigned seed = 1; seed <= 3; seed++)
  {
    DepthMap::Ptr ref = makeMap(20000, seed);
    DepthMap::Ptr opt = makeMap(20000, seed);
    ASSERT_EQ(ref->size(), opt->size());

    auto t0 = std::chrono::steady_clock::now();
    referenceApply(*ref, radius, minNb, minClose, norm);
    auto t1 = std::chrono::steady_clock::now();
    auto cfg = makeConfig(norm, radius, minNb, minClose);
    DepthRegularization reg(cfg, numThread);
    reg.apply(opt);
    auto t2 = std::chrono::steady_clock::now();

    double maxAbsDiff = 0.0;
    const size_t mismatches = countMismatches(*ref, *opt, maxAbsDiff);
    std::cout << norm << " r=" << radius << " threads=" << numThread << " seed=" << seed
              << ": " << opt->size() << " points, mismatches " << mismatches << " (max |diff| " << maxAbsDiff
              << "), reference " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms, optimised "
              << std::chrono::duration<double, std::milli>(t2 - t1).count() << " ms" << std::endl;
    EXPECT_EQ(mismatches, 0u);
    EXPECT_EQ(ref->size(), opt->size());
  }
}
} // namespace

// VECtor's configuration: Tdist, radius 20, 32/32 neighbours.
TEST(DepthRegularization, MatchesReferenceTdistRadius20SingleThread) { checkAgainstReference("Tdist", 20, 32, 32, 1); }
TEST(DepthRegularization, MatchesReferenceTdistRadius20FourThreads) { checkAgainstReference("Tdist", 20, 32, 32, 4); }
// Upstream's small-scale default: l2, radius 5, 8/8 neighbours.
TEST(DepthRegularization, MatchesReferenceL2Radius5FourThreads) { checkAgainstReference("l2", 5, 8, 8, 4); }
