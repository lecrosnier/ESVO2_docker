#ifndef ESVO2_CORE_TEST_GOLDEN_HARNESS_H
#define ESVO2_CORE_TEST_GOLDEN_HARNESS_H

// Offline replay of golden captures (tools/golden_capture.h) through EventBM
// and DepthProblemSolver, configured as esvo2_Mapping configures them.

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <esvo2_core/tools/golden_capture.h>
#include <esvo2_core/core/EventBM.h>
#include <esvo2_core/core/DepthProblemSolver.h>

namespace golden
{
using namespace esvo2_core;

inline std::string goldenDir()
{
  const char *e = std::getenv("ESVO2_GOLDEN_DIR");
  return e ? e : "/root/datasets/evk4/golden/slide4_bias";
}

// The config the code under test runs with: the committed EVK4 mapping YAML
// and the calibration folder the EVK4 launch file passes.
inline tools::GoldenConfig repoConfig()
{
  const std::string src = ESVO2_CORE_SOURCE_DIR;
  const YAML::Node y = YAML::LoadFile(src + "/cfg/mapping/mapping_evk4_AA_mapping.yaml");
  tools::GoldenConfig c;
  c.patch_size_X = y["patch_size_X"].as<int>();
  c.patch_size_Y = y["patch_size_Y"].as<int>();
  c.BM_min_disparity = y["BM_min_disparity"].as<int>();
  c.BM_max_disparity = y["BM_max_disparity"].as<int>();
  c.invDepth_min_range = y["invDepth_min_range"].as<double>();
  c.invDepth_max_range = y["invDepth_max_range"].as<double>();
  c.BM_step = y["BM_step"].as<int>();
  c.BM_ZNCC_Threshold = y["BM_ZNCC_Threshold"].as<double>();
  c.PROCESS_EVENT_NUM = y["PROCESS_EVENT_NUM"].as<int>();
  c.num_threads = NUM_THREAD_MAPPING;
  // esvo2_Mapping builds the static solver's config from LSnorm_ln, Tdist_nu, Tdist_scale.
  c.LSnorm = y["LSnorm_ln"].as<std::string>();
  c.Tdist_nu = y["Tdist_nu"].as<double>();
  c.Tdist_scale = y["Tdist_scale"].as<double>();
  c.calibInfoDir = src + "/calib/evk4_stereo";
  return c;
}

struct Offline
{
  tools::GoldenConfig cfg;
  CameraSystem::Ptr cam;              // DepthProblemSolver keeps a reference to this
  std::pair<size_t, size_t> disp;     // static BM disparity range, as the node computes it
  std::vector<std::shared_ptr<core::DepthProblemConfig>> dpConfigs;

  explicit Offline(const tools::GoldenConfig &g) : cfg(g)
  {
    cam = std::make_shared<CameraSystem>(g.calibInfoDir, false);
    disp = core::EventBM::disparityRange(*cam, g.invDepth_min_range, g.invDepth_max_range,
                                         g.BM_min_disparity, g.BM_max_disparity);
  }

  // Surfaces are captured after the node's blur, so no smoothing here.
  std::unique_ptr<core::EventBM> makeBM()
  {
    std::unique_ptr<core::EventBM> bm(new core::EventBM(cam, cfg.num_threads, false));
    bm->resetParameters(cfg.patch_size_X, cfg.patch_size_Y, disp.first, disp.second,
                        cfg.BM_step, cfg.BM_ZNCC_Threshold, false);
    return bm;
  }

  // Each solver gets its own config: DepthProblemSolver halves the patch size in place.
  std::unique_ptr<core::DepthProblemSolver> makeSolver()
  {
    dpConfigs.push_back(std::make_shared<core::DepthProblemConfig>(
      cfg.patch_size_X, cfg.patch_size_Y, cfg.LSnorm, cfg.Tdist_nu, cfg.Tdist_scale, 1, 5, 8, 8));
    return std::unique_ptr<core::DepthProblemSolver>(new core::DepthProblemSolver(
      cam, dpConfigs.back(), core::NUMERICAL, cfg.num_threads, true));
  }
};

// Heap-allocated because EventBM and DepthProblemSolver keep pointers to it.
inline std::unique_ptr<constStampedTimeSurfaceObs> makeObservation(const tools::GoldenCycle &c)
{
  std::unique_ptr<constStampedTimeSurfaceObs> obs(
    new constStampedTimeSurfaceObs(ros::Time(0), container::TimeSurfaceObservation()));
  obs->second.TS_left_ = c.TS_left;
  obs->second.TS_right_ = c.TS_right;
  obs->second.tr_ = Transformation(
    Eigen::Vector3d(c.position[0], c.position[1], c.position[2]),
    Eigen::Quaterniond(c.q_wxyz[0], c.q_wxyz[1], c.q_wxyz[2], c.q_wxyz[3]));
  return obs;
}

inline std::vector<dvs_msgs::Event> makeEvents(const tools::GoldenCycle &c)
{
  std::vector<dvs_msgs::Event> evs(c.events.size());
  for (size_t i = 0; i < evs.size(); i++)
  {
    evs[i].x = c.events[i].x;
    evs[i].y = c.events[i].y;
    evs[i].ts = ros::Time(c.events[i].sec, c.events[i].nsec);
    evs[i].polarity = true; // not read by BM
  }
  return evs;
}

inline std::vector<dvs_msgs::Event *> pointersTo(std::vector<dvs_msgs::Event> &evs)
{
  std::vector<dvs_msgs::Event *> p;
  for (dvs_msgs::Event &e : evs)
    p.push_back(&e);
  return p;
}

// The captured (double) matches, as input to the depth solve.
inline std::vector<core::EventMatchPair> makeMatches(const tools::GoldenCycle &c,
                                                     const constStampedTimeSurfaceObs &obs)
{
  std::vector<core::EventMatchPair> v(c.matches.size());
  for (size_t i = 0; i < v.size(); i++)
  {
    const tools::GoldenMatch &m = c.matches[i];
    v[i].x_left_raw_ = Eigen::Vector2d(m.x_left_raw[0], m.x_left_raw[1]);
    v[i].x_left_ = Eigen::Vector2d(m.x_left[0], m.x_left[1]);
    v[i].x_right_ = Eigen::Vector2d(m.x_right[0], m.x_right[1]);
    v[i].invDepth_ = m.invDepth;
    v[i].cost_ = m.cost;
    v[i].disp_ = m.disp;
    v[i].trans_ = obs.second.tr_;
  }
  return v;
}

inline double percentile(std::vector<double> v, double q)
{
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, (size_t)(q * (v.size() - 1) + 0.5))];
}
} // namespace golden

#endif // ESVO2_CORE_TEST_GOLDEN_HARNESS_H
