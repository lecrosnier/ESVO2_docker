#ifndef ESVO2_CORE_TOOLS_GOLDEN_CAPTURE_H
#define ESVO2_CORE_TOOLS_GOLDEN_CAPTURE_H

// Golden capture: the exact inputs and double-precision outputs of one mapping
// cycle's static block matching and static depth solve, saved so an offline
// test can replay them through other implementations (the float CPU path, a
// later GPU path) with no ROS runtime and no replay timing noise. Written by
// esvo2_Mapping when its golden_capture_dir parameter is set.

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace esvo2_core
{
namespace tools
{
// Values that change the result of static BM or of the static depth solve. A
// capture is only comparable with code run under the same values.
struct GoldenConfig
{
  int patch_size_X = 0, patch_size_Y = 0;
  int BM_min_disparity = 0, BM_max_disparity = 0; // as configured (before the node narrows them)
  double invDepth_min_range = 0, invDepth_max_range = 0;
  int BM_step = 0;
  double BM_ZNCC_Threshold = 0;
  int PROCESS_EVENT_NUM = 0;
  int num_threads = 0;
  std::string LSnorm;
  double Tdist_nu = 0, Tdist_scale = 0;
  std::string calibInfoDir;
};

// Names of the fields that differ, in declaration order (empty when equal).
std::vector<std::string> configDifferences(const GoldenConfig &a, const GoldenConfig &b);

// The part of a dvs_msgs::Event that static BM reads.
struct GoldenEvent
{
  uint16_t x, y;
  uint32_t sec, nsec;
};

// One accepted EventMatchPair from static BM.
struct GoldenMatch
{
  std::array<double, 2> x_left_raw, x_left, x_right;
  double invDepth, cost, disp;
};

// One DepthPoint from DepthProblemSolver::solve, before culling.
struct GoldenDepth
{
  std::array<double, 2> x;
  double invDepth, variance, residual;
};

struct GoldenCycle
{
  int cycle = 0;
  Eigen::MatrixXd TS_left, TS_right;    // after the optional blur: what BM reads
  std::array<double, 4> q_wxyz{};       // tr_ rotation, stored exactly
  std::array<double, 3> position{};     // tr_ translation
  std::vector<GoldenEvent> events;      // BM candidates, in the order BM received them
  std::vector<GoldenMatch> matches;     // static BM output (vEMP), node order
  std::vector<GoldenDepth> depths;      // static depth solve output (vdp), node order
};

// Writes <dir>/cycle_NNNNN.yml and cycle_NNNNN_left.png / _right.png. The
// surfaces must be integer-valued in [0,255] (they come from mono8 images),
// which makes PNG lossless; returns false with a message in *err otherwise.
bool writeGoldenCycle(const std::string &dir, const GoldenConfig &cfg,
                      const GoldenCycle &c, std::string *err);
bool readGoldenCycle(const std::string &yml_path, GoldenConfig &cfg,
                     GoldenCycle &c, std::string *err);
// Sorted paths of every cycle_*.yml in dir (empty if dir is missing).
std::vector<std::string> listGoldenCycles(const std::string &dir);
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_GOLDEN_CAPTURE_H
