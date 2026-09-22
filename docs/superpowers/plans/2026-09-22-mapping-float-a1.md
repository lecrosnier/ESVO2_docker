# Mapping in Float Without Per-Event Allocation (A1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make ESVO2's static block matching and static depth solve read float copies of the time surfaces and allocate nothing per event, behind a `MAPPING_FLOAT` switch, and prove it equivalent and faster with an offline golden-capture harness.

**Architecture:** The double path stays as the reference, and a parallel float path is selected at runtime (`EventBM::setUseFloat`, `DepthProblemSolver::setUseFloat`). The mapping node can dump the exact inputs and outputs of every Nth cycle (the "golden capture"). An offline gtest replays those dumps through both paths with no ROS runtime, and a benchmark times them. Three 1× replays confirm that tracking is unchanged.

**Tech Stack:** C++14, Eigen 3, OpenCV 4, glog, TBB, gtest (catkin_add_gtest), yaml-cpp, ROS Noetic (catkin_make), Python 3 (rosbag) for evaluation.

**Spec:** `docs/superpowers/specs/2026-09-22-mapping-float-a1-design.md`. Read it, including its "Planning notes" section, which overrides earlier sections where they differ.

## Global Constraints

- Work on branch `evk4-gyro-locked-tracking` in `/root/catkin_ws/src/ESVO2`, in place. The catkin workspace builds from this path, so do not create a worktree. Do not push.
- Build: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core`. Build and run one gtest: `catkin_make -j4 --pkg esvo2_core` once after any CMakeLists.txt change, then `cd /root/catkin_ws/build && make -j6 <name> && /root/catkin_ws/devel/lib/esvo2_core/<name>`. (`catkin_make --pkg esvo2_core run_tests_...` does not work: catkin_make takes the target for a package name and ignores it.)
- `MAPPING_FLOAT` defaults to `False` in the committed YAML throughout this plan.
- Float path scope: static block matching (`EventBM::match_an_event2`) and the static depth solve (`dpSolver_`, `slove_lr == true`) only. Temporal BM, the temporal depth solve (`dpSolver_ln_`), fusion, regularisation and tracking are not modified.
- Double-path code changes only by the behaviour-preserving extractions named in the tasks: `EventBM::disparityRange` (Task 4) and `DepthProblemSolver::appendDepthPoint` (Task 6). The golden sanity test (Task 4) must stay green after every later task.
- Tracking-side members of `TimeSurfaceObservation` and all tracking code are untouched. `NUM_THREAD_MAPPING` stays 4.
- Equivalence acceptance, verbatim from the spec: BM **≥ 99% of events make the same accept/reject decision**; events matched by both agree on disparity within **1 px** and inverse depth within **0.1% relative**. Depth solve: variance and residual agree within **0.1% relative** for **≥ 99.9% of points**.
- Tracking acceptance: three 1× replays of `slide4_bias.bag` with `MAPPING_FLOAT: True`. Every leg must lie within **63–76%** of the true 1 m.
- Golden data lives outside the repo, in `/root/datasets/evk4/golden/slide4_bias/`.
- Never ask anyone to move or touch the camera rig. All work is offline on recorded bags.
- Replays need a ROS master. Check with `rosnode list`, and if it fails start `roscore` in the background. Run one replay at a time, and nothing heavy during the benchmark. Do not wait on replays by counting process names with `ps | grep`, because zombie processes match. Wait on the PID or run in the foreground.
- End every commit message with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `esvo2_core/include/esvo2_core/container/TimeSurfaceObservation.h` | modify | adds `TS_left_f_`, `TS_right_f_`, `refreshFloatMirrors()` |
| `esvo2_core/include/esvo2_core/tools/golden_capture.h` | create | golden capture data types and read/write API (Eigen and std only) |
| `esvo2_core/src/tools/golden_capture.cpp` | create | PNG + cv::FileStorage implementation |
| `esvo2_core/include/esvo2_core/esvo2_Mapping.h`, `esvo2_core/src/esvo2_Mapping.cpp` | modify | capture hook, `MAPPING_FLOAT` wiring, uses `EventBM::disparityRange` |
| `esvo2_core/include/esvo2_core/core/EventBM.h`, `esvo2_core/src/core/EventBM.cpp` | modify | `disparityRange`, float BM path, per-thread scratch |
| `esvo2_core/include/esvo2_core/core/DepthProblem.h`, `esvo2_core/src/core/DepthProblem.cpp` | modify | `residualsFloat`, `patchInterpolationFloat` |
| `esvo2_core/include/esvo2_core/core/DepthProblemSolver.h`, `esvo2_core/src/core/DepthProblemSolver.cpp` | modify | `setUseFloat`, `init_single_point_f`, `appendDepthPoint` |
| `esvo2_core/test/synthetic_scene.h` | create | synthetic textured stereo scene shared by the float unit tests |
| `esvo2_core/test/golden_harness.h` | create | loads golden cycles and builds offline BM/solver objects |
| `esvo2_core/test/test_float_mirrors.cpp`, `test_golden_capture.cpp`, `test_bm_float.cpp`, `test_depth_float.cpp`, `test_mapping_equivalence.cpp` | create | gtests |
| `esvo2_core/test/bench_mapping_float.cpp` | create | per-stage benchmark executable |
| `esvo2_core/scripts/eval_roundtrip.py` | create (copied) | still-period round-trip evaluation |
| `esvo2_core/cfg/mapping/mapping_evk4_AA_mapping.yaml` | modify | `MAPPING_FLOAT: False` with comment |
| `esvo2_core/CMakeLists.txt` | modify | new sources, test helper, benchmark target |
| `docs/superpowers/specs/2026-09-22-mapping-float-a1-results.md` | create | A1 results and verdict |

---

### Task 1: Float mirrors of the static time surfaces

**Files:**
- Modify: `esvo2_core/include/esvo2_core/container/TimeSurfaceObservation.h` (member block near line 246)
- Modify: `esvo2_core/CMakeLists.txt` (the `if(CATKIN_ENABLE_TESTING)` block at the end)
- Test: `esvo2_core/test/test_float_mirrors.cpp`

**Interfaces:**
- Produces: `Eigen::MatrixXf TimeSurfaceObservation::TS_left_f_, TS_right_f_;` and `void TimeSurfaceObservation::refreshFloatMirrors();`. It uses the existing CMake function `esvo2_core_add_lib_gtest(<name>)`, which builds `test/<name>.cpp` linked with the core library and defines `ESVO2_CORE_SOURCE_DIR`.

- [ ] **Step 1: Write the failing test**

Create `esvo2_core/test/test_float_mirrors.cpp`:

```cpp
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
```

- [ ] **Step 2: Register the test**

The CMake function `esvo2_core_add_lib_gtest` already exists (added with `test_depth_problem_temporal` in `2f279fd`). In `esvo2_core/CMakeLists.txt`, after `esvo2_core_add_lib_gtest(test_depth_problem_temporal)`, add:

```cmake
  esvo2_core_add_lib_gtest(test_float_mirrors)
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_float_mirrors 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_float_mirrors 2>&1 | tail -20`
Expected: compile error, `'struct esvo2_core::container::TimeSurfaceObservation' has no member named 'refreshFloatMirrors'`.

- [ ] **Step 4: Implement**

In `TimeSurfaceObservation.h`, directly above the line `Eigen::MatrixXd TS_left_, TS_right_, TS_last_, AA_map_, TS_last_du, TS_last_dv;`, add:

```cpp
  // Single-precision copies of TS_left_ / TS_right_, read only by the mapping
  // node's float path (MAPPING_FLOAT); tracking never reads them. Refreshed by
  // EventBM::createMatchProblem after its optional blur, so they always hold
  // exactly what the double path reads (the surfaces are 8-bit, so the cast is exact).
  inline void refreshFloatMirrors()
  {
    TS_left_f_ = TS_left_.cast<float>();
    TS_right_f_ = TS_right_.cast<float>();
  }
  Eigen::MatrixXf TS_left_f_, TS_right_f_;

```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_float_mirrors 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_float_mirrors 2>&1 | grep -E "\[  (PASSED|FAILED) \]|tests? ran"`
Expected: `[  PASSED  ] 2 tests.`

- [ ] **Step 6: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/container/TimeSurfaceObservation.h esvo2_core/CMakeLists.txt esvo2_core/test/test_float_mirrors.cpp
git commit -m "Mapping float path: float mirrors of the static time surfaces

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Golden capture file format

**Files:**
- Create: `esvo2_core/include/esvo2_core/tools/golden_capture.h`
- Create: `esvo2_core/src/tools/golden_capture.cpp`
- Modify: `esvo2_core/CMakeLists.txt` (`HEADERS`, `SOURCES`, library link line, test registration)
- Test: `esvo2_core/test/test_golden_capture.cpp`

**Interfaces:**
- Produces (namespace `esvo2_core::tools`): the structs `GoldenConfig`, `GoldenEvent`, `GoldenMatch`, `GoldenDepth` and `GoldenCycle` exactly as in Step 3, plus these functions:
  - `std::vector<std::string> configDifferences(const GoldenConfig&, const GoldenConfig&)`
  - `bool writeGoldenCycle(const std::string& dir, const GoldenConfig&, const GoldenCycle&, std::string* err)`
  - `bool readGoldenCycle(const std::string& yml_path, GoldenConfig&, GoldenCycle&, std::string* err)`
  - `std::vector<std::string> listGoldenCycles(const std::string& dir)`

- [ ] **Step 1: Write the failing test**

Create `esvo2_core/test/test_golden_capture.cpp`:

```cpp
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
```

- [ ] **Step 2: Register the test and the new library files**

In `esvo2_core/CMakeLists.txt`:
- add `include/esvo2_core/tools/golden_capture.h` to `HEADERS`, after `include/esvo2_core/tools/params_helper.h`
- add `src/tools/golden_capture.cpp` to `SOURCES`, after `src/tools/cayley.cpp`
- change `target_link_libraries(${PROJECT_NAME}_LIB ${CERES_LIBRARIES})` to `target_link_libraries(${PROJECT_NAME}_LIB ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})`
- after `esvo2_core_add_lib_gtest(test_float_mirrors)` add `esvo2_core_add_lib_gtest(test_golden_capture)`

- [ ] **Step 3: Write the header**

Create `esvo2_core/include/esvo2_core/tools/golden_capture.h`:

```cpp
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
```

- [ ] **Step 4: Run the test to verify it fails**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_golden_capture 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_golden_capture 2>&1 | tail -20`
Expected: link error, `undefined reference to esvo2_core::tools::writeGoldenCycle` (the header exists but the .cpp does not). CMake may instead stop earlier with a missing `src/tools/golden_capture.cpp`. Either counts as failing.

- [ ] **Step 5: Implement**

Create `esvo2_core/src/tools/golden_capture.cpp`:

```cpp
#include <esvo2_core/tools/golden_capture.h>

#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

namespace esvo2_core
{
namespace tools
{
namespace
{
std::string cycleStem(const std::string &dir, int cycle)
{
  char name[32];
  std::snprintf(name, sizeof(name), "cycle_%05d", cycle);
  return dir + "/" + name;
}

bool fail(std::string *err, const std::string &msg)
{
  if (err)
    *err = msg;
  return false;
}

bool surfaceToPng(const Eigen::MatrixXd &m, const std::string &path, std::string *err)
{
  if (m.size() == 0 || m.minCoeff() < 0 || m.maxCoeff() > 255 ||
      !(m.array() == m.array().round()).all())
    return fail(err, "surface is not integer-valued in [0,255]: " + path);
  cv::Mat d, u8;
  cv::eigen2cv(m, d);
  d.convertTo(u8, CV_8U);
  if (!cv::imwrite(path, u8))
    return fail(err, "cannot write " + path);
  return true;
}

bool pngToSurface(const std::string &path, Eigen::MatrixXd &m, std::string *err)
{
  cv::Mat u8 = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (u8.empty() || u8.type() != CV_8UC1)
    return fail(err, "cannot read an 8-bit image from " + path);
  cv::Mat d;
  u8.convertTo(d, CV_64F);
  cv::cv2eigen(d, m);
  return true;
}
} // namespace

std::vector<std::string> configDifferences(const GoldenConfig &a, const GoldenConfig &b)
{
  std::vector<std::string> d;
  if (a.patch_size_X != b.patch_size_X) d.push_back("patch_size_X");
  if (a.patch_size_Y != b.patch_size_Y) d.push_back("patch_size_Y");
  if (a.BM_min_disparity != b.BM_min_disparity) d.push_back("BM_min_disparity");
  if (a.BM_max_disparity != b.BM_max_disparity) d.push_back("BM_max_disparity");
  if (a.invDepth_min_range != b.invDepth_min_range) d.push_back("invDepth_min_range");
  if (a.invDepth_max_range != b.invDepth_max_range) d.push_back("invDepth_max_range");
  if (a.BM_step != b.BM_step) d.push_back("BM_step");
  if (a.BM_ZNCC_Threshold != b.BM_ZNCC_Threshold) d.push_back("BM_ZNCC_Threshold");
  if (a.PROCESS_EVENT_NUM != b.PROCESS_EVENT_NUM) d.push_back("PROCESS_EVENT_NUM");
  if (a.num_threads != b.num_threads) d.push_back("num_threads");
  if (a.LSnorm != b.LSnorm) d.push_back("LSnorm");
  if (a.Tdist_nu != b.Tdist_nu) d.push_back("Tdist_nu");
  if (a.Tdist_scale != b.Tdist_scale) d.push_back("Tdist_scale");
  if (a.calibInfoDir != b.calibInfoDir) d.push_back("calibInfoDir");
  return d;
}

bool writeGoldenCycle(const std::string &dir, const GoldenConfig &cfg,
                      const GoldenCycle &c, std::string *err)
{
  const std::string stem = cycleStem(dir, c.cycle);
  if (!surfaceToPng(c.TS_left, stem + "_left.png", err) ||
      !surfaceToPng(c.TS_right, stem + "_right.png", err))
    return false;

  cv::FileStorage fs(stem + ".yml", cv::FileStorage::WRITE);
  if (!fs.isOpened())
    return fail(err, "cannot write " + stem + ".yml");
  fs << "cycle" << c.cycle
     << "patch_size_X" << cfg.patch_size_X << "patch_size_Y" << cfg.patch_size_Y
     << "BM_min_disparity" << cfg.BM_min_disparity << "BM_max_disparity" << cfg.BM_max_disparity
     << "invDepth_min_range" << cfg.invDepth_min_range << "invDepth_max_range" << cfg.invDepth_max_range
     << "BM_step" << cfg.BM_step << "BM_ZNCC_Threshold" << cfg.BM_ZNCC_Threshold
     << "PROCESS_EVENT_NUM" << cfg.PROCESS_EVENT_NUM << "num_threads" << cfg.num_threads
     << "LSnorm" << cfg.LSnorm << "Tdist_nu" << cfg.Tdist_nu << "Tdist_scale" << cfg.Tdist_scale
     << "calibInfoDir" << cfg.calibInfoDir;

  cv::Mat pose(1, 7, CV_64F);
  for (int i = 0; i < 4; i++) pose.at<double>(0, i) = c.q_wxyz[i];
  for (int i = 0; i < 3; i++) pose.at<double>(0, 4 + i) = c.position[i];
  fs << "pose_qwxyz_xyz" << pose;

  cv::Mat ev((int)c.events.size(), 4, CV_64F);
  for (int i = 0; i < ev.rows; i++)
  {
    ev.at<double>(i, 0) = c.events[i].x;
    ev.at<double>(i, 1) = c.events[i].y;
    ev.at<double>(i, 2) = c.events[i].sec;
    ev.at<double>(i, 3) = c.events[i].nsec;
  }
  fs << "events" << ev;

  cv::Mat mt((int)c.matches.size(), 9, CV_64F);
  for (int i = 0; i < mt.rows; i++)
  {
    const GoldenMatch &m = c.matches[i];
    const double row[9] = {m.x_left_raw[0], m.x_left_raw[1], m.x_left[0], m.x_left[1],
                           m.x_right[0], m.x_right[1], m.invDepth, m.cost, m.disp};
    for (int k = 0; k < 9; k++) mt.at<double>(i, k) = row[k];
  }
  fs << "matches" << mt;

  cv::Mat dp((int)c.depths.size(), 5, CV_64F);
  for (int i = 0; i < dp.rows; i++)
  {
    const GoldenDepth &d = c.depths[i];
    const double row[5] = {d.x[0], d.x[1], d.invDepth, d.variance, d.residual};
    for (int k = 0; k < 5; k++) dp.at<double>(i, k) = row[k];
  }
  fs << "depths" << dp;
  return true;
}

bool readGoldenCycle(const std::string &yml_path, GoldenConfig &cfg,
                     GoldenCycle &c, std::string *err)
{
  cv::FileStorage fs(yml_path, cv::FileStorage::READ);
  if (!fs.isOpened())
    return fail(err, "cannot read " + yml_path);
  fs["cycle"] >> c.cycle;
  fs["patch_size_X"] >> cfg.patch_size_X;
  fs["patch_size_Y"] >> cfg.patch_size_Y;
  fs["BM_min_disparity"] >> cfg.BM_min_disparity;
  fs["BM_max_disparity"] >> cfg.BM_max_disparity;
  fs["invDepth_min_range"] >> cfg.invDepth_min_range;
  fs["invDepth_max_range"] >> cfg.invDepth_max_range;
  fs["BM_step"] >> cfg.BM_step;
  fs["BM_ZNCC_Threshold"] >> cfg.BM_ZNCC_Threshold;
  fs["PROCESS_EVENT_NUM"] >> cfg.PROCESS_EVENT_NUM;
  fs["num_threads"] >> cfg.num_threads;
  fs["LSnorm"] >> cfg.LSnorm;
  fs["Tdist_nu"] >> cfg.Tdist_nu;
  fs["Tdist_scale"] >> cfg.Tdist_scale;
  fs["calibInfoDir"] >> cfg.calibInfoDir;

  cv::Mat pose, ev, mt, dp;
  fs["pose_qwxyz_xyz"] >> pose;
  if (pose.rows != 1 || pose.cols != 7)
    return fail(err, "bad pose in " + yml_path);
  for (int i = 0; i < 4; i++) c.q_wxyz[i] = pose.at<double>(0, i);
  for (int i = 0; i < 3; i++) c.position[i] = pose.at<double>(0, 4 + i);

  fs["events"] >> ev;
  c.events.clear();
  for (int i = 0; i < ev.rows; i++)
    c.events.push_back({(uint16_t)ev.at<double>(i, 0), (uint16_t)ev.at<double>(i, 1),
                        (uint32_t)ev.at<double>(i, 2), (uint32_t)ev.at<double>(i, 3)});

  fs["matches"] >> mt;
  c.matches.clear();
  for (int i = 0; i < mt.rows; i++)
  {
    GoldenMatch m;
    m.x_left_raw = {mt.at<double>(i, 0), mt.at<double>(i, 1)};
    m.x_left = {mt.at<double>(i, 2), mt.at<double>(i, 3)};
    m.x_right = {mt.at<double>(i, 4), mt.at<double>(i, 5)};
    m.invDepth = mt.at<double>(i, 6);
    m.cost = mt.at<double>(i, 7);
    m.disp = mt.at<double>(i, 8);
    c.matches.push_back(m);
  }

  fs["depths"] >> dp;
  c.depths.clear();
  for (int i = 0; i < dp.rows; i++)
  {
    GoldenDepth d;
    d.x = {dp.at<double>(i, 0), dp.at<double>(i, 1)};
    d.invDepth = dp.at<double>(i, 2);
    d.variance = dp.at<double>(i, 3);
    d.residual = dp.at<double>(i, 4);
    c.depths.push_back(d);
  }

  const std::string stem = yml_path.substr(0, yml_path.size() - 4); // strip ".yml"
  return pngToSurface(stem + "_left.png", c.TS_left, err) &&
         pngToSurface(stem + "_right.png", c.TS_right, err);
}

std::vector<std::string> listGoldenCycles(const std::string &dir)
{
  std::vector<std::string> files;
  DIR *d = opendir(dir.c_str());
  if (!d)
    return files;
  while (dirent *e = readdir(d))
  {
    const std::string name = e->d_name;
    if (name.size() > 10 && name.compare(0, 6, "cycle_") == 0 &&
        name.compare(name.size() - 4, 4, ".yml") == 0)
      files.push_back(dir + "/" + name);
  }
  closedir(d);
  std::sort(files.begin(), files.end());
  return files;
}
} // namespace tools
} // namespace esvo2_core
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_golden_capture 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_golden_capture 2>&1 | grep -E "\[  (PASSED|FAILED) \]|FAILED"`
Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 7: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/tools/golden_capture.h esvo2_core/src/tools/golden_capture.cpp esvo2_core/CMakeLists.txt esvo2_core/test/test_golden_capture.cpp
git commit -m "Golden capture file format for offline mapping equivalence tests

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Capture golden cycles from the mapping node

**Files:**
- Modify: `esvo2_core/include/esvo2_core/esvo2_Mapping.h` (includes, method declaration, members)
- Modify: `esvo2_core/src/esvo2_Mapping.cpp` (constructor parameter block near line 144; `MappingAtTime` after `dpSolver_.solve(&vEMP, TS_obs_ptr_, vdp);` near line 411; a new method)

**Interfaces:**
- Consumes: `tools::GoldenConfig`, `tools::GoldenCycle` and `tools::writeGoldenCycle` from Task 2.
- Produces: two mapping-node params, `golden_capture_dir` (string, empty = off) and `golden_capture_every` (int, default 20). Also the capture directory `/root/datasets/evk4/golden/slide4_bias/` with `cycle_*.yml`/`*.png`, which Tasks 4–8 read.

- [ ] **Step 1: Declare**

In `esvo2_Mapping.h`, add `#include <esvo2_core/tools/golden_capture.h>` next to the other `esvo2_core` includes. In the class's private methods, add:

```cpp
    // Writes this cycle's static BM / static depth-solve inputs and outputs
    // (see tools/golden_capture.h). Called every golden_capture_every_ cycles.
    void captureGoldenCycle(const std::vector<EventMatchPair> &vEMP,
                            const std::vector<DepthPoint> &vdp);
```

In the private members, add:

```cpp
    // Golden capture: empty dir = off.
    std::string golden_capture_dir_;
    int golden_capture_every_;
    size_t golden_cycle_count_ = 0;
```

- [ ] **Step 2: Read the params**

In the constructor, directly after `distance_from_last_frame_ = tools::param(pnh_, "distance_from_last_frame", 0.04);`, add:

```cpp

    // Golden capture for offline equivalence tests (off unless a directory is given)
    golden_capture_dir_ = tools::param(pnh_, "golden_capture_dir", std::string(""));
    golden_capture_every_ = std::max(1, (int)tools::param(pnh_, "golden_capture_every", 20));
    if (!golden_capture_dir_.empty())
      LOG(INFO) << "Golden capture every " << golden_capture_every_ << " cycles into " << golden_capture_dir_;
```

- [ ] **Step 3: Hook the capture**

In `MappingAtTime`, directly after the line `dpSolver_.solve(&vEMP, TS_obs_ptr_, vdp);`, add:

```cpp
    if (!golden_capture_dir_.empty() && golden_cycle_count_++ % golden_capture_every_ == 0)
      captureGoldenCycle(vEMP, vdp);
```

This is before `pointCulling`, so `vdp` holds the solver's full output.

- [ ] **Step 4: Implement the method**

Add to `esvo2_Mapping.cpp`, after `MappingAtTime`:

```cpp
  void esvo2_Mapping::captureGoldenCycle(const std::vector<EventMatchPair> &vEMP,
                                         const std::vector<DepthPoint> &vdp)
  {
    tools::GoldenConfig cfg;
    cfg.patch_size_X = BM_patch_size_X_;
    cfg.patch_size_Y = BM_patch_size_Y_;
    // As configured: the constructor overwrites BM_min/max_disparity_ with the
    // range narrowed to [invDepth_min_range_, invDepth_max_range_].
    cfg.BM_min_disparity = tools::param(pnh_, "BM_min_disparity", 3);
    cfg.BM_max_disparity = tools::param(pnh_, "BM_max_disparity", 40);
    cfg.invDepth_min_range = invDepth_min_range_;
    cfg.invDepth_max_range = invDepth_max_range_;
    cfg.BM_step = BM_step_;
    cfg.BM_ZNCC_Threshold = BM_ZNCC_Threshold_;
    cfg.PROCESS_EVENT_NUM = PROCESS_EVENT_NUM_;
    cfg.num_threads = NUM_THREAD_MAPPING;
    cfg.LSnorm = dpConfigPtr_->LSnorm_;
    cfg.Tdist_nu = dpConfigPtr_->td_nu_;
    cfg.Tdist_scale = dpConfigPtr_->td_scale_;
    cfg.calibInfoDir = calibInfoDir_;

    tools::GoldenCycle c;
    c.cycle = (int)golden_cycle_count_ - 1;
    c.TS_left = TS_obs_ptr_->second.TS_left_;
    c.TS_right = TS_obs_ptr_->second.TS_right_;
    const Eigen::Quaterniond q = TS_obs_ptr_->second.tr_.getRotation().toImplementation();
    const Eigen::Vector3d p = TS_obs_ptr_->second.tr_.getPosition();
    c.q_wxyz = {q.w(), q.x(), q.y(), q.z()};
    c.position = {p(0), p(1), p(2)};
    c.events.reserve(vDenoisedEventsPtr_left_dx2_.size());
    for (const dvs_msgs::Event *e : vDenoisedEventsPtr_left_dx2_)
      c.events.push_back({e->x, e->y, e->ts.sec, e->ts.nsec});
    for (const EventMatchPair &m : vEMP)
    {
      tools::GoldenMatch g;
      g.x_left_raw = {m.x_left_raw_(0), m.x_left_raw_(1)};
      g.x_left = {m.x_left_(0), m.x_left_(1)};
      g.x_right = {m.x_right_(0), m.x_right_(1)};
      g.invDepth = m.invDepth_;
      g.cost = m.cost_;
      g.disp = m.disp_;
      c.matches.push_back(g);
    }
    for (const DepthPoint &d : vdp)
    {
      tools::GoldenDepth g;
      g.x = {d.x()(0), d.x()(1)};
      g.invDepth = d.invDepth();
      g.variance = d.variance();
      g.residual = d.residual();
      c.depths.push_back(g);
    }

    std::string err;
    if (tools::writeGoldenCycle(golden_capture_dir_, cfg, c, &err))
      LOG(INFO) << "Golden capture: cycle " << c.cycle << ", " << c.events.size() << " events, "
                << c.matches.size() << " matches, " << c.depths.size() << " depth points.";
    else
      LOG(ERROR) << "Golden capture failed: " << err;
  }
```

If `getRotation().toImplementation()` does not compile, check `/root/catkin_ws/src/minkindr/minkindr/include/kindr/minimal/rotation-quaternion.h` for the accessor that returns the `Eigen::Quaterniond`, and use that instead.

- [ ] **Step 5: Build**

Run: `cd /root/catkin_ws && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E "error|Error|\[100%\]" | head`
Expected: `[100%] Built target ...` with no errors.

- [ ] **Step 6: Record the capture over `slide4_bias.bag`**

Create the configs and run one 1× replay. It takes about 90 s.

```bash
G=/root/datasets/evk4/golden
mkdir -p $G/slide4_bias
cd /root/catkin_ws/src/ESVO2/esvo2_core
cp cfg/tracking/tracking_evk4_AA.yaml $G/trk_slide4.yaml
echo "GYRO_BIAS: [0.002993, -0.004062, -0.003344]" >> $G/trk_slide4.yaml
cp cfg/mapping/mapping_evk4_AA_mapping.yaml $G/map_capture.yaml
printf 'golden_capture_dir: "%s"\ngolden_capture_every: 20\n' $G/slide4_bias >> $G/map_capture.yaml
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
rosnode list >/dev/null 2>&1 || { roscore >/dev/null 2>&1 & sleep 5; }
cd /root/catkin_ws/src/ESVO2
PLAYRATE=1 esvo2_core/scripts/replay_eval.sh $G/capture_run /root/datasets/evk4/slide4_bias.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  esvo2_core/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false \
  tracking_cfg:=$G/trk_slide4.yaml mapping_cfg:=$G/map_capture.yaml
ls $G/slide4_bias/*.yml | wc -l; du -sh $G/slide4_bias; grep -c "Golden capture: cycle" $G/capture_run.log; grep -c "Golden capture failed" $G/capture_run.log
```

Expected: 30–80 `.yml` files, and the same count of `Golden capture: cycle` lines. `Golden capture failed` should appear 0 times, and the directory should be well under 0.5 GB. If `Golden capture failed ... not integer-valued` appears, stop and report it: it would mean the time surfaces are not 8-bit, which breaks a premise of the spec.

- [ ] **Step 7: Commit (code only; the capture stays outside the repo)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/esvo2_Mapping.h esvo2_core/src/esvo2_Mapping.cpp
git commit -m "esvo2_Mapping: optional golden capture of static BM and depth-solve cycles

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Offline harness, and proof that it reproduces the node

**Files:**
- Modify: `esvo2_core/include/esvo2_core/core/EventBM.h`, `esvo2_core/src/core/EventBM.cpp` (new static `disparityRange`)
- Modify: `esvo2_core/src/esvo2_Mapping.cpp` (constructor lines computing `minDisparity`/`maxDisparity`, near line 155)
- Create: `esvo2_core/test/golden_harness.h`
- Test: `esvo2_core/test/test_mapping_equivalence.cpp`
- Modify: `esvo2_core/CMakeLists.txt` (register `test_mapping_equivalence`)

**Interfaces:**
- Consumes: the Task 2 API, and the Task 3 capture at `/root/datasets/evk4/golden/slide4_bias/`.
- Produces:
  - `static std::pair<size_t, size_t> esvo2_core::core::EventBM::disparityRange(const CameraSystem &cam, double invDepth_min_range, double invDepth_max_range, size_t BM_min_disparity, size_t BM_max_disparity);`
  - In `test/golden_harness.h`, namespace `golden`:
    - `std::string goldenDir()`
    - `tools::GoldenConfig repoConfig()`
    - `struct Offline { explicit Offline(const tools::GoldenConfig&); std::unique_ptr<core::EventBM> makeBM(); std::unique_ptr<core::DepthProblemSolver> makeSolver(); ... }`
    - `std::unique_ptr<constStampedTimeSurfaceObs> makeObservation(const tools::GoldenCycle&)`
    - `std::vector<dvs_msgs::Event> makeEvents(const tools::GoldenCycle&)`
    - `std::vector<core::EventMatchPair> makeMatches(const tools::GoldenCycle&, const constStampedTimeSurfaceObs&)`
    - `double percentile(std::vector<double>, double q)`

- [ ] **Step 1: Extract `EventBM::disparityRange` (behaviour-preserving)**

In `EventBM.h`, in the `public:` section after `resetParameters(...)`, add:

```cpp
  // Static BM disparity search range: the depth range [1/invDepth_max_range,
  // 1/invDepth_min_range] converted to disparity with this camera system,
  // clipped to [BM_min_disparity, BM_max_disparity].
  static std::pair<size_t, size_t> disparityRange(
    const CameraSystem &cam, double invDepth_min_range, double invDepth_max_range,
    size_t BM_min_disparity, size_t BM_max_disparity);
```

In `EventBM.cpp`, after `resetParameters`, add:

```cpp
std::pair<size_t, size_t> esvo2_core::core::EventBM::disparityRange(
  const CameraSystem &cam, double invDepth_min_range, double invDepth_max_range,
  size_t BM_min_disparity, size_t BM_max_disparity)
{
  double f = (cam.cam_left_ptr_->P_(0, 0) + cam.cam_left_ptr_->P_(1, 1)) / 2;
  double b = cam.baseline_;
  size_t minDisparity = std::max(size_t(std::floor(f * b * invDepth_min_range)), (size_t)0);
  size_t maxDisparity = size_t(std::ceil(f * b * invDepth_max_range));
  minDisparity = std::max(minDisparity, BM_min_disparity);
  maxDisparity = std::min(maxDisparity, BM_max_disparity);
  return {minDisparity, maxDisparity};
}
```

In `esvo2_Mapping.cpp`, replace these lines in the constructor:

```cpp
    double f = (camSysPtr_->cam_left_ptr_->P_(0, 0) + camSysPtr_->cam_left_ptr_->P_(1, 1)) / 2;
    double b = camSysPtr_->baseline_;
    size_t minDisparity = max(size_t(std::floor(f * b * invDepth_min_range_)), (size_t)0);
    size_t maxDisparity = size_t(std::ceil(f * b * invDepth_max_range_));
    minDisparity = max(minDisparity, BM_min_disparity_);
    maxDisparity = min(maxDisparity, BM_max_disparity_);
```

with:

```cpp
    const std::pair<size_t, size_t> dispRange = EventBM::disparityRange(
        *camSysPtr_, invDepth_min_range_, invDepth_max_range_, BM_min_disparity_, BM_max_disparity_);
    size_t minDisparity = dispRange.first;
    size_t maxDisparity = dispRange.second;
```

If `f` or `b` is used later in the constructor, the build will fail. In that case keep the two lines that define them.

- [ ] **Step 2: Write the harness header**

Create `esvo2_core/test/golden_harness.h`:

```cpp
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
```

- [ ] **Step 3: Write the sanity test**

Create `esvo2_core/test/test_mapping_equivalence.cpp`:

```cpp
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
```

Register the test: after `esvo2_core_add_lib_gtest(test_golden_capture)` in `CMakeLists.txt`, add `esvo2_core_add_lib_gtest(test_mapping_equivalence)`.

- [ ] **Step 4: Run the test**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_mapping_equivalence 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_mapping_equivalence 2>&1 | grep -E "\[golden\]|\[  (PASSED|FAILED) \]|Failure|different config" | head -20`
Expected: `[golden] N cycles, ... reproduced` and `[  PASSED  ] 1 test.`

This test verifies the harness, not new code, so it should pass the first time. If it fails:
- **A config mismatch** means the capture is stale; redo Task 3 Step 6.
- **Exact-equality failures in `cost_`/`variance()` only, with differences below 1e-12 relative**, point to pose reconstruction rounding. Replace those `EXPECT_EQ`s with `EXPECT_NEAR(a, b, 1e-12 * std::abs(b))` and state that in the commit message.
- **Any other mismatch** (different match counts, disparities, or point counts) is a harness bug. Find and fix it before continuing: every later comparison depends on this test.

- [ ] **Step 5: Verify the node refactor**

Run: `cd /root/catkin_ws && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E "error|\[100%\]" | head -3`
Expected: builds. The sanity test above passing also shows that the capture's disparity range agrees with the one `disparityRange` computes.

- [ ] **Step 6: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/core/EventBM.h esvo2_core/src/core/EventBM.cpp esvo2_core/src/esvo2_Mapping.cpp esvo2_core/test/golden_harness.h esvo2_core/test/test_mapping_equivalence.cpp esvo2_core/CMakeLists.txt
git commit -m "Offline golden harness; EventBM::disparityRange shared with the node

The harness replays golden captures through EventBM and DepthProblemSolver
without ROS; its first test checks it reproduces the node's own output.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Float static block matching

**Files:**
- Modify: `esvo2_core/include/esvo2_core/core/EventBM.h`
- Modify: `esvo2_core/src/core/EventBM.cpp` (`createMatchProblem`, `match2`, new float methods)
- Create: `esvo2_core/test/synthetic_scene.h`
- Test: `esvo2_core/test/test_bm_float.cpp`; add a golden test to `esvo2_core/test/test_mapping_equivalence.cpp`
- Modify: `esvo2_core/test/golden_harness.h` (`Offline::makeBM` gains a `use_float` argument)
- Modify: `esvo2_core/CMakeLists.txt` (register `test_bm_float`)

**Interfaces:**
- Consumes: `TimeSurfaceObservation::TS_left_f_ / TS_right_f_ / refreshFloatMirrors()` (Task 1), and the golden harness (Task 4).
- Produces (public on `EventBM`):
  - `struct BmScratch { Eigen::VectorXf colSum, colSquareSum; std::vector<char> searching_or_not; std::vector<size_t> searching_radius; };`
  - `void setUseFloat(bool use_float);`
  - `bool useFloat() const;`
  - `static bool floatSumsAreExact(size_t patch_size_X, size_t patch_size_Y);`
  - `void prepareScratch(BmScratch &s) const;`
  - `bool match_an_event2_f(const dvs_msgs::Event *pEvent, std::pair<size_t, size_t> &pDisparityBound, EventMatchPair &emPair, BmScratch &s);`
  - In the harness: `std::unique_ptr<core::EventBM> Offline::makeBM(bool use_float = false)`.
  - In `test/synthetic_scene.h`: `struct SyntheticScene { CameraSystem::Ptr cam; std::unique_ptr<constStampedTimeSurfaceObs> obs; std::vector<dvs_msgs::Event> events; std::vector<dvs_msgs::Event*> ptrs; SyntheticScene(); };` with a true disparity of 40 px.

- [ ] **Step 1: Write the synthetic scene**

Create `esvo2_core/test/synthetic_scene.h`:

```cpp
#ifndef ESVO2_CORE_TEST_SYNTHETIC_SCENE_H
#define ESVO2_CORE_TEST_SYNTHETIC_SCENE_H

// A textured, rectified stereo pair at the EVK4 rig's resolution, where every
// left pixel reappears 40 px to the left in the right image, plus a grid of
// candidate events. Surfaces are 8-bit and blurred, like the node's.

#include <memory>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <esvo2_core/container/CameraSystem.h>
#include <esvo2_core/container/TimeSurfaceObservation.h>

struct SyntheticScene
{
  static constexpr int kDisparity = 40;
  esvo2_core::container::CameraSystem::Ptr cam;
  std::unique_ptr<esvo2_core::container::constStampedTimeSurfaceObs> obs;
  std::vector<dvs_msgs::Event> events;
  std::vector<dvs_msgs::Event *> ptrs;

  SyntheticScene()
  {
    using namespace esvo2_core::container;
    cam = std::make_shared<CameraSystem>(std::string(ESVO2_CORE_SOURCE_DIR) + "/calib/evk4_stereo", false);
    const int W = cam->cam_left_ptr_->width_, H = cam->cam_left_ptr_->height_;
    cv::Mat noise(H, W + kDisparity, CV_8UC1), tex;
    cv::RNG rng(7);
    rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(noise, tex, cv::Size(5, 5), 0.0);
    const cv::Mat left = tex(cv::Rect(0, 0, W, H)).clone();
    const cv::Mat right = tex(cv::Rect(kDisparity, 0, W, H)).clone(); // right(y,x) = left(y,x+40)

    obs.reset(new constStampedTimeSurfaceObs(ros::Time(0), TimeSurfaceObservation()));
    cv::cv2eigen(left, obs->second.TS_left_);
    cv::cv2eigen(right, obs->second.TS_right_);
    obs->second.tr_.setIdentity();

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
    for (dvs_msgs::Event &e : events)
      ptrs.push_back(&e);
  }
};

#endif // ESVO2_CORE_TEST_SYNTHETIC_SCENE_H
```

- [ ] **Step 2: Write the failing unit test**

Create `esvo2_core/test/test_bm_float.cpp`:

```cpp
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
```

Register it: after `esvo2_core_add_lib_gtest(test_mapping_equivalence)` in `CMakeLists.txt`, add `esvo2_core_add_lib_gtest(test_bm_float)`.

- [ ] **Step 3: Run the test to verify it fails**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_bm_float 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_bm_float 2>&1 | grep -E "error" | head -5`
Expected: compile errors, `'class esvo2_core::core::EventBM' has no member named 'setUseFloat'` (and similar).

- [ ] **Step 4: Declare the float path**

In `EventBM.h`, in the `public:` section after `getMSSIM(...)`, add:

```cpp
  // ---- Float path (MAPPING_FLOAT), static block matching only ----
  // Reads TimeSurfaceObservation::TS_left_f_ / TS_right_f_ through views and
  // reuses per-thread scratch, so matching an event allocates nothing. Same
  // algorithm, thresholds and counters as match_an_event2. Sums of 8-bit
  // values and products are accumulated in float, which is exact while the
  // patch has at most 258 pixels (floatSumsAreExact); ratios are taken in double.
  struct BmScratch
  {
    Eigen::VectorXf colSum, colSquareSum;
    std::vector<char> searching_or_not;
    std::vector<size_t> searching_radius;
  };
  void setUseFloat(bool use_float); // after resetParameters; aborts if the patch is too large
  bool useFloat() const { return use_float_; }
  static bool floatSumsAreExact(size_t patch_size_X, size_t patch_size_Y);
  void prepareScratch(BmScratch &s) const; // sizes s for this matcher's patch and disparity range
  bool match_an_event2_f(
    const dvs_msgs::Event *pEvent,
    std::pair<size_t, size_t>& pDisparityBound,
    EventMatchPair &emPair,
    BmScratch &s);
```

In the second `private:` section (after `triangulatePoint`), add:

```cpp
  using ConstPatchF = Eigen::Ref<const Eigen::MatrixXf, 0, Eigen::OuterStride<> >;
  bool epipolarSearchingCoarse_f(
    double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp,
    size_t searching_start_pos, size_t searching_end_pos, size_t searching_step,
    Eigen::Vector2i& x1, const ConstPatchF& patch_src, BmScratch& s, size_t nColSum,
    double mean_l, double Tl_square, double& Tr, double& Tr_square);
  bool epipolarSearchingFine_f(
    double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp,
    Eigen::Vector2i& x1, const ConstPatchF& patch_src, const BmScratch& s);
  double zncc_cost_fast_f(
    const BmScratch& s, size_t nColSum, const ConstPatchF& patch_left, const ConstPatchF& patch_right,
    int disp_to_rm, int step_to_rm, double mean_l, double Tl_square, double& Tr, double& Tr_square) const;
  double zncc_cost2_f(const ConstPatchF& patch_left, const ConstPatchF& patch_right,
                      double var_l, double mean_l) const;
  bool use_float_ = false;
```

- [ ] **Step 5: Implement**

In `EventBM.cpp`, in `createMatchProblem`, directly after the `if(bSmoothTS_) { ... }` block, add:

```cpp
  // The float path reads float copies of the (possibly blurred) surfaces.
  if(use_float_ && pStampedTsObs_)
    pStampedTsObs_->second.refreshFloatMirrors();
```

Replace the body of `match2` with:

```cpp
void esvo2_core::core::EventBM::match2(
  EventBM::Job& job)
{
  size_t i_thread = job.i_thread_;
  size_t totalNumEvents = job.pvEventPtr_->size();
  job.pvEventMatchPair_->reserve(totalNumEvents / NUM_THREAD_ + 1);
  BmScratch scratch;
  if(use_float_)
    prepareScratch(scratch);
  auto ev_it = job.pvEventPtr_->begin();
  std::advance(ev_it, i_thread);
  for(size_t i = i_thread; i < totalNumEvents; i+=NUM_THREAD_, std::advance(ev_it, NUM_THREAD_))
  {
    EventMatchPair emp;
    std::pair<size_t, size_t> pDisparityBound = (*job.pvpDisparitySearchBound_)[i];
    const bool matched = use_float_ ? match_an_event2_f(*ev_it, pDisparityBound, emp, scratch)
                                    : match_an_event2(*ev_it, pDisparityBound, emp);
    if(matched)
      job.pvEventMatchPair_->push_back(emp);
  }
}
```

Append the float path to the end of `EventBM.cpp`:

```cpp
/*************************** Float path (MAPPING_FLOAT) ***************************/

bool esvo2_core::core::EventBM::floatSumsAreExact(size_t patch_size_X, size_t patch_size_Y)
{
  // The largest sum is a patch dot product, at most area * 255^2; float holds
  // every integer up to 2^24 exactly.
  return patch_size_X * patch_size_Y * 255 * 255 <= (size_t(1) << 24);
}

void esvo2_core::core::EventBM::setUseFloat(bool use_float)
{
  if(use_float && !floatSumsAreExact(patch_size_X_, patch_size_Y_))
    LOG(FATAL) << "MAPPING_FLOAT: a " << patch_size_X_ << "x" << patch_size_Y_
               << " patch is too large for exact float sums (at most 258 pixels).";
  use_float_ = use_float;
}

void esvo2_core::core::EventBM::prepareScratch(BmScratch &s) const
{
  s.colSum.resize(patch_size_X_ + max_disparity_);
  s.colSquareSum.resize(patch_size_X_ + max_disparity_);
  s.searching_or_not.reserve(max_disparity_ + 1);
  s.searching_radius.reserve(max_disparity_ + 1);
}

bool esvo2_core::core::EventBM::match_an_event2_f(
  const dvs_msgs::Event* pEvent,
  std::pair<size_t, size_t>& pDisparityBound,
  esvo2_core::core::EventMatchPair& emPair,
  BmScratch& s)
{
  size_t lowDisparity = 1;
  size_t upDisparity  = pDisparityBound.second;

  // Prevent zncc calculation from going out of bounds
  int updisp = pEvent->x - (patch_size_X_ + 1)/2;
  if(updisp < 1)
    return false;
  if(updisp < (int)upDisparity)
    upDisparity = (size_t)updisp;
  if(upDisparity < lowDisparity)
    return false;

  Eigen::Vector2d x_rect = camSysPtr_->cam_left_ptr_->getRectifiedUndistortedCoordinate(pEvent->x, pEvent->y);
  if(x_rect(0) < 0 || x_rect(0) > camSysPtr_->cam_left_ptr_->width_ - 1 ||
     x_rect(1) < 0 || x_rect(1) > camSysPtr_->cam_left_ptr_->height_ - 1)
  {
    infoNoiseRatioLowNum_++;
    return false;
  }
  if(camSysPtr_->cam_left_ptr_->UndistortRectify_mask_((int)x_rect(1), (int)x_rect(0)) <= 125)
  {
    infoNoiseRatioLowNum_++;
    return false;
  }
  Eigen::Vector2i x1(std::floor(x_rect(0)), std::floor(x_rect(1)));
  Eigen::Vector2i x1_left_top;
  if(!isValidPatch(x1, x1_left_top, patch_size_Y_, patch_size_X_))
  {
    infoNoiseRatioLowNum_++;
    return false;
  }

  const Eigen::MatrixXf &TS_left = pStampedTsObs_->second.TS_left_f_;
  const Eigen::MatrixXf &TS_right = pStampedTsObs_->second.TS_right_f_;
  const ConstPatchF patch_src = TS_left.block(x1_left_top(1), x1_left_top(0), patch_size_Y_, patch_size_X_);
  if((patch_src.array() < 1).count() > 0.95 * patch_src.size())
  {
    infoNoiseRatioLowNum_++;
    return false;
  }

  double min_cost = ZNCC_MAX_;
  Eigen::Vector2i bestMatch;
  size_t bestDisp;
  s.searching_or_not.assign(upDisparity - lowDisparity + 1, 0);

  // Running sums for the fast ZNCC ("Optimizing ZNCC calculation in binocular
  // stereo matching"), as in match_an_event2, over a view of the right strip.
  const double n = patch_src.size();
  const double mean_l = patch_src.sum() / n;
  const double Tl_square = patch_src.array().square().sum();
  if(x1_left_top(0) - (int)upDisparity < 0)
    return false;
  const ConstPatchF strip = TS_right.block(x1_left_top(1), x1_left_top(0) - (int)upDisparity,
                                           patch_size_Y_, patch_size_X_ + upDisparity);
  const size_t nColSum = strip.cols();
  s.colSum.head(nColSum).noalias() = strip.colwise().sum().transpose();
  s.colSquareSum.head(nColSum).noalias() = strip.array().square().colwise().sum().matrix().transpose();
  double Tr = 0, Tr_square = 0;
  for(int m = lowDisparity; m < (int)(lowDisparity + patch_src.cols()) && ((int)nColSum - 1 - m >= 0); m++)
  {
    Tr += s.colSum(nColSum - 1 - m);
    Tr_square += s.colSquareSum(nColSum - 1 - m);
  }

  if(!epipolarSearchingCoarse_f(min_cost, bestMatch, bestDisp,
    lowDisparity, upDisparity, step_,
    x1, patch_src, s, nColSum, mean_l, Tl_square, Tr, Tr_square))
  {
    coarseSearchingFailNum_++;
    return false;
  }

  s.searching_radius.clear();
  for(size_t i = 0; i < s.searching_or_not.size(); i++)
    if(s.searching_or_not[i])
      s.searching_radius.push_back(lowDisparity + i);
  if(step_ > 1)
  {
    if(!epipolarSearchingFine_f(min_cost, bestMatch, bestDisp, x1, patch_src, s))
    {
      fineSearchingFailNum_++;
      return false;
    }
  }

  // transfer best match to emPair
  if(min_cost <= ZNCC_Threshold_*1.02)
  {
    emPair.x_left_raw_ = Eigen::Vector2d((double)pEvent->x, (double)pEvent->y);
    emPair.x_left_ = x_rect;
    emPair.x_right_ = Eigen::Vector2d((double)bestMatch(0), (double)bestMatch(1)) ;
    emPair.t_ = pEvent->ts;
    double disparity;
    if(bUpDownConfiguration_)
      disparity = x1(1) - bestMatch(1);
    else
      disparity = x1(0) - bestMatch(0);
    double depth = camSysPtr_->baseline_ * camSysPtr_->cam_left_ptr_->P_(0,0) / disparity;
    emPair.trans_ = pStampedTsObs_->second.tr_;
    emPair.invDepth_ = 1.0 / depth;
    emPair.cost_ = min_cost;
    emPair.disp_ = disparity;
    return true;
  }
  return false;
}

bool esvo2_core::core::EventBM::epipolarSearchingCoarse_f(
  double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp,
  size_t searching_start_pos, size_t searching_end_pos, size_t searching_step,
  Eigen::Vector2i& x1, const ConstPatchF& patch_src, BmScratch& s, size_t nColSum,
  double mean_l, double Tl_square, double& Tr, double& Tr_square)
{
  const Eigen::MatrixXf &TS_right = pStampedTsObs_->second.TS_right_f_;
  for(size_t disp = searching_start_pos; disp <= searching_end_pos; disp += searching_step)
  {
    Eigen::Vector2i x2;
    if(!bUpDownConfiguration_)
      x2 << x1(0) - disp, x1(1);
    else
      x2 << x1(0), x1(1) - disp;
    Eigen::Vector2i x2_left_top;
    if(!isValidPatch(x2, x2_left_top, patch_size_Y_, patch_size_X_))
      continue;
    const ConstPatchF patch_dst = TS_right.block(x2_left_top(1), x2_left_top(0), patch_size_Y_, patch_size_X_);
    const double cost = zncc_cost_fast_f(s, nColSum, patch_src, patch_dst, (int)disp, (int)searching_step,
                                         mean_l, Tl_square, Tr, Tr_square);

    // add to preliminarily match list
    if(cost <= ZNCC_Threshold_*1.035)
    {
      const int rel = (int)disp - (int)searching_start_pos;
      for(int i = rel - (int)searching_step; i < rel + (int)searching_step + 1; i++)
        if(i >= 0 && i < (int)s.searching_or_not.size())
          s.searching_or_not[i] = 1;
    }
    if(cost <= min_cost)
    {
      min_cost = cost;
      bestMatch = x2;
      bestDisp = disp;
    }
  }
  return min_cost < ZNCC_Threshold_*1.03;
}

bool esvo2_core::core::EventBM::epipolarSearchingFine_f(
  double& min_cost, Eigen::Vector2i& bestMatch, size_t& bestDisp,
  Eigen::Vector2i& x1, const ConstPatchF& patch_src, const BmScratch& s)
{
  const Eigen::MatrixXf &TS_right = pStampedTsObs_->second.TS_right_f_;
  const double n = patch_src.size();
  const double mean_l = patch_src.sum() / n;
  const double var_l = (double)patch_src.array().square().sum() - n * mean_l * mean_l;
  for(size_t i = 0; i < s.searching_radius.size(); i++)
  {
    Eigen::Vector2i x2;
    size_t disp = s.searching_radius[i];
    if(!bUpDownConfiguration_)
      x2 << x1(0) - disp, x1(1);
    else
      x2 << x1(0), x1(1) - disp;
    Eigen::Vector2i x2_left_top;
    if(!isValidPatch(x2, x2_left_top, patch_size_Y_, patch_size_X_))
      continue;
    const ConstPatchF patch_dst = TS_right.block(x2_left_top(1), x2_left_top(0), patch_size_Y_, patch_size_X_);
    const double cost = zncc_cost2_f(patch_src, patch_dst, var_l, mean_l);
    if(cost <= min_cost)
    {
      min_cost = cost;
      bestMatch = x2;
      bestDisp = disp;
    }
  }
  return min_cost < ZNCC_Threshold_*1.02;
}

double esvo2_core::core::EventBM::zncc_cost_fast_f(
  const BmScratch& s, size_t nColSum, const ConstPatchF& patch_left, const ConstPatchF& patch_right,
  int disp_to_rm, int step_to_rm, double mean_l, double Tl_square, double& Tr, double& Tr_square) const
{
  // Same arithmetic, in the same order, as zncc_cost_fast.
  const double n = patch_right.rows() * patch_right.cols();
  double cost;
  double mean_r = Tr / n;
  if(mean_r == 0.0)
    mean_r = 1e-3;
  if(abs(mean_l - mean_r) / mean_l  > 5  || abs(mean_l - mean_r) / mean_r  > 5)
    cost = 0;
  else
  {
    const double cov = (double)(patch_left.array() * patch_right.array()).sum() - mean_l * Tr;
    const double var_l = Tl_square - n * mean_l * mean_l;
    const double var_r = Tr_square - Tr * Tr / n;
    if(var_l * var_r == 0)
      cost = 0;
    else
      cost = cov / sqrt(var_l * var_r);
  }

  const int last = (int)nColSum - 1;
  const int cols = (int)patch_right.cols();
  for(int i = 0; i < step_to_rm; i++)
  {
    if(last - disp_to_rm - cols > 0)
    {
      Tr = Tr - s.colSum[last - disp_to_rm] + s.colSum[last - disp_to_rm - cols];
      Tr_square = Tr_square - s.colSquareSum[last - disp_to_rm] + s.colSquareSum[last - disp_to_rm - cols];
      disp_to_rm++;
    }
  }
  return 0.5 * (1 - cost);
}

double esvo2_core::core::EventBM::zncc_cost2_f(
  const ConstPatchF& patch_left, const ConstPatchF& patch_right, double var_l, double mean_l) const
{
  // Same arithmetic, in the same order, as zncc_cost2.
  const double n = patch_right.size();
  const double mean_r = patch_right.sum() / n;
  const double cov = (double)(patch_left.array() * patch_right.array()).sum() - n * mean_l * mean_r;
  const double var_r = (double)patch_right.array().square().sum() - n * mean_r * mean_r;
  double cost;
  if(var_l * var_r == 0)
    cost = 0;
  else
    cost = cov / sqrt(var_l * var_r);
  return 0.5 * (1 - cost);
}
```

`abs` and `sqrt` are deliberately unqualified, as in `zncc_cost_fast` and `zncc_cost2`, so both paths resolve to the same overloads.

- [ ] **Step 6: Run the unit test to verify it passes**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_bm_float 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_bm_float 2>&1 | grep -E "\[  (PASSED|FAILED) \]|Failure" | head`
Expected: `[  PASSED  ] 4 tests.`

If `SameDecisionAndMatchPerEvent` fails on exact equality, diff the double and float arithmetic operation by operation. The claim in the spec's planning note 3 is that they are identical. Do not loosen the test to a tolerance without finding and reporting the cause.

- [ ] **Step 7: Add the golden BM equivalence test**

In `golden_harness.h`, replace `makeBM` with:

```cpp
  // Surfaces are captured after the node's blur, so no smoothing here.
  std::unique_ptr<core::EventBM> makeBM(bool use_float = false)
  {
    std::unique_ptr<core::EventBM> bm(new core::EventBM(cam, cfg.num_threads, false));
    bm->resetParameters(cfg.patch_size_X, cfg.patch_size_Y, disp.first, disp.second,
                        cfg.BM_step, cfg.BM_ZNCC_Threshold, false);
    bm->setUseFloat(use_float);
    return bm;
  }
```

Append to `test_mapping_equivalence.cpp`:

```cpp
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
```

- [ ] **Step 8: Run all tests**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_mapping_equivalence test_bm_float 2>&1 | grep error; { /root/catkin_ws/devel/lib/esvo2_core/test_mapping_equivalence 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_bm_float 2>&1; } | grep -E "\[golden|\[  (PASSED|FAILED) \]|Failure"`
Expected: both suites `PASSED`. `OfflineDoubleReproducesTheNode` is still green, and `[golden BM]` reports 100% same decisions with all differences 0.

- [ ] **Step 9: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/core/EventBM.h esvo2_core/src/core/EventBM.cpp esvo2_core/test/synthetic_scene.h esvo2_core/test/test_bm_float.cpp esvo2_core/test/golden_harness.h esvo2_core/test/test_mapping_equivalence.cpp esvo2_core/CMakeLists.txt
git commit -m "EventBM: float static block matching without per-event allocation

Reads float mirrors through views, reuses per-thread scratch. Selected by
setUseFloat (off by default). Equal to the double path on synthetic data and
on the golden capture.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Float static depth solve

**Files:**
- Modify: `esvo2_core/include/esvo2_core/core/DepthProblem.h`, `esvo2_core/src/core/DepthProblem.cpp`
- Modify: `esvo2_core/include/esvo2_core/core/DepthProblemSolver.h`, `esvo2_core/src/core/DepthProblemSolver.cpp`
- Test: `esvo2_core/test/test_depth_float.cpp`; add a golden test to `esvo2_core/test/test_mapping_equivalence.cpp`
- Modify: `esvo2_core/test/golden_harness.h` (`Offline::makeSolver` gains `use_float`)
- Modify: `esvo2_core/CMakeLists.txt` (register `test_depth_float`)

**Interfaces:**
- Consumes: the float mirrors (Task 1), `EventBM::setUseFloat` (Task 5, used by the test to refresh the mirrors), and `SyntheticScene` (Task 5).
- Produces:
  - `static constexpr size_t DepthProblem::kMaxPatchArea = 256;`
  - `int DepthProblem::residualsFloat(double invDepth, double *fvec) const;`
  - `bool DepthProblem::patchInterpolationFloat(const Eigen::MatrixXf &img, const Eigen::Vector2d &location, double *patch) const;`
  - `void DepthProblemSolver::setUseFloat(bool use_float);`
  - `bool DepthProblemSolver::useFloat() const;`
  - `std::unique_ptr<core::DepthProblemSolver> Offline::makeSolver(bool use_float = false)`

- [ ] **Step 1: Write the failing unit test**

Create `esvo2_core/test/test_depth_float.cpp`:

```cpp
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
```

Register it: after `esvo2_core_add_lib_gtest(test_bm_float)`, add `esvo2_core_add_lib_gtest(test_depth_float)`.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_depth_float 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_depth_float 2>&1 | grep error | head -3`
Expected: `'class esvo2_core::core::DepthProblemSolver' has no member named 'setUseFloat'`.

- [ ] **Step 3: Float residual in `DepthProblem`**

In `DepthProblem.h`, inside `struct DepthProblem`, after the second `patchInterpolation` declaration, add:

```cpp
  // ---- Float path (MAPPING_FLOAT), static (left-right) problems only ----
  // Same residual as operator() for LSnorm Tdist and l2, reading
  // TimeSurfaceObservation::TS_left_f_ / TS_right_f_; writes wx*wy values to
  // fvec and allocates nothing.
  static constexpr size_t kMaxPatchArea = 256;
  int residualsFloat(double invDepth, double *fvec) const;
  // Bilinear patch of wy x wx around location, row-major into patch[y*wx + x].
  bool patchInterpolationFloat(
    const Eigen::MatrixXf &img,
    const Eigen::Vector2d &location,
    double *patch) const;
```

In `DepthProblem.cpp`, before the closing `}// core`, add:

```cpp
int DepthProblem::residualsFloat(double invDepth, double *fvec) const
{
  const size_t wx = dpConfigPtr_->patchSize_X_;
  const size_t wy = dpConfigPtr_->patchSize_Y_;
  const size_t patchSize = wx * wy;
  const bool l2 = dpConfigPtr_->LSnorm_ == "l2";
  Eigen::Vector2d x1_s, x2_s;
  double tau1[kMaxPatchArea], tau2[kMaxPatchArea];
  const bool pass = warping(coordinate_, invDepth, vT_left_virtual_[0], x1_s, x2_s) &&
                    patchInterpolationFloat(pStampedTsObs_->second.TS_left_f_, x1_s, tau1) &&
                    patchInterpolationFloat(pStampedTsObs_->second.TS_right_f_, x2_s, tau2);
  if (!pass)
  {
    // the same constant residuals operator() assigns when the patch is lost
    for (size_t i = 0; i < patchSize; i++)
    {
      if (l2)
        fvec[i] = 255;
      else
      {
        double residual = 255;
        double weight = (dpConfigPtr_->td_nu_ + 1) / (dpConfigPtr_->td_nu_ + std::pow(residual / dpConfigPtr_->td_scale_, 2));
        fvec[i] = sqrt(weight) * residual;
      }
    }
    return 0;
  }

  if (l2)
  {
    for (size_t index = 0; index < patchSize; index++)
      fvec[index] = tau1[index] - tau2[index];
    return 1;
  }

  // Tdist, as in operator()
  double vResidual[kMaxPatchArea], vResidualSquared[kMaxPatchArea];
  double scaleSquaredTmp1 = dpConfigPtr_->td_scaleSquared_;
  double scaleSquaredTmp2 = -1.0;
  bool first_iteration = true;
  // loop for scale until it converges
  while (fabs(scaleSquaredTmp2 - scaleSquaredTmp1) / scaleSquaredTmp1 > 0.05 || first_iteration)
  {
    if (!first_iteration)
      scaleSquaredTmp1 = scaleSquaredTmp2;

    double sum_scaleSquared = 0;
    for (size_t index = 0; index < patchSize; index++)
    {
      if (first_iteration)
      {
        vResidual[index] = tau1[index] - tau2[index];
        vResidualSquared[index] = std::pow(vResidual[index], 2);
      }
      if (vResidual[index] != 0)
        sum_scaleSquared += vResidualSquared[index] * (dpConfigPtr_->td_nu_ + 1) /
                            (dpConfigPtr_->td_nu_ + vResidualSquared[index] / scaleSquaredTmp1);
    }
    if (sum_scaleSquared == 0)
    {
      scaleSquaredTmp2 = dpConfigPtr_->td_scaleSquared_;
      break;
    }
    scaleSquaredTmp2 = sum_scaleSquared / patchSize;
    first_iteration = false;
  }

  // assign reweighted residual
  for (size_t index = 0; index < patchSize; index++)
  {
    double weight = (dpConfigPtr_->td_nu_ + 1) / (dpConfigPtr_->td_nu_ + vResidualSquared[index] / scaleSquaredTmp2);
    fvec[index] = sqrt(weight) * vResidual[index];
  }
  return 1;
}

bool DepthProblem::patchInterpolationFloat(
  const Eigen::MatrixXf &img,
  const Eigen::Vector2d &location,
  double *patch) const
{
  // Same bounds checks and arithmetic order as patchInterpolation.
  const int wx = dpConfigPtr_->patchSize_X_;
  const int wy = dpConfigPtr_->patchSize_Y_;
  const int lx = (int)floor(location[0]);
  const int ly = (int)floor(location[1]);
  const int left = lx - (wx - 1) / 2;
  const int top = ly - (wy - 1) / 2;
  if (left < 0 || top < 0)
    return false;
  if (lx + (wx - 1) / 2 >= img.cols() || ly + (wy - 1) / 2 >= img.rows())
    return false;
  if (top + wy >= img.rows() || left + wx >= img.cols())
    return false;

  const double q1 = (lx + 1) - location[0]; // x
  const double q2 = location[0] - lx;       // x
  const double q3 = (ly + 1) - location[1]; // y
  const double q4 = location[1] - ly;       // y
  for (int y = 0; y < wy; y++)
    for (int x = 0; x < wx; x++)
    {
      const double r0 = q1 * img(top + y, left + x) + q2 * img(top + y, left + x + 1);
      const double r1 = q1 * img(top + y + 1, left + x) + q2 * img(top + y + 1, left + x + 1);
      patch[y * wx + x] = q3 * r0 + q4 * r1;
    }
  return true;
}
```

- [ ] **Step 4: Float path in `DepthProblemSolver`, with the shared point construction extracted**

In `DepthProblemSolver.h`, in the `public:` section after `init_single_point`, add:

```cpp
  // ---- Float path (MAPPING_FLOAT), static (slove_lr) solver only ----
  // Aborts for the temporal solver, for LSnorm other than Tdist/l2, or for a
  // patch above DepthProblem::kMaxPatchArea. solve() then runs
  // init_single_point_f, which reads the float mirrors that
  // EventBM::createMatchProblem refreshed and allocates nothing per point.
  void setUseFloat(bool use_float);
  bool useFloat() const { return use_float_; }
  bool init_single_point_f(Job & job);
```

In its `private:` section, add:

```cpp
  // Builds the DepthPoint for one solved event and appends it to job.vdpPtr_.
  void appendDepthPoint(Job &job, const Eigen::Vector2d &coor, const float result[3],
                        Eigen::Matrix<double, 4, 4> &T_world_virtual);
  bool use_float_ = false;
```

In `DepthProblemSolver.cpp`, in `init_single_point`, replace the block from `DepthPoint dp(std::floor(coor(1)), std::floor(coor(0)));` through `job.vdpPtr_->push_back(dp);` with:

```cpp
      appendDepthPoint(job, coor, result, T_world_virtual);
```

Then add these functions after `init_single_point`:

```cpp
void DepthProblemSolver::appendDepthPoint(
  Job &job, const Eigen::Vector2d &coor, const float result[3],
  Eigen::Matrix<double, 4, 4> &T_world_virtual)
{
  DepthPoint dp(std::floor(coor(1)), std::floor(coor(0)));
  dp.update_x(coor);
  Eigen::Vector3d p_cam;
  camSysPtr_->cam_left_ptr_->cam2World(coor, result[0], p_cam);
  dp.update_p_cam(p_cam);
  if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0 || strcmp(dpConfigPtr_->LSnorm_.c_str(), "zncc") == 0)
    dp.update(result[0], result[1]);
  else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
  {
    double scale2_rho = result[1] * (dpConfigPtr_->td_nu_ - 2) / dpConfigPtr_->td_nu_;
    dp.update_studentT(result[0], scale2_rho, result[1], dpConfigPtr_->td_nu_);
  }
  else
    exit(-1);
  dp.residual() = result[2];
  dp.updatePose(T_world_virtual);
  job.vdpPtr_->push_back(dp);
}

void DepthProblemSolver::setUseFloat(bool use_float)
{
  if(use_float)
  {
    if(!slove_lr_)
      LOG(FATAL) << "MAPPING_FLOAT covers the static (left-right) depth solve only.";
    if(dpConfigPtr_->LSnorm_ != "Tdist" && dpConfigPtr_->LSnorm_ != "l2")
      LOG(FATAL) << "MAPPING_FLOAT supports LSnorm Tdist or l2, not " << dpConfigPtr_->LSnorm_;
    if(dpConfigPtr_->patchSize_X_ * dpConfigPtr_->patchSize_Y_ > DepthProblem::kMaxPatchArea)
      LOG(FATAL) << "MAPPING_FLOAT: depth patch " << dpConfigPtr_->patchSize_X_ << "x"
                 << dpConfigPtr_->patchSize_Y_ << " exceeds " << DepthProblem::kMaxPatchArea << " pixels.";
  }
  use_float_ = use_float;
}

bool DepthProblemSolver::init_single_point_f(
  Job & job)
{
  size_t i_thread = job.i_thread_;
  size_t numEvent = job.pvEMP_->size();
  job.vdpPtr_->clear();
  job.vdpPtr_->reserve(numEvent / NUM_THREAD_ + 1);

  constStampedTimeSurfaceObs* pStampedTsObs = job.pStamped_TS_obs_;
  Eigen::Matrix<double, 4, 4> T_world_virtual = pStampedTsObs->second.tr_.getTransformationMatrix();
  const size_t n = dpConfigPtr_->patchSize_X_ * dpConfigPtr_->patchSize_Y_;
  alignas(16) double val1[DepthProblem::kMaxPatchArea], val2[DepthProblem::kMaxPatchArea];
  Eigen::Map<const Eigen::VectorXd, Eigen::Aligned16> v1(val1, n), v2(val2, n);
  for(size_t i = i_thread; i < numEvent; i+=NUM_THREAD_)
  {
    Eigen::Vector2d coor = (*job.pvEMP_)[i].x_left_;
    double d_init = (*job.pvEMP_)[i].invDepth_;
    if(i == i_thread)
      job.dProblemPtr_->setProblem(coor, T_world_virtual, pStampedTsObs, slove_lr_);
    else
      job.dProblemPtr_->setProblem(coor);

    // Same two residual evaluations and finite difference as init_single_point.
    job.dProblemPtr_->residualsFloat(d_init, val1);
    const double h = d_init * 0.05;
    job.dProblemPtr_->residualsFloat(d_init + h, val2);
    double R = ((v2 - v1) / h).norm();
    if(R < 0.1 && R > -0.1)
      R = 0.1;

    float result[3];
    result[0] = d_init;
    result[1] = std::pow(dpConfigPtr_->td_stdvar_, 2) / (R*R);
    result[2] = v1.norm() * v1.norm();
    appendDepthPoint(job, coor, result, T_world_virtual);
  }
  return true;
}
```

In `solve`, replace:

```cpp
    threads.emplace_back(std::bind(&DepthProblemSolver::init_single_point, this, jobs[i]));
```

with:

```cpp
    threads.emplace_back(std::bind(use_float_ ? &DepthProblemSolver::init_single_point_f
                                              : &DepthProblemSolver::init_single_point,
                                   this, jobs[i]));
```

At the top of `solve`, after the opening brace, add:

```cpp
  if(use_float_ && (pStampedTsObs->second.TS_left_f_.rows() != pStampedTsObs->second.TS_left_.rows() ||
                    pStampedTsObs->second.TS_left_f_.cols() != pStampedTsObs->second.TS_left_.cols()))
    LOG(FATAL) << "MAPPING_FLOAT: float time surfaces not prepared; EventBM::createMatchProblem "
                  "(with setUseFloat(true)) must run before the float depth solve.";
```

- [ ] **Step 5: Run the unit test to verify it passes**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_depth_float 2>&1 | grep -E 'error' ; /root/catkin_ws/devel/lib/esvo2_core/test_depth_float 2>&1 | grep -E "\[  (PASSED|FAILED) \]|Failure" | head`
Expected: `[  PASSED  ] 2 tests.`

- [ ] **Step 6: Add the golden depth equivalence test**

In `golden_harness.h`, replace `makeSolver` with:

```cpp
  // Each solver gets its own config: DepthProblemSolver halves the patch size in place.
  std::unique_ptr<core::DepthProblemSolver> makeSolver(bool use_float = false)
  {
    dpConfigs.push_back(std::make_shared<core::DepthProblemConfig>(
      cfg.patch_size_X, cfg.patch_size_Y, cfg.LSnorm, cfg.Tdist_nu, cfg.Tdist_scale, 1, 5, 8, 8));
    std::unique_ptr<core::DepthProblemSolver> s(new core::DepthProblemSolver(
      cam, dpConfigs.back(), core::NUMERICAL, cfg.num_threads, true));
    s->setUseFloat(use_float);
    return s;
  }
```

Append to `test_mapping_equivalence.cpp`:

```cpp
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
```

- [ ] **Step 7: Run every esvo2_core test**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core >/dev/null 2>&1; cd build && make -j6 test_mapping_equivalence test_bm_float test_depth_float test_golden_capture test_float_mirrors test_gyro_bias test_gyro_prediction test_depth_problem_temporal 2>&1 | grep error; { /root/catkin_ws/devel/lib/esvo2_core/test_mapping_equivalence 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_bm_float 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_depth_float 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_golden_capture 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_float_mirrors 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_gyro_bias 2>&1; /root/catkin_ws/devel/lib/esvo2_core/test_gyro_prediction 2>&1; } | grep -E "\[golden|\[  (PASSED|FAILED) \]|Failure"`
Expected: every suite `PASSED`, including `OfflineDoubleReproducesTheNode` (which proves the `appendDepthPoint` extraction preserved the double path). `[golden depth]` should report 100% within 0.1%.

- [ ] **Step 8: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/core/DepthProblem.h esvo2_core/src/core/DepthProblem.cpp esvo2_core/include/esvo2_core/core/DepthProblemSolver.h esvo2_core/src/core/DepthProblemSolver.cpp esvo2_core/test/test_depth_float.cpp esvo2_core/test/golden_harness.h esvo2_core/test/test_mapping_equivalence.cpp esvo2_core/CMakeLists.txt
git commit -m "DepthProblemSolver: float static depth solve without per-point allocation

Static (left-right) solver only; the temporal solver stays double (A1 scope).
The DepthPoint construction is shared by both paths.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: `MAPPING_FLOAT` in the mapping node

**Files:**
- Modify: `esvo2_core/include/esvo2_core/esvo2_Mapping.h` (member)
- Modify: `esvo2_core/src/esvo2_Mapping.cpp` (constructor, after `ebm_.resetParameters(...)` and the `BM_max_disparity_ = maxDisparity;` line)
- Modify: `esvo2_core/cfg/mapping/mapping_evk4_AA_mapping.yaml`

**Interfaces:**
- Consumes: `EventBM::setUseFloat` (Task 5) and `DepthProblemSolver::setUseFloat` (Task 6).
- Produces: mapping-node param `MAPPING_FLOAT` (bool, default false). It is also present in the EVK4 YAML as `False`.

- [ ] **Step 1: Wire the param**

In `esvo2_Mapping.h`, next to `bool bRegularization_;`, add `bool bMappingFloat_;`.

In the constructor, directly after `BM_max_disparity_ = maxDisparity;`, add:

```cpp

    // Float static BM and static depth solve (GPU programme step A1). After
    // resetParameters, which sets the patch size setUseFloat checks.
    bMappingFloat_ = tools::param(pnh_, "MAPPING_FLOAT", false);
    ebm_.setUseFloat(bMappingFloat_);
    dpSolver_.setUseFloat(bMappingFloat_);
    if (bMappingFloat_ && !golden_capture_dir_.empty())
    {
      LOG(WARNING) << "golden_capture_dir ignored: the golden capture records the double path, and MAPPING_FLOAT is on.";
      golden_capture_dir_.clear();
    }
    LOG(INFO) << "MAPPING_FLOAT: " << (bMappingFloat_ ? "on (static BM and depth solve in float)" : "off");
```

Check that `golden_capture_dir_` is assigned earlier in the constructor than this block (Task 3 put it after `distance_from_last_frame_`). If it is not, move the Task 3 lines above this block.

- [ ] **Step 2: Add the YAML key**

In `mapping_evk4_AA_mapping.yaml`, directly after the `PROCESS_EVENT_NUM_AA: 4000 #20000` line, add:

```yaml
# Static block matching and the static depth solve read float copies of the
# time surfaces and allocate nothing per event (GPU programme step A1, see
# docs/superpowers/specs/2026-09-22-mapping-float-a1-design.md). Off until A1's
# validation passes; the temporal path always stays double.
MAPPING_FLOAT: False
```

- [ ] **Step 3: Build and run all tests**

Run the Task 6 Step 7 command.
Expected: all `PASSED`. `repoConfig()` does not read `MAPPING_FLOAT`, so the golden tests are unaffected.

- [ ] **Step 4: Smoke-test the node with the flag on**

```bash
G=/root/datasets/evk4/golden
sed 's/^MAPPING_FLOAT: False/MAPPING_FLOAT: True/' /root/catkin_ws/src/ESVO2/esvo2_core/cfg/mapping/mapping_evk4_AA_mapping.yaml > $G/map_float.yaml
grep -c "^MAPPING_FLOAT: True" $G/map_float.yaml
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
rosnode list >/dev/null 2>&1 || { roscore >/dev/null 2>&1 & sleep 5; }
cd /root/catkin_ws/src/ESVO2
PLAYRATE=1 PLAYARGS="-u 40" esvo2_core/scripts/replay_eval.sh $G/smoke_float /root/datasets/evk4/slide4_bias.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  esvo2_core/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false \
  tracking_cfg:=$G/trk_slide4.yaml mapping_cfg:=$G/map_float.yaml
grep -m1 "MAPPING_FLOAT" $G/smoke_float.log; grep -c "FATAL\|Segmentation\|core dumped" $G/smoke_float.log
```

Expected: `1` from the first `grep -c`. The log shows `MAPPING_FLOAT: on (static BM and depth solve in float)`, and the crash count is `0`. The replay summary line should show an SGM initialisation and running tracking. Compare the reset count with a double-path run only loosely, since 40 s is a short run.

- [ ] **Step 5: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/esvo2_Mapping.h esvo2_core/src/esvo2_Mapping.cpp esvo2_core/cfg/mapping/mapping_evk4_AA_mapping.yaml
git commit -m "esvo2_Mapping: MAPPING_FLOAT switch (off by default)

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Per-stage benchmark

**Files:**
- Create: `esvo2_core/test/bench_mapping_float.cpp`
- Modify: `esvo2_core/CMakeLists.txt` (benchmark target inside the testing block)

**Interfaces:**
- Consumes: the golden harness with `makeBM(bool)` and `makeSolver(bool)` (Tasks 5–6), and `tools::TicToc`.
- Produces: the executable `/root/catkin_ws/devel/lib/esvo2_core/bench_mapping_float [golden_dir] [repetitions]`, which prints a markdown table.

- [ ] **Step 1: Write the benchmark**

Create `esvo2_core/test/bench_mapping_float.cpp`:

```cpp
// Times static BM and the static depth solve, double vs float, over every
// golden cycle, repeated to average out noise. Prints a markdown table.
// usage: bench_mapping_float [golden_dir] [repetitions=5]
// Run on an otherwise idle machine.

#include <cstdio>
#include <esvo2_core/tools/TicToc.h>
#include "golden_harness.h"

using namespace golden;

int main(int argc, char **argv)
{
  const std::string dir = argc > 1 ? argv[1] : goldenDir();
  const int reps = argc > 2 ? std::atoi(argv[2]) : 5;
  const std::vector<std::string> files = tools::listGoldenCycles(dir);
  if (files.empty())
  {
    std::fprintf(stderr, "no golden capture in %s\n", dir.c_str());
    return 1;
  }

  std::vector<double> bm[2], dp[2], mirror;
  size_t events = 0, matches = 0;
  for (int rep = 0; rep < reps; rep++)
    for (const std::string &file : files)
    {
      tools::GoldenConfig cfg;
      tools::GoldenCycle c;
      std::string err;
      if (!tools::readGoldenCycle(file, cfg, c, &err))
      {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
      }
      Offline off(cfg);
      std::unique_ptr<constStampedTimeSurfaceObs> obs = makeObservation(c);
      std::vector<dvs_msgs::Event> evs = makeEvents(c);
      std::vector<dvs_msgs::Event *> ptrs = pointersTo(evs);
      for (int f = 0; f < 2; f++)
      {
        std::unique_ptr<core::EventBM> m = off.makeBM(f == 1);
        std::unique_ptr<core::DepthProblemSolver> s = off.makeSolver(f == 1);
        std::vector<core::EventMatchPair> vEMP;
        std::vector<DepthPoint> vdp;
        tools::TicToc t;
        m->createMatchProblem(obs.get(), nullptr, &ptrs); // float: includes refreshing the mirrors
        m->match_all_HyperThread(vEMP);
        bm[f].push_back(t.toc());
        t.tic();
        s->solve(&vEMP, obs.get(), vdp);
        dp[f].push_back(t.toc());
        if (rep == 0 && f == 0)
        {
          events += ptrs.size();
          matches += vEMP.size();
        }
      }
      tools::TicToc t;
      obs->second.refreshFloatMirrors();
      mirror.push_back(t.toc());
    }

  std::printf("%zu cycles x %d repetitions; %zu candidate events, %zu matches per repetition\n\n",
              files.size(), reps, events, matches);
  std::printf("| stage | double median | double p90 | float median | float p90 | speedup (median) |\n");
  std::printf("|---|---|---|---|---|---|\n");
  const char *names[2] = {"static BM (ms)", "static depth solve (ms)"};
  std::vector<double> *stages[2] = {bm, dp};
  for (int k = 0; k < 2; k++)
  {
    const double d50 = percentile(stages[k][0], 0.5), f50 = percentile(stages[k][1], 0.5);
    std::printf("| %s | %.2f | %.2f | %.2f | %.2f | %.2fx |\n", names[k], d50,
                percentile(stages[k][0], 0.9), f50, percentile(stages[k][1], 0.9), d50 / f50);
  }
  std::printf("| float mirror refresh (ms, inside float BM) | - | - | %.2f | %.2f | - |\n",
              percentile(mirror, 0.5), percentile(mirror, 0.9));
  return 0;
}
```

- [ ] **Step 2: Register it**

Inside the `if(CATKIN_ENABLE_TESTING)` block, after the last `esvo2_core_add_lib_gtest(...)` line, add:

```cmake
  cs_add_executable(bench_mapping_float test/bench_mapping_float.cpp)
  target_link_libraries(bench_mapping_float ${PROJECT_NAME}_LIB
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
  target_compile_definitions(bench_mapping_float PRIVATE ESVO2_CORE_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
```

- [ ] **Step 3: Build and run on an idle machine**

```bash
cd /root/catkin_ws && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E "error|bench_mapping_float" | head -3
pgrep -f "rosbag play|esvo2_Mapping" && echo "BUSY: wait for replays to finish" || \
  /root/catkin_ws/devel/lib/esvo2_core/bench_mapping_float /root/datasets/evk4/golden/slide4_bias 5 | tee /root/datasets/evk4/golden/bench_a1.md
```

Expected: a table with both stages. Record the numbers as measured, whatever they show. The spec accepts a small speedup (A1 is still groundwork), but the number must be reported honestly.

- [ ] **Step 4: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/test/bench_mapping_float.cpp esvo2_core/CMakeLists.txt
git commit -m "Benchmark: static BM and depth solve, double vs float, on golden cycles

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 9: Tracking confirmation and results

**Files:**
- Create: `esvo2_core/scripts/eval_roundtrip.py` (copied)
- Create: `docs/superpowers/specs/2026-09-22-mapping-float-a1-results.md`

**Interfaces:**
- Consumes: `$G/map_float.yaml` and `$G/trk_slide4.yaml` (Tasks 3, 7), `bench_a1.md` (Task 8), and the test outputs (Tasks 4–6).
- Produces: the results document, with a verdict for each acceptance criterion.

- [ ] **Step 1: Bring the evaluation script into the repo**

```bash
cp /tmp/claude-0/-root/ad7f23c7-5147-4161-a2c7-63cb1fa073d9/scratchpad/eval_roundtrip.py /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_roundtrip.py
chmod +x /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_roundtrip.py
head -12 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_roundtrip.py
```

Expected: the docstring starts `Evaluate an out-and-back slide replay, in ABSOLUTE timestamps.` If the scratch file is gone, stop and report it. Do not rewrite the metric: the 63–76% band was measured with this exact script.

- [ ] **Step 2: Three 1× replays with `MAPPING_FLOAT: True`**

Run them one after another, in the foreground (each takes about 90 s):

```bash
G=/root/datasets/evk4/golden
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
rosnode list >/dev/null 2>&1 || { roscore >/dev/null 2>&1 & sleep 5; }
cd /root/catkin_ws/src/ESVO2
for r in 1 2 3; do
  PLAYRATE=1 esvo2_core/scripts/replay_eval.sh $G/float_run$r /root/datasets/evk4/slide4_bias.bag \
    "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
    esvo2_core/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false \
    tracking_cfg:=$G/trk_slide4.yaml mapping_cfg:=$G/map_float.yaml | tail -1
  python3 esvo2_core/scripts/eval_roundtrip.py $G/float_run$r.bag /root/datasets/evk4/slide4_bias.bag 0.02
done
```

For each run, the two real legs are the two largest `|dx|` displacements, near bag IMU time 24–32.5 s (out) and 48.5–58 s (back). Leg recovery is `|dx| / 1.0 m`.

Pass: every leg of every run is in 63–76%.

Ruling (known issue, independent of A1): the pipeline sometimes starts tracking late, in about 1 run in 3, on the double path too. It then misses the first leg, which shows as a missing or tiny first displacement and a late first pose. Rerun such a run, up to 2 extra runs in total, and report every run including the reruns. A run that tracks both legs but falls outside the band is a fail, not a rerun.

- [ ] **Step 3: Write the results**

Create `docs/superpowers/specs/2026-09-22-mapping-float-a1-results.md` with these sections, filled from the actual outputs:

```markdown
# A1 results: float static mapping path

Date: <date>. Spec: `2026-09-22-mapping-float-a1-design.md`. Commits: <first>..<last>.

## Equivalence (golden capture: <N> cycles of slide4_bias.bag, every 20th)
- Harness reproduces the node: <pass/fail, counts>
- Static BM: same decision <x>% (need >= 99%); disparity |d| max <x> px (need <= 1); invDepth rel max <x> (need <= 0.1%)
- Static depth solve: within 0.1%: <x>% of <n> points (need >= 99.9%); variance rel p99/max <x>/<x>

## Benchmark (5 repetitions, idle machine)
<paste bench_a1.md table>

## Tracking (1x replay, MAPPING_FLOAT: True; double path reference 63-76% per leg)
| run | out leg | back leg | closure | resets | note |
|---|---|---|---|---|---|
<one row per run, including any rerun and why>

## Verdict
<For each acceptance criterion: met or not, with the number. Then one paragraph
on what the speedup means for the A2 decision.>
```

- [ ] **Step 4: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/scripts/eval_roundtrip.py docs/superpowers/specs/2026-09-22-mapping-float-a1-results.md
git commit -m "A1 results: float static mapping path equivalence, speed, tracking

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## After this plan (not part of it)

If the results meet the acceptance criteria, the user decides whether to flip `MAPPING_FLOAT` to `True` in the committed YAML and delete the double static BM and depth-solve paths (spec §3). The spec treats these as a follow-up commit and this plan does not include them.
