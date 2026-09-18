# Gyro-locked tracking rotation and gyro bias correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Take tracking rotation from the bias-corrected gyro and solve translation only, so the optimizer cannot trade a sideways slide for yaw.

**Architecture:** A header-only `GyroBiasEstimator` (no ROS) finds the gyro bias from the first still window. `esvo2_Tracking` subtracts the bias (configured `GYRO_BIAS` or estimated) at integration time in its existing gyro rotation prediction. When `IMU_ROTATION_LOCK` is on and the frame has a bias-corrected prediction, it tells the registration problem to zero the rotation columns of its Jacobian, so LM solves translation only. Replay tooling lets every gate run offline on recorded bags.

**Tech Stack:** C++14, Eigen, ROS Noetic (catkin_make), glog, gtest; Python 3 (rosbag, numpy, PyYAML) for evaluation scripts.

**Spec:** `docs/superpowers/specs/2026-09-18-gyro-locked-tracking-design.md`

## Global Constraints

- Branch: `evk4-gyro-locked-tracking` in `/root/catkin_ws/src/ESVO2`. Commit per task; never commit the `DBG-TMP` instrumentation.
- Commit messages end with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- New tracking keys, all optional: `IMU_ROTATION_LOCK` (default `False`), `GYRO_BIAS` (`[bx, by, bz]`, rad/s, IMU frame; unset by default), `GYRO_BIAS_WINDOW` (default `2.0` s), `GYRO_STILL_MAX_STD` (default `0.0035` rad/s). Max sample gap inside a still window: `0.02` s.
- Configs without `IMU_ROTATION_PREDICTION: True` must behave exactly as before.
- The lock works with the analytical solver only (`RegProblemType: 1`); otherwise warn once and ignore it.
- Build: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core`. Do not use `--only-pkg-with-deps` (it leaves a package whitelist in the build cache).
- Replays need a running ROS master (`roscore`) and nothing else publishing `/evk4_*` or `/davis/*` topics.
- Gyro bias of the slide bag (its still period, 10.5–23 s): `0.003425,-0.004210,-0.003853` rad/s.

## File map

| File | Change | Responsibility |
|---|---|---|
| `esvo2_core/src/esvo2_Tracking.cpp`, `esvo2_core/src/core/RegProblemLM.cpp` | modify | Task 1: revert `DBG-TMP`. Tasks 3–4: bias and lock. |
| `esvo2_core/launch/system/system_evk4_mapping.launch` | modify | Args for sim time, configs and TS rate (replay) |
| `esvo2_core/launch/system/system_upenn.launch` | modify | Args for configs and GUI (replay) |
| `esvo2_core/scripts/replay_eval.sh` | create | Play a bag through a launch file, record poses |
| `esvo2_core/scripts/eval_slide.py` | create | Slide bag metrics: x, tracked vs gyro yaw |
| `esvo2_core/scripts/eval_traj.py` | create | Trajectory vs ground truth: path ratio, ATE |
| `esvo2_core/include/esvo2_core/tools/gyro_bias.h` | create | `GyroBiasEstimator`, `subtractBias` |
| `esvo2_core/test/test_gyro_bias.cpp` | create | gtest for `gyro_bias.h` |
| `esvo2_core/CMakeLists.txt` | modify | Register `test_gyro_bias` |
| `esvo2_core/include/esvo2_core/esvo2_Tracking.h` | modify | Bias/lock members, prediction signature |
| `esvo2_core/include/esvo2_core/core/RegProblemLM.h`, `RegProblemSolverLM.h`, `src/core/RegProblemSolverLM.cpp` | modify | `setFixRotation` |
| `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml` | modify | Enable the lock for the rig |
| `EVK4_STEREO_SETUP.md` | modify | Findings, keys, still start |

---

### Task 1: Clean slate, replay tooling, and baseline

Removes the temporary instrumentation, makes offline replays reproducible from the repo, and records the current baseline for the slide bag (gate G2 before any change).

**Files:**
- Modify: `esvo2_core/src/esvo2_Tracking.cpp`, `esvo2_core/src/core/RegProblemLM.cpp` (revert only)
- Modify: `esvo2_core/launch/system/system_evk4_mapping.launch`
- Modify: `esvo2_core/launch/system/system_upenn.launch`
- Create: `esvo2_core/scripts/replay_eval.sh`, `esvo2_core/scripts/eval_slide.py`, `esvo2_core/scripts/eval_traj.py`

**Interfaces:**
- Produces: `replay_eval.sh OUT_BAG BAG "TOPICS" LAUNCH_FILE [roslaunch args...]` (env `PLAYRATE`, `PLAYARGS`); writes `OUT_BAG` (poses) and `OUT_BAG.log` (node output). `eval_slide.py POSES.bag SOURCE.bag CALIB_DIR BX,BY,BZ` prints a line with `min_x=<m>` and `max_dyaw=<deg>`. `eval_traj.py GT.txt POSES.bag [label]` prints `ratio <r>`. Launch args: `system_evk4_mapping.launch` gains `use_sim_time`, `mapping_cfg`, `tracking_cfg`, `ts_rate`; `system_upenn.launch` gains `mapping_cfg`, `tracking_cfg`, `gui`.

- [ ] **Step 1: Revert the temporary instrumentation**

Every change in these two files is `DBG-TMP` test code (check with `git diff`), so restore them:

```bash
cd /root/catkin_ws/src/ESVO2
git diff --stat esvo2_core/src/esvo2_Tracking.cpp esvo2_core/src/core/RegProblemLM.cpp
git checkout -- esvo2_core/src/esvo2_Tracking.cpp esvo2_core/src/core/RegProblemLM.cpp
grep -rn "DBG-TMP" esvo2_core/ || echo "clean"
```
Expected: `clean`.

- [ ] **Step 2: Move the slide bag somewhere durable**

```bash
mkdir -p /root/datasets/evk4
mv /tmp/claude-0/-root/ad7f23c7-5147-4161-a2c7-63cb1fa073d9/scratchpad/slide_lr.bag /root/datasets/evk4/slide_lr.bag
ls -la /root/datasets/evk4/slide_lr.bag
```
Expected: ~1.4 GB file. (If the scratchpad is gone, the bag must be re-recorded on the rig: 1 m slide left, 2 m from a textured wall, topics `/evk4_left/events /evk4_right/events /imu/data_synced /imu/data`.)

- [ ] **Step 3: Add replay args to `system_evk4_mapping.launch`**

Replace
```xml
  <rosparam param="/use_sim_time">false</rosparam>
```
with
```xml
  <!-- Replay of a recorded bag: use_sim_time:=true, gui:=false, and optionally
       other configs / time-surface rate (see scripts/replay_eval.sh) -->
  <arg name="use_sim_time" default="false"/>
  <param name="/use_sim_time" value="$(arg use_sim_time)"/>
  <arg name="mapping_cfg" default="$(find esvo2_core)/cfg/mapping/mapping_evk4_AA_mapping.yaml"/>
  <arg name="tracking_cfg" default="$(find esvo2_core)/cfg/tracking/tracking_evk4_AA.yaml"/>
  <arg name="ts_rate" default="25"/>
```
Replace both occurrences of `<param name="generation_rate_hz" value="25" />` with `<param name="generation_rate_hz" value="$(arg ts_rate)" />`.
Replace `file="$(find esvo2_core)/cfg/mapping/mapping_evk4_AA_mapping.yaml"` with `file="$(arg mapping_cfg)"` and `file="$(find esvo2_core)/cfg/tracking/tracking_evk4_AA.yaml"` with `file="$(arg tracking_cfg)"`.

- [ ] **Step 4: Add replay args to `system_upenn.launch`**

After `<rosparam param="/use_sim_time">true</rosparam>` add:
```xml
  <arg name="mapping_cfg" default="$(find esvo2_core)/cfg/mapping/mapping_upenn_AA.yaml"/>
  <arg name="tracking_cfg" default="$(find esvo2_core)/cfg/tracking/tracking_upenn_AA.yaml"/>
  <arg name="gui" default="true"/>
```
Replace `file="$(find esvo2_core)/cfg/mapping/mapping_upenn_AA.yaml"` with `file="$(arg mapping_cfg)"`, `file="$(find esvo2_core)/cfg/tracking/tracking_upenn_AA.yaml"` with `file="$(arg tracking_cfg)"`, and wrap the two visualization nodes:
```xml
  <!-- Visualization -->
  <group if="$(arg gui)">
    <node pkg="rqt_gui" type="rqt_gui" name="rqt_gui" args="--perspective-file $(find esvo2_core)/esvo2_system_DSEC.perspective" />
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find esvo2_core)/esvo2_system.rviz" />
  </group>
```
Check both files parse: `source /root/catkin_ws/devel/setup.bash && roslaunch --files esvo2_core system_evk4_mapping.launch && roslaunch --files esvo2_core system_upenn.launch`. Expected: file lists, no error.

- [ ] **Step 5: Create `esvo2_core/scripts/replay_eval.sh`**

```bash
#!/bin/bash
# Replay a bag through an ESVO2 launch file and record the tracked poses.
# usage: replay_eval.sh OUT_BAG BAG "TOPICS" LAUNCH_FILE [roslaunch args...]
#   OUT_BAG  poses (/esvo2_tracking/pose_pub) are recorded here; node output goes to OUT_BAG.log
#   TOPICS   space-separated topics to play from BAG
# env: PLAYRATE (default 1), PLAYARGS (extra rosbag play args, e.g. "-s 10.5")
# Needs a running roscore. Prints SGM init and tracking reset counts at the end.
set -u
OUT=$1; BAG=$2; TOPICS=$3; LAUNCH=$4; shift 4
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
roslaunch --wait "$LAUNCH" "$@" > "$OUT.log" 2>&1 &
LPID=$!
sleep 6
rosbag record -O "$OUT" /esvo2_tracking/pose_pub __name:=replay_eval_record > /dev/null 2>&1 &
RPID=$!
sleep 1
rosbag play --clock -q -r "${PLAYRATE:-1}" ${PLAYARGS:-} "$BAG" --topics $TOPICS
sleep 2
kill -INT $RPID; wait $RPID 2>/dev/null
kill -INT $LPID; wait $LPID 2>/dev/null
echo "SGM inits: $(grep -c 'Initialization (SGM)' "$OUT.log"), tracking resets: $(grep -c 're-initialized' "$OUT.log")"
```
`chmod +x esvo2_core/scripts/replay_eval.sh`

- [ ] **Step 6: Create `esvo2_core/scripts/eval_slide.py`**

```python
#!/usr/bin/env python3
"""Metrics for a replayed sideways slide: tracked translation, and tracked yaw vs gyro yaw.

usage: eval_slide.py POSES.bag SOURCE.bag CALIB_DIR BX,BY,BZ
  POSES.bag   /esvo2_tracking/pose_pub recorded during the replay (replay_eval.sh)
  SOURCE.bag  the replayed bag (read for /imu/data_synced)
  CALIB_DIR   calibration folder (T_b_c from left.yaml)
  BX,BY,BZ    gyro bias, rad/s, IMU frame

Everything is expressed in the camera frame of the first tracked pose (x right,
y down, z forward); yaw is rotation about camera y. The IMU/camera time offset
(a few ms) is ignored: it is negligible for yaw accumulated over seconds.
Prints x every 2 s, then: min_x=<m> final_x=<m> max_dyaw=<deg>.
"""
import sys
import numpy as np
import rosbag
import yaml


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(R):
    a = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


poses_bag, src_bag, calib_dir, bias_s = sys.argv[1:5]
bias = np.array([float(v) for v in bias_s.split(",")])
R_b_c = np.array(yaml.safe_load(open(calib_dir + "/left.yaml"))["T_b_c"]["data"]).reshape(3, 4)[:, :3]

P = np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
               m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)
              for _, m, _ in rosbag.Bag(poses_bag).read_messages(topics=["/esvo2_tracking/pose_pub"])])
if len(P) < 10:
    sys.exit("too few poses: %d" % len(P))
G = np.array([(m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
              for _, m, _ in rosbag.Bag(src_bag).read_messages(topics=["/imu/data_synced"])])

t = P[:, 0] - P[0, 0]
R0 = q2R(*P[0, 4:])
d = (P[:, 1:4] - P[0, 1:4]) @ R0                       # camera frame of the first pose
yaw_trk = np.degrees([rotvec(R0.T @ q2R(*p[4:]))[1] for p in P])

G = G[G[:, 0] >= P[0, 0]]
w_cam = (R_b_c.T @ (G[:, 1:] - bias).T).T               # bias-corrected rate, camera frame
yaw_gyro_samples = np.degrees(np.concatenate([[0.0], np.cumsum(w_cam[1:, 1] * np.diff(G[:, 0]))]))
yaw_gyro = np.interp(P[:, 0], G[:, 0], yaw_gyro_samples)

row = " ".join("%+.2f" % d[min(np.searchsorted(t, k), len(t) - 1), 0] for k in range(0, int(t[-1]) + 1, 2))
print("poses %d  x every 2 s: %s" % (len(P), row))
print("min_x=%.3f final_x=%.3f max_dyaw=%.2f" % (d[:, 0].min(), d[-1, 0], np.abs(yaw_trk - yaw_gyro).max()))
```
`chmod +x esvo2_core/scripts/eval_slide.py`

- [ ] **Step 7: Create `esvo2_core/scripts/eval_traj.py`**

```python
#!/usr/bin/env python3
"""Compare an estimated trajectory with ground truth.

usage: eval_traj.py GT.txt POSES.bag [label]
  GT.txt     "t x y z qx qy qz qw" per line (results/gt/... format)
  POSES.bag  /esvo2_tracking/pose_pub recorded during a replay
Poses are matched to ground truth within 20 ms. Path length is measured on
0.5 s samples so per-frame jitter does not inflate it. Prints the path ratio
(estimate / ground truth), the Sim3 scale, and ATE after SE3 and Sim3 alignment.
"""
import sys
import numpy as np
import rosbag

gt = np.loadtxt(sys.argv[1])[:, :4]
est = np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z)
                for _, m, _ in rosbag.Bag(sys.argv[2]).read_messages(topics=["/esvo2_tracking/pose_pub"])])
label = sys.argv[3] if len(sys.argv) > 3 else sys.argv[2]

i = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
i = np.where(np.abs(gt[i - 1, 0] - est[:, 0]) < np.abs(gt[i, 0] - est[:, 0]), i - 1, i)
ok = np.abs(gt[i, 0] - est[:, 0]) < 0.02
A, B = est[ok, 1:4], gt[i[ok], 1:4]


def align(A, B, with_scale):
    ma, mb = A.mean(0), B.mean(0)
    a, b = A - ma, B - mb
    U, D, Vt = np.linalg.svd(b.T @ a / len(A))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / a.var(0).sum() if with_scale else 1.0
    return s, np.sqrt((((s * (R @ a.T)).T - b) ** 2).sum(1).mean())


T = est[ok, 0]
keep = np.searchsorted(T, np.arange(T[0], T[-1], 0.5))
path = lambda X: np.linalg.norm(np.diff(X[keep], axis=0), axis=1).sum()
_, ate = align(A, B, False)
s, ate_s = align(A, B, True)
print("%s: matched %d/%d over %.1f s | path est %.2f m, gt %.2f m, ratio %.2f | Sim3 scale %.2f | ATE SE3 %.3f m, Sim3 %.3f m"
      % (label, ok.sum(), len(est), T[-1] - T[0], path(A), path(B), path(A) / path(B), s, ate, ate_s))
```
`chmod +x esvo2_core/scripts/eval_traj.py`

- [ ] **Step 8: Build and record the baseline (G2 before any change)**

```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E " error|Built target esvo2_Tracking"
source /root/catkin_ws/devel/setup.bash
pgrep -x rosmaster > /dev/null || (roscore > /dev/null 2>&1 &) ; sleep 3
O=/root/datasets/evk4/replays; mkdir -p $O
PLAYRATE=0.25 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/replay_eval.sh $O/baseline.bag /root/datasets/evk4/slide_lr.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  $(rospack find esvo2_core)/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false
python3 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_slide.py $O/baseline.bag /root/datasets/evk4/slide_lr.bag \
  $(rospack find esvo2_core)/calib/evk4_stereo 0.003425,-0.004210,-0.003853
```
Expected: `Built target esvo2_Tracking`; `SGM inits` ≥ 1; `min_x` between −0.15 and −0.30 (earlier replays: −0.20 to −0.26). Record the numbers in the commit message.

- [ ] **Step 9: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/launch/system/system_evk4_mapping.launch esvo2_core/launch/system/system_upenn.launch esvo2_core/scripts/replay_eval.sh esvo2_core/scripts/eval_slide.py esvo2_core/scripts/eval_traj.py
git commit -m "Replay tooling: launch args, replay_eval.sh, eval_slide.py, eval_traj.py

Slide bag baseline (0.25x): min_x=<value> max_dyaw=<value>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: `GyroBiasEstimator` and `subtractBias`

**Files:**
- Create: `esvo2_core/include/esvo2_core/tools/gyro_bias.h`
- Create: `esvo2_core/test/test_gyro_bias.cpp`
- Modify: `esvo2_core/CMakeLists.txt` (inside `if(CATKIN_ENABLE_TESTING)`)

**Interfaces:**
- Consumes: `tools::GyroSample {double t; Eigen::Vector3d w;}`, `tools::gyroDeltaRotation`, `tools::so3Exp` from `gyro_prediction.h`.
- Produces (namespace `esvo2_core::tools`):
  - `class GyroBiasEstimator { GyroBiasEstimator(double window_s, double max_std, double max_gap = 0.02); bool add(const GyroSample &s); bool hasBias() const; Eigen::Vector3d bias() const; Eigen::Vector3d biasStd() const; double window() const; }`
  - `void subtractBias(std::vector<GyroSample> &samples, const Eigen::Vector3d &bias);`

- [ ] **Step 1: Write the failing tests**

Create `esvo2_core/test/test_gyro_bias.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Geometry>
#include <esvo2_core/tools/gyro_bias.h>

using esvo2_core::tools::GyroBiasEstimator;
using esvo2_core::tools::GyroSample;
using esvo2_core::tools::gyroDeltaRotation;
using esvo2_core::tools::so3Exp;
using esvo2_core::tools::subtractBias;

static const Eigen::Vector3d kBias(0.003425, -0.004210, -0.003853);

// Feeds samples every dt from t0 to t1 (inclusive); returns the time of the sample on
// which add() returned true, or -1 if it never did.
template <typename RateFn>
static double feed(GyroBiasEstimator &e, double t0, double t1, double dt, RateFn rate)
{
  double accepted_at = -1.0;
  for (double t = t0; t <= t1 + 1e-12; t += dt)
    if (e.add({t, rate(t)}) && accepted_at < 0)
      accepted_at = t;
  return accepted_at;
}

// Still: bias plus +/-0.0005 rad/s alternating noise (std 0.0005, mean exactly 0 over pairs).
static Eigen::Vector3d stillRate(double t)
{
  const double n = (static_cast<long>(std::llround(t / 0.005)) % 2 == 0) ? 0.0005 : -0.0005;
  return kBias + Eigen::Vector3d(n, -n, n);
}

TEST(GyroBias, StillWindowGivesItsMean)
{
  GyroBiasEstimator e(2.0, 0.0035);
  const double at = feed(e, 0.0, 2.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_NEAR(at, 2.0, 0.006);
  EXPECT_LT((e.bias() - kBias).norm(), 1e-5);
  EXPECT_NEAR(e.biasStd().x(), 0.0005, 1e-5);
}

TEST(GyroBias, MovingWindowRejectedThenStillAccepted)
{
  GyroBiasEstimator e(2.0, 0.0035);
  // hand-held motion: ~0.07 rad/s swings for the first 2 s
  feed(e, 0.0, 2.0, 0.005, [](double t) { return Eigen::Vector3d(kBias + 0.1 * std::sin(2 * M_PI * t) * Eigen::Vector3d::Ones()); });
  EXPECT_FALSE(e.hasBias());
  const double at = feed(e, 2.005, 4.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_NEAR(at, 4.005, 0.006);
  EXPECT_LT((e.bias() - kBias).norm(), 2e-5);
}

TEST(GyroBias, WindowWithGapRejected)
{
  GyroBiasEstimator e(2.0, 0.0035);
  feed(e, 0.0, 1.0, 0.005, stillRate);
  feed(e, 1.03, 2.0, 0.005, stillRate);  // 30 ms gap inside the first window
  EXPECT_FALSE(e.hasBias());
  const double at = feed(e, 2.005, 4.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_GT(at, 4.0);
}

TEST(GyroBias, IgnoresSamplesAfterAcceptance)
{
  GyroBiasEstimator e(2.0, 0.0035);
  feed(e, 0.0, 2.5, 0.005, stillRate);
  ASSERT_TRUE(e.hasBias());
  EXPECT_FALSE(e.add({2.6, Eigen::Vector3d(1, 1, 1)}));
  EXPECT_LT((e.bias() - kBias).norm(), 1e-5);
}

TEST(GyroBias, SubtractedBiasLeavesTrueRotation)
{
  const Eigen::Vector3d w(0.1, -0.4, 0.2);
  std::vector<GyroSample> s;
  for (double t = 0.0; t <= 1.0 + 1e-12; t += 0.005)
    s.push_back({t, w + kBias});
  subtractBias(s, kBias);
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.40, 0.44, 0.0, R));
  EXPECT_LT(Eigen::AngleAxisd(R.transpose() * so3Exp(w * 0.04)).angle(), 1e-9);
}
```

Register it in `esvo2_core/CMakeLists.txt`, after the `test_gyro_prediction` block and still inside `if(CATKIN_ENABLE_TESTING)`:
```cmake
  catkin_add_gtest(test_gyro_bias test/test_gyro_bias.cpp)
  if(TARGET test_gyro_bias)
    target_link_libraries(test_gyro_bias gtest_main)
  endif()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core run_tests_esvo2_core_gtest_test_gyro_bias 2>&1 | grep -E "error|gyro_bias.h" | head -5`
Expected: compile error, `esvo2_core/tools/gyro_bias.h: No such file or directory`.

- [ ] **Step 3: Implement `gyro_bias.h`**

Create `esvo2_core/include/esvo2_core/tools/gyro_bias.h`:
```cpp
#ifndef ESVO2_CORE_TOOLS_GYRO_BIAS_H
#define ESVO2_CORE_TOOLS_GYRO_BIAS_H

#include <vector>
#include <Eigen/Core>
#include <esvo2_core/tools/gyro_prediction.h>

namespace esvo2_core
{
namespace tools
{
// Estimates a constant gyro bias from the first window in which the rig is still.
// Samples are grouped into consecutive, non-overlapping windows of window_s seconds
// (a window closes on the first sample at least window_s after its first sample).
// A window is accepted when every axis' standard deviation is below max_std and no two
// consecutive samples in it are more than max_gap apart. The first accepted window's mean
// is the bias; samples after that are ignored. A rejected window is discarded and the next
// one starts at the next sample.
class GyroBiasEstimator
{
public:
  GyroBiasEstimator(double window_s, double max_std, double max_gap = 0.02)
    : window_s_(window_s), max_std_(max_std), max_gap_(max_gap)
  {
    resetWindow();
  }

  // Returns true on the sample that completes the first accepted window.
  bool add(const GyroSample &s)
  {
    if (has_bias_)
      return false;
    if (n_ == 0)
      t_start_ = s.t;
    else if (s.t - t_last_ > max_gap_)
      gap_ = true;
    sum_ += s.w;
    sum_sq_ += s.w.cwiseProduct(s.w);
    n_++;
    t_last_ = s.t;
    if (s.t - t_start_ < window_s_)
      return false;

    const Eigen::Vector3d mean = sum_ / static_cast<double>(n_);
    const Eigen::Vector3d var = (sum_sq_ / static_cast<double>(n_) - mean.cwiseProduct(mean)).cwiseMax(0.0);
    const Eigen::Vector3d sd = var.cwiseSqrt();
    const bool still = !gap_ && n_ >= 2 && sd.maxCoeff() < max_std_;
    if (still)
    {
      bias_ = mean;
      bias_std_ = sd;
      has_bias_ = true;
    }
    resetWindow();
    return still;
  }

  bool hasBias() const { return has_bias_; }
  Eigen::Vector3d bias() const { return bias_; }
  Eigen::Vector3d biasStd() const { return bias_std_; }
  double window() const { return window_s_; }

private:
  void resetWindow()
  {
    sum_.setZero();
    sum_sq_.setZero();
    n_ = 0;
    gap_ = false;
  }

  double window_s_, max_std_, max_gap_;
  Eigen::Vector3d sum_, sum_sq_;
  size_t n_ = 0;
  double t_start_ = 0.0, t_last_ = 0.0;
  bool gap_ = false;
  bool has_bias_ = false;
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero(), bias_std_ = Eigen::Vector3d::Zero();
};

// Removes a constant bias from every sample (rates in the same frame as the bias).
inline void subtractBias(std::vector<GyroSample> &samples, const Eigen::Vector3d &bias)
{
  for (auto &s : samples)
    s.w -= bias;
}
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_GYRO_BIAS_H
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash
catkin_make -j4 --pkg esvo2_core run_tests_esvo2_core_gtest_test_gyro_bias 2>&1 | grep -E '\[  (PASSED|FAILED) |tests? ran' | head -5
catkin_test_results build/test_results/esvo2_core
```
Expected: `[  PASSED  ] 5 tests.` and `0 errors, 0 failures`. Also re-run `run_tests_esvo2_core_gtest_test_gyro_prediction` to confirm it still passes.

- [ ] **Step 5: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/tools/gyro_bias.h esvo2_core/test/test_gyro_bias.cpp esvo2_core/CMakeLists.txt
git commit -m "Add GyroBiasEstimator (first still window) and subtractBias, with gtests

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Bias-corrected gyro prediction in tracking

**Files:**
- Modify: `esvo2_core/include/esvo2_core/esvo2_Tracking.h` (includes; `predictRotationWithGyro` declaration at line 67; gyro members at lines 159–167)
- Modify: `esvo2_core/src/esvo2_Tracking.cpp` (constructor after line 57; `imuPredictionCallback` ~line 725; call site ~line 327; `predictRotationWithGyro` ~line 736)

**Interfaces:**
- Consumes: `tools::GyroBiasEstimator`, `tools::subtractBias` (Task 2).
- Produces: `bool esvo2_Tracking::predictRotationWithGyro(double t_prev_frame, double t_cur_frame, bool &biasCorrected)`: returns true when the prediction was applied; `biasCorrected` is true when a bias was subtracted. Members `bGyroBiasKnown_`, `gyroBias_` (both guarded by `gyro_mutex_`).

- [ ] **Step 1: Header changes**

In `esvo2_Tracking.h`, next to `#include <esvo2_core/tools/gyro_prediction.h>` add:
```cpp
#include <esvo2_core/tools/gyro_bias.h>
#include <memory>
```
Replace the declaration
```cpp
    void predictRotationWithGyro(double t_prev_frame, double t_cur_frame);
```
with
```cpp
    // Returns true if the gyro prediction was applied; biasCorrected tells whether a gyro bias was removed.
    bool predictRotationWithGyro(double t_prev_frame, double t_cur_frame, bool &biasCorrected);
```
After `bool bGyroJumpWarned_ = false; // re-arms once a sample is accepted normally again` add:
```cpp
    // gyro bias: GYRO_BIAS if configured, else estimated from the first still window
    std::unique_ptr<tools::GyroBiasEstimator> gyroBiasEstimator_; // null when GYRO_BIAS is configured
    bool bGyroBiasKnown_ = false;                          // guarded by gyro_mutex_
    Eigen::Vector3d gyroBias_ = Eigen::Vector3d::Zero();   // IMU frame, rad/s; guarded by gyro_mutex_
```

- [ ] **Step 2: Constructor: bias source**

In `esvo2_Tracking.cpp`, after `imuTimeOffset_ = tools::param(pnh_, "IMU_TIME_OFFSET", 0.0);` add:
```cpp
  // tools::param cannot print a vector, so GYRO_BIAS is read directly.
  std::vector<double> vGyroBias;
  if (pnh_.getParam("GYRO_BIAS", vGyroBias) && vGyroBias.size() == 3)
  {
    gyroBias_ = Eigen::Vector3d(vGyroBias[0], vGyroBias[1], vGyroBias[2]);
    bGyroBiasKnown_ = true;
  }
  else
  {
    if (pnh_.hasParam("GYRO_BIAS"))
      LOG(WARNING) << "GYRO_BIAS must be a list of 3 numbers (rad/s, IMU frame); estimating the bias instead";
    gyroBiasEstimator_.reset(new tools::GyroBiasEstimator(tools::param(pnh_, "GYRO_BIAS_WINDOW", 2.0),
                                                           tools::param(pnh_, "GYRO_STILL_MAX_STD", 0.0035)));
  }
```
Inside the existing `if (bImuRotationPrediction_)` block, after the `LOG(INFO) << "IMU rotation prediction enabled, ...` line, add:
```cpp
    if (bGyroBiasKnown_)
      LOG(INFO) << "Gyro bias from GYRO_BIAS: " << gyroBias_.transpose() << " rad/s";
    else
      LOG(INFO) << "Gyro bias: estimating from the first " << gyroBiasEstimator_->window()
                << " s still window (hold the rig still); prediction is uncorrected until then";
```

- [ ] **Step 3: Feed the estimator in the gyro callback**

In `imuPredictionCallback`, after
```cpp
  gyroBuf_.push_back({t, Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)});
```
add
```cpp
  if (gyroBiasEstimator_ && !bGyroBiasKnown_ && gyroBiasEstimator_->add(gyroBuf_.back()))
  {
    gyroBias_ = gyroBiasEstimator_->bias();
    bGyroBiasKnown_ = true;
    LOG(INFO) << "Gyro bias estimated from " << gyroBiasEstimator_->window() << " s still window: "
              << gyroBias_.transpose() << " rad/s (std " << gyroBiasEstimator_->biasStd().transpose() << ")";
  }
```

- [ ] **Step 4: Subtract the bias at integration time**

Replace the head of `predictRotationWithGyro` up to and including the `if (tools::gyroDeltaRotation(...))` line:
```cpp
void esvo2_Tracking::predictRotationWithGyro(double t_prev_frame, double t_cur_frame)
{
  std::vector<tools::GyroSample> samples;
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    samples.assign(gyroBuf_.begin(), gyroBuf_.end());
  }
  Eigen::Matrix3d R_imu;
  if (tools::gyroDeltaRotation(samples, t_prev_frame, t_cur_frame, imuTimeOffset_, R_imu))
```
with
```cpp
bool esvo2_Tracking::predictRotationWithGyro(double t_prev_frame, double t_cur_frame, bool &biasCorrected)
{
  std::vector<tools::GyroSample> samples;
  Eigen::Vector3d bias;
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    samples.assign(gyroBuf_.begin(), gyroBuf_.end());
    biasCorrected = bGyroBiasKnown_;
    bias = gyroBias_;
  }
  // Subtracted here rather than on arrival so samples buffered before the estimate completed are corrected too.
  if (biasCorrected)
    tools::subtractBias(samples, bias);
  Eigen::Matrix3d R_imu;
  const bool applied = tools::gyroDeltaRotation(samples, t_prev_frame, t_cur_frame, imuTimeOffset_, R_imu);
  if (applied)
```
and add `return applied;` as the last statement of the function (after the periodic log block).

- [ ] **Step 5: Update the call site**

In `curDataTransferring()`, replace
```cpp
      predictRotationWithGyro(t_prev_frame, cur_.t_.toSec());
```
with
```cpp
    {
      bool biasCorrected = false;
      predictRotationWithGyro(t_prev_frame, cur_.t_.toSec(), biasCorrected);
    }
```
(The lock decision that uses both results is added in Task 4.)

- [ ] **Step 6: Build and check G4 (startup estimate) on the bag's still period**

```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E " error|Built target esvo2_Tracking"
source /root/catkin_ws/devel/setup.bash
O=/root/datasets/evk4/replays
PLAYARGS="-s 10.5" PLAYRATE=0.25 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/replay_eval.sh $O/g4_estimate.bag /root/datasets/evk4/slide_lr.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  $(rospack find esvo2_core)/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false
grep -E "Gyro bias" $O/g4_estimate.bag.log
```
Expected: `Gyro bias: estimating from the first 2 s still window ...`, then `Gyro bias estimated from 2 s still window: <bx> <by> <bz> rad/s`, each within 0.001 of `0.003425 -0.004210 -0.003853` (G4).

- [ ] **Step 7: Check G2 still holds (no lock yet, keys unset)**

Re-run the Task 1 Step 8 baseline commands with output `$O/task3_g2.bag`. Expected: `min_x` between −0.15 and −0.30, and the log shows the bias estimated only after the slide (≥ 10 s of bag time).

- [ ] **Step 8: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/esvo2_Tracking.h esvo2_core/src/esvo2_Tracking.cpp
git commit -m "Tracking: remove gyro bias (GYRO_BIAS or first still window) from the rotation prediction

G4: estimated bias <values> vs configured 0.003425,-0.004210,-0.003853

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Rotation lock (translation-only solve)

**Files:**
- Modify: `esvo2_core/include/esvo2_core/core/RegProblemLM.h` (public section near `void setPose();`, members near `bool bPrint_;`)
- Modify: `esvo2_core/src/core/RegProblemLM.cpp` (end of `df()`, just before its `return 0;` ~line 274)
- Modify: `esvo2_core/include/esvo2_core/core/RegProblemSolverLM.h`, `esvo2_core/src/core/RegProblemSolverLM.cpp`
- Modify: `esvo2_core/include/esvo2_core/esvo2_Tracking.h`, `esvo2_core/src/esvo2_Tracking.cpp`

**Interfaces:**
- Consumes: `predictRotationWithGyro(double, double, bool &)` (Task 3).
- Produces: `void RegProblemLM::setFixRotation(bool fix)`, `void RegProblemSolverLM::setFixRotation(bool fix)`; tracking members `bImuRotationLock_`, `bLockThisFrame_`, `nLocked_`.

- [ ] **Step 1: Registration problem flag**

In `RegProblemLM.h`, after `void setPose();` add:
```cpp
  // When set, df() zeroes the rotation (Cayley) columns: LM solves translation only and
  // the rotation stays at its initial value (e.g. the gyro prediction).
  void setFixRotation(bool fix) { bFixRotation_ = fix; }
```
and after `bool bPrint_;` add:
```cpp
  bool bFixRotation_ = false;
```
In `RegProblemLM.cpp`, at the end of `RegProblemLM::df`, replace the final
```cpp
  // LOG(INFO) << "fjac:\n" << fjac;
  // LOG(INFO) << "Jacobian Computation takes " << tt.toc() << " ms.";
  return 0;
}
```
with
```cpp
  // LOG(INFO) << "fjac:\n" << fjac;
  // LOG(INFO) << "Jacobian Computation takes " << tt.toc() << " ms.";
  // Rotation locked: no rotation update. Eigen's LM rescales zero-norm columns to 1, so the
  // system stays well-posed.
  if (bFixRotation_)
    fjac.leftCols(3).setZero();
  return 0;
}
```

- [ ] **Step 2: Solver pass-through**

In `RegProblemSolverLM.h`, after `bool solve_analytical();// faster` add:
```cpp
  // Translation-only solve for the next solve_analytical() (analytical problem only).
  void setFixRotation(bool fix);
```
In `RegProblemSolverLM.cpp`, before `void RegProblemSolverLM::setRegPublisher(` add:
```cpp
void RegProblemSolverLM::setFixRotation(bool fix)
{
  if (regProblemPtr_)
    regProblemPtr_->setFixRotation(fix);
}

```

- [ ] **Step 3: Tracking: config and members**

In `esvo2_Tracking.h`, after the gyro bias members from Task 3 add:
```cpp
    // IMU_ROTATION_LOCK: translation-only solve on frames with a bias-corrected gyro prediction
    bool bImuRotationLock_ = false;
    bool bLockThisFrame_ = false;
    size_t nLocked_ = 0; // frames locked since the last periodic prediction log
```
In the constructor, after the Task 3 `GYRO_BIAS` block, add:
```cpp
  bImuRotationLock_ = tools::param(pnh_, "IMU_ROTATION_LOCK", false);
  if (bImuRotationLock_ && (!bImuRotationPrediction_ || bUseImu_))
  {
    LOG(WARNING) << "IMU_ROTATION_LOCK needs IMU_ROTATION_PREDICTION: True and USE_IMU: False; lock disabled";
    bImuRotationLock_ = false;
  }
  if (bImuRotationLock_ && rpType_ != REG_ANALYTICAL)
  {
    LOG(WARNING) << "IMU_ROTATION_LOCK only works with RegProblemType: 1 (analytical); lock disabled";
    bImuRotationLock_ = false;
  }
  if (bImuRotationLock_)
    LOG(INFO) << "IMU rotation lock enabled: rotation from the gyro, translation-only registration";
```

- [ ] **Step 4: Tracking: per-frame decision**

In `curDataTransferring()`, right after `const double t_prev_frame = cur_.t_.toSec();` add:
```cpp
  bLockThisFrame_ = false;
```
Replace the Task 3 call-site block
```cpp
    {
      bool biasCorrected = false;
      predictRotationWithGyro(t_prev_frame, cur_.t_.toSec(), biasCorrected);
    }
```
with
```cpp
    {
      bool biasCorrected = false;
      const bool predicted = predictRotationWithGyro(t_prev_frame, cur_.t_.toSec(), biasCorrected);
      bLockThisFrame_ = bImuRotationLock_ && predicted && biasCorrected;
      if (bLockThisFrame_)
        nLocked_++;
    }
```
(The enclosing `if` already requires `ESVO2_System_Status_ == "WORKING"`.)

In `TrackingLoop()`, inside `if(rpSolver_.resetRegProblem(&ref_, &cur_))`, immediately before `if(rpType_ == REG_NUMERICAL)` add:
```cpp
      rpSolver_.setFixRotation(bLockThisFrame_);
```

- [ ] **Step 5: Tracking: periodic log**

In `predictRotationWithGyro`, inside the 5 s log block, after the existing skip-rate warning and before `nPredOk_ = nPredSkip_ = 0;`, add:
```cpp
    if (bImuRotationLock_)
    {
      LOG(INFO) << "IMU rotation lock: " << nLocked_ << " of " << nTotal << " frames locked";
      if (nTotal > 0 && 5 * nLocked_ < 4 * nTotal)
        LOG(WARNING) << "IMU rotation lock applied to only " << nLocked_ << " of " << nTotal
                     << " frames in the last 5 s (bias not known yet, or gyro gaps)";
    }
    nLocked_ = 0;
```

- [ ] **Step 6: Build and check G1**

```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core 2>&1 | grep -E " error|Built target esvo2_Tracking"
source /root/catkin_ws/devel/setup.bash
O=/root/datasets/evk4/replays; C=$(rospack find esvo2_core)
cp $C/cfg/tracking/tracking_evk4_AA.yaml $O/tracking_g1.yaml
printf 'IMU_ROTATION_LOCK: True\nGYRO_BIAS: [0.003425, -0.004210, -0.003853]\n' >> $O/tracking_g1.yaml
PLAYRATE=0.25 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/replay_eval.sh $O/g1_lock.bag /root/datasets/evk4/slide_lr.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  $C/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false tracking_cfg:=$O/tracking_g1.yaml
grep -E "rotation lock|Gyro bias from" $O/g1_lock.bag.log | head -5
python3 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_slide.py $O/g1_lock.bag /root/datasets/evk4/slide_lr.bag $C/calib/evk4_stereo 0.003425,-0.004210,-0.003853
```
Expected: `IMU rotation lock enabled`, periodic `N of M frames locked` with N close to M; `min_x ≤ -0.50` and `max_dyaw ≤ 3.0` (G1). The temporary test gave −0.55 m. If G1 fails, stop and report; do not tune thresholds to pass.

- [ ] **Step 7: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/core/RegProblemLM.h esvo2_core/src/core/RegProblemLM.cpp esvo2_core/include/esvo2_core/core/RegProblemSolverLM.h esvo2_core/src/core/RegProblemSolverLM.cpp esvo2_core/include/esvo2_core/esvo2_Tracking.h esvo2_core/src/esvo2_Tracking.cpp
git commit -m "Tracking: IMU_ROTATION_LOCK, translation-only registration on bias-corrected gyro frames

G1 (slide bag, 0.25x): min_x=<value> max_dyaw=<value>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Enable for the rig, regression gates, docs

**Files:**
- Modify: `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`
- Modify: `EVK4_STEREO_SETUP.md`

- [ ] **Step 1: Enable the lock in the EVK4 tracking config**

Append to `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`:
```yaml
# Rotation from the bias-corrected gyro, translation-only registration (see
# docs/superpowers/specs/2026-09-18-gyro-locked-tracking-design.md). Without it,
# beside a flat wall the solver explains a sideways slide as yaw.
# Gyro bias: estimated from the first still window after start: hold the rig
# still for GYRO_BIAS_WINDOW seconds. Set GYRO_BIAS: [bx, by, bz] (rad/s, IMU
# frame) instead to skip estimation, e.g. for replays that do not start still.
IMU_ROTATION_LOCK: True
GYRO_BIAS_WINDOW: 2.0
GYRO_STILL_MAX_STD: 0.0035
```

- [ ] **Step 2: G2 with the new keys removed**

```bash
source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash
O=/root/datasets/evk4/replays; C=$(rospack find esvo2_core)
grep -v -E "^(IMU_ROTATION_LOCK|GYRO_BIAS|GYRO_BIAS_WINDOW|GYRO_STILL_MAX_STD):" $C/cfg/tracking/tracking_evk4_AA.yaml > $O/tracking_g2.yaml
PLAYRATE=0.25 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/replay_eval.sh $O/g2_default.bag /root/datasets/evk4/slide_lr.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  $C/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false tracking_cfg:=$O/tracking_g2.yaml
python3 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_slide.py $O/g2_default.bag /root/datasets/evk4/slide_lr.bag $C/calib/evk4_stereo 0.003425,-0.004210,-0.003853
```
Expected: `min_x` between −0.15 and −0.30 (G2).

- [ ] **Step 3: G3, MVSEC vision-only, two runs**

The MVSEC bag is at `/root/datasets/mvsec/indoor_flying1_data.bag` (if missing: `curl -L -o /root/datasets/mvsec/indoor_flying1_data.bag https://visiondata.cis.upenn.edu/mvsec/indoor_flying/indoor_flying1_data.bag`, 1.28 GB).
```bash
source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash
O=/root/datasets/mvsec/replays; mkdir -p $O; C=$(rospack find esvo2_core)
sed -E 's/^USE_IMU:.*/USE_IMU: False/' $C/cfg/mapping/mapping_upenn_AA.yaml > $O/mapping_noimu.yaml
sed -E 's/^USE_IMU:.*/USE_IMU: False/' $C/cfg/tracking/tracking_upenn_AA.yaml > $O/tracking_noimu.yaml
for r in 1 2; do
  /root/catkin_ws/src/ESVO2/esvo2_core/scripts/replay_eval.sh $O/g3_run$r.bag /root/datasets/mvsec/indoor_flying1_data.bag \
    "/davis/left/events /davis/right/events /davis/left/imu /davis/right/imu" \
    $C/launch/system/system_upenn.launch gui:=false mapping_cfg:=$O/mapping_noimu.yaml tracking_cfg:=$O/tracking_noimu.yaml
  python3 /root/catkin_ws/src/ESVO2/esvo2_core/scripts/eval_traj.py \
    /root/catkin_ws/src/ESVO2/results/gt/upenn/stamped_groundtruth_of_indoorflying1.txt $O/g3_run$r.bag run$r
done
```
Expected per run: `tracking resets: 0` and `ratio` ≥ 0.80 (G3; before this branch: 0.88–0.92).

- [ ] **Step 4: Document in `EVK4_STEREO_SETUP.md`**

Update the "Current status" paragraph (top of the file) to reflect: side swap fixed; tracking loses translation beside planar scenes because the solver trades it for yaw; gyro-locked rotation implemented (offline G1 result); remaining shortfall attributed to event-rate saturation. Add under "Gotchas discovered along the way":
```markdown
- **Tracking traded sideways translation for yaw.** Beside a flat wall at one
  depth (mostly vertical edges, 41° horizontal FOV), a lateral slide and a yaw
  give almost the same image motion. On a recorded 1 m slide the solver
  reported 10–24° of yaw the rig never made (bias-corrected gyro: −0.4°) and
  only 0.2–0.26 m of translation. Fixed by `IMU_ROTATION_LOCK` (rotation from
  the bias-corrected gyro, translation-only registration): <G1 result>.
- **The SBG gyro has ~0.004 rad/s (0.23 °/s) bias per axis,** which the
  round-1 prediction integrated as rotation. Tracking now removes it
  (`GYRO_BIAS`, or estimated from the first 2 s still window). **Hold the rig
  still for 2 s after starting**; the log prints `Gyro bias estimated from ...`.
- **The cameras saturate the 4 M ev/s rate cap during motion** (left sat at
  3.6–3.95 M ev/s through the whole slide; ~0.5 M ev/s noise floor at rest).
  Time surfaces come out faint and noisy. Not fixed yet: next rig session,
  test a higher cap and the sensor's noise filter.
- **ESVO2's IMU mode (`USE_IMU: True`) is broken on this machine, upstream
  included:** on MVSEC `indoor_flying1` mapping prints an accelerometer bias of
  ~6.9e-310 (uninitialized memory) and crashes (`std::length_error`). Vision-
  only mode tracks MVSEC (path ratio 0.88–0.92).
```
Add to "Changes made" → "This repo (`ESVO2`)": the new keys, `gyro_bias.h` + `test_gyro_bias`, the replay scripts, and the launch args. Add to "How to launch everything": replaying a bag with `scripts/replay_eval.sh` (the Task 4 Step 6 command as the example). Fill `<G1 result>` with the measured numbers.

- [ ] **Step 5: Commit**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/cfg/tracking/tracking_evk4_AA.yaml EVK4_STEREO_SETUP.md
git commit -m "EVK4: enable IMU_ROTATION_LOCK; document gyro bias, rotation lock and event-rate saturation

G2 (keys removed): min_x=<value>. G3 (MVSEC vision-only): ratio <run1>, <run2>, 0 resets.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```
