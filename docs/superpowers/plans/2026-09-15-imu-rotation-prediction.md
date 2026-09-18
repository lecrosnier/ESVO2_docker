# IMU Rotation Prediction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make ESVO2 tracking follow the live EVK4 rig's rotation by starting each frame from a gyro-predicted rotation.

**Architecture:** A Python relay re-stamps SBG IMU samples with the IMU's own clock (`/imu/data_synced`). A Python calibration script estimates the camera→IMU rotation `R_b_c` and time offset `t_d` by aligning event-based camera angular velocity with the gyro. A ROS-free C++ header integrates gyro samples between frames; `esvo2_Tracking` uses it to predict the initial rotation (translation stays vision-only).

**Tech Stack:** ROS Noetic, C++14 (catkin_simple, Eigen, gtest), Python 3 (rospy, numpy, OpenCV, PyYAML), SBG Ellipse2-N, Prophesee EVK4.

**Spec:** `docs/superpowers/specs/2026-09-15-imu-rotation-prediction-design.md`

## Global Constraints

- Scope: rotation prediction only; translation vision-only; `USE_IMU: False` in mapping and tracking stays.
- `R_b_c` convention: `p_imu = R_b_c · p_cam`, camera = **rectified** left camera.
- `t_d` convention: camera time = `/imu/data_synced` time + `t_d`.
- Predictor: zero-order hold, samples must reach within 20 ms of both interval ends, interval ≤ 0.2 s, else no prediction.
- Camera rotation from IMU rotation: `R_c = R_b_cᵀ · R_imu · R_b_c`; predicted pose `R_world_cur = R_world_prev · R_c`, `t_world_cur = t_world_prev`.
- New tracking params: `IMU_ROTATION_PREDICTION` (default False), `IMU_TIME_OFFSET` (s); new tracking topic name `imu_prediction` remapped to `/imu/data_synced`.
- Relay: in `/sbg/imu_data`, out `/imu/data_synced`, `frame_id: imu_link`, orientation covariance[0] = -1, offset = min over last ~2 s of (receive − device time).
- No new dependencies (no SciPy, PyTorch, Kalibr). No raw event recordings; keep scripts lightweight (an event bag loaded into memory OOM-killed the desktop once).
- Stop the pipeline before rebuilding its binaries; build with `-j4`.
- **Ask the user before every test that needs the rig moved.** Ask the user before every commit or push.
- Nothing is written into `left.yaml` / `right.yaml` until the user approves the calibration numbers.

## Paths used throughout

- Workspace: `/root/catkin_ws`; ESVO2 repo: `/root/catkin_ws/src/ESVO2`; package: `/root/catkin_ws/src/ESVO2/esvo2_core` (below: `esvo2_core/`).
- Source ROS before any ROS command: `source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash`.
- Scratch directory for run logs: `/tmp/claude-0/-root/81829425-7159-421a-8a0d-878de4cc0ff4/scratchpad` (below: `$S`).

## File structure

| File | Responsibility |
|---|---|
| `esvo2_core/scripts/imu_restamp_core.py` (create) | Pure re-stamping logic: unwrap, sliding-min offset, reset on backward jump |
| `esvo2_core/scripts/imu_restamp.py` (create) | ROS node wrapping the core: `/sbg/imu_data` → `/imu/data_synced` |
| `esvo2_core/test/test_imu_restamp_core.py` (create) | unittest for the re-stamping core |
| `esvo2_core/scripts/imu_cam_calib_core.py` (create) | Pure math: SO(3), Kabsch, RANSAC rotation, time-offset correlation, `R_b_c` fit, plausibility |
| `esvo2_core/scripts/calibrate_imu_camera_rotation.py` (create) | ROS capture: event frames, LK tracking, camera ω; runs the core fit, writes results YAML |
| `esvo2_core/test/test_imu_cam_calib_core.py` (create) | unittest for the calibration core on synthetic data |
| `esvo2_core/include/esvo2_core/tools/gyro_prediction.h` (create) | ROS-free `GyroSample`, `so3Exp`, `gyroDeltaRotation`, `imuToCameraRotation` |
| `esvo2_core/test/test_gyro_prediction.cpp` (create) | gtest for `gyro_prediction.h` |
| `esvo2_core/CMakeLists.txt` (modify) | Add the gtest target |
| `esvo2_core/include/esvo2_core/esvo2_Tracking.h` (modify) | Predictor members and methods |
| `esvo2_core/src/esvo2_Tracking.cpp` (modify) | Params, subscriber, callback, prediction in `curDataTransferring()` |
| `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml` (modify) | `IMU_ROTATION_PREDICTION`, `IMU_TIME_OFFSET` |
| `esvo2_core/launch/system/system_evk4_mapping.launch` (modify) | Remap `imu_prediction` for the tracking node |
| `esvo2_core/launch/system/evk4_live_all.launch` (modify) | Start the relay when `imu:=true` |
| `esvo2_core/calib/evk4_stereo/{left,right}.yaml` (modify, after approval) | Rotation part of `T_b_c` |
| `esvo2_core/scripts/validate_rotation_tracking.py` (create) | Validation: slow turn (net rotation at holds) and mixed motion (speed bins) |
| `esvo2_core/src/core/RegProblemSolverLM.cpp` (temporary edit, then revert) | Add dark-share metric to existing `[diag]`; removed in Task 8 |
| `EVK4_STEREO_SETUP.md` (modify) | Correct the 1 ms note; document relay, calibration, predictor, results |

---

### Task 0: Commit pending work and the design documents

**Files:**
- Commit only: `EVK4_STEREO_SETUP.md`, `evk4_drivers/sbg_ros_driver/config/sbg_device_evk4_rig.yaml`, `docs/superpowers/specs/2026-09-15-imu-rotation-prediction-design.md`, `docs/superpowers/plans/2026-09-15-imu-rotation-prediction.md`
- Do NOT commit: `esvo2_core/src/core/RegProblemSolverLM.cpp` (temporary `[diag]` instrumentation)

**Interfaces:** none.

- [ ] **Step 1: Show the pending state**

Run: `cd /root/catkin_ws/src/ESVO2 && git status --short && git branch --show-current`
Expected: modified `EVK4_STEREO_SETUP.md`, `esvo2_core/src/core/RegProblemSolverLM.cpp`, `evk4_drivers/sbg_ros_driver/config/sbg_device_evk4_rig.yaml`; untracked `docs/`; branch `main`.

- [ ] **Step 2: Ask the user** whether to commit the SBG rate docs/config and the spec+plan now, and on which branch (proposal: create `imu-rotation-prediction` from `main`). Stop until they answer.

- [ ] **Step 3: Commit (only after approval)**

```bash
cd /root/catkin_ws/src/ESVO2
git switch -c imu-rotation-prediction
git add EVK4_STEREO_SETUP.md evk4_drivers/sbg_ros_driver/config/sbg_device_evk4_rig.yaml
git commit -m "SBG IMU: 200 Hz IMU_DATA and EKF_QUAT outputs, document IMU findings

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
git add docs/superpowers/specs/2026-09-15-imu-rotation-prediction-design.md docs/superpowers/plans/2026-09-15-imu-rotation-prediction.md
git commit -m "Add IMU rotation prediction design and implementation plan

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
git status --short
```
Expected: only `esvo2_core/src/core/RegProblemSolverLM.cpp` remains modified.

---

### Task 1: IMU re-stamping relay

**Files:**
- Create: `esvo2_core/scripts/imu_restamp_core.py`
- Create: `esvo2_core/scripts/imu_restamp.py`
- Create: `esvo2_core/test/test_imu_restamp_core.py`
- Modify: `esvo2_core/launch/system/evk4_live_all.launch` (IMU include block, currently `<include if="$(arg imu)" file="$(find sbg_driver)/launch/sbg_evk4_rig.launch" />`)

**Interfaces:**
- Consumes: `/sbg/imu_data` (`sbg_driver/SbgImuData`: `uint32 time_stamp` µs, `geometry_msgs/Vector3 accel`, `gyro`, ENU).
- Produces: class `Restamper(window_s=2.0)` with `update(device_us: int, receive_s: float) -> Optional[float]`; topic `/imu/data_synced` (`sensor_msgs/Imu`, `frame_id: imu_link`).

- [ ] **Step 1: Write the failing test**

Create `esvo2_core/test/test_imu_restamp_core.py`:

```python
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from imu_restamp_core import Restamper  # noqa: E402


class RestamperTest(unittest.TestCase):
    def test_even_spacing_and_min_offset(self):
        r = Restamper(window_s=2.0)
        true_offset = 1000.0
        out = []
        for i in range(1000):
            dev_us = 5000 * i
            jitter = 0.0 if i % 7 == 0 else 0.004 + 0.011 * ((i * 37) % 10) / 10.0
            out.append(r.update(dev_us, dev_us * 1e-6 + true_offset + jitter))
        # after the first window, stamps are device time + min offset (jitter-free samples every 7th)
        tail = out[500:]
        gaps = [b - a for a, b in zip(tail[:-1], tail[1:])]
        self.assertTrue(all(abs(g - 0.005) < 1e-6 for g in gaps), gaps[:5])
        self.assertAlmostEqual(tail[0] - 500 * 0.005, true_offset, places=6)

    def test_uint32_wrap(self):
        r = Restamper(window_s=2.0)
        start = 2**32 - 5000 * 10
        stamps = []
        for i in range(20):
            dev_us = (start + 5000 * i) % 2**32
            stamps.append(r.update(dev_us, 50.0 + 0.005 * i))
        gaps = [b - a for a, b in zip(stamps[:-1], stamps[1:])]
        self.assertTrue(all(abs(g - 0.005) < 1e-6 for g in gaps), gaps)

    def test_backward_jump_resets_and_drops(self):
        r = Restamper(window_s=2.0)
        for i in range(100):
            self.assertIsNotNone(r.update(5000 * i, 10.0 + 0.005 * i))
        # device clock restarts (not a wrap: jump back by ~0.5 s, far from 2^32)
        self.assertIsNone(r.update(5000 * 0, 10.5))
        self.assertIsNotNone(r.update(5000 * 1, 10.505))

    def test_output_strictly_increasing(self):
        r = Restamper(window_s=0.05)
        prev = None
        for i in range(400):
            dev_us = 5000 * i
            rx = dev_us * 1e-6 + 3.0 + (0.02 if (i // 20) % 2 else 0.0)
            t = r.update(dev_us, rx)
            if prev is not None:
                self.assertGreater(t, prev)
            prev = t


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /root/catkin_ws/src/ESVO2/esvo2_core && python3 -m unittest test/test_imu_restamp_core.py -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'imu_restamp_core'`.

- [ ] **Step 3: Write the core**

Create `esvo2_core/scripts/imu_restamp_core.py`:

```python
"""Re-stamp IMU samples with the IMU's own clock, anchored to ROS time.

offset = min over the last window of (receive_time - device_time). Serial
delay and batching only ever add delay, so the minimum is the best estimate;
the window follows slow clock drift.
"""
from collections import deque

WRAP_US = 2 ** 32
BACKWARD_JUMP_S = 1.0  # a device time step further back than this (and not a wrap) is a reset


class Restamper:
    def __init__(self, window_s=2.0):
        self.window_s = window_s
        self._reset()

    def _reset(self):
        self._last_raw_us = None
        self._wraps = 0
        self._last_dev_s = None
        self._win = deque()  # monotonic deque of (device_s, offset), increasing offsets
        self._last_out = None

    def update(self, device_us, receive_s):
        """Return the published stamp in seconds, or None if the sample is dropped."""
        if self._last_raw_us is not None and device_us < self._last_raw_us:
            if self._last_raw_us - device_us > WRAP_US // 2:
                self._wraps += 1
            else:
                self._reset()
                return None
        self._last_raw_us = device_us
        dev_s = (self._wraps * WRAP_US + device_us) * 1e-6
        if self._last_dev_s is not None and dev_s < self._last_dev_s - BACKWARD_JUMP_S:
            self._reset()
            return None
        self._last_dev_s = dev_s

        off = receive_s - dev_s
        while self._win and self._win[-1][1] >= off:
            self._win.pop()
        self._win.append((dev_s, off))
        while self._win[0][0] < dev_s - self.window_s:
            self._win.popleft()

        out = dev_s + self._win[0][1]
        if self._last_out is not None and out <= self._last_out:
            out = self._last_out + 1e-6
        self._last_out = out
        return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /root/catkin_ws/src/ESVO2/esvo2_core && python3 -m unittest test/test_imu_restamp_core.py -v`
Expected: 4 tests, `OK`.

- [ ] **Step 5: Write the ROS node**

Create `esvo2_core/scripts/imu_restamp.py`, then `chmod +x esvo2_core/scripts/imu_restamp.py`:

```python
#!/usr/bin/env python3
"""/sbg/imu_data (IMU clock) -> /imu/data_synced (even 5 ms stamps on ROS time)."""
import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

from imu_restamp_core import Restamper


def main():
    rospy.init_node('imu_restamp')
    window_s = rospy.get_param('~window_s', 2.0)
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    restamper = Restamper(window_s=window_s)
    pub = rospy.Publisher('/imu/data_synced', Imu, queue_size=400)
    last_rx = [rospy.get_time()]

    def cb(msg):
        now = rospy.get_time()
        last_rx[0] = now
        stamp = restamper.update(msg.time_stamp, now)
        if stamp is None:
            rospy.logwarn('imu_restamp: IMU clock jumped backwards, offset estimate reset, sample dropped')
            return
        out = Imu()
        out.header.stamp = rospy.Time.from_sec(stamp)
        out.header.frame_id = frame_id
        out.orientation_covariance[0] = -1.0
        out.angular_velocity = msg.gyro
        out.linear_acceleration = msg.accel
        pub.publish(out)

    rospy.Subscriber('/sbg/imu_data', SbgImuData, cb, queue_size=400, tcp_nodelay=True)

    def watchdog(_):
        if rospy.get_time() - last_rx[0] > 0.5:
            rospy.logwarn_throttle(5.0, 'imu_restamp: no /sbg/imu_data for more than 0.5 s')

    rospy.Timer(rospy.Duration(0.25), watchdog)
    rospy.spin()


if __name__ == '__main__':
    main()
```

- [ ] **Step 6: Start the relay with the IMU driver in the launch file**

In `esvo2_core/launch/system/evk4_live_all.launch`, replace
```xml
  <include if="$(arg imu)" file="$(find sbg_driver)/launch/sbg_evk4_rig.launch" />
```
with
```xml
  <group if="$(arg imu)">
    <include file="$(find sbg_driver)/launch/sbg_evk4_rig.launch" />
    <!-- Re-stamps /sbg/imu_data with the IMU clock: publishes /imu/data_synced -->
    <node pkg="esvo2_core" type="imu_restamp.py" name="imu_restamp" output="screen" />
  </group>
```

Run: `source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch --nodes esvo2_core evk4_live_all.launch gui:=false && roslaunch --nodes esvo2_core evk4_live_all.launch gui:=false imu:=false`
Expected: first list contains `/imu_restamp` and `/sbg_device`; second contains neither.

- [ ] **Step 7: Live check (no rig motion needed)**

Make sure no pipeline is running (`ps -eo stat,args | awk '$1 !~ /^Z/' | grep -cE 'sbg_device|esvo2_|prophesee_ros_stereo'` → 0). Write `$S/check_restamp.py`:

```python
import time, numpy as np, rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData
syn, raw = [], []
rospy.init_node('check_restamp', anonymous=True)
rospy.Subscriber('/imu/data_synced', Imu, lambda m: syn.append((time.time(), m.header.stamp.to_sec())), queue_size=5000)
rospy.Subscriber('/sbg/imu_data', SbgImuData, lambda m: raw.append(1), queue_size=5000)
time.sleep(1.0); syn.clear(); raw.clear()
T = 180.0; time.sleep(T)
a = np.array(syn); g = np.diff(a[:, 1]) * 1000; d = a[:, 0] - a[:, 1]
print("synced=%d raw=%d  gap ms: min %.3f median %.3f max %.3f  |gap-5|>0.1: %d" % (len(a), len(raw), g.min(), np.median(g), g.max(), int((np.abs(g - 5) > 0.1).sum())))
thirds = np.array_split(d, 3)
print("receive-minus-stamp mean per third (ms): %s  min overall %.2f ms" % ([round(1000 * x.mean(), 2) for x in thirds], 1000 * d.min()))
```

Run:
```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch sbg_driver sbg_evk4_rig.launch > $S/relay_sbg.log 2>&1 & P1=$!
rosrun esvo2_core imu_restamp.py > $S/relay_node.log 2>&1 & P2=$!
sleep 8 && timeout 220 python3 $S/check_restamp.py; kill -INT $P2 $P1; wait
```
Expected: `synced` equals `raw` within ±2 (subscriber start/stop edges); median gap 5.000 ms; `|gap-5|>0.1` count 0 or only a handful around offset updates; mean receive-minus-stamp ≥ 0 and the three thirds within ~1 ms of each other.

- [ ] **Step 8: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/scripts/imu_restamp_core.py esvo2_core/scripts/imu_restamp.py esvo2_core/test/test_imu_restamp_core.py esvo2_core/launch/system/evk4_live_all.launch
git commit -m "Add IMU re-stamping relay: /sbg/imu_data -> /imu/data_synced

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Calibration math core

**Files:**
- Create: `esvo2_core/scripts/imu_cam_calib_core.py`
- Create: `esvo2_core/test/test_imu_cam_calib_core.py`

**Interfaces:**
- Produces (all numpy, float64):
  - `so3_exp(v) -> R (3x3)`, `so3_log(R) -> v (3,)`
  - `kabsch(A, B, w=None) -> R` minimising Σ wᵢ‖Aᵢ − R·Bᵢ‖² (A, B: Nx3), i.e. `A ≈ R B`
  - `fit_rotation_ransac(f_prev, f_next, thresh_rad=0.005, iters=100, rng=None) -> (R, inliers bool N)` with `f_prev ≈ R · f_next` (unit bearings Nx3); returns `(None, None)` if < 3 points
  - `estimate_time_offset(t_cam, s_cam, t_imu, s_imu, max_offset=0.1, step=0.001) -> (t_d, peak_corr, sharpness)` where camera time = imu time + t_d
  - `fit_R_b_c(w_imu, w_cam, iters=3) -> (R, inliers bool N, rms)` with `w_imu ≈ R · w_cam`
  - `signed_permutation_distance_deg(R) -> float`

- [ ] **Step 1: Write the failing test**

Create `esvo2_core/test/test_imu_cam_calib_core.py`:

```python
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import imu_cam_calib_core as c  # noqa: E402


def angle_deg(R):
    return np.degrees(np.linalg.norm(c.so3_log(R)))


class CalibCoreTest(unittest.TestCase):
    def test_exp_log_roundtrip(self):
        v = np.array([0.3, -0.2, 0.1])
        np.testing.assert_allclose(c.so3_log(c.so3_exp(v)), v, atol=1e-12)
        np.testing.assert_allclose(c.so3_exp(np.zeros(3)), np.eye(3), atol=1e-12)

    def test_kabsch_recovers_rotation(self):
        rng = np.random.default_rng(0)
        R = c.so3_exp(np.array([0.5, 0.1, -0.7]))
        B = rng.normal(size=(50, 3))
        A = B @ R.T
        self.assertLess(angle_deg(c.kabsch(A, B).T @ R), 1e-9)

    def test_ransac_rotation_sign_convention_and_outliers(self):
        rng = np.random.default_rng(1)
        R_c = c.so3_exp(np.array([0.0, 0.02, 0.0]))  # camera k+1 orientation in camera k frame
        p = rng.normal(size=(200, 3)); p[:, 2] = np.abs(p[:, 2]) + 3.0
        f_prev = p / np.linalg.norm(p, axis=1, keepdims=True)
        q = p @ R_c              # R_c^T p (row-vector form)
        f_next = q / np.linalg.norm(q, axis=1, keepdims=True)
        f_next[:40] = rng.normal(size=(40, 3)); f_next[:40] /= np.linalg.norm(f_next[:40], axis=1, keepdims=True)
        R, inl = c.fit_rotation_ransac(f_prev, f_next, thresh_rad=0.002, iters=200, rng=np.random.default_rng(2))
        self.assertLess(angle_deg(R.T @ R_c), 0.01)
        self.assertGreaterEqual(inl.sum(), 160)
        self.assertFalse(inl[:40].any())

    def test_time_offset(self):
        t = np.arange(0.0, 20.0, 0.005)
        s = np.abs(np.sin(1.3 * t)) + 0.5 * np.abs(np.sin(3.1 * t + 0.4))
        t_d = 0.023
        t_cam = np.arange(0.2, 19.8, 0.01)
        s_cam = np.interp(t_cam - t_d, t, s)   # camera at time tc sees what IMU logged at tc - t_d
        est, corr, sharp = c.estimate_time_offset(t_cam, s_cam, t, s, max_offset=0.1, step=0.001)
        self.assertAlmostEqual(est, t_d, delta=0.0015)
        self.assertGreater(corr, 0.99)
        self.assertGreater(sharp, 0.0)

    def test_fit_R_b_c_with_outliers(self):
        rng = np.random.default_rng(3)
        R_b_c = c.so3_exp(np.array([1.2, -0.4, 0.3]))
        w_cam = rng.normal(size=(500, 3))
        w_imu = w_cam @ R_b_c.T + rng.normal(scale=0.01, size=(500, 3))
        w_imu[:50] = rng.normal(scale=3.0, size=(50, 3))
        R, inl, rms = c.fit_R_b_c(w_imu, w_cam)
        self.assertLess(angle_deg(R.T @ R_b_c), 0.2)
        self.assertLess(rms, 0.05)
        self.assertLess(inl[:50].mean(), 0.2)

    def test_signed_permutation_distance(self):
        P = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
        self.assertAlmostEqual(c.signed_permutation_distance_deg(P), 0.0, places=6)
        tilted = P @ c.so3_exp(np.array([np.radians(3.0), 0, 0]))
        self.assertAlmostEqual(c.signed_permutation_distance_deg(tilted), 3.0, delta=0.05)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /root/catkin_ws/src/ESVO2/esvo2_core && python3 -m unittest test/test_imu_cam_calib_core.py -v`
Expected: ERROR `ModuleNotFoundError: No module named 'imu_cam_calib_core'`.

- [ ] **Step 3: Write the core**

Create `esvo2_core/scripts/imu_cam_calib_core.py`:

```python
"""Pure math for the camera-IMU rotation / time-offset calibration.

Conventions (see the design spec):
  p_imu = R_b_c * p_cam            (camera = rectified left camera)
  w_imu = R_b_c * w_cam
  camera time = IMU time + t_d
  f_prev = R * f_next              (R = orientation of the later camera frame in the earlier one)
"""
import itertools

import numpy as np


def _hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def so3_exp(v):
    v = np.asarray(v, dtype=np.float64)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3) + _hat(v)
    K = _hat(v / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def so3_log(R):
    cos_th = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(cos_th)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * w
    if np.pi - th < 1e-6:
        B = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(B), 0.0, None))
        k = int(np.argmax(axis))
        axis = B[:, k] / np.sqrt(B[k, k])
        return th * axis
    return th / (2.0 * np.sin(th)) * w


def kabsch(A, B, w=None):
    """R minimising sum w_i |A_i - R B_i|^2."""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    w = np.ones(len(A)) if w is None else np.asarray(w, dtype=np.float64)
    H = (B * w[:, None]).T @ A
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def fit_rotation_ransac(f_prev, f_next, thresh_rad=0.005, iters=100, rng=None):
    n = len(f_prev)
    if n < 3:
        return None, None
    rng = np.random.default_rng() if rng is None else rng
    cos_thr = np.cos(thresh_rad)
    best = None
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        R = kabsch(f_prev[idx], f_next[idx])
        inl = np.sum(f_prev * (f_next @ R.T), axis=1) >= cos_thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best.sum() < 3:
        return None, None
    R = kabsch(f_prev[best], f_next[best])
    inl = np.sum(f_prev * (f_next @ R.T), axis=1) >= cos_thr
    return R, inl


def estimate_time_offset(t_cam, s_cam, t_imu, s_imu, max_offset=0.1, step=0.001):
    """t_d maximising corr(s_cam(t), s_imu(t - t_d)); returns (t_d, peak_corr, sharpness)."""
    t_cam = np.asarray(t_cam, dtype=np.float64)
    s_cam = np.asarray(s_cam, dtype=np.float64)
    offsets = np.arange(-max_offset, max_offset + step / 2, step)
    lo, hi = t_imu[0] + max_offset, t_imu[-1] - max_offset
    m = (t_cam >= lo) & (t_cam <= hi)
    tc, sc = t_cam[m], s_cam[m]
    sc = (sc - sc.mean()) / (sc.std() + 1e-12)
    corrs = []
    for d in offsets:
        si = np.interp(tc - d, t_imu, s_imu)
        si = (si - si.mean()) / (si.std() + 1e-12)
        corrs.append(float(np.mean(sc * si)))
    corrs = np.array(corrs)
    k = int(np.argmax(corrs))
    far = np.abs(offsets - offsets[k]) > 0.05  # hand motion is smooth over ~100 ms; compare against clearly different offsets
    sharpness = float(corrs[k] - (corrs[far].max() if far.any() else corrs[k]))
    return float(offsets[k]), float(corrs[k]), sharpness


def fit_R_b_c(w_imu, w_cam, iters=3):
    w_imu = np.asarray(w_imu, dtype=np.float64)
    w_cam = np.asarray(w_cam, dtype=np.float64)
    inl = np.ones(len(w_imu), dtype=bool)
    R = kabsch(w_imu, w_cam)
    for _ in range(iters):
        r = np.linalg.norm(w_imu - w_cam @ R.T, axis=1)
        med = np.median(r[inl])
        inl = r <= 3.0 * max(med, 1e-9)
        R = kabsch(w_imu[inl], w_cam[inl])
    r = np.linalg.norm(w_imu - w_cam @ R.T, axis=1)
    rms = float(np.sqrt(np.mean(r[inl] ** 2)))
    return R, inl, rms


def signed_permutation_distance_deg(R):
    best = 180.0
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            P = np.zeros((3, 3))
            for row, (col, sg) in enumerate(zip(perm, signs)):
                P[row, col] = sg
            if np.linalg.det(P) < 0:
                continue
            best = min(best, float(np.degrees(np.linalg.norm(so3_log(P.T @ R)))))
    return best
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /root/catkin_ws/src/ESVO2/esvo2_core && python3 -m unittest test/test_imu_cam_calib_core.py -v`
Expected: 6 tests, `OK`.

- [ ] **Step 5: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/scripts/imu_cam_calib_core.py esvo2_core/test/test_imu_cam_calib_core.py
git commit -m "Add camera-IMU rotation calibration math with unit tests

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Calibration capture script

**Files:**
- Create: `esvo2_core/scripts/calibrate_imu_camera_rotation.py`

**Interfaces:**
- Consumes: `imu_cam_calib_core` (Task 2) functions `fit_rotation_ransac`, `so3_log`, `estimate_time_offset`, `fit_R_b_c`, `signed_permutation_distance_deg`; `/imu/data_synced` (Task 1); `/evk4_left/events` (`dvs_msgs/EventArray`: `uint16 x, uint16 y, time ts, bool polarity`, 13 bytes per event); `esvo2_core/calib/evk4_stereo/left.yaml` keys `camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`, `image_width`, `image_height` (each matrix as `data` list).
- Produces: CLI `rosrun esvo2_core calibrate_imu_camera_rotation.py _duration:=60 _out:=<yaml>`; results YAML keys `R_b_c` (9 floats, row-major), `t_d` (s), `quality` (dict).

- [ ] **Step 1: Write the script**

Create `esvo2_core/scripts/calibrate_imu_camera_rotation.py`, then `chmod +x` it:

```python
#!/usr/bin/env python3
"""Calibrate R_b_c (rectified left camera -> IMU) and t_d (camera time = IMU time + t_d).

Run with only the camera driver and imu_restamp running. Rotate the rig by
hand about all axes in front of a distant (>= 3 m) textured scene.
Params: ~duration (s, default 60), ~window (s, default 0.01),
        ~calib (left.yaml path), ~out (results YAML path).
"""
import os
import queue
import struct
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Imu

import imu_cam_calib_core as core

EVT = np.dtype([('x', '<u2'), ('y', '<u2'), ('s', '<u4'), ('ns', '<u4'), ('p', 'u1')])
SCALE = 2  # count images at half resolution


def load_calib(path):
    c = yaml.safe_load(open(path))
    M = lambda k, r, cols: np.array(c[k]['data'], dtype=np.float64).reshape(r, cols)
    return (M('camera_matrix', 3, 3), np.array(c['distortion_coefficients']['data'], dtype=np.float64),
            M('rectification_matrix', 3, 3), M('projection_matrix', 3, 4), c['image_width'], c['image_height'])


class EventFramer:
    """Accumulates events into count images of fixed length by event time."""

    def __init__(self, width, height, window, out_q):
        self.w, self.h, self.window, self.q = width // SCALE, height // SCALE, window, out_q
        self.t0 = None
        self.idx = []
        self.dropped = 0

    def add(self, raw):
        L = struct.unpack_from('<I', raw, 12)[0]
        n = struct.unpack_from('<I', raw, 24 + L)[0]
        if n == 0:
            return
        e = np.frombuffer(raw, dtype=EVT, count=n, offset=28 + L)
        ts = e['s'].astype(np.float64) + e['ns'] * 1e-9
        if self.t0 is None:
            self.t0 = ts[0]
        win_end = self.t0 + self.window
        if ts[-1] >= win_end:
            before = ts < win_end
            if before.any():
                self.idx.append((e['y'][before] // SCALE).astype(np.int64) * self.w + e['x'][before] // SCALE)
            self._flush(win_end)
            self.t0 = win_end if ts[-1] < win_end + self.window else ts[-1]
            rest = ~before
            self.idx.append((e['y'][rest] // SCALE).astype(np.int64) * self.w + e['x'][rest] // SCALE)
        else:
            self.idx.append((e['y'] // SCALE).astype(np.int64) * self.w + e['x'] // SCALE)

    def _flush(self, t_end):
        if not self.idx:
            return
        counts = np.bincount(np.concatenate(self.idx), minlength=self.w * self.h)[: self.w * self.h]
        self.idx = []
        img = np.clip(counts.reshape(self.h, self.w) * 60, 0, 255).astype(np.uint8)
        try:
            self.q.put_nowait((t_end - self.window / 2.0, img))
        except queue.Full:
            self.dropped += 1


def camera_rates(frames_q, stop, calib, window, results):
    K, D, R_rect, P, _, _ = calib
    prev = None
    rng = np.random.default_rng(0)
    while not (stop.is_set() and frames_q.empty()):
        try:
            t, img = frames_q.get(timeout=0.2)
        except queue.Empty:
            continue
        img = cv2.GaussianBlur(img, (5, 5), 0)
        if prev is not None:
            t0, img0 = prev
            pts0 = cv2.goodFeaturesToTrack(img0, maxCorners=300, qualityLevel=0.01, minDistance=8)
            if pts0 is not None and len(pts0) >= 20:
                pts1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img, pts0, None, winSize=(21, 21), maxLevel=3)
                back, st2, _ = cv2.calcOpticalFlowPyrLK(img, img0, pts1, None, winSize=(21, 21), maxLevel=3)
                good = (st[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm((back - pts0)[:, 0], axis=1) < 1.0)
                if good.sum() >= 20:
                    full0 = (pts0[good] * SCALE + 0.5).astype(np.float64)
                    full1 = (pts1[good] * SCALE + 0.5).astype(np.float64)
                    u0 = cv2.undistortPoints(full0, K, D, R=R_rect).reshape(-1, 2)
                    u1 = cv2.undistortPoints(full1, K, D, R=R_rect).reshape(-1, 2)
                    f0 = np.hstack([u0, np.ones((len(u0), 1))]); f0 /= np.linalg.norm(f0, axis=1, keepdims=True)
                    f1 = np.hstack([u1, np.ones((len(u1), 1))]); f1 /= np.linalg.norm(f1, axis=1, keepdims=True)
                    R, inl = core.fit_rotation_ransac(f0, f1, thresh_rad=0.003, iters=100, rng=rng)
                    if R is not None and inl.mean() >= 0.5 and inl.sum() >= 20:
                        results.append(((t0 + t) / 2.0, core.so3_log(R) / (t - t0), float(inl.mean())))
        prev = (t, img)


def solve(cam, imu, label):
    t_cam = np.array([c[0] for c in cam]); w_cam = np.array([c[1] for c in cam])
    t_imu = np.array([i[0] for i in imu]); w_imu = np.array([i[1] for i in imu])
    t_d, corr, sharp = core.estimate_time_offset(t_cam, np.linalg.norm(w_cam, axis=1), t_imu, np.linalg.norm(w_imu, axis=1))
    w_imu_at_cam = np.stack([np.interp(t_cam - t_d, t_imu, w_imu[:, k]) for k in range(3)], axis=1)
    moving = np.linalg.norm(w_imu_at_cam, axis=1) > 0.2
    R, inl, rms = core.fit_R_b_c(w_imu_at_cam[moving], w_cam[moving])
    print("[%s] t_d=%.4f s (corr %.3f, sharpness %.3f) | pairs %d moving, inliers %.0f%%, rms %.3f rad/s" % (
        label, t_d, corr, sharp, int(moving.sum()), 100.0 * inl.mean(), rms))
    return R, t_d, dict(corr=corr, sharpness=sharp, pairs=int(moving.sum()), inlier_share=float(inl.mean()), rms=rms)


def main():
    rospy.init_node('calibrate_imu_camera_rotation', anonymous=True)
    duration = rospy.get_param('~duration', 60.0)
    window = rospy.get_param('~window', 0.01)
    calib_path = rospy.get_param('~calib', os.path.join(os.path.dirname(__file__), '..', 'calib', 'evk4_stereo', 'left.yaml'))
    out_path = rospy.get_param('~out', '/tmp/imu_cam_rotation_result.yaml')
    calib = load_calib(calib_path)

    frames_q = queue.Queue(maxsize=200)
    framer = EventFramer(calib[4], calib[5], window, frames_q)
    imu, cam = [], []
    capturing = threading.Event()
    stop = threading.Event()

    rospy.Subscriber('/evk4_left/events', rospy.AnyMsg, lambda m: framer.add(m._buff) if capturing.is_set() else None,
                     queue_size=100000, buff_size=2 ** 24, tcp_nodelay=True)
    rospy.Subscriber('/imu/data_synced', Imu, lambda m: imu.append((m.header.stamp.to_sec(), np.array(
        [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]))) if capturing.is_set() else None,
        queue_size=20000)
    worker = threading.Thread(target=camera_rates, args=(frames_q, stop, calib, window, cam), daemon=True)
    worker.start()

    time.sleep(1.0)
    print(">>> capturing for %.0f s: rotate the rig about all axes, little translation" % duration)
    capturing.set()
    time.sleep(duration)
    capturing.clear()
    stop.set()
    worker.join()
    print("camera rate samples %d, IMU samples %d, frames dropped (queue full) %d" % (len(cam), len(imu), framer.dropped))
    if len(cam) < 200 or len(imu) < 1000:
        print("not enough data; repeat the capture with more texture / motion")
        return

    R, t_d, q = solve(cam, imu, 'all')
    half = cam[len(cam) // 2][0]
    R1, t1, _ = solve([c for c in cam if c[0] < half], imu, 'first half')
    R2, t2, _ = solve([c for c in cam if c[0] >= half], imu, 'second half')
    rep_deg = float(np.degrees(np.linalg.norm(core.so3_log(R1.T @ R2))))
    perm_deg = core.signed_permutation_distance_deg(R)
    print("R_b_c =\n%s" % np.array2string(R, precision=4, suppress_small=True))
    print("repeatability: halves differ by %.2f deg and %.1f ms | distance to nearest axis permutation %.2f deg" % (
        rep_deg, 1000.0 * abs(t1 - t2), perm_deg))
    q.update(half_rotation_diff_deg=rep_deg, half_t_d_diff_ms=1000.0 * abs(t1 - t2), permutation_distance_deg=perm_deg)
    with open(out_path, 'w') as fh:
        yaml.safe_dump({'R_b_c': [float(x) for x in R.reshape(-1)], 't_d': float(t_d), 'quality': q}, fh, sort_keys=False)
    print("results written to %s (calibration files NOT modified)" % out_path)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax and import check (no ROS master needed)**

Run: `cd /root/catkin_ws/src/ESVO2/esvo2_core/scripts && python3 -c "import ast,sys; ast.parse(open('calibrate_imu_camera_rotation.py').read()); import imu_cam_calib_core; print('ok')"`
Expected: `ok`.

- [ ] **Step 3: Dry run without motion (ask the user whether the rig may stay still for 20 s)**

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch prophesee_ros_driver stereo.launch > $S/cal_drv.log 2>&1 & P1=$!
roslaunch sbg_driver sbg_evk4_rig.launch > $S/cal_sbg.log 2>&1 & P2=$!
rosrun esvo2_core imu_restamp.py > $S/cal_relay.log 2>&1 & P3=$!
sleep 12
timeout 60 rosrun esvo2_core calibrate_imu_camera_rotation.py _duration:=15 _out:=$S/cal_dry.yaml
kill -INT $P3 $P2 $P1; wait; free -m | awk '/Mem/{print "available MB:",$7}'
```
Expected: the script runs to the end without exceptions, prints sample counts and `frames dropped` close to 0 (a still rig gives "not enough data" — that is fine for this step); available memory stays above 2000 MB.
If `frames dropped` is large, re-run with `_window:=0.02` and use that value in Task 4. If frames are still dropped at 0.02, stop and report to the user: the spec's fallback (downscaled frames written to disk in chunks, processed after capture) is then needed as an extra task before Task 4.

- [ ] **Step 4: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/scripts/calibrate_imu_camera_rotation.py
git commit -m "Add camera-IMU rotation and time-offset calibration script

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Calibration capture and approval (user-gated)

**Files:**
- Modify (only after user approval): `esvo2_core/calib/evk4_stereo/left.yaml` and `right.yaml` (`T_b_c.data`, 12 values, row-major 3x4: rotation in positions 0-2, 4-6, 8-10; translation positions 3, 7, 11 stay 0.0)

**Interfaces:**
- Consumes: Task 3 script and results YAML (`R_b_c` 9 floats row-major, `t_d`, `quality`).
- Produces: `T_b_c` rotation in both calib files; value of `t_d` for Task 6 (`IMU_TIME_OFFSET`).

- [ ] **Step 1: Ask the user if they are ready** to rotate the rig by hand for ~60 s in front of a distant (≥3 m) textured scene, about pan, tilt and roll, moderate speed, little translation. Stop until they confirm.

- [ ] **Step 2: Capture**

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch prophesee_ros_driver stereo.launch > $S/cal_drv.log 2>&1 & P1=$!
roslaunch sbg_driver sbg_evk4_rig.launch > $S/cal_sbg.log 2>&1 & P2=$!
rosrun esvo2_core imu_restamp.py > $S/cal_relay.log 2>&1 & P3=$!
sleep 12
timeout 150 rosrun esvo2_core calibrate_imu_camera_rotation.py _duration:=60 _out:=$S/cal_run1.yaml
kill -INT $P3 $P2 $P1; wait
```

- [ ] **Step 3: Check quality against the acceptance limits**

Accept only if all hold: `corr` ≥ 0.8; `sharpness` > 0.02 (peak minus best correlation more than 50 ms away); `inlier_share` ≥ 0.7; `half_rotation_diff_deg` ≤ 2.0; `half_t_d_diff_ms` ≤ 5.0; `permutation_distance_deg` ≤ 15.0.
If any fails: ask the user for a second capture (Step 1-2 with `_out:=$S/cal_run2.yaml`), report both results. Do not proceed with failing numbers.

- [ ] **Step 4: Present `R_b_c`, `t_d` and all quality numbers to the user and ask for approval** to write them. Stop until approved.

- [ ] **Step 5: Write the rotation into both calib files (after approval)**

```bash
cd /root/catkin_ws/src/ESVO2/esvo2_core
python3 - $S/cal_run1.yaml <<'EOF'
import re, sys, yaml
res = yaml.safe_load(open(sys.argv[1]))
R = res['R_b_c']
data = [R[0], R[1], R[2], 0.0, R[3], R[4], R[5], 0.0, R[6], R[7], R[8], 0.0]
line = "  data: [" + ", ".join("%.6f" % v for v in data) + "]"
for side in ('left', 'right'):
    path = 'calib/evk4_stereo/%s.yaml' % side
    txt = open(path).read()
    new, n = re.subn(r'(T_b_c:\n  rows: 3\n  cols: 4\n)  data: \[[^\]]*\]', lambda m: m.group(1) + line, txt)
    assert n == 1, path
    open(path, 'w').write(new)
    print(path, 'updated')
print("t_d for IMU_TIME_OFFSET:", res['t_d'])
EOF
git diff --stat calib/
```
Expected: both files updated, one changed line each; note the printed `t_d`.

- [ ] **Step 6: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/calib/evk4_stereo/left.yaml esvo2_core/calib/evk4_stereo/right.yaml
git commit -m "EVK4 calib: camera-IMU rotation from gyro/event alignment

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Gyro prediction math (C++, TDD)

**Files:**
- Create: `esvo2_core/include/esvo2_core/tools/gyro_prediction.h`
- Create: `esvo2_core/test/test_gyro_prediction.cpp`
- Modify: `esvo2_core/CMakeLists.txt` (append gtest target)

**Interfaces:**
- Produces, in namespace `esvo2_core::tools`:
  - `struct GyroSample { double t; Eigen::Vector3d w; };` (t = `/imu/data_synced` stamp in s, w = rad/s in IMU frame)
  - `inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d& v);`
  - `inline bool gyroDeltaRotation(const std::vector<GyroSample>& samples, double t_from, double t_to, double t_d, Eigen::Matrix3d& R_imu);` — samples sorted by `t`; `R_imu` = IMU orientation at camera time `t_to` expressed in the IMU frame at `t_from`
  - `inline Eigen::Matrix3d imuToCameraRotation(const Eigen::Matrix3d& R_imu, const Eigen::Matrix3d& R_b_c);` — returns `R_b_cᵀ R_imu R_b_c`

- [ ] **Step 1: Write the failing test**

Create `esvo2_core/test/test_gyro_prediction.cpp`:

```cpp
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <esvo2_core/tools/gyro_prediction.h>

using esvo2_core::tools::GyroSample;
using esvo2_core::tools::gyroDeltaRotation;
using esvo2_core::tools::imuToCameraRotation;
using esvo2_core::tools::so3Exp;

static std::vector<GyroSample> constantRate(double t0, double t1, double dt, const Eigen::Vector3d &w)
{
  std::vector<GyroSample> s;
  for (double t = t0; t <= t1 + 1e-12; t += dt)
    s.push_back({t, w});
  return s;
}

static double angleBetween(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B)
{
  return Eigen::AngleAxisd(A.transpose() * B).angle();
}

TEST(GyroPrediction, ConstantRateGivesExpectedAngle)
{
  Eigen::Vector3d w(0.1, -0.4, 0.2);
  auto s = constantRate(0.0, 1.0, 0.005, w);
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.4023, 0.4431, 0.0, R));
  EXPECT_LT(angleBetween(R, so3Exp(w * (0.4431 - 0.4023))), 1e-9);
}

TEST(GyroPrediction, ClipsPartialIntervalsAtBothEnds)
{
  std::vector<GyroSample> s = {{0.000, Eigen::Vector3d(0, 0, 1)}, {0.010, Eigen::Vector3d(0, 0, 2)}, {0.020, Eigen::Vector3d(0, 0, 4)}};
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 0.005, 0.015, 0.0, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 1.0 * 0.005 + 2.0 * 0.005, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(R).axis().z(), 1.0, 1e-9);
}

TEST(GyroPrediction, TimeOffsetShiftsSamples)
{
  // IMU rotates only during IMU time [1.000, 1.050); with t_d = 0.2 that is camera time [1.200, 1.250)
  std::vector<GyroSample> s;
  for (int i = 0; i <= 400; i++)
  {
    double t = 0.5 + 0.005 * i;
    s.push_back({t, (t >= 1.0 - 1e-9 && t < 1.05 - 1e-9) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d::Zero()});
  }
  Eigen::Matrix3d R;
  ASSERT_TRUE(gyroDeltaRotation(s, 1.2, 1.25, 0.2, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 0.05, 1e-9);
  ASSERT_TRUE(gyroDeltaRotation(s, 1.0, 1.05, 0.2, R));
  EXPECT_NEAR(Eigen::AngleAxisd(R).angle(), 0.0, 1e-12);
}

TEST(GyroPrediction, RejectsMissingCoverageAndLongIntervals)
{
  auto s = constantRate(1.0, 2.0, 0.005, Eigen::Vector3d(0, 1, 0));
  Eigen::Matrix3d R;
  EXPECT_FALSE(gyroDeltaRotation(s, 0.95, 1.05, 0.0, R));   // first sample 50 ms after t_from
  EXPECT_FALSE(gyroDeltaRotation(s, 1.95, 2.05, 0.0, R));   // last sample 50 ms before t_to
  EXPECT_TRUE(gyroDeltaRotation(s, 0.99, 1.05, 0.0, R));    // 10 ms gap is tolerated
  EXPECT_FALSE(gyroDeltaRotation(s, 1.1, 1.35, 0.0, R));    // longer than 0.2 s
  EXPECT_FALSE(gyroDeltaRotation(s, 1.2, 1.2, 0.0, R));     // empty interval
  EXPECT_FALSE(gyroDeltaRotation({}, 1.0, 1.04, 0.0, R));   // no samples
}

TEST(GyroPrediction, ImuToCameraRotationConvention)
{
  // IMU z-up, camera y-down: p_imu = R_b_c p_cam with camera y = -IMU z, camera x = IMU x, camera z = IMU y
  Eigen::Matrix3d R_b_c;
  R_b_c << 1, 0, 0,
           0, 0, 1,
           0, -1, 0;
  Eigen::Vector3d w_cam(0, 0.3, 0);          // rotation about camera y
  Eigen::Vector3d w_imu = R_b_c * w_cam;     // = about IMU -z
  Eigen::Matrix3d R_c = imuToCameraRotation(so3Exp(w_imu), R_b_c);
  EXPECT_LT(angleBetween(R_c, so3Exp(w_cam)), 1e-12);
}
```

In `esvo2_core/CMakeLists.txt`, append at the end of the file (if the file ends with `cs_install()` / `cs_export()`, insert immediately before those lines):

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_gyro_prediction test/test_gyro_prediction.cpp)
endif()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core run_tests_esvo2_core_gtest_test_gyro_prediction 2>&1 | grep -E 'error|gyro_prediction.h' | head -5`
Expected: compile error `esvo2_core/tools/gyro_prediction.h: No such file or directory`.

- [ ] **Step 3: Write the implementation**

Create `esvo2_core/include/esvo2_core/tools/gyro_prediction.h`:

```cpp
#ifndef ESVO2_CORE_TOOLS_GYRO_PREDICTION_H
#define ESVO2_CORE_TOOLS_GYRO_PREDICTION_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace esvo2_core
{
namespace tools
{
// One gyro sample: t = /imu/data_synced stamp (s), w = angular velocity in the IMU frame (rad/s).
struct GyroSample
{
  double t;
  Eigen::Vector3d w;
};

inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &v)
{
  double th = v.norm();
  if (th < 1e-12)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, v / th).toRotationMatrix();
}

// Integrates gyro samples over camera-time interval (t_from, t_to].
// A sample at IMU time s belongs to camera time s + t_d and is held until the
// next sample (zero-order hold); the sample preceding t_from covers the start.
// R_imu = IMU orientation at t_to expressed in the IMU frame at t_from.
// Returns false if the samples do not reach within 20 ms of both ends, the
// interval is empty, or it is longer than 0.2 s.
inline bool gyroDeltaRotation(const std::vector<GyroSample> &samples, double t_from, double t_to, double t_d,
                              Eigen::Matrix3d &R_imu)
{
  const double kMaxGap = 0.02, kMaxInterval = 0.2;
  if (samples.empty() || !(t_to > t_from) || t_to - t_from > kMaxInterval)
    return false;
  if (samples.front().t + t_d > t_from + kMaxGap || samples.back().t + t_d < t_to - kMaxGap)
    return false;

  // first sample whose hold interval overlaps t_from: last sample with camera time <= t_from, else the first one
  size_t i = 0;
  while (i + 1 < samples.size() && samples[i + 1].t + t_d <= t_from)
    i++;

  R_imu.setIdentity();
  double t = t_from;
  for (; i < samples.size() && t < t_to; i++)
  {
    double seg_end = (i + 1 < samples.size()) ? samples[i + 1].t + t_d : t_to;
    seg_end = std::min(seg_end, t_to);
    if (seg_end > t)
    {
      R_imu = R_imu * so3Exp(samples[i].w * (seg_end - t));
      t = seg_end;
    }
  }
  return true;
}

// Camera rotation from IMU rotation with p_imu = R_b_c * p_cam.
inline Eigen::Matrix3d imuToCameraRotation(const Eigen::Matrix3d &R_imu, const Eigen::Matrix3d &R_b_c)
{
  return R_b_c.transpose() * R_imu * R_b_c;
}
} // namespace tools
} // namespace esvo2_core

#endif // ESVO2_CORE_TOOLS_GYRO_PREDICTION_H
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash
catkin_make -j4 --pkg esvo2_core run_tests_esvo2_core_gtest_test_gyro_prediction 2>&1 | grep -E '\[  (PASSED|FAILED) |tests? ran' | head -5
catkin_test_results build/test_results/esvo2_core
```
Expected: `[  PASSED  ] 5 tests.` and `Summary: 5 tests, 0 errors, 0 failures, 0 skipped`.

- [ ] **Step 5: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/tools/gyro_prediction.h esvo2_core/test/test_gyro_prediction.cpp esvo2_core/CMakeLists.txt
git commit -m "Add ROS-free gyro rotation prediction with gtest suite

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Wire the predictor into tracking (prediction off by default)

**Files:**
- Modify: `esvo2_core/include/esvo2_core/esvo2_Tracking.h` (includes block; public methods near `void refImuCallback(...)` at line 64; private members near `ros::Subscriber imu_sub_;` line 98 and `bool bUseImu_;` line 154)
- Modify: `esvo2_core/src/esvo2_Tracking.cpp` (constructor after `bUseImu_ = tools::param(pnh_, "USE_IMU", true);`; subscriptions after `imu_sub_ = nh_.subscribe(...)`; `curDataTransferring()` lines ~244-312; new methods at end of namespace)
- Modify: `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`
- Modify: `esvo2_core/launch/system/system_evk4_mapping.launch` (tracking node block)

**Interfaces:**
- Consumes: Task 5 `tools::GyroSample`, `tools::gyroDeltaRotation`, `tools::imuToCameraRotation`; topic `/imu/data_synced` (Task 1); `R_b_c_` (already loaded from `T_b_c`); `IMU_TIME_OFFSET` value from Task 4.
- Produces: params `IMU_ROTATION_PREDICTION` (bool, default false), `IMU_TIME_OFFSET` (double, default 0.0); subscriber on `imu_prediction`; log line every 5 s `IMU rotation prediction: <ok> predicted, <skipped> skipped, mean <deg> deg/frame`.

- [ ] **Step 1: Header changes**

In `esvo2_core/include/esvo2_core/esvo2_Tracking.h`, after `#include <esvo2_core/factor/imu_integration.h>` add:
```cpp
#include <esvo2_core/tools/gyro_prediction.h>
```
After `void refImuCallback(const sensor_msgs::ImuPtr &msg);` add:
```cpp
    void imuPredictionCallback(const sensor_msgs::ImuConstPtr &msg);
    void predictRotationWithGyro(double t_prev_frame, double t_cur_frame);
```
After `ros::Subscriber imu_sub_;` add:
```cpp
    ros::Subscriber imu_prediction_sub_;
```
After `bool bUseImu_;` add:
```cpp
    // gyro rotation prediction (IMU_ROTATION_PREDICTION)
    bool bImuRotationPrediction_;
    double imuTimeOffset_; // camera time = IMU time + imuTimeOffset_
    std::mutex gyro_mutex_;
    std::deque<tools::GyroSample> gyroBuf_;
    size_t nPredOk_ = 0, nPredSkip_ = 0;
    double predAngleSumDeg_ = 0.0;
    ros::WallTime lastPredLog_;
```

- [ ] **Step 2: Constructor params and subscriber**

In `esvo2_core/src/esvo2_Tracking.cpp`, after `bUseImu_ = tools::param(pnh_, "USE_IMU", true);` add:
```cpp
  bImuRotationPrediction_ = tools::param(pnh_, "IMU_ROTATION_PREDICTION", false);
  imuTimeOffset_ = tools::param(pnh_, "IMU_TIME_OFFSET", 0.0);
  lastPredLog_ = ros::WallTime::now();
```
After `imu_sub_ = nh_.subscribe("/imu/data", 0, &esvo2_Tracking::refImuCallback, this);// local map in the ref view.` add:
```cpp
  if (bImuRotationPrediction_)
  {
    if (bUseImu_)
      LOG(WARNING) << "IMU_ROTATION_PREDICTION is ignored while USE_IMU is true";
    imu_prediction_sub_ = nh_.subscribe("imu_prediction", 2000, &esvo2_Tracking::imuPredictionCallback, this);
    LOG(INFO) << "IMU rotation prediction enabled, IMU_TIME_OFFSET = " << imuTimeOffset_ << " s";
  }
```

- [ ] **Step 3: Use the prediction in `curDataTransferring()`**

In `curDataTransferring()`, replace
```cpp
  if(cur_.t_ == TS_it->first)
    return false;
  cur_.t_ = TS_it->first;
```
with
```cpp
  if(cur_.t_ == TS_it->first)
    return false;
  const double t_prev_frame = cur_.t_.toSec();
  cur_.t_ = TS_it->first;
```
and replace
```cpp
    Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
    T_world_cur_.block(0, 0, 3, 3) = fixRotationMatrix(R_w_c);
    cur_.tr_ = Transformation(T_world_cur_);
```
with
```cpp
    if(bImuRotationPrediction_ && !bUseImu_ && ESVO2_System_Status_ == "WORKING") // no prediction during INITIALIZATION
      predictRotationWithGyro(t_prev_frame, cur_.t_.toSec());
    Eigen::Matrix3d R_w_c = T_world_cur_.block(0, 0, 3, 3);
    T_world_cur_.block(0, 0, 3, 3) = fixRotationMatrix(R_w_c);
    cur_.tr_ = Transformation(T_world_cur_);
```

- [ ] **Step 4: Add the two methods**

At the end of `esvo2_core/src/esvo2_Tracking.cpp`, inside the `esvo2_core` namespace (before its closing brace), add:
```cpp
void esvo2_Tracking::imuPredictionCallback(const sensor_msgs::ImuConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(gyro_mutex_);
  const double t = msg->header.stamp.toSec();
  if (!gyroBuf_.empty() && t <= gyroBuf_.back().t)
    return;
  gyroBuf_.push_back({t, Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)});
  while (!gyroBuf_.empty() && gyroBuf_.front().t < t - 2.0)
    gyroBuf_.pop_front();
}

// Rotates T_world_cur_ (the previous frame's pose) by the gyro-integrated
// camera rotation between the two frames. Translation is left unchanged.
void esvo2_Tracking::predictRotationWithGyro(double t_prev_frame, double t_cur_frame)
{
  std::vector<tools::GyroSample> samples;
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    samples.assign(gyroBuf_.begin(), gyroBuf_.end());
  }
  Eigen::Matrix3d R_imu;
  if (tools::gyroDeltaRotation(samples, t_prev_frame, t_cur_frame, imuTimeOffset_, R_imu))
  {
    Eigen::Matrix3d R_c = tools::imuToCameraRotation(R_imu, R_b_c_);
    T_world_cur_.block<3, 3>(0, 0) = T_world_cur_.block<3, 3>(0, 0) * R_c;
    nPredOk_++;
    predAngleSumDeg_ += Eigen::AngleAxisd(R_c).angle() * 180.0 / M_PI;
  }
  else
  {
    nPredSkip_++;
  }
  if ((ros::WallTime::now() - lastPredLog_).toSec() >= 5.0)
  {
    LOG(INFO) << "IMU rotation prediction: " << nPredOk_ << " predicted, " << nPredSkip_ << " skipped, mean "
              << (nPredOk_ ? predAngleSumDeg_ / nPredOk_ : 0.0) << " deg/frame";
    if (nPredSkip_ > 0 && nPredOk_ == 0)
      LOG(WARNING) << "IMU rotation prediction: no gyro coverage (is /imu/data_synced publishing?)";
    nPredOk_ = nPredSkip_ = 0;
    predAngleSumDeg_ = 0.0;
    lastPredLog_ = ros::WallTime::now();
  }
}
```

- [ ] **Step 5: Params and remap**

In `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`, after `USE_IMU: False` add (use the `t_d` from Task 4, shown here as `0.0` until then):
```yaml
# Gyro rotation prediction (rotation only; translation stays vision-only).
# Needs /imu/data_synced (imu_restamp.py) and a calibrated T_b_c rotation.
IMU_ROTATION_PREDICTION: False
# camera time = IMU time + IMU_TIME_OFFSET (s), from calibrate_imu_camera_rotation.py
IMU_TIME_OFFSET: 0.0
```
In `esvo2_core/launch/system/system_evk4_mapping.launch`, inside the `esvo2_Tracking` node, after `<remap from="pointcloud" to="/esvo2_mapping/pointcloud_local2" />` add:
```xml
    <remap from="imu_prediction" to="/imu/data_synced" />
```

- [ ] **Step 6: Build**

Confirm no pipeline is running, then:
```bash
cd /root/catkin_ws && source /opt/ros/noetic/setup.bash
catkin_make -j4 --pkg esvo2_core > $S/build_task6.log 2>&1; echo "exit=$?"; grep -E 'error' $S/build_task6.log | head
catkin_make -j4 --pkg esvo2_core run_tests_esvo2_core_gtest_test_gyro_prediction > /dev/null 2>&1; catkin_test_results build/test_results/esvo2_core
```
Expected: `exit=0`, no errors; `Summary: 5 tests, 0 errors, 0 failures`.

- [ ] **Step 7: Regression smoke run with prediction off (ask the user whether the rig may be moved freely for ~60 s)**

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch esvo2_core evk4_live_all.launch gui:=false > $S/smoke_off.log 2>&1 & LP=$!
sleep 15; timeout 10 rostopic hz /imu/data_synced | grep average | tail -1
sleep 45; kill -INT $LP; wait $LP
grep -ac 'has died' $S/smoke_off.log; grep -ac 'IMU rotation prediction' $S/smoke_off.log
```
Expected: `/imu/data_synced` ≈ 200 Hz; `has died` count 0; `IMU rotation prediction` count 0 (feature off).

- [ ] **Step 8: Commit (ask the user first)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/include/esvo2_core/esvo2_Tracking.h esvo2_core/src/esvo2_Tracking.cpp esvo2_core/cfg/tracking/tracking_evk4_AA.yaml esvo2_core/launch/system/system_evk4_mapping.launch
git commit -m "Tracking: optional gyro rotation prediction (IMU_ROTATION_PREDICTION, off)

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Enable and validate (user-gated)

**Files:**
- Create: `esvo2_core/scripts/validate_rotation_tracking.py`
- Temporary edit: `esvo2_core/src/core/RegProblemSolverLM.cpp` (existing `[diag]` block: add dark-share metric)
- Modify: `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml` (`IMU_ROTATION_PREDICTION: True`, `IMU_TIME_OFFSET: <t_d>`)

**Interfaces:**
- Consumes: Tasks 1, 4, 6; `/imu/data_synced`, `/esvo2_tracking/pose_pub` (`geometry_msgs/PoseStamped`); existing `[diag solver]` log lines (`valid=`, `mean_valid_residual=`, `rot_update=`, `cur_vs_ref_rot=`).
- Produces: pass/fail against the Global Constraints' success criteria; numbers for Task 8 docs.

- [ ] **Step 1: Write the validation script**

Create `esvo2_core/scripts/validate_rotation_tracking.py`, `chmod +x`:

```python
#!/usr/bin/env python3
"""Compare tracking poses with the gyro.

mode slowturn: net rotation (gyro-integrated vs pose) at each hold (>= 1 s with |w| < 0.05 rad/s).
mode mixed:    pose/gyro rotation speed ratio in gyro speed bins.
Params: ~mode (slowturn|mixed), ~duration (s).
"""
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


def q2R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def exp_so3(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def main():
    rospy.init_node('validate_rotation_tracking', anonymous=True)
    mode = rospy.get_param('~mode', 'slowturn')
    duration = rospy.get_param('~duration', 70.0)
    imu, poses = [], []
    rospy.Subscriber('/imu/data_synced', Imu, lambda m: imu.append(
        (m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)), queue_size=20000)
    rospy.Subscriber('/esvo2_tracking/pose_pub', PoseStamped, lambda m: poses.append(
        (m.header.stamp.to_sec(), m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z)),
        queue_size=5000)
    time.sleep(1.0)
    imu.clear(); poses.clear()
    print(">>> recording %.0f s (%s)" % (duration, mode))
    time.sleep(duration)

    I = np.array(imu)
    P = np.array(sorted(poses))
    print("imu samples %d, poses %d" % (len(I), len(P)))
    if len(P) < 20:
        print("FAIL: too few poses")
        return
    wnorm = np.linalg.norm(I[:, 1:4], axis=1)

    if mode == 'slowturn':
        still = wnorm < 0.05
        bias = I[still, 1:4].mean(axis=0) if still.sum() > 200 else np.zeros(3)
        R = np.eye(3); gR = []
        for k in range(len(I)):
            dt = I[k + 1, 0] - I[k, 0] if k + 1 < len(I) else 0.005
            gR.append(R.copy())
            R = R @ exp_so3((I[k, 1:4] - bias) * dt)
        gR = np.array(gR)
        holds, k = [], 0
        while k < len(I):
            if still[k]:
                j = k
                while j + 1 < len(I) and still[j + 1]:
                    j += 1
                if I[j, 0] - I[k, 0] >= 1.0:
                    holds.append(0.5 * (I[k, 0] + I[j, 0]))
                k = j + 1
            else:
                k += 1
        if len(holds) < 2:
            print("FAIL: fewer than two holds detected")
            return
        g0 = gR[np.argmin(np.abs(I[:, 0] - holds[0]))]
        p0 = P[np.argmin(np.abs(P[:, 0] - holds[0]))]
        R_p0 = q2R(*p0[1:5])
        ok = True
        for h in holds[1:]:
            g = angle_deg(g0.T @ gR[np.argmin(np.abs(I[:, 0] - h))])
            pk = P[np.argmin(np.abs(P[:, 0] - h))]
            p = angle_deg(R_p0.T @ q2R(*pk[1:5]))
            rel = abs(p - g) / max(g, 1e-6)
            passed = g < 5.0 or rel <= 0.2
            ok &= passed
            print("hold at %.1f s: gyro net %.1f deg, pose net %.1f deg, error %.0f%% -> %s" % (
                h - I[0, 0], g, p, 100 * rel, 'ok' if passed else 'FAIL'))
        print("slowturn result: %s" % ('PASS' if ok else 'FAIL'))
    else:
        rows = []
        for a, b in zip(P[:-1], P[1:]):
            dt = b[0] - a[0]
            if not 0.005 < dt <= 0.2:
                continue
            ang = 2 * np.arccos(min(1.0, abs(float(np.dot(a[1:5], b[1:5])))))
            if ang / dt > 10:
                continue
            m = (I[:, 0] >= a[0]) & (I[:, 0] < b[0])
            if m.sum() >= 3:
                rows.append((wnorm[m].mean(), ang / dt))
        R = np.array(rows)
        for lo, hi in ((0.0, 0.1), (0.1, 0.5), (0.5, 1.2), (1.2, 99.0)):
            m = (R[:, 0] >= lo) & (R[:, 0] < hi)
            if m.sum() < 10:
                print("bin %.1f-%.1f rad/s: too few (%d)" % (lo, hi, m.sum()))
                continue
            print("bin %.1f-%.1f rad/s: n=%d gyro %.3f pose %.3f ratio %.2f" % (
                lo, hi, m.sum(), np.median(R[m, 0]), np.median(R[m, 1]), np.median(R[m, 1]) / np.median(R[m, 0])))


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Add the dark-share metric to the temporary solver instrumentation**

In `esvo2_core/src/core/RegProblemSolverLM.cpp`:
1. Add `#include <algorithm> // [diag]` and `#include <vector> // [diag]` next to the existing `[diag]` includes.
2. In `namespace diag`, add `static double dark_share = 0;` to the statics, add a parameter `double dark_share_p5` as the last parameter of `add_input` with body line `dark_share += dark_share_p5;`, append `<< " dark_p5=" << 100 * dark_share / s << "%"` to the `LOG(INFO)` stream before its final `;`, and add `dark_share` to the reset line.
3. In `solve_analytical()`'s `[diag]` input block, before the call to `diag::add_input(...)`, add:
```cpp
    const Eigen::MatrixXd &tsn = regProblemPtr_->cur_->pTsObs_->TS_negative_left_;
    std::vector<double> grid;
    grid.reserve((tsn.rows() / 4 + 1) * (tsn.cols() / 4 + 1));
    for (int r = 0; r < tsn.rows(); r += 4)
      for (int col = 0; col < tsn.cols(); col += 4)
        grid.push_back(tsn(r, col));
    std::nth_element(grid.begin(), grid.begin() + grid.size() / 20, grid.end());
    const double p5 = grid[grid.size() / 20];
    size_t dark = 0;
    for (const auto &ri : regProblemPtr_->ResItemsStochSampled_)
      if (ri.residual_(0) < 254.999 && ri.residual_(0) <= p5)
        dark++;
```
and pass `n ? double(dark) / n : 0.0` as the new last argument of `diag::add_input(...)`.

Build (pipeline stopped): `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core > $S/build_task7.log 2>&1; echo "exit=$?"`
Expected: `exit=0`.

- [ ] **Step 3: Baseline with prediction OFF (ask the user if ready: slow turn protocol)**

Protocol to tell the user: hold still 5 s, slowly turn ~45° over ~20 s, hold 5 s, turn back ~20 s, hold still until the end (~70 s), textured scene 1-2 m away.

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch esvo2_core evk4_live_all.launch gui:=false > $S/val_off.log 2>&1 & LP=$!
sleep 15; timeout 120 rosrun esvo2_core validate_rotation_tracking.py _mode:=slowturn _duration:=70
kill -INT $LP; wait $LP
grep -a '\[diag solver\]' $S/val_off.log | sed -E 's/.*dark_p5=([0-9.]+)%.*/\1/' | sort -n | awk '{a[NR]=$1} END {print "median dark_p5 (off):", a[int((NR+1)/2)] "%"}'
```
Record the printed holds, result, and median `dark_p5`.

- [ ] **Step 4: Enable prediction**

Set in `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`: `IMU_ROTATION_PREDICTION: True` and `IMU_TIME_OFFSET: <t_d from Task 4>`.

- [ ] **Step 5: Slow turn with prediction ON (ask the user if ready, same protocol)**

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch esvo2_core evk4_live_all.launch gui:=false > $S/val_on_slow.log 2>&1 & LP=$!
sleep 15; timeout 120 rosrun esvo2_core validate_rotation_tracking.py _mode:=slowturn _duration:=70
kill -INT $LP; wait $LP
grep -a 'IMU rotation prediction:' $S/val_on_slow.log | tail -3
grep -a '\[diag solver\]' $S/val_on_slow.log | sed -E 's/.*dark_p5=([0-9.]+)%.*/\1/' | sort -n | awk '{a[NR]=$1} END {print "median dark_p5 (on):", a[int((NR+1)/2)] "%"}'
grep -ac 'has died' $S/val_on_slow.log
```
Expected (pass): `slowturn result: PASS`; prediction log shows predictions, few skips; median `dark_p5` clearly above the Step 3 value; `has died` 0.

- [ ] **Step 6: Mixed motion with prediction ON (ask the user if ready: alternate a few seconds still / slow / fast turning for ~70 s)**

```bash
source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash
roslaunch esvo2_core evk4_live_all.launch gui:=false > $S/val_on_mixed.log 2>&1 & LP=$!
sleep 15; timeout 120 rosrun esvo2_core validate_rotation_tracking.py _mode:=mixed _duration:=70
top -b -n1 -o %CPU | awk '$8 !~ /Z/ && /image_r|esvo2_|prophes/ {printf "%s %s%% %s\n",$12,$9,$6}'; free -m | awk '/Mem/{print "available MB:",$7}'
kill -INT $LP; wait $LP; grep -ac 'has died' $S/val_on_mixed.log
```
Expected (pass): bin 0.1-0.5 ratio between 0.8 and 1.2; bins above clearly higher than today's 0.04-0.12; CPU/memory similar to earlier runs (mapping ~2-3.5 cores, tracking < 1 core, ≥ 2 GB available); `has died` 0.

- [ ] **Step 7: On failure**

If Step 5 or 6 fails: set `IMU_ROTATION_PREDICTION: False` again, keep all logs, and report the measured numbers (holds, bins, `dark_p5`, prediction counts) to the user before changing anything else. Stop.

- [ ] **Step 8: Commit (ask the user first; commit only the validation script and the config, NOT RegProblemSolverLM.cpp)**

```bash
cd /root/catkin_ws/src/ESVO2
git add esvo2_core/scripts/validate_rotation_tracking.py esvo2_core/cfg/tracking/tracking_evk4_AA.yaml
git commit -m "Enable gyro rotation prediction for EVK4 tracking, add validation script

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Cleanup and documentation

**Files:**
- Revert: `esvo2_core/src/core/RegProblemSolverLM.cpp` (all `[diag]` code)
- Modify: `EVK4_STEREO_SETUP.md`

**Interfaces:**
- Consumes: numbers from Tasks 1, 4, 7.
- Produces: final documented state.

- [ ] **Step 1: Remove the temporary instrumentation**

Run: `cd /root/catkin_ws/src/ESVO2 && git checkout -- esvo2_core/src/core/RegProblemSolverLM.cpp && grep -c 'diag' esvo2_core/src/core/RegProblemSolverLM.cpp`
Expected: `0`.

Rebuild (pipeline stopped): `cd /root/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -j4 --pkg esvo2_core > $S/build_task8.log 2>&1; echo "exit=$?"`
Expected: `exit=0`.

- [ ] **Step 2: Correct the wrong IMU note**

In `EVK4_STEREO_SETUP.md`, replace the bullet that starts `  - **Tracking hardcodes the IMU sample period:**` (and its continuation lines up to `motion by 5x unless the real dt is used.`) with:
```markdown
  - **IMU sample period:** `esvo2_Tracking::refImuCallback()` uses the real
    stamp difference; only the very first sample gets `0.001` s. (An earlier
    version of this note wrongly said it assumed 1000 Hz.) With the bunched
    `/imu/data` stamps those differences were wrong per sample; the rotation
    predictor uses `/imu/data_synced` instead.
```

- [ ] **Step 3: Document the new pieces**

Append to the "This repo (`ESVO2`)" list under "Changes made" in `EVK4_STEREO_SETUP.md`, filling in the measured values from Tasks 1, 4 and 7:
```markdown
- **New:** `esvo2_core/scripts/imu_restamp.py` (+ `imu_restamp_core.py`,
  unit tests in `esvo2_core/test/`): re-stamps `/sbg/imu_data` with the
  IMU's own clock, offset = min over 2 s of (receive − device time), and
  publishes `/imu/data_synced`. Started by `evk4_live_all.launch` with
  `imu:=true`. Measured: stamp gaps <median/max from Task 1>, receive −
  stamp <mean> ms, stable over 3 min.
- **New:** `esvo2_core/scripts/calibrate_imu_camera_rotation.py`
  (+ `imu_cam_calib_core.py`, unit tests): estimates `R_b_c` (rectified left
  camera → IMU, `p_imu = R_b_c · p_cam`) and `t_d` (camera time = IMU time +
  `t_d`) by aligning event-based camera angular velocity with the gyro.
  Procedure: camera driver + SBG driver + relay only; rig rotated by hand
  about all axes in front of a ≥3 m textured scene for 60 s. Result:
  `t_d = <value> s`, peak correlation <value>, halves differ by <value>° /
  <value> ms, <value>° from the nearest axis permutation. Written into the
  rotation part of `T_b_c` in both calib files (translation still 0).
- **New:** gyro rotation prediction in tracking
  (`esvo2_core/include/esvo2_core/tools/gyro_prediction.h`, gtest
  `test_gyro_prediction`, run with
  `catkin_make run_tests_esvo2_core_gtest_test_gyro_prediction`). Params in
  `tracking_evk4_AA.yaml`: `IMU_ROTATION_PREDICTION` and `IMU_TIME_OFFSET`;
  tracking topic `imu_prediction` remapped to `/imu/data_synced`. Each frame
  starts from `R_world_prev · R_b_cᵀ · R_imu · R_b_c`, translation from vision;
  ESVO2's original `USE_IMU` path stays off. Validation
  (`esvo2_core/scripts/validate_rotation_tracking.py`): slow turn
  <per-hold numbers, off vs on>; mixed motion bins <ratios>; share of map
  points in the darkest 5% of the negative TS <off> → <on>.
```
Also update the "Known limitations" IMU bullet: remove the timestamp sub-bullet (solved by the relay) and the `T_b_c` sub-bullet (calibrated, rotation only), and keep: translation prediction and mapping's IMU backend still off; lever arm not calibrated.

- [ ] **Step 4: Verify the doc renders sensibly**

Run: `cd /root/catkin_ws/src/ESVO2 && grep -n 'hardcodes the IMU sample period\|<value>\|<mean>\|<per-hold\|<ratios>\|<off>' EVK4_STEREO_SETUP.md`
Expected: no output (no stale note, no unfilled values).

- [ ] **Step 5: Commit and push (ask the user first; pushing needs a token with Contents: write on `lecrosnier/ESVO2_docker`)**

```bash
cd /root/catkin_ws/src/ESVO2
git status --short   # expect only EVK4_STEREO_SETUP.md modified
git add EVK4_STEREO_SETUP.md
git commit -m "Document IMU re-stamping, camera-IMU calibration and rotation prediction

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
git log --oneline -10
```
Push only if the user asks: `git push -u fork imu-rotation-prediction`.
