# IMU rotation prediction for ESVO2 tracking (live EVK4 + SBG rig)

Date: 2026-09-15
Status: design approved section by section, awaiting spec review

## Problem

On the live stereo EVK4 rig, ESVO2 tracking reports `WORKING` but does not
follow the rig's motion:

- In a slow ~70° turn the gyro measured 70°, the poses 5–6°.
- Across all motion speeds the pose rotation speed stays at ~0.06 rad/s
  (gyro 0.03–1.9 rad/s). The solver applies ~0.17–0.2° per solve regardless
  of motion, i.e. noise-driven steps.

Measured causes (see `EVK4_STEREO_SETUP.md`):

- Geometry is consistent: projecting a local map with tracking's pose at the
  map's own timestamp puts 24% of points in the darkest 5% of the negative
  time surface (random: 5%). The advantage halves within 40 ms of motion.
- Each frame starts from the previous pose with no motion prior
  (`USE_IMU: False`), time surfaces arrive at 25 Hz, the 1280×720 sensor with
  fx ≈ 1690 px moves ~7 px per frame even in a slow turn, and edges in the
  negative TS are thin and grey. The solver's convergence range is smaller than
  the inter-frame image motion, so it never locks on.

ESVO2's intended remedy is IMU motion prediction. Its existing tracking IMU
path cannot be enabled as is: it also predicts translation from
double-integrated acceleration with a default gravity and no velocity (mapping's
IMU backend, which would supply them, is off), its integration window is tied
to the reference-map time, and it composes the IMU rotation as
`R_b_c · q · R_b_c⁻¹`, the opposite of the camera→IMU convention used by
ESVO2's backend (`RIC_`).

## Goal and scope

Round 1: **rotation prediction only**.

- Tracking starts each frame from a gyro-predicted rotation; translation stays
  vision-only.
- Mapping's IMU backend stays off (`USE_IMU: False` in mapping and tracking).

Success: the pose's net rotation follows the gyro within 20% in a slow
controlled turn, and pose/gyro rotation speed ratio is 0.8–1.2 at
0.1–0.5 rad/s (today 0.04–0.45), with a clear improvement above that.

Out of scope: translation prediction, mapping's IMU backend, gyro bias
estimation, lever-arm measurement, higher TS rates, a full Kalibr calibration.

## Architecture

```
/sbg/imu_data ──► imu_restamp.py ──► /imu/data_synced ──► esvo2_Tracking
 (IMU clock µs)    (section 1)        (even 5 ms, ROS time)   gyro buffer
                                                                  │
/evk4_left/events ──► calibrate_imu_camera_rotation.py            ▼
/imu/data_synced  ──►  (section 2, run once)  ──► R_b_c, t_d ──► gyroDeltaRotation()
                                                                  │  (section 3)
                                                                  ▼
                                                   initial rotation for the solver
```

## Section 1: IMU re-stamping relay

**Why:** `/imu/data` is stamped with ROS receive time (`time_reference: "ros"`)
and the serial link delivers batches: 65% of consecutive stamp gaps are under
1 ms, p90 15 ms, max 20 ms. The IMU's own clock (`SbgImuData.time_stamp`) is
exactly 5.00 ms apart with no lost samples.

**What:** `esvo2_core/scripts/imu_restamp.py`, a Python ROS node started by
`evk4_live_all.launch` when `imu:=true`.

- In: `/sbg/imu_data` (`sbg_driver/SbgImuData`: `time_stamp` uint32 µs,
  `accel`, `gyro`, already ENU when `use_enu: true`).
- Out: `/imu/data_synced` (`sensor_msgs/Imu`, `frame_id: imu_link`),
  `linear_acceleration` and `angular_velocity` copied, orientation marked
  unavailable (`orientation_covariance[0] = -1`).

**Timestamps:**

1. Unwrap `time_stamp` (uint32 µs wraps every ~71.6 min) to continuous
   seconds `s`.
2. `offset = min over the last ~2 s of (receive_ros_time − s)`. Serial delay
   and batching only add delay, so the minimum is the best estimate; the
   window tracks slow clock drift.
3. Publish at `s + offset`.

**Failure handling:** a backward jump in unwrapped device time that is not a
wrap resets the offset window, logs a warning and drops that sample; no input
for >0.5 s logs a throttled warning.

**Not handled here:** the constant IMU↔camera latency, which is calibrated in
section 2 as `t_d` and applied in section 3.

**Tests (no motion):** published stamp spacing 5.00 ± 0.1 ms; mean
(receive − published) ≥ 0 and stable over several minutes; sample count equals
`/sbg/imu_data`.

## Section 2: camera–IMU rotation and time-offset calibration

**Target quantities:**

- `R_b_c`: rotation from the **rectified** left camera frame to the IMU frame,
  `p_imu = R_b_c · p_cam` (ESVO2 poses and projections use the rectified left
  camera, `P` in `left.yaml`).
- `t_d`: camera time = IMU (`/imu/data_synced`) time + `t_d`.

**What:** `esvo2_core/scripts/calibrate_imu_camera_rotation.py` (kept in the
repo). Dependencies: numpy, OpenCV, rospy, PyYAML (all installed; no SciPy,
no PyTorch, no Kalibr). Runs with only the camera driver and the relay up.

**Method:**

1. **Camera angular velocity from events.** Accumulate `/evk4_left/events`
   into 10 ms count images by event timestamp, at 640×360. Track corners
   between consecutive images with pyramidal Lucas-Kanade. Map points to the
   rectified left frame with `cv2.undistortPoints(K, D, R_rect, P)` and fit a
   pure rotation between the two bearing sets with RANSAC. Output ω_cam every
   10 ms plus inlier share. Processing is online and keeps only rotation
   estimates. Fallback if Python cannot keep up: 20 ms windows and downscaled
   frames written to disk in chunks, never a raw event recording.
2. **Time offset.** Cross-correlate |ω_cam| with |ω_imu| over ±100 ms in
   1 ms steps; peak → `t_d`.
3. **Rotation.** After shifting by `t_d`, solve `ω_imu ≈ R_b_c · ω_cam` with
   Kabsch least squares and iterative outlier rejection.

**Quality output:** peak correlation and peak sharpness; RMS of
`ω_imu − R_b_c · ω_cam` and inlier share; first-half vs second-half
repeatability (`R_b_c` within ~2°, `t_d` within ~5 ms); plausibility of
`R_b_c` as an axis permutation (IMU z up in ENU, camera y down) plus a small
tilt.

**Output handling:** results are printed and written to a results YAML.
Nothing is written into `left.yaml` / `right.yaml` until the user reviews the
numbers; then only the rotation part of `T_b_c` changes (translation stays 0).
`t_d` goes into the tracking config (section 3).

**Capture procedure (user asked before starting):** hand-held rig pointed at a
distant (≥3 m) textured scene; rotate about pan, tilt and roll at moderate
speed with little translation, ~60 s; optionally a second capture for
repeatability.

**Risks:** translation, low texture, or very fast motion (blur, rate cap)
degrade the fit; the quality metrics detect this and the remedy is another
capture.

## Section 3: gyro rotation predictor in tracking

**Behaviour:** when `IMU_ROTATION_PREDICTION` is true and tracking is
`WORKING`, `esvo2_Tracking::curDataTransferring()` initialises the new frame
with

- `R_world_cur = R_world_prev · R_c(prev→cur)`
- `t_world_cur = t_world_prev`

instead of the unchanged previous pose; the solver then refines as today. No
change in `INITIALIZATION`. ESVO2's original `USE_IMU` path is untouched and
stays off.

**Components:**

1. `esvo2_core/include/esvo2_core/tools/gyro_prediction.h`, ROS-free:
   `bool gyroDeltaRotation(const std::vector<GyroSample>& samples, double t_from, double t_to, double t_d, Eigen::Matrix3d& R_imu)`.
   A sample at IMU time `s` belongs to camera time `s + t_d`. Integrates the
   samples covering `(t_from, t_to]`, zero-order hold, clipping partial first
   and last intervals, composing on SO(3). Returns false if samples do not
   reach within 20 ms of both ends or `t_to − t_from > 0.2 s`.
2. Frame conversion: `R_c = R_b_cᵀ · R_imu · R_b_c`.
3. Tracking wiring: new subscriber on topic `imu_prediction` (remapped to
   `/imu/data_synced`) filling a 2 s ring buffer under its own mutex;
   `curDataTransferring()` keeps the previous frame time before overwriting
   `cur_.t_`, calls the function, and on failure falls back to today's
   behaviour with a warning throttled to 5 s. A 5 s summary logs predictions
   made/skipped and the mean predicted rotation per frame.
4. Parameters in `cfg/tracking/tracking_evk4_AA.yaml`:
   `IMU_ROTATION_PREDICTION: False` (true only after validation),
   `IMU_TIME_OFFSET: <t_d>`. `R_b_c` comes from `T_b_c` in the calibration
   file.
5. No gyro bias correction (measured ~0.003 rad/s ≈ 0.007° per frame).

**Tests (TDD):** a gtest suite written before the function, added to
esvo2_core with `catkin_add_gtest` (built only with `catkin_make run_tests`):

- constant rate → expected angle;
- partial interval clipping at both ends;
- `t_d` shifts which samples are used;
- insufficient coverage or interval > 0.2 s → false;
- frame conversion: a pure rotation about camera y maps correctly for a known
  `R_b_c`.

## Section 4: validation, rollout, cleanup

**Order, each step gated by its check:**

1. Relay: implement, add to `evk4_live_all.launch`, verify (section 1 tests).
2. Calibration: implement script; user capture (user asked first); user
   reviews `R_b_c`, `t_d` and quality metrics; on approval write `T_b_c`
   rotation and `IMU_TIME_OFFSET`.
3. Predictor: failing gtests, implementation, wiring, params; build, tests,
   headless smoke run with prediction **off** to confirm no regression.
4. Enable `IMU_ROTATION_PREDICTION: True` and validate (user asked before each
   motion test):
   - slow controlled turn: pose net rotation within 20% of gyro at each hold;
   - mixed motion speed bins: pose/gyro ratio 0.8–1.2 at 0.1–0.5 rad/s, clear
     improvement above;
   - solver instrumentation: share of map points in the darkest 5% of the
     negative TS clearly above today's, and cur-vs-ref rotation follows the
     motion instead of staying within ~0.5°;
   - no node deaths or new crashes, CPU/memory similar to today.
   On failure the flag goes back to `False` and findings are reported before
   further changes.
5. Cleanup and docs: remove the temporary `[diag]` solver instrumentation;
   correct the wrong "tracking hardcodes a 1 ms IMU period" note in
   `EVK4_STEREO_SETUP.md` (only the first sample uses 0.001 s; the real issue
   was bunched stamps); document relay, calibration procedure and results,
   predictor, parameters, validation numbers.
6. Git: first commit the pending SBG output-rate docs and config copy on their
   own; keep the `[diag]` instrumentation uncommitted until it is removed; IMU
   work in topic commits on a branch. Commits and pushes only after asking.

**Constraints carried over:** keep capture and analysis scripts lightweight
(an earlier 2 s event bag loaded into memory got the desktop session
OOM-killed); build with limited parallelism; the pipeline must be stopped
before rebuilding its binaries.
