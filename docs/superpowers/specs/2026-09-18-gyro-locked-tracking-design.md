# Gyro-locked tracking rotation and gyro bias correction (live EVK4 + SBG rig)

Date: 2026-09-18
Status: design approved section by section, awaiting spec review
Builds on: `2026-09-15-imu-rotation-prediction-design.md` (round 1)

## Problem

After the stereo side swap was fixed (serials, calibration; see
`EVK4_STEREO_SETUP.md`), mapping is stable and depth is metric (median
2.10 m on a wall at ~2.05 m), but tracking loses most translation:

- Live, a 1 m sideways slide and 13–15 s hallway walks were tracked as
  0.01–0.4 m.
- On a recorded 1 m slide bag (`slide_lr.bag`, rig slid 1 m left, heading
  held), replay at 0.25× gives −0.26 m.

Measured causes:

1. **Rotation–translation ambiguity.** Beside a planar wall at one depth,
   with mostly vertical edges and a 41° horizontal FOV, a lateral slide and a
   yaw produce near-identical image motion. The optimizer explains the slide
   as 10–24° of yaw that did not happen (bias-corrected gyro: −0.4°).
   Zeroing the rotation columns of the registration Jacobian (translation-only
   solve, rotation from the gyro) raised the replayed slide from −0.26 m to
   **−0.55 m**.
2. **Gyro bias is not removed.** The round-1 gyro prediction integrates raw
   rates. On this SBG unit the bias is ~0.004 rad/s (0.23 °/s) per axis,
   i.e. ~0.4 °/s of false rotation fed into every prediction.
3. **Event-rate saturation** (not addressed here): during the slide the left
   camera sat at the 4 M ev/s ERC cap, with a ~0.5 M ev/s noise floor at rest.
   Time surfaces are faint and noisy. Likely the main part of the remaining
   45% shortfall; needs the rig (rate cap / sensor noise filter).

Ruled out: calibration scale, sensor resolution (2× and 4× binning), event
delivery lag, time-surface rate, mapping fusion profile, and a code regression
(vision-only ESVO2, fork and upstream, tracks MVSEC `indoor_flying1` at path
ratio 0.88–0.92).

## Goal and scope

Round 2: **rotation from the gyro, translation from vision**.

- Correct gyro bias in the rotation prediction.
- Optional hard lock: when a frame has a bias-corrected gyro prediction, the
  registration solves translation only.

Out of scope: soft rotation priors, periodic vision correction of gyro drift,
event-rate/noise handling, ESVO2's `USE_IMU` backend (its IMU path reads
uninitialized bias memory on this machine; separate issue), mapping changes.

## Design

### Units

**`esvo2_core/include/esvo2_core/tools/gyro_bias.h`** (new, header-only, no
ROS dependency, like `gyro_prediction.h`):

- `GyroBiasEstimator(double window_s, double max_std, double max_gap = 0.02)`.
- `bool add(const GyroSample &s)`: feeds one raw sample (IMU frame, rad/s).
  Samples are grouped into consecutive, non-overlapping windows of
  `window_s`. A window is accepted when every axis' standard deviation is
  below `max_std` and no gap between consecutive samples exceeds `max_gap`.
  The first accepted window's mean becomes the bias; `add` returns true on
  that sample and ignores further samples. A rejected window is discarded and
  the next window starts at the next sample.
- `bool hasBias() const`, `Eigen::Vector3d bias() const`,
  `Eigen::Vector3d biasStd() const` (per-axis std of the accepted window).

### Bias source (tracking)

In order:

1. `GYRO_BIAS: [bx, by, bz]` (rad/s, IMU frame) present in the config: used
   from the first sample; no estimation runs.
2. Otherwise the startup estimate from `GyroBiasEstimator`, fed by the gyro
   callback until it has a bias. Logged once:
   `gyro bias estimated from <window> s still window: [..] rad/s (std [..])`.
3. Until a bias is known: gyro prediction runs uncorrected (round-1
   behaviour) and the rotation lock is off.

The bias is fixed for the session once known.

### Configuration (tracking YAML, all optional)

| Key | Default | Meaning |
|---|---|---|
| `IMU_ROTATION_LOCK` | `False` | Translation-only solve on frames with a bias-corrected gyro prediction. Needs `IMU_ROTATION_PREDICTION: True`; ignored with a warning otherwise. |
| `GYRO_BIAS` | unset | Fixed gyro bias `[bx, by, bz]`, rad/s, IMU frame. Skips estimation. |
| `GYRO_BIAS_WINDOW` | `2.0` | Still-window length, s. |
| `GYRO_STILL_MAX_STD` | `0.0035` | Stillness threshold per axis, rad/s (0.2 °/s; bag: still 0.0007, hand-held ~0.07). |

With none of these keys set, behaviour is identical to round 1. No existing
config sets them, so MVSEC/DSEC/etc. are unaffected.

### Per-frame flow (`esvo2_Tracking`)

- Gyro callback: unchanged raw buffering under `gyro_mutex_`; additionally
  feeds the estimator while it has no bias (source 2 only).
- `predictRotationWithGyro()` copies the buffer, subtracts the bias from the
  copies when known (so samples buffered before the estimate completed are
  corrected too), integrates as before, and now returns `bool` (prediction
  applied this frame).
- Lock decision:
  `lock = IMU_ROTATION_LOCK && biasKnown && predictionApplied && status == WORKING`.
  No prediction runs during INITIALIZATION, so no lock there either.
- After `rpSolver_.resetRegProblem()` and before `solve_analytical()`:
  `rpSolver_.setFixRotation(lock)`.

### Solver

- `RegProblemLM::setFixRotation(bool)`: stores a flag; when set, `df()`
  zeroes `fjac.leftCols(3)` (Cayley rotation columns). The LM step then has a
  zero rotation update; Eigen's LM scales zero-norm columns to 1, so the
  system stays well-posed.
- `RegProblemSolverLM::setFixRotation(bool)`: pass-through to the analytical
  problem.
- Analytical solver only (`RegProblemType: 1`, the EVK4 setting). With
  `RegProblemType: 0` and the lock enabled: one warning at startup, lock
  ignored.

### Logging

- Once at startup: bias source (configured / estimating) and whether the lock
  is active.
- In the existing periodic gyro-prediction statistics: locked frames vs total
  frames; a warning when more than 20% of WORKING frames fall back to 6-DoF.

## Testing and validation

### Unit tests (`esvo2_core/test/test_gyro_bias.cpp`, gtest)

- A still window (constant rate + small noise) is accepted and the bias equals
  its mean.
- A moving window is rejected; a following still window is accepted.
- A window containing a gap > 20 ms is rejected.
- Integrating a constant-rate stream minus its own bias with
  `gyroDeltaRotation` gives the identity rotation.

### Offline gates

Slide bag replayed at 0.25× with the EVK4 configs; `GYRO_BIAS` = mean of the
bag's still period (10.5–23 s): `[0.003425, -0.004210, -0.003853]` rad/s.

| # | Setup | Pass |
|---|---|---|
| G1 | `IMU_ROTATION_LOCK: True` + `GYRO_BIAS` | x ≤ −0.50 m; tracked yaw within ±3° of gyro-integrated (bias-corrected) yaw throughout |
| G2 | new keys unset | x between −0.15 and −0.30 m (current baseline −0.20 to −0.26 m) |
| G3 | MVSEC `indoor_flying1`, vision-only upenn configs, new keys unset | path ratio ≥ 0.8, 0 tracking resets, on each of 2 runs |
| G4 | startup estimation (no `GYRO_BIAS`), bag trimmed to start at 10.5 s | estimated bias within 0.001 rad/s of the configured value per axis |

### Reproducibility

- Move the slide bag to `/root/datasets/evk4/slide_lr.bag` (outside the repo).
- Add to `esvo2_core/scripts/`: the replay launch/runner and the evaluation
  script (tracked translation, and tracked yaw vs gyro-integrated yaw).

### Queued for the rig (not part of this round)

- Live 1 m slide and hallway walk with the lock on, rig held still 2 s at
  start.
- Event-rate cap / sensor noise filter tests for the remaining shortfall.

## Housekeeping in this branch

- Remove the temporary `DBG-TMP` instrumentation from `esvo2_Tracking.cpp`
  and `RegProblemLM.cpp`.
- `EVK4_STEREO_SETUP.md`: findings (ambiguity, gyro bias, event-rate
  saturation), the new keys, and the 2 s still start.
