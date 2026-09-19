# Live Stereo EVK4 + IMU Setup for ESVO2

This documents the work done to get ESVO2 running live on a stereo pair of
Prophesee EVK4 (IMX636) event cameras with an SBG IMU, since the upstream
repo only ships launch files for offline rosbags (DSEC/RPG) and a live
DVXplorer rig.

**Current status:** the full pipeline launches and runs (cameras, IMU,
time surfaces, mapping, tracking, visualization); the crashes, black-outs
and memory leaks found along the way (hot pixels, the event-rate cap, the
time-surface lock fix) are fixed, and so is the left/right camera swap
(see "Gotchas": "The camera sides were swapped"). Gyro rotation
prediction is calibrated and **enabled** (`IMU_ROTATION_PREDICTION:
True`): tracking's rotation follows the gyro at ratios 1.00-1.04 across
every speed band with real motion (was 0.04-0.45 with it off). Beside a
flat wall, though, tracking traded sideways translation for yaw the rig
never made: at one depth, with mostly vertical edges and this rig's
narrow 41° horizontal FOV, a lateral slide and a yaw look almost the same
in the image, so the solver explained a recorded 1 m slide as 10-24° of
yaw (bias-corrected gyro: -0.4°) and only 0.2-0.26 m of translation. Fixed
by `IMU_ROTATION_LOCK` (rotation from the bias-corrected gyro,
translation-only registration; see "Changes made"), now **enabled** in
`tracking_evk4_AA.yaml`. Measured offline on that same 1 m slide, replayed
at 0.25x: min_x went from -0.262 m (without the lock) to -0.527 m (with
it), and max yaw deviation from the gyro from 13.6° to 0.21°. The rest of
the shortfall (0.47 of the 1 m slide) is attributed to event-rate
saturation on this scene (the left camera sat at 3.6-3.95M ev/s through
the whole slide, against the 4M ev/s cap), not yet fixed. Translation
itself is still vision-only — the lock only replaces the *rotation*
estimate — and ESVO2's own IMU fusion path (`USE_IMU`) remains off. See
"Known limitations".

## Hardware

- 2x Prophesee EVK4, IMX636 sensor, 1280x720
- Serials:
  - `00051860`: physically **left** camera, sync master (cabled SYNC OUT)
  - `00051182`: physically **right** camera, sync slave (cabled SYNC IN)
  - "Left" means seen from behind the rig, facing where it looks. Until
    2026-09-18 these were swapped (see "Gotchas": "The camera sides were
    swapped")
- Hardware sync cable between the two (required, see "Gotchas")
- SBG Systems IMU (red unit, `MAIN`/`ANT` connectors), mounted centered
  between the two cameras on the rig, connected via USB (shows up as
  `/dev/ttyUSB0`, an FTDI USB-serial adapter)

## Calibration

### Stereo cameras

Intrinsics/extrinsics were calibrated using Prophesee's own
`metavision_calibration_pipeline` (blinking-chessboard pattern, displayed on
a monitor and alternated with a solid white frame, not a printed board).
Results:

- Baseline: 155.8 mm
- Focal length: ≈1647 px (left) / ≈1634 px (right), consistent with an
  ~8mm lens on the IMX636's 4.86µm pixels
- 23–30 intrinsics views per camera, 30 synced extrinsics views
- Output: `esvo2_core/calib/evk4_stereo/left.yaml` and `right.yaml`, in
  ESVO2's standard OpenCV pinhole/plumb_bob + rectification format

Rectification has been checked numerically (projection matrices are
consistent with the baseline) but **not yet visually** on live time
surfaces, so a rectification problem hasn't been ruled out as a cause of
the poor SGM results.

The calibration working files (raw detections, per-view errors, the
`metavision_calibration_pipeline` JSON config used) live outside this repo
in a scratch directory and were not preserved. The two YAML files are the
only durable output. If you need to redo this (new rig, different baseline,
lens change), the working method was:

1. Generate a chessboard pattern sized for your display's actual pixel
   pitch (so square size in meters is known exactly, not measured by eye).
2. Display it full-screen, alternating with a solid white frame (**not** a
   full photometric inverse, which floods every pixel with events so the
   detector can't tell pattern from background) at ~80ms per state. Faster
   flipping aliases with the monitor's rolling row-scan (a moving "wave");
   much slower (~300ms) makes the reconstructed squares fade between flips.
3. Run `metavision_calibration_pipeline -i <pipeline.json>` with
   `device-id` set to each camera's serial for intrinsics, then a synced
   step for extrinsics. Each step opens a live GUI window: hold at least
   30 well-distributed detections (varying position/angle across the full
   frame) before closing it.
4. Convert the tool's `camera-geometry.json` + `extrinsics.json` output to
   ESVO2's YAML format via `cv2.stereoRectify`, being careful about which
   physical side is "master" vs "left" in your rig (they need not match)
   and about `T_right_left`'s direction convention: `X_right = R·X_left + t`
   (see `RegProblemLM.cpp`'s comment on this).

### IMU-to-camera extrinsics: rotation calibrated, lever arm not

`T_b_c`'s rotation in both calib YAMLs was measured with
`esvo2_core/scripts/calibrate_imu_camera_rotation.py` (see "Changes made"
for the method and measured values); translation is still 0 (lever arm not
calibrated — the cameras and IMU are close together on this rig, so this
was judged lower priority than the rotation, which previously caused
runaway pose drift when IMU fusion was enabled, see "Known limitations").

## Changes made

### `prophesee_ros_wrapper` (sibling package, own git repo)

These changes are stored in this repo as
`evk4_drivers/prophesee_ros_wrapper.patch`, made against upstream commit
`59f2aca`. To apply them to a fresh clone, run
`git checkout 59f2aca && git apply /path/to/ESVO2/evk4_drivers/prophesee_ros_wrapper.patch`.

- **New:** `prophesee_ros_driver/src/prophesee_ros_stereo_publisher.cpp`
  (+ matching header): a node that opens **both** cameras of the synced
  pair from a single process and publishes each directly as
  `dvs_msgs::EventArray` on `/evk4_left/events` and `/evk4_right/events`
  (plus `camera_info` on `/evk4_<side>/camera_info`). See "Gotchas" for why
  this exists instead of two `prophesee_ros_publisher` instances.
- **New:** `prophesee_ros_driver/launch/stereo.launch`: launches the above
  with this rig's serial-to-side mapping.
- **Modified:** `prophesee_ros_driver/src/prophesee_ros_publisher.cpp` (+
  header): added a `serial_number` param so a single camera can be opened
  by serial instead of always grabbing "the first available" one.
- **Fixed:** `prophesee_ros_driver/CMakeLists.txt` had `curl` listed in
  `add_dependencies()` for `prophesee_ros_viewer`, which requires `curl` to
  be a CMake target (it's just a system library, already correctly in
  `target_link_libraries`). This broke `catkin_make`; removed the bad line.
- **Modified:** `package.xml`: added a `dvs_msgs` dependency for the new
  stereo publisher.
- **New:** hot pixel masking in the stereo publisher. The
  `left_masked_pixels` / `right_masked_pixels` params (lists of `[x, y]`)
  are written to the IMX636's hardware digital event mask (64 slots per
  camera) right after the cameras open, read back, and logged as
  `[left] masked pixel (x, y)`. `stereo.launch` sets this rig's three hot
  pixels (see "Gotchas").
- **New:** hardware event rate cap in the stereo publisher. The
  `event_rate_limit` param (events/s per camera, `0` disables it) sets the
  IMX636's event rate controller (ERC) and enables its event dropping,
  read back and logged as `[left] event rate capped at 4000000 ev/s`.
  `stereo.launch` exposes it as an arg, default 4000000, and
  `evk4_live_all.launch` passes it through.

### `prophesee_to_dvs` (sibling package, not version controlled)

- **Modified:** `src/prophesee_to_dvs.py`: parameterized `input_topic` /
  `output_topic` via private params instead of hardcoded topic names, and
  copies `width`/`height` into the output message (previously left at zero).
- **New:** `launch/stereo.launch`: two bridge instances, one per camera.
- **Not used in the live pipeline:** this Python bridge can't keep up with
  the EVK4's event rate (see "Gotchas"), so the stereo publisher emits
  `dvs_msgs` directly instead.

### `sbg_ros_driver` (new sibling package, cloned from upstream)

Cloned from `https://github.com/SBG-Systems/sbg_ros_driver.git` (now listed
in this repo's `dependencies.yaml`). Two apt packages were needed to build
it: `ros-noetic-rtcm-msgs` and `ros-noetic-nmea-msgs`.

Copies of the two new files are in this repo under
`evk4_drivers/sbg_ros_driver/` (upstream commit `140344e`). Copy them into
the same paths in the `sbg_ros_driver` clone.

- **New:** `config/sbg_device_evk4_rig.yaml`: copy of the shipped
  `sbg_device_uart_default.yaml` with the changes this unit needs (see
  "Gotchas"): `baudRate: 921600`, `use_enu: true`,
  `output.ros_standard: true`, `log_imu_short: 0`, `frame_id: "imu_link"`,
  and `portName` set to the stable
  `/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FT2AO6CH-if00-port0` (the
  adapter came back as `ttyUSB1` instead of `ttyUSB0` after a replug).
- **New:** `launch/sbg_evk4_rig.launch`: loads that config and starts
  `sbg_device`.
- **Device settings changed on the unit itself (not in any file):** the
  `IMU_DATA` (msg 3) and `EKF_QUAT` (msg 7) outputs on port A were raised
  from 25 Hz (mode 8) to 200 Hz (mode 1) and saved to the Ellipse2-N's
  flash, using a one-off program built against the driver's bundled
  sbgECom (`sbgEComCmdOutputGetConf` to read, `sbgEComCmdOutputSetConf`,
  read-back, then `sbgEComCmdSettingsAction(SBG_ECOM_SAVE_SETTINGS)`).
  Nothing else on the unit was touched. The program lived in a scratch
  directory and was not kept. `log_imu_data: 1` / `log_ekf_quat: 1` in the
  YAML only record this (see "Gotchas": the driver doesn't apply them).
  Measured afterwards: `/sbg/imu_data` 201.5 Hz, `/sbg/ekf_quat` 200.0 Hz,
  `/imu/data` 200.0 Hz, gravity 9.824 m/s² at rest.
- No source code in this package was modified; its fixed-point scaling bug
  (see "Gotchas") is worked around via config.

### This repo (`ESVO2`)

- **New:** `esvo2_core/calib/evk4_stereo/{left,right}.yaml`: the stereo
  calibration described above. `T_b_c`'s rotation is now calibrated (see
  below); translation is still 0.
- **New:** `esvo2_core/cfg/mapping/mapping_evk4_AA_mapping.yaml`: copy of
  `mapping_dvx_AA_mapping.yaml` with:
  - `USE_IMU: False` (see "Known limitations")
  - `bVisualizeGlobalPC: False` (unbounded growth, see "Gotchas")
  - `invDepth_min_range: 0.1`, `invDepth_max_range: 1.0` (was 0.05/0.33,
    i.e. accepted depth 1–10 m instead of 3–20 m, for close-range use at
    our 156 mm baseline)
  - `BM_max_disparity: 320` (was 150, to reach those nearer depths; must
    stay divisible by 16 for `cv::StereoSGBM`, and costs CPU)
- **New:** `esvo2_core/cfg/tracking/tracking_evk4_AA.yaml`: copy of the
  previously unused `tracking_online_AA.yaml` template with `USE_IMU: False`
  and `PATH_TO_SAVE_TRAJECTORY: /root/esvo2_output/`.
- **New:** `esvo2_core/launch/system/system_evk4_mapping.launch`: the live
  pipeline: image_representation (left/right) → mapping → tracking → rviz,
  wired to `/evk4_left|right/events`, `use_sim_time=false`. It overrides
  `generation_rate_hz` to 25 for both image_representation nodes (the
  shared cfg's 100 Hz is unreachable here; see "Gotchas"). No IMU remap is
  needed: the SBG driver publishes `/imu/data`, the absolute topic that
  `esvo2_Mapping`/`esvo2_Tracking` subscribe to.
- **Fixed:** `esvo2_core/src/core/BackendOptimization.cpp`: two defensive
  guards at the top of `sloveProblem()` (and one in `double2Vector()`)
  that skip the IMU backend optimization until there's enough history.
  Both prevented real SIGSEGV crashes (see "Gotchas").
- **Fixed:** `image_representation/src/ImageRepresentation.cpp`:
  - The event subscriber's `queue_size` was 0 (unbounded in roscpp). It's
    now bounded by a new private param `event_sub_queue_size` (default
    10000, ~1 s of backlog at the EVK4's ~9500 msg/s).
  - `AA_thread()`'s reverse event loop started at `ptr_e`, which is
    `end()` whenever every buffered event predates the sync time; it now
    walks `[begin, ptr_e)` with a `reverse_iterator`.
  - `createImageRepresentationAtTime()` held `data_mutex_` for the whole
    TS/AA/Sobel computation, starving `eventsCallback()` (see "Gotchas").
    It now copies the cycle's events into `vBatch_` and clears them from
    `vEvents_` under the mutex, releases it, and builds the images from
    `vBatch_`. `AA_thread()` reads `vBatch_` too. Same events, same
    images.
- **New:** `dependencies.yaml` entry for `sbg_ros_driver`.
- **New:** `esvo2_core/scripts/imu_restamp.py` (+ `imu_restamp_core.py`,
  unit tests in `esvo2_core/test/`): re-stamps `/sbg/imu_data` with the
  IMU's own clock, offset = min over 2 s of (receive − device time), and
  publishes `/imu/data_synced`. Started by `evk4_live_all.launch` with
  `imu:=true`. Measured over a 3-minute live check with the rig still:
  36022 published for 36022 input, stamp gaps median 5.000 ms (min 4.818,
  max 5.142 ms; 11 of 36021 gaps outside ±0.1 ms of 5 ms), receive − stamp
  7.95 / 7.70 / 7.52 ms across the three minutes (stable, always ≥ 0), no
  warnings.
- **New:** `esvo2_core/scripts/calibrate_imu_camera_rotation.py`
  (+ `imu_cam_calib_core.py`, unit tests): estimates `R_b_c` (rectified left
  camera → IMU, `p_imu = R_b_c · p_cam`) and `t_d` (camera time = IMU time +
  `t_d`) by aligning event-based camera angular velocity with the gyro.
  Procedure: camera driver + SBG driver + relay only; rig rotated by hand
  about all axes in front of a ≥3 m textured scene for 60 s, `_window:=0.02`.
  Two independent 60 s captures were run: run 1 gave `t_d = -0.005 s`, peak
  correlation 0.988, sharpness 0.070, 94% inliers, halves differ by 0.94° /
  4.0 ms, 4.11° from the nearest axis permutation, from 2065 camera samples
  and 1720 moving pairs (rms 0.054 rad/s); run 2 gave `t_d = -0.006 s`,
  correlation 0.985, sharpness 0.065, 93% inliers, halves differ by 0.38° /
  2.0 ms, 4.33° from the nearest permutation. The two runs agree to 0.29°
  and 1.0 ms. Run 1's rotation was written into `T_b_c` in both calib files
  (translation still 0): camera x ≈ −IMU y, camera y ≈ −IMU z (camera y
  points down, IMU z points up), camera z ≈ +IMU x, with the ~4.2° residual
  from the rig's real mounting tilt (not calibration error). At the 20 ms
  window, 710 of ~2775 frame pairs (26%) were dropped under real hand
  motion (0 dropped with the rig still); a 10 ms window dropped 359 of
  ~1230 (29%) and is not used.
- **New:** gyro rotation prediction in tracking
  (`esvo2_core/include/esvo2_core/tools/gyro_prediction.h`, gtest
  `test_gyro_prediction`, run with
  `cd /root/catkin_ws/build && make run_tests_esvo2_core_gtest_test_gyro_prediction`
  — `catkin_make --pkg esvo2_core run_tests_...` no-ops in this workspace).
  Params in `tracking_evk4_AA.yaml`: `IMU_ROTATION_PREDICTION: True` and
  `IMU_TIME_OFFSET: -0.005`; tracking topic `imu_prediction` remapped to
  `/imu/data_synced`. Each frame starts from
  `R_world_prev · R_b_cᵀ · R_imu · R_b_c`, translation still from vision;
  ESVO2's original `USE_IMU` path stays off. Validation
  (`esvo2_core/scripts/validate_rotation_tracking.py`), prediction OFF vs
  ON:
  - Slow-turn holds, coverage = pose / gyro, error = |coverage − 100%|:
    prediction OFF, gyro 71.3° / 71.0° vs pose 11.3° / 11.8° — coverage
    16% / 17% (84% / 83% error); prediction ON, gyro 77.4° / 77.8° vs
    pose 84.1° / 84.6° — coverage 109% / 109% (9% error each); one later
    ON hold, gyro 73.3° vs pose 89.5° — coverage 122% (22% error, see
    "Known limitations").
  - Mixed motion, pose/gyro rotation-speed ratio by gyro band (OFF → ON):
    0.0–0.1 rad/s 2.06 → 1.56; 0.1–0.5 0.45 → 1.04; 0.5–1.2 0.12 → 1.01;
    >1.2 0.04 → 1.00.
  - Share of map points in the darkest 5% of the negative time surface:
    7.1% → 13.3% on the slow turn (10.5% on the mixed run); mean valid
    residual 235 → 229; rot_update 0.161° → 0.138° per solve; cur_vs_ref
    0.23° → 0.44°.
  - Prediction activity: 125–126 predictions per 5 s, 0 skipped, no
    skip-rate warnings; 0 re-inits (baseline 2) and 0 node deaths over the
    run; mapping ~307% CPU / 5.0 GB, tracking ~87% / 3.0 GB, ~2.3 GB free.
- **New:** gyro bias estimation and rotation-lock, translation-only
  registration in tracking
  (`esvo2_core/include/esvo2_core/tools/gyro_bias.h`, gtest
  `test_gyro_bias`, run the same way as `test_gyro_prediction` above). New
  `tracking_evk4_AA.yaml` keys: `IMU_ROTATION_LOCK` (rotation comes from the
  bias-corrected gyro, and registration only solves for translation),
  `GYRO_BIAS` (skip estimation, use this constant bias instead — needed for
  replays that don't start still), `GYRO_BIAS_WINDOW` (seconds of stillness
  needed to accept a bias estimate) and `GYRO_STILL_MAX_STD` (per-axis
  std-dev threshold, rad/s, for a window to count as "still"). Enabled in
  the EVK4 tracking cfg; see "Current status" and "Gotchas" for the
  measured effect.
- **New:** replay/evaluation tooling for regression-testing tracking
  against recorded bags instead of the live rig:
  `esvo2_core/scripts/replay_eval.sh` (plays a bag through a launch file,
  records `/esvo2_tracking/pose_pub`, reports SGM init and tracking reset
  counts), `esvo2_core/scripts/eval_slide.py` (checks a recorded lateral
  slide: tracked x displacement and yaw deviation from the gyro) and
  `esvo2_core/scripts/eval_traj.py` (compares a tracked trajectory against
  a ground-truth file, e.g. MVSEC, reporting path-length ratio and ATE).
  `system_evk4_mapping.launch` gained matching args: `use_sim_time` (true
  for bag replay), `mapping_cfg` / `tracking_cfg` (override the default
  cfg files, e.g. to test a config with keys added/removed), `ts_rate`
  (time-surface `generation_rate_hz`, still 25 by default) and `gui`
  (already existed; `false` skips rqt/rviz for headless replay runs).

> **Local debug edits (not part of this work):** someone's in-progress
> debug edits to `image_representation/src/ImageRepresentation.cpp`
> (commented-out `r.sleep()`, `ROS_WARN`/`ROS_INFO` prints in the event
> callback and publish path) were **stashed** as `stash@{0}` so the fixes
> above could be built on clean source. Restore with `git stash pop`, but
> it will refuse while the file has the fixes applied, and rebuilding with
> those edits compiles in a busy loop (no `r.sleep()`) plus log output at
> ~9500 Hz. `image_representation/launch/image_representation.launch`
> also has local edits; it was left untouched.

## Gotchas discovered along the way

- **The camera sides were swapped (fixed 2026-09-18).** `stereo.launch`
  published serial `00051182` as left, but covering each lens showed it is
  physically on the right (covering the physically left lens silenced
  `/evk4_right/events`: 71k events in 5 s vs ~850k normally). The
  calibration had captured the real geometry, so `T_right_left` came out
  with t_x = +0.156 m and `P_right[0,3]` = +263 (every other dataset in
  `calib/` has both negative). ESVO2 takes the baseline as a length
  (`CameraSystem::computeBaseline()` uses `.norm()`), assumes
  `x_right = x_left - disparity`, and only searches positive disparities,
  so almost no stereo match survived. That left the inverse depth map and
  the left reprojected map nearly empty, and the local map collapsed.
  Fix: serials swapped in `stereo.launch` and `evk4_live_all.launch`, the
  hot-pixel masks moved with their sensors, and both calib YAMLs
  regenerated: intrinsics swapped between files, `T_right_left` inverted,
  rectification redone with the settings that reproduce the old files
  exactly (`cv2.stereoRectify`, `CALIB_ZERO_DISPARITY`, `alpha=0`).
  `T_b_c` is unchanged: the rectified frame's orientation is identical
  (0.000 deg difference). Sanity check for any future calibration:
  `P_right[0,3]` must be negative. Result, live and vision-only (IMU
  driver off): rectified events now match at positive disparity (+68 to
  +78 px, NCC up to 0.69); SGM init returned 670 to 905 points (before:
  mean 28 to 231 over 11 runs); in a 30 s hand-held run the local map held
  1,900 to 5,100 points, tracking ran at 20 to 22 poses/s, and there was
  no reset for 2.5+ min. Pose accuracy/drift not yet measured. Notes below written before this fix
  that say "left"/"right" mean the old topics: old left = `00051182`, old
  right = `00051860`.

- **USB2 vs USB3**: both EVK4s must be on genuine USB3 SuperSpeed ports.
  On USB2 the event rate overruns the link and the driver crashes with
  `Evt3 protocol violation` / `NonMonotonicTimeHigh` errors. Check via
  `cat /sys/bus/usb/devices/<dev>/speed` (want `5000` or higher, not `480`).
- **Don't run both EVK4s and the IMU through one unpowered hub.** With
  everything on a single bus-powered hub, the rig dropped off the bus
  twice in one session while being moved by hand. It starts as one camera
  stuttering (working for a second, then freezing), then that camera
  disconnects (`LIBUSB_TRANSFER_NO_DEVICE` in the driver log, `USB
  disconnect` in `dmesg`), and the whole hub follows a few seconds later.
  It may come back on a USB2 port. Plug each EVK4 into its own USB3 port
  (or use a powered USB3 hub) and strain-relieve the cables. A stuttering
  camera also feeds stereo matching a stale time surface, so rule this out
  before tuning mapping.
- **USB-C docks can silently drop to USB2.** This laptop (Dell Precision
  5560) has only USB-C ports. Plugging a camera into a StarTech DKT30CHV
  multiport dock gave 480 Mbps for everything behind it, dock hub and
  its USB3 Ethernet chip included. That's typical when a dock's USB-C
  link carries video (DisplayPort alternate mode), leaving only USB2 lanes
  for data. Connect each EVK4 to its own laptop USB-C port with a plain
  cable or a simple USB-C-to-USB-A 3.x adapter. To see what a camera is
  really behind, check the `manufacturer`/`product` files of its parent
  device under `/sys/bus/usb/devices/`.
- **Two processes can't each open one camera of this pair.** Opening both
  via `Camera::from_serial()` works within a single process, but two
  separate `prophesee_ros_publisher` processes (the DVXplorer-style
  pattern) reliably fail with "camera not found" for both, even staggered
  several seconds apart. Likely the hardware sync relationship is
  process-exclusive at the HAL level. Fixed by the single-process
  `prophesee_ros_stereo_publisher` node.
- **The Python bridge is too slow.** `prophesee_to_dvs.py` converting
  events in a pure-Python per-event loop can't sustain ~9500 msg/s:
  observed throughput dropped to ~210 msg/s (98% loss).
- **Hot pixels flood both cameras and black out the time surfaces.** Three
  stuck pixels, all firing OFF events only, produced ~97% of all events:
  left (307, 456) and (502, 34) at ~4.6M events/s each, right (596, 504)
  at ~9M events/s, against ~0.2M events/s from the rest of the frame. The
  same pixels showed up in repeated runs. The time surface nodes can't
  ingest ~9.5M events/s, so they fall behind; since each TS is drawn at
  `ros::Time::now()` with a 20 ms decay, a node more than ~100 ms behind
  renders fully black. Symptom: the left image goes black a few seconds
  after start (its node does the most work), the right one briefly freezes
  and flashes. This is very likely what "Left time surface stalls under
  CPU load" below was. Fixed by masking the pixels in the sensor (see
  "Changes made"). Measured after masking: no pixel above 1000 events/s,
  0.3 to 1.1M events/s per camera with the rig still, and both time
  surfaces at 21 to 25 Hz under hand-held motion with no black-outs. To
  find new hot pixels, count events per pixel over a few seconds from
  `/evk4_<side>/events` and look for pixels above ~1000 events/s. Don't
  record a rosbag for this: at ~10M events/s, two seconds is ~400 MB and
  loading it into memory got the desktop session OOM-killed.
- **Fast motion still blacks out the time surfaces without a rate cap.**
  With the hot pixels masked, hand-held shaking produces 5 to 10M real
  events/s per camera, and the same falling-behind effect turns the images
  black (left first). The time surface nodes kept up at 2 to 5M events/s
  and fell behind at ~10M. Fixed with the ERC cap (`event_rate_limit`,
  default 4M events/s; see "Changes made"): above the cap the sensor drops
  events itself. Measured while shaking the rig hard: the right camera sat
  at the cap (3.99M events/s), both time surfaces stayed at 25 Hz every
  second, and the brightest TS pixel stayed at 96 to 177 (it had dropped
  to 0 before). Raise the cap only if the time surface nodes get faster.
  When checking it, pass params through `stereo.launch`, not `rosrun`:
  `rosrun _left_masked_pixels:=...` delivers the list as a string, so the
  driver ignores it (it logs a warning) and the hot pixels come back.
- **The left camera records fewer events than the right on the same
  scene.** Under identical motion, left sent 0.15 to 0.5M events/s while
  right sent 0.5 to 1.8M, and the left time surface was 1.6 to 4.6% lit vs
  5 to 22% for the right. Not investigated yet: check focus and aperture
  on both lenses, then biases. It matters for stereo matching, which needs
  both cameras to see the same edges.
- **`metavision_calibration_pipeline`'s synced-cameras JSON path has an
  unguarded field read** (`device_node["settings-file"]` with no
  `.contains()` check, unlike the single-camera path). Omitting the key
  crashes with a `type_error`; it must be present and point to a real
  (even if no-op) settings JSON file.
- **This SBG unit isn't at its factory-default 115200 baud.** With the
  wrong rate, `sbg_device` fails with `Unable to get the device Info :
  SBG_TIME_OUT` even though the port isn't silent. The real rate (921600)
  was found by sniffing `/dev/ttyUSB0` at candidate rates for the sbgECom
  `0xFF 0x5A` frame sync marker at a consistent byte spacing.
- **`sbg_ros_driver` only publishes standard `sensor_msgs/Imu` in ENU
  mode** (`output.use_enu: true`). In NED (the shipped default) it logs
  `ROS standard message are disabled` and never advertises `/imu/data`.
- **`sbg_ros_driver` has a unit-conversion bug** in the "IMU short" path.
  With both `log_imu_data` and `log_imu_short` enabled (the default), the
  standard IMU message is built from `SbgLogImuShort`, whose
  `deltaVelocity`/`deltaAngle` are fixed-point integers (1048576 LSB per
  m/s², 67108864 LSB per rad/s), but
  `createRosImuMessage(const SbgImuShort&, ...)` never divides by those
  scales. `linear_acceleration.z` read ≈10<sup>7</sup> instead of ≈9.8.
  Worked around with `log_imu_short: 0`, which falls back to the
  float-typed `SbgImuData` log.
- **The SBG YAML's output rates are not applied to the device.** With
  `confWithRos: false` the driver only reads; the unit keeps the output
  modes stored in its flash, so editing `log_imu_data` alone changed
  nothing (still 25 Hz). `confWithRos: true` would apply them, but it also
  writes every other YAML group to the unit (motion profile, alignment,
  lever arms, magnetometer, GNSS, odometer) and saves, overwriting its
  current settings with mostly stock defaults, so the rates were set
  directly instead (see "Changes made").
- **`/imu/data` only publishes when an IMU sample and an EKF quaternion
  have the same timestamp** (`MessagePublisher::processRosImuMessage()`).
  With `IMU_DATA` at 200 Hz but `EKF_QUAT` at 25 Hz, `/sbg/imu_data` ran at
  200 Hz while `/imu/data` stayed at 25 Hz. Keep both outputs at the same
  rate. ESVO2 only reads `linear_acceleration`, `angular_velocity` and the
  header stamp from it, not the orientation.
- **`/imu/data` timestamps are bunched.** `time_reference: "ros"` stamps
  each message when the serial read returns, and the link delivers
  samples in batches: 65% of consecutive `header.stamp` gaps were under
  1 ms (median 0.04 ms, p90 15 ms, max 20 ms), while the IMU's own clock
  (`/sbg/imu_data` `time_stamp`) showed exactly 5.00 ms every time. Fine
  for display, not for preintegration (see "Known limitations").
- **`bVisualizeGlobalPC: True` grows memory without bound.**
  `esvo2_Mapping.cpp` reserves `pc_global_` for 5M points and only ever
  appends to it, re-serializing and re-publishing it to rviz on every
  refresh. It's visualization-only (not used by tracking or depth
  estimation), so it's disabled in the EVK4 mapping config.
- **ESVO2's IMU backend (`BackendOptimization.cpp`) segfaults under real
  motion with `USE_IMU: True`.** Two separate crashes, both caught with
  gdb backtraces:
  1. `double2Vector()` indexes the shared depth-point deque with
     `size() - WINDOW_SIZE - 1 + i`. `size()` is unsigned, so when the
     deque holds fewer than `WINDOW_SIZE + 1` entries the index underflows
     to a huge value. The deque can shrink mid-optimization because the
     mapping thread `pop_front()`s it with no lock while Ceres solves.
  2. `solveGyroscopeBias()` dereferences `pre_integrations[0..WINDOW_SIZE-1]`
     unconditionally, but those pointers are NULL until `slideWindow()` has
     run `WINDOW_SIZE` times (`frame_count` counts up to it).

  Fixed with guards that skip the optimization until enough history exists.
  The underlying unsynchronized deque access is still there; the guards
  only stop it from crashing.
- **`image_representation` leaks memory through an unbounded subscriber
  queue.** After ~3.5 days running, `image_representation_left` was at
  11.1 GB RSS and `_right` at 2.2 GB, with the container near OOM. The
  events subscriber used `queue_size` 0 (unbounded), and
  `createImageRepresentationAtTime()` holds `data_mutex_` for the whole
  TS/AA/Sobel computation, which `eventsCallback()` also needs. Any time a
  cycle outlasts the event stream, batches pile up without limit. Left
  does strictly more per cycle (an extra `AA_thread`, Sobel gradients, five
  publishers vs one), so it falls behind more. Now bounded (see "Changes
  made"). Under heavy load the node drops old event batches rather than
  grow.
- **Left time surface stalls under CPU load.** With the shared cfg's
  `generation_rate_hz: 100`, both nodes only reached ~20 Hz; the left one
  (227% CPU vs right's 133%) intermittently stopped publishing entirely
  while right stayed steady. Symptom in rqt: "the right image moves, the
  left doesn't". The launch file now targets 25 Hz.
- **The left time surface flashed black for 1 to 5 frames under load**
  (even with hot pixels masked and the rate capped). Per-frame monitoring
  showed left events reaching ROS within 3 ms while the left node drew
  from events 100+ ms old. Temporary instrumentation in the node found the
  cause: `createImageRepresentationAtTime()` held `data_mutex_` for its
  whole cycle (25 to 52 ms on the left node, against a 40 ms period at
  25 Hz), and `eventsCallback()` needs that mutex for every message. The
  callback was starved, event messages were already 145 to 170 ms old
  when it got them, and each black flash matched a second with cycles
  starting on events over 70 ms old. The right node does less per cycle
  and never got there. Fixed by holding the mutex only to take the
  cycle's events out (see "Changes made"). Measured with the left camera
  at the 4M events/s cap for 28 of 30 s: no black frames on either side,
  worst callback lock wait 16.8 ms (was 44 ms), worst message age at the
  callback 33 ms (was 170 ms).
- **roslaunch rejects `--` inside XML comments** (`not well-formed
  (invalid token)`), and a Python `xml.etree` well-formedness check does
  not catch it. Validate launch files with `roslaunch --nodes <pkg> <file>`.
- **`python3 -m unittest test/...` fails with `ModuleNotFoundError`** for
  this repo's Python unit tests: Python resolves `test.` against its own
  stdlib `test` package, not this repo's `test/` directory. Run the file
  directly instead: `python3 test/<file>.py -v`.
- **`catkin_make --pkg esvo2_core run_tests_esvo2_core_gtest_...` no-ops**
  in this workspace (returns success without running anything). Build and
  run the gtest binary directly:
  `cd /root/catkin_ws/build && make run_tests_esvo2_core_gtest_test_gyro_prediction`.
  `catkin_test_results` then double-counts results (it reported 10 for 6
  actual tests); read the gtest binary's own output instead.
- **Tracking traded sideways translation for yaw.** Beside a flat wall at one
  depth (mostly vertical edges, 41° horizontal FOV), a lateral slide and a yaw
  give almost the same image motion. On a recorded 1 m slide the solver
  reported 10–24° of yaw the rig never made (bias-corrected gyro: −0.4°) and
  only 0.2–0.26 m of translation. Fixed by `IMU_ROTATION_LOCK` (rotation from
  the bias-corrected gyro, translation-only registration): replayed at 0.25x,
  min_x went from −0.262 m (without the lock) to −0.527 m (with it), and max
  yaw deviation from the gyro from 13.6° to 0.21°.
- **The SBG gyro has ~0.004 rad/s (0.23 °/s) bias per axis,** which the
  round-1 prediction integrated as rotation. Tracking now removes it
  (`GYRO_BIAS`, or estimated from the first 2 s still window). **Hold the rig
  still for 2 s after starting**; the log prints `Gyro bias estimated from ...`.
  On the slide bag's still period the online estimator gave [0.00338,
  -0.00419, -0.00376] rad/s against [0.003425, -0.004210, -0.003853] rad/s
  measured offline — within 0.001 rad/s per axis.
- **The cameras saturate the 4 M ev/s rate cap during motion** (left sat at
  3.6–3.95 M ev/s through the whole slide; ~0.5 M ev/s noise floor at rest).
  Time surfaces come out faint and noisy. Not fixed yet: next rig session,
  test a higher cap and the sensor's noise filter.
- **ESVO2's IMU mode (`USE_IMU: True`) is broken on this machine, upstream
  included:** on MVSEC `indoor_flying1` mapping prints an accelerometer bias of
  ~6.9e-310 (uninitialized memory) and crashes (`std::length_error`). Vision-
  only mode tracks MVSEC, but not deterministically: 5 back-to-back runs of
  the identical unchanged config (`USE_IMU: False`, no `IMU_ROTATION_*`/
  `GYRO_BIAS*` keys involved) gave path-length ratio 0.76–0.95 (0.85, 0.95,
  0.76, 0.81, 0.82) with tracking resets 32, 0, 31, 0, 0 — 2 of the 5 runs
  reset repeatedly and landed at the low end of the ratio range, the other
  3 didn't reset at all. ESVO2 is non-deterministic run to run even with a
  fixed bag, rate and config.
- **`rosparam load` doesn't clear keys missing from the new file.** Because a
  `roscore` stays up across separate `roslaunch` invocations (e.g. between
  replay runs), a key set by one config (say `IMU_ROTATION_LOCK: True` from a
  gyro-lock run) stays on the parameter server even after launching a config
  that omits it, silently changing the next run's behavior. Symptom: a replay
  meant to test the "keys removed" / default path instead reproduces the
  locked results almost exactly. `scripts/replay_eval.sh` now deletes the
  `/esvo2_Mapping`, `/esvo2_Tracking`, `/image_representation_left` and
  `/image_representation_right` parameter namespaces before every run it
  launches, so this only bites a manual `roslaunch` on a long-lived
  `roscore` — clear those same namespaces first (e.g.
  `rosparam delete /esvo2_Tracking`) or restart `roscore` between configs
  that add/remove keys.
- **Every replay ends with `terminate called without an active exception`
  or `std::length_error` / `vector::_M_range_insert`** when the nodes are
  killed on `SIGINT` at the end of a replay (seen in every log this branch
  produced, including the pre-existing baseline). Results recorded before
  shutdown are unaffected; not investigated.

## Known limitations / next steps

- **Pose estimation doesn't stay up.** Mapping must get at least
  `INIT_SGM_DP_NUM_THRESHOLD` (500) SGM depth points to initialize. With
  the DVXplorer depth/disparity ranges it got 30–95 per attempt; after
  widening them (see "Changes made") it initializes intermittently: 28 of
  375 attempts in a 10-minute hand-held test, best 1726 points. But it
  never stays initialized. Once `WORKING`, mapping's local map comes only
  from event block matching, and a live count of every published map
  shows it collapsing within a few cycles (3180 → 658 → 344 → 12, or
  4690 → 828 → 98 points). Tracking sets the system back to
  `INITIALIZATION` whenever the map has fewer than `BATCH_SIZE` (300)
  points (`RegProblemSolverLM.cpp:47`, `esvo2_Tracking.cpp:183-186`), so
  it loops between init and reset every 1–2 s. Poses produced in those
  short windows still drift: one 259 s trajectory spanned x −10.7…0.7 m
  and z −4.9…21.2 m. Suspects for the low block-matching yield, all
  inherited unscaled from the 640x480 DVXplorer config: `large_scale:
  True` (fusion uses only 1 of every 4 frames, `esvo2_Mapping.cpp:452-459`),
  `maxNumFusionFrames: 5` (the cfg notes the original ESVO value was 40),
  patch sizes 15x7 / 5x31, and `BM_ZNCC_Threshold: 0.2`. Rectification
  also hasn't been visually verified. This was measured before the camera
  side swap was fixed (see "Gotchas"); since then the local map has held
  1,900–5,100 points with no reset for 2.5+ min in a 30 s hand-held test,
  and the EVK4/MVSEC regression replays used for `IMU_ROTATION_LOCK`
  (see "Changes made", "Current status") ran with 0 tracking resets. Part
  of what looked like drift here was the solver trading translation for
  yaw (now fixed by `IMU_ROTATION_LOCK`, see "Gotchas"); whether the
  block-matching suspects above still apply hasn't been re-checked.
- **Gyro rotation prediction overshoots real rotation when the rig is
  nearly stationary.** In the mixed-motion validation, the pose/gyro
  rotation-speed ratio with prediction on is 1.00–1.04 in every band where
  the rig is actually moving (>0.1 rad/s), but 1.56 in the 0.0–0.1 rad/s
  band, where real motion (~0.05 rad/s) sits below the pre-existing
  ~0.08 rad/s pose noise floor — the predictor amplifies that noise rather
  than tracking real rotation. This accumulated stationary drift is what
  pushed one slow-turn hold (prediction ON) to 22% error, above the plan's
  20% criterion; the other two evaluated ON holds were 9% each (a further
  near-still hold was excluded by the validation script's <5° rule).
  This is a known limitation, not a pass: it is not fixed by better
  calibration, only by a motion/stillness gate on the predictor.
- **IMU fusion is disabled** (`USE_IMU: False` in both cfgs). Gyro-based
  rotation prediction is on in tracking (see "Changes made"), with
  `IMU_ROTATION_LOCK` now also replacing registration's rotation solve
  outright and gyro bias removed (`GYRO_BIAS`, or estimated from the first
  still window; see "Changes made", "Gotchas"). Translation still comes
  only from vision, and mapping's IMU backend (`BackendOptimization.cpp`)
  stays off. With `USE_IMU: True` and `T_b_c`
  at identity, tracking previously produced poses that diverged to hundreds
  of meters within seconds of hand-held motion; with it disabled, mapping
  doesn't subscribe to `/imu/data` but still calls `getIMUInterval()` every
  cycle, which is where the repeated `not receive imu data` error comes
  from (harmless). Before re-enabling `USE_IMU`:
  - **Lever arm not calibrated.** `T_b_c`'s rotation is now measured (see
    "Changes made"), but its translation is still 0.
  - **Extrinsic convention mismatch.** This calibration measured `R_b_c`
    under `p_imu = R_b_c * p_cam`, and the rotation predictor applies it as
    `R_b_c^T * R_imu * R_b_c`. ESVO2's own IMU fusion path composes the
    extrinsic the other way round, as `R_b_c * q * R_b_c^T` — the opposite
    convention. Enabling `USE_IMU` on the strength of "the rotation is now
    calibrated" would apply it backwards; the extrinsic must be re-derived
    (or transposed and verified) for the fusion path before relying on it.
  - **IMU sample period:** `esvo2_Tracking::refImuCallback()` uses the real
    stamp difference; only the very first sample gets `0.001` s. (An earlier
    version of this note wrongly said it assumed 1000 Hz.) With the bunched
    `/imu/data` stamps those differences were wrong per sample; the rotation
    predictor uses `/imu/data_synced` instead.
  - Rate is done: `/imu/data` now runs at 200 Hz (was 25 Hz).
- **`esvo2_Mapping` and `esvo2_Tracking` use a lot of memory, but it's
  bounded, not a leak.** Each keeps a history of `TS_HISTORY_LENGTH` (100)
  time-surface observations, and each observation stores 6–10
  `Eigen::MatrixXd` (TS left/right, AA map, negative TS and gradients;
  see `TimeSurfaceObservation.h`). At 1280x720 that's 44–74 MB per entry,
  so ~4.4–7.4 GB for mapping. Measured: mapping reaches ~5 GB within 30 s
  of receiving data and then fluctuates around ~5.5 GB (it also sat at
  4.9 GB after a 3.5-day run). Tracking holds steady at ~3.2 GB. The
  history size was chosen for DVXplorer's 640x480 sensor, which has 4x
  fewer pixels. Lowering `TS_HISTORY_LENGTH` in the mapping/tracking cfgs
  shrinks this roughly proportionally, but hasn't been tested for its
  effect on tracking quality. (Both nodes show only ~150–180 MB before
  the first data arrives, which is where the earlier "growth" impression
  came from.)
- **The `image_representation` leak fix has only a 10-minute test behind
  it.** Under hand-held load the right node settled at ~370–390 MB and the
  left oscillated between ~605 and ~775 MB with no upward trend. TS rates
  held at ~21 Hz (left) and ~23 Hz (right), with no stalls or crashes. The
  original leak averaged only ~21 MB per 10 minutes over 3.5 days, so a
  multi-hour run is still needed to call it fixed.
- prophesee_to_dvs's launch/script aren't part of the live path and could
  be removed if confirmed unneeded.

---

## How to launch everything

All of this assumes the workspace is already built and sourced:

```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

### One command

```bash
roslaunch esvo2_core evk4_live_all.launch
```

This starts the camera driver, the SBG IMU driver, and the pipeline
(time surfaces, mapping, tracking, rqt, rviz). Options: `imu:=false` if
the IMU isn't plugged in, `gui:=false` to run without rqt/rviz,
`event_rate_limit:=<events/s>` to change the per-camera event rate cap
(default 4000000, `0` disables it). All output
lands in one terminal, and a camera that fails to open is only reported
near the top of it, so use the three-terminal way below when debugging
hardware. Ctrl+C stops everything.

### Three terminals (easier to debug)

Run in order.

**Terminal 1: camera driver** (opens both EVK4s, publishes events):

```bash
roslaunch prophesee_ros_driver stereo.launch
```

Wait for it to report the camera geometry (1280x720) for both `left` and
`right` before continuing. If you see "camera not found" for both, check
they're on USB3 ports and the sync cable is connected (see "Gotchas").

Sanity check in another terminal:
```bash
rostopic hz /evk4_left/events
rostopic hz /evk4_right/events
```
Both should show ~9000+ msg/s.

**Terminal 2: SBG IMU:**

```bash
roslaunch sbg_driver sbg_evk4_rig.launch
```

Sanity check:
```bash
rostopic echo -n 1 /imu/data
```
The magnitude of `linear_acceleration` should be close to 9.8 (gravity)
when the rig is still. If `/imu/data` never appears, or you see
`SBG_TIME_OUT` in this terminal, re-check the baud rate (see "Gotchas").
Mapping never uses the IMU. Tracking has two separate IMU flags: its own
fusion path (`USE_IMU`) is off, but gyro rotation prediction
(`IMU_ROTATION_PREDICTION: True`, see "Changes made") is on and needs
`/imu/data_synced`, published by `imu_restamp.py` — not started by this
three-terminal flow (it's started by `evk4_live_all.launch`'s `imu:=true`,
or run it manually with `rosrun esvo2_core imu_restamp.py`). Without that
topic, prediction is inactive in this manual flow: tracking still runs but
with no rotation prior, and logs a skip-rate warning every 5 s. Running
this terminal at least keeps `/imu/data` available for the sanity check
above.

**Terminal 3: mapping, tracking, and visualization:**

```bash
roslaunch esvo2_core system_evk4_mapping.launch
```

This starts:
- `image_representation_left` / `_right`: build time surfaces from events
- `esvo2_Mapping`: stereo depth estimation (visual-only)
- `esvo2_Tracking`: pose tracking (publishes on `/esvo2_tracking/pose_pub`
  once mapping has initialized)
- `rqt_gui` and `rviz` for visualization

Point the rig at a textured scene roughly 1–2 m away and keep it moving.
Event cameras produce no output from a static view, so mapping/tracking
sit idle until there's motion. Watch mapping's log for
`Initialization (SGM) returns N points`; tracking only starts once N
reaches 500.

If `rqt_gui`/`rviz` fail with `Authorization required, but no
authorization protocol specified`, the X11 connection was stale. It
cleared up after the session restarted; relaunching them with
`DISPLAY=:1` worked.

To stop: `Ctrl+C` in each terminal (mapping/tracking are `required="true"`,
so killing either one shuts down the rest of that launch file).

### Replaying a bag

To regression-test tracking without the rig, replay a recorded bag through
`system_evk4_mapping.launch` with `scripts/replay_eval.sh` instead of live
drivers. Example (the slide-tracking check from "Changes made" /
"Gotchas: Tracking traded sideways translation for yaw"):

```bash
source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash
O=/root/datasets/evk4/replays; C=$(rospack find esvo2_core)
PLAYRATE=0.25 $C/scripts/replay_eval.sh $O/g1_lock.bag /root/datasets/evk4/slide_lr.bag \
  "/evk4_left/events /evk4_right/events /imu/data_synced /imu/data" \
  $C/launch/system/system_evk4_mapping.launch use_sim_time:=true gui:=false tracking_cfg:=$O/tracking_g1.yaml
python3 $C/scripts/eval_slide.py $O/g1_lock.bag /root/datasets/evk4/slide_lr.bag $C/calib/evk4_stereo 0.003425,-0.004210,-0.003853
```

`use_sim_time:=true` makes the pipeline follow the bag's clock instead of
the wall clock, `gui:=false` skips rqt/rviz, and `PLAYRATE` (env var, not a
launch arg) slows playback so the cameras' real 4M ev/s cap doesn't clip a
faster-than-real-time replay. `tracking_g1.yaml` here is
`tracking_evk4_AA.yaml` plus `GYRO_BIAS: [0.003425, -0.004210, -0.003853]`:
`GYRO_BIAS` is needed because this bag doesn't start with the rig still
for `GYRO_BIAS_WINDOW` seconds, so the estimator would never accept a
window and rotation prediction would run bias-uncorrected. `eval_slide.py`
takes the same gyro bias (for reporting the pose/gyro yaw comparison) and
prints tracked x-displacement over time plus the max yaw deviation from
the gyro. Swap the launch file, topics and eval script to replay other
scenarios, e.g. `eval_traj.py` against a ground-truth file for MVSEC bags
(`system_upenn.launch`, used for the `USE_IMU: True` regression check in
"Gotchas").

### Rebuilding after changes

```bash
cd ~/catkin_ws
catkin_make --pkg prophesee_ros_driver   # stereo publisher / driver
catkin_make --pkg sbg_driver             # SBG driver
catkin_make --pkg esvo2_core             # mapping/tracking, BackendOptimization
catkin_make --pkg image_representation   # time surface node
```

Stop the running pipeline before rebuilding a node's binary, and check the
stash note above before rebuilding `image_representation`.
