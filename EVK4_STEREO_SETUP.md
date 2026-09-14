# Live Stereo EVK4 + IMU Setup for ESVO2

This documents the work done to get ESVO2 running live on a stereo pair of
Prophesee EVK4 (IMX636) event cameras with an SBG IMU, since the upstream
repo only ships launch files for offline rosbags (DSEC/RPG) and a live
DVXplorer rig.

**Current status:** the full pipeline launches and runs (cameras, IMU,
time surfaces, mapping, tracking, visualization), and the crashes and
memory leaks found so far are fixed. Pose estimation does **not** work
reliably yet: mapping rarely completes its stereo (SGM) initialization, and
IMU fusion is disabled because it diverged. See "Known limitations".

## Hardware

- 2x Prophesee EVK4, IMX636 sensor, 1280x720
- Serials:
  - `00051182`: physically **left** camera, sync slave (cabled SYNC IN)
  - `00051860`: physically **right** camera, sync master (cabled SYNC OUT)
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

### IMU-to-camera extrinsics: not done yet

`T_b_c` in both calib YAMLs is still the identity placeholder inherited
from the DVXplorer example. It is **not** an actual calibration: no
rotation alignment or lever-arm offset has been measured. With IMU fusion
enabled this produced runaway pose drift (see "Known limitations"). A real
IMU-camera extrinsic calibration (e.g. Kalibr) is needed before re-enabling
`USE_IMU`.

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
- No source code in this package was modified; its fixed-point scaling bug
  (see "Gotchas") is worked around via config.

### This repo (`ESVO2`)

- **New:** `esvo2_core/calib/evk4_stereo/{left,right}.yaml`: the stereo
  calibration described above. `T_b_c` is still an identity placeholder.
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
- **New:** `dependencies.yaml` entry for `sbg_ros_driver`.

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
  made"). The mutex is still held for the whole cycle, so under heavy load
  the node will drop old event batches rather than grow.
- **Left time surface stalls under CPU load.** With the shared cfg's
  `generation_rate_hz: 100`, both nodes only reached ~20 Hz; the left one
  (227% CPU vs right's 133%) intermittently stopped publishing entirely
  while right stayed steady. Symptom in rqt: "the right image moves, the
  left doesn't". The launch file now targets 25 Hz.
- **roslaunch rejects `--` inside XML comments** (`not well-formed
  (invalid token)`), and a Python `xml.etree` well-formedness check does
  not catch it. Validate launch files with `roslaunch --nodes <pkg> <file>`.

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
  also hasn't been visually verified.
- **IMU fusion is disabled** (`USE_IMU: False` in both cfgs). With it
  enabled and `T_b_c` at identity, tracking did produce poses but they
  diverged to hundreds of meters within seconds of hand-held motion.
  Needs a real IMU-camera extrinsic calibration first.
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

Three terminals, run in order. All assume the workspace is already built
and sourced:

```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

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
The IMU isn't currently used by mapping/tracking (`USE_IMU: False`), but
running it keeps `/imu/data` available.

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
