# Reaching paper-level tracking on the EVK4 rig: what was wrong, 2026-09-22/23

Audience: anyone working on this fork. It records two investigations — first
whether this code can reproduce the paper at all, then why it behaved so much
worse on the live rig — with the measurements behind each conclusion, and the
hypotheses that turned out to be wrong.

Branch: `evk4-ts-rate-aa-window`, seven commits on top of
`evk4-gyro-locked-tracking`; Part B3 is on `evk4-stillness-hold`, on top of it.

## Summary

The algorithm was never the problem. Two classes of defect were:

1. **Data rate and packing.** ESVO2 renders a time surface only when a new event
   message arrives, and it tracks once per time surface. Anything that starves
   that chain — 30 ms event batches in a public dataset, a 25 Hz generation rate
   on the rig — costs most of the translation.
2. **Latency, not throughput.** Several stages did full-frame work nobody
   consumed, and mapping's subscriber queues let it run a fixed ~200 ms behind
   the stream. The tracker was registering against maps 300–400 ms old.

On MVSEC `indoor_flying1`, vision-only now matches the published trajectory
(ATE 7.9 cm against the paper's 7.6 cm). On the rig, a 1 m out-and-back slide at
1× replay recovers +1.02 / −0.96 m per leg, where on 2026-09-21 the same replay
recovered 41–45% of the true translation.

## Part A — Reproducing the paper on MVSEC

Goal: separate code problems from rig problems by running the paper's own data.
The target is the paper's published trajectory for `indoor_flying1`
(`results/ours/ours_upenn/`), scored against its ground truth with our own
script: **ATE 7.6 cm, path ratio 0.99** over a 25.8 s window
(1504645187.57–1504645213.40).

| Run | ATE | Path ratio |
|---|---|---|
| Paper's published trajectory | 7.6 cm | 0.99 |
| Ours, vision-only, original bag | 42–62 cm | 0.74–0.79 |
| **Ours, vision-only, events repacked to 5 ms** | **7.9 cm** | **0.98** |
| Ours, IMU in the mapping back end only | 9.0 cm | 0.97 |
| Ours, IMU in tracking, upstream code | diverges | — |
| Ours, IMU in tracking, after the fix of A5 | 0.082 / 0.079 | 0.96 |

### A1. Event packing decides the tracking rate

MVSEC stores events in 30–40 ms `EventArray` messages.
`ImageRepresentation::createImageRepresentationAtTime` only renders when its
`bcreat_` flag was set by an event callback, so time surfaces — and therefore
poses — came out at 30 Hz instead of the paper's 100 Hz, whatever
`generation_rate_hz` said. Splitting the events into 5 ms messages
(`esvo2_core/scripts/diagnostics/rebag_events.py`) closed the entire gap.

The EVK4 driver publishes ~2,800 messages/s, so the rig never had this problem;
it had the equivalent one through `ts_rate` (Part B).

### A2. A long-lived roscore corrupts runs

`rosparam load` merges, it does not replace. On a roscore left running between
experiments, EVK4 keys (`GYRO_BIAS`, `IMU_TIME_OFFSET`, `MAPPING_FLOAT`) leaked
into MVSEC runs, and stereo initialization found 0–8 points instead of
200–1500. `esvo2_core/scripts/replay_eval.sh` clears the four node namespaces
before each run; ad-hoc launches must do the same.

### A3. Upstream's MVSEC launch never connects the IMU

`system_upenn.launch` (and our copy of it) remaps `imu` for the representation
nodes but never `/imu/data`, which is what `esvo2_Mapping` and `esvo2_Tracking`
subscribe to. Every earlier "IMU on" MVSEC run here was in fact vision-only,
printing "not receive imu data" hundreds of times. `system_vector.launch` has
the remap; the upenn and dsec ones do not.

### A4. A crash on the global point cloud (fixed)

With `bVisualizeGlobalPC: True` — set by every upstream dataset config — an
empty voxel-filtered cloud, as happens right after a tracking reset, made
`min(pc_length, threshold) - 1` underflow `size_t`, and the range insert threw
`std::length_error`, aborting the mapping node. Fixed in `167ad8f` (on the base
branch). The EVK4 config has that flag off, which is why the rig never hit it.

### A5. ESVO2's IMU tracking path diverged (fixed in `aa2d54b`)

With the IMU actually connected, upstream's `USE_IMU` mode diverged on both
public datasets: 6×10⁶ m within 10 s on MVSEC `indoor_flying1`, 5×10² to
10¹¹ m on VECtor `desk-normal`, and the mapping node crashed in every such run
observed. The mapping back end alone was never the problem.

**Root cause.** In the `bUseImu_` branch of `curDataTransferring()`, every
frame's translation prior is

    prior = R_b_c^T · Δp_imu  +  mean(last five registered displacements)

and the second term is unbounded. Instrumenting both terms per frame showed
the divergence always starting with one bad registration step (130 mm, later
266–398 mm). That step entered the moving average, the next frame started
tens of centimetres off, registered wrong again, and the error compounded
until it ran away. Vision-only mode has no such term, which is why it was
stable all along.

**How it was confirmed.** Removing only that term, on both datasets, stopped
the divergence and every crash (0 in 10 runs, against 3 crashes in 3 runs with
it). Restoring it through the new parameter reproduces the divergence
(3.9×10⁷ m) and the crash. `IMU_CONSTANT_VELOCITY_PRIOR` (default false)
controls it; `true` is upstream's behaviour.

**Results with the fix**, stock configs, ATE SE3 / Sim3:

| Sequence | IMU mode, fixed | Vision-only | Paper |
|---|---|---|---|
| MVSEC `indoor_flying1` | 0.082 / 0.079, ratio 0.96 | 0.079 | 0.076 / 0.076, 0.99 |
| MVSEC `indoor_flying2` | 0.141 / 0.075, ratio 0.93 | 0.107–0.126 | 0.100 / 0.066, 0.96 |
| MVSEC `indoor_flying3` | 0.073 / 0.046, ratio 0.97 | 0.068–0.074 | 0.073 / 0.049, 0.95 |
| VECtor `desk-normal`, 3 runs* | 0.198–0.224, ratio 0.89–0.93 | 0.210–0.227, ratio 0.81 | 0.165 / 0.146, 0.89 |

\* with `Regularization: False` (A9).

IMU mode now matches vision-only on ATE, and on VECtor the gyro rotation prior
lifts path recovery from 0.81 to the paper's 0.89. It does not close the
remaining ATE gap to the paper on VECtor (0.20–0.22 against 0.165), so that gap
is not explained by the IMU after all.

**Checked and not significant — left as they are:**

- *Gravity in the IMU displacement.* `Δp_imu` is preintegrated from raw
  accelerometer readings without subtracting gravity, so it carries a constant
  ~0.3 mm per frame (½·g·Δt²) whatever the motion. It is a real defect, but
  dropping the term entirely changed nothing measurable (ATE 0.200–0.209
  against 0.203–0.235): registration absorbs it.
- *The IMU-integrated velocity* in `curImuTransferring()` accumulates
  `Δv` with gravity in it and grows at roughly g between back-end updates
  (to ~40 m/s). It is never used: the consistency check that would select it
  fails on every frame.
- *The crash.* All three crashes observed were in runs with the velocity prior
  on; none in ten runs without it. That is consistent with the crash being
  downstream of the divergence (absurd poses reaching the map), but it was
  never caught in the act: under gdb the node slowed enough that it did not
  crash at all, and glog's failure signal handler caught nothing in the runs
  that followed. Treat it as unexplained if it ever reappears with the prior
  off; the first suspect would then be the unsynchronised access to the
  depth-point deque that the back end's Ceres solve shares with the mapping
  thread, which `EVK4_STEREO_SETUP.md` records as guarded but not fixed.

Two traps from this investigation: gdb changed the timing enough to hide the
crash, and `tracking_vector_AA.yaml` has no trailing newline, so appending a
key with `>>` silently produced `USE_IMU: TrueNEW_KEY: ...`, invalid YAML, and
a run that recorded nothing.

### A6. Checked on two sequences this fork had never run

`indoor_flying2` and `indoor_flying3` were downloaded afterwards and run
without tuning anything, vision-only, with the upenn configs as committed.
ATE is SE3 / Sim3 in metres, over each sequence's published trajectory window.

| Sequence | Paper | Ours, original packing | Ours, 5 ms packing |
|---|---|---|---|
| `indoor_flying1` | 0.076 / 0.076, ratio 0.99 | 0.42–0.66, ratio 0.74–0.79 | 0.079 / 0.077, ratio 0.98 |
| `indoor_flying2` (unseen) | 0.100 / 0.066, ratio 0.96 | 0.874 / 0.873, ratio 0.65 | 0.126 / 0.067 and 0.107 / 0.063, ratio 0.94 |
| `indoor_flying3` (unseen) | 0.073 / 0.049, ratio 0.95 | 0.520 / 0.456, ratio 0.72 | 0.074 / 0.046, 0.068 / 0.043, 0.070 / 0.046, ratio 0.96–0.97 |

The packing result holds on data that had no part in finding it, and the
code as committed — float time surface rendering, the narrowed SGM range,
lazy conversion, the tracking changes — reproduces the paper on all three.

One difference worth noting: the repacked runs reset tracking 98–239 times
per run while the 30 Hz runs never reset. Resets are not, by themselves, a
quality signal here — the runs with hundreds of them are the accurate ones,
because at 100 Hz the tracker outruns a 20 Hz mapping thread, recovers, and
still produces a far better trajectory than the slow, stable 30 Hz runs.

**`aa_window_ms` does not transfer to MVSEC**, which is why its default is 0.
On `indoor_flying3`, setting it to 40 removes the resets completely (125 → 0)
and costs accuracy: ATE 0.114 and 0.120 against 0.068–0.074 without it. The
rig needs it because 10 ms of events is sparse across 1280×720; MVSEC's
346×260 DAVIS is dense enough that a longer window only stales the map.
`TS_QUEUE_SIZE: 2` made no difference there either (0.110), as expected:
MVSEC's frames are small, so mapping never falls behind the stream.

### A7. VECtor `desk-normal`: a different sensor, the same failure mode

A harder test than more MVSEC flights: VECtor uses two Prophesee event
cameras at 640×480 (closer to the EVK4 than MVSEC's 346×260 DAVIS), a
different scene and a different IMU. Its bags publish
`prophesee_event_msgs/EventArray`, whose layout is identical to
`dvs_msgs/EventArray` (same md5), so `/root/datasets/vector/retype.py`
rewrites them raw under the topic names the launch expects. No repacking is
needed — VECtor already publishes ~3,900 small event messages per second.

Scored over the published trajectory's window (89.1 s), ATE SE3 / Sim3:

| Run | ATE | Path ratio |
|---|---|---|
| Paper's published trajectory (its config has `USE_IMU: True`) | 0.165 / 0.146 | 0.89 |
| Ours, vision-only, 0.5× replay, 3 runs | 0.196–0.223 / 0.165–0.208 | 0.81–0.82 |
| Ours, vision-only, 1×, 4 runs | 1.32–1.54 / 0.30–0.33 | 1.59–1.68 |
| Ours, 1×, lighter mapping (`BM_step: 3`, regularization off) | 0.240 / 0.220 | 0.80 |
| Ours, 1×, after the regularizer work (A8), 4 runs | 0.41–0.67 / 0.29–0.31 | 0.94–1.11 |
| **Ours, 1×, regularization off (A9), 2 runs** | **0.210–0.227 / 0.186–0.206** | **0.81** |
| Upstream IMU mode (`USE_IMU: True`) | diverges or segfaults; fixed in A5 | — |
| IMU mode after the A5 fix, regularization off, 3 runs | 0.198–0.224 / 0.191–0.213 | 0.89–0.93 |

A caution about single runs, since this one caught me out: the first 0.5×
run scored 0.118 and I reported that this build beats the paper on its own
data. Repeats put 0.5× at 0.196–0.223, i.e. comparable to the paper's 0.165,
not better; the old, unmodified regularizer gives 0.201–0.222 on the same
runs, so nothing regressed — the 0.118 was simply a lucky draw. VECtor's
run-to-run spread is wide enough that no single run means anything here.

Two things follow.

**The algorithm and this fork's changes are sound.** Given enough compute
(0.5×), vision-only lands in the same range as the paper's own published
trajectory on its own data, without tuning anything.

**At 1× the failure was the rig's failure exactly.** The first 68 s track as
well as the paper (ATE 0.154 against 0.131); the last 21 s blow up
(1.116 against 0.078), and that is where the motion is fastest — mean gyro
rate rises from ~0.10 rad/s at the start to 0.34 rad/s, peaks 0.75. Mapping
publishes at **2.0 Hz at 1× against 5.5 Hz at 0.5×**, so the map the tracker
registers against is stale, which is the same mechanism as B5/B6.

**But the EVK4's fixes do not transfer, because the bottleneck is not the
same.** `TS_QUEUE_SIZE` and the rest changed nothing here (1.405): VECtor's
frames are small, so mapping never queues — its own cycle is simply slow,
because the stock VECtor config is far heavier than the rig's (`BM_step: 1`
against 3, regularization on, `mapping_rate_hz: 10`). Lightening it the way
the rig's config was lightened fixes the divergence (late window 0.100
against 1.116). The transferable finding is *map age*, not any particular
parameter — and `aa_window_ms` (A6) is the same story in reverse.

The gyro rotation lock, this fork's own feature, does not help here (1.495):
the failure is not the rotation-for-translation ambiguity it was built for.

### A8. The regularizer was 93% of the mapping cycle

Profiling that 2 Hz mapping cycle on VECtor gave, per cycle: regularization
404–552 ms, block matching 11–14 ms, fusion 12–14 ms, the depth solve
2.4–2.9 ms, denoising ~2 ms. The stage the rig had simply switched off was
almost the whole cycle.

`DepthRegularization::apply` walks every point in the map (15k–26k on
VECtor) and, for each, scans a (2r+1)² neighbourhood — 1681 cells at the
configured radius 20. Four changes, none touching the arithmetic
(`f9bc34a`): run the per-point loop on `NUM_THREAD_MAPPING` threads, since
each point reads the old map and writes only its own cell; reuse the
neighbour buffers instead of a 13 kB allocation per point; clamp the scan's
bounds once and walk each row through a single pointer instead of
bounds-checking every cell and chasing `_grid[r]` three times; and compare
squared distances, because `diff < 2*sqrt(var)` is `diff² < 4*var`.

Regularization 404–552 → 50–107 ms, the cycle ~500 → 84–143 ms, ATE at 1×
1.32–1.54 → 0.41–0.67. Most of that is the parallelisation; the scan and
sqrt changes are worth little on their own.

Two notes. The build was already `-O3` (`esvo2_core/CMakeLists.txt` sets it
regardless of `CMAKE_BUILD_TYPE`, which the catkin cache leaves empty), so
none of this was a missing compiler flag. And on the rig, whose map is ~750
points rather than 20k, the stage now costs 5–6 ms and the whole cycle fits
in 38–44 ms with regularization on — but the legs are unchanged
(+1.04 / −0.93 against +1.02 / −0.96 with it off), so the rig config leaves
it off.

`test/test_depth_regularization.cpp` pins the result: on nine random
VECtor-sized maps (≈19k points, Tdist at radius 20 and l2 at radius 5, one
and four threads) the optimised regularizer matches a verbatim copy of the
upstream algorithm bit for bit — including the squared-distance comparison,
which is equivalent in exact arithmetic but could in principle flip at a
floating-point boundary; it never did. In isolation it scales: ~100 ms
single-threaded against 27 ms on four threads. Live it takes 50–107 ms for
similar map sizes, so contention with the other nodes costs it a factor of
2–4 there, and eight threads are no faster than four.

### A9. On VECtor, regularization is not worth its cost

Before optimising the regularizer further, the question was whether it buys
anything. Isolating that one setting (`Regularization: False`, everything
else stock), ATE SE3 over the published window:

| | 0.5× | 1× |
|---|---|---|
| Regularization on | 0.196, 0.222, 0.223 | 0.41–0.67 (4 runs) |
| Regularization off | 0.228, 0.228 | 0.227, 0.210 |

At 0.5× the difference is within run-to-run noise, and at 1× turning it off
**closes the real-time gap completely**: 1× matches 0.5×. The rig reached the
same conclusion for the same reason (Part B). The upstream dataset configs
are left as upstream ships them; to run VECtor in real time on this machine,
set `Regularization: False` in `mapping_vector_AA.yaml` (and `USE_IMU: False`,
which was needed until the IMU fix of A5; with it, `USE_IMU: True` works and
improves path recovery).

Against the paper's published 0.165 m, ours at 0.21–0.23 m is vision-only and
the paper's is not; the remaining difference is plausibly the IMU, which we
cannot run (A5). The regularizer work of A8 stays: it is exact, tested, and
makes regularization affordable if a scene turns out to need it.

## Part B — The live rig, 1280×720 at real time

All numbers below: `slide4_bias.bag`, gyro lock on, evaluated with
`esvo2_core/scripts/eval_roundtrip.py`, which compares mean poses over
gyro-detected still periods. The rig's true legs are ~1.17 m out (a 1.03 m move
plus a 0.14 m step) and ~1.17 m back.

**Replay at 0.5× is the compute-free control.** Everything is in sim time, so a
slower replay gives every stage more wall-clock time per frame without changing
the data. Comparing 0.5× against 1× separates algorithm problems from compute
problems, and that comparison drove most of what follows.

### B1. Tracking rate is what buys accuracy

| `ts_rate` (= tracking rate) | Out leg | Back leg | Closure (x) |
|---|---|---|---|
| 25 (the rig's old setting) | 0.84 m | −0.71 m | 0.28 m |
| 50 | 0.99 m | −1.05 m | 0.08 m |
| 100 | 1.03 m | −1.13 m | 0.04 m |

50 Hz captures most of the benefit. Tracking at 25 Hz while time surfaces run at
100 Hz gives the old poor result, so it is the tracking rate that matters, not
the surface rate by itself.

### B2. The AA map must not depend on the rate (`7afb0de`)

Mapping samples its candidate points from the AA map, which was built only from
the events since the previous render — 40 ms of events at 25 Hz but 10 ms at
100 Hz. On a 1280×720 sensor with a still rig, 10 ms is too sparse: candidates
fell from 4,000 to ~1,100, the local map decayed below the tracking minimum, and
tracking reset 3,601 times in one replay. `aa_window_ms` builds the map from a
sliding window instead (default 0 = upstream behaviour; the rig uses 40).

### B3. Making the nodes fast enough for 50 Hz

| Change | Effect |
|---|---|
| Float time surface rendering, LUT, fixed-point remap (`80eb43c`) | left node 32 → 20 ms/frame, 28 → 45 Hz |
| Skip the unsubscribed reprojection map; stop spawning threads per residual evaluation (`4c0b6fc`) | tracking solve 23 → 5.3 ms |
| SGM at half resolution over the block matcher's disparity range (`2924ecb`) | initialization ~2 s → ~0.1 s per attempt |
| Shallow subscriber queues, `TS_QUEUE_SIZE: 2` (`3508840`) | map age 300–400 → ~150 ms |
| Lazy cv→Eigen conversion (`3985fbc`) | TS callback 17–19 → 2.2–3.2 ms |
| `MAPPING_FLOAT: True` (`1191b9d`) | A1's 1.30× on block matching, now on the critical path |

Two of these were pure waste rather than algorithmic cost: an 11 ms/frame debug
visualisation that ran with zero subscribers, and ~25 thread spawns per frame to
split 300 points across 4 threads.

### B4. The late start was SGM, not "mapping is slow"

Initialization ran `cv::StereoSGBM` over the full frame and the whole
0–`BM_max_disparity` range (320): ~295M pixel-disparity evaluations, ~2 s per
attempt. At 1× that is 2 s of mission time per try, so the system took 25–30 s
to initialize and a slide starting at 13 s was over before tracking began. This
is the "late start" that appeared in 11 of 13 runs on 2026-09-22 and was
previously blamed on map density. `SGM_DOWNSAMPLE: 2` plus the narrowed range
fixed it; the disparity map is only sampled at event pixels, so a coarse solve
is enough to seed the map.

### B5. Stale maps, from queue backlog

The decisive measurement of the whole investigation: **the age of the map the
tracker registers against**, 65–78 ms at 0.5× against 300–400 ms at 1×.

Mapping subscribes to six full-frame images with queues of 10. At 50 Hz a node
that cannot keep up therefore works through a 200 ms backlog instead of skipping
to the newest frame — the surface it processed was 250–280 ms old at 1×, against
55 ms at 0.5×. `TS_QUEUE_SIZE` sets those queues and the synchronizer
(default 10, the rig uses 2). **Queue 1 is worse than 10**: the approximate-time
sync starves and the age jumps to 380–500 ms.

### B6. What the callback was actually doing

Even with shallow queues, mapping's time surface callback cost 17–19 ms per
frame — close to a full core at 50 Hz. It converted six 1280×720 images into
double Eigen matrices (~7 MB each) for *every* frame, while mapping only reads
the ~20 per second that `dataTransferring()` selects.
`TimeSurfaceObservation::ensureEigen()` defers that to the two places an
observation is selected. `isEmpty()` now treats an observation that still holds
its images as loaded.

### B7. Where the rig stands

| | Out leg | Back leg | Closure (x) | Closure (z) |
|---|---|---|---|---|
| 1×, two runs | 1.02, 1.03 m | −0.96, −0.93 m | 0.24 m | −0.41, −0.79 m |
| 0.5× | 1.02 m | −1.08 m | 0.06 m | −0.29 m |
| 1×, with B1+B2 only (before the SGM, queue and callback fixes) | 0.54 m | −0.38 m | 0.21 m | — |

## Part B2 — The hallway session: a miscalibrated rig

A test with real depth variation and tape-measured ground truth: the rig on a
wheeled cart in a hallway, facing an alcove whose front wall is 3.00 m away
and whose back (two doors) is 4.10 m away, rolling along its optical axis.
Bags in `/root/datasets/evk4/hallway*.bag`.

**Sensor settings for a dim scene.** With the biases used for `slide4_bias`,
the cameras produced 2.7–4.0 Mev/s *at rest* in the dim hallway, spread over
200–270k pixels (noise, not hot pixels or flicker), and both hit the 4 Mev/s
rate cap during motion, dropping events. `bias_diff_on/off: 20` cut the rest
rate to 0.02–0.05 Mev/s while motion still gave 0.4–5 Mev/s. The later bags
use those biases and an 8 Mev/s cap.

**ESVO2 cannot handle the rig standing still.** With quiet biases, a still
event camera sees nothing. On the stop-and-go bag (`hallway2`), tracking
worked during every roll and reset continuously during every stop — about
3,400 resets per run, at 0.5× as at 1×. Stereo initialisation found a median
of 14 points at rest (it needs 500). Every leg was then measured between
disconnected tracks. `slide4_bias` never showed this only because its noisy
sensor kept firing at rest; the public datasets never show it because they
are recorded in continuous motion. A continuous bag (`hallway3`: 4.10 →
2.10 → 4.10 m twice, no stops) avoids it. The real fix — holding pose and map
while the gyro says the rig is still — is not done yet.

**The rig had lost its calibration.** On `hallway3`, forward legs came out
~12% short with 0.3–0.5 m of closure error, identically at 0.5× and 1×. The
map explained why: ~60% of its points sat at 7–9 m, in a scene with nothing
beyond 4.10 m. In disparity space both walls were off by the same −29 px
(measured 52–56 and 28–32 px against 82 and 60 px expected), the signature of
a changed relative orientation rather than of false matches. The cause: the
camera mounts had been swapped for identical ones after 2026-09-21.
Recalibrating (`esvo2_core/scripts/calibration/`) found the right camera
moved by ~0.70° yaw and 0.49° pitch, and the baseline grown from 146.6 to
154.3 mm. With the new calibration (`calib/evk4_stereo_2026-09-23/`):

| `hallway3`, forward, truth 2.00 m legs | Legs | Mean | Closure (z) |
|---|---|---|---|
| Old calibration | 1.46–2.11 | ~1.75 (−12%) | −0.29 / −0.49 m |
| New calibration, 1×, 2 runs | 1.80–1.98 | ~1.87 (−6.5%) | +0.09 / +0.10 m |
| New calibration, 0.5× | 1.69–1.92 | ~1.81 | +0.28 m |

and the map now places the front wall at 2.75–3.25 m and the doors at
4.00–4.25 m. The first leg is nearly exact (1.96–1.98 m); later legs are
5–10% short, which is the next thing to understand. Sideways (`hallway4`,
truth 1.00 m) the legs come out at 0.82–0.89 m with a 0.2–0.5 m forward or
backward component. A cart pushed sideways on swivelling casters can crab, and
each sideways leg is a separate track starting after the motion does, so that
figure is not a clean measurement.

**Cross-check on the pre-swap bags.** If the mount swap really changed the
geometry, bags recorded before it should be *worse* with the new calibration,
the mirror image of the hallway. They are:

| Pre-swap bag | Old calibration | New calibration |
|---|---|---|
| `wall_tex_185`, wall at a measured 1.85 m (RANSAC plane) | 1.920 m (+3.8%, as originally measured) | 1.565 m (−15.4%) |
| `slide4_bias`, ~1.17 m out and back, 2 runs each | +1.01 / +1.03 out, −0.98 / −0.96 back | +0.75 / +0.73 out, −0.63 / −0.61 back |

So each calibration is right for the bags of its own era, and the change is
physical rather than an artefact of either calibration session.
`calib/evk4_stereo/` stays the one to use for anything recorded before
2026-09-23; the launch files default to the new one.

**What this says about the earlier "z-drift".** The drift along the optical
axis chased since 2026-09-21 was measured on a correctly calibrated rig and is
not explained by this. But every rig result from after the mount swap and
before this recalibration is invalid, and the IMU-to-camera rotation (`T_b_c`,
used by the gyro lock) was not recalibrated and is probably stale too.

**Dead ends in this session**, in order:
- *"Forward motion is under-estimated because each frame's solve stops short
  in the weak-flow direction."* Proposed from the 12% bias before the depth
  check. The depths were wrong instead; with the calibration fixed, most of the
  bias went.
- *A one-angle software patch.* Fitting a single yaw (0.93°) to the two known
  depths removed the drift (closure −0.02 m) but made the legs 24% short. It
  was absorbing three changes — 0.70° yaw, 0.49° pitch and a 5% longer
  baseline — into one parameter. Useful as a diagnosis, wrong as a fix.
- *"Neither camera streams; a sync mode is stuck."* Both cameras streamed
  fine. The test that seemed to show otherwise was broken twice over: its
  timer started before opening the camera, which alone outlasts the window,
  and it called `len()` on the SDK's buffer type.
- The calibration tool's `LIBUSB_ERROR_TIMEOUT` was a consequence, not the
  cause: its detection step ended at once and the error came from stopping
  the camera abruptly. What made it work on the third try is not certain.

## Part B3 — Holding state at rest (2026-09-24, branch `evk4-stillness-hold`)

A still event camera in a quiet scene sees almost nothing. Mapping's local map
thins below `BATCH_SIZE` (300 points; ~120 in the hallway), registration
refuses, and upstream sets the whole system back to `INITIALIZATION`: the
reference pose restarts at identity and mapping re-runs SGM. On `hallway2`
(stop-and-go, 2/2/3/3 m legs) that is 3,504 tracking resets and 930 SGM
re-initialisations, and the bag cannot be scored at all: every leg starts from
zero.

**What the tracker now does (`STILLNESS_HOLD`, on in the EVK4 config, off by
default).** Events decide first; the IMU is asked only when they cannot.

| Events | IMU (`/imu/data_synced`) | Tracker |
|---|---|---|
| Registration supported: J^T J/N ≥ `STILL_MIN_INFO` (and, if the IMU says still, time-surface structure ≥ `STILL_MIN_SUPPORT`) | any | publish the registration |
| Not supported, or map too sparse | still | **hold**: republish the last pose |
| Not supported, time surface quiet | moving | **coast** on the last pose (events see no motion) |
| Not supported, time surface active | moving | coast ≤ `HOLD_COAST_S` (0.5 s), then upstream |
| any | missing, stale, gappy or frozen | upstream: publish the registration if any, else reset |

- *Stillness* (`tools/stillness.h`, 11 unit tests): over the last 0.5 s, gyro
  std < 0.0035 rad/s, |gyro mean| < 0.02 rad/s and accelerometer std < 0.03 m/s².
  The accelerometer is needed because a cart rolling straight barely rotates.
  No sample within 0.1 s, a gap over 20 ms, a window under 90% full or a window
  with no noise at all (a driver repeating its last message) give `UNKNOWN`,
  never "still".
- *Observability*: the smallest eigenvalue of J^T J/N from the last LM batch
  (translation block when the rotation is locked). Moving frames: p5 4×10^5;
  frames registered on a still camera: 5×10^4–2.6×10^5. Threshold 10^5.
- *Structure*: from the time surface the tracker already has (8-bit,
  255·exp(−age/20 ms)), the share of pixels fired within one decay constant
  that sit in 8×8 cells holding ≥ 4 of them. Noise is isolated, edges cluster:
  ~0.9 moving, ~0 on a still camera. The same metric on raw events over the
  three hallway bags: median 0.95 moving, 0.01–0.02 still; event rate
  1.4 M ev/s vs 25 k ev/s.
- *Map*: while holding, and for `HOLD_MAP_GRACE_S` (1 s) after the first sparse
  map or the end of a hold, the tracker keeps its last map with ≥ `BATCH_SIZE`
  points instead of swapping in a near-empty one. It now owns that cloud
  (`refCloud_`), so the points outlive `refPCMap_`'s eviction.
- *Hysteresis*: once holding or coasting, the tracker resumes only on a
  structured time surface (`support` ≥ `STILL_MIN_SUPPORT`) as well as J^T J ≥
  `STILL_MIN_INFO`. Without it, isolated frames of a still camera cross the
  J^T J threshold by chance and are published: jumps of up to 0.5 m while the
  cart was handled at the end of `hallway4_lateral`. While tracking, a quiet
  surface alone does not stop it (7–10% of moving frames have one and register
  well).
- A hold always publishes a pose: mapping resets itself when poses stop for
  0.5 s (`stampedPoseCallback`), so "hold" must never mean "stay silent".
- `HOLD_MAX_S` (120 s) ends any hold or coast. `MOTION_LOG:=file.csv` (launch
  arg `motion_log`) writes every frame's inputs and decision; `stillness_hold`
  overrides the config from the launch line.

**Results** (1× replay, new calibration with the corrected `T_b_c` below; the
`slide4_bias` runs predate the hysteresis, which only changes how holds end):

| Bag | Hold off | Hold on |
|---|---|---|
| `hallway2`, legs 2 / 2 / 3 / 3 m, 2 runs | 3,504 resets; each leg from zero | **1.99–2.04 / 1.77–1.79 / 2.89–2.92 / 2.74–2.76 m**, closure 0.41–0.43 m over 10 m, 0 resets |
| `hallway3`, far points 2.00 m | 1.97 / 1.90 m, closure 0.11 m, 2,041 resets | 1.95 / 1.89 m, closure 0.12 m, 0 resets, one track |
| `hallway4_lateral`, 1.00 m out and back | −0.48 / +1.29 m, closure 0.83 m, 3,498 resets | **−0.88 / +0.86 m**, closure 0.08 m, 0 resets |
| `slide4_bias` (old calib), 3 runs each | out 1.04 / 1.01 / 1.04, back 0.96 / 1.04 / 0.94 | out 1.01 / 0.96 / 1.02, back 0.88 / 0.92 / 0.89 |
| `hallway2`, IMU cut 55 s in | — | holds until the cut; after it one warning, then upstream behaviour (resets at stops), tracking and mapping keep publishing |

`slide4_bias` looks like a regression on the return leg and is not one. That
near, textured wall still produces events at rest, so with the hold off the
tracker keeps registering, and drifts, during the stops: −5 cm over 33–42 s,
−9 cm over 53–69 s. `eval_roundtrip.py` averages the pose over each stop, so
that drift lengthened the measured return leg. During the motion itself both
modes measure the same leg (43 → 52 s: 0.895 m off, 0.884 m on).

**Residuals.**
- Isolated frames crossing J^T J = 10^5 on a quiet time surface while
  coasting (a 10 cm jump on `hallway3`, up to 0.5 m on `hallway4_lateral`).
  Fixed by the hysteresis above. Vetoing every quiet frame would not have
  been: 7–10% of genuinely moving frames have a quiet surface, in runs up to
  0.7 s, and register well (median 4×10^5).
- The forward legs are still 2–7% short (B2); the hold only stopped resets from
  hiding it.
- The hallway legs back towards the start are 7–11% short, the forward ones
  0–4% (B2).

**`T_b_c` corrected for the new rectification.** The IMU was not touched, but
the new stereo calibration rotated the *rectified* left frame by 3.5° (mostly
about y), and `T_b_c` refers to that frame. Assuming the left camera did not
move against the IMU, R_b_c,new = R_b_c,old · R1,old · R1,newᵀ (R1 = each
`left.yaml`'s `rectification_matrix`, raw → rectified). Supporting evidence:
the corrected rotation is 1.1° from a square axis permutation, against 4.1°
before, as expected for housings mounted square to each other. It makes no
measurable difference on the hallway bags (same code, corrected vs old, legs
within run-to-run noise: `hallway2` 1.99/1.79/2.89/2.74 vs 1.99/1.79/2.72/2.71
and 2.04/1.77/2.92/2.76 vs 2.03/1.78/2.93/2.75; `hallway4_lateral`
−0.88/+0.86 vs −0.89/+0.90). Those are straight cart runs with little
rotation, so they cannot validate it either: that needs a capture with real
rotation (`calibrate_imu_camera_rotation.py`, ~60 s turning the rig by hand).

## Part B4 — Review of the whole fork (2026-09-24, branch `review-fixes`)

A review of everything upstream `main` does not have (87 commits) found the
problems below. Each fix was checked against a new replay regression gate
(`esvo2_core/scripts/regress/regress.sh`, Part D), run on the code before and
after.

| Problem | Fix |
|---|---|
| `evk4_live_all.launch` ran time surfaces at 25 Hz without the AA window, with default sensor biases and a 4 Mev/s cap: none of it the configuration any result was measured with (B1: 25 Hz loses 15–30% of the translation) | `system_evk4_mapping.launch` defaults to `ts_rate` 50 and `aa_window_ms` 40; `evk4_live_all.launch` sets biases `{bias_diff_on: 20, bias_diff_off: 20}` and an 8 Mev/s cap, the hallway recording settings |
| `evk4_drivers/prophesee_ros_wrapper.patch` had fallen behind the driver's own fork (no bias parameters, a missing hot pixel, pre-swap serials) | Removed; the docs point to `lecrosnier/prophesee_ros_wrapper`, branch `evk4-noise-filters` |
| Mapping's detached publishing threads (one per cycle) rebuilt the shared clouds, including the tracker's `pc_color_`, with no lock, and read `dqvDepthPoints_` and the status string while the mapping thread changed them. Inherited from upstream; the fork's ~20 Hz mapping made overlaps likelier | The publish decision is taken on the mapping thread and passed in; the clouds are rebuilt under `publish_mutex_`, which `reset()` also takes |
| With `USE_IMU` set but no IMU data, mapping segfaulted: the initial orientation came from 0/0 samples and the back end dereferenced pre-integrations that were never created. The stock `system_upenn.launch` does exactly this (it never remaps the IMU) | The first IMU pose waits for samples; the back end is skipped until the whole window has IMU data. The stock launch now runs the whole of `indoor_flying1` (6,493 poses) |
| The time-surface node, on a backward time jump (a looped bag, a clock step), insertion-sorted every new event past up to 5 M buffered ones and read its new look-up table at negative indices. The event bounds check also let x = width through | Jumps over 1 s drop the buffer and reset the per-pixel times and AA window; ages are clamped; bounds are `>=`. A looped 6 s slice of `hallway3`: a warning per jump, no deaths |
| The gyro bias was estimated once, at startup | `GYRO_BIAS_REFRESH` (on for the EVK4) re-estimates it from each 2 s still window of a hold: 21–34 refreshes per hallway bag, changes of ~10^-4 rad/s |

**Regression gate, same day, before → after:**

| Case | Before | After |
|---|---|---|
| `hallway2` legs / truth, closure | 1.01 / 0.89 / 0.99 / 0.91, 0.41 m | 0.98 / 0.87 / 0.97 / 0.93, 0.43 m |
| `hallway3` far points, end | 2.05 / 1.92 m, 0.02 m | 1.95 / 2.03 m, 0.13 m |
| `hallway4_lateral` legs, closure | 0.91 / 0.86 m, 0.05 m | 0.96 / 0.87 m, 0.09 m |
| MVSEC `indoor_flying1` ATE (vision only), 2 runs each | 0.085 / 0.088 m | 0.092 / 0.086 m |

All within run-to-run spread; 0 tracking resets throughout. All 45 C++ and 13
Python unit tests pass.

## Part C — Dead ends, in the order they were tried

Recorded because each one cost time and none of them is obviously wrong in
advance.

1. **"A1's float path is marginal."** Measured offline on 2026-09-22 against a
   52 ms cycle, it was: 1.30× on block matching, ~3.6 ms saved. That conclusion
   did not survive the pipeline changing — once mapping became the stage that
   overruns, the same 1.30× moved the outward leg from 0.85–0.89 m to
   0.97–0.99 m. Benchmarks describe the pipeline they were run on.
2. **Map starvation at 100 Hz blamed on compute.** It was the AA map's batch
   window (B2); at 0.25× replay, where compute is free, the starvation was
   identical.
3. **"Mapping's rate limits accuracy at 1×."** Disproved: capping mapping to
   10 Hz at 0.5× still gives good legs (+1.03 / −1.02).
4. **Fewer candidates (4,000 → 3,000 → 2,500).** Raised the mapping rate
   barely (9.1 → 10.4 → 10.1 Hz) and made tracking worse. At 0.5×, 3,000
   candidates track fine, so candidate count was never the limit.
5. **`maxNumFusionFrames: 5 → 3`.** Catastrophic: 2,272 tracking resets.
6. **OpenCV thread tuning (`opencv_threads` 3 and 6).** No change in delivered
   rate. The parameter is committed because oversubscription is real — remap
   costs 0.94 ms alone and 7.4 ms inside the running pipeline — but tuning it
   did not help.
7. **`TS_QUEUE_SIZE: 1`.** Worse than the default 10 (B5).
8. **Narrowing the callback mutex plus an async spinner.** Staleness 87 → 79 ms,
   inside run-to-run noise, and a multi-threaded spinner lets two callbacks call
   `reset()` concurrently, which joins the mapping thread. Reverted.
9. **"Tracking is behind like mapping was."** Disproved: tracking always
   processes the newest surface in its history (28 ms old at 0.5×, ~50 ms at
   1×). Its queues never build up.
10. **"The map decays during the still period before the return leg."** It does
    — to ~800 points with variance ~0.012 — but identically at 0.5× and 1×, so
    it explains nothing about the difference.
11. **Stereo stamp misalignment under load.** Left/right surfaces are *better*
    aligned at 1× (median 0 ms) than at 0.5× (a constant 15 ms offset).
12. **Event delivery lag into the representation node.** Under 1 ms at both
    speeds, with matching batch sizes.
13. **Solver quality.** Registration residuals and iteration counts are
    identical at both speeds (RMS ~111, 10 iterations, 300 points): the solver
    was fitting bad data, not fitting badly.
14. **"Map age explains the short legs."** It does not. Forcing a 168 ms map age
    at 0.5× left the legs correct (+1.01 / −0.99) while closure in z went from
    −0.06 m to −0.79 m. Map age drives the **z-drift**; the legs were the
    callback cost (B6).

15. **Holding without keeping the map (B3).** The first version held only
    while the newest map was unusable, but by then it had already been swapped
    in (9 points). The rig then reset the moment it moved.
16. **Registration alone as the "events can see" test.** At rest the held map
    registers against an empty time surface with J^T J just above any sensible
    threshold, and the pose wandered 0.9 m over a 27 s stop. Hence the
    time-surface structure test.
17. **Trusting a "moving" IMU at the start of a leg.** The accelerometer crosses
    its threshold ~0.2 s before the scene shows anything; publishing those
    registrations (J^T J 10^2–10^4) jumped 0.2–0.6 m. Hence coasting.
18. **Resuming on J^T J alone.** Once holding or coasting on a quiet time
    surface, single frames still crossed the threshold by chance, and each
    one jumped the pose. Resuming now also needs a structured time surface.
19. **A fixed cap on coasting.** With the cart being handled at the end of
    `hallway2`, the cap expired on a still-empty time surface and an
    uninformed registration (J^T J 553) jumped 0.9 m. A quiet time surface now
    means "no motion" whatever the IMU says.
20. **`STILL_MIN_INFO: 1.0e5` in YAML.** PyYAML, and rosparam, read it as a
    string (YAML 1.1 needs `1.0e+5`). Written as `100000.0`.

21. **A per-frame speed gate** (`MAX_SPEED` 2 m/s, B4). Meant to catch
    registration jumps, it rejected 89–242 frames per hallway bag, median
    2.8 m/s: 5.6 cm in one 20 ms frame is ordinary frame-to-frame
    registration noise here. Holding the pose on those frames trimmed real
    progress (`hallway4` leg 2: 0.86 → 0.785 m). Removed; the hysteresis (B3)
    already stops the jumps it was meant for.
22. **Rejecting gyro intervals with an internal gap.** Two existing tests
    specify, on purpose, that sparse samples are integrated with a
    zero-order hold and only the interval's ends need coverage. The rig's IMU
    runs at 200 Hz, so the gap case is minor; reverted rather than rewrite
    the tested contract.

Two tooling traps also cost time: `rostopic hz` reports nothing useful under
`use_sim_time` (use a subscriber node — `scripts/diagnostics/ratemon.py`), and
`pkill -f <script>` matches the shell running it.

## Part D — How to reproduce these measurements

- `scripts/replay_eval.sh` — replay a bag through a launch file, record poses,
  report SGM inits and tracking resets. Clears node namespaces first.
- `scripts/eval_roundtrip.py` — still-period leg metric for the slide bags.
- `scripts/eval_traj.py` — ATE and path ratio against a ground-truth file;
  `WIN="t0 t1"` restricts scoring to a published trajectory's window.
- `scripts/diagnostics/rebag_events.py` — split event messages into N ms slices.
- `scripts/diagnostics/ratemon.py` — delivered rates of time surfaces and poses.
- `scripts/diagnostics/pcmon.py`, `pcdepth.py` — local map size, and map depth
  by phase.
- `motion_log:=file.csv` + `scripts/diagnostics/motion_log_summary.py` — every
  tracking frame's hold inputs and decision, and a timeline of them (B3).
- `scripts/diagnostics/imu_cut_relay.py` — replays with the IMU dying at a
  chosen stamp.
- `scripts/regress/regress.sh OUT_DIR [case...]` — the regression gate: replays
  `hallway2`, `hallway3`, `hallway4_lateral` and MVSEC `indoor_flying1`
  (vision only) through the committed launch files and checks legs, closure,
  resets and ATE against fixed bounds (`regress_check.py`). ~12 min at 1×;
  exit status 0 only if every check passes. Run it before merging.

Replay at 0.5× and 1× and compare: if a change helps at 0.5× but not 1×, it is
a compute or latency problem; if it fails at both, it is the algorithm or the
configuration. Run repeats — run-to-run variance at 1× is large enough that a
single run misled this investigation more than once.

## Part E — Open

- **The remaining under-estimate (B2, B3)**: on `hallway2`, 0–4% on the legs out, 7–11% on the legs back.
- **`T_b_c` after the mount swap (B3).** Corrected on paper for the new
  rectification; not yet validated on a capture with real rotation.
- **Z-drift** (on the pre-swap rig). Closure along the optical axis was −0.41 to −0.79 m at 1× and
  −0.29 m at 0.5×, against a few cm in x. Map age drives it (C14), and the flat
  wall makes z the softest direction to absorb error. Untested: whether a scene
  with real depth variation removes it. That is the next bag to record.
- **Remaining latency.** Map age is ~150 ms at 1× against 65–78 ms at 0.5×:
  mapping's cycle (~60 ms) plus surface delivery (~70 ms). No GPU work is
  justified until the z-drift is understood.
- **`USE_IMU` on the rig** is untested since the fix (A5). The rig's IMU-to-camera
  lever arm is uncalibrated and its event stream lags the IMU by ~5.5 s in the
  recorded bags, so it needs checking before it is trusted there.
- **MVSEC's IMU mode** remains non-deterministic run to run.
