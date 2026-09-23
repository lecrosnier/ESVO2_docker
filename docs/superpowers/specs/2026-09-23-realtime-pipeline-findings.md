# Reaching paper-level tracking on the EVK4 rig: what was wrong, 2026-09-22/23

Audience: anyone working on this fork. It records two investigations — first
whether this code can reproduce the paper at all, then why it behaved so much
worse on the live rig — with the measurements behind each conclusion, and the
hypotheses that turned out to be wrong.

Branch: `evk4-ts-rate-aa-window`, seven commits on top of
`evk4-gyro-locked-tracking`.

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
| Ours, IMU in tracking | diverges | — |

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

### A5. ESVO2's IMU tracking path diverges (not fixed)

With the IMU actually connected, tracking diverges on MVSEC: positions reach
10^5 m and the map resets hundreds of times. The mapping back end alone is
harmless (ATE 9.0 cm vs 7.9 cm vision-only), so the problem is the tracking-side
prediction in `esvo2_Tracking::curImuTransferring` / the `bUseImu_` branch:

- `Imu_t` is the preintegrated `delta_p`, integrated from raw accelerometer
  readings with gravity left in (`imu_integration.h` never subtracts `G` there);
- it is added in the body/camera frame, never rotated into the world frame;
- it is added on top of a constant-velocity term (`last_t_`), whose own errors
  feed back into the next frame.

The IMU-to-camera rotation is not the cause: fitted against ground-truth
angular velocity, it is within 3° of identity, which is what `calib/upenn`
declares. This is upstream behaviour, unmodified by this fork, and it is why
`USE_IMU: False` remains right for the rig.

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

Replay at 0.5× and 1× and compare: if a change helps at 0.5× but not 1×, it is
a compute or latency problem; if it fails at both, it is the algorithm or the
configuration. Run repeats — run-to-run variance at 1× is large enough that a
single run misled this investigation more than once.

## Part E — Open

- **Z-drift.** Closure along the optical axis is −0.41 to −0.79 m at 1× and
  −0.29 m at 0.5×, against a few cm in x. Map age drives it (C14), and the flat
  wall makes z the softest direction to absorb error. Untested: whether a scene
  with real depth variation removes it. That is the next bag to record.
- **Remaining latency.** Map age is ~150 ms at 1× against 65–78 ms at 0.5×:
  mapping's cycle (~60 ms) plus surface delivery (~70 ms). No GPU work is
  justified until the z-drift is understood.
- **The IMU tracking path** (A5) is still broken upstream and unused here.
- **MVSEC's IMU mode** remains non-deterministic run to run.
