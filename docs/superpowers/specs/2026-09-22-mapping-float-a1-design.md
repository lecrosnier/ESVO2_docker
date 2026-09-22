# Mapping in single precision without per-event allocation (A1, CPU)

Date: 2026-09-22
Status: design approved section by section, awaiting spec review
Builds on: `2026-09-18-gyro-locked-tracking-design.md`; first step of the
GPU programme below.

## Problem

On the live stereo EVK4 rig, mapping is the real-time bottleneck. On
`slide4_bias.bag` replayed at 1×, gyro lock on, regularisation off:

| Candidates per cycle | Mapping cycle | Tracking, 1 m out-and-back slide |
|---|---|---|
| 10 000 | 80 ms | 41–45% of true translation |
| 4 000 (current config, `9b9f6f7`) | 52 ms | 63–76% per leg, 3 runs |

The budget is 40 ms (`ts_rate` 25). At 4 000 candidates, 1× tracking now
matches what 0.25× replay gave with 10 000, so real time is reached, but with
no headroom: faster motion or a denser map pushes it back over budget. At
10 000 candidates the profile was BM 25%, variance 21%, fusion 17%
(serial) and regularisation 31% (serial, now off).

Two properties of the code waste most of the BM and variance time:

- **Everything is double precision.** `TimeSurfaceObservation` stores the
  surfaces as `Eigen::MatrixXd`. The laptop's GPU (NVIDIA TU106) runs FP64 at
  1/32 rate, so any GPU port needs float anyway.
- **Block matching allocates and copies per event.** `EventBM::match_an_event2`
  (`src/core/EventBM.cpp:165-226`) copies the source patch into a heap
  `MatrixXd`, zero-allocates a destination patch, copies a 7 × (15 + up to
  320) strip of the right time surface, and builds fresh `std::vector`s. At
  4 000 events per cycle that is tens of thousands of heap allocations and
  ~70 MB of copying per cycle. `DepthProblemSolver::init_single_point`
  (`src/core/DepthProblemSolver.cpp:186`) allocates its residual vectors per
  point (`:207`, `:216`).

## The programme (approach A) and where A1 sits

Four sub-projects, in order, each separately shippable and each able to stop
the programme:

| # | Sub-project | Delivers | Gate to go on |
|---|---|---|---|
| **A1** | Single precision + no per-event allocation (CPU) — **this spec** | faster BM and variance, no new toolchain | measured speedup, equivalence holds |
| A2 | GPU feasibility spike | yes/no: can this container run CUDA kernels, at what copy cost | a kernel runs; copy cost ≪ stage cost |
| A3 | Variance stage on GPU | first real kernel, full host↔device path | beats A1's CPU time for that stage |
| A4 | Block matching on GPU | the largest kernel | beats A1's CPU time for BM |

A2–A4 get their own specs, written with A1's numbers in hand.

**Programme target:** headroom. Mapping comfortably under 40 ms at 4 000
candidates, leaving room for faster motion or a return towards 10 000.

## Goal and scope of A1

Make block matching and the depth/variance solve faster on the CPU, with
unchanged algorithms and unchanged threading, and build the measurement
harness that A1 through A4 all use.

**Not touched:** tracking (and its double-precision members of
`TimeSurfaceObservation`), fusion, regularisation, the temporal SSIM path
(OpenCV, stays double), thread counts (`NUM_THREAD_MAPPING` stays 4), and
any algorithm change.

## Design

### 1. Float mirrors of the mapping surfaces

`TimeSurfaceObservation` gains `Eigen::MatrixXf` members mirroring **every
surface the mapping path reads**: `TS_left_`, `TS_right_`, `AA_map_`,
`TS_last_`, `TS_last_du`, `TS_last_dv` (the last two are read by
`DepthProblem.cpp:220`). The plan confirms this list against the code; any
further surface read by `EventBM` or `DepthProblem` is added to it.

- The mirrors are filled from the incoming `cv::Mat` when the mapping node
  builds the observation, and only when `MAPPING_FLOAT` is true.
- The double members stay, so the tracker (`RegProblemLM.cpp:169, 227`) is
  unchanged.
- Cost: one 1280×720 float conversion per mirrored surface per cycle.

### 2. No per-event allocation

- Source and destination patches become Eigen `Block` views into the float
  surfaces instead of copies.
- The right-hand search strip is read in place, not copied.
- Each BM thread owns fixed-size scratch buffers, sized once from the config
  (patch sizes, `BM_max_disparity`, `BM_step`), and reuses them for every
  event. This includes the per-event `std::vector`s
  (`searching_or_not`, `searching_radius`, `costs`, `fine_search`).
- `init_single_point` reuses a per-thread residual buffer allocated once.

The ZNCC arithmetic, coarse/fine search order, thresholds and bounds/mask
checks are unchanged.

### 3. Runtime switch

`MAPPING_FLOAT: True/False` in the mapping YAML, **default `False`** until
A1 is accepted. Both paths live in the same binary, so the equivalence test
compares them on identical input, and rollback is a config edit. After
acceptance, a follow-up commit sets it `True` in
`mapping_evk4_AA_mapping.yaml` and deletes the double BM/depth path; the
duplication is temporary.

### 4. Golden capture

An opt-in dump mode in the mapping node (off by default, enabled by a param
naming the output directory) writes, for **every 20th cycle** of the
`slide4_bias.bag` replay:

- the inputs to BM and the depth solve: mirrored surfaces (stored as float),
  candidate events, poses and the stereo calibration;
- the double-precision outputs as reference: match pairs (accepted and
  rejected, with disparity and cost) and depth points (inverse depth,
  variance, residual);
- the config values that affect the result (patch sizes, candidate count,
  disparity range, `BM_step`, ZNCC threshold).

About 35 cycles, ~0.5 GB, stored under `/root/datasets/evk4/golden/`,
outside the repo. Capturing every cycle would be ~20 GB.

## Validation and acceptance

**Equivalence test** (offline gtest in `esvo2_core/test/`, no ROS runtime,
deterministic; reads the golden capture, skips with a message if absent):

- **Block matching:** float vs double reference on each captured cycle.
  Pass if **≥ 99% of events make the same accept/reject decision**, and
  events matched by both agree on disparity within **1 px** and on inverse
  depth within **0.1% relative**. Not "identical", because ZNCC near its 0.2
  threshold can legitimately flip under float rounding.
- **Depth solve:** on the *same* match pairs, variance and residual agree
  within **0.1% relative** for **≥ 99.9% of points**. No discrete decisions
  here, so the bound is tighter.
- The test prints the distribution of differences (median, p99, max, count
  of flips), not only pass/fail.
- If the capture's config differs from the one under test, the test fails
  with the mismatching keys instead of comparing.

**Benchmark** (same harness, separate executable): median and p90 time per
stage (BM, depth solve), double vs float, over all captured cycles, repeated
5×.

**Tracking confirmation:** three 1× replays of `slide4_bias.bag` with
`MAPPING_FLOAT: True`, lock on, evaluated with the same still-period
round-trip metric that produced the double path's band (`eval_roundtrip.py`,
currently a scratch script; A1 moves it into `esvo2_core/scripts/`). Pass if
every leg lies within the 63–76% band the double path produced.

**A1 is accepted when** the equivalence test and the tracking confirmation
pass and BM + variance time drops measurably on the benchmark. If the
speedup is negligible, A1 still stands as groundwork (float storage is a
prerequisite for A3/A4), and its numbers feed the A2 decision.

## Risks

- **Float flips near the ZNCC threshold** change which points enter the map.
  Bounded by the 99% decision criterion and checked end to end by the
  tracking confirmation.
- **Eigen `Block` views on column-major storage:** strides differ from the
  copied patches. Covered by the equivalence test; no layout change to the
  surfaces themselves.
- **Extra conversion cost** of the mirrors could eat part of the gain. The
  benchmark measures the conversion as its own line.
