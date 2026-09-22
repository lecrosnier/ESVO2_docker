# A1 results: float static mapping path

Date: 2026-09-22. Spec: `2026-09-22-mapping-float-a1-design.md`. Commits: `2f279fd`..`2044f19` (this working session's HEAD; results doc and eval script commit follows).

## Equivalence (golden capture: 51 cycles of slide4_bias.bag, every 20th)

Command: `/root/catkin_ws/devel/lib/esvo2_core/test_mapping_equivalence 2>&1 | grep -E "\[golden|PASSED|FAILED"`

```
[golden] 51 cycles, 83081 matches and 83081 depth points reproduced
[golden BM] 171189 events, same decision 171189 (100.0000%), matched by both 83081
[golden BM] |d disparity| px: p50 0 p99 0 max 0
[golden BM] rel d invDepth: p50 0 p99 0 max 0
[golden depth] 83081 points, within 0.1%: 83081 (100.0000%)
[golden depth] rel d variance: p50 0 p99 0 max 0
[golden depth] rel d residual: p50 0 p99 0 max 0
[  PASSED  ] 3 tests.
```

- Harness reproduces the node exactly: 51 cycles, 83081 matches, 83081 depth points — pass.
- Static BM: same decision 100.0000% (need ≥ 99%); disparity |d| max 0 px (need ≤ 1); invDepth rel max 0 (need ≤ 0.1%) — pass.
- Static depth solve: within 0.1%: 100.0000% of 83081 points (need ≥ 99.9%); variance rel p99/max 0/0 — pass.

## Benchmark (5 repetitions, idle machine)

51 cycles x 5 repetitions; 171189 candidate events, 83081 matches per repetition

| stage | double median | double p90 | float median | float p90 | speedup (median) |
|---|---|---|---|---|---|
| static BM (ms) | 13.22 | 21.20 | 10.17 | 15.04 | 1.30x |
| static depth solve (ms) | 12.31 | 14.29 | 11.73 | 13.47 | 1.05x |
| float mirror refresh (ms, inside float BM) | - | - | 1.05 | 1.12 | - |

paired per-cycle speedup, static BM: median 1.33x, p10 1.12x, p90 1.44x
paired per-cycle speedup, static depth solve: median 1.07x, p10 1.02x, p90 1.12x

## Tracking (1x replay, MAPPING_FLOAT: True)

Ruling R1 (binding, supersedes the brief's single-band pass criterion): report the verdict twice —
(a) against the spec band, every leg 63–76% of the true 1 m;
(b) against the double path's spread measured 2026-09-22 with the same code/bag/configs: back leg |dx| 0.564–0.673 m, out leg (runs without a late start) 0.638–0.764 m, closures 0.43–1.10 m.

3 required runs were done, plus 2 reruns (the brief's cap) because every run showed an incomplete leg pair. All 5 runs are reported below. Legs are identified as the two largest-magnitude still-to-still transitions per the brief; `dx` is the slide axis (matches the sign convention of the reference numbers in R1). "late start?" flags a tiny (~0.11–0.16 m) leading transition and pose data starting only ~26 s into the 81 s source bag, matching the brief's documented late-start signature.

| run | out leg (dx / %) | back leg (dx / %) | closure |d| | resets | late start? | note |
|---|---|---|---|---|---|---|
| float_run1 | -0.633 / 63.3% | +0.023 / 2.3% | 0.495 m | 0 | yes | back leg not recovered (near-zero) |
| float_run2 | -0.534 / 53.4% | -0.002 / 0.2% | 0.713 m | 0 | yes | out leg also has a large dz (-0.634 m), non-planar drift; back leg not recovered |
| float_run3 | -0.677 / 67.7% | n/a | 0.610 m* | 0 | yes | back-leg still windows have too few poses to compute a mean; no measurement at all |
| float_run4 | -0.639 / 63.9% | +0.035 / 3.5% | 0.524 m | 0 | yes | back leg not recovered (near-zero) |
| float_run5 | -0.724 / 72.4% | -0.024 / 2.4% | 0.708 m | 0 | yes | back leg not recovered (near-zero) |

\* run3's printed "closure" is only the first-valid-mean-to-last-valid-mean difference (2 valid still means past the out leg), not a true start-to-end loop closure, because the trailing still windows lack pose coverage.

Raw eval outputs (for the record):

```
float_run1: pose span 55.1 s, 1378 poses; 6 still periods
  still[  6.5- 10.5 s] -> still[ 12.0- 22.5 s]: dx +0.116 dy -0.019 dz +0.073  |d| 0.139 m
  still[ 12.0- 22.5 s] -> still[ 32.0- 48.0 s]: dx -0.633 dy +0.007 dz +0.055  |d| 0.635 m
  still[ 32.0- 48.0 s] -> still[ 48.5- 55.2 s]: dx +0.023 dy -0.028 dz -0.111  |d| 0.117 m
  closure: dx -0.493 dy -0.040 dz +0.017  |c| 0.495 m

float_run2: pose span 54.6 s, 1364 poses; 6 still periods
  still[  6.0- 10.0 s] -> still[ 11.5- 22.0 s]: dx +0.114 dy +0.009 dz +0.025  |d| 0.117 m
  still[ 11.5- 22.0 s] -> still[ 31.5- 47.5 s]: dx -0.534 dy +0.015 dz -0.634  |d| 0.829 m
  still[ 31.5- 47.5 s] -> still[ 48.0- 54.7 s]: dx -0.002 dy +0.011 dz +0.036  |d| 0.038 m
  closure: dx -0.423 dy +0.035 dz -0.573  |c| 0.713 m

float_run3: pose span 55.6 s, 1389 poses; 6 still periods
  still[  6.9- 10.9 s] -> still[ 12.4- 22.9 s]: dx +0.092 dy -0.005 dz -0.125  |d| 0.155 m
  still[ 12.4- 22.9 s] -> still[ 32.4- 48.4 s]: dx -0.677 dy +0.035 dz -0.065  |d| 0.681 m
  closure: dx -0.578 dy +0.046 dz -0.191  |c| 0.610 m

float_run4: pose span 55.6 s, 1391 poses; 6 still periods
  still[  7.0- 11.0 s] -> still[ 12.5- 23.0 s]: dx +0.108 dy -0.001 dz +0.043  |d| 0.117 m
  still[ 12.5- 23.0 s] -> still[ 32.5- 48.5 s]: dx -0.639 dy +0.071 dz -0.162  |d| 0.664 m
  still[ 32.5- 48.5 s] -> still[ 49.0- 55.7 s]: dx +0.035 dy -0.005 dz -0.039  |d| 0.053 m
  closure: dx -0.495 dy +0.065 dz -0.158  |c| 0.524 m

float_run5: pose span 55.7 s, 1393 poses; 6 still periods
  still[  7.1- 11.1 s] -> still[ 12.6- 23.1 s]: dx +0.106 dy +0.042 dz -0.011  |d| 0.114 m
  still[ 12.6- 23.1 s] -> still[ 32.6- 48.6 s]: dx -0.724 dy -0.025 dz -0.253  |d| 0.768 m
  still[ 32.6- 48.6 s] -> still[ 49.1- 55.7 s]: dx -0.024 dy +0.001 dz -0.033  |d| 0.041 m
  closure: dx -0.643 dy +0.018 dz -0.298  |c| 0.708 m
```

Late-start count: 5/5 runs show the late-start signature (tiny leading transition, first pose ~26 s into the 81 s bag). The 2026-09-22 double-path reference had late starts in 6/8 runs (75%); this sample (5/5, 100%) is higher but too small (n=5) to call a clear regression on its own — see Findings/Concerns. It is not, by itself, the decisive tracking problem here: the out leg is recovered close to the reference band in 4/5 runs despite the late start. The back leg failing in *every* run is a distinct and new pattern, not explained by the known late-start issue (which affects the *first* leg, not the last).

## Verdict

**(a) Against the spec band (63–76% every leg): FAIL.**
Out legs: 4/5 in band (63.3%, 67.7%, 63.9%, 72.4%); run2 below band at 53.4% (and additionally shows a large non-planar dz component, -0.634 m, not seen in the other runs). Back legs: 0/5 in band — every measured back leg is 0.2–3.5% (near zero), and run3 produced no back-leg measurement at all. The spec's per-leg 63–76% criterion is not met.

**(b) Against the double path's 2026-09-22 spread (back |dx| 0.564–0.673 m; out |dx| 0.638–0.764 m for non-late-start runs; closures 0.43–1.10 m): FAIL, but for a different reason than (a).**
Out legs: 3/5 fall inside the reference band (0.677, 0.639, 0.724 → all within 0.638–0.764); run1 (0.633) is just under the band by 0.005 m; run2 (0.534) is well under, and its motion is contaminated by a large dz. So the out leg is broadly consistent with the double path's historical spread, allowing for one likely-late-start run and one anomalous run. Back legs, however, are not a "falls outside the range" situation: they are 0.002–0.035 m in magnitude (essentially zero, i.e., not recovered at all), against a reference band whose *floor* is 0.564 m. No float run produced a back-leg measurement anywhere close to the double path's historical spread, even loosely. Closures (0.495–0.713 m) do fall inside the double path's 0.43–1.10 m range, but that is expected given closure is dominated by the (correctly sized) out leg and an unreturned back leg.

**Both verdicts are FAIL, and for the same underlying reason: the back leg is not being tracked/recovered in any of the 5 runs**, not merely landing outside a numeric band. This is a different, apparently new failure mode from the previously known "late start misses the first leg" issue (documented in the double-path baseline and reproduced here too — 5/5 runs show the late-start signature on the *first* leg but still recover a first "out" leg of plausible size). The double path's own reference numbers on 2026-09-22 (back leg 0.564–0.673 m across 5 tlast + 3 earlier runs) show the double path *can* recover the back leg reliably; the float runs here could not, in 5/5 attempts.

Context: the double path itself fell short of the spec's 63–76% band on 2026-09-22 (its measured legs were 0.564–0.673 m = 56.4–67.3% and 0.638–0.764 m = 63.8–76.4%, i.e. some legs below 63%), which is why R1 substitutes the double path's own measured spread as the practical bar. The float path fails even that more permissive bar, because it is missing an entire leg rather than measuring one imprecisely.

Equivalence and benchmark results are unaffected by this: the static BM and depth-solve paths are bit-for-bit equivalent to the double path on the golden capture, and the float path is 1.30x/1.05x faster (median) on the two stages it touches. Tracking uses the mapped point cloud but no tracking code was touched (per the global constraints); the tracking failure observed here is not explained by anything in the equivalence data, and is most plausibly either an artifact of this run's session (rig, timing, event backlog) or a marginal effect of the float path's faster/different-shaped mapping output on downstream tracking stability that the offline equivalence test cannot see (it only checks per-cycle BM/depth-solve agreement, not full closed-loop tracking dynamics).

For the A2 decision: the equivalence and speed results support proceeding with A2 as planned (correctness is exact, speed gain is real but modest — 1.30x median on static BM, only 1.05x on static depth solve, so A2's further speedups matter more for depth solve than for BM). The tracking verdict should not be read as a regression caused by MAPPING_FLOAT without further investigation: it is unclear whether double-path runs collected in the same session under the same rig/timing conditions would have fared any differently, since the double path's own 2026-09-22 back-leg values were themselves already below the spec band. The user should decide whether to (1) collect a same-session double-path comparison run before ruling this a float-specific regression, or (2) investigate the back-leg-missing pattern directly (e.g. whether SGM re-initializes correctly after the sustained far-position dwell, or whether the point cloud composition changes enough under MAPPING_FLOAT to affect tracking's re-lock on the return leg) before flipping the default.

## Findings

1. `T_last_now_` in the temporal depth solve was read uninitialised and got the right value only via heap reuse; fixed in `2f279fd` before A1.
2. `dynamic_reconfigure` clamped the `BM_max_disparity` param on the param server to 150 (cfg bound), corrupting only the capture metadata; bound raised to 500 in `b1ced99`.
3. Upstream quirk in `EventBM::epipolarSearchingCoarse`: its marking loop mixes `int` and `size_t`, so when the first coarse candidate passes, no neighbouring disparities are marked for the fine search; the float path reproduces it deliberately (`e4744d7`); fixing it is a behaviour change left for the user.
4. (New, from this task) All 5 tracking replay runs with `MAPPING_FLOAT: True` recovered the out leg (4/5 within the spec band, 3/5 within the double path's reference band) but failed to recover the back leg in every run (0/5, magnitudes 0.2–3.5% of 1 m vs. a reference floor of 56.4%); run3 produced no back-leg measurement at all because the trailing still windows had too few poses. This was not anticipated by the brief's late-start rerun rule (which targets a missed *first* leg) and persisted through both allowed reruns, so it is reported as-is rather than papered over with more reruns.

## Concerns

- No same-session double-path (`MAPPING_FLOAT: False`) comparison run was collected today, so it cannot be said with certainty whether the back-leg-missing pattern is specific to the float path or would also appear on the double path under today's conditions (bag, rig state, timing all otherwise held constant per the task's setup). This is the single most important gap before treating the tracking failure as float-specific.
- Late-start rate this session (5/5) is nominally higher than the 2026-09-22 double-path reference (6/8), but n=5 is too small to distinguish from the known noise, and the out leg was still recovered in 4/5 of these late-start runs, so per the controller ruling this is reported as a line item, not scored as a regression.
- run2's out leg has a substantial dz component (-0.634 m) not seen in the other 4 runs, suggesting a possibly different failure mode (drift, not just a late start) in that specific run.
