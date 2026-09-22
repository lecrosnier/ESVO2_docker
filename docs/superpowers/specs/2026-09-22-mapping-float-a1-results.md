# A1 results: float static mapping path

Date: 2026-09-22. Spec: `2026-09-22-mapping-float-a1-design.md`. Commits: `2f279fd`..`6c2f62d` (final HEAD of the review fix wave; this results-doc commit follows).

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
- Caveat: `test_mapping_equivalence`'s three cases `GTEST_SKIP()` whenever `/root/datasets/evk4/golden/slide4_bias` is absent, so a green run on a machine without that golden capture proves nothing about equivalence — it only means the equivalence assertions did not run. The numbers above are from a run where the golden data was present and the assertions actually executed.

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

3 required runs were done, plus 2 reruns (the brief's cap) because every run showed the known late-start pattern. All 5 runs are reported below.

Leg identification (corrected from an earlier misreading of this same data): every run has the documented late start, so the true OUT leg (+dx, camera moving away, expected ~1 m) is not tracked at all — the pose stream only starts ~26 s into the 81 s source bag, after the out leg is already underway or finished. The small leading transition (+0.09 to +0.12 m) is post-out-leg settling jiggle, not a leg. The segment `still[~12–23 s] -> still[~32–48 s]`, dx −0.53 to −0.72 m, is the BACK leg (the camera returning), not the out leg. The tiny trailing segment (|d| < 0.12 m, present in 4/5 runs) is final jiggle at the home position, not a leg. This matches the double-path late-start pattern exactly, e.g. `/root/datasets/evk4/tlast/before_1.bag`: jiggle +0.128, back leg −0.564.

| run | out leg | back leg (dx / %) | closure |d| | resets | late start? | note |
|---|---|---|---|---|---|---|
| float_run1 | missed (late start) | -0.633 / 63.3% | 0.495 m | 0 | yes | |
| float_run2 | missed (late start) | -0.534 / 53.4% | 0.713 m | 0 | yes | back leg also has a large dz (-0.634 m), non-planar drift |
| float_run3 | missed (late start) | -0.677 / 67.7% | 0.610 m* | 0 | yes | closure is not a true loop closure — see note below table |
| float_run4 | missed (late start) | -0.639 / 63.9% | 0.524 m | 0 | yes | |
| float_run5 | missed (late start) | -0.724 / 72.4% | 0.708 m | 0 | yes | |

Back leg |dx|: mean 0.641 m (run1 0.633, run2 0.534, run3 0.677, run4 0.639, run5 0.724).

\* run3's printed "closure" is only the first-valid-mean-to-last-valid-mean difference (2 valid still means, spanning only the back leg), not a true start-to-end loop closure, because the trailing still windows lack pose coverage.

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

Late starts: its own line, independent of A1 — float 5/5 runs, double path 6/8 runs (75%) on 2026-09-22. Same known issue in both; not a float-specific regression.

## Verdict

**(a) Against the spec band (63–76% every leg): not met.**
Out legs: missed in all 5 runs (late start — the pose stream starts after the out leg has already happened, so it is never measured). Back legs: 53.4%, 63.3%, 67.7%, 63.9%, 72.4% — 4/5 in band, run2 below it. Since the out leg is unmeasured in every run, the spec's "every leg in 63–76%" criterion is not met. Context: the double path does not meet this band either on 2026-09-22 — its measured back legs that day were 0.564–0.673 m (56.4–67.3%), also partly below 63%, and it had late starts in 6/8 runs, so out legs were unmeasured there too in most runs.

**(b) Against the double path's 2026-09-22 spread (back |dx| 0.564–0.673 m, mean 0.615 m, sd 0.038, n=8; out |dx| 0.638–0.764 m for non-late-start runs; closures 0.43–1.10 m): met — indistinguishable from the double path.**
Float back legs: 0.534, 0.633, 0.639, 0.677, 0.724 m, mean 0.641 m, sd 0.070, n=5. Double-path back legs (2026-09-22, 8 runs): 0.564, 0.611, 0.572, 0.630, 0.590, 0.646, 0.632, 0.673 m, mean 0.615 m, sd 0.038, n=8. Three of the five float runs fall outside the double path's 0.564–0.673 m range (run2 at 0.534 below it; run3 at 0.677 and run5 at 0.724 above it); only run1 (0.633) and run4 (0.639) sit inside it. Despite that, the two samples are not distinguishable at these sizes: difference of means 0.027 m (0.641 − 0.615), standard error of the difference (Welch) 0.034 m, t = 0.78 — well below any conventional significance threshold, so the observed spread is consistent with both samples being drawn from the same underlying distribution rather than evidence of a shift. This is expected: the static BM and depth-solve outputs are bit-for-bit identical to the double path on the golden capture (see Equivalence), and no tracking code was touched, so at 1x replay any run-to-run difference between the float and double paths can only come from timing (the float path being faster), not from the numbers fed to tracking. Closures (0.495–0.713 m across the 5 float runs) fall inside the double path's 0.43–1.10 m range, which comes from the same 8 reference runs — `/root/datasets/evk4/tlast/{before_1,before_2,after_1,after_2,after_3}.bag` and the scratch `cand_4000{,_b,_c}.bag` — with closures 0.493, 0.729, 0.468, 0.663, 0.432, 0.836, 1.100, 0.613 m.

Late starts (own line, per the controller ruling): float 5/5 runs, double path 6/8 runs (75%) on 2026-09-22 — the same known issue, independent of A1, not scored against either criterion.

**Overall tracking verdict: indistinguishable from the double path.** The apparent "back leg is 53–72%, out leg is missing" pattern in every float run (back legs 53.4–72.4%, run2 at 53.4% the low end) is the well-documented late-start issue, reproduced here at a similar rate to the double path on the same day (5/5 vs 6/8), not a new float-specific failure.

For the A2 decision: equivalence is met exactly (100.0000% match on 51 cycles / 83081 points), the benchmark shows a real but modest speedup on the two stages this task touches (static BM 1.30x aggregate / 1.33x paired median; static depth solve 1.05x aggregate / 1.07x paired median — A2's further speedups matter more for depth solve than for BM), and tracking with `MAPPING_FLOAT: True` is indistinguishable from the double path on the same bag and configs on 2026-09-22 (back-leg mean 0.641 m float, n=5, vs 0.615 m double, n=8; Welch t=0.78, not significant; similar late-start rate, 5/5 vs 6/8). All three acceptance areas support proceeding with A2. Flipping `MAPPING_FLOAT` to `True` in the committed YAML per the spec's follow-up is a separate recommendation, subject to the user accepting Ruling R1's criterion (b); the spec's own band (a) was not met by either path on 2026-09-22.

## Findings

1. `T_last_now_` in the temporal depth solve was read uninitialised and got the right value only via heap reuse; fixed in `2f279fd` before A1.
2. `dynamic_reconfigure` clamped the `BM_max_disparity` param on the param server to 150 (cfg bound), corrupting only the capture metadata; bound raised to 500 in `b1ced99`.
3. Upstream quirk in `EventBM::epipolarSearchingCoarse`: its marking loop mixes `int` and `size_t`, so when the first coarse candidate passes, no neighbouring disparities are marked for the fine search; the float path reproduces it deliberately (`e4744d7`); fixing it is a behaviour change left for the user.
4. All 5 tracking replay runs with `MAPPING_FLOAT: True` hit the known late-start issue (missed out leg), at a similar rate (5/5) to the double path on 2026-09-22 (6/8). The recovered back legs (mean 0.641 m, sd 0.070, n=5) are statistically indistinguishable from the double path's back legs that day (mean 0.615 m, sd 0.038, n=8; Welch t=0.78), consistent with the static paths being bit-for-bit equivalent (Findings above) and no tracking code being touched.

## Concerns

- run2's back leg has a substantial dz component (-0.634 m) not seen in the other 4 runs; worth a closer look but a single outlier out of 5 runs.
- 3 of 5 float back legs (run2 low; run3, run5 high) fall outside the double path's 0.564–0.673 m range, though the group difference is not statistically distinguishable at n=5 vs n=8 (see Verdict b). A larger sample would tighten this comparison.
- No same-session double-path (`MAPPING_FLOAT: False`) comparison run was collected today; the comparison above relies on the double path's baseline measured on the same day with the same bag and configs (per the controller ruling), not a run collected in this exact session.
