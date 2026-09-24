#!/usr/bin/env python3
"""Check regress.sh results against fixed bounds; exit 1 on any failure.

Bounds come from repeated runs on 2026-09-24 (branch evk4-stillness-hold) and
leave room for run-to-run spread, which at 1x is several centimetres:
  hallway2  stop-and-go, legs 2/2/3/3 m: each leg 0.85-1.10 of truth, closure <= 0.6 m, 0 resets
  hallway3  continuous, far point 2.00 m twice: both 1.80-2.20 m, back on the start mark within 0.3 m, 0 resets
  hallway4  1.00 m sideways and back: both legs 0.80-1.10 m, closure <= 0.3 m, 0 resets
  mvsec1    MVSEC indoor_flying1, paper window: ATE (SE3) <= 0.10 m, path ratio 0.90-1.05
usage: regress_check.py OUT_DIR DATA_DIR case...
"""
import os
import re
import subprocess
import sys

import numpy as np
import rosbag

OUT, DATA, CASES = sys.argv[1], sys.argv[2], sys.argv[3:]
SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
MVSEC1_WIN = "1504645187.57 1504645213.40"
results = []


def check(case, name, value, lo, hi):
    ok = lo <= value <= hi
    results.append((case, name, value, lo, hi, ok))


def resets(case):
    return open(os.path.join(OUT, case + ".log")).read().count("re-initialized")


def roundtrip(case, bag):
    out = subprocess.run([sys.executable, os.path.join(SCRIPTS, "eval_roundtrip.py"), os.path.join(OUT, case + ".bag"),
                          os.path.join(DATA, "evk4", bag), "0.3"], capture_output=True, text=True).stdout
    legs = [float(x) for x in re.findall(r"\|d\| ([0-9.]+)", out)]
    closure = float(re.findall(r"\|c\| ([0-9.]+)", out)[-1])
    return legs, closure


def poses(case):
    return np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z)
                     for _, m, _ in rosbag.Bag(os.path.join(OUT, case + ".bag")).read_messages(
                         topics=["/esvo2_tracking/pose_pub"])])


for case in CASES:
    try:
        if case == "hallway2":
            legs, closure = roundtrip(case, "hallway2.bag")
            for i, truth in enumerate((2.0, 2.0, 3.0, 3.0)):
                check(case, "leg %d / truth" % (i + 1), legs[i] / truth if i < len(legs) else 0.0, 0.85, 1.10)
            check(case, "closure (m)", closure, 0.0, 0.6)
            check(case, "tracking resets", resets(case), 0, 0)
        elif case == "hallway3":
            P = poses(case)
            z = P[:, 3] - P[0, 3]
            half = len(z) // 2
            check(case, "far point 1 (m)", z[:half].max(), 1.8, 2.2)
            check(case, "far point 2 (m)", z[half:].max(), 1.8, 2.2)
            check(case, "end from start (m)", float(np.linalg.norm(P[-20:, 1:].mean(0) - P[0, 1:])), 0.0, 0.3)
            check(case, "tracking resets", resets(case), 0, 0)
        elif case == "hallway4":
            legs, closure = roundtrip(case, "hallway4_lateral.bag")
            for i in range(2):
                check(case, "leg %d (m)" % (i + 1), legs[i] if i < len(legs) else 0.0, 0.8, 1.1)
            check(case, "closure (m)", closure, 0.0, 0.3)
            check(case, "tracking resets", resets(case), 0, 0)
        elif case == "mvsec1":
            gt = os.path.join(SCRIPTS, "..", "..", "results", "gt", "upenn", "stamped_groundtruth_of_indoorflying1.txt")
            out = subprocess.run([sys.executable, os.path.join(SCRIPTS, "eval_traj.py"), gt,
                                  os.path.join(OUT, case + ".bag"), case], capture_output=True, text=True,
                                 env=dict(os.environ, WIN=MVSEC1_WIN)).stdout
            check(case, "ATE SE3 (m)", float(re.search(r"ATE SE3 ([0-9.]+)", out).group(1)), 0.0, 0.10)
            check(case, "path ratio", float(re.search(r"ratio ([0-9.]+)", out).group(1)), 0.90, 1.05)
    except Exception as e:  # a missing or empty result is a failure, not a crash
        results.append((case, "evaluation: %s" % e, float("nan"), 0, 0, False))

for case, name, value, lo, hi, ok in results:
    print("%-4s %-9s %-22s %8.3f   [%g, %g]" % ("ok" if ok else "FAIL", case, name, value, lo, hi))
failed = sum(not r[-1] for r in results)
print("%d checks, %d failed" % (len(results), failed))
sys.exit(1 if failed else 0)
