#!/usr/bin/env python3
"""Summarise a tracker MOTION_LOG (launch arg motion_log:=file.csv).

One CSV row per tracking frame: the hold decision (TRACK / HOLD / COAST / RESET)
and its inputs -- reference map size, IMU state and statistics, time-surface
activity (fresh, support), J^T J/N smallest eigenvalue (info), rotation lock,
published position. Prints the decision counts, the inputs' distributions per
decision, and a timeline of every run of one decision lasting min_frames or more.

usage: motion_log_summary.py MOTION_LOG.csv [min_frames]
"""
import collections
import sys

import numpy as np


def load(path):
    rows = [line.rstrip("\n").split(",") for line in open(path)]
    head, body = rows[0], rows[1:]
    d = {}
    for i, k in enumerate(head):
        col = [r[i] for r in body]
        d[k] = np.array(col) if k in ("action", "imu") else np.array(col, float)
    d["t"] = d["t"] - d["t"][0]
    return d


d = load(sys.argv[1])
min_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 25
print("frames by (decision, IMU):", sorted(collections.Counter(zip(d["action"], d["imu"])).items()))
print("IMU sample age at the frame, p50/p99/max (s):", np.round(np.quantile(d["imu_age"], [.5, .99, 1]), 3))
for act in ("TRACK", "HOLD", "COAST", "RESET"):
    for imu in ("STILL", "MOVING", "UNKNOWN"):
        m = (d["action"] == act) & (d["imu"] == imu)
        if m.sum() == 0:
            continue
        info = d["info"][m & (d["info"] >= 0)]
        q = "%.3g / %.3g / %.3g" % tuple(np.quantile(info, [.05, .5, .95])) if len(info) else "-"
        print("%-5s %-7s n=%5d  info p5/p50/p95 %s  support p50 %.2f  map pts p50 %d"
              % (act, imu, m.sum(), q, np.median(d["support"][m]), np.median(d["ref_pts"][m])))
a = d["action"]
cuts = np.r_[0, np.where(a[1:] != a[:-1])[0] + 1, len(a)]
print("timeline (runs of >= %d frames):" % min_frames)
for s, e in zip(cuts[:-1], cuts[1:]):
    if e - s >= min_frames:
        imu = collections.Counter(d["imu"][s:e]).most_common(1)[0][0]
        print("  %-5s %7.1f-%7.1f s  n=%5d  IMU mostly %-7s z %+.2f -> %+.2f"
              % (a[s], d["t"][s], d["t"][e - 1], e - s, imu, d["z"][s], d["z"][e - 1]))
