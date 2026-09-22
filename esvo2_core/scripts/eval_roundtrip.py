#!/usr/bin/env python3
"""Evaluate an out-and-back slide replay, in ABSOLUTE timestamps.

The event and IMU streams in these bags do not start together (events lag the
IMU by ~5.5 s), so anything that mixes IMU-relative and pose-relative time
misassigns the motion segments. Everything here uses header stamps.

Still periods come from the gyro; displacement is the difference of the mean
pose over consecutive still periods, so per-frame jitter averages out.
usage: eval_roundtrip.py POSE_BAG IMU_BAG [min_leg_m]
"""
import sys, numpy as np, rosbag

MINLEG = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

def imu_stills(bag):
    r = []
    for _, m, _ in rosbag.Bag(bag).read_messages(topics=["/imu/data_synced"]):
        g = m.angular_velocity
        r.append((m.header.stamp.to_sec(), g.x, g.y, g.z))
    d = np.array(r)
    t0, t1 = d[0, 0], d[-1, 0]
    w = 0.5
    flags = []
    for s in np.arange(t0, t1 - w, w):
        m = (d[:, 0] >= s) & (d[:, 0] < s + w)
        if m.sum() >= 5:
            flags.append((s, d[m, 1:4].std(0).max() < 0.0035))
    stills, cur = [], None
    for s, st in flags:
        if st and cur is None: cur = s
        elif not st and cur is not None:
            if s - cur > 1.5: stills.append((cur, s))
            cur = None
    if cur is not None and t1 - cur > 1.5: stills.append((cur, t1))
    return stills

def poses(bag):
    r = []
    for _, m, _ in rosbag.Bag(bag).read_messages(topics=["/esvo2_tracking/pose_pub"]):
        p = m.pose.position
        r.append((m.header.stamp.to_sec(), p.x, p.y, p.z))
    return np.array(r)

st = imu_stills(sys.argv[2]); P = poses(sys.argv[1])
if len(P) == 0: print("no poses"); sys.exit(1)
print("pose span %.1f s, %d poses; %d still periods" % (P[-1,0]-P[0,0], len(P), len(st)))
means, labels = [], []
for a, b in st:
    m = (P[:, 0] >= a) & (P[:, 0] <= b)
    if m.sum() > 2:
        means.append(P[m, 1:4].mean(0)); labels.append((a - P[0,0], b - P[0,0], m.sum()))
    else:
        means.append(None); labels.append((a - P[0,0], b - P[0,0], 0))
for i in range(len(means) - 1):
    if means[i] is None or means[i+1] is None: continue
    d = means[i+1] - means[i]
    if np.linalg.norm(d) < MINLEG: continue
    print("  still[%5.1f-%5.1f s] -> still[%5.1f-%5.1f s]: dx %+.3f dy %+.3f dz %+.3f  |d| %.3f m" %
          (labels[i][0], labels[i][1], labels[i+1][0], labels[i+1][1], d[0], d[1], d[2], np.linalg.norm(d)))
ok = [m for m in means if m is not None]
if len(ok) > 1:
    c = ok[-1] - ok[0]
    print("  closure: dx %+.3f dy %+.3f dz %+.3f  |c| %.3f m" % (c[0], c[1], c[2], np.linalg.norm(c)))
