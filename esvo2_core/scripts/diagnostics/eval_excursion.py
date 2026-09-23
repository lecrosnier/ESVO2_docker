#!/usr/bin/env python3
"""Score an out-and-back run that never stops mid-way.

The rig starts on a mark, rolls along its optical axis to a far point,
reverses without stopping, and so on, finishing back on the start mark. With
no still periods between legs the still-period metric (eval_roundtrip.py)
does not apply; the ground truth is the geometry instead: every far
turnaround is EXCURSION metres forward of the start, and the end coincides
with the start.

The tracker resets whenever the rig is at rest (a quiet event camera sees
nothing), so this scores the longest continuous track: no gap over 0.2 s and
no jump over 0.1 m between consecutive poses.

usage: eval_excursion.py POSES.bag [excursion_m]
"""
import sys

import numpy as np
import rosbag

bag_path = sys.argv[1]
excursion = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

P = np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z)
              for _, m, _ in rosbag.Bag(bag_path).read_messages(topics=["/esvo2_tracking/pose_pub"])])
if len(P) < 10:
    sys.exit("too few poses: %d" % len(P))

# split into continuous tracks
breaks = np.where((np.diff(P[:, 0]) > 0.2) | (np.linalg.norm(np.diff(P[:, 1:], axis=0), axis=1) > 0.1))[0] + 1
segments = np.split(np.arange(len(P)), breaks)
seg = max(segments, key=len)
S = P[seg]
t = S[:, 0] - S[0, 0]
xyz = S[:, 1:] - S[0, 1:]           # relative to where this track started
z = xyz[:, 2]                       # along the initial optical axis, + towards the wall

print("%d poses, %d continuous tracks; scoring the longest: %d poses over %.1f s"
      % (len(P), len(segments), len(S), t[-1]))

# turnarounds: extrema of a lightly smoothed z, separated by at least 2 s
k = max(1, int(len(z) / t[-1] * 0.3))   # ~0.3 s moving average
zs = np.convolve(z, np.ones(k) / k, mode="same")
ext = []
for i in range(1, len(zs) - 1):
    if (zs[i] >= zs[i - 1] and zs[i] > zs[i + 1]) or (zs[i] <= zs[i - 1] and zs[i] < zs[i + 1]):
        if not ext or t[i] - t[ext[-1]] > 2.0:
            ext.append(i)
        elif abs(zs[i]) > abs(zs[ext[-1]]):
            ext[-1] = i

far = [i for i in ext if zs[i] > excursion / 2]
near = [i for i in ext if zs[i] <= excursion / 2 and 0 < i < len(zs) - 1]
print("far turnarounds (truth %.2f m):" % excursion)
for i in far:
    print("   t %5.1f s   z %+.3f m   (%+5.1f%%)   lateral x %+.3f  y %+.3f"
          % (t[i], zs[i], 100 * (zs[i] - excursion) / excursion, xyz[i, 0], xyz[i, 1]))
print("near turnarounds (truth ~0 m, not measured exactly):")
for i in near:
    print("   t %5.1f s   z %+.3f m" % (t[i], zs[i]))
end = xyz[-10:].mean(0)
print("end of track (truth: back on the start mark, 0 m): x %+.3f  y %+.3f  z %+.3f  |d| %.3f m"
      % (end[0], end[1], end[2], np.linalg.norm(end)))
