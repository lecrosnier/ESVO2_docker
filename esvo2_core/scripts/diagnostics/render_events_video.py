#!/usr/bin/env python3
"""Render a stereo event bag to a side-by-side video (left | right).

Each frame accumulates 1/FPS s of events: ON events in blue, OFF in red, on
black, at half resolution. Events are decoded straight from the serialized
dvs_msgs/EventArray bytes with numpy, so a multi-GB EVK4 bag renders in a
couple of minutes rather than the hours per-event Python deserialisation takes.

usage: render_events_video.py BAG OUT.mp4 [fps] [left_topic] [right_topic]
"""
import struct
import sys

import cv2
import numpy as np
import rosbag

bag_path, out_path = sys.argv[1], sys.argv[2]
fps = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
topics = [sys.argv[4] if len(sys.argv) > 4 else "/evk4_left/events",
          sys.argv[5] if len(sys.argv) > 5 else "/evk4_right/events"]

# dvs_msgs/Event: uint16 x, uint16 y, time ts (uint32 secs, uint32 nsecs), bool polarity -- packed, 13 bytes
EVENT = np.dtype([("x", "<u2"), ("y", "<u2"), ("s", "<u4"), ("ns", "<u4"), ("p", "u1")])
W, H, SCALE = 1280, 720, 2
PW, PH = W // SCALE, H // SCALE


def decode(raw):
    """Return (x, y, polarity) arrays from a serialized dvs_msgs/EventArray."""
    o = 12                                   # header: seq, stamp secs, stamp nsecs
    (n,) = struct.unpack_from("<I", raw, o)  # frame_id length
    o += 4 + n + 8                           # frame_id, height, width
    (count,) = struct.unpack_from("<I", raw, o)
    ev = np.frombuffer(raw, dtype=EVENT, count=count, offset=o + 4)
    return ev["x"], ev["y"], ev["p"]


bag = rosbag.Bag(bag_path)
t0, t1 = bag.get_start_time(), bag.get_end_time()
frame_dt = 1.0 / fps
n_frames = int((t1 - t0) / frame_dt) + 1
writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (2 * PW, PH))

acc = [np.zeros((2, PH, PW), np.uint16) for _ in topics]  # per camera: [ON, OFF] counts
current = 0


def flush(frame_index):
    img = np.zeros((PH, 2 * PW, 3), np.uint8)
    for c in range(2):
        on, off = acc[c]
        panel = img[:, c * PW:(c + 1) * PW]
        panel[..., 0] = np.minimum(on * 80, 255)   # blue: ON
        panel[..., 2] = np.minimum(off * 80, 255)  # red: OFF
        acc[c][:] = 0
    cv2.line(img, (PW, 0), (PW, PH), (80, 80, 80), 1)
    cv2.putText(img, "LEFT", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    cv2.putText(img, "RIGHT", (PW + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    cv2.putText(img, "t = %6.2f s" % (frame_index * frame_dt), (2 * PW - 190, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    writer.write(img)


for topic, raw, t in bag.read_messages(topics=topics, raw=True):
    k = int((t.to_sec() - t0) / frame_dt)
    while current < k:
        flush(current)
        current += 1
    x, y, p = decode(raw[1])
    c = topics.index(topic)
    xs, ys = x // SCALE, y // SCALE
    ok = (xs < PW) & (ys < PH)
    np.add.at(acc[c][0], (ys[ok & (p == 1)], xs[ok & (p == 1)]), 1)
    np.add.at(acc[c][1], (ys[ok & (p == 0)], xs[ok & (p == 0)]), 1)

while current < n_frames:
    flush(current)
    current += 1
writer.release()
print("wrote %s: %d frames at %.0f fps (%.1f s)" % (out_path, n_frames, fps, n_frames / fps))
