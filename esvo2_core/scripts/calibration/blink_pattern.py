#!/usr/bin/env python3
"""Display a chessboard image and alternate it with its photometric inverse
at native (unscaled) pixel resolution, positioned on a chosen monitor.
Purely a display tool -- opens no camera. Press 'q' or Ctrl+C to stop.
"""
import argparse
import time

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--pattern-png", required=True)
p.add_argument("--mon-x", type=int, default=0, help="Monitor x offset (from xrandr)")
p.add_argument("--mon-y", type=int, default=0, help="Monitor y offset (from xrandr)")
p.add_argument("--period-ms", type=int, default=16, help="Time between flips (ms); display refresh caps the real rate")
args = p.parse_args()

img = cv2.imread(args.pattern_png, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise SystemExit(f"could not read {args.pattern_png}")
# Match Metavision's PatternBlinker: alternate pattern <-> solid white,
# NOT a full photometric inverse (which would also flip the white squares
# and background, making them indistinguishable from the pattern).
blank = np.full_like(img, 255)

win = "blink_pattern"
cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
cv2.moveWindow(win, args.mon_x, args.mon_y)

state = 0
last_flip = time.monotonic()
cv2.imshow(win, img)

print("Blinking pattern. Press 'q' in the window (or Ctrl+C here) to stop.")
try:
    while True:
        now = time.monotonic()
        if (now - last_flip) * 1000 >= args.period_ms:
            state ^= 1
            cv2.imshow(win, blank if state else img)
            last_flip = now
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q') or key == 27:
            break
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
