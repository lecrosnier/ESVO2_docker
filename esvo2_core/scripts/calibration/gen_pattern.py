#!/usr/bin/env python3
"""Generate a chessboard pattern image sized precisely for a given monitor's
pixel pitch, plus the pattern.json descriptor the Metavision calibration
pipeline expects. Displaying pattern.png at native (1:1, unscaled) pixel
resolution on that monitor makes each square exactly square_size_m meters.
"""
import argparse
import json
import os

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--out-dir", required=True)
p.add_argument("--mon-width-px", type=int, required=True)
p.add_argument("--mon-height-px", type=int, required=True)
p.add_argument("--mon-width-mm", type=float, required=True)
p.add_argument("--mon-height-mm", type=float, required=True)
p.add_argument("--internal-corners-w", type=int, default=9)
p.add_argument("--internal-corners-h", type=int, default=6)
p.add_argument("--square-px", type=int, default=100)
p.add_argument("--quiet-zone-px", type=int, default=100)
args = p.parse_args()

px_per_mm_x = args.mon_width_px / args.mon_width_mm
px_per_mm_y = args.mon_height_px / args.mon_height_mm

squares_w = args.internal_corners_w + 1
squares_h = args.internal_corners_h + 1

board_w = squares_w * args.square_px
board_h = squares_h * args.square_px
img_w = board_w + 2 * args.quiet_zone_px
img_h = board_h + 2 * args.quiet_zone_px

assert img_w <= args.mon_width_px, f"pattern width {img_w}px exceeds monitor width {args.mon_width_px}px"
assert img_h <= args.mon_height_px, f"pattern height {img_h}px exceeds monitor height {args.mon_height_px}px"

img = np.full((img_h, img_w), 255, dtype=np.uint8)
for r in range(squares_h):
    for c in range(squares_w):
        if (r + c) % 2 == 0:
            y0 = args.quiet_zone_px + r * args.square_px
            x0 = args.quiet_zone_px + c * args.square_px
            img[y0:y0 + args.square_px, x0:x0 + args.square_px] = 0

os.makedirs(args.out_dir, exist_ok=True)
png_path = os.path.join(args.out_dir, "pattern.png")
cv2.imwrite(png_path, img)

square_size_m = (args.square_px / px_per_mm_x) / 1000.0
square_size_m_y = (args.square_px / px_per_mm_y) / 1000.0

pattern_json = {
    "type": "Chessboard",
    "width": args.internal_corners_w,
    "height": args.internal_corners_h,
    "square-width": round(square_size_m, 6),
    "square-height": round(square_size_m_y, 6),
}
json_path = os.path.join(args.out_dir, "pattern.json")
with open(json_path, "w") as f:
    json.dump(pattern_json, f, indent=2)

print(f"pattern.png: {img_w}x{img_h}px (board {board_w}x{board_h}px + {args.quiet_zone_px}px quiet zone)")
print(f"square size: {square_size_m*1000:.2f}mm x {square_size_m_y*1000:.2f}mm (at native 1:1 display)")
print(f"internal corners: {args.internal_corners_w}x{args.internal_corners_h}")
print(f"wrote {png_path}")
print(f"wrote {json_path}")
