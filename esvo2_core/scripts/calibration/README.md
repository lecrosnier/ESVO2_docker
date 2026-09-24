# Stereo calibration of the EVK4 rig

Tools for recalibrating the two EVK4s with Prophesee's
`metavision_calibration_pipeline`, using a blinking chessboard shown on a
monitor. Redo this whenever the cameras or their mounts are touched: on
2026-09-23 a swap to identical mounts shifted the right camera by ~0.7° yaw
and 0.5° pitch and lengthened the baseline from 146.6 to 154.3 mm. That was
enough to place a wall at 4.10 m around 8 m away (see
`docs/superpowers/specs/2026-09-23-realtime-pipeline-findings.md`).

| File | Role |
|---|---|
| `gen_pattern.py` | Generates `pattern.png` and `pattern.json` sized to a monitor's pixel pitch, so each square has a known size in metres. The committed pair is for the rig laptop's 1920×1080 screen (25.80 × 25.76 mm squares). Regenerate for any other screen. |
| `blink_pattern.py` | Shows the chessboard at native resolution, alternating with a plain white frame (not a photometric inverse, which floods every pixel with events). |
| `pipeline_template.json` | The pipeline: left intrinsics, right intrinsics, then synced detection and extrinsics. Left = serial 00051860, right = 00051182. |
| `neutral_settings.json` | Camera settings for the synced step (trail filter off). |
| `convert_to_esvo2.py` | Converts the tool's `camera-geometry.json` × 2 and `extrinsics.json` into ESVO2's `left.yaml` / `right.yaml` (rectified with `cv2.stereoRectify`, `alpha=0`). |

## Procedure

Stop the ROS camera driver first; the tool opens the cameras itself.

```shell
C=/path/to/workdir; mkdir -p $C
cp pattern.png pattern.json neutral_settings.json $C/
sed "s#CALIB_DIR#$C#g" pipeline_template.json > $C/pipeline.json
# the container needs display access: on the host, xhost +si:localuser:root
DISPLAY=:1 python3 blink_pattern.py --pattern-png $C/pattern.png --period-ms 80 &
DISPLAY=:1 metavision_calibration_pipeline -i $C/pipeline.json
python3 convert_to_esvo2.py --calib-dir $C --out-dir $C/esvo2_yaml --master-is left
```

Each detection step waits for `q`: move the board (or the rig as a whole,
never one camera) so it covers the whole view, near and far and tilted, about
30 views per step. In the synced step it must be seen by both cameras at once.

`convert_to_esvo2.py` writes an identity `T_b_c`. `T_b_c` refers to the
*rectified* left frame, so do not copy the previous one unchanged: a new
rectification rotates that frame (by 3.5° on 2026-09-23) even when the IMU is
untouched. If the left camera did not move against the IMU, use
R_b_c,new = R_b_c,old · R1,old · R1,newᵀ (R1 = each `left.yaml`'s
`rectification_matrix`). Otherwise recalibrate it with
`../calibrate_imu_camera_rotation.py`.

**Validate before trusting it.** Replay a bag of a scene with surfaces at
tape-measured distances and check where the map puts them
(`../diagnostics/pcpoints_dump.py`, then a depth histogram). A constant disparity offset — every surface
wrong by the same number of pixels — means the relative yaw is off.

Traps met on 2026-09-23:
- The first two pipeline runs failed with `LIBUSB_ERROR_TIMEOUT`, after a day
  of starting and interrupting the ROS driver: the detection step ended at once
  and the error came from stopping the camera. The third run worked, after each
  camera had been opened once through the SDK's HAL; that may be what cleared
  it, but it was not confirmed.
- `pkill -f blink_pattern` matches the shell that runs it; kill by PID.
