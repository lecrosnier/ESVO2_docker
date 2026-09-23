#!/usr/bin/env python3
"""Convert Metavision calibration pipeline output (camera-geometry.json x2 +
extrinsics.json) into ESVO2's expected left.yaml / right.yaml format
(OpenCV pinhole/plumb_bob + stereo rectification, as used under
esvo2_core/calib/<rig>/).
"""
import argparse
import json
import os

import cv2
import numpy as np
import yaml


def load_geometry(path):
    with open(path) as f:
        j = json.load(f)
    assert j["type"] == "pinhole", f"unsupported camera model: {j['type']}"
    K = np.array(j["K"], dtype=np.float64).reshape(3, 3)
    D = np.array(j["D"], dtype=np.float64)
    return j["width"], j["height"], K, D


def load_extrinsics(path, slave_key="slave-0"):
    with open(path) as f:
        j = json.load(f)
    entry = next(e for e in j["T_slave_master"] if e["camera"] == slave_key)
    rvec = np.array(entry["rvec"], dtype=np.float64)
    tvec = np.array(entry["tvec"], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec


def yaml_matrix(rows, cols, data):
    return {"rows": rows, "cols": cols, "data": [float(x) for x in np.asarray(data).flatten()]}


def write_calib_yaml(path, width, height, camera_name, K, D, R, P, T_right_left, T_b_c):
    doc = {
        "image_width": width,
        "image_height": height,
        "camera_name": camera_name,
        "camera_matrix": yaml_matrix(3, 3, K),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": yaml_matrix(1, len(D), D),
        "rectification_matrix": yaml_matrix(3, 3, R),
        "projection_matrix": yaml_matrix(3, 4, P),
        "T_right_left": yaml_matrix(3, 4, T_right_left),
        "T_b_c": yaml_matrix(3, 4, T_b_c),
    }
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=None, sort_keys=False)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--calib-dir", required=True, help="dir containing master/, slave/, extrinsics/")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--camera-name", default="Prophesee_EVK4_IMX636")
    p.add_argument("--master-is", choices=["left", "right"], required=True,
                   help="which physical side the 'master' (sync) camera is on")
    args = p.parse_args()

    master_geom = os.path.join(args.calib_dir, "master", "camera-geometry.json")
    slave_geom = os.path.join(args.calib_dir, "slave", "camera-geometry.json")
    extrinsics_path = os.path.join(args.calib_dir, "extrinsics", "extrinsics.json")

    w_m, h_m, K_m, D_m = load_geometry(master_geom)
    w_s, h_s, K_s, D_s = load_geometry(slave_geom)
    assert (w_m, h_m) == (w_s, h_s), "left/right resolutions differ"

    # extrinsics.json gives X_slave = R_sm * X_master + t_sm
    R_sm, t_sm = load_extrinsics(extrinsics_path)

    if args.master_is == "right":
        # master=right, slave=left: need T_right_left s.t. X_right = R*X_left + t,
        # which is the inverse of X_left(slave) = R_sm*X_right(master) + t_sm.
        w_l, h_l, K_l, D_l = w_s, h_s, K_s, D_s
        w_r, h_r, K_r, D_r = w_m, h_m, K_m, D_m
        R_rl = R_sm.T
        t_rl = -R_sm.T @ t_sm
    else:
        # master=left, slave=right: X_right(slave) = R_sm*X_left(master) + t_sm
        # is already T_right_left directly.
        w_l, h_l, K_l, D_l = w_m, h_m, K_m, D_m
        w_r, h_r, K_r, D_r = w_s, h_s, K_s, D_s
        R_rl, t_rl = R_sm, t_sm

    baseline_m = np.linalg.norm(t_rl)

    image_size = (w_l, h_l)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K_l, D_l, K_r, D_r, image_size, R_rl, t_rl,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    T_right_left = np.hstack([R_rl, t_rl.reshape(3, 1)])
    T_b_c = np.hstack([np.eye(3), np.zeros((3, 1))])

    os.makedirs(args.out_dir, exist_ok=True)
    write_calib_yaml(os.path.join(args.out_dir, "left.yaml"), w_l, h_l, args.camera_name + "_left",
                     K_l, D_l, R1, P1, T_right_left, T_b_c)
    write_calib_yaml(os.path.join(args.out_dir, "right.yaml"), w_r, h_r, args.camera_name + "_right",
                     K_r, D_r, R2, P2, T_right_left, T_b_c)

    print(f"baseline: {baseline_m*1000:.2f} mm")
    print(f"left K:\n{K_l}")
    print(f"right K:\n{K_r}")
    print(f"wrote {args.out_dir}/left.yaml and right.yaml")


if __name__ == "__main__":
    main()
