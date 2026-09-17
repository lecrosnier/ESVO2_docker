#!/usr/bin/env python3
"""Calibrate R_b_c (rectified left camera -> IMU) and t_d (camera time = IMU time + t_d).

Run with only the camera driver and imu_restamp running. Rotate the rig by
hand about all axes in front of a distant (>= 3 m) textured scene.
Params: ~duration (s, default 60), ~window (s, default 0.01),
        ~calib (left.yaml path), ~out (results YAML path).
"""
import os
import queue
import struct
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import Imu

import imu_cam_calib_core as core

EVT = np.dtype([('x', '<u2'), ('y', '<u2'), ('s', '<u4'), ('ns', '<u4'), ('p', 'u1')])
SCALE = 2  # count images at half resolution


def load_calib(path):
    c = yaml.safe_load(open(path))
    M = lambda k, r, cols: np.array(c[k]['data'], dtype=np.float64).reshape(r, cols)
    return (M('camera_matrix', 3, 3), np.array(c['distortion_coefficients']['data'], dtype=np.float64),
            M('rectification_matrix', 3, 3), M('projection_matrix', 3, 4), c['image_width'], c['image_height'])


class EventFramer:
    """Accumulates events into count images of fixed length by event time."""

    def __init__(self, width, height, window, out_q):
        self.w, self.h, self.window, self.q = width // SCALE, height // SCALE, window, out_q
        self.t0 = None
        self.idx = []
        self.dropped = 0

    def add(self, raw):
        L = struct.unpack_from('<I', raw, 12)[0]
        n = struct.unpack_from('<I', raw, 24 + L)[0]
        if n == 0:
            return
        e = np.frombuffer(raw, dtype=EVT, count=n, offset=28 + L)
        ts = e['s'].astype(np.float64) + e['ns'] * 1e-9
        if self.t0 is None:
            self.t0 = ts[0]
        win_end = self.t0 + self.window
        if ts[-1] >= win_end:
            before = ts < win_end
            if before.any():
                self.idx.append((e['y'][before] // SCALE).astype(np.int64) * self.w + e['x'][before] // SCALE)
            self._flush(win_end)
            self.t0 = win_end if ts[-1] < win_end + self.window else ts[-1]
            rest = ~before
            self.idx.append((e['y'][rest] // SCALE).astype(np.int64) * self.w + e['x'][rest] // SCALE)
        else:
            self.idx.append((e['y'] // SCALE).astype(np.int64) * self.w + e['x'] // SCALE)

    def _flush(self, t_end):
        if not self.idx:
            return
        counts = np.bincount(np.concatenate(self.idx), minlength=self.w * self.h)[: self.w * self.h]
        self.idx = []
        img = np.clip(counts.reshape(self.h, self.w) * 60, 0, 255).astype(np.uint8)
        try:
            self.q.put_nowait((t_end - self.window / 2.0, img))
        except queue.Full:
            self.dropped += 1


def camera_rates(frames_q, stop, calib, window, results):
    K, D, R_rect, P, _, _ = calib
    prev = None
    rng = np.random.default_rng(0)
    while not (stop.is_set() and frames_q.empty()):
        try:
            t, img = frames_q.get(timeout=0.2)
        except queue.Empty:
            continue
        img = cv2.GaussianBlur(img, (5, 5), 0)
        if prev is not None:
            t0, img0 = prev
            pts0 = cv2.goodFeaturesToTrack(img0, maxCorners=300, qualityLevel=0.01, minDistance=8)
            if pts0 is not None and len(pts0) >= 20:
                pts1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img, pts0, None, winSize=(21, 21), maxLevel=3)
                back, st2, _ = cv2.calcOpticalFlowPyrLK(img, img0, pts1, None, winSize=(21, 21), maxLevel=3)
                good = (st[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm((back - pts0)[:, 0], axis=1) < 1.0)
                if good.sum() >= 20:
                    full0 = (pts0[good] * SCALE + 0.5).astype(np.float64)
                    full1 = (pts1[good] * SCALE + 0.5).astype(np.float64)
                    u0 = cv2.undistortPoints(full0, K, D, R=R_rect).reshape(-1, 2)
                    u1 = cv2.undistortPoints(full1, K, D, R=R_rect).reshape(-1, 2)
                    f0 = np.hstack([u0, np.ones((len(u0), 1))]); f0 /= np.linalg.norm(f0, axis=1, keepdims=True)
                    f1 = np.hstack([u1, np.ones((len(u1), 1))]); f1 /= np.linalg.norm(f1, axis=1, keepdims=True)
                    R, inl = core.fit_rotation_ransac(f0, f1, thresh_rad=0.003, iters=100, rng=rng)
                    if R is not None and inl.mean() >= 0.5 and inl.sum() >= 20:
                        results.append(((t0 + t) / 2.0, core.so3_log(R) / (t - t0), float(inl.mean())))
        prev = (t, img)


def solve(cam, imu, label):
    t_cam = np.array([c[0] for c in cam]); w_cam = np.array([c[1] for c in cam])
    t_imu = np.array([i[0] for i in imu]); w_imu = np.array([i[1] for i in imu])
    t_d, corr, sharp = core.estimate_time_offset(t_cam, np.linalg.norm(w_cam, axis=1), t_imu, np.linalg.norm(w_imu, axis=1))
    w_imu_at_cam = np.stack([np.interp(t_cam - t_d, t_imu, w_imu[:, k]) for k in range(3)], axis=1)
    moving = np.linalg.norm(w_imu_at_cam, axis=1) > 0.2
    R, inl, rms = core.fit_R_b_c(w_imu_at_cam[moving], w_cam[moving])
    print("[%s] t_d=%.4f s (corr %.3f, sharpness %.3f) | pairs %d moving, inliers %.0f%%, rms %.3f rad/s" % (
        label, t_d, corr, sharp, int(moving.sum()), 100.0 * inl.mean(), rms))
    return R, t_d, dict(corr=corr, sharpness=sharp, pairs=int(moving.sum()), inlier_share=float(inl.mean()), rms=rms)


def main():
    rospy.init_node('calibrate_imu_camera_rotation', anonymous=True)
    duration = rospy.get_param('~duration', 60.0)
    window = rospy.get_param('~window', 0.01)
    calib_path = rospy.get_param('~calib', os.path.join(os.path.dirname(__file__), '..', 'calib', 'evk4_stereo', 'left.yaml'))
    out_path = rospy.get_param('~out', '/tmp/imu_cam_rotation_result.yaml')
    calib = load_calib(calib_path)

    frames_q = queue.Queue(maxsize=200)
    framer = EventFramer(calib[4], calib[5], window, frames_q)
    imu, cam = [], []
    capturing = threading.Event()
    stop = threading.Event()

    rospy.Subscriber('/evk4_left/events', rospy.AnyMsg, lambda m: framer.add(m._buff) if capturing.is_set() else None,
                     queue_size=100000, buff_size=2 ** 24, tcp_nodelay=True)
    rospy.Subscriber('/imu/data_synced', Imu, lambda m: imu.append((m.header.stamp.to_sec(), np.array(
        [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]))) if capturing.is_set() else None,
        queue_size=20000)
    worker = threading.Thread(target=camera_rates, args=(frames_q, stop, calib, window, cam), daemon=True)
    worker.start()

    time.sleep(1.0)
    print(">>> capturing for %.0f s: rotate the rig about all axes, little translation" % duration)
    capturing.set()
    time.sleep(duration)
    capturing.clear()
    stop.set()
    worker.join()
    print("camera rate samples %d, IMU samples %d, frames dropped (queue full) %d" % (len(cam), len(imu), framer.dropped))
    if len(cam) < 200 or len(imu) < 1000:
        print("not enough data; repeat the capture with more texture / motion")
        return

    R, t_d, q = solve(cam, imu, 'all')
    half = cam[len(cam) // 2][0]
    R1, t1, _ = solve([c for c in cam if c[0] < half], imu, 'first half')
    R2, t2, _ = solve([c for c in cam if c[0] >= half], imu, 'second half')
    rep_deg = float(np.degrees(np.linalg.norm(core.so3_log(R1.T @ R2))))
    perm_deg = core.signed_permutation_distance_deg(R)
    print("R_b_c =\n%s" % np.array2string(R, precision=4, suppress_small=True))
    print("repeatability: halves differ by %.2f deg and %.1f ms | distance to nearest axis permutation %.2f deg" % (
        rep_deg, 1000.0 * abs(t1 - t2), perm_deg))
    q.update(half_rotation_diff_deg=rep_deg, half_t_d_diff_ms=1000.0 * abs(t1 - t2), permutation_distance_deg=perm_deg)
    with open(out_path, 'w') as fh:
        yaml.safe_dump({'R_b_c': [float(x) for x in R.reshape(-1)], 't_d': float(t_d), 'quality': q}, fh, sort_keys=False)
    print("results written to %s (calibration files NOT modified)" % out_path)


if __name__ == '__main__':
    main()
