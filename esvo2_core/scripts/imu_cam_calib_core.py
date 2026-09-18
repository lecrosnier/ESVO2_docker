"""Pure math for the camera-IMU rotation / time-offset calibration.

Conventions (see the design spec):
  p_imu = R_b_c * p_cam            (camera = rectified left camera)
  w_imu = R_b_c * w_cam
  camera time = IMU time + t_d
  f_prev = R * f_next              (R = orientation of the later camera frame in the earlier one)
"""
import itertools

import numpy as np


def _hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def so3_exp(v):
    v = np.asarray(v, dtype=np.float64)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3) + _hat(v)
    K = _hat(v / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def so3_log(R):
    cos_th = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(cos_th)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return 0.5 * w
    if np.pi - th < 1e-6:
        B = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(B), 0.0, None))
        k = int(np.argmax(axis))
        axis = B[:, k] / np.sqrt(B[k, k])
        return th * axis
    return th / (2.0 * np.sin(th)) * w


def kabsch(A, B, w=None):
    """R minimising sum w_i |A_i - R B_i|^2."""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    w = np.ones(len(A)) if w is None else np.asarray(w, dtype=np.float64)
    H = (B * w[:, None]).T @ A
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def fit_rotation_ransac(f_prev, f_next, thresh_rad=0.005, iters=100, rng=None):
    n = len(f_prev)
    if n < 3:
        return None, None
    rng = np.random.default_rng() if rng is None else rng
    cos_thr = np.cos(thresh_rad)
    best = None
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        R = kabsch(f_prev[idx], f_next[idx])
        inl = np.sum(f_prev * (f_next @ R.T), axis=1) >= cos_thr
        if best is None or inl.sum() > best.sum():
            best = inl
    if best.sum() < 3:
        return None, None
    R = kabsch(f_prev[best], f_next[best])
    inl = np.sum(f_prev * (f_next @ R.T), axis=1) >= cos_thr
    return R, inl


def estimate_time_offset(t_cam, s_cam, t_imu, s_imu, max_offset=0.1, step=0.001):
    """t_d maximising corr(s_cam(t), s_imu(t - t_d)); returns (t_d, peak_corr, sharpness)."""
    t_cam = np.asarray(t_cam, dtype=np.float64)
    s_cam = np.asarray(s_cam, dtype=np.float64)
    offsets = np.arange(-max_offset, max_offset + step / 2, step)
    lo, hi = t_imu[0] + max_offset, t_imu[-1] - max_offset
    m = (t_cam >= lo) & (t_cam <= hi)
    tc, sc = t_cam[m], s_cam[m]
    if tc.size < 10:
        raise ValueError(
            "estimate_time_offset: insufficient overlap between camera and IMU spans "
            "(camera [%.3f, %.3f], IMU [%.3f, %.3f], max_offset=%.3f) -- only %d samples "
            "remain after trimming max_offset off each end" % (
                t_cam[0] if t_cam.size else float('nan'),
                t_cam[-1] if t_cam.size else float('nan'),
                t_imu[0], t_imu[-1], max_offset, tc.size))
    sc = (sc - sc.mean()) / (sc.std() + 1e-12)
    corrs = []
    for d in offsets:
        si = np.interp(tc - d, t_imu, s_imu)
        si = (si - si.mean()) / (si.std() + 1e-12)
        corrs.append(float(np.mean(sc * si)))
    corrs = np.array(corrs)
    k = int(np.argmax(corrs))
    far = np.abs(offsets - offsets[k]) > 0.05  # hand motion is smooth over ~100 ms; compare against clearly different offsets
    sharpness = float(corrs[k] - (corrs[far].max() if far.any() else corrs[k]))
    return float(offsets[k]), float(corrs[k]), sharpness


def fit_R_b_c(w_imu, w_cam, iters=3):
    w_imu = np.asarray(w_imu, dtype=np.float64)
    w_cam = np.asarray(w_cam, dtype=np.float64)
    inl = np.ones(len(w_imu), dtype=bool)
    R = kabsch(w_imu, w_cam)
    for _ in range(iters):
        r = np.linalg.norm(w_imu - w_cam @ R.T, axis=1)
        med = np.median(r[inl])
        inl = r <= 3.0 * max(med, 1e-9)
        R = kabsch(w_imu[inl], w_cam[inl])
    r = np.linalg.norm(w_imu - w_cam @ R.T, axis=1)
    rms = float(np.sqrt(np.mean(r[inl] ** 2)))
    return R, inl, rms


def signed_permutation_distance_deg(R):
    best = 180.0
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            P = np.zeros((3, 3))
            for row, (col, sg) in enumerate(zip(perm, signs)):
                P[row, col] = sg
            if np.linalg.det(P) < 0:
                continue
            best = min(best, float(np.degrees(np.linalg.norm(so3_log(P.T @ R)))))
    return best
