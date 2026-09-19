#!/usr/bin/env python3
"""Compare an estimated trajectory with ground truth.

usage: eval_traj.py GT.txt POSES.bag [label]
  GT.txt     "t x y z qx qy qz qw" per line (results/gt/... format)
  POSES.bag  /esvo2_tracking/pose_pub recorded during a replay
Poses are matched to ground truth within 20 ms. Path length is measured on
0.5 s samples so per-frame jitter does not inflate it. Prints the path ratio
(estimate / ground truth), the Sim3 scale, and ATE after SE3 and Sim3 alignment.
"""
import sys
import numpy as np
import rosbag

gt = np.loadtxt(sys.argv[1])[:, :4]
est = np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z)
                for _, m, _ in rosbag.Bag(sys.argv[2]).read_messages(topics=["/esvo2_tracking/pose_pub"])])
label = sys.argv[3] if len(sys.argv) > 3 else sys.argv[2]

i = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
i = np.where(np.abs(gt[i - 1, 0] - est[:, 0]) < np.abs(gt[i, 0] - est[:, 0]), i - 1, i)
ok = np.abs(gt[i, 0] - est[:, 0]) < 0.02
if ok.sum() < 3:
    sys.exit("too few pose/ground-truth matches within 20 ms: %d (need >= 3)" % ok.sum())
A, B = est[ok, 1:4], gt[i[ok], 1:4]


def align(A, B, with_scale):
    ma, mb = A.mean(0), B.mean(0)
    a, b = A - ma, B - mb
    U, D, Vt = np.linalg.svd(b.T @ a / len(A))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / a.var(0).sum() if with_scale else 1.0
    return s, np.sqrt((((s * (R @ a.T)).T - b) ** 2).sum(1).mean())


T = est[ok, 0]
keep = np.searchsorted(T, np.arange(T[0], T[-1], 0.5))
path = lambda X: np.linalg.norm(np.diff(X[keep], axis=0), axis=1).sum()
_, ate = align(A, B, False)
s, ate_s = align(A, B, True)
print("%s: matched %d/%d over %.1f s | path est %.2f m, gt %.2f m, ratio %.2f | Sim3 scale %.2f | ATE SE3 %.3f m, Sim3 %.3f m"
      % (label, ok.sum(), len(est), T[-1] - T[0], path(A), path(B), path(A) / path(B), s, ate, ate_s))
