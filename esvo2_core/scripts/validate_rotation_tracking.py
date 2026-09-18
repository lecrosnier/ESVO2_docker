#!/usr/bin/env python3
"""Compare tracking poses with the gyro.

mode slowturn: net rotation (gyro-integrated vs pose) at each hold (>= 1 s with |w| < 0.05 rad/s).
mode mixed:    pose/gyro rotation speed ratio in gyro speed bins.
Params: ~mode (slowturn|mixed), ~duration (s).
"""
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


def q2R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def exp_so3(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def main():
    rospy.init_node('validate_rotation_tracking', anonymous=True)
    mode = rospy.get_param('~mode', 'slowturn')
    duration = rospy.get_param('~duration', 70.0)
    imu, poses = [], []
    rospy.Subscriber('/imu/data_synced', Imu, lambda m: imu.append(
        (m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)), queue_size=20000)
    rospy.Subscriber('/esvo2_tracking/pose_pub', PoseStamped, lambda m: poses.append(
        (m.header.stamp.to_sec(), m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z)),
        queue_size=5000)
    time.sleep(1.0)
    imu.clear(); poses.clear()
    print(">>> recording %.0f s (%s)" % (duration, mode))
    time.sleep(duration)

    I = np.array(imu)
    P = np.array(sorted(poses))
    print("imu samples %d, poses %d" % (len(I), len(P)))
    if len(P) < 20:
        print("FAIL: too few poses")
        return
    wnorm = np.linalg.norm(I[:, 1:4], axis=1)

    if mode == 'slowturn':
        still = wnorm < 0.05
        bias = I[still, 1:4].mean(axis=0) if still.sum() > 200 else np.zeros(3)
        R = np.eye(3); gR = []
        for k in range(len(I)):
            dt = I[k + 1, 0] - I[k, 0] if k + 1 < len(I) else 0.005
            gR.append(R.copy())
            R = R @ exp_so3((I[k, 1:4] - bias) * dt)
        gR = np.array(gR)
        holds, k = [], 0
        while k < len(I):
            if still[k]:
                j = k
                while j + 1 < len(I) and still[j + 1]:
                    j += 1
                if I[j, 0] - I[k, 0] >= 1.0:
                    holds.append(0.5 * (I[k, 0] + I[j, 0]))
                k = j + 1
            else:
                k += 1
        if len(holds) < 2:
            print("FAIL: fewer than two holds detected")
            return
        g0 = gR[np.argmin(np.abs(I[:, 0] - holds[0]))]
        p0 = P[np.argmin(np.abs(P[:, 0] - holds[0]))]
        R_p0 = q2R(*p0[1:5])
        ok = True
        for h in holds[1:]:
            g = angle_deg(g0.T @ gR[np.argmin(np.abs(I[:, 0] - h))])
            pk = P[np.argmin(np.abs(P[:, 0] - h))]
            p = angle_deg(R_p0.T @ q2R(*pk[1:5]))
            rel = abs(p - g) / max(g, 1e-6)
            passed = g < 5.0 or rel <= 0.2
            ok &= passed
            print("hold at %.1f s: gyro net %.1f deg, pose net %.1f deg, error %.0f%% -> %s" % (
                h - I[0, 0], g, p, 100 * rel, 'ok' if passed else 'FAIL'))
        print("slowturn result: %s" % ('PASS' if ok else 'FAIL'))
    else:
        rows = []
        for a, b in zip(P[:-1], P[1:]):
            dt = b[0] - a[0]
            if not 0.005 < dt <= 0.2:
                continue
            ang = 2 * np.arccos(min(1.0, abs(float(np.dot(a[1:5], b[1:5])))))
            if ang / dt > 10:
                continue
            m = (I[:, 0] >= a[0]) & (I[:, 0] < b[0])
            if m.sum() >= 3:
                rows.append((wnorm[m].mean(), ang / dt))
        R = np.array(rows)
        for lo, hi in ((0.0, 0.1), (0.1, 0.5), (0.5, 1.2), (1.2, 99.0)):
            m = (R[:, 0] >= lo) & (R[:, 0] < hi)
            if m.sum() < 10:
                print("bin %.1f-%.1f rad/s: too few (%d)" % (lo, hi, m.sum()))
                continue
            print("bin %.1f-%.1f rad/s: n=%d gyro %.3f pose %.3f ratio %.2f" % (
                lo, hi, m.sum(), np.median(R[m, 0]), np.median(R[m, 1]), np.median(R[m, 1]) / np.median(R[m, 0])))


if __name__ == '__main__':
    main()
