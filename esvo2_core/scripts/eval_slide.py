#!/usr/bin/env python3
"""Metrics for a replayed sideways slide: tracked translation, and tracked yaw vs gyro yaw.

usage: eval_slide.py POSES.bag SOURCE.bag CALIB_DIR BX,BY,BZ
  POSES.bag   /esvo2_tracking/pose_pub recorded during the replay (replay_eval.sh)
  SOURCE.bag  the replayed bag (read for /imu/data_synced)
  CALIB_DIR   calibration folder (T_b_c from left.yaml)
  BX,BY,BZ    gyro bias, rad/s, IMU frame

Everything is expressed in the camera frame of the first tracked pose (x right,
y down, z forward); yaw is rotation about camera y. The IMU/camera time offset
(a few ms) is ignored: it is negligible for yaw accumulated over seconds.
Prints x every 2 s, then: min_x=<m> final_x=<m> max_dyaw=<deg>.
"""
import sys
import numpy as np
import rosbag
import yaml


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(R):
    a = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


poses_bag, src_bag, calib_dir, bias_s = sys.argv[1:5]
bias = np.array([float(v) for v in bias_s.split(",")])
R_b_c = np.array(yaml.safe_load(open(calib_dir + "/left.yaml"))["T_b_c"]["data"]).reshape(3, 4)[:, :3]

P = np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
               m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)
              for _, m, _ in rosbag.Bag(poses_bag).read_messages(topics=["/esvo2_tracking/pose_pub"])])
if len(P) < 10:
    sys.exit("too few poses: %d" % len(P))
G = np.array([(m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
              for _, m, _ in rosbag.Bag(src_bag).read_messages(topics=["/imu/data_synced"])])

t = P[:, 0] - P[0, 0]
R0 = q2R(*P[0, 4:])
d = (P[:, 1:4] - P[0, 1:4]) @ R0                       # camera frame of the first pose
yaw_trk = np.degrees([rotvec(R0.T @ q2R(*p[4:]))[1] for p in P])

G = G[G[:, 0] >= P[0, 0]]
if len(G) == 0:
    sys.exit("no IMU samples remain after the first pose time (%.3f); check the bag has /imu/data_synced covering the poses" % P[0, 0])
w_cam = (R_b_c.T @ (G[:, 1:] - bias).T).T               # bias-corrected rate, camera frame
yaw_gyro_samples = np.degrees(np.concatenate([[0.0], np.cumsum(w_cam[1:, 1] * np.diff(G[:, 0]))]))
yaw_gyro = np.interp(P[:, 0], G[:, 0], yaw_gyro_samples)

row = " ".join("%+.2f" % d[min(np.searchsorted(t, k), len(t) - 1), 0] for k in range(0, int(t[-1]) + 1, 2))
print("poses %d  x every 2 s: %s" % (len(P), row))
print("min_x=%.3f final_x=%.3f max_dyaw=%.2f" % (d[:, 0].min(), d[-1, 0], np.abs(yaw_trk - yaw_gyro).max()))
