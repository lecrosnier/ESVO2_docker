#!/usr/bin/env python3
"""Simulate the IMU dying during a replay.

Relays /imu_raw to /imu/data_synced until the given header stamp, then drops
everything. Play the bag with the IMU remapped away from the tracker:

  python3 imu_cut_relay.py STAMP &
  PLAYARGS="/imu/data_synced:=/imu_raw" ../replay_eval.sh ...

usage: imu_cut_relay.py CUT_STAMP_S
"""
import sys

import rospy
from sensor_msgs.msg import Imu

cut = float(sys.argv[1])
rospy.init_node("imu_cut_relay")
pub = rospy.Publisher("/imu/data_synced", Imu, queue_size=200)
warned = [False]


def relay(msg):
    if msg.header.stamp.to_sec() < cut:
        pub.publish(msg)
    elif not warned[0]:
        warned[0] = True
        rospy.logwarn("imu_cut_relay: IMU cut at %.2f", cut)


rospy.Subscriber("/imu_raw", Imu, relay, queue_size=200)
rospy.spin()
