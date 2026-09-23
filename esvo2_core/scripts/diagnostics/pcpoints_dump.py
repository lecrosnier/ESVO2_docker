#!/usr/bin/env python3
"""Dump every point of every published local map as "stamp x y z" lines, for
depth histograms against tape-measured surfaces. usage: pcpoints_dump.py OUT.txt"""
import rospy, sys, numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
out=open(sys.argv[1],"w")
def cb(m):
    p=np.array([q[:3] for q in pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)])
    if len(p): np.savetxt(out, np.c_[np.full(len(p), m.header.stamp.to_sec()), p], fmt="%.4f"); out.flush()
rospy.init_node("pcz", anonymous=True)
rospy.Subscriber("/esvo2_mapping/pointcloud_local2", PointCloud2, cb, queue_size=50)
rospy.spin()
