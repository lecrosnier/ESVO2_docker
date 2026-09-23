import rospy, sys, numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
out = open(sys.argv[1], "w")
def cb(m):
    p = np.array([q[:3] for q in pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)])
    if len(p) == 0: return
    d = np.linalg.norm(p, axis=1)
    out.write("%.3f %d %.3f %.3f %.3f\n" % (m.header.stamp.to_sec(), len(p), np.median(d), np.percentile(d,10), np.percentile(d,90)))
    out.flush()
rospy.init_node("pcdepth", anonymous=True)
rospy.Subscriber("/esvo2_mapping/pointcloud_local2", PointCloud2, cb, queue_size=50)
rospy.spin()
