import rospy, sys
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
out = open(sys.argv[1], "w")
import sensor_msgs.point_cloud2 as pc2
def cb(m):
    lab = sum(1 for p in pc2.read_points(m, field_names=("label",)) if p[0] == 1)
    out.write("pc %.3f %d %d\n" % (m.header.stamp.to_sec(), m.width * m.height, lab)); out.flush()
rospy.init_node("pcmon", anonymous=True)
rospy.Subscriber("/esvo2_mapping/pointcloud_local2", PointCloud2, cb, queue_size=100)
rospy.spin()
