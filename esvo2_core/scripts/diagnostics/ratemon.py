import rospy, sys, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
out = open(sys.argv[1], "w")
def mk(tag):
    def cb(m): out.write("%s %.4f %.4f\n" % (tag, m.header.stamp.to_sec(), time.time())); out.flush()
    return cb
rospy.init_node("ratemon", anonymous=True)
rospy.Subscriber("/image_representation_TS_l", Image, mk("tsl"), queue_size=200, buff_size=2**26)
rospy.Subscriber("/image_representation_TS_r", Image, mk("tsr"), queue_size=200, buff_size=2**26)
rospy.Subscriber("/esvo2_tracking/pose_pub", PoseStamped, mk("pose"), queue_size=500)
rospy.spin()
