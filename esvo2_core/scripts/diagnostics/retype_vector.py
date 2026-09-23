"""Merge VECtor's event and IMU bags into one ESVO2-ready bag.

VECtor publishes prophesee_event_msgs/EventArray, whose layout is identical to
dvs_msgs/EventArray (same md5), so the messages are rewritten raw, without
deserialising, under the topic names the launch files expect.
usage: retype.py LEFT.bag RIGHT.bag IMU.bag OUT.bag
"""
import sys, rosbag
left, right, imu, out = sys.argv[1:5]
DVS_MD5 = "5e8beee5a6c107e504c2e78903c224b8"
with rosbag.Bag(out, "w") as o:
    for src, topic in ((left, "/davis/left/events"), (right, "/davis/right/events")):
        n = 0
        for _, (datatype, data, md5, pos, pytype), t in rosbag.Bag(src).read_messages(raw=True):
            assert md5 == DVS_MD5, "unexpected message layout: %s %s" % (datatype, md5)
            o.write(topic, ("dvs_msgs/EventArray", data, md5, pos, pytype), t, raw=True)
            n += 1
        print("%s -> %s: %d messages" % (src, topic, n))
    n = 0
    for topic, msg, t in rosbag.Bag(imu).read_messages():
        o.write("/imu/data", msg, t); n += 1
    print("%s -> /imu/data: %d messages" % (imu, n))
