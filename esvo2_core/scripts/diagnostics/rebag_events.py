"""Split dvs_msgs/EventArray messages into CHUNK_S slices so time surfaces can
be generated at 100 Hz (image_representation only renders after a new event
message). Each slice is stamped, and written at, the time of its last event.
usage: rebag_events.py IN.bag OUT.bag [chunk_s]"""
import sys, copy, rosbag, rospy
src, dst = sys.argv[1], sys.argv[2]
chunk = float(sys.argv[3]) if len(sys.argv) > 3 else 0.005
ev_topics = ["/davis/left/events", "/davis/right/events"]
keep = ev_topics + ["/davis/left/imu", "/davis/right/imu"]
with rosbag.Bag(dst, "w") as out:
    for topic, msg, t in rosbag.Bag(src).read_messages(topics=keep):
        if topic not in ev_topics:
            out.write(topic, msg, t); continue
        evs = msg.events; i = 0
        while i < len(evs):
            end = evs[i].ts.to_sec() + chunk; j = i
            while j < len(evs) and evs[j].ts.to_sec() < end: j += 1
            part = copy.copy(msg); part.events = evs[i:j]
            part.header = copy.copy(msg.header); part.header.stamp = evs[j - 1].ts
            out.write(topic, part, evs[j - 1].ts); i = j
