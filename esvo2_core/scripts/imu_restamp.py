#!/usr/bin/env python3
"""/sbg/imu_data (IMU clock) -> /imu/data_synced (IMU-clock stamps mapped onto ROS time, see imu_restamp_core)."""
import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

from imu_restamp_core import Restamper


def main():
    rospy.init_node('imu_restamp')
    window_s = rospy.get_param('~window_s', 2.0)
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    restamper = Restamper(window_s=window_s)
    pub = rospy.Publisher('/imu/data_synced', Imu, queue_size=400)
    last_rx = [rospy.get_time()]

    def cb(msg):
        now = rospy.get_time()
        last_rx[0] = now
        stamp = restamper.update(msg.time_stamp, now)
        if stamp is None:
            rospy.logwarn('imu_restamp: IMU clock jumped backwards, offset estimate reset, sample dropped')
            return
        out = Imu()
        out.header.stamp = rospy.Time.from_sec(stamp)
        out.header.frame_id = frame_id
        out.orientation_covariance[0] = -1.0
        out.angular_velocity = msg.gyro
        out.linear_acceleration = msg.accel
        pub.publish(out)

    rospy.Subscriber('/sbg/imu_data', SbgImuData, cb, queue_size=400, tcp_nodelay=True)

    def watchdog(_):
        if rospy.get_time() - last_rx[0] > 0.5:
            rospy.logwarn_throttle(5.0, 'imu_restamp: no /sbg/imu_data for more than 0.5 s')

    rospy.Timer(rospy.Duration(0.25), watchdog)
    rospy.spin()


if __name__ == '__main__':
    main()
