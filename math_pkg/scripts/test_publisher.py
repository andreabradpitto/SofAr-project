#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu


def talker():

    pub = rospy.Publisher('smartphone', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100 Hz
	
    imu = Imu()
    #imu.header.frame_id = imu_frame_id
    imu.header.stamp = rospy.Time.from_sec(1)
    imu.orientation.x = 1
    imu.orientation.y = 0
    imu.orientation.z = 0
    imu.orientation.w = 0
    imu.linear_acceleration.x = 0
    imu.linear_acceleration.y = 0
    imu.linear_acceleration.z = 1
    imu.angular_velocity.x = 0
    imu.angular_velocity.y = 1
    imu.angular_velocity.z = 0

    #pub.publish(imu)

    while not rospy.is_shutdown():
        pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
