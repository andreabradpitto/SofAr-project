#!/usr/bin/env python

import rospy
import numpy as np
import scipy.io
from sensor_msgs.msg import Imu
import time


def talker():

    pub = rospy.Publisher('smartphone', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10 Hz for testing
	
    # Load reference signals from MATLAB files
    agmat = scipy.io.loadmat("ag1valley.mat")
    wgmat = scipy.io.loadmat("wg1valley.mat")
    quamat = scipy.io.loadmat("quatg1valley.mat")
    ag = agmat["ag1"]
    wg = wgmat["wg1"]
    qua = quamat["quatg1"]

    K = 0
    imu = Imu()
    time.sleep(5) # wait for subs...
    while True:
        imu.orientation.w = qua[0,K]
        imu.orientation.x = qua[1,K]
        imu.orientation.y = qua[2,K]
        imu.orientation.z = qua[3,K]
        imu.linear_acceleration.x = ag[0,K]
        imu.linear_acceleration.y = ag[1,K]
        imu.linear_acceleration.z = ag[2,K]
        imu.angular_velocity.x = wg[0,K]
        imu.angular_velocity.y = wg[1,K]
        imu.angular_velocity.z = wg[2,K]
        if K < 4000:
            K = K + 1
        pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
