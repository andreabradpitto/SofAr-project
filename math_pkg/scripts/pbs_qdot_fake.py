#!/usr/bin/env python

import rospy,time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState

def qdot_simulate():
    pub = rospy.Publisher('cmdtopic', JointState, queue_size=10)

    rospy.init_node('fake_qdot_node', anonymous=True)

    time.sleep(1)

    rate = rospy.Rate(100) # 100hz
    k = 1
    while not rospy.is_shutdown():
        qdot = JointState()
        qdot.velocity = [0.1,0,0,0,0,0,0]
        qdot.effort = [k]
        qdot.header.stamp = rospy.get_rostime()
        #rospy.loginfo("qdot published")
        pub.publish(qdot)
        rate.sleep()
        k = k + 1

if __name__ == '__main__':
    try:
        qdot_simulate()
    except rospy.ROSInterruptException:
        pass
