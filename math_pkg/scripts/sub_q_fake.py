#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState

def callback (q_data):
    rospy.loginfo(q_data.position[0])


def sub_simulate():

    rospy.init_node('fake_q_node', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    rospy.Subscriber("logtopic", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        sub_simulate()
    except rospy.ROSInterruptException:
        pass
