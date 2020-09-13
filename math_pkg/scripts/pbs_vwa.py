#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rospy_tutorials.msg import Floats
from std_msgs.msg import MultiArrayDimension

def vwa():
    pub = rospy.Publisher('tracking', Float64MultiArray, queue_size=10)

    rospy.init_node('v_w_a_node', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        vwa = Float64MultiArray()
        vwa.layout.dim.append(MultiArrayDimension())
        vwa.layout.dim.append(MultiArrayDimension())
        vwa.layout.dim[0].label ="rows"
        vwa.layout.dim[0].size = 9
        vwa.layout.dim[1].label ="columns"
        vwa.layout.dim[1].size = 1
        vwa.data = [-0.0001,0.1175,0,0,0,0,-0.0118,0,0]
        pub.publish(vwa)
        rate.sleep()

if __name__ == '__main__':
    try:
        vwa()
    except rospy.ROSInterruptException:
        pass
