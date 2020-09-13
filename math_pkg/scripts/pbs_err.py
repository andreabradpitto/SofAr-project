#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from rospy_tutorials.msg import Floats

def errs():
    pub = rospy.Publisher('errors', Float64MultiArray, queue_size=10)

    rospy.init_node('computed_errors_node', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        err = Float64MultiArray()
        err.layout.dim.append(MultiArrayDimension())
        err.layout.dim.append(MultiArrayDimension())
        err.layout.dim[0].label ="rows"
        err.layout.dim[0].size = 12
        err.layout.dim[1].label ="columns"
        err.layout.dim[1].size = 1
        err.data = [2.1676,-2.1676,0.1136,0.3956,0.3680,-0.1168,0,0,0]
        pub.publish(err)
        rate.sleep()

if __name__ == '__main__':
    try:
        errs()
    except rospy.ROSInterruptException:
        pass
