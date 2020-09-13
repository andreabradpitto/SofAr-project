#!/usr/bin/env python

import rospy
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg

def jack():
    pub = rospy.Publisher('jacobian', Float64MultiArray, queue_size=10)

    rospy.init_node('Jac_node', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
	   J = Float64MultiArray()
	   J.layout.dim.append(MultiArrayDimension())
	   J.layout.dim.append(MultiArrayDimension())
	   J.data = [0.368,0.0408,0.1895,-0.0875,-0.0765,0,-0.3660,0.7803,0,0.3314,0,0.01,0,0,0,-0.7113,-0.3154,-0.3637,-0.36,0,-0.0385,0,0,0.8572,0,0.9781,0,.1045,0,1,0,1,0,1,0,1,0,.515,0,-0.2079,0,-0.9945];pub.publish(J);rate.sleep()

if __name__ == '__main__':
    try:
        jack()
    except rospy.ROSInterruptException:
        pass
