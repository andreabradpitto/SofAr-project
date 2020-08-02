#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState

def vrep_simulate():
    pub = rospy.Publisher('logtopic', JointState, queue_size=10)

    rospy.init_node('vrep_node', anonymous=True)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        q = JointState();q.position = [0,-0.5411,0,0.7505,0,1.2566,0];pub.publish(q);rate.sleep()

if __name__ == '__main__':
    try:
        vrep_simulate()
    except rospy.ROSInterruptException:
        pass
