#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState

def talker():

    pub = rospy.Publisher('logtopic', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100 Hz

    q = JointState()
    q.position = [0, 0, 0, 0, 0, 0, 0]
    pub.publish(q)

#    while not rospy.is_shutdown():
#         pub.publish(q)
#         rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
