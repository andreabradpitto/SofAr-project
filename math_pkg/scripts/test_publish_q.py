#!/usr/bin/env python

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def talker():

    pub = rospy.Publisher('baxter_data', numpy_msg(Floats), queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(100) # 100 Hz

    q = np.array([0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
    pub.publish(q)

##  while not rospy.is_shutdown():
##        pub.publish(q)
##        rate.sleep()
##


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
