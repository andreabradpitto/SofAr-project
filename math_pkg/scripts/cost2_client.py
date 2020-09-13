#!/usr/bin/env python

import sys
import rospy
from math_pkg.srv import *

def task_client(x):
    rospy.wait_for_service('cost', timeout = 0.4)
    try:
        task = rospy.ServiceProxy('cost', Cost)
        resp1 = task()
        return [resp1.qdot1opt1 ,resp1.qdot1opt2 ,resp1.qdot2opt1 ,resp1.qdot2opt2]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
	x = 'test'
	print "Requesting %s"%(x)
	print "%s -> %s"%(x, task_client(x))
