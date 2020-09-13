#!/usr/bin/env python

import sys
import rospy
from math_pkg.srv import *

def task_client(x):
    rospy.wait_for_service('safety')
    try:
        task = rospy.ServiceProxy('safety', Safety)
        resp1 = task()
        return resp1.qdot
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
	x = 'test'
	print "Requesting %s"%(x)
	print "%s -> %s"%(x, task_client(x))
