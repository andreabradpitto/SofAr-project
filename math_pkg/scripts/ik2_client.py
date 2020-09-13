#!/usr/bin/env python

import sys
import rospy
from math_pkg.srv import *

def task_client(x):
    rospy.wait_for_service('ik')
    try:
        task = rospy.ServiceProxy('ik', IK)
        resp1 = task()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    x = 'test'
    print "Requesting %s"%(x)
    res = task_client(x)
    print "%s -> %s %s"%(x, res.qdot1, res.qdot2)