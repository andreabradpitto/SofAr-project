#!/usr/bin/env python
# Software License Agreement (BSD License)
#

import rospy
import numpy as np
import math
import T_computations as t
import J_computations as j

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg



# Calculations to compute 6 dof Jacobian
def calculations_6(q_smarphone):

    p = np.pi
    n_joints = 6

    # Links length. [mm]
    L0 = 270.35
    L1 = 69.00
    L2 = 364.35
    L3 = 69.00
    Lh = math.sqrt(L2** 2 + L3** 2)
    L4 = 374.29
    L5 = 10.00
    L6 = 368.30

    # DH table of Baxter: alpha(i-1), a(i-1), d(i), theta(i).
    # Last row relates last joint to end-effector.
    DH = np.array([[0, 0, L0, 0],
                   [-p/2, L1, 0, p/2],
                   [0, Lh, 0, p/2],
                   [p/2, 0, L4, 0],
                   [-p/2, L5, 0, 0],
                   [p/2, 0, 0, 0],
                   [0, 0, L6, 0]])

    # Trasformation matrices given DH table. T0,1 T1,2 ... T7,e
    T_rel_ini = t.DH_to_T(DH)
    
    # type of joints, 1 = revolute, 0 = prismatic.
    info = np.array([1, 1, 1, 1, 1, 1])

    # initial q
    q = np.array([0, 0, 0, 0, 0, 0])

    ########
    # Entry point when receiving q from coppeliasim!!
    # Use q_smartphone

    # transformations matrices given the configuration.
    T_trans = t.transformations(T_rel_ini, q, info)

    # T0,1 T0,2 T0,3...T0,e
    T_abs = t.abs_trans(T_trans)

    # extract geometric vectors needed for computations.
    geom_v = j.geometric_vectors(T_abs)

    np.set_printoptions(precision = 4)
    np.set_printoptions(suppress = True)

    k = geom_v[0] # axis of rotation of the revolute joins projected on zero
    r = geom_v[1] # distances end_effector, joints projected on zero.

    Js = j.jacob(k, r, n_joints, info)
    J = Js[0]
    return J


# Calculations to compute 7 dof Jacobian
def calculations_7(q_smarphone):
    
    p = np.pi
    n_joints = 7

    # Links length. [mm]
    L0 = 270.35
    L1 = 69.00
    L2 = 364.35
    L3 = 69.00
    L4 = 374.29
    L5 = 10.00
    L6 = 368.30

    # DH table of Baxter: alpha(i-1), a(i-1), d(i), theta(i).
    # Last row relates 7-th joint to end-effector.
    DH = np.array([[0, 0, L0, 0],
                   [-p/2, L1, 0, p/2],
                   [p/2, 0, L2, 0],
                   [-p/2, L3, 0, 0],
                   [p/2, 0, L4, 0],
                   [-p/2, L5, 0, 0],
                   [p/2, 0, 0, 0],
                   [0, 0, L6, 0]])

    # Trasformation matrices given DH table. T0,1 T1,2 ... T7,e
    T_rel_ini = t.DH_to_T(DH)
    
    # type of joints, 1 = revolute, 0 = prismatic.
    info = np.array([1, 1, 1, 1, 1, 1, 1])

    # initial q
    q = np.array([0, 0, 0, 0, 0, 0, 0])

    ########
    # Entry point when receiving q from coppeliasim!!
    # Use q_smartphone

    # transformations matrices given the configuration.
    T_trans = t.transformations(T_rel_ini, q, info)

    # T0,1 T0,2 T0,3...T0,e
    T_abs = t.abs_trans(T_trans)

    # extract geometric vectors needed for computations.
    geom_v = j.geometric_vectors(T_abs)

    np.set_printoptions(precision = 4)
    np.set_printoptions(suppress = True)

    k = geom_v[0] # axis of rotation of the revolute joins projected on zero
    r = geom_v[1] # distances end_effector, joints projected on zero.

    Js = j.jacob(k, r, n_joints, info)
    J = Js[0]
    return J


def jacobian(data):

    q_smartphone=np.array(data.data)
    rospy.loginfo(rospy.get_caller_id() + "\n\n\n\nJoint positions", str(q_smartphone))
    
    # Compute matrices
    J_6=calculations_6(q_smartphone)
    J_7=calculations_7(q_smartphone)

    # Reshape matrices to send
    J_ros=np.concatenate((J_6.flatten(),J_7.flatten()),axis=None)

    # Define publisher for Jacobian Matrix
    pub = rospy.Publisher('Jacobian_matrix', numpy_msg(Floats), queue_size=10)

    # Publish jacobians
    while not rospy.is_shutdown(): # so if there is control-C
        rate = rospy.Rate(10) # 10hz, 10 times per second
        rospy.loginfo(J_ros) #prints message to screen, log and rosout
        pub.publish(J_ros)
        rate.sleep()

def jac_mat():

    # Init node
    rospy.init_node('jac_mat', anonymous=True)

    # Subscribe for joint positions
    rospy.Subscriber("joint_pos", Floats, jacobian)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        jac_mat()
    except rospy.ROSInterruptException:
        pass
