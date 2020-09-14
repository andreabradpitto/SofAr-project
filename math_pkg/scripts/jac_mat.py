#!/usr/bin/env python
# Software License Agreement (BSD License)
#

from numpy import diag, exp, matmul, zeros
from utilities import init_float64_multiarray
import rospy
import numpy as np
import math
import T_computations as t
import J_computations as j

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import JointState

# Obtained by building the file IK_JTA.srv
from math_pkg.srv import IK_JTA
from math_pkg.srv import IK_JTARequest
from math_pkg.srv import IK_JTAResponse

# Flags to define Availability of the data required by the Service
readyErr = False
readyJ = False
readyVel = False


def sat(x, xmin, xmax):
    """!
    Saturates the input value in the given interval
    @param x: value to saturate
    @param xmin: minimum value of the interval
    @param xmax: maximum value of the interval
    @return: saturated value
    """
    if x > xmax:
        return xmax
    if x < xmin:
        return xmin
    return x


def bell(s):
    """!
    Creates a bell shaped function to regularize singular values
    @param s: singular value
    @return p: regularized singular value
    """

    b = -np.log(0.5)/0.01
    p = np.exp(-b*s*s)

    return p


def regularized_pseudoinverse(J):
    """!
    Computes the regularized pseudo inverse of mxn matrix J 
    by applying svd decomposition and multiplication for bell shaped function
    @param J: nxn matrix
    @return Jx: regularized pseudo inverse of the matrix
    """

    rows = len(J)
    cols = len(J[0])

    # Svd decomposition of the input matrix
    U, s, Vt = np.linalg.svd(J)
    Sx = np.zeros((cols, rows))

    for i in range(cols):
        for j in range(rows):
            if i == j:
                Sx[i][j] = s[i]/(s[i]**2 + bell(s[i])**2)

    Ut = U.transpose()
    V = Vt.transpose()

    # Reconstruct the pseudo inverse of the matrix with the new regularized values
    Jx = V.dot(Sx.dot(Ut))

    return Jx


def calculations_6(q_coppelia):
    """!
    Builds the 6 dof Jacobian by computing the transformation matrices,
    extracting the geometric vectors from the results and finally building the matrix.
    @param q_coppelia: vector of the joint positions
    @return Js: Jacobian matrix
    """

    p = np.pi
    n_joints = 6

    # Links lengths [m]
    L0 = 270.35/1000
    L1 = 69.00/1000
    L2 = 364.35/1000
    L3 = 69.00/1000
    Lh = math.sqrt(L2 ** 2 + L3 ** 2)
    L4 = 374.29/1000
    L5 = 0
    L6 = 368.30/1000

    # DH table of Baxter: alpha(i-1), a(i-1), d(i), theta(i)
    DH = np.array([[0, 0, L0, 0],
                   [-p/2, L1, 0, 0],
                   [0, Lh, 0, p/2],
                   [p/2, 0, L4, 0],
                   [-p/2, L5, 0, 0],
                   [p/2, 0, 0, 0],
                   [0, 0, L6, 0]])

    # Transformation matrices given DH table. T0,1 T1,2 ... T7,e
    T_rel_ini = t.DH_to_T(DH)

    # Type of joints, 1 = revolute, 0 = prismatic
    info = np.array([1, 1, 1, 1, 1, 1])

    # Initial q
    q = np.array([0, 0, 0, 0, 0, 0])

    # Transformation matrices given the configuration
    T_trans = t.transformations(T_rel_ini, q_coppelia, info)

    # T0,1 T0,2 T0,3...T0,e
    T_abs = t.abs_trans(T_trans)

    # Extract geometric vectors needed for computations
    geom_v = j.geometric_vectors(T_abs)

    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    # axis of rotation of the revolute joins projected on zero
    k = geom_v[0]
    # distances end_effector-joints projected on zero
    r = geom_v[1]

    Js = j.jacob(k, r, n_joints, info)

    return Js


def jacobian_callback(data):
    """!
    Callback Function for the Joints Positions
    """

    global J_6, readyJ

    q_coppelia = np.array(data.velocity)

    # It assumes one joint to be fixed (3rd in this case), in order to pass from 7 to 6
    q_coppelia = np.delete(q_coppelia, 2, 0)

    rospy.loginfo("Joint positions: %s\n", str(q_coppelia))

    # Compute the matrix
    J_6 = calculations_6(q_coppelia)

    rospy.loginfo("Jacobian:\n%s\n", J_6)

    # Set the Jacobian as available
    readyJ = True


def error_callback(message):
    """!
   Callback Function for the error on the position of the ee
    """

    global error, readyErr
    err_orient = np.array([message.data[:3]]).T
    err_pos = np.array([message.data[3:6]]).T
    error = np.concatenate((err_pos, err_orient), axis=0)

    rospy.loginfo("Received Position Error:\n%s\n", str(error))

    # Set the Error as available
    readyErr = True


def vel_callback(message):
    """!
    Callback Function for the linear and angular velocities of the ee
    """

    global vel, readyVel
    vel = np.array([message.data[:6]]).T

    rospy.loginfo("Received Velocities End Effector:\n%s\n", str(vel))

    # Set the Vel as available
    readyVel = True


def handle_IK_JAnalytic(req):
    """!
    Handler for the Server: computes the ee velocity Vee and the joint velocities vector q_dot 
        Vee is computed as: Vee = vel + K*error
        q_dot is computed as: q_dot = J# * Vee
    then each value of q_dot is saturated in a pre-determined interval.
    @param: request for service
    @return: Jacobian and ee velocity are sent as service
    """

    # Declaration to work with global variables
    global readyErr, readyJ, readyVel

    print("Server Analytic accepted request\n")

    if (not (readyErr and readyJ and readyVel)):
        readyErr = readyJ = readyVel = False
        rospy.logerr("Analytic J_6 service could not run: missing data.")
        return

    else:
        readyErr = readyJ = readyVel = False

        # q_dot initialization
        q_dot = JointState()

        # Gain for the Control Law
        K = 10

        # Inverse of the 6dof jacobian
        J_inv = regularized_pseudoinverse(J_6)

        # Ee velocity
        vee = vel+K*error

        # q_dot computation
        q_dot_6 = J_inv.dot(vee)

        # q_dot saturation
        for i in range(6):
            q_dot_6[i] = sat(q_dot_6[i], -1, 1)

        # Since the third Joint is blocked, its velocity must be set to 0
        q_dot.velocity = np.insert(q_dot_6, 2, 0)

        # Set vee to multiarray
        veeMultiArray = init_float64_multiarray(6, 1)
        veeMultiArray.data = vee

        return IK_JTAResponse(q_dot, veeMultiArray)


def jac_mat():
    """
    This ROS node computes the 6dof analytic jacobian. 
    It subscribes to the following topics: errors, tracking, logtopic. 
    It also works as a server for the IK_JTA service, on which it sends 
    the q_dot and the end effector velocity computed by the algorithm.
    """

    # Init node
    rospy.init_node('jac_mat', anonymous=True)

    rospy.loginfo("Server Analytic Initialized\n")

    # Subscribe for error positions
    rospy.Subscriber("errors", Float64MultiArray, error_callback)

    # Subscribe for ee velocity
    rospy.Subscriber("tracking", Float64MultiArray, vel_callback)

    # Subscribe for joint positions
    rospy.Subscriber("logtopic", JointState, jacobian_callback)

    # Service Definition
    s_vel = rospy.Service('IK_JAnalytic', IK_JTA, handle_IK_JAnalytic)

    rospy.spin()


if __name__ == '__main__':
    try:
        jac_mat()
    except rospy.ROSInterruptException:
        pass
