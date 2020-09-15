#!/usr/bin/env python

import rospy
import numpy as np
from JT_enhance import JT_enhance
from utilities import init_float64_multiarray
from utilities import sat

# Obtained by building the file IK_Jtra.srv
from math_pkg.srv import IK_Jtra
from math_pkg.srv import IK_JtraRequest
from math_pkg.srv import IK_JtraResponse

# Necessary to use Numpy Arrays 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray,MultiArrayDimension
from sensor_msgs.msg import JointState

# Flags to define Availability of the data required by the Service
readyErr = False
readyJ = False
readyJp = False

# Weights for the J transpose:
wp = [10, 10, 10, 1, 1, 1]
wd = [0,0,0,0,0,0] #[0.01,0.01,0.01,0,0,0] 

# Joint Velocities to reach Elbow Configuration when stucked in a Singularity [7x1]
q_dot_elbow = np.array([[0, -1, 0, 10, 0, -0.0001, 0]], dtype=float).T

# Tuning Variables for the Correction Term
ALPHA = 250
BETA = 250

# Extremes for the Output Saturation (Joint Velocities)
MIN = -1
MAX = 1

# Path to write the log file
import os
script_dir = os.path.dirname(__file__)  # absolute directory of the code
rel_path = "./Output_Jtranspose.txt"
abs_file_path = os.path.join(script_dir, rel_path)

# Set to write (1) or not (0) a Log file containing the output
LOG_FLAG = 0

def j_transp(err, err_dot, J, Wp, Wd, delta_t, Jp, Ji, alpha, beta, q_dot_pref, min, max):
    """! 
    Function that performs Inverse kinematics using the Jacobian Transpose
    approach, considering also the velocity error and the enhancement for 
    singularity escaping (from "Advances in Robot Kinematics: Analysis and
    Control", J.Lenarcic and M. L. Husty).
    @param err: the Error on the position and orientation of the end effector.
    @param J: the Jacobian matrix of the manipulator.
    @param delta_t: sampling time.
    @param Wp: weights for the error on the position and orientation. 
    @param Wd: weights for the error on the velocity.
    @param Jp: matrix with inter-joints axes (by rows) used in the enhancement.
    @param Ji: matrix with rotation axes (by rows) used in the enhancement.
    @param alpha: tuning parameter (scalar) used in the enhancement.
    @param beta: tuning parameter (scalar) used in the enhancement.
    @param q_dot_pref: joint velocities to escape from singularity.
    @param min: minimum value for the output saturation.
    @param max: maximum value for the output saturation.
    @return: a Float64MultiArray containing the Joint Velocities.
    """
    # Norm of the position error
    err_norm = np.linalg.norm(err)

    # Norm of the linear part of the position error
    err_norm_lin = np.linalg.norm(err[:3])

    # Norm of the rotational part of the position error
    err_norm_rot = np.linalg.norm(err[3:])

    # Norm of the derivative of the position error
    err_dot_norm = np.linalg.norm(err_dot)

    # Norm of the linear part of the derivative of the position error
    err_dot_lin = np.linalg.norm(err_dot[:3])
   
    # Norm of the rotational part of the derivative of the position error
    err_dot_rot = np.linalg.norm(err_dot[3:])

    # q_dot initialization
    q_dot = JointState()

    # Working Case
    if ((err_norm>0.0001) or (err_dot_norm>0.001)):

        # Weights as diagonal Matrices:
        Kp = np.diag([err_norm_lin*Wp[0], err_norm_lin*Wp[1], err_norm_lin*Wp[2], err_norm_rot*Wp[3], err_norm_rot*Wp[4], err_norm_rot*Wp[5]])

        Kd = np.diag([err_dot_lin*Wd[0], err_dot_lin*Wd[1], err_dot_lin*Wd[2], err_dot_rot*Wd[3], err_dot_rot*Wd[4], err_dot_rot*Wd[5]])

        # Delta Joint positions using: J_transpose*K*error_transpose + correction term (Paper Formula)
        dq = J.T.dot(Kp.dot(err)+Kd.dot(err_dot))+JT_enhance(Jp, Ji, err[:3], alpha, beta, q_dot_pref)

        # Joint velocities, being dq = q_dot*delta_t
        q_dot.velocity = dq/delta_t
        
        # Saturation of the output within the desired interval (min,max)
        for i in range(len(q_dot.velocity)):
            q_dot.velocity[i] = sat(q_dot.velocity[i],min,max)
        
    # Position Reached (within the indicated precision)
    else:

        # Stop the Joints: zero velocities
        q_dot.velocity = np.zeros((max(J.shape))).T

    return q_dot
    

# Handler for the Server
def handle_IK_Jtransp(req):

    # Declaration to work with global variables
    global readyErr, readyJ, readyJp, Jp, Ji, q_dot_elbow, wp, wd

    print"Server J Transpose accepted request.\n"
    
    # Case in which the necessary data is not available
    if (not (readyErr and readyJ and readyJp)):
        readyErr = readyJ = readyJp = False
        rospy.logerr("J Transpose service could not run: missing data.")
        return 
    # Case in which data is available
    else:
        readyErr = readyJ = readyJp = False
        if (LOG_FLAG):
            now = rospy.get_rostime()
            timestamp =  now.secs + float(now.nsecs)/1000000000
            with open(abs_file_path, 'a') as f:
                f.write(str(timestamp))
                f.write("\t")
                f.write(str(j_transp(error[:6], error[6:], J, wp, wd, 0.01, Jp, Ji, ALPHA, BETA, q_dot_elbow, MIN, MAX).velocity.T))
                f.write("\n\n")

        return IK_JtraResponse(j_transp(error[:6], error[6:], J, wp, wd, 0.01, Jp, Ji, ALPHA, BETA, q_dot_elbow,MIN, MAX))        

# Callback Function for the error on the position (error on Xee)
def error_callback(message):

    # Declaration to work with global variables
    global error,readyErr

    # Re-arranging the error (position and velocity) vector in the needed form:
    err_orient = np.array([message.data[:3]]).T
    err_pos = np.array([message.data[3:6]]).T

    err_vel_lin = np.array([message.data[6:9]]).T
    err_vel_rot = np.array([[0,0,0]]).T

    error = np.concatenate((err_pos, err_orient, err_vel_lin, err_vel_rot), axis=0)
    
    # Set the Error as available
    readyErr = True

# Callback Function for the Jacobian matrix
def jacobian_callback(message):
    
    # Declaration to work with global variables
    global J, readyJ
    
    # Baxter Jacobian Matrix 
    J = np.array(message.data)
    J = J.reshape(6,7)

    # Set the J as available
    readyJ = True

# Callback Function for the Axes required for the Correction Term
def axes_callback(message):
    
    # Declaration to work with global variables
    global Jp, Ji, readyJp

    # Inter-joints axes and rotation axes used in the correction term
    Jp = np.array(message.data[:18]).reshape(6,3)
    Ji = np.array(message.data[18:]).reshape(3,3)

    # Set the Jp as available
    readyJp = True

# Main body containing 2 Subscribers and the Service defition
def JT_server():

    # Node Initialization
    rospy.init_node('IK_Jtransp_server')

    rospy.loginfo("Server Initialized.\n")

    # Subscribers
    rospy.Subscriber("errors", Float64MultiArray, error_callback)
    rospy.Subscriber("jacobian", Float64MultiArray, jacobian_callback)
    rospy.Subscriber("axes", Float64MultiArray, axes_callback)

    s = rospy.Service('IK_Jtransp', IK_Jtra, handle_IK_Jtransp)
    rospy.spin()
    
if __name__ == "__main__":
    JT_server()

