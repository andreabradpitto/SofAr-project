#!/usr/bin/env python

import rospy
import numpy as np
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

# Weights for the J transpose:
wp = [20, 20, 20, 2, 2, 2]

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

def j_transp(err, J, Wp, delta_t, min, max):
    """! 
    Function that performs Inverse kinematics using the Jacobian Transpose
    approach.
    @param err: the Error on the position and orientation of the end effector.
    @param J: the Jacobian matrix of the manipulator.
    @param delta_t: sampling time.
    @param Wp: weights for the error on the position and orientation.
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

    # q_dot initialization
    q_dot = JointState()

    # Working Case
    if (err_norm>0.0001):

        # Weights as diagonal Matrices:
        Kp = np.diag([err_norm_lin*Wp[0], err_norm_lin*Wp[1], err_norm_lin*Wp[2], err_norm_rot*Wp[3], err_norm_rot*Wp[4], err_norm_rot*Wp[5]])

        # Delta Joint positions using:J_transpose*K*error_transpose (Paper Formula)
        dq = J.T.dot(Kp.dot(err))
        
        # Joint velocities, being dq = q_dot*delta_t
        q_dot.velocity = dq/delta_t

        # Saturation of the output within the desired interval (min,max)
        for i in range(len(q_dot.velocity)):
            q_dot.velocity[i] = sat(q_dot.velocity[i],min,max)
        
    # Position Reached
    else:

        # Stop the Joints: zero velocities
        q_dot.velocity = np.zeros((max(J.shape))).T

    return q_dot
    

# Handler for the Server
def handle_IK_Jtransp(req):

    # Declaration to work with global variables
    global readyErr, readyJ, wp

    print"Server J Transpose accepted request.\n"

    # Case in which the necessary data is not available
    if (not (readyErr and readyJ)):
        readyErr = readyJ = False
        rospy.logerr("J Transpose service could not run: missing data.")
        return 
    # Case in which data is available
    else:
        readyErr = readyJ = False
        if (LOG_FLAG):
            now = rospy.get_rostime()
            timestamp =  now.secs + float(now.nsecs)/1000000000
            with open(abs_file_path, 'a') as f:
                f.write(str(timestamp))
                f.write("\t")
                f.write(str(j_transp(error[:6], J, wp, 0.01, MIN, MAX).velocity.T))
                f.write("\n\n")

    return IK_JtraResponse(j_transp(error[:6], J, wp, 0.01, MIN, MAX))        

# Callback Function for the error on the position (error on Xee)
def error_callback(message):

    # Declaration to work with global variables
    global error,readyErr

    # Re-arranging the error (position and orientation) vector in the needed form:
    err_orient = np.array([message.data[:3]]).T
    err_pos = np.array([message.data[3:6]]).T

    error = np.concatenate((err_pos, err_orient), axis=0)
    
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

# Main body containing 2 Subscribers and the Service defition
def JT_server():

    # Node Initialization
    rospy.init_node('IK_Jtransp_server')

    rospy.loginfo("Server Initialized.\n")

    # Subscribers
    rospy.Subscriber("errors", Float64MultiArray, error_callback)
    rospy.Subscriber("jacobian", Float64MultiArray, jacobian_callback)

    s = rospy.Service('IK_Jtransp', IK_Jtra, handle_IK_Jtransp)
    rospy.spin()
    
if __name__ == "__main__":
    JT_server()

