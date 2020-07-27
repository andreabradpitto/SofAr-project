#!/usr/bin/env python

import rospy
import numpy as np

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

def init_float64_multiarray(rows,columns):
    """!
    Function that initializes a Float64MultiArray of size rows x columns.
    @param rows: Number of rows of the returned multiarray.
    @param columns: Number of columns of the returned multiarray.
    @return empty Float64MultiArray instance.
    """
    a = Float64MultiArray()
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim[0].label ="rows"
    a.layout.dim[0].size = rows
    a.layout.dim[1].label ="columns"
    a.layout.dim[1].size = columns
    return a

def j_transp(err, J, delta_t):
    """! 
    Function that performs Inverse kinematics using the Jacobian Transpose
    approach.
    @param err: the Error on the position and orientation of the end effector.
    @param J: the Jacobian matrix of the manipulator.
    @param delta_t: sampling time.
    @return: a Float64MultiArray containing the Joint Velocities.
    """

    # q_dot initialization
    q_dot = JointState()

    # Alpha (Regulation Factor) computation
    numerator = J.dot(J.T.dot(err))
    alpha = (err.T).dot(numerator)/(numerator.T.dot(numerator))
    
    # Delta Joint positions using: alpha*J_transpose*error_transpose (Paper Formula)
    dq = alpha*(J.T.dot(err))
        
    # Joint velocities, being dq = q_dot*delta_t
    q_dot.velocity = dq/delta_t

    return q_dot
    

# Handler for the Server
def handle_IK_Jtransp(req):

    print"Server J Transpose accepted request\n"
    return IK_JtraResponse(j_transp(error, J, 0.01))

# Callback Function for the error on the position (error on Xee)
def error_callback(message):
    
    global error
    err_orient = np.array([message.data[:3]]).T
    err_pos = np.array([message.data[3:6]]).T
    error = np.concatenate((err_pos,err_orient), axis=0)
    #error = np.array([message.data[:6]]).T
    #rospy.loginfo("Received Position Error:\n%s\n", str(error))

# Callback Function for the Jacobian matrix
def jacobian_callback(message):

    global J
    J = np.array(message.data)
    J = J.reshape(6,7)
    #rospy.loginfo("Received Jacobian::\n%s\n", str(J))

# Main body containing 2 Subscribers and the Service defition
def JT_server():

    # Node Initialization
    rospy.init_node('IK_Jtransp_server')

    rospy.loginfo("Server Initialized\n")

    # Subscribers
    rospy.Subscriber("errors", Float64MultiArray, error_callback)
    rospy.Subscriber("jacobian", Float64MultiArray, jacobian_callback)

    s = rospy.Service('IK_Jtransp', IK_Jtra, handle_IK_Jtransp)
    rospy.spin()
    
if __name__ == "__main__":
    JT_server()