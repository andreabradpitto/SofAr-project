#!/usr/bin/env python

import rospy
import numpy as np

# Obtained by building the file IK_Jtra.srv
from ros_essentials_cpp.srv import IK_Jtra
from ros_essentials_cpp.srv import IK_JtraRequest
from ros_essentials_cpp.srv import IK_JtraResponse

# Necessary to use Numpy Arrays 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray,MultiArrayDimension


def j_transp(err, J, delta_t):
    """! 
    Function that performs Inverse kinematics using the Jacobian Transpose
    approach.
    @param err: the Error on the position and orientation of the end effector.
    @param J: the Jacobian matrix of the manipulator.
    @param delta_t: sampling time.
    @return: a Float64MultiArray containing the Joint Velocities.
    """
    
    # Number of Joints
    NJOINTS = 7

    # q_dot initialization
    q_dot = init_float64_multiarray(NJOINTS,1)

    # Alpha (Regulation Factor) computation
    numerator = J.dot(J.T.dot(err))
    alpha = (err.T).dot(numerator)/(numerator.T.dot(numerator))
    
    # Delta Joint positions using: alpha*J_transpose*error_transpose (Paper Formula)
    dq = alpha*(J.T.dot(err))
        
    # Joint velocities, being dq = q_dot*delta_t
    q_dot.data = dq/delta_t

    return q_dot
    

# Handler for the Server
def handle_IK_Jtransp(req):

    print"Server accepted request\n"
    return IK_JtraResponse(j_transp(error, J, 0.1))

# Callback Function for the error on the position (error on Xee)
def error_callback(message):

    global error
    error = np.array([message.data[:6]]).T
    rospy.loginfo("Received Position Error:\n%s\n", str(error))

# Callback Function for the Jacobian matrix
def jacobian_callback(message):

    global J
    J = np.array(message.data[36:])
    J = J.reshape(6,7)
    rospy.loginfo("Received Jacobian: %s\n", str(J))

# Main body containing 2 Subscribers and the Service defition
def JT_server():

    # Node Initialization
    rospy.init_node('IK_Jtransp_server')

    rospy.loginfo("Server Initialized\n")

    # Subscribers
    rospy.Subscriber("errors", numpy_msg(Floats), error_callback)
    rospy.Subscriber("Jacobian_matrix", numpy_msg(Floats), jacobian_callback)

    s = rospy.Service('IK_Jtransp', IK_Jtra, handle_IK_Jtransp)
    rospy.spin()
    
if __name__ == "__main__":
    JT_server()
