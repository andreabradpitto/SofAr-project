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

# Flags to define Availability of the data required by the Service
readyErr = False
readyJ = False

# Weights for the J transpose:
wp = [50, 50, 50, 1, 1, 1]#[5000, 5000, 5000, 2, 2, 2]#[50, 50, 50, 1, 1, 1] #[100, 100, 100, 2, 2, 2]
wd = [50,50,50,0.5,0.5,0.5] #[0, 0, 0, 0, 0, 0]#[50,50,50,0.5,0.5,0.5]

# Path to write the log file
import os
script_dir = os.path.dirname(__file__)  # absolute directory the script is in
rel_path = "./Output/Output.txt"
abs_file_path = os.path.join(script_dir, rel_path)
LOG_FLAG = 0

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

def j_transp(err, err_dot, J, Wp, Wd, delta_t):
    """! 
    Function that performs Inverse kinematics using the Jacobian Transpose
    approach.
    @param err: the Error on the position and orientation of the end effector.
    @param J: the Jacobian matrix of the manipulator.
    @param delta_t: sampling time.
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
        Kp = np.diag([err_norm_lin*10, err_norm_lin*10, err_norm_lin*10, err_norm_rot, err_norm_rot, err_norm_rot]) #200 and 50 before
        #Kp = np.diag(Wp)
        
        Kd = np.diag([err_dot_lin*0, err_dot_lin*0, err_dot_lin*0, 5*err_dot_rot, 5*err_dot_rot, 5*err_dot_rot])
        #Kd = np.diag(Wd)

        # Delta Joint positions using: K*J_transpose*error_transpose (Paper Formula)
        dq = J.T.dot(Kp.dot(err)+Kd.dot(err_dot))
        
        # Joint velocities, being dq = q_dot*delta_t
        #q_dot.velocity = dq/delta_t
	q_dot.velocity = np.zeros((max(J.shape))).T
        
    # Position Reached
    else:

        # Stop the Joint: zero velocities
        q_dot.velocity = np.zeros((max(J.shape))).T

    return q_dot
    

# Handler for the Server
def handle_IK_Jtransp(req):

    # Declaration to work with global variables
    global readyErr, readyJ, wp, wd

    print"Server J Transpose accepted request\n"

    if (not (readyErr and readyJ)):
        readyErr = readyJ = False
        rospy.logerr("J Transpose service could not run: missing data.")
        return 
    else:
        readyErr = readyJ = False
	if (LOG_FLAG):
            now = rospy.get_rostime()
            timestamp =  now.secs + float(now.nsecs)/1000000000
            with open(abs_file_path, 'a') as f:
                f.write(str(timestamp))
                f.write("\t")
                f.write(str(j_transp(error[:6], error[6:], J, wp, wd, 0.01).velocity.T))
                f.write("\n\n")

        return IK_JtraResponse(j_transp(error[:6], error[6:], J, wp, wd, 0.01))        
    #return IK_JtraResponse(j_transp(error, J, 0.01))

# Callback Function for the error on the position (error on Xee)
def error_callback(message):

    # Declaration to work with global variables
    global error,readyErr

    # Re-arranging the error vector in the needed form:
    err_orient = np.array([message.data[:3]]).T
    err_pos = np.array([message.data[3:6]]).T
    err_vel_lin = np.array([message.data[6:9]]).T
    err_vel_rot = np.array([[0,0,0]]).T
    error = np.concatenate((err_pos, err_orient, err_vel_lin, err_vel_rot), axis=0)
    #error = np.array([message.data[:6]]).T
    rospy.loginfo("Received Position Error:\n%s\n", str(error))
    #print(readyErr)
    
    # Set the Error as available
    readyErr = True
    #print(readyErr)

# Callback Function for the Jacobian matrix
def jacobian_callback(message):
    
    # Declaration to work with global variables
    global J, readyJ

    J = np.array(message.data)
    J = J.reshape(6,7)
    #rospy.loginfo("Received Jacobian::\n%s\n", str(J))

    # Set the J as available
    readyJ = True

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

