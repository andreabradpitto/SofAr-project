#!/usr/bin/env python

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from threading import Lock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import T_computations as t
import J_computations as j

# GLOBAL VARIABLES

pub_err = rospy.Publisher('Data_for_errors', Float64MultiArray, queue_size=10)
pub_track = rospy.Publisher('tracking', Float64MultiArray, queue_size=10)
pub_jac = rospy.Publisher('jacobian', Float64MultiArray, queue_size=10)

mutex = Lock()

#######################################
# Define initial configuration of arm.
#######################################
p = np.pi
n_joints = 7

# Links length. [cm]
L0 = 27.035
L1 = 6.900
L2 = 36.435
L3 = 6.900
L4 = 37.429
L5 = 1.000
L6 = 36.830

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
T_dh = t.DH_to_T(DH)

# Jacobian at time k minus 1
Jkmin1 = np.zeros((6,7))

# type of joints, 1 = revolute, 0 = prismatic.
info = np.array([1, 1, 1, 1, 1, 1, 1])

# Need this variable to store initial rotation matrix, because it's equal to
# rotation matrix from 0 to inertial frame.
ini = 0

# Sync variables
key_bax = 0
key_smart = 0
key_dot = 0

# Sampling time
dt = 0.01

# Assing position of e.e. w.r.t. 0. One used for integration, the other to compute errors.
x_0e_kmin1 = np.zeros((3,1))
x_0e_k = x_0e_kmin1
x_0e_kmin1B = np.zeros((3,1))

# Initial velocity of end effector w.r.t. zero
v_0e_kmin1 = np.zeros((3,1)) # starting velocity
v_0e_k = v_0e_kmin1
v_0e_kmin1B = np.zeros((3,1))

# Define the q's and q dots
q = np.zeros((1,7))
q_dot = np.zeros((7,1))

# Definition of some variables that change over time when a callback is triggered
R0inert = np.zeros((3,3))
R0e_kmin1 = np.zeros((3,3))
R0e_k = np.zeros((3,3))
Rimu_inert_k = np.zeros((3,3))

omega_imu_inert = np.zeros((3,1))
a_imu_inert = np.zeros((3,1))

omega_0e = np.zeros((3,1))
a_0e = np.zeros((3,1))

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

def main_callback():

    ######################################################################
    # Entry point when receveing all the data, from smart and baxter.
    global x_0e_kmin1B, x_0e_k, v_0e_kmin1B, v_0e_k, a_0e, omega_0e, R0e_kmin1, R0e_k, Jkmin1

    ################################
    # Send Jacobian matrix
    ################################
    J = init_float64_multiarray(6*7, 1)
    J.data = Jkmin1.reshape(6*7, 1)
    pub_jack.publish(J)
    
    ################################
    # Send to Error block the data
    ################################

    # Send R0e_k, R0e_kmin1; x_0e_k, x_0e_kmin1B; v_0e_k, v_0e_kmin1B;
    Rg_Re_xg_xe_vg_ve = np.array([R0e_k[0][0], R0e_k[0][1], R0e_k[0][2], R0e_k[1][0], R0e_k[1][1], R0e_k[1][2], R0e_k[2][0], R0e_k[2][1], R0e_k[2][2], R0e_kmin1[0][0], R0e_kmin1[0][1], R0e_kmin1[0][2], R0e_kmin1[1][0], R0e_kmin1[1][1], R0e_kmin1[1][2], R0e_kmin1[2][0], R0e_kmin1[2][1], R0e_kmin1[2][2], x_0e_k[0][0], x_0e_k[1][0], x_0e_k[2][0], x_0e_kmin1B[0][0], x_0e_kmin1B[1][0], x_0e_kmin1B[2][0], v_0e_k[0][0], v_0e_k[1][0], v_0e_k[2][0], v_0e_kmin1B[0][0], v_0e_kmin1B[1][0], v_0e_kmin1B[2][0]], dtype=np.float64)
    R_x_v = init_float64_multiarray(30, 1)
    R_x_v.data = Rg_Re_xg_xe_vg_ve
    pub_err.publish(R_x_v)

    ################################
    # Send to Math block the data.
    ################################

    # send v_0e_k, omega_0e, a_0e
    vg_omega_a = np.array([v_0e_k[0][0], v_0e_k[1][0], v_0e_k[2][0], omega_0e[0][0], omega_0e[1][0], omega[2][0], a_0e[0][0], a_0e[1][0], a_0e[2][0]], dtype=np.float64)
    v_w_a = init_float64_multiarray(9, 1)
    v_w_a.data = vg_omega_a
    pub_track.publish(v_w_a)
    

def baxter_callback(data):

    global ini, q, R0e_kmin1, R0inert, Jkmin1, x_0e_kmin1B, x_0e_kmin1, v_0e_kmin1B, key_bax, key_dot, key_smart
    
    ####################################################
    # Read from publisher of v-rep the q configuration.
    ####################################################
    
    # configuration at time kmin1
    q = np.array([data.position])

    # relative T's with the configuration passed.
    T_rel_kmin1 = t.transformations(T_dh, q, info)
    # absolute T's
    T_abs_kmin1 = t.abs_trans(T_rel_kmin1)

    # geometric vectors needed to compute jacobian.
    geom = j.geometric_vectors(T_abs_kmin1)

    # jacobian computation
    Jkmin1 = j.jacob(geom[0], geom[1], n_joints, info)

    # Transformation matrix from 0 to end effector at time k
    T0e_kmin1 = T_abs_kmin1[7]

    # end effector position of baxter at time k
    for j in range(3):
      x_0e_kmin1B[j] = T0e_kmin1[j][3]

    # end effector orientation of baxter at time k. At time 0 i have the
    # orientation of zero with respect of inertial frame also.
    for j in range(3):
      for k in range(3):
        R0e_kmin1[j][k] = T0e_kmin1[j][k]

    if ini == 0:
      R0inert = R0e_kmin1 # Constant in time.
      x_0e_kmin1 = x_0e_kmin1B # Initially they are equal
      ini = ini + 1
    
    mutex.acquire()
    try:
        if key_bax == 0:
            key_bax = key_bax + 1
        
        if (key_bax == 1 and key_dot == 1):
            x_dot = np.dot(Jkmin1, q_dot)
            for i in range(3):
                v_0e_kmin1B[i][0] = x_dot[i][0]

            if(key_smart >= 1):
                key_bax = 0
                key_dot = 0
                key_smart = 0

                main_callback()    
    finally:
        mutex.release()

def dot_callback(data):

    global q_dot, Jkmin1, v_0e_kmin1B, key_bax, key_dot, key_smart

    ###################################################################################
    # Read from topic to get the generated q_dot, needed to compute the velocity of
    # baxter's arm.
    ###################################################################################

    q_dot = np.array([data.velocity]).transpose()

    mutex.acquire()
    try:
        if key_dot == 0:
            key_dot = key_dot + 1
        
        if (key_bax == 1 and key_dot == 1):
            x_dot = np.dot(Jkmin1, q_dot)
            for i in range(3):
                v_0e_kmin1B[i][0] = x_dot[i][0]

            if(key_smart >= 1):
                key_bax = 0
                key_dot = 0
                key_smart = 0

                main_callback()    
    finally:
        mutex.release()

def smart_callback(data):

    global omega_imu_inert, a_imu_inertial, Rimu_inert_k, R0e_k, x_0e_kmin1, x_0e_k, v_0e_kmin1, v_0e_k, a_0e, omega_0e, key_bax, key_dot, key_smart
    Data = data.data
    #####################################################################################
    # Read from topic, get Rimu,inertial; angular velocity and linear acceleration
    # of imu with respect to inertial frame, all expressed in imu frame at time kplus1.
    #####################################################################################

    # Supposing all passed via array.
    k = 0
    
    for i in range(3):
        for j in range(3):
            Rimu_inert_k[i][j] = Data[k+j]
        k = k + 3
    
    Rinert_imu_k = np.transpose(Rimu_inert_k)
    
    # angular velocity of imu (end effector) w.r.t. inertial frame projected on imu frame
    for i in range(3):
        omega_imu_inert[i] = Data[k+i]

    k = k + 3
    
    # linear acceleration of imu (end effector) w.r.t. inertial frame projected on imu frame
    for i in range(3):
        a_imu_inert[i] = Data[k+i]

    # imu frame at time k is superimposed to e.e. frame at time k. Innertial and zero
    # are not moving and since the inertial is placed where the e.e. was at its initial conditions,
    # i can compute R0e_k
    R0e_k = np.dot(R0inert, Rinert_imu_k)

    # Since inertial is not moving, the angular velocity and linear acceleration are the same
    # if calculated w.r.t. 0, however i need to project them in zero.
    omega_0e = np.dot(R0e_k, omega_imu_inert)

    a_0e = np.dot(R0e_k, a_imu_inert)

    ##############
    # Integration
    ##############

    # Target velocity.
    v_0e_k = v_0e_kmin1 + a_0e*dt

    # Target position.
    x_0e_k = x_0e_kmin1 + v_0e_kmin1*dt + 0.5*a_0e*dt*dt

    mutex.acquire()
    try:
        key_smart = key_smart + 1
        if (key_bax == 1 and key_dot == 1 and key_smart >= 1):
            key_bax = 0
            key_dot = 0
            key_smart = 0
            main_callback()
    finally:
        mutex.release()

    # Update this vectors to compute integration at next steps.

    x_0e_kmin1 = x_0e_k
    v_0e_kmin1 = v_0e_k

        
def subs():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subs', anonymous=True)

    # Receive data from smartphone and baxter.
    rospy.Subscriber("smart_data", numpy_msg(Floats), smart_callback)
    rospy.Subscriber("cmdtopic", JointState, baxter_callback)
    rospy.Subscriber("jointvel", JointState, dot_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()  

  
if __name__ == '__main__':
    
    subs()