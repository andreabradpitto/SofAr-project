#!/usr/bin/env python

import rospy
import numpy as np
import math
import tf
import utilities as util
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import T_computations as t
import J_computations as j
import time

# GLOBAL VARIABLES

pub_err = rospy.Publisher('Data_for_errors', Float64MultiArray, queue_size=10)
pub_track = rospy.Publisher('tracking', Float64MultiArray, queue_size=10)
pub_jac = rospy.Publisher('jacobian', Float64MultiArray, queue_size=10)

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
               [p/2, 0, 0, -p/2],
               [0, 0, L6, 0]])
  

# Trasformation matrices given DH table. T0,1 T1,2 ... T7,e
T_dh = t.DH_to_T(DH)

# Jacobian at time k minus 1
Jkmin1 = np.zeros((6,7))

# type of joints, 1 = revolute, 0 = prismatic.
info = np.array([1, 1, 1, 1, 1, 1, 1])

# Need this variable to store initial rotation matrix, because it's equal to
# rotation matrix from 0 to inertial frame.
ini_bax = 0
ini_dot = 0 # needed to differentiate from initial condition to computed qdots.

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
q = np.zeros(7)
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

def anglesCompensate(angles):
    """!
    Function used to filter unwanted minimal incoming data fluctuations,
    due to noise as well as human operator shake
    @param orientation with respect to X, Y, Z axes
    @returns returns a filtered version (if necessary) of the input angles
    """
    dx = 0.0174  # min angle perceived [rad], about 1 [deg]
    # reduce sensibility of sensor: minimum precision is dx
    compensatedAngles = [0, 0, 0]

    for i in range(0, 3):  # i = 0, 1, 2
        if abs(angles[i] / dx) >= 1:
            compensatedAngles[i] = angles[i]

    return compensatedAngles

def eulerAnglesToRotationMatrix(angles):  # angles [roll, pitch, yaw]
    """!
    Function that transforms euler angle coordinates into the rotation matrix
    @param angles euler angles, i.e. orientation with respect to X, Y, Z axes
    @returns rotation matrix
    """

    R_x = np.array([[1,         0,                  0],
                    [0,         math.cos(angles[0]),   math.sin(angles[0])],
                    [0,         -math.sin(angles[0]),  math.cos(angles[0])]
                    ])

    R_y = np.array([[math.cos(angles[1]),    0,      -math.sin(angles[1])],
                    [0,                      1,      0],
                    [math.sin(angles[1]),    0,      math.cos(angles[1])]
                    ])

    R_z = np.array([[math.cos(angles[2]),      math.sin(angles[2]),     0],
                    [-math.sin(angles[2]),     math.cos(angles[2]),     0],
                    [0,                        0,                       1]
                    ])

    R = np.dot(R_x, np.dot(R_y, R_z))

    return R

def main_callback():
    """!
    Publish to error node and inverse kinematics nodes the data, in particular the
    jacobian matrix, the tracking vectors and the data needed to compute the errors.
    """

    ##########################################################################
    # Entry point when receveing all the data, from smart, baxter and weighter.
    global x_0e_kmin1B, x_0e_k, v_0e_kmin1B, v_0e_k, a_0e, omega_0e, R0e_kmin1, R0e_k, Jkmin1

    ################################
    # Send Jacobian matrix
    ################################
    J = util.init_float64_multiarray(6*7, 1)
    J.data = Jkmin1.reshape(6*7, 1)
    pub_jac.publish(J)
    
    ################################
    # Send to Error block the data
    ################################

    # Send R0e_k, R0e_kmin1; x_0e_k, x_0e_kmin1B; v_0e_k, v_0e_kmin1B;
    Rg_Re_xg_xe_vg_ve = np.array([R0e_k[0][0], R0e_k[0][1], R0e_k[0][2], R0e_k[1][0], R0e_k[1][1], R0e_k[1][2], R0e_k[2][0], R0e_k[2][1], R0e_k[2][2], R0e_kmin1[0][0], R0e_kmin1[0][1], R0e_kmin1[0][2], R0e_kmin1[1][0], R0e_kmin1[1][1], R0e_kmin1[1][2], R0e_kmin1[2][0], R0e_kmin1[2][1], R0e_kmin1[2][2], x_0e_k[0][0], x_0e_k[1][0], x_0e_k[2][0], x_0e_kmin1B[0][0], x_0e_kmin1B[1][0], x_0e_kmin1B[2][0], v_0e_k[0][0], v_0e_k[1][0], v_0e_k[2][0], v_0e_kmin1B[0][0], v_0e_kmin1B[1][0], v_0e_kmin1B[2][0]], dtype=np.float32)
    R_x_v = util.init_float64_multiarray(30, 1)
    R_x_v.data = Rg_Re_xg_xe_vg_ve
    pub_err.publish(R_x_v)

    ################################
    # Send to Math block the data.
    ################################

    # send v_0e_k, omega_0e, a_0e
    vg_omega_a = np.array([v_0e_k[0][0], v_0e_k[1][0], v_0e_k[2][0], omega_0e[0][0], omega_0e[1][0], omega_0e[2][0], a_0e[0][0], a_0e[1][0], a_0e[2][0]], dtype=np.float32)
    v_w_a = util.init_float64_multiarray(9, 1)
    v_w_a.data = vg_omega_a
    pub_track.publish(v_w_a)

    #print("Published")

def baxter_callback(data):
    """!
    Computes the configuration of baxter's arm whenever the data are available, then extracts the rotation
    matrix from 0 to e.e and the position of the e.e. with respect to 0. It also computes the jacobian matrix.
    In case all the other callbacks have been called then it computes the velocity of the
    e.e. with respect to 0 and the end it calls the main_callback.
    @param data: coming from baxter node which provides a JointState message.
    """

    start = time.time()

    global ini_bax, q, R0e_kmin1, R0inert, Jkmin1, x_0e_kmin1B, x_0e_kmin1, v_0e_kmin1B, key_bax, key_dot, key_smart
    
    ####################################################
    # Read from publisher of v-rep the q configuration.
    ####################################################
    if ini_bax != 0:
        # configuration at time kmin1
        q = np.array(data.position)
        
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
    for i in range(3):
        x_0e_kmin1B[i] = T0e_kmin1[i][3]

    # end effector orientation of baxter at time k. At time 0 i have the
    # orientation of zero with respect of inertial frame also.
    for i in range(3):
        for k in range(3):
            R0e_kmin1[i][k] = T0e_kmin1[i][k]

    if ini_bax == 0:
        R0inert = R0e_kmin1 # Constant in time.
        #print(R0e_kmin1)
        x_0e_kmin1 = x_0e_kmin1B # Initially they are equal
        ini_bax = ini_bax + 1
    
    key_bax = key_bax + 1
    
    if (key_bax >= 1 and key_dot >= 1):
        x_dot = np.dot(Jkmin1, q_dot)
        for i in range(3):
            v_0e_kmin1B[i][0] = x_dot[i][0]

        if(key_smart >= 1):
            key_bax = 0
            key_dot = 0
            key_smart = 0

            main_callback()

    end = time.time()
    #print("Bax Frequency: " + str(1/(end-start)))

def dot_callback(data):
    """!
    Reads the q_dots provided by the weighter. In case all the other callbacks have been called then it computes the velocity of the
    e.e. with respect to 0 and the end it calls the main_callback.
    @param data: coming from weighter node which provides a JointState message.
    """
    global ini_dot, q_dot, Jkmin1, v_0e_kmin1B, key_bax, key_dot, key_smart

    ###################################################################################
    # Read from topic to get the generated q_dot, needed to compute the velocity of
    # baxter's arm.
    ###################################################################################
    if ini_dot != 0:
        q_dot = np.transpose(np.array([data.velocity]))

    key_dot = key_dot + 1

    if ini_dot == 0:
        ini_dot = ini_dot + 1
    
    if (key_bax >= 1 and key_dot >= 1):
        x_dot = np.dot(Jkmin1, q_dot)
        for i in range(3):
            v_0e_kmin1B[i][0] = x_dot[i][0]    
        if(key_smart >= 1):
            key_bax = 0
            key_dot = 0
            key_smart = 0

            main_callback()    


def smart_callback(data):
    """!
    Computes the linear acceleration, angular velocity projected on 0 given the data. In case all the other callbacks have been called then it computes the velocity of the
    e.e. with respect to 0 and the end it calls the main_callback.
    @param data: coming from smartphone which provides a Imu() message.
    """

    start = time.time()

    global omega_imu_inert, a_imu_inertial, Rimu_inert_k, R0e_k, x_0e_kmin1, x_0e_k, v_0e_kmin1, v_0e_k, a_0e, omega_0e, key_bax, key_dot, key_smart
    #####################################################################################
    # Read from topic, get Rimu,inertial; angular velocity and linear acceleration
    # of imu with respect to inertial frame, all expressed in imu frame at time kplus1.
    #####################################################################################

    # Get orientation
    orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    
    # transform quaternion to euler angles
    tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

    angles = anglesCompensate(tempAngles)
    
    Rimu_inert_k = eulerAnglesToRotationMatrix(angles)
    #print(Rimu_inert_k)

    Rinert_imu_k = np.transpose(Rimu_inert_k)
    
    # angular velocity of imu (end effector) w.r.t. inertial frame projected on imu frame
    omega_imu_inert[0][0] = data.angular_velocity.x
    omega_imu_inert[1][0] = data.angular_velocity.y
    omega_imu_inert[2][0] = data.angular_velocity.z
    #print(omega_imu_inert)
    
    # linear acceleration of imu (end effector) w.r.t. inertial frame projected on imu frame
    a_imu_inert[0][0] = data.linear_acceleration.x
    a_imu_inert[1][0] = data.linear_acceleration.y
    a_imu_inert[2][0] = data.linear_acceleration.z
    #print(a_imu_inert)

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

    key_smart = key_smart + 1
    if (key_bax >= 1 and key_dot >= 1 and key_smart >= 1):
        key_bax = 0
        key_dot = 0
        key_smart = 0
        main_callback()
    
    # Update this vectors to compute integration at next steps.

    x_0e_kmin1 = x_0e_k
    v_0e_kmin1 = v_0e_k

    end = time.time()
   # print("Smart Frequency: " + str(1/(end-start)))

        
def subs():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subs', anonymous=True)

    # Receive data from smartphone and baxter.
    rospy.Subscriber("smartphone", Imu, smart_callback)
    rospy.Subscriber("logtopic", JointState, baxter_callback)
    rospy.Subscriber("cmdtopic", JointState, dot_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()  

  
if __name__ == '__main__':
    baxter_callback(0) # to set the initial conditions.
    dot_callback(0) # to set the initial conditions.
    subs()
