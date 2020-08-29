#!/usr/bin/env python

import rospy
import numpy as np
import tf
import utilities as util
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Int8

# GLOBAL VARIABLES

pub_rot_matrices = rospy.Publisher('rot_matrices', Float64MultiArray, queue_size=10)
                                   
# Since the calibration involves an human arm and not a robot manipulator,
# this orientations have been computed offline on Baxter arm model.

# q = [0, 0, .., 0]
R0e =  np.array([[ 0,  0,  1],
                 [ 0,  1,  0],
                 [-1,  0,  0]])
# q = [pi/2, 0, .., 0]. rotation of 90 degrees along z axis w.r.t. zero.
R0e1 = np.array([[ 0, -1,  0],
                 [ 0,  0,  1],
                 [-1,  0,  0]])
# q = [pi/2, 0, 0, -pi/2, 0, 0, 0]. rotation along y axis w.r.t. 4th joint.
R0e2 = np.array([[ 0, -1,  0],
                 [ 1,  0,  0],
                 [ 0,  0,  1]])

# list that will contain the three rotation matrices from imu frame to global frame.
Rigs = []

# variable that will be set to 1 if the calibration phase has finished.
calib_ok = 0

# counts the current configuration number. Every time it computes a new
# orientation matrix, config is incremented and when equals 3 the number
# of configuration needed is reached.
config = 0

# understand if the configuration has changed.
prev_R = np.zeros((3,3))
prev_angvel = np.zeros(3)

# determines the starting phase.
start = 1

# for initial condition.
ini = 1

# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros((4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0])

def get_quaternion(X):
    """!
    Computes the quaternion of the input rotation matrix.
    @param X: orientation matrix input.
    @return q: quaternion representation of X.
    """
    
    # need this two vectors to obtain an homogeneous matrix with the given orientation.
    t = np.zeros((3,1))
    s = np.array([[0, 0, 0, 1]])

    # computing the homogeneous matrices because quaternion_from_matrix takes in input
    # a 4x4 homogeneous matrix.
    X = np.concatenate((X, t), axis = 1)
    X = np.concatenate((X, s), axis = 0)

    # computing the quaternions and rearrenging them in [w,x,y,z] format.
    q = tf.transformations.quaternion_from_matrix(X)
    q = np.array([q[3], q[0], q[1], q[2]])

    return q

def Q_matrix(X1, X2, X3):
    """!
    Computes the quaternions of the input orientation matrices.
    @param X1, X2, X3: orientation matrices estimates.
    @return Q: matrix of the quaternions.
    """

    # get quaternion in format [w,x,y,z].
    q1 = get_quaternion(X1)
    q2 = get_quaternion(X2)
    q3 = get_quaternion(X3)
      
    Q = np.array([[q1],
                  [q2],
                  [q3]])
    return Q
    
    
def get_orientation_eeimu_0global():
    """!
    Algorithm presented in:
    "An Iterative Approach to the Hand-Eye and Base-World Calibration"
    by Robert Lee Hirsh, Guilherme Nelson DeSouza, Avinash C. Kak.

    Computes X: rotation from e.e. to imu frame.
             Y: rotation from base to world frame.
    Sends them through a publisher.
    """

    global calib_ok
    
    # initial guess
    Ym = np.identity(3);

    # more steps implies more accuracy.
    steps = 100;

    for i in range(steps):

      X1 = np.dot(np.dot(Rigs[0], np.transpose(Ym)), R0e)
      X2 = np.dot(np.dot(Rigs[1], np.transpose(Ym)), R0e1)
      X3 = np.dot(np.dot(Rigs[2], np.transpose(Ym)), R0e2)

      Q = Q_matrix(X1, X2, X3)
      
      avgq = averageQuaternions(Q)

      Xm = tf.transformations.quaternion_matrix((avgq[1], avgq[2], avgq[3], avgq[0]))
      Xm = Xm[:3, :3]

      Y1 = np.dot(np.dot(R0e, Xm.transpose()), Rigs[0])
      Y2 = np.dot(np.dot(R0e1, Xm.transpose()), Rigs[1])
      Y3 = np.dot(np.dot(R0e2, Xm.transpose()), Rigs[2])

      Q = Q_matrix(Y1, Y2, Y3)
      
      avgq = averageQuaternions(Q)

      Ym = tf.transformations.quaternion_matrix((avgq[1], avgq[2], avgq[3], avgq[0]))
      Ym = Ym[:3, :3]

    # Re,imu
    X = Xm.transpose()
    # R0,g
    Y = Ym

    calib_ok = 1

    # manipulate the 2 orientation matrices to send them via a publisher.
    X_Y = util.init_float64_multiarray(9*2, 1)
    X = X.reshape(9,1)
    Y = Y.reshape(9,1)
    x_y = np.concatenate((X,Y), axis=0)
    X_Y.data = x_y

    pub_rot_matrices.publish(X_Y)
    
    
def imu_ee_calibration(data):
    """!
    Computes the orientation between the e.e. and the imu using
    three configurations.
    @param data: inertial data coming from smartphone. The focus is on the
    orientation info given by a quaternion.
    """
    global Rigs, config, prev_R, prev_angvel

    if start == 1:
        
        if calib_ok == 0:
            
            if config == 3:
                get_orientation_eeimu_0global()
            else:
                # get orientation
                orient = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
                # get acceleration
                angvel = [data.angular.x, data.angular_velocity.y, data.angular_velocity.z]

                Ttemp = tf.transformations.quaternion_matrix((orient[0], orient[1], orient[2], orient[3]))
                Rimu_global = Ttemp[:3, :3]

                A1 = np.dot(prev_R, Rimu_global.transpose())
                q = get_quaternion(A1)
                angle = q[0]*180/(np.pi)

                if ini == 0:
                    if ((np.linalg.norm(angvel-prev_angvel) < 0.01) and (abs(angle-90) < 0.01)):
                        Rigs.append(Rimu_global)

                        prev_R = Rimu_global
                        prev_angvel = angvel

                        config = config + 1
                else:
                    Rigs.append(Rimu_global)
                    
                    prev_R = Rimu_global
                    prev_angvel = angvel

                    config = config + 1
                    ini = 0


def simulate_callback(data):
    """!
    Waits for the start from handle simulation topic.
    @param data: integer used to understand which part of the simulation is on.
    """

    global start

    if data.data == 3:
        start = 1

def calibrate():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calibrate', anonymous=True)

    rospy.Subscriber("smartphone", Imu, imu_ee_calibration)
    rospy.Subscriber("handleSimulation", Int8, simulate_callback)

    rospy.spin()


if __name__ == '__main__':

    calibrate()
