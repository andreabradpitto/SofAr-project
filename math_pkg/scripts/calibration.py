#!/usr/bin/env python

import rospy
import numpy as np
import tf
import utilities as util
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

# GLOBAL VARIABLES

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
prev_orient = np.zeros(4)
prev_acc = np.zeros(4)

# determines the starting phase.
start = 1

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
    return numpy.real(eigenVectors[:,0].A1)

def get_orientation_eeimu_0global():
    """!
    Algorithm presented in:
    "An Iterative Approach to the Hand-Eye and Base-World Calibration"
    by Robert Lee Hirsh, Guilherme Nelson DeSouza, Avinash C. Kak.

    Computes X: rotation from e.e. to imu frame.
             Y: rotation from base to world frame.
    Sends them through a publisher.
    """

    # initial guess
    Ym = np.identity(3);

    # more steps implies more accuracy.
    steps = 100;

    for i in range(steps):

      X1 = np.dot(np.dot(Rigs(0), np.transpose(Ym)), R0e)
      X2 = np.dot(np.dot(Rigs(1), np.transpose(Ym)), R0e1)
      X3 = np.dot(np.dot(Rigs(2), np.transpose(Ym)), R0e2)

      r1 = R.from_matrix(X1)
      r2 = R.from_matrix(X2)
      r3 = R.from_matrix(X3)

      q1 = r1.as_quat()
      q2 = r2.as_quat()
      q3 = r3.as_quat()

      Q = np.array([q1],
                   [q2],
                   [q3]])
      
      avgq = averageQuaternions(Q)

      Xm = 

def imu_ee_calibration(data):
    """!
    Computes the orientation between the e.e. and the imu using
    three configurations.
    @param data: inertial data coming from smartphone. The focus is on the
    orientation info given by a quaternion.
    """
    global start, Rigs, config, prev_orient, prev_acc

    if calib_ok == 0:

      if config == 3:
        get_orientation_eeimu_0global()

      # get orientation
      orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
      
      # get acceleration
      acc = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
      
      if start == 0:

        if (np.linalg.norm(acc-prev_acc) < 0.01):
          # transform quaternion to euler angles
          tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

          angles = util.anglesCompensate(tempAngles)
            
          Rimu_global = util.eulerAnglesToRotationMatrix(angles)

          Rigs.append(Rimu_global)

          prev_orient = orientation
          prev_acc = acc

          config = config + 1

      elif:
        # transform quaternion to euler angles
        tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

        angles = util.anglesCompensate(tempAngles)
          
        Rimu_global = util.eulerAnglesToRotationMatrix(angles)

        Rigs.append(Rimu_global)

        prev_orient = orientation
        prev_acc = acc

        config = config + 1
        
        start = 0


def calibrate():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calibrate', anonymous=True)

    rospy.subscriber("smartphone", Imu, imu_ee_calibration)

    rospy.spin()


if __name__ == '__main__':

    calibrate()
