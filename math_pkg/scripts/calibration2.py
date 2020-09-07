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
# this orientation has been computed offline on Baxter arm model.

# q = [0, 0, .., 0]
R0e =  np.array([[ 0,  0,  1],
                 [ 0,  1,  0],
                 [-1,  0,  0]])

# determines the starting phase.
start = 0
    
    
def imu_ee_calibration(data):
    """!
    Computes the orientation between 0 and global frame using an
    initial configuration.
    @param data: inertial data coming from smartphone. The focus is on the
    orientation info given by a quaternion.
    """

    if start == 1:
        
        orient = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        Ttemp = tf.transformations.quaternion_matrix((orient[0], orient[1], orient[2], orient[3]))
        Rimu_global = Ttemp[:3, :3]

        R0_global = np.dot(R0e, Rimu_global)

        R = util.init_float64_multiarray(9, 1)
        R0_global = R0_global.reshape(9, 1)
        R.data = R0_global
        
        start == 0
        
        rospy.logerr("Setup done!")

        pub_rot_matrices.publish(R)


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
