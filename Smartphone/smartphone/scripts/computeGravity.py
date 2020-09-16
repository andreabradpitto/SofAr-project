#!/usr/bin/env python

"""
Documentation for the compute gravity tool.
This code analyzes the first 'maxIteration' samples and computes an estimate of the gravity vector.
"""

import rospy
import numpy as np
import tf
import math
import csv
import os
from rotationMatrix import eulerAnglesToRotationMatrix
from removeGravity import removeGravity
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from datetime import datetime
import time
import sys

script_dir = os.path.dirname(__file__)  # absolute directory the script is in
rel_path_gravity = "../calibration/gravity.csv"
abs_file_path_gravity = os.path.join(script_dir, rel_path_gravity)

counter = 0 # current number of sensor data received
sum_x = 0.0 # sum of the first 'maxIteration' samples of the linear acceleration
sum_y = 0.0
sum_z = 0.0

delta = 0.05 # error on the sensor, learned by experience
safety_coeff = 1.1 
max_lin_acc = [0, 0, 0]
lin_acc_no_g = [0, 0, 0]
orientation = [0, 0, 0, 0]
g = [0, 0, 0] # ravity vector initialization
maxIteration = 50 # number of iterations needed to estimate the current gravity vector

def dataFileInitializer():
    """!
    Function used initialize the files in which data will be stored; it will be stored in the calibration folder
    """
    # initialize files to store data
    with open(abs_file_path_gravity, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["", "X", "Y", "Z"])

def callback(data):
    """!
    This is the callback function: it is invoked every time there is incoming data and
    has the duty of calling all the previously mentioned functions as well as eulerAnglesToRotationMatrix().
    After these calls, it finally invokes the talker(). In order to achieve all of this,
    it translates incoming quaternions into euler angles beforehand
    @param data data incoming from the imu sensor
    """
    global counter, sum_x, sum_y, sum_z, delta, lin_acc_no_g, orientation, max_lin_acc, g

    # get data
    orientation = [data.orientation.x, data.orientation.y,
                   data.orientation.z, data.orientation.w]

    # transform quaternion to euler angles
    tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

    #compute the rotation matrix
    rot_matrix = eulerAnglesToRotationMatrix(tempAngles)
    
    #define the linear acceleration vector
    linear_acceleration = [data.linear_acceleration.x,
                           data.linear_acceleration.y, data.linear_acceleration.z]

    # calibration phase for gravity removal
    if counter < maxIteration:

        g_frame_i = np.dot(rot_matrix.transpose(), linear_acceleration)

        sum_x += g_frame_i[0]
        sum_y += g_frame_i[1]
        sum_z += g_frame_i[2]

    elif counter == maxIteration:
        #estimate gravity vector
    	g = [sum_x/maxIteration, sum_y/maxIteration, sum_z/maxIteration]
        
        #store gravity vector in .csv file
        with open(abs_file_path_gravity, 'a') as file:
            writer = csv.writer(file)
            writer.writerow([1, g[0], g[1], g[2]])
            print("File ok")
            rospy.signal_shutdown("")

    counter += 1  #update


def listener():
    """!
    The listener is used to instantiate the homonymous node and to subscribe to the android/imu_corrected topic,
    from which it receives incoming data of smarthpone accelerometers
    """

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous = True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('computeGravity', anonymous=True)

    # This declares that your node subscribes to the android/imu topic,
    # which is of type sensor_msgs.msg.Imu. When new data is received,
    # callback is invoked with that data as argument.
    rospy.Subscriber("/android/imu_corrected", Imu, callback)
    
    rospy.spin() #simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    """!
    Main function: initialize output files with dataFileInitializer() and call listener()
    """

    dataFileInitializer()

    listener()
    
