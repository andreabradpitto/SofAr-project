#!/usr/bin/env python

"""
Documentation for the clipping tool.
This piece of code receives data from the imu sensor and removes noise from
the incoming linear acceleration through a clipping algorithm.
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
import pandas as pd

script_dir = os.path.dirname(__file__)  # absolute directory the script is in
rel_path1 = "../output/lin_acc.csv"
abs_file_path1 = os.path.join(script_dir, rel_path1)
rel_path2 = "../output/orientation.csv"
abs_file_path2 = os.path.join(script_dir, rel_path2)
rel_path3 = "../output/angVel.csv"
abs_file_path3 = os.path.join(script_dir, rel_path3)

# absolute directory the script is in, in case of calibration data
rel_path_gravity = "../calibration/gravity.csv"
abs_file_gravity = os.path.join(script_dir, rel_path_gravity)

# flagWriteData = 1 means to store simple data received from imu, after gravity removal
flagWriteData = 1

index = 1  # used to store data for offline analysis

# initializations
delta = 0.05  # clipping threshold - value achieved empirically
lin_acc_no_g = [0, 0, 0]
angular_velocity = [0, 0, 0]
orientation = [0, 0, 0, 0]
g = [0 ,0 ,0]
max_lin_acc = [0, 0, 0]
safety_coeff = 1.1
counter = 0 #current number of sensor data received
maxIteration = 50 #number of iterations needed to estimate the current gravity vector

def dataFileInitializer():
    """!
    Function used initialize the files in which sensor data will be stored; it will generate three files in the 'output' folder
    """
    # initialize files to store data
    with open(abs_file_path1, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["", "X", "Y", "Z"])

    with open(abs_file_path2, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["", "X", "Y", "Z"])

    with open(abs_file_path3, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["", "X", "Y", "Z"])

def lin_acc_compensate(lin_acc_no_g, threshold):
    """!
    Function used to filter unwanted minimal incoming data fluctuations,
    due to noise as well as human operator shake
    @param lin_acc_no_g linear acceleration along X, Y, Z axes
    @param threshold a threshold computed in the calibration phase to remove unwanted vibrations  -----------------------------
    @returns returns a filtered version (if necessary) of the input linear acceleration without gravity
    """
    # initialization
    res = [0, 0, 0]

    result = math.sqrt(
        pow(lin_acc_no_g[0], 2) + pow(lin_acc_no_g[1], 2) + pow(lin_acc_no_g[2], 2))

    if result >= threshold:
        res = lin_acc_no_g

    return res

def storeDataInFiles(fileName, modality, data):
    """!
    Function which stores incoming data into .csv files
    @params fileName name of the generated file
    @params modality parameter requested by the open() function: 'a' stands for 'open for appending at the end of the file without truncating it'
    @params data data to be inserted in the .csv file (orientation/linear acceleration/angular velocity)
    """
    with open(fileName, modality) as file:
        writer = csv.writer(file)
        writer.writerow([index, data[0], data[1], data[2]])

def callback(data):
    """!
    This is the callback function: it is invoked every time there is incoming data and
    has the duty of calling all the previously mentioned functions as well as eulerAnglesToRotationMatrix().
    After these calls, it finally invokes the talker(). In order to achieve all of this,
    it translates incoming quaternions into euler angles beforehand
    @param data data incoming from the imu sensor
    """
    global index, lin_acc_no_g, angular_velocity, orientation, counter, max_lin_acc, delta

    # get data
    orientation = [data.orientation.x, data.orientation.y,
                   data.orientation.z, data.orientation.w]

    # transform quaternion to euler angles
    tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

    rot_matrix = eulerAnglesToRotationMatrix(tempAngles)

    angular_velocity = [data.angular_velocity.x,
                        data.angular_velocity.y, data.angular_velocity.z]
    linear_acceleration = [data.linear_acceleration.x,
                           data.linear_acceleration.y, data.linear_acceleration.z]

    lin_acc_no_g = removeGravity(linear_acceleration, rot_matrix, g)

    if counter < maxIteration:
        #compute root of gravity
        g_root = math.sqrt(
            pow(lin_acc_no_g[0], 2) + pow(lin_acc_no_g[1], 2) + pow(lin_acc_no_g[2], 2))

        max_root = math.sqrt(
            pow(max_lin_acc[0], 2) + pow(max_lin_acc[1], 2) + pow(max_lin_acc[2], 2))

        if g_root > max_root:  # if true, the 'maximum linear acceleration vector' has to be updated
            max_lin_acc = lin_acc_no_g

    elif counter == maxIteration:
        #increase 'max_lin_acc' for safety reason
        max_lin_acc = [safety_coeff * i for i in max_lin_acc]
        max_root = math.sqrt(
            pow(max_lin_acc[0], 2) + pow(max_lin_acc[1], 2) + pow(max_lin_acc[2], 2))
        
        delta_root = math.sqrt(3 * (pow(delta, 2)))

        print("Calibration terminated")
        # if true, the 'maximum linear acceleration vector' is very small (which is good): lower delta accordingly
        if max_root > delta_root:
            delta = max_root

    else:
        #reduce noise on incoming linear acceleration data 
        lin_acc_no_g = lin_acc_compensate(lin_acc_no_g, delta)

        if flagWriteData == 1:
            # store data into .csv files in order to analyse them offline
            storeDataInFiles(abs_file_path1, 'a', lin_acc_no_g)

            angles_in_deg = [(tempAngles[0]*180) / math.pi,
                             (tempAngles[1]*180) / math.pi, (tempAngles[2]*180) / math.pi]
            storeDataInFiles(abs_file_path2, 'a', angles_in_deg)
            storeDataInFiles(abs_file_path3, 'a', angular_velocity)

            index += 1  # update index

        # modify Imu message to be sent
        data.orientation.x = orientation[0]
        data.orientation.y = orientation[1]
        data.orientation.z = orientation[2]

        data.angular_velocity.x = angular_velocity[0]
        data.angular_velocity.y = angular_velocity[1]
        data.angular_velocity.z = angular_velocity[2]
        data.linear_acceleration.x = lin_acc_no_g[0]
        data.linear_acceleration.y = lin_acc_no_g[1]
        data.linear_acceleration.z = lin_acc_no_g[2]

        # publish message
        try:
            talker(data)
        except rospy.ROSInterruptException:
            pass

    counter +=1

def listener():
    """!
    The listener is used to instantiate the homonymous node and to subscribe to the android/imu_corected topic,
    from which it receives incoming data of smarthpone accelerometers
    """

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous = True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # This declares that your node subscribes to the android/imu topic,
    # which is of type sensor_msgs.msg.Imu. When new data is received,
    # callback is invoked with that data as argument.
    rospy.Subscriber("/android/imu_corrected", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker(msg):
    """!
    The talker is used to publish the refined data on the so called 'smartphone' topic
    """
    pub = rospy.Publisher('smartphone', Imu, queue_size=10)
    pub.publish(msg)


if __name__ == '__main__':
    """!
    Main function: initialize output files with dataFileInitializer() and call listener()
    """

    #read gravity from calibration files
    temp = pd.read_csv(abs_file_gravity, names=['X', 'Y', 'Z'], header=0, decimal=',')
    g[0] = float(temp.X[1])
    g[1] = float(temp.Y[1])
    g[2] = float(temp.Z[1])
    
    dataFileInitializer()

    listener()