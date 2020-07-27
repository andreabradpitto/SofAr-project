#!/usr/bin/env python

"""
Documentation for the gravity removal tool
This is the main piece of code of the subproject: it handles the problem of the removal of Earth gravity, while also 
"""

import rospy
import numpy as np
import tf
import math
import csv
import os
from rotationMatrix import eulerAnglesToRotationMatrix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from datetime import datetime

script_dir = os.path.dirname(__file__)  # absolute directory the script is in
rel_path1 = "output/lin_acc.csv"
abs_file_path1 = os.path.join(script_dir, rel_path1)
rel_path2 = "output/orientation.csv"
abs_file_path2 = os.path.join(script_dir, rel_path2)
rel_path3 = "output/angVel.csv"
abs_file_path3 = os.path.join(script_dir, rel_path3)

# flags used to turn on features
# flagWriteData = 1 means to store simple data received from imu, after gravity removal
flagWriteData = 1
index = 1  # used to store data for offline analysis

if flagWriteData == 1:
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


def removeGravity(lin_acc, Rot_m):
    """!
    This function is the one that actually carries out the gravity removal
    @param lin_acc linear acceleration data incoming from the accelerometer
    @param Rot_m rotation matrix computed with eulerAnglesToRotationMatrix()
    @returns the output is the same linear acceleration vector provided, but without the influence of the gravity
    """
    g = [0, 0, 9.81]

    # rotate g vector in the current frame
    g_frame_i = np.dot(Rot_m, g)
    g = [0, 0, 9.81]
    g_removed = [0, 0, 0]  # define linear acceleration without gravity

    for i in range(0, 3):
        if lin_acc[i] >= 0:
            g_removed[i] = lin_acc[i] - abs(g_frame_i[i])
        if lin_acc[i] < 0:
            g_removed[i] = lin_acc[i] + abs(g_frame_i[i])

    return g_removed


def storeDataInFiles(fileName, modality, data):
    """!
    Function whose aim to simply to paste incoming accelerometer data into .csv files
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
    @param data incoming from the imu sensor
    """
    global index, header

    # get data
    orientation = [data.orientation.x, data.orientation.y,
                   data.orientation.z, data.orientation.w]

    # transform quaternion to euler angles
    tempAngles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

    angular_velocity = [data.angular_velocity.x,
                        data.angular_velocity.y, data.angular_velocity.z]
    linear_acceleration = [data.linear_acceleration.x,
                           data.linear_acceleration.y, data.linear_acceleration.z]

    angles = anglesCompensate(tempAngles)

    rot_matrix = eulerAnglesToRotationMatrix(angles)

    lin_acc_no_g = removeGravity(linear_acceleration, rot_matrix)

    if flagWriteData == 1:
        # store data into .csv files in order to analyse them offline
        storeDataInFiles(abs_file_path1, 'a', lin_acc_no_g)

        angles_in_deg = [(angles[0]*180) / math.pi,
                         (angles[1]*180) / math.pi, (angles[2]*180) / math.pi]
        storeDataInFiles(abs_file_path2, 'a', angles_in_deg)
        storeDataInFiles(abs_file_path3, 'a', angular_velocity)

        index += 1  # update index

    # modify Imu message to be sent
    data.linear_acceleration.x = lin_acc_no_g[0]
    data.linear_acceleration.y = lin_acc_no_g[1]
    data.linear_acceleration.z = lin_acc_no_g[2]

    # publish message
    try:
        talker(data)
    except rospy.ROSInterruptException:
        pass


def listener():
    """!
    The listener is used to instantiate the homonymous node and to subscribe to the android/imu topic,
    from which it receives incoming data of smarthpone/smartwatch accelerometers
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
    rospy.Subscriber("android/imu", Imu, callback)

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
    Main function
    """

    listener()
