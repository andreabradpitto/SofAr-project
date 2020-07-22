#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from smartwatch.srv import Smartwatch, SmartwatchResponse
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

#global variables
dx = 0.0174  # min angle perceived [rad], about 1 [deg]
# dx = 0.087 # min angle perceived [rad], about 5 [deg]
angles = [0, 0, 0]


def eulerAnglesToRotationMatrix(angles):  # angles [roll, pitch, yaw]

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


def anglesCompensate(angles):
    # reduce sensibility of sensor: minimum precision is dx
    compensatedAngles = [0, 0, 0]

    for i in range(0, 3):  # i = 0, 1, 2
        if abs(angles[i] / dx) >= 1:
            compensatedAngles[i] = angles[i]

    return compensatedAngles


def callback(data):
    global angles

    # get data
    orientation = [data.orientation.x, data.orientation.y,
                   data.orientation.z, data.orientation.w]

    # transform quaternion to euler angles
    angles = tf.transformations.euler_from_quaternion(orientation, "sxyz")

    angles = anglesCompensate(angles)


def serv_callback():

    serv_rot_matrix = Float64MultiArray()
    serv_rot_matrix.layout.dim.append(MultiArrayDimension())
    serv_rot_matrix.layout.dim.append(MultiArrayDimension())
    serv_rot_matrix.layout.dim[0].label = "rows"
    serv_rot_matrix.layout.dim[0].size = 3
    serv_rot_matrix.layout.dim[1].label = "columns"
    serv_rot_matrix.layout.dim[1].size = 3
    serv_rot_matrix.data = eulerAnglesToRotationMatrix(angles)

    return SmartwatchResponse(serv_rot_matrix)


def smartwatch_server_setup():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'smartwatch_service' node, so that multiple nodes can
    # run simultaneously without any issue.
    rospy.init_node('smartwatch_service', anonymous=True)  # initialize node

    # This declares that your node subscribes to the android/imu topic,
    # which is of type sensor_msgs.msg.Imu. When new data is received,
    # callback is invoked with that data as argument.
    rospy.Subscriber("android/imu", Imu, callback)

    # activate service
    serv = rospy.Service('smartwatch_serv', Smartwatch, serv_callback)
    print ("\nService server setup and running")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    smartwatch_server_setup()
