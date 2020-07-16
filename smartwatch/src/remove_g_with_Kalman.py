#!/usr/bin/env python2
import rospy
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import csv
from scipy.linalg import block_diag
from KalmanFilter import Kalman 

#flags used to turn on features
flagWriteData = 1 # flagWriteData = 1 means to store simple data received from imu, after gravity removal

#global variables
g = [ 0, 0, 9.81]
dx = 0.0174 # min angle perceived [rad], about 1 [deg]
#dx = 0.087 # min angle perceived [rad], about 5 [deg]
index = 1 #used to store data for offline analysis

# initialize files to store data
with open('lin_acc.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('orientation.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('angVel.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

def readDataFromSensors(data):

	# get data from sensor
	orientation = [data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w]
	angular_velocity = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
	linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

	# transform quaternion to euler angles
	angles = np.array(tf.transformations.euler_from_quaternion(orientation,"sxyz"))

	# create measurements column vector
	measurements = np.concatenate((angles, angular_velocity), axis = 0)
	measurements = np.concatenate((measurements, linear_acceleration), axis = 0)
	measurements = np.array([measurements]).T

	return measurements

def eulerAnglesToRotationMatrix(angles): #angles [yaw, pitch, roll]
    
    R_z = np.array([[1,         0,                  0                     ],
                    [0,         math.cos(angles[2]),   math.sin(angles[2]) ],
                    [0,         -math.sin(angles[2]),  math.cos(angles[2]) ]
                    ])
                    
    R_y = np.array([[math.cos(angles[1]),    0,      -math.sin(angles[1])  ],
                    [0,                      1,      0                    ],
                    [math.sin(angles[1]),    0,      math.cos(angles[1])  ]
                    ])
                
    R_x = np.array([[math.cos(angles[0]),      math.sin(angles[0]),     0],
                    [-math.sin(angles[0]),     math.cos(angles[0]),     0],
                    [0,                        0,                       1]
                    ])
                                 
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def anglesCompensate(angles):
	# reduce sensibility of sensor: minimum precision is dx

	compensatedAngles = [0, 0, 0]

	for i in range (0,3): # i = 0, 1, 2
		if abs(angles[i]/ dx) >= 1 : 
			compensatedAngles[i] = angles[i]
		
	return compensatedAngles
	
def removeGravity(lin_acc, Rot_m):
	# rotate g vector in the current frame 
	g_frame_i = np.dot(Rot_m, g)
	g_removed = [0, 0, 0] # define linear acceleration without gravity

	for i in range(0,3):
		if lin_acc[i] >= 0:
			g_removed[i] = lin_acc[i] - abs(g_frame_i[i])
		if lin_acc[i] < 0:
			g_removed[i] = lin_acc[i] + abs(g_frame_i[i])
			
	return g_removed

def storeDataInFiles(fileName, modality, data):
	with open(fileName,modality) as file:
		writer = csv.writer(file)
		writer.writerow([ index,data[0], data[1], data[2] ])

def callback(data):

	global index

	measurements = readDataFromSensors(data)

	# Kalman initialization
	if kalman.first:
		kalman.x = measurements
		kalman.sigma *= 0.001
		
		# check if data has 'orientation_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'orientation_covariance')) and (np.any(data.orientation_covariance)):
			orientation_covariance = np.array(data.orientation_covariance).reshape(3,3)
		else:
			orientation_covariance = 0.01 * np.identity(3)

		# check if data has 'angular_velocity_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'angular_velocity_covariance')) and (np.any(data.angular_velocity_covariance)):
			angular_velocity_covariance = np.array(data.angular_velocity_covariance).reshape(3,3)
		else:
			angular_velocity_covariance = 0.01 * np.identity(3)
		
		# check if data has 'linear_acceleration_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'linear_acceleration_covariance')) and (np.any(data.linear_acceleration_covariance)):
			linear_acceleration_covariance = np.array(data.linear_acceleration_covariance).reshape(3,3)
		else:
			linear_acceleration_covariance = 0.01 * np.identity(3)

		# create R matrix
		kalman.R = block_diag(orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance)

		kalman.first = False
	
	# prediction and estimation phases
	kalman.predict()
	kalman.update(measurements)
	
	# kalman outputs
	angles_hat = np.concatenate((np.concatenate((kalman.x[0],kalman.x[1]), axis = 0), kalman.x[2]), axis = 0) 
	angular_velocity_hat = np.concatenate((np.concatenate((kalman.x[3],kalman.x[4]), axis = 0), kalman.x[5]), axis = 0)
	linear_acceleration_hat = np.concatenate((np.concatenate((kalman.x[6],kalman.x[7]), axis = 0), kalman.x[8]), axis = 0)
	
	rot_matrix = eulerAnglesToRotationMatrix(angles_hat)

	lin_acc_no_g = removeGravity(linear_acceleration_hat, rot_matrix)

	#write data
	if flagWriteData == 1:
		#write data without any filter

		#store data into .csv files in order to analyse them offline
		storeDataInFiles('lin_acc.csv','a',lin_acc_no_g)

		angles_in_deg = [(angles_hat[0]*180) / math.pi, (angles_hat[1]*180) / math.pi,(angles_hat[2]*180) / math.pi]
		storeDataInFiles('orientation.csv','a',angles_in_deg)

		storeDataInFiles('angVel.csv','a',angular_velocity_hat)

		index += 1

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous = True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous = True)

	# This declares that your node subscribes to the android/imu topic,
	# which is of type sensor_msgs.msg.Imu. When new data is received,
	# callback is invoked with that data as argument. 
	rospy.Subscriber("android/imu", Imu, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

	kalman = Kalman(n_state_variables = 9, n_measurements = 9)

	listener()