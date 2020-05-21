#!/usr/bin/env python
import rospy
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import csv
from scipy.linalg import block_diag

# global variables
g = [ 0, 0, 9.81]
dx = 0.0174 # min angle sensed [rad],about 1 [deg]
# dx = 0.087 # min angle sesed [rad], about 5 [deg]
pastAngles_hat = [0, 0, 0]
index=1 # used for storing data for offline analysis
deltaT = 0.01 # between 2 measurements

# initialize files to store datas
with open('lin_acc.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('orientation.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('angVel.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

class Kalman(object):

	def __init__(self, n_state_variables, n_measurements):
		super(Kalman, self).__init__()
		self.n_state_variables = n_state_variables
		self.n_measurements = n_measurements

		self.x = np.zeros((n_state_variables, 1))
		self.sigma = np.identity(n_state_variables) 
		self.F = np.identity(n_state_variables)
		self.F[0, 3] = deltaT
		self.F[1, 4] = deltaT
		self.F[2, 5] = deltaT
		self.G = np.identity(n_state_variables)
		self.R = np.identity(n_measurements)
		self.I = np.identity(n_state_variables)

		self.first = True

	def predict(self):
		self.x = np.dot(self.F, self.x)
		self.sigma = np.dot(self.F, np.dot(self.sigma, self.F.T))

	def update(self, Y):

		# Y: new sensor values
		w = Y - np.dot(self.G, self.x)
		S = np.dot(self.G, np.dot(self.sigma, self.G.T)) + self.R
		H = np.dot(self.sigma, np.dot(self.G.T, np.linalg.inv(S)))
		self.x = self.x + np.dot(H, w)
		self.sigma = self.sigma - np.dot(H, np.dot(S, H.T))

def readDataFromSensors(data):

	# get data from sensor
	orientation = [data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w]
	angular_velocity = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
	linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

	# transform quaternion to euler angles
	angles = np.array(tf.transformations.euler_from_quaternion(orientation,"sxyz"))

	# create measurements column vector
	measurements = np.concatenate((angles, angular_velocity), axis=0)
	measurements = np.concatenate((measurements, linear_acceleration), axis=0)
	measurements = np.array([measurements]).T

	rospy.loginfo("Measured angles (degrees): %lf %lf %lf", math.degrees(angles[0]), math.degrees(angles[1]), math.degrees(angles[2]))
	rospy.loginfo("Measured ang vel: %lf %lf %lf", angular_velocity[0], angular_velocity[1], angular_velocity[2])
	rospy.loginfo("Measured lin acc: %lf %lf %lf\n", linear_acceleration[0], linear_acceleration[1], linear_acceleration[2])

	return measurements

def eulerAnglesToRotationMatrix(angles): #angles[x,y,z]
    
    R_z = np.array([[1,         0,                  0                     ],
                    [0,         math.cos(angles[2]),   math.sin(angles[0]) ],
                    [0,         -math.sin(angles[2]),  math.cos(angles[0]) ]
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
	#rospy.loginfo("angles before :   %lf %lf %lf",angles[0], angles[1], angles[2])

	for i in range (0,3):
		if abs(angles[i]/ dx) >= 1 : 
			compensatedAngles[i] = angles[i]
	#rospy.loginfo("angles after:   %lf %lf %lf",compensatedAngles[0], compensatedAngles[1], compensatedAngles[2])
		
	return compensatedAngles
	
def removeGravity(lin_acc, Rot_m):
	# rotate g vector in the current frame 
	g_frame_i = np.dot(Rot_m, g)
	g_removed = [0, 0, 0] # define linear acceleration without gravity
	
	#rospy.loginfo("%lf %lf %lf", Rot_m[0,0], Rot_m[0,1], Rot_m[0,2])
	#rospy.loginfo("%lf %lf %lf", Rot_m[1,0], Rot_m[1,1], Rot_m[1,2])
	#rospy.loginfo("%lf %lf %lf\n\n", Rot_m[2,0], Rot_m[2,1], Rot_m[2,2])

	for i in range(0,3):
		if lin_acc[i] >= 0:
			g_removed[i] = lin_acc[i] - abs(g_frame_i[i])
		if lin_acc[i] <0:
			g_removed[i] = lin_acc[i] + abs(g_frame_i[i])

	#rospy.loginfo("Acc_x=lin_acc+g\t%lf\t%lf\t%lf", g_removed[0],lin_acc[0],g_frame_i[0])
	#rospy.loginfo("Acc_y=lin_acc+g\t%lf\t%lf\t%lf", g_removed[1],lin_acc[1],g_frame_i[1])
	#rospy.loginfo("Acc_z=lin_acc+g\t%lf\t%lf\t%lf\n\n", g_removed[2],lin_acc[2],g_frame_i[2])

	return g_removed

def callback(data):

	global index

	measurements = readDataFromSensors(data)

	#rospy.loginfo("Imu angles: %lf %lf %lf", measurements[0], measurements[1], measurements[2])
	#rospy.loginfo("Imu acc: %lf %lf %lf", measurements[6], measurements[7], measurements[9])

	# Kalman initialization
	if kalman.first:
		kalman.x = measurements
		kalman.sigma *= 0.001
		
		# check if data has 'orientation_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'orientation_covariance')) and (np.any(data.orientation_covariance)):
			orientation_covariance = np.array(data.orientation_covariance).reshape(3,3)
		else:
			orientation_covariance = 0.01*np.identity(3)

		# check if data has 'angular_velocity_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'angular_velocity_covariance')) and (np.any(data.angular_velocity_covariance)):
			angular_velocity_covariance = np.array(data.angular_velocity_covariance).reshape(3,3)
		else:
			angular_velocity_covariance = 0.01*np.identity(3)
		
		# check if data has 'linear_acceleration_covariance' field and if it is not all zero, otherwise set default value
		if (hasattr(data, 'linear_acceleration_covariance')) and (np.any(data.linear_acceleration_covariance)):
			linear_acceleration_covariance = np.array(data.linear_acceleration_covariance).reshape(3,3)
		else:
			linear_acceleration_covariance = 0.01*np.identity(3)

		# create R matrix
		kalman.R = block_diag(orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance)

		kalman.first = False
	
	# prediction and estimation phases
	kalman.predict()
	kalman.update(measurements)
	
	# kalman outputs
	angles_hat = np.concatenate((np.concatenate((kalman.x[0],kalman.x[1]), axis=0), kalman.x[2]), axis=0) 
	angular_velocity_hat = np.concatenate((np.concatenate((kalman.x[3],kalman.x[4]), axis=0), kalman.x[5]), axis=0)
	linear_acceleration_hat = np.concatenate((np.concatenate((kalman.x[6],kalman.x[7]), axis=0), kalman.x[8]), axis=0)

	rospy.loginfo("Kalman angles (degrees): %lf %lf %lf", math.degrees(angles_hat[0]), math.degrees(angles_hat[1]), math.degrees(angles_hat[2]))
	rospy.loginfo("Kalman ang vel: %lf %lf %lf", angular_velocity_hat[0], angular_velocity_hat[1], angular_velocity_hat[2])
	rospy.loginfo("Kalman lin acc: %lf %lf %lf\n", linear_acceleration_hat[0], linear_acceleration_hat[1], linear_acceleration_hat[2])

	#angles_hat = anglesCompensate(angles_hat)
	#for i in range(0,3) :
	#	pastAngles_hat[i] = angles_hat[i]
	
	rot_matrix = eulerAnglesToRotationMatrix(angles_hat)

	lin_acc_no_g = removeGravity(linear_acceleration_hat, rot_matrix)

	rospy.loginfo("Acc without g: %lf %lf %lf\n\n\n", lin_acc_no_g[0], lin_acc_no_g[1], lin_acc_no_g[2])

	# write data
	with open('lin_acc.csv','a') as file:
		writer = csv.writer(file)
		writer.writerow([index,lin_acc_no_g[0],lin_acc_no_g[1],lin_acc_no_g[2]])
		
	with open('orientation.csv','a') as file:
		writer = csv.writer(file)
		writer.writerow([index,(angles_hat[0]* 180) / math.pi, (angles_hat[1]*180 )/ math.pi,(angles_hat[2]*180) / math.pi])
	
	with open('angVel.csv','a') as file:
		writer = csv.writer(file)
		writer.writerow([index,angular_velocity_hat[0],angular_velocity_hat[1],angular_velocity_hat[2]])
	index+=1

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