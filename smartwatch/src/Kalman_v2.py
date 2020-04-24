#!/usr/bin/env python
import rospy
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import time

g = [ 0, 0,-9.801]
#flag = 1
dx = 0.0174533 # min angle sensed [rad], 1 [deg]

pastAngles = [0, 0, 0]

class Kalman(object):
	"""docstring for Kalman"""
	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors

		self.x = np.matrix(np.zeros(shape=(n_states, 1)))
		self.sigma = np.matrix(np.identity(n_states)) 
		self.F = np.matrix(np.identity(n_states))
		self.u = np.matrix(np.zeros(shape=(n_states, 1)))
		self.G = np.matrix(np.identity(n_states))
		self.R = np.matrix(np.identity(n_sensors))
		self.I = np.matrix(np.identity(n_states))

		self.first = True

	def predict(self):
		self.x = self.F * self.x + self.u
		self.sigma = self.F * self.sigma * self.F.getT()

	def update(self, Y):
		'''Y: new sensor values as numpy matrix'''

		w = Y - self.G * self.x
		S = self.G * self.sigma * self.G.getT() + self.R
		H = self.sigma * self.G.getT() * S.getI()
		self.x = self.x + H * w
		self.sigma = self.sigma - (H * S * H.getT())

def eulerAnglesToRotationMatrix(angles) : #angles[x,y,z]
    
    R_x = np.array([[1,         0,                  0                     ],
                    [0,         math.cos(angles[0]), -math.sin(angles[0]) ],
                    [0,         math.sin(angles[0]),  math.cos(angles[0]) ]
                    ])
                    
    R_y = np.array([[math.cos(angles[1]),    0,      math.sin(angles[1])  ],
                    [0,                      1,      0                    ],
                    [-math.sin(angles[1]),   0,      math.cos(angles[1])  ]
                    ])
                
    R_z = np.array([[math.cos(angles[2]),    -math.sin(angles[2]),     0],
                    [math.sin(angles[2]),     math.cos(angles[2]),     0],
                    [0,                       0,                       1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def anglesCompensate(angles) :
	compensatedAngles = [0, 0, 0]
	rospy.loginfo("angles before :   %lf %lf %lf",angles[0], angles[1], angles[2])

	for i in compensatedAngles:
		if abs(angles[i]/ dx) >= 1 : 
			compensatedAngles[i] = angles[i]
	#rospy.loginfo("angles after:   %lf %lf %lf",compensatedAngles[0], compensatedAngles[1], compensatedAngles[2])
		
	return compensatedAngles

def removeGravity(lin_acc, Rot_m):
	#rotate g vector in the current frame 
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

	rospy.loginfo("Lin_acc x %lf = lin_acc %lf + g %lf", g_removed[0],lin_acc[0],g_frame_i[0])
	rospy.loginfo("Lin_acc y %lf = lin_acc %lf + g %lf", g_removed[1],lin_acc[1],g_frame_i[1])
	rospy.loginfo("Lin_acc z %lf = lin_acc %lf + g %lf\n\n", g_removed[2],lin_acc[2],g_frame_i[2])

	return res

def callback(data):
	global pastAngles

	orientation = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]
	#angular_velocity = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
	linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

	angles = anglesCompensate([ data.orientation.x, data.orientation.y, data.orientation.z ])
	for i in range(0,3) :
		pastAngles[i] = angles[i]
	
	rot_matrix = eulerAnglesToRotationMatrix(angles)

	no_g = removeGravity(linear_acceleration, rot_matrix)

	Z = np.matrix(linear_acceleration).getT()

	if kalman.first:
		kalman.x = Z
		kalman.first = False
	
	kalman.predict()
	kalman.update(Z)

	vec = [kalman.x[0], kalman.x[1], kalman.x[2]]

	#rospy.loginfo("Kalman result:   %lf %lf %lf", vec[0], vec[1], vec[2])
	#rospy.loginfo("Rotation result: %lf %lf %lf", vec[0], vec[1], vec[2])
	
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

	kalman.sigma *= 0.1
	kalman.R *= 0.01

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

	kalman = Kalman(n_states = 3, n_sensors = 3)


	listener()

