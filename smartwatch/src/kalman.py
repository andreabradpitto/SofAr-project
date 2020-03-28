#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class Kalman(object):
	"""docstring for Kalman"""
	def __init__(self, n_states, n_sensors):
		super(Kalman, self).__init__()
		self.n_states = n_states
		self.n_sensors = n_sensors

		self.x = np.matrix(np.zeros(shape=(n_states,1)))
		self.P = np.matrix(np.identity(n_states)) 
		self.F = np.matrix(np.identity(n_states))
		self.u = np.matrix(np.zeros(shape=(n_states,1)))
		self.H = np.matrix(np.zeros(shape=(n_sensors, n_states)))
		self.R = np.matrix(np.identity(n_sensors))
		self.I = np.matrix(np.identity(n_states))

		self.first = True

	def update(self, Z):
		'''Z: new sensor values as numpy matrix'''

		w = Z - self.H * self.x
		S = self.H * self.P * self.H.getT() + self.R
		K = self.P * self.H.getT() * S.getI()
		self.x = self.x + K * w
		self.P = (self.I - K * self.H) * self.P

	def predict(self):
		self.x = self.F * self.x + self.u
		self.P = self.F * self.P * self.F.getT()


def callback(data):

	linear_acceleration = Vector3()

	linear_acceleration.x = data.linear_acceleration.x
	linear_acceleration.y = data.linear_acceleration.y
	linear_acceleration.z = data.linear_acceleration.z

	rospy.loginfo("Imu result: %lf %lf %lf", linear_acceleration.x,linear_acceleration.y,linear_acceleration.z)

	Z = np.matrix([linear_acceleration.x,linear_acceleration.y,linear_acceleration.z]).getT()

	if kalman.first:
		kalman.x = Z
		kalman.first = False

	
	kalman.predict()
	kalman.update(Z)

	vec = Vector3()

	vec.x = kalman.x[0]
	vec.y = kalman.x[1]
	vec.z = kalman.x[2]


	rospy.loginfo("Kalman result: %lf %lf %lf", vec.x,vec.y,vec.z)
	
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("android/imu", Imu, callback)

	
	kalman.H = np.matrix(np.identity(kalman.n_states))
	kalman.P *= 0.1
	kalman.R *= 0.01

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

	kalman = Kalman(n_states = 3, n_sensors = 3)

	listener()