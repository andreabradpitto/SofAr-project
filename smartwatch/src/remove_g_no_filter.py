import rospy
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import csv

#flags for different feature
flagWriteData=1 # write simple data received from imu, after removed gravity
flagWriteFilteredData=0 # used if you want to store filtered data, if any

#global variables
g = [ 0, 0, 9.81]
dx = 0.0174 # min angle sensed [rad],about 1 [deg]
#dx = 0.087 # min angle sesed [rad], about 5 [deg]
index=1 #used for storing data for offline analysis

#initialize files to store datas
with open('lin_acc_NO_EKF.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('orientation_NO_EKF.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

with open('angVel_NO_EKF.csv','w') as file:
	writer = csv.writer(file)
	writer.writerow(["X","Y","Z"])

def eulerAnglesToRotationMatrix(angles) : #angles[x,y,z]
    
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

def anglesCompensate(angles) :
	#reduce sensibility of sensor: minimum precision is dx
	compensatedAngles = [0, 0, 0]
	#rospy.loginfo("angles before :   %lf %lf %lf",angles[0], angles[1], angles[2])

	for i in range (0,3):
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

	#rospy.loginfo("Acc_x=lin_acc+g\t%lf\t%lf\t%lf", g_removed[0],lin_acc[0],g_frame_i[0])
	#rospy.loginfo("Acc_y=lin_acc+g\t%lf\t%lf\t%lf", g_removed[1],lin_acc[1],g_frame_i[1])
	#rospy.loginfo("Acc_z=lin_acc+g\t%lf\t%lf\t%lf\n\n", g_removed[2],lin_acc[2],g_frame_i[2])

	return g_removed

def storeDataInFiles(fileName,modality, data):
	with open(fileName,modality) as file:
		writer = csv.writer(file)
		writer.writerow([ index,data[0], data[1], data[2] ])

def callback(data):
	global index

	#get data
	orientation = [ data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w]

	#transform quaternion to euler angles
	angles = tf.transformations.euler_from_quaternion(orientation,"sxyz")

	angular_velocity = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
	linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]

	angles = anglesCompensate(angles)
	
	rot_matrix = eulerAnglesToRotationMatrix(angles)

	lin_acc_no_g = removeGravity(linear_acceleration, rot_matrix)

	if flagWriteData==1:
		#write data without any filter
		storeDataInFiles('lin_acc_NO_EKF.csv','a',lin_acc_no_g)

		angles_in_deg =[ (angles[0]* 180) / math.pi, (angles[1]*180 )/ math.pi,(angles[2]*180) / math.pi]
		storeDataInFiles('orientation_NO_EKF.csv','a',angles_in_deg)

		storeDataInFiles('angVel_NO_EKF.csv','a',angular_velocity)

		#if flagWriteFilteredData==1:
			#now write data after filter

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
	
	listener()


