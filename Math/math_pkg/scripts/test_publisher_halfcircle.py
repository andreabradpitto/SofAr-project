#!/usr/bin/env python

import rospy
import numpy as np
import scipy.io
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8


key = 0 # 1 if publisher must publish, 0 otherwise
reset = 0 # 1 if publisher must reset to initial conditions, 0 otherwise
pub = None


def simulate_callback(data):
    """!
    Handles changes in the simulation. If data = 0, then the initial conditions must be resetted.
    If data = 1, then the algorithm moves on. If data = 2, then the algorithm pauses.
    @param data: coming from coppelia_sim.
    """
    global key,reset
    if data.data == 0:
        key = 0; reset = 1
    elif data.data == 1:
        key = 1; reset = 0
    elif data.data == 2:
        key = 0; reset = 0
        
        
        
def talker():
    """!
    Publishes mock smartphone signals, read from files.
    """
    global pub,key,reset
    
    rate = rospy.Rate(100)
	
    # Load reference signals from MATLAB files
    # Must be run in same folder as such files
    agmat = scipy.io.loadmat("ag1halfcircle.mat")
    wgmat = scipy.io.loadmat("wg1halfcircle.mat")
    quamat = scipy.io.loadmat("quatg1halfcircle.mat")
    ag = agmat["ag1"]
    wg = wgmat["wg1"]
    qua = quamat["quatg1"]

    K = 0
    imu = Imu()
    
    while True:
        if key == 1:
           imu.orientation.w = qua[0,K]
           imu.orientation.x = qua[1,K]
           imu.orientation.y = qua[2,K]
           imu.orientation.z = qua[3,K]
           imu.linear_acceleration.x = ag[0,K]
           imu.linear_acceleration.y = ag[1,K]
           imu.linear_acceleration.z = ag[2,K]
           imu.angular_velocity.x = wg[0,K]
           imu.angular_velocity.y = wg[1,K]
           imu.angular_velocity.z = wg[2,K]
           if K < 300: K = K + 1 # 300 is the number of samples of the trajectory
           pub.publish(imu)
        elif reset == 1: K = 0
        rate.sleep()

if __name__ == '__main__':
    try:
       rospy.init_node('fakeSmart', anonymous=True)
       pub = rospy.Publisher('smartphone', Imu, queue_size=10)
       rospy.Subscriber("handleSimulation", Int8, simulate_callback) # subscribe to simulation messages
       talker()
    except rospy.ROSInterruptException:
       pass
