#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension,Int8
from sensor_msgs.msg import JointState

q = np.zeros((7,1))
qdot = np.zeros((7,1))
qdotprev = None
qdotprevprev = None
qdotppnone = True
qdotpnone = True
qold = np.zeros((7,1))
eff = 0
key = 0
DT = 0.01

def qdot_callback (qdot_data):
    """!
    Receives qdot vector and sents q vector obtained by integration.
    @params qdot_data: qdot message received from weighter.
    """
    global q,qdot,eff,key,pub,qdotprev,qdotprevprev,qold,qdotpnone,qdotppnone
    qdot = np.array([qdot_data.velocity]).transpose() # store received vector in global variable
    eff = qdot_data.effort[0] # store new seq number
    tosend = JointState() # joint state object to be sent
    qtmp = q # store old q in qtmp

    # Integration
    if not qdotppnone: # can use Simpson integration
        q = qold + DT * (qdot + qdotprevprev + 4 * qdotprev) / 3
    elif not qdotpnone: # can use trap integration
        q = q + DT * (qdot + qdotprev) * .5
        #qdotppnone = False
    else: # can use rectangular integration
        q = q + DT * qdot
        qdotpnone = False

    # Update past values
    qdotprev = qdot
    qdotprevprev = qdotprev
    qold = qtmp

    # Fill in and send object tosend
    tosend.velocity = q
    tosend.effort = [eff]
    pub.publish(tosend)


def simulate_callback(data):
    """!
    Handles changes in the simulation. If data = 0, then the initial conditions must be resetted.
    @param data: coming from coppelia_sim.
    """
    global q,key
    if data.data == 0:
        q = np.zeros((7,1));key = 0


def integr():
    """!
    Integrator function
    """
    global q,qdot,key
    rospy.Subscriber("cmdtopic", JointState, qdot_callback) # subscribe to weighter
    rospy.Subscriber("handleSimulation", Int8, simulate_callback) # subscribe to simulation messages
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()  

if __name__ == '__main__':
    try:
        rospy.init_node('integrator_node', anonymous=True)
        pub = rospy.Publisher('logtopic', JointState, queue_size=10)
        integr()
    except rospy.ROSInterruptException:
        pass
