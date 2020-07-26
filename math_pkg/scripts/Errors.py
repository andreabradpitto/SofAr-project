#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Float64MultiArray

# Initialization of publisher
pub = rospy.Publisher('errors', Float64MultiArray, queue_size=10)

# Initialization of Goal and e.e. orientation matrices.
Rg = np.zeros((3,3))
Re = np.zeros((3,3))

# Initialization of Goad and e.e. position and velocity vectors
xg = np.zeros((3,1))
xe = np.zeros((3,1))

vg = np.zeros((3,1))
ve = np.zeros((3,1))

def init_float64_multiarray(rows,columns):
    """!
    Function that initializes a Float64MultiArray of size rows x columns.
    @param rows: Number of rows of the returned multiarray.
    @param columns: Number of columns of the returned multiarray.
    @return empty Float64MultiArray instance.
    """
    a = Float64MultiArray()
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim[0].label ="rows"
    a.layout.dim[0].size = rows
    a.layout.dim[1].label ="columns"
    a.layout.dim[1].size = columns
    return a

## Computes the angular misalingment of goal frame and e.e. frame
def ang_mis(Rg, Re):
  
  i = np.transpose(np.array([[1, 0, 0]]))
  j = np.transpose(np.array([[0, 1, 0]]))
  k = np.transpose(np.array([[0, 0, 1]]))

  skew_i = np.array([[0, 0, 0],
                     [0, 0, -1],
                     [0, 1, 0]])

  skew_j = np.array([[0, 0, 1],
                     [0, 0, 0],
                     [-1, 0, 0]])
  
  skew_k = np.array([[0, -1, 0],
                     [1, 0, 0],
                     [0, 0, 0]])

  # this term equals 1 + 2cos(theta), with theta the misalignement angle
  summ = np.dot(np.transpose(np.dot(Re, i)), np.dot(Rg, i)) + np.dot(np.transpose(np.dot(Re, j)), np.dot(Rg, j)) + np.dot(np.transpose(np.dot(Re, k)), np.dot(Rg, k))

  Re_ = np.transpose(Re)
  
  # this term equals 2*v*sin(theta), with v the angular misalignement vector
  prod = np.dot(np.dot(Re, np.dot(skew_i, Re_)), np.dot(Rg, i)) + np.dot(np.dot(Re, np.dot(skew_j, Re_)), np.dot(Rg, j)) + np.dot(np.dot(Re, np.dot(skew_k, Re_)), np.dot(Rg, k))

  delta = (summ - 1)/2 # = cos(theta)

  if np.absolute(delta) < 1:
    theta = np.arccos(delta)
    v = prod/(2*np.sin(theta))
    v = v/(np.linalg.norm(v))
    rho = theta*v
    
    return rho
  elif delta == 1:
    # theta multiple of 2*k*pi
    theta = 0
    rho = np.array([[0, 0, 0]])
    
    return rho.transpose()
  else:
    theta = np.pi
    summ1 = np.dot(Re, i) + np.dot(Re, j) + np.dot(Re, k) + np.dot(Rg, i) + np.dot(Rg, j) + np.dot(Rg, k)
    v0 = summ1/(np.linalg.norm(summ1))
    rho = theta*v0
    
    return rho

def errors(data):

  Data = data.data
  global Rg, Re, xg, xe, vg, ve
  
  # Moves along Data vector.
  k = 0
  
  for i in range(3):
    for j in range(3):
      Rg[i][j] = Data[k+j]

    k = k + 3

  for i in range(3):
    for j in range(3):
      Re[i][j] = Data[k+j]

    k = k + 3

  for i in range(3):
    xg[i][0] = Data[k+i]

  k = k + 3

  for i in range(3):
    xe[i][0] = Data[k+i]

  k = k + 3

  for i in range(3):
    vg[i][0] = Data[k+i]

  k = k + 3

  for i in range(3):
    ve[i][0] = Data[k+i] 

  # angular misalignment
  rho = ang_mis(Rg, Re)  

  # position error
  eta = xg - xe

  # velocity error
  ni = vg - ve

  ######################
  # Send to Math block
  ######################

  # Send rho, eta, ni
  err = np.array([rho[0], rho[1], rho[2], eta[0], eta[1], eta[2], ni[0], ni[1], ni[2]], dtype=np.float32)
  errors = init_float64_multiarray(6, 1)
  errors.data = err
  pub.publish(errors)


def listener():

    ##############################################################
    # Read from Forward_kine topic following data
    # R0e_kplus1, R0e_k; v_0e_k, v_0e_kplus1; x_0e_k, x_0e_kplus1
    ##############################################################

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # errors is the callback functions
    rospy.Subscriber("Data_for_errors", Float64MultiArray, errors)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()  


if __name__ == '__main__':
    
    listener()
