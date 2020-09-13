#!/usr/bin/env python

import numpy as np

p = np.pi

def DH_to_T(DH):
  """!
  Computes the transformation matrices given the DH table of the serial link.
  @param DH: devavitt-hartemberg parameters.
  @return T: transformation matrices of a joint with respect to previous joint.
  """
  # Get the number of rows, to know how many T matrices should create.
  rows = len(DH)

  T = []

  for i in range(rows):
    Tmp = np.array([[np.cos(DH[i,3]), -np.sin(DH[i,3]), 0, DH[i,1]],
            [np.sin(DH[i,3])*np.cos(DH[i,0]), np.cos(DH[i,3])*np.cos(DH[i,0]), -np.sin(DH[i,0]), -DH[i,2]*np.sin(DH[i,0])],
            [np.sin(DH[i,3])*np.sin(DH[i,0]), np.cos(DH[i,3])*np.sin(DH[i,0]), np.cos(DH[i,0]), DH[i,2]*np.cos(DH[i,0])],
            [0, 0, 0, 1]])
    T.append(Tmp)

  return T

def transformations(T_rel_ini, q, info):
  """!
  Computes tranformations given T_relatives, q's and the info.
  @param T_rel_ini: the ones computed with DH_to_T.
  @param q: current configuration of baxter's arm.
  @param info: 1->revolute, 0->prismatic.
  @return T: transformation matrices of a joint with respect to previous joint in
  the new configuration.
  """
  row_q = q.size
  row_info = info.size

  T = []

  if row_q != row_info:
    print("Warning. q and info must have same size.")
    return

  for i in range(row_q):
    if info[i] == 1:
      Tel = np.array([[np.cos(q[i]), -np.sin(q[i]), 0 , 0],
                      [np.sin(q[i]), np.cos(q[i]), 0 , 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
  #  else:
      # Case in which there are prismatic joints.
##      Tel = np.array([[1, 0, 0, 0],
##                      [0, 1, 0, 0],
##                      [0, 0, 0, q[i]]
##                      [0, 0, 0, 1]])

    Tmp = np.dot(T_rel_ini[i], Tel)
    #print("++++Tmp")
    #print(Tmp)
    T.append(Tmp)

  # Last matrix is constant in time. T_7,e.e
  T.append(T_rel_ini[row_q])    

  return T
  
def abs_trans(T_rel):
  """!
  Computes trasformations matrices w.r.t. 0 frame.
  @param T_rel: trasformation matrices of a joint with respect to previous one.
  @return T: absolute transformation matrices.
  """
  T = []
  # First is the same.
  T.append(T_rel[0])

  for i in range(1, len(T_rel)):
    Tmp = np.dot(T[i-1], T_rel[i])
    T.append(Tmp)

  return T
