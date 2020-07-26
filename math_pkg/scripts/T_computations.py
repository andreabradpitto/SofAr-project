import numpy as np

p = np.pi

## Computes the transformation matrices given the DH table of the serial link.
#
#
def DH_to_T(DH):
  pass
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

## Computes tranformations given T_relative, q's and the info.
#
#
def transformations(T_rel_ini, q, info):
  pass
  row_q = len(q)
  row_info = len(info)

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
    T.append(Tmp)

  # Last matrix is constant in time. T_7,e.e
  T.append(T_rel_ini[row_q])    

  return T
  
## Computes trasformations matrices w.r.t. 0 frame.
#
#
def abs_trans(T_rel):
  pass
  T = []
  # First is the same.
  T.append(T_rel[0])

  for i in range(1, len(T_rel)):
    Tmp = np.dot(T[i-1], T_rel[i])
    T.append(Tmp)

  return T
