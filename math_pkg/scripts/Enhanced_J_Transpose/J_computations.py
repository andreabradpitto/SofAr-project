import numpy as np

def geometric_vectors(T_abs):
  """!
  Computes the vectors needed to compute geometric jacobian.
  @param Tabs: the transformation matrices from joint to 0 frame in current configuration
  @return geom_v: geometric vectors exctracted from Tabs that allow to compute the jacobian.
  """
  r = []
  k = []
  geom_v = []

  n_matrices = len(T_abs)

  for i in range(n_matrices-1):
    tmp_k = np.array([[T_abs[i][0][2], T_abs[i][1][2], T_abs[i][2][2]]])
    tmp_k = np.transpose(tmp_k)
    k.append(tmp_k)

    tmp_r = np.array([[T_abs[n_matrices-1][0][3] - T_abs[i][0][3], T_abs[n_matrices-1][1][3] - T_abs[i][1][3], T_abs[n_matrices-1][2][3] - T_abs[i][2][3]]])
    tmp_r = np.transpose(tmp_r)
    r.append(tmp_r)

  geom_v.append(k)
  geom_v.append(r)

  return geom_v

#######################################
# New part

def i_j(T_abs):
  """!
  Computes the vectors needed to optimize Jtransp
  @param Tabs: the transformation matrices from joint to 0 frame in current configuration
  @return i_j: axes of joints projected on zero.
  """

  i = []
  j = []
  i_j = []

  n_matrices = len(T_abs)

  for h in range(n_matrices-1):
    tmp_i = np.array([[T_abs[h][0][0], T_abs[h][1][0], T_abs[h][2][0]]])
    tmp_j = np.array([[T_abs[h][0][1], T_abs[h][1][1], T_abs[h][2][1]]])

    tmp_i = np.transpose(tmp_i)
    tmp_j = np.transpose(tmp_j)

    i.append(tmp_i)
    j.append(tmp_j)

  i_j.append(i)
  i_j.append(j)

  return i_j

def axis_vector(i, j, k):
  """!
  Computes the vector needed for Jtransp optimisation.
  @param i, j, k: joint axes projected on zero.
  @return v: vector containing some of this axes.
  """

  v = np.zeros((9,3))

  v[0] = np.transpose(i[0])
  v[1] = np.transpose(j[1])
  v[2] = np.transpose(k[2])
  v[3] = np.transpose(j[3])
  v[4] = np.transpose(k[4])
  v[5] = np.transpose(j[5])
  v[6] = np.transpose(k[1])
  v[7] = np.transpose(k[3])
  v[8] = np.transpose(k[5])

  return v

#################################

def jacob(k, r, n_joints, info):
  """!
  Computes the jacobian matrix given the geometric vectors, number of joints and info.
  @param k: versors of axis z of the joints projected on 0.
  @param r: distance between joints and e.e. projected on 0.
  @param n_joints: explains it self.
  @param info: 1->revolute, 0->prismatic. In case there is a change in the serial chain the algorithm still works.
  @return J: jacobian matrix.
  """
  
  Ja = np.array([[],
                 [],
                 []])
  Jl = np.array([[],
                 [],
                 []])

  for i in range(n_joints):
    if info[i] == 1:
      Ja = np.concatenate((Ja, k[i]), axis = 1)
      kx = k[i][0][0]
      ky = k[i][1][0]
      kz = k[i][2][0]
      k_skew = np.array([[0, -kz, ky],
                         [kz, 0, -kx],
                         [-ky, kx, 0]])
      l_column = np.dot(k_skew, r[i])
      Jl = np.concatenate((Jl, l_column), axis = 1)

##    else:
##      zero = np.array([[0],
##                       [0],
##                       [0]])
##      Ja = np.concatenate((Ja, zero), axis = 1)
##      Jl = np.concatenate((Jl, k[i]), axis = 1)
  J = np.concatenate((Jl, Ja), axis = 0)

  return J
