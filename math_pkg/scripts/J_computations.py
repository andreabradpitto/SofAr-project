import numpy as np

## Computes the vectors needed to compute geometric jacobian.
#
#
def geometric_vectors(T_abs):
  pass
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

## Computes jacobians.
#
def jacob(k, r, n_joints, info):
  pass
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
