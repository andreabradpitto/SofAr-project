## @package Jacobian
# Computation of the geometric jacobian given DH parameters and joints angles
#

import numpy as np
import T_computations as t
import J_computations as j

## Wrapper that calls all the functions needed to compute J.
#
#
def main():
    pass

    p = np.pi
    n_joints = 7

    # Links length. [mm]
    L0 = 27.035
    L1 = 6.900
    L2 = 36.435
    L3 = 6.900
    L4 = 37.429
    L5 = 1.000
    L6 = 36.830

    # DH table of Baxter: alpha(i-1), a(i-1), d(i), theta(i).
    # Last row relates 7-th joint to end-effector.
    DH = np.array([[0, 0, L0, 0],
                   [-p/2, L1, 0, p/2],
                   [p/2, 0, L2, 0],
                   [-p/2, L3, 0, 0],
                   [p/2, 0, L4, 0],
                   [-p/2, L5, 0, 0],
                   [p/2, 0, 0, 0],
                   [0, 0, L6, 0]])

    # Trasformation matrices given DH table. T0,1 T1,2 ... T7,e
    T_rel_ini = t.DH_to_T(DH)
    
    # type of joints, 1 = revolute, 0 = prismatic.
    info = np.array([1, 1, 1, 1, 1, 1, 1])

    ###################################################
    # Entry point when receiving q from coppeliasim!!
    ###################################################

    # q in configuration at time k
    q = np.array([0, pi/4, 0,pi/4, 0, 0, 0])

    # transformations matrices given the configuration.
    T_trans = t.transformations(T_rel_ini, q, info)

    # T0,1 T0,2 T0,3...T0,e
    T_abs = t.abs_trans(T_trans)

    # extract geometric vectors needed for computations.
    geom_v = j.geometric_vectors(T_abs)

    k = geom_v[0] # axis of rotation of the revolute joins projected on zero
    r = geom_v[1] # distances end_effector, joints projected on zero.

    Js = j.jacob(k, r, n_joints, info)
    J = Js[0]
    Jl = Js[1]
    Ja = Js[2]

if __name__ == '__main__':
    main()
