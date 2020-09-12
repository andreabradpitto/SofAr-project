#!/usr/bin/env python
import numpy as np
"""
This carries out the gravity removal
@param lin_acc linear acceleration data incoming from the accelerometer
@param Rot_m rotation matrix computed with eulerAnglesToRotationMatrix()
@param g gravity vector
@returns the output is the same linear acceleration vector provided, but without the influence of the gravity
"""

def removeGravity(lin_acc, Rot_m, g):

    # rotate g vector in the current frame
    g_frame_i = np.dot(Rot_m, g)
    g_removed = [0, 0, 0]  # define linear acceleration without gravity

    #check the sign of the current linear acceleration, and remove the gravity properly
    for i in range(0, 3):
        if lin_acc[i] >= 0: 
            g_removed[i] = lin_acc[i] - abs(g_frame_i[i])
        if lin_acc[i] < 0:
            g_removed[i] = lin_acc[i] + abs(g_frame_i[i])

    return g_removed
