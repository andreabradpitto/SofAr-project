import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

def init_float64_multiarray(rows,columns):
    """!
    Function that initializes a Float64MultiArray of size rows x columns.
    @param rows: Number of rows of the returned multiarray.
    @param columns: Number of columns of the returned multiarray.
    @return a: empty Float64MultiArray instance.
    """
    a = Float64MultiArray()
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim.append(MultiArrayDimension())
    a.layout.dim[0].label ="rows"
    a.layout.dim[0].size = rows
    a.layout.dim[1].label ="columns"
    a.layout.dim[1].size = columns
    return a

def anglesCompensate(angles):
    """!
    Function used to filter unwanted minimal incoming data fluctuations,
    due to noise as well as human operator shake
    @param angles: orientation with respect to X, Y, Z axes
    @returns compensatedAngles: returns a filtered version (if necessary) of the input angles
    """
    dx = 0.0174  # min angle perceived [rad], about 1 [deg]
    # reduce sensibility of sensor: minimum precision is dx
    compensatedAngles = [0, 0, 0]

    for i in range(0, 3):  # i = 0, 1, 2
        if abs(angles[i] / dx) >= 1:
            compensatedAngles[i] = angles[i]

    return compensatedAngles

def eulerAnglesToRotationMatrix(angles):  # angles [roll, pitch, yaw]
    """!
    Function that transforms euler angle coordinates into the rotation matrix
    @param angles euler angles, i.e. orientation with respect to X, Y, Z axes
    @returns R: rotation matrix
    """

    R_x = np.array([[1,         0,                  0],
                    [0,         np.cos(angles[0]),   np.sin(angles[0])],
                    [0,         -np.sin(angles[0]),  np.cos(angles[0])]
                    ])

    R_y = np.array([[np.cos(angles[1]),    0,      -np.sin(angles[1])],
                    [0,                      1,      0],
                    [np.sin(angles[1]),    0,      np.cos(angles[1])]
                    ])

    R_z = np.array([[np.cos(angles[2]),      np.sin(angles[2]),     0],
                    [-np.sin(angles[2]),     np.cos(angles[2]),     0],
                    [0,                        0,                       1]
                    ])

    R = np.dot(R_x, np.dot(R_y, R_z))

    return R
