from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

flag = 0 # set this flag to 1 in order to plot the results
         # obtained without Kalman Filter, if any

font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 12,
        }

def plotData(df, subplt, x_axis, y_lable, titlePlot):
    plt.subplot(subplt)
    plt.plot(x_axis, df.X)
    plt.plot(x_axis, df.Y)
    plt.plot(x_axis, df.Z)
    plt.xlabel('#samples', fontdict = font)
    plt.ylabel(y_lable, fontdict = font)
    plt.title(titlePlot, fontdict = font)
    plt.legend()

if __name__ == '__main__':

    df_linacc = pd.read_csv('lin_acc.csv',names=['X','Y','Z'], header = 0,decimal = ',') 
    df_rot = pd.read_csv('orientation.csv',names=['X','Y','Z'],header = 0,decimal = ',')
    df_angVel = pd.read_csv('angVel.csv',names=['X','Y','Z'],header = 0,decimal = ',')
    
    # initialize x axis using number of rows of data
    t = len(df_linacc.X)
    x_axis = np.arange(start = 1,stop = t + 1,step = 1)

    # plots data 
    plt.figure(figsize = (12,12))
    temp = df_linacc.astype(float)
    plotDatas(temp,311,x_axis,'Filtered linear acceleration [m/s^2]','Linear Acceleration')

    temp = df_rot.astype(float)
    plotData(temp,312,x_axis,'Filtered Orientation [deg]','Orientation angles (yaw, pitch, roll)')

    temp = df_angVel.astype(float)
    plotData(temp,313,x_axis,'Filtered angular vel [rad/s]','Angular velocities')

 # next lines are relevant only if interested in seeing results from imu withouth EKF
    if flag == 1: # set flag = 1 at the beginning of this file

        plt.figure(figsize = (12,12))
        df_linacc_NO_KF = pd.read_csv('lin_acc_NO_KF.csv',names=['X','Y','Z'], header=0,decimal=',') 
        df_rot_NO_KF = pd.read_csv('orientation_NO_KF.csv',names=['X','Y','Z'],header=0,decimal=',')
        df_angVel_NO_KF = pd.read_csv('angVel_NO_KF.csv',names=['X','Y','Z'],header=0,decimal=',')
    
        # initialize x axis using number of rows of data
        t = len(df_linacc_NO_KF.X)
        x_axis = np.arange(start = 1,stop = t + 1, step = 1)
        # plots data 
        plt.figure(2)
        temp = df_linacc_NO_KF.astype(float)
        plotData(temp,311,x_axis,'linear acceleration [m/s^2]','Linear Acceleration computed withouth the Kalman Filter')

        temp = df_rot_NO_KF.astype(float)
        plotData(temp,312,x_axis,'Orientation [deg]','Orientation angles (yaw, pitch, roll) computed withouth the Kalman Filter')

        temp = df_angVel_NO_KF.astype(float)
        plotData(temp,313,x_axis,'angular vel [rad/s]','Angular velocities computed withouth the Kalman Filter')
    
    plt.show()