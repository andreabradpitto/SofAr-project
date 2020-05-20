from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

flag = 0 # set this flag to 1 in order to see the results
          # obtained  without EKF

font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 12,
        }

def plotDatas(df,subplt,x_axis,y_lable,titlePlot):
    plt.subplot(subplt)
    plt.plot(x_axis,df.X)
    plt.plot(x_axis,df.Y)
    plt.plot(x_axis,df.Z)
    plt.xlabel('#samples',fontdict=font)
    plt.ylabel(y_lable,fontdict=font)
    plt.title(titlePlot,fontdict=font)
    plt.legend()

if __name__ == '__main__':

    df_linacc_NO_EKF = pd.read_csv('lin_acc_NO_EKF.csv',names=['X','Y','Z'], header=0,decimal=',') 
    df_rot_NO_EKF = pd.read_csv('orientation_NO_EKF.csv',names=['X','Y','Z'],header=0,decimal=',')
    df_angVel_NO_EKF = pd.read_csv('angVel_NO_EKF.csv',names=['X','Y','Z'],header=0,decimal=',')

    # initialize x axis using number of rows of datas
    t=len(df_linacc_NO_EKF.X)
    x_axis = np.arange(start=1 ,stop=t+1,step=1)

    #plots data 
    plt.figure(figsize=(12,12))
    temp = df_linacc_NO_EKF.astype(float)
    plotDatas(temp,311,x_axis,'linear acceleration [m/s^2]','Linear Acceleration NO EKF')

    temp = df_rot_NO_EKF.astype(float)
    plotDatas(temp,312,x_axis,'Orientation [deg]','Orientation angles (yaw, pitch, roll) NO EKF')

    temp = df_angVel_NO_EKF.astype(float)
    plotDatas(temp,313,x_axis,'angular vel [rad/s]','Angular velocities NO EKF')

 #------do it only if you want to see results from imu withouth EKF----------
 # set flag = 1 at start of this file
    if flag == 1:
        plt.figure(figsize=(12,12))

        df_linacc = pd.read_csv('lin_acc.csv',names=['X','Y','Z'], header=0,decimal=',') 
        df_rot = pd.read_csv('orientation.csv',names=['X','Y','Z'],header=0,decimal=',')
        df_angVel = pd.read_csv('angVel.csv',names=['X','Y','Z'],header=0,decimal=',')

        # initialize x axis using number of rows of datas
        t=len(df_linacc.X)
        x_axis = np.arange(start=1,stop=t+1,step=1)

        #plot data
        temp = df_linacc.astype(float)
        plotDatas(temp,311,x_axis,'linear acceleration [m/s^2]','Estimated Linear Acceleration without gravity')

        temp = df_rot.astype(float)
        plotDatas(temp,312,x_axis,'Orientation [deg]','Orientation angles (yaw, pitch, roll)')

        temp = df_angVel.astype(float)
        plotDatas(temp,313,x_axis,'angular vel [rad/s]','Angular velocities')
    
    plt.show()