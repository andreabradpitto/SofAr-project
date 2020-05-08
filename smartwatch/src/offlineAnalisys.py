from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

def plotDatas(df,subplt,x_axis,y_lable,titlePlot):
    plt.subplot(subplt)
    plt.plot(x_axis,df.X)
    plt.plot(x_axis,df.Y)
    plt.plot(x_axis,df.Z)
    plt.xlabel('#samples')
    plt.ylabel(y_lable,fontsize=13)
    plt.title(titlePlot)
    plt.legend()

if __name__ == '__main__':

    df_linacc = pd.read_csv('lin_acc.csv',names=['X','Y','Z'], header=0,decimal=',') 
    df_rot = pd.read_csv('orientation.csv',names=['X','Y','Z'],header=0,decimal=',')
    df_angVel = pd.read_csv('angVel.csv',names=['X','Y','Z'],header=0,decimal=',')
    
    # initialize x axis using number of rows of datas
    t=len(df_linacc.X)
    x_axis = [1] * t
    for i in range(0,t):
        x_axis[i] = i

    #plots data 
    plt.figure(1)
    temp = df_linacc.astype(float)
    plotDatas(temp,311,x_axis,'linear acceleration','Estimated Linear Acceleration without gravity')

    temp = df_rot.astype(float)
    plotDatas(temp,312,x_axis,'Orientation','Orientation angles (yaw, pitch, roll)')

    temp = df_angVel.astype(float)
    plotDatas(temp,313,x_axis,'angular vel','Angular velocities')
    plt.show()
