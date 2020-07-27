#!/usr/bin/env python

"""
Documentation for the offline analysis tool
This piece of code is mostly used to run tests relative to the incoming imu data
and filtering/transformation performed in 'remove_gravity.py'
"""

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import os

script_dir = os.path.dirname(__file__)  # absolute directory the script is in
rel_path1 = "../output/lin_acc.csv"
abs_file_path1 = os.path.join(script_dir, rel_path1)
rel_path2 = "../output/orientation.csv"
abs_file_path2 = os.path.join(script_dir, rel_path2)
rel_path3 = "../output/angVel.csv"
abs_file_path3 = os.path.join(script_dir, rel_path3)

font = {'family': 'serif',
        'color': 'darkred',
        'weight': 'normal',
        'size': 12,
        }


def plotData(df, subplt, x_axis, y_lable, titlePlot):
    """!
    This function simply plots data into a a graph
    @param df specifies the data set (orientation/linear acceleration/angular velocity) to be plotted
    @param subplt this specifies the position of the plot in the figure
    @param x_axis ascending numbers (i.e. data samples)
    @param y_lable data type and measurement unit
    @param titlePlot title of the plot
    """
    plt.subplot(subplt)
    plt.plot(x_axis, df.X)
    plt.plot(x_axis, df.Y)
    plt.plot(x_axis, df.Z)
    plt.xlabel('#samples', fontdict=font)
    plt.ylabel(y_lable, fontdict=font)
    plt.title(titlePlot, fontdict=font)
    plt.legend()


if __name__ == '__main__':
    """!
    Main function. It reads data from .csv files and plots it by using plotData()
    """

    df_linacc = pd.read_csv(abs_file_path1, names=[
                            'X', 'Y', 'Z'], header=0, decimal=',')
    df_rot = pd.read_csv(abs_file_path2, names=[
                         'X', 'Y', 'Z'], header=0, decimal=',')
    df_angVel = pd.read_csv(abs_file_path3, names=[
                            'X', 'Y', 'Z'], header=0, decimal=',')

    # initialize x axis using number of rows of data
    t = len(df_linacc.X)
    x_axis = np.arange(start=1, stop=t + 1, step=1)

    # plots data
    plt.figure(figsize=(12, 12))
    temp = df_linacc.astype(float)
    plotData(temp, 311, x_axis,
             'Filtered linear acceleration [m/s^2]', 'Linear Acceleration')

    temp = df_rot.astype(float)
    plotData(temp, 312, x_axis,
             'Filtered Orientation [deg]', 'Orientation angles (yaw, pitch, roll)')

    temp = df_angVel.astype(float)
    plotData(temp, 313, x_axis,
             'Filtered angular vel [rad/s]', 'Angular velocities')

    plt.show()
