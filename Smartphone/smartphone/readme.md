# Smartphone node

## Authors

[Matteo Dicenzi](https://github.com/mattedicenzi), [Marco Demutti](https://github.com/marcodemutti), [Andrea Pitto](https://github.com/andreabradpitto), [Simone Voto](https://github.com/Cavalletta98)

The *imu_calib* package work was entirely made by [Daniel Koch](https://github.com/dpkoch); the original version can be found [here](https://github.com/dpkoch/imu_calib)

## Summary

This project handles incoming IMU sensor data from an Android smartphone and performs data clipping and gravity influence removal. It then publishes on the *smartphone* topic the filtered result.

## How to install the package

1. Unzip *org.ros.android.android_tutorial_camera_imu_1.0.apk* in order to install the *CameraImu* app in your smartphone. Warning: the app works best with Android 8.1 or older; earlier OS versions may cause frequent freezes/crashes

2. Install the following libraries: *numpy*, *matplotlib*, *pandas* (you can use **pip list** in terminal, beforehand, to check if these are already installed):

   1. Type **python -m pip install numpy** in terminal to install numpy. Check [SciPy.org](https://scipy.org/install.html) for more information.
   2. Type **python -m pip install -U matplotlib** in terminal to install matplotlib. Head over to [matplotlib.org](https://matplotlib.org/3.3.0/users/installing.html) for further information.
   3. Type **sudo apt-get install python-pandas**. In case of errors, you can take a look [here](https://pandas.pydata.org/pandas-docs/version/0.13.1/install.html) for more information.

## How to run the package

1. Launch ROS by typing **roscore** in a terminal.

2. Start the *CameraImu* app in your smartphone. Then you have to type the IP address of your computer into the app's *Master URI* textbox (i.e. http://YOUR_IP_ADDRESS:11311, as the default port is *11311*). In order to find your IP address, type **hostname -I** in terminal if you are using a Unix system, or type **ipconfig** in command prompt if using Windows.
   - It is possible to open another terminal and subscribe to the *android/imu* topic, by typing **rostopic echo /android/imu**, in order to check the incoming data.

3. In a second terminal, type **rosrun imu_calib do_calib** and start calibrating the smartphone IMU, following on-screen instructions. Then, apply the performed calibration by using **rosrun imu_calib apply_calib**. The calibration parameters are stored in *imu_calib.yaml*, available in the current user's home folder.
   - Again, it is possible to open another terminal and type **rostopic echo /android/imu_corrected**, in order to view the incoming calibrated data.

4. In a third terminal, type **rosrun smartphone computeGravity.py** to start collecting IMU data and removing the gravity influence from incoming linear acceleration samples. Notice that this step is needed only if executing the whole *SofAr-project*. Otherwise, gravity gets already neglected in the next step.

5. In a fourth terminal, launch **rosrun smartphone clipping.py** to start clipping IMU data, when needed, and removing the gravity influence from incoming linear acceleration samples.
   - Remember to set *flagWriteData = 1* in the python file if you want to analyze incoming data through plots. Otherwise set *flagWriteData = 0* to skip the offline analysis. If *flagWriteData* is set to 1, then also run **rosrun smartphone offlineAnalysis.py** in yet another terminal, in order to plot the data obtained by running *clipping.py*
