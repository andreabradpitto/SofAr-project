# Smartphone subproject readme

## Authors

Matteo Dicenzi, Marco Demutti, Andrea Pitto, Simone Voto

# Summary

This project handles incoming Imu sensor data from an Android smartphone and performs a gravity removal. It then publishes on the *smartphone* topic the filtered result.

## How to install the package

1. Unzip *org.ros.android.android_tutorial_camera_imu_1.0.apk* in order to install the *CameraImu* app in your smartphone. Warning: the app works better with Android 8.1+; earlier os version cause frequent freezes/crashes

2. Install the following libraries: *numpy*, *matplotlib*, *pandas* (you can use **pip list** in terminal beforehand to check if these are already installed):

   1. Type **python -m pip install numpy** in terminal to install numpy. Check [SciPy.org](https://scipy.org/install.html) for more information.
   2. Type **python -m pip install -U matplotlib** in terminal to install matplotlib. Head over to [matplotlib.org](https://matplotlib.org/3.3.0/users/installing.html) for further information.

3. Type **sudo apt-get install python-pandas**. In case of errors, go [here](https://pandas.pydata.org/pandas-docs/version/0.13.1/install.html) for more information.


## How to run the package
1. Launch ROS by typing **roscore** in a terminal.

2. Start the *CameraImu* app in your smartphone. Then you have to assign the IP address of your computer into the app's *Master URI* textbox (i.e. http://YOUR_IP_ADDRESS:11311). In order to find your IP address, type **hostname -I** in terminal if you are using a Unix system, or type **ipconfig** in command prompt if using Windows.

3. In a second terminal, subscribe to the *android/imu* topic by typing **rostopic echo /android/imu**.

4. In a third terminal, aunch **python remove_gravity.py** to start collecting imu data and removing the gravity from incoming linear acceleration samples.
   - Remember to set *flagWriteData = 1* in the python file if you want to analyze incoming data through plots. Otherwise set *flagWriteData = 0* to skip the offline analysis.

5. If you set *flagWriteData = 1* (see previous step), also run **offlineAnalysis.py** in a fourth terminal, in order to plot the data obtained by running 'remove_gravity.py'
