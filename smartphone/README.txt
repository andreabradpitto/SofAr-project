How to install and run the package

-unzip 'org.ros.android.android_tutorial_camera_imu_1.0.apk' and install it in your smartphone.
 Be aware: you have to make sure that your smartphone has an android 8.1 or above.
 If the android version is not at least 8.1, you will have troubles and the package  will not work properly.

Then you have to install the following libraries: numpy, matplotlib, pandas.

-Type 'python -m pip install numpy' in the terminal to install numpy. Check https://scipy.org/install.html for more informations.
-Type 'python -m pip install -U matplotlib' in the terminal to install matplotlib.
-In case of errors, check https://matplotlib.org/3.3.0/users/installing.html for more informations.

-Type 'sudo apt-get install python-pandas'. In case of errors check: https://pandas.pydata.org/pandas-docs/version/0.13.1/install.html for more informations.


How to run the package
-Open ROS by typing 'roscore' in the terminal.

-Open the application in your smartphone. Then you have to assign the IP address of your computer into the available space in the application (i.e. http://YOUR_IP_ADDRESS:11311).
 In order to see your IP address, you can either type 'hostname -I' or you can go in 'connection'-> 'connection informations'-> copy the IPv4 address.

-Open 'remove_gravity.py' and set 'flagWriteData' = 1 if you want to analyze, through plots, the data offline (see the following optional part). 
 Otherwise you can set 'flagWriteData' = 0 and you will not make any analysis.

-Type 'python remove_gravity.py' to start collecting imu data and removing the gravity from your linear acceleration data.

-(optional) if you previously set 'flagWriteData' = 1,  by typing 'python offlineAnalysis.py' you can plot the data obtained running remove_gravity.py


