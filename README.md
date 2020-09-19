# SofAr-project

<p align="center">
  <img height="500" width="500" src="https://github.com/andreabradpitto/SofAr-project/blob/master/Images%20and%20multimedia/Image.jpeg?raw=true "Title"">
</p>

The project's goal was to design and implement a software component for the teleoperation of the Baxter robot simulation in Coppelia. The teleoperation works as follows: the human operator moves its arm keeping a smartphone into its hand, and the sensor data from the smartphone's IMU is sent to the software and used to allow Baxter's end-effector to follow the trajectory and orientation of the human hand.  
The project's original goal was actually to not only track the end-effector's configuration, but also to replicate the motion of the human arm into Baxter's, using Mocap technology; this idea, as well as the objective of using the software on the real robot, were later rejected due to the Covid emergence and the consequent impossibility of access the EMARO Lab.  
Unfortunately, the elimination of the position measurement with the motion capture do not allow the perfect tracking, but the developed modules try to work inside this limitation.  

### Prerequisites

In order to run this software, the following prerequisites are needed:  
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu),  
- [CoppeliaSim Edu V4](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm) (which has to be linked with ROS),  
- [Ubuntu 16.04](https://releases.ubuntu.com/16.04/)(Other Ubuntu versions may work, but this is the offcialy supported one by ROS kinetic, as well as the one on which all this code was produced.)  

Then, it is required to install the app on an Android mobile phone. Unzip *org.ros.android.android_tutorial_camera_imu_1.0.apk* in order to install the *CameraImu* app in your smartphone. Warning: the app works best with Android 8.1 or older; earlier OS versions may cause frequent freezes/crashes  
Moreover, launching the software on a virtual machine cause great instability, so, it is strongly adviced against.  
In the testing phase, the following hardware characteristic were found to work discretely, which is why they are going to be taken as advised configuration.  
Characteristics:  
- i5 processors, 3.1GHz, 2 cores
- 8GB DDR4 SDRAM
- 256 GB SSD memory
- 103.5GB dedicated to Ubuntu ( in testing 39.2GB of memory were used)


### Installing

In order to have a working version of this package running on your computer, you need to:  
- Place the package in the src folder of your src foulder of the catkin workspace, and having it named "SofAr-project"
- In case the folder "SofAr-project/Math/imu_calib" is empty, you can solve it with the same code
```sh
cd SofAr-project/Smartphone
rm imu_calib
git clone https://github.com/dpkoch/imu_calib
```
- Have the Coppelia environment foulder in any place under the HOME directory (it is advised to put it on the Desktop or directly in HOME)
- If you don't have the following libraries installed on your system procede with this code
```sh
python -m pip install numpy
python -m pip install -U matplotlib
sudo apt-get install python-pandas
sudo apt-get install python-scipy
sudo apt-get install ros-kinetic-cmake-modules
```
- Change the current directory to "SofAr-project" and activate the install.sh script
```sh
cd "Your catkin workspace"/src/SofAr-project
chmod +x install.sh
```
- Install the package via dedicated script
```sh
./install.sh
```
### Running the tests: running the simulation without the smartphone to inspect that the simulation is working

In order to see a test of the working system (without the smartphone application sending signals), proceed with the following commands
```sh
cd "Your catkin workspace"/src/SofAr-project
./launcher_test.sh
```
When all components are open, find the user interface (the terminal where you have launched the system and the only one asking for the user input).  
There you can type "help" and enter to obtain the command list, or you can type "start" and enter to start the animation.  
At any moment, you can type "pause" or "stop" to interrupt the simulation and respectively stay in the position or return to the starting one.  
When stopped (not paused) you can type "set_default" to set the starting configuration.  
Once you are done, you can type "exit" and press enter to close all this project components.  
The movement that should be obtained in this test is the increment of the first joint angular position, as can be seen in the animation.  
<div align="center">
  <img height="500" width="500" src="https://github.com/andreabradpitto/SofAr-project/blob/master/Images%20and%20multimedia/Animated%20GIF-downsized_large.gif">
</div>
 
### Deployment
Here we are going to present how to use the software to attempt a tracking of the user arm using the data from the smartphone sensor.  
Note: when you are asked to take the phone in your hand during the procedures, you are to take it in your right hand, arm stretched, horizontal and at more or less 45 degrees with respect to the plane of the chest, with the smartphone screen vertical, facing you, and the camera in the direction of the thumb. When you give the command "calibration", you can take it as you are most comfortable.    
To use the whole system at once, the following action have to take place:  
- Start the smartphone application
- Verify the IP address to be inserted in the app
```sh
ifconfig wlan0 | grep "inet " | awk -F'[: ]+' '{ print $4 }'
```
- Start the software on the computer with the following commands
```sh
cd "Your catkin workspace"/src/SofAr-project
./launcher.sh
```
- Find the terminal in which you started the software and follow the instructions written on it always checking the app is working properly (you can look at the terminal printing the linear acceleration, orientation and angular velocity)
- Once all the instructions are completed and the Coppelia environment opened, with the smartphone hand still type "calibration" and press enter  
- When you are done you can type "start" and procede with the movement

After the command "calibration", you can apply every other string presented in the section "Running test".  
When you type start, be sure you are more or less in the same configuration as baxter.

### Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

### Authors

[Marco Demutti](https://github.com/marcodemutti), [Matteo Dicenzi](https://github.com/mattedicenzi), [Vincenzo Di Pentima](https://github.com/VinDp), [Elena Merlo](https://github.com/RobElena), [Matteo Palmas](https://github.com/Matt98x), [Andrea Pitto](https://github.com/andreabradpitto), [Emanuele Rosi](https://github.com/emanuelericcardo), [Chiara Saporetti](https://github.com/ChiaraSapo), [Giulia Scorza Azzarà](https://github.com/Giulia24091997), [Luca Tarasi](https://github.com/LucaTars), [Simone Voto](https://github.com/Cavalletta98), [Gerald Xhaferaj](https://github.com/Geraldone)


### License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
