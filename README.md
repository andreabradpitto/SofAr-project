To use this package:
1) Download the math_pkg folder.
2) Put it in ```catkin_ws/src```, where ```catkin_ws``` is the absolute path to your catkin workspace.
3) In the catkin folder, be sure to source your setup.bash file with the command  
```$ source devel/setup.bash```  
4) In the catkin folder, run:  
```$ cd src/math_pkg/```  
```$ ./startMath.bash```  
5) In the catkin folder, compile the package by running  
```$ catkin_make```  
6) If you have the V-Rep package installed, go to ```src/math_pkg/launch``` and comment or delete the row of the last one (it is a node that simulates V-Rep).
7) In the catkin folder, launch all the math_pkg nodes by executing  
```$ roslaunch math_pkg mathAll.launch```  
8) Enjoy your inverse kinematics!