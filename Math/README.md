## Math node

To use this package:
1) Download the math_pkg folder.
2) Put it in ```catkin_ws/src```, where ```catkin_ws``` is the absolute path to your catkin workspace.
3) In the catkin folder, be sure to source your setup.bash file with the command  
```$ source devel/setup.bash```  
4) Make all the Python files executable by running    
```$ a=$(find "$(pwd)" -name  *.py )```  
```$ chmod +x $a```
5) In the catkin folder, compile the package by running  
```$ catkin_make```  
6) In the catkin folder, launch all the math_pkg nodes by executing  
```$ roslaunch math_pkg mathAll.launch```  
7) Run a publisher node (for example, ```test_publisher_halfcircle.py``` is provided in the package).
