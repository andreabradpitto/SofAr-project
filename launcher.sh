#!/bin/bash


#launch smartphone calibration
echo "Place the phone on an even horizontal surface"
echo "Press enter to proceed"
read a
gnome-terminal -x sh -c "rostopic echo smartphone; bash"
echo "If you want to launch the sensor calibration process type 'yes' and press enter"
read b 
if [[ $b == "yes" ]] ; then 
	rosrun imu_calib do_calib
fi
gnome-terminal -x sh -c "rosrun imu_calib apply_calib; bash"
rosrun smartphone removeGravity.py
echo "With your hand, hold the phone in the calibration position and mantain the position"
echo "Press enter to proceed"
read a
gnome-terminal -x sh -c "rosrun smartphone clipping.py; bash"
#launch mathematica
gnome-terminal -x sh -c "roslaunch --wait math_pkg mathAll.launch; bash"

#launch coppelia enviroment
#roslaunch V-rep baxter_sim.launch

kill $(ps aux | grep "sh -c rosrun" | tr -s ' '| cut -d ' ' -f 2)
kill $(ps aux | grep "sh -c roslaunch" | tr -s ' '| cut -d ' ' -f 2)


