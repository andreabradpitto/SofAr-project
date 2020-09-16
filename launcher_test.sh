#!/bin/bash


#launch mathematica weighte
gnome-terminal --title="title1" -x sh -c "rosrun math_pkg weighter2; bash"

#launch mathematica forward kinematic
gnome-terminal --title="title2" -x sh -c "rosrun math_pkg Forward_Kine.py; bash"

#launch coppelia enviroment
roslaunch V-rep baxter_sim.launch
kill $(ps aux | grep "sh -c rosrun" | tr -s ' '| cut -d ' ' -f 2)



