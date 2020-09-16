#!/bin/bash


#launch mathematica
gnome-terminal -x sh -c "roslaunch --wait math_pkg mathAll.launch; bash"

#launch coppelia enviroment
#roslaunch V-rep baxter_sim.launch

kill $(ps aux | grep "sh -c rosrun" | tr -s ' '| cut -d ' ' -f 2)
kill $(ps aux | grep "sh -c roslaunch" | tr -s ' '| cut -d ' ' -f 2)
