#!/bin/bash

#gnome-terminal -x sh -c "roscore; bash"

#launch mathematica
gnome-terminal -x sh -c "roslaunch --wait math_pkg mathAll_halfcircle.launch; bash"

cd Math/math_pkg/scripts/halfcircle_files

gnome-terminal -x sh -c "rosrun math_pkg test_publisher_halfcircle.py; bash"

#launch coppelia enviroment
roslaunch V-rep baxter_sim.launch



kill $(ps aux | grep "sh -c rosrun" | tr -s ' '| cut -d ' ' -f 2)
kill $(ps aux | grep "sh -c roslaunch" | tr -s ' '| cut -d ' ' -f 2)
kill $(ps aux | grep "sh -c roscore" | tr -s ' '| cut -d ' ' -f 2)
