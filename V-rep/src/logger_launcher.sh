#!/bin/bash
#launcher for the logger node in order to start the logger file in the correct position
cd ~
path2=$(find . -name baxter_scene -print 2>/dev/null | grep 'src/SofAr-project/V-rep/baxter_scene')
cd $path2
echo $path2
rosrun V-rep logger
