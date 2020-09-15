#!/bin/bash
#launcher for the logger node in order to start the logger file in the correct position
cd ~
a=$(find . -name baxter_scene -print 2>/dev/null | grep 'src/V-rep/baxter_scene')
cd $a
echo $a
rosrun V-rep logger
