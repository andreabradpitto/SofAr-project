#!/bin/bash

cd ~
a=$(find . -name baxter_scene -print 2>/dev/null | grep 'src/V-rep/baxter_scene')
cd $a
echo $a
rosrun V-rep logger

