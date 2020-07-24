#!/bin/bash

cd ~
a=$(locate -b V-rep/src/baxter_scene | head -n 1)
cd $a
rosrun V-rep logger
echo $a
