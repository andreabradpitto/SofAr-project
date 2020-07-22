#!/bin/bash

cd ~
a=$(locate -b baxter_scene | head -n 1)
cd $a
rosrun baxter logger
echo $a
