#!/bin/bash
cd ~
path1=$(locate -b coppeliaSim.sh | head -n 1)
path2=$(locate -b baxter_scene | head -n 1)
cd $path2
pwd
#path2=$(locate -b baxter_sofar.ttt | head -n 1)
echo $path1
echo $path2

$path1 $path2/baxter_sofar.ttt
