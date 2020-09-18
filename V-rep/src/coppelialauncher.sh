#!/bin/bash
#Launcher for the correct baxter scene
cd ~
path1=$(locate -b coppeliaSim.sh | head -n 1)
path2=$(find . -name baxter_sofar.ttt -print 2>/dev/null | grep 'src/teleop/V-rep/baxter_scene')
echo $path1
echo $path2
$path1 $path2
