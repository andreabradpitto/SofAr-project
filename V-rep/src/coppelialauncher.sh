#!/bin/bash
cd ~
path1=$(locate -b coppeliaSim.sh | head -n 1)
path2=$(locate -b baxter_scene | head -n 1)
$path1 $path2/baxter_sofar.ttt
