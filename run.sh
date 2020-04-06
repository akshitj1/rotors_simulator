#!/bin/bash
set -xv
if [ "$1" = "dbg" ]; then
    ARGS="debug:=true gui:=false paused:=false"
fi
roslaunch rotors_gazebo tailsitter.launch $ARGS
