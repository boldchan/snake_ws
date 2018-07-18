#!/bin/bash

# This bash file executes the steps needed to set up environment for running a wheel less snake robot in gazebo
# Run as . ./wheel_less.sh (basically source it and not run it)
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
cd src/snake_control/scripts/
sudo chmod +x gaits.py

# Run this command to launch gazebo-
roslaunch snake_control gazebo.launch gait:=true paused:=false
