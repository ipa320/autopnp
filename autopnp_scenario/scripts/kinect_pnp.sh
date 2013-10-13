#!/bin/bash
# old openni driver
# source /opt/ros/groovy/setup.bash
# roslaunch openni_launch openni.launch camera:=cam3d depth_registration:=true

# modern openni2 driver
source /home/rbormann/git/care-o-bot_catkin/devel/setup.bash
roslaunch openni2_launch openni2.launch camera:=cam3d depth_registration:=true

# usage
# -----
# 1. create udev rule on pc 2 of the robot
# sudo gedit /etc/udev/rules.d/01-kinect.rules
# -> insert the following line into the file: SUBSYSTEMS=="usb", ATTRS{idVendor}=="1d27", MODE="0666", SYMLINK+="kinect", RUN+="/home/rbormann/git/autopnp/autopnp_scenario/scripts/kinect_pnp.sh"
# -> adapt path for the rule
# -> save and close
# 2. adapt path in this file for loading the driver
# 3. update rules
# sudo udevadm control --reload-rules
# 4. try out pnp
# -> if it works, you will find a kinect device for: ls -l /dev/kinect
