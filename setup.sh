#!/bin/bash
# change dir to workspace
(cd ~/catkin_ws &&
# cleaning catkin workspace
catkin_make clean &&
# build action lib header files
catkin_make --pkg motion_actions &&
catkin_make --pkg task_commons &&
# building rest of the pkgs
# motion library
catkin_make --pkg motion_forward &&
catkin_make --pkg motion_sideward &&
catkin_make --pkg motion_turn &&
catkin_make --pkg motion_upward &&
# task handlers
catkin_make --pkg task_buoy &&
catkin_make --pkg linedetection &&
catkin_make --pkg linefollowing &&
# build master layer
catkin_make --pkg the_master $$
# build hardware layer
catkin_make --pkg hardware_arduino &&
catkin_make --pkg hardware_camera &&
catkin_make --pkg hardware_imu )