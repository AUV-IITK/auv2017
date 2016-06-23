#!/bin/bash
#
# This script compiles all the packages in auv repo.
#
# compiling individual pkgs is prefered because it helps maintain a list of all the pkgs
# xyz_commons folder contain actionlib files on which other pkgs depend, so they are
# compiled first.
# change dir to workspace
(cd ~/catkin_ws &&
  # build action lib header files
  catkin_make --pkg motion_commons &&
  catkin_make --pkg task_commons &&
  # building rest of the pkgs and lint checking
  # motion library
  catkin_make --pkg motion_forward &&
  catkin_make roslint_motion_forward &&
  catkin_make --pkg motion_sideward &&
  catkin_make roslint_motion_sideward &&
  catkin_make --pkg motion_turn &&
  catkin_make roslint_motion_turn &&
  catkin_make --pkg motion_upward &&
  catkin_make roslint_motion_upward &&
  # task handlers
  catkin_make --pkg task_buoy &&
  catkin_make roslint_task_buoy &&
  catkin_make --pkg task_line_detection &&
  catkin_make roslint_task_line_detection &&
  catkin_make --pkg task_line_following &&
  catkin_make roslint_task_line_following &&
  # build master layer
  catkin_make --pkg the_master &&
  catkin_make roslint_the_master &&
  # build hardware layer
  catkin_make --pkg hardware_arduino &&
  catkin_make roslint_hardware_arduino &&
  catkin_make --pkg hardware_camera &&
  catkin_make roslint_hardware_camera &&
  catkin_make --pkg hardware_imu &&
  catkin_make roslint_hardware_imu &&
  # build debug layer
  catkin_make --pkg varun_description &&
  catkin_make --pkg varun_gazebo &&
  catkin_make roslint_varun_gazebo &&
  catkin_make --pkg remote_control &&
catkin_make roslint_remote_control)
