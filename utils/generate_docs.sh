#!/bin/bash
#
# This script generates documentation of all the packages in auv repo.
#
# debug layer
(cd ~/catkin_ws/src/auv/debug_layer/ &&
  rosdoc_lite remote_control -o ../docs/remote_control &&
  rosdoc_lite varun_description -o ../docs/varun_description &&
rosdoc_lite remote_gazebo -o ../docs/varun_gazebo )
# hardware_layer
(cd ~/catkin_ws/src/auv/hardware_layer/ &&
  rosdoc_lite hardware_arduino -o ../docs/hardware_arduino &&
  rosdoc_lite hardware_imu -o ../docs/hardware_imu &&
rosdoc_lite hardware_camera -o ../docs/hardware_camera )
# master_layer
(cd ~/catkin_ws/src/auv/master_layer/ &&
rosdoc_lite the_master -o ../docs/hardware_arduino )
# motion_library_layer
(cd ~/catkin_ws/src/auv/motion_library_layer/ &&
  rosdoc_lite motion_forward -o ../docs/motion_forward &&
  rosdoc_lite motion_upward -o ../docs/motion_upward &&
  rosdoc_lite motion_turn -o ../docs/motion_turn &&
  rosdoc_lite motion_commons -o ../docs/motion_commons &&
rosdoc_lite motion_sideward -o ../docs/motion_sideward )
# task_handler_layer
(cd ~/catkin_ws/src/auv/task_handler_layer/ &&
  rosdoc_lite task_buoy -o ../docs/task_buoy &&
  rosdoc_lite task_line_detection -o ../docs/task_line_detection &&
  rosdoc_lite task_line_following -o ../docs/task_line_following &&
rosdoc_lite task_commons -o ../docs/task_commons )
