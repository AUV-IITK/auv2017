#!/bin/bash
#
# This script compiles all the packages in auv repo.
# Only those packages that should not be build on odroid are build here; rest all go to odroid_build.sh
#
# change dir to workspace
(cd ~/catkin_ws &&
  ./src/auv/utils/odroid_build.sh &&
  # build debug layer
  catkin_make --pkg varun_description &&
  catkin_make --pkg varun_gazebo &&
catkin_make roslint_varun_gazebo)
