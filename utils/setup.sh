#!/bin/bash
#
# This script downloads and installs all the dependencies of packages in auv repo.
#
ROS_DISTRO=$'kinetic'

STR=$'This script does not install ros
Please refer to https://github.com/AUV-IITK/AUVWiki/wiki for recommended version of ros'
echo "$STR"

echo "installing required packages"
sudo apt-get update
sudo apt-get install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
# Prepare rosdep to install dependencies.
rosdep update

echo "Installing dependencies"
sudo apt-get install -y \
ros-$ROS_DISTRO-vision-opencv \
libopencv-dev \
ros-$ROS_DISTRO-rosserial-arduino \
ros-$ROS_DISTRO-rosserial \
ros-$ROS_DISTRO-convex-decomposition \
ros-$ROS_DISTRO-pr2-description \
ros-$ROS_DISTRO-actionlib \
ros-$ROS_DISTRO-dynamic-reconfigure \
ros-$ROS_DISTRO-image-transport \
ros-$ROS_DISTRO-gazebo-ros \
ros-$ROS_DISTRO-roslint
source /opt/ros/$ROS_DISTRO/setup.bash
# package depdencies: install using rosdep.
cd ~/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# setup rosserial arduino
echo "setting up rosserial_arduino"
mkdir -p ~/sketchbook/libraries
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

# suggest to setup environment variables
echo "Please add these to your ~/.bashrc file"
echo "source /opt/ros/kinetic/setup.sh"
echo "source /usr/share/gazebo-7/setup.sh"
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH/home/$USER/catkin_ws/src/:"