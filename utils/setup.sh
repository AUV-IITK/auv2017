#!/bin/bash
#
# This script downloads and installs all the dependencies of packages in auv repo.
#
ROS_DISTRO=$'indigo'

STR=$'This script does not install ros
Please refer to https://github.com/AUV-IITK/AUVWiki/wiki for recommended version of ros'
echo "$STR"

echo "installing required packages"
sudo apt-get update
sudo apt-get install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
# installing for syntax check
sudo apt-get install python-pip
sudo pip install autopep8
sudo apt-get install clang-format-3.6
# source ros setup script
source /opt/ros/$ROS_DISTRO/setup.bash
# Prepare rosdep to install dependencies.
cd ~/catkin_ws/src
rosdep update
wstool init
if [[ -f $ROSINSTALL_FILE ]] ; then wstool merge $ROSINSTALL_FILE ; fi
wstool up

echo "Installing dependencies"
sudo apt-get install -y \
ros-$ROS_DISTRO-vision-opencv \
libopencv-dev \
ros-$ROS_DISTRO-rosbash \
ros-$ROS_DISTRO-rosserial-arduino \
ros-$ROS_DISTRO-rosserial \
ros-$ROS_DISTRO-convex-decomposition \
ros-$ROS_DISTRO-ivcon \
ros-$ROS_DISTRO-pr2-description \
ros-$ROS_DISTRO-actionlib \
ros-$ROS_DISTRO-dynamic-reconfigure \
ros-$ROS_DISTRO-image-transport \
ros-$ROS_DISTRO-gazebo-ros \
ros-$ROS_DISTRO-roslint \
doxygen \
doxypy \
expect \
ros-$ROS_DISTRO-rosdoc-lite
source /opt/ros/$ROS_DISTRO/setup.bash

# package depdencies: install using rosdep.
cd ~/catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# setup rosserial arduino
source /opt/ros/$ROS_DISTRO/setup.bash
echo "setting up rosserial_arduino"
mkdir -p ~/sketchbook/libraries
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

# setup pressure sensor library
cd ~/sketchbook/libraries
git clone https://github.com/bluerobotics/BlueRobotics_MS5837_Library.git

# udev rules setup for imu
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{serial}=="0", ATTRS{idProduct}=="0010", SYMLINK+="navstik"' > /etc/udev/rules.d/99-imu-serial.rules

# udev rules setup for arduino
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino"' > /etc/udev/rules.d/100-arduino-serial.rules

# suggest to setup environment variables
echo "Please add these to your ~/.bashrc file"
echo "source /opt/ros/$ROS_DISTRO/setup.sh"
echo "source /usr/share/gazebo-7/setup.sh"
echo "For using gazebosim on your system plz install by running #curl -ssL http://get.gazebosim.org | sh"
echo "and configure these settings #export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH/home/$USER/catkin_ws/src/auv/debug_layer/:/home/$USER/catkin_ws/src/auv/debug_layer/varun_gazebo/models/:"
