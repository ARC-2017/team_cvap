#!/bin/bash

# Install ROS

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-indigo-desktop-full

sudo rosdep init

rosdep update

sudo apt-get install python-rosinstall

mkdir -p ~/catkin_ws/src

source /opt/ros/indigo/setup.bash

cd ~/catkin_ws

catkin_make
catkin_make install

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# install baxter dependencies

bash baxter_install.sh

