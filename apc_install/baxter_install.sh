#!/bin/bash

cp apc.rosinstall ~/

# for installing baxter dependencies

sudo apt-get install python-rosinstall python-pip libspatialindex* meshlab
sudo apt-get update

sudo pip install rtree

sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers ros-indigo-moveit* ros-indigo-openni* ros-indigo-cmake-modules



# baxter gazebo

sudo apt-get install gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser

cd ~/

wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init

# baxter stuff
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall

# baxter gazebo
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall

# NB: make sure to say yes when prompted about modifying baxter_interface!
wstool merge ~/apc.rosinstall

wstool update


cd ~/catkin_ws
catkin_make
catkin_make install


# still have to install freenect2
