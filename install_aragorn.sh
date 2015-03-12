#!/bin/bash

source /opt/ros/hydro/setup.bash
sudo apt-get install ros-hydro-moveit* 

WS=$HOME/amazon_challenge_ws

# create catkin workspace
# and install

mkdir -p $WS/src
cp amazon_challenge.rosinstall $WS/src/
cd $WS/src

catkin_init_workspace
wstool init

cd $WS
catkin_make

echo "source ~/amazon_challenge_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://pr2-c1:11311" >> ~/.bashrc
echo "export ROS_IP=130.237.218.145" >> ~/.bashrc
source ~/.bashrc

cd $WS/src

wstool merge amazon_challenge.rosinstall
wstool update

cd $WS
catkin_make




