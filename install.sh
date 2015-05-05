#!/bin/bash
sudo apt-get install git emacs ssh

# install ROS hydro
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-hydro-desktop-full

# install ROS workspace

source /opt/ros/hydro/setup.bash
sudo apt-get install ros-hydro-moveit* ros-hydro-pr2* ros-hydro-gazebo* ros-hydro-cmake-modules python-pip
sudo easy_install wstool

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
# echo "export ROS_MASTER_URI=http://pr2-c1:11311" >> ~/.bashrc
# echo "export ROS_IP=130.237.218.145" >> ~/.bashrc
source ~/.bashrc

cd $WS/src

wstool merge amazon_challenge.rosinstall
wstool update

sudo rosdep init
rosdep update

cd $WS
rosdep install --from-paths src --ignore-src
catkin_make