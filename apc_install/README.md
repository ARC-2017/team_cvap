# apc_install
Install scripts and rosinstall files for APC 2016.

It includes Baxter repositories and our own APC repositories

## Download git repostories

To download all repositories to your workspace (make sure you have wstool [wstool](http://wiki.ros.org/wstool) installed!):

```
  cp ./apc.rosinstall ~/catkin_ws/src/
  cd ~/catkin_ws/src/
  wstool init
  wstool merge apc.rosinstall 
  wstool update
  cd ~/catkin_ws
  catkin_make install

```

Remember to source in your **.bashrc** your catkin workspace:

```
  echo "source ~/catkin_ws/devel/setup.bash">> ~/.bashrc
```

## Install baxter debian dependencies + git repositories

Installs git repositories in **~/catkin_ws**

**WARNING: IT MAY OVERWRITE STUFF IN YOUR CATKIN WS!!!**

```
  ./baxter_install.sh

```

** NB: make sure you say YES when it prompts to override baxter_interface when merging the rosinstall file! **

This also installs the **baxter.sh** script in your home folder


## Full ROS + baxter install

In case of a machine without ROS installed run the following script for full installation of ROS + baxter dependencies + baxter SDK.

**WARNING: IT MAY OVERWRITE STUFF IN YOUR CATKIN WS!!!**

```
  ./ros_baxter_install.sh
```

It creates workspace, downloads all repos, etc.
