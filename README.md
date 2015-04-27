# Amazon Challenge Motion package #

BT nodes for executing motion plans on the PR2 for the Amazon Challenge.

## Arm position server ##

BT node that initializes the arm position depending on the bin we will grasp from

```
#!python

roslaunch amazon_challenge_motion arm_position_server.launch
```


## Launching MoveIt with shelf in planning scene ##

```
#!python

roslaunch amazon_challenge_motion moveit.launch
```

## PR2 MoveIt simulation ##

```
#!python

roslaunch amazon_challenge_motion shelf_planning_scene_sim.launch
```

## Launching nodes for initial shelf calibration ## 

```
#!python

roslaunch amazon_challenge_motion shelf_calibration.launch
```


## Saving joint position waypoints ##

Make sure you have downloaded [Francisco's branch of the moveit_commander package](https://github.com/fevb/moveit_commander/tree/hydro-devel-normalize-angles) (it is also in the **.rosinstall** file of the **amazon_challenge_install** package.


```
#!python

rosrun moveit_commander moveit_commander_cmdline.py
```

This will bring up an interactive command line

Then select e.g. the **left_arm** group, record different joint positions in variables **bin_A**, **bin_B** and **bin_C** and save all these poses to a text file:
```
#!python

use left_arm
record bin_A
record bin_B
record bin_C
save left_arm_bin_A_B_C.txt
```

You can also use several different groups, such as the right_arm, torso, etc.

Then you can copy-paste the joint poses to the **.yaml** files in the **./config** folder of this package.


## Loading/reading pose dictionaries ##

Use the [ROS parameter server](http://wiki.ros.org/rosparam) and the [python interface](http://wiki.ros.org/rospy/Overview/Parameter%20Server).

The dictionaries are in the **./config/** folder. You can load them via roslaunch files (see for example  **launch/arm_position_server.launch**) or via the command line (make sure you have a running ROS core):


```
#!python

rosparam load left_arm_joint_pose_dict.yaml
```

Then read the dictionary in python like this:

```
#!python
import rospy
...
left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')

q = left_arm_joint_pos_dict['start']
q2 = left_arm_joint_pos_dict['bin_A_start']

```