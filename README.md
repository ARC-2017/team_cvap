# Amazon Challenge Motion package #

BT nodes for executing motion plans on the PR2 for the Amazon Challenge.

## Loading/reading pose dictionaries ##

Use the [ROS parameter server](http://wiki.ros.org/rosparam) and the [python interface](http://wiki.ros.org/rospy/Overview/Parameter%20Server).

The dictionaries are in the **./config/** folder. You can load them via roslaunch files (see **launch/pose_intializer.launch**) or via the command line (make sure you have a running ROS core):


```
#!python

rosparam load left_arm_joint_pose_dict
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


