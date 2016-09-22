# apc_perception_development
Skeleton for APC perception RGBD segmentation server- development repository


PCL module for APC 16 Development

Maintainer:   Sergio Caccamo <caccamo@kth.se>

Based on the following documentations:
http://wiki.ros.org/pcl/Tutorials

Contents
=========

## Requirements
* PCL 1.7
* Eigen
* RGBD sensor (e.g. Kinect2 or Openni2 compatible camera)

## To start example:
```
$ roslaunch openni2_launch openni2.launch
or (with kinect2)
$ roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
or (use a rosbag)

```
On a new terminal (from camera):
```
$ roslaunch apc_objects_detection_rgbd rgbd_server.launch 
```
(change camera topic and settings on the launch file)

see the output:
```
$ rviz 
(NB: load the view, kinect2_devel) or
$ rostopic echo output
```

Run the client:
```
$ rosrun apc_objects_detection_rgbd rgbd_client "name_object_to_detect" "bin_ID" (e.g. by default is womens_knit_gloves , bin_H)
```

## Args
1. Enable debug mode
2. Enable baxter face control 
3. Camera topic

## Tested with 
	objectslist_ = "cherokee_easy_tee_shirt";
	objectslist_ = "womens_knit_gloves";
	objectslist_ = "kyjen_squeakin_eggs_plush_puppies";
	objectslist_ = "fiskars_scissors_red";
	objectslist_ = "cloud_b_plush_bear";

## Limitations
 1. must be tested
