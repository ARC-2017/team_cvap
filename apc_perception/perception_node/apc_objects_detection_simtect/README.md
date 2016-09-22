# apc_objects_detection_texture
Skeleton for APC perception simtect segmentation server- development repository

Maintainer:   Sergio Caccamo <caccamo@kth.se> , Rares Ambrus <raambrus@kth.se>


Contents
=========

Node which provides a service for the perception server . Given a set of target objects the node provides stable tfs and objects info (e.g. occlusions) using simtrack

## Requirements
* simtrack 
* RGB camera sensor (e.g. Kinect2 or IDS camera)

## To start example:
 1. roscore
 2. run simtrack + ids camera or kinect2:
 
 ```
(with kinect2)
$ roslaunch simtrack_nodes main_kinect2.launch 
(with kinect2, on a new terminal)
$ roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
or (use a rosbag)
```
 3. rviz (with tf):
 
 ```
$ rviz 
```

 4. run the server
 ```
$ roslaunch apc_objects_detection_texture texture_server.launch 
```

 5. Run the client 
  ```
$ rosrun apc_objects_detection_texture texture_client "name_object_to_detect"  "bin_ID" (e.g. by default is crayola_24_ct, bin_H) 
 ```
 
Look at the tfs. Enjoy it.
 
## Args
1. Enable debug mode
2. Enable baxter face control 
3. Camera topic
