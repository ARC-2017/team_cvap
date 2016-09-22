# apc_objects_detection_action
Perception server for KTH APC'16

Maintainer:   Sergio Caccamo <caccamo@kth.se>

Contents
=========

Node which provides a service for the HL bt . Given a set of target objects the node provides stable tfs and objects info (e.g. occlusions)

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
$ roslaunch apc_objects_detection_action apc_objects_detection_server.launch 
```

 5. Run the client 
  ```
$ rosrun apc_objects_detection_action apc_objects_detection_client "name_object_to_detect"  "bin_ID" "camera_ID"(e.g. by default is crayola_24_ct, bin_H , kinect_chest and kinect_head depending on the bin) 
 ```
 
Look at the tfs. Enjoy it.


## Suggestions
1. To test the rgbd server :
  ```
$ rosrun apc_objects_detection_action apc_objects_detection_client "cherokee_easy_tee_shirt"  "bin_ID" "camera_ID"(or "womens_knit_gloves") 
 ```

2. To test the texture server :
  ```
$ rosrun apc_objects_detection_action apc_objects_detection_client "crayola_24_ct"  "bin_ID" "camera_ID" (or "scotch_duct_tape") 
 ```

3. You can run the servers alone and bypass the main server :
  ```
$ rosrun apc_objects_detection_texture texture_client "crayola_24_ct"  "bin_H"  
 ```
 
4. Test all the servers with the actionLib xclient
  ```
$ rosrun actionlib axclient.py /lookforobject 
 ```


## Args
1. Enable debug mode
2. Enable baxter face control 
3. Camera topic
