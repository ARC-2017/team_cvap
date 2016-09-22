# apc_perception
Perception system APC 2016

Maintainer:   Sergio <caccamo@kth.se>, Rares <raambrus@kth.se>, Silvia <cruciani@kth.se>, Joao <jfpbdc@kth.se>, Xi <xi8@kth.se>, Ramviyas <ramviyas@kth.se>

This README concerns the problem of Perception for KTH APC'16.


Contents
=========

# perception_tf_frames

Node which reads a transform (from extrinsic calibration) from a file and publishes it between a source frame and target frame. 


# apc_objects_detection_action

Node which provides a service (perception server) for the High Level bt. Given a set of target objects the node provides stable tfs and objects info (e.g. occlusions)

# rgb_server

Node that creates a RGBD server for the perception node. It uses a RGBD camera and basic pcl algorithms to create tf of target objects. Used to complement the perception node on some difficult texture-less objects.

# apc_pc_server

Coming soon..


## Limitations
1. PC not integrated 
2. Poorly tested
3. No occlusions
4. Few objects
