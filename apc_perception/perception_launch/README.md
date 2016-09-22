Perception launch
================

To launch the complete perception system execute:

```
roslaunch perception_launch perception.launch
```

This launches the following launch files:
* `perception_tf_frames.launch`
* `simtrack_demo1.launch`
* `perception_kinect_robot.launch`
 
**Note** if you want to use the IDS cameras, separately run:

```
roslaunch perception_launch perception_ids_robot.launch
```

**This has to be run before `perception.launch` (see https://gits-15.sys.kth.se/KTH-APC/apc_perception/issues/5)**.

## Individual components

### Kinect robot

The launch file `perception_kinect_robot.launch` starts  [`kinect2_bridge`](../drivers/iai_kinect2/kinect2_bridge/launch/kinect2_bridge.launch) with the following cameras:
* `kinect_chest` ; the camera intrinsics are loaded from [here](https://gits-15.sys.kth.se/KTH-APC/apc_calibration/tree/master/calibration_data/intrinsics)
* **More to be added here later on**

### Ids robot

The launch file `perception_ids_robot.launch` starts [`xs_cam`](../drivers/ueye_cam/launch/xs_cam.launch) with the following cameras:
* `ids_left` ; the camera intrinsics are loaded from [here](https://gits-15.sys.kth.se/KTH-APC/apc_calibration/tree/master/calibration_data/intrinsics)
* **More to be added here later on**

**This has to be run before `perception.launch` (see https://gits-15.sys.kth.se/KTH-APC/apc_perception/issues/5)**.

### Perception TF frames

This starts an instance of the [`calibration_publisher`](https://gits-15.sys.kth.se/KTH-APC/apc_calibration/blob/master/calibration_publisher/launch/calibration_publisher.launch) for each sensor defined, loading the appropriate extrinsic transformation by name (from [here](https://gits-15.sys.kth.se/KTH-APC/apc_calibration/tree/master/calibration_data/extrinsics)). If the extrinsic calibration file cannot be found, identity will be published instead. Transforms for the following are published:
* `kinect_chest` - between `base` and `kinect_chest_link`
* `kinect_head` - between `base` and `kinect_head_link` 
* `kinect_external` - between `base` and `kinect_external_link` 
* `ids_left` - between `base` and `ids_left_rgb_optical_center` (**Note - `base` will change to relevant robot joint once the extrinsics have been computed**).
* `ids_right` - between `base` and `ids_right_rgb_optical_center` (**Note - `base` will change to relevant robot joint once the extrinsics have been computed**).

### Kinect external

To be added, launching `kinect_external`.
