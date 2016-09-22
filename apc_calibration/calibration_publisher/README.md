Calibration publisher
====================

Node which publishes a transform between two tf frames. 

### Parameters

* `calibration_file` - the name of the file where the calibration data is stored
* `source_frame` - the TF frame on the robot (usually `robot_base` or something similar).
* `target_frame` - the TF frame on the sensor (e.g. `kinect_chest_rgb_optical_center`). 
 
### Calibration file

The data stored in the file should be in the format: 

`tx ty tz qx qy qz qw`

### Launch file

To run this node execute (for example):

```
roslaunch calibration_publisher calibration_publisher.launch source_frame:=base_link calibration_file:=test_calib.txt target_frame:=kinect2_link
```
