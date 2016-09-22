Camera Calibration
==============

- make sure that you have generated the sift features for the markers in simtrack/data/object_models.
- attach the advanced and professional calibration gripper to the baxter and enable the robot.
- start the calibration node with the launch file.
- move the marker in fron of the camera.
- record the position: rosservice call /read_tfs (collect at least 20 measurements).
- compute the transformation: rosservice call /compute_frames. You can check the result in rviz.

kinect
--------------
- roslaunch calibration_routine calibration_sequence_kinect camera_name:=(kinect_chest, kinect_head, kinect_external)

ids
----------------

- roslaunch calibration_routine calibration_sequence_ids.launch ids_name:=(right, left)

----------------
The results of the calibration are saved as translation (tx ty tz) and quaternion (qx qy qz qw) representing the transformation World_to_Camera.

Tool Calibration
==============

How to run it
----------------
```
rosrun calibration_routine tool_calib.py -h
rosrun calibration_routine tool_calib.py <left/right> <name of gripper>
```

E.g., to calibrate the tooltips use for **left gripper** with 90 degree suction cup use:
```
rosrun calibration_routine tool_calib.py left right_end_effector_90
```
For the **right gripper**:
```
rosrun calibration_routine tool_calib.py right right_end_effector_90
```
Collecting measurements
----------------

The calibration procedure will first ask you to collect at least 4 measurements for calibrating the tooltip **position**. You do this by varying the orientation of the gripper between each measurement while keeping the position of the tooltip fixed in a specific point (use a fixed, pointy object as a reference). 

**Collect measurements by pressing the gray circular button on the baxter wrist**
**Stop by pressing the white oval button on the baxter wrist**
  
In the next step of the calibration you need to place the suction cup flat w.r.t the base frame. You can place the suction cup flat on the screw that is in the front part of the Baxter, at the chest (belly button) level.
Again use the gray (circular) / white (oval) buttons on the wrist to collect measurement/stop.

URDF
----------------
The calibrations are stored in ``apc_calibration/calibration_data/extrinsics/<gripper_name>.txt`` (e.g. left_end_effector_90.txt).

**NB: You then have to manually modify the URDF file corresponding to that end effector / gripper**. You do this by placing the calibration values (x, y, z, roll, pitch, yaw) in the corresponding urdf file at ``apc_calibration/calibration_data/urdf/<gripper_name>.urdf.xacro``.

The calibration files in calibration_data ar expressed in quaternions. Print them out in RPY format using e.g.:

```
rosrun calibration_data calib_rpy.py left_end_effector_90.txt
```

When the simulation runs, it includes the calibrated end effectors directly in the URDF of the robot. 
For the real robot, the URDF is modified at runtime by publishing on the **/robot/urdf** topic as described in these pages [Baxter MRSP](http://sdk.rethinkrobotics.com/wiki/Gripper_Customization#Mutable_Robot_State_Publisher), [Gripper Customization](http://sdk.rethinkrobotics.com/wiki/URDF_Configuration_Example).

Look at **apc_grasping/apc_manipulation/launch/tool_calib_publisher.launch**.

After running the system, you should see the following in Rviz:
![alt text](http://i.imgur.com/nW00Vhp.png "Tool Calibration")

