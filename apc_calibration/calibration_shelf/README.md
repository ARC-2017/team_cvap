Shelf Calibration
==============
calibrate the shelf pose with respect to the kinect chest link.

shelf_publisher.launch will run the laser simulater and also the shelf pose estimator. 
it may show some red error at the beginning.

check list:
if it doesn't work, check 
-1: is kinect working?
-2: run rosrun pluto_xi simulate_laser and check the topic "simu_scan", is it seeing the legs? 

