# amazon_challenge_launch #

Collection of launch files for amazon challenge

## Launching the super awesome CVAP amazon challenge demo ##

* **NB**: Make sure you checkout **ALL** amazon challenge packages from the **cvap-demo** branch!!!

* The arrangement of objects in the shelf should be as shown in the following picture:

![IMG_20150610_202745.jpg](https://bitbucket.org/repo/Eqdaxz/images/607827857-IMG_20150610_202745.jpg)

* Release emergency buttons on the PR2

* Launch kinect on the PR2

* If launching from another account other than **pr2admin** edit the username in the "machine" tag in the **amazon_challenge_launch/launch/shelf_calibration.launch** file 

* Configure the IMAX camera:

```
#!python

guvcview
```

Set the exposure (manual) to 600, autofocus off, focus to zero.

* Launch **shelf calibration**:


```
#!python

roslaunch amazon_challenge_launch shelf_calibration.launch contest:=false
```

* Launch the behavior tree stuff:

```
#!python

roslaunch amazon_challenge_launch test_bt_random.launch
```