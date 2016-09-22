# robot_face

## Table of Contents

* [Description](README.md#description)
* [Run](README.md#run)
* [Mainteners](README.md#maintainers)
* [Launch](README.md#launch)



## Description

This package allows to change image on the Baxter head display. It allows to show different expressions and move the eyes.

## Run

It is possible to change face by publishing and ID (Float32 > 0) to the topic `/robot/expressions/selector`.

e.g.:  `rostopic pub /robot/expressions/selector std_msgs/Float32 "data: 1.0"`.

Values:
* 0 : Baxter is happy
* 1 : Baxter is sad
* 2 : Baxter is sweet
* 3 : Baxter is crazy
* 4 : Baxter is sleepy
* 5 : Baxter is focused
* 6 : Baxter is doubtful
* 7 : Baxter is sweaty
* >=7 : Baxter is (happy)

It is possible to change face background by publishing and ID (Float32 > 0) to the topic `/robot/background/selector`.

e.g.:  `rostopic pub /robot/background/selector std_msgs/Float32 "data: 1.0"`.

Values:
* 0 : Baxter is red
* 1 : Baxter is powerup
* 2 : Baxter is rgbd
* 3 : Baxter is lumberjack
* 4 : Baxter is yolo



It is possible to move Baxter eyes in vertical and horizontal by publishing values (Float32MultiArray -1<value<1) on the topic `/robot/expressions/pos_eye`.

e.g.: 
```
rostopic pub /robot/expressions/pos_eye std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [-0.2,0.4]"
``` 

The first value moves the eyes in horizontal the second values move the eyes in vertical.

## Maintainers

* Sergio Caccamo <caccamo@kth.se> 

## Launch

    1.  run the ros master Baxter and ssh to it (alternatively rocore on #terminal1)
    2.  `roslaunch robot_face display_face.launch #terminal2`. 
    3.  `rqt #terminal3`. 
    
    ```
    This will be explained more in details later.
    ```
    
