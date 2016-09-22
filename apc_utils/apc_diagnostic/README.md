# apc_diagnostic

## Table of Contents

* [Description](README.md#description)
* [Test](README.md#test)
* [Mainteners](README.md#maintainers)
* [Define your diagnostics](README.md#define-your-diagnostic)
* [Launch](README.md#launch)
  * [Example](README.md#example)


## Description

This is the diagnostic package for KTH APC 2016.
It builds upon the [ros diagnostic tool](http://wiki.ros.org/diagnostics) 
Please refer to:
* [diagnostic_aggregator](http://wiki.ros.org/diagnostic_aggregator?distro=indigo) (to group your diagnostic info) 
* [diagnostic_updater](http://wiki.ros.org/diagnostic_updater?distro=indigo) (to define and publish your diagnostic info) 
* [diagnostic_msgs](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html) (diagnostic message definition) 


## Test

The files examples.cpp and example.py contain examples in cpp and python of diagnostic_updaters (with classes).
Group your diagnostic messages using your group prefix (e.g. Grasping_, Perception_ etc.) 

## Define your diagnostic messages

* not writen yet... after Demo 1 :D

## Maintainers

* Sergio Caccamo <caccamo@kth.se> (Perception)
* ... (Grasping) `sudo apt-get install libusb-1.0-0-dev`
* ... (High Level Planning)
* ... (Integration)

## Launch

### Example

* To launch the example file (cpp)

    1.  run: `roscore #terminal1`. 
    2.  `roslaunch apc_diagnostic aggregator.launch #terminal2`. 
    3.  `rosrun rqt_runtime_monitor rqt_monitor #terminal3`. 
    
    ```
    This will be explained more in details later.
    ```
    
