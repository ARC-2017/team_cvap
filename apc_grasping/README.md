# apc_grasping
Grasping for APC 2016


Maintainer:   Joshua <haustein@kth.se>, Sergio <caccamo@kth.se>, Anastasia <>, Diogo <>, Ramviyas <>

This README concerns the problem of Manipulation and Grasping for the APC 2016.
--------------------------------------------------------------------------------------

PROBLEM STATEMENT:
 Manipulation is hard. The real world is a ....!

CONTENTS:
The repository currently consists of two packages: 
 - shelf_pose_optimization
 - manipulation_node
 
The shelf_pose_optimization package contains code that can be used to find a somehow optimal 
pose of the shelf in respect to Baxter. It will serve in future as base to compute the same for the tote.
You don't need this package if you are not interested on working on that.

The package manipulation_node contains the code for the manipulation node. It will eventually provide
five different actions:
 - MoveArm 
 - CheckPickFeasibility
 - Pick
 - CheckPlaceFeasibility 
 - Place

We focus on MoveArm, Pick and Place for now.
MoveArm
-------
**PREFIX**: /apc/manipulation/move_arm/

Input arguments required on parameter server: 

Name           | Type        | Description
-----------    | ----------  | -----------
$PREFIX/input/goal | String      | End-effector pose to move to
$PREFIX/input/arm  | String      | left_arm or right_arm

**Output**:

Success or Failure. Reasons for failure will be printed. In case of success, the arm moves.

PickObject
-------
**PREFIX**: /apc/manipulation/pick_object

Input arguments required on parameter server: 

Name           | Type        | Description
-----------    | ----------  | -----------
$PREFIX/input/target | String      | Name of the object to pick.
$PREFIX/input/obstacles  | List of Strings      | Name of all other objects in the bin/tote
$PREFIX/input/bin_id  | String         | Name of the bin or 'tote'
$PREFIX/input/arm | String | left_arm or right_arm

**Output**:

Success or Failure. Reasons for failure will be printed. In case of success, the target object will be picked.

Output parameter on parameter server:

Name | Type | Description
-----| ---- | -----------
$PREFIX/output/eef_object_pose | PoseStamped as Dictionary | Estimate of where the object is w.r.t eef

PlaceObject
----------

**PREFIX**: /apc/manipulation/place_object

Input arguments required on parameter server: 

Name | Type | Description
---- | ---- | -----------
$PREFIX/input/bin_id       |    String | Name of the bin or 'tote'
$PREFIX/input/obstacles     |   List of Strings | Name of all objects in the bin/tote
$PREFIX/input/eef_object_pose |  PoseStamped as Dict | Pose of the object picked w.r.t eef
$PREFIX/input/arm | String | Either left_arm or right_arm

**Output**:
Success or Failure. Reasons for failure will be printed. In case of success, the target object will be placed.

--------------------------------------------------

HOW TO USE:
 1. Don't, or ask Joshua for now.
 
NOTE: This is work in progress.
