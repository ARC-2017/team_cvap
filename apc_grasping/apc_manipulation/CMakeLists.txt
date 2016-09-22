cmake_minimum_required(VERSION 2.8.3)
project(apc_manipulation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf
  genmsg
  actionlib_msgs
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(Eigen REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveArm.action
  PickObject.action
  PlaceObject.action
  Approach.action
  Retreat.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES apc_manipulation
 CATKIN_DEPENDS moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(apc_manipulation_node
	src/apc_manipulation_node.cpp
	src/apc_manipulation/BaxterGripper.cpp
	src/apc_manipulation/RobotInterface.cpp
	src/apc_manipulation/BaxterGripper.cpp
	src/apc_manipulation/MoveArm.cpp
	src/apc_manipulation/PickObject.cpp
	src/apc_manipulation/PlaceObject.cpp
)
target_link_libraries(apc_manipulation_node ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
