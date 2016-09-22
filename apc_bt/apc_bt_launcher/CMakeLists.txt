cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_launcher)

find_package(catkin REQUIRED COMPONENTS)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS
 DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
