cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_status_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  apc_bt_comms
  behavior_trees
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp std_msgs apc_bt_comms behavior_trees
 DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(apc_bt_status_action src/apc_bt_status_action.cpp)
target_link_libraries(apc_bt_status_action ${catkin_LIBRARIES})
