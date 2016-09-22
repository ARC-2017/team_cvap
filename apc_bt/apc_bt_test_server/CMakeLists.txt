cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_test_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  apc_bt_comms
  behavior_trees
  apc_manipulation
  apc_objects_detection_action
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES apc_bt_test_server
 CATKIN_DEPENDS roscpp std_msgs apc_bt_comms behavior_trees apc_manipulation apc_objects_detection_action
 DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(apc_bt_test_server src/apc_bt_test_server.cpp)
target_link_libraries(apc_bt_test_server ${catkin_LIBRARIES})

add_executable(apc_bt_test_move src/apc_bt_test_move.cpp)
target_link_libraries(apc_bt_test_move apc_bt_test_server ${catkin_LIBRARIES})

add_executable(apc_bt_test_look_for_object src/apc_bt_test_look_for_object.cpp)
target_link_libraries(apc_bt_test_look_for_object apc_bt_test_server ${catkin_LIBRARIES})

add_executable(apc_bt_test_pick src/apc_bt_test_pick.cpp)
target_link_libraries(apc_bt_test_pick apc_bt_test_server ${catkin_LIBRARIES})

add_executable(apc_bt_test_place src/apc_bt_test_place.cpp)
target_link_libraries(apc_bt_test_place apc_bt_test_server ${catkin_LIBRARIES})
