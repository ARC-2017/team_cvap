cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_task_manager)
set(CMAKE_CXX_COMPILER=/usr/bin/g++-4.9)
set(CMAKE_C_COMPILER=/usr/bin/gcc-4.9)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  actionlib
  apc_bt_comms
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp roslib std_msgs std_srvs actionlib apc_bt_comms
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(task_manager src/task_manager.cpp)
add_dependencies(task_manager ${apc_bt_comms_EXPORTED_TARGETS})
target_link_libraries(task_manager ${catkin_LIBRARIES})
