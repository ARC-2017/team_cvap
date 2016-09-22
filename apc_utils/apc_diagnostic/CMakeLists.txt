cmake_minimum_required(VERSION 2.8.3)
project(apc_diagnostic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  diagnostic_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apc_diagnostic
#  CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS system_lib diagnostic_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)

target_link_libraries(example
   ${catkin_LIBRARIES}
 )

catkin_install_python(PROGRAMS scripts/example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
