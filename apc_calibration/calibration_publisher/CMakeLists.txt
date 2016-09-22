cmake_minimum_required(VERSION 2.8.3)
project(calibration_publisher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs tf)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf
  DEPENDS 
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(calibration_publisher src/calibration_publisher.cpp)


target_link_libraries(calibration_publisher
  ${catkin_LIBRARIES}
)


