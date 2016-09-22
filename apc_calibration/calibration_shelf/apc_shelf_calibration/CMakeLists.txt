cmake_minimum_required(VERSION 2.8.3)
project(apc_shelf_calibration)


find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pluto_xi
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(simulate_laser src/simulate_laser.cpp)
add_executable(shelf_icpfuser src/shelf_icpfuser.cpp)

target_link_libraries(simulate_laser
  ${catkin_LIBRARIES}
)

target_link_libraries(shelf_icpfuser
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
