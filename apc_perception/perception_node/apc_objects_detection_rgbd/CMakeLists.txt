cmake_minimum_required(VERSION 2.8.3)
project(apc_objects_detection_rgbd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  
  pcl_conversions
  pcl_ros
  pcl_msgs
  
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  
  apc_objects_detection_action
)


link_directories(${catkin_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system)




## Copy header files
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
#)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

#add_action_files(DIRECTORY action FILES LookForObjectRGBD.action)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs apc_objects_detection_action)
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs apc_objects_detection_action
  DEPENDS
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############




 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


#----------Server and client
include_directories(include ${catkin_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS} )

add_executable(rgbd_server src/rgbd_server.cpp)
add_dependencies(rgbd_server ${PROJECT_NAME}_gencpp)
target_link_libraries(rgbd_server ${catkin_LIBRARIES})

add_executable(rgbd_client src/rgbd_client.cpp)
add_dependencies(rgbd_client ${PROJECT_NAME}_gencpp)
target_link_libraries(rgbd_client ${catkin_LIBRARIES})

# pcl libraries
target_link_libraries(rgbd_server ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so )


