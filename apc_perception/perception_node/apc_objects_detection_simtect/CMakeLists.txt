cmake_minimum_required(VERSION 2.8.3)
project(apc_objects_detection_simtect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  perception_services
  apc_objects_detection_action

  cmake_modules
)


link_directories(${catkin_LIBRARY_DIRS})

find_package(Eigen REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
 

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
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs tf apc_objects_detection_action
  DEPENDS system_lib
)



###########
## Build ##
###########

#----------Server and client
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

#############
## Install ##
#############

add_executable(simtect_server src/simtect_server.cpp)
add_dependencies(simtect_server ${PROJECT_NAME}_gencpp)
target_link_libraries(simtect_server ${catkin_LIBRARIES})

add_executable(simtect_client src/simtect_client.cpp)
add_dependencies(simtect_client ${PROJECT_NAME}_gencpp)
target_link_libraries(simtect_client ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
