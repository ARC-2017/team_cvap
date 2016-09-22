cmake_minimum_required(VERSION 2.8.3)
project(apc_objects_detection_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  simtrack_nodes
)

link_directories(${catkin_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
 
add_action_files(DIRECTORY action FILES LookForObject.action) #Averaging.action )
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
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
  CATKIN_DEPENDS roscpp rospy std_msgs tf
  DEPENDS system_lib
)



###########
## Build ##
###########

#----------Server and client
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#############
## Install ##
#############

add_executable(apc_objects_detection_server src/apc_objects_detection_server.cpp)
add_dependencies(apc_objects_detection_server ${PROJECT_NAME}_gencpp)
target_link_libraries(apc_objects_detection_server ${catkin_LIBRARIES})

add_executable(apc_objects_detection_client src/apc_objects_detection_client.cpp)
add_dependencies(apc_objects_detection_client ${PROJECT_NAME}_gencpp)
target_link_libraries(apc_objects_detection_client ${catkin_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/gen_numbers.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
