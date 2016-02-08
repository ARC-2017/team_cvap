cmake_minimum_required(VERSION 2.8.3)
project(amazon_challenge_bt_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  roslib
  tf
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread program_options)



# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   BinTrigger.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES BT.action ObjectsList.action Detector.action
)




# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)


# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_actionlib
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
#)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(learning_actionlib
#   src/${PROJECT_NAME}/learning_actionlib.cpp
# )

set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})

add_executable(Action src/action_server.cpp)

target_link_libraries(
  Action
  ${catkin_LIBRARIES}
)

add_dependencies(
 Action
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)


add_executable(Condition src/condition_server.cpp)

target_link_libraries(
  Condition
  ${catkin_LIBRARIES}
)

add_dependencies(
 Condition
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)






add_executable(ActionClient src/action_client.cpp)

target_link_libraries(
  ActionClient
  ${catkin_LIBRARIES}
)

add_dependencies(
 ActionClient
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)




add_executable(ConditionClient src/condition_client.cpp)

target_link_libraries(
  ConditionClient
  ${catkin_LIBRARIES}
)

add_dependencies(
 ConditionClient
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(DetectServer src/pr2_detect_server.cpp)

target_link_libraries(
  DetectServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  DetectServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)


add_executable(GraspServer src/pr2_grasp_server.cpp)

target_link_libraries(
  GraspServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  GraspServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)



#add_executable(DropServer src/pr2_drop_server.cpp)

#target_link_libraries(
#  DropServer
#  ${catkin_LIBRARIES}
#)

#add_dependencies(
#  DropServer
#  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
#)



# add_executable(PoseArmServer src/pr2_pose_arm_server.cpp)

# target_link_libraries(
#   PoseArmServer
#   ${catkin_LIBRARIES}
# )

# add_dependencies(
#   PoseArmServer
#   ${bt_actions_EXPORTED_TARGETS}
# )



add_executable(ObjectDetectedServer src/pr2_object_detected_server.cpp)

target_link_libraries(
  ObjectDetectedServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  ObjectDetectedServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)

add_executable(ObjectDroppedServer src/pr2_object_dropped_server.cpp)

target_link_libraries(
  ObjectDroppedServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  ObjectDroppedServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)




add_executable(ObjectGraspedServer src/pr2_object_grasped_server.cpp)

target_link_libraries(
  ObjectGraspedServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  ObjectGraspedServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)



add_executable(ArmPosedServer src/pr2_arm_posed_server.cpp)

target_link_libraries(
  ArmPosedServer
  ${catkin_LIBRARIES}
)

add_dependencies(
  ArmPosedServer
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)





# add_executable(ArmPosedServer src/pr2_arm_posed_server.cpp)

# target_link_libraries(
#   ArmPosedServer
#   ${catkin_LIBRARIES}
# )

# add_dependencies(
#   ArmPosedServer
#   ${bt_actions_EXPORTED_TARGETS}
# )



add_executable(ListDone src/list_done.cpp)

target_link_libraries(
  ListDone
  ${catkin_LIBRARIES}
)

add_dependencies(
  ListDone
  ${amazon_challenge_bt_actions_EXPORTED_TARGETS}
)



## Declare a cpp executable
# add_executable(learning_actionlib_node src/learning_actionlib_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(learning_actionlib_node learning_actionlib_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(learning_actionlib_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS learning_actionlib learning_actionlib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_learning_actionlib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
