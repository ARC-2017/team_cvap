cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_comms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib_msgs
)

link_directories(${catkin_LIBRARY_DIRS})

add_service_files(FILES
   TaskManager.srv
   SetBool.srv
 )

 add_action_files(
  DIRECTORY action
  FILES Apc.action Manager.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
 DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
