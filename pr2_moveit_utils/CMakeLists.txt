cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_utils)

find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)
