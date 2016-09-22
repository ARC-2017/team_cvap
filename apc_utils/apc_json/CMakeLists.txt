cmake_minimum_required(VERSION 2.8.3)
project(apc_json)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)