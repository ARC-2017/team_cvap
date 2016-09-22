cmake_minimum_required(VERSION 2.8.7)
project(perception_pc_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_transport
  roscpp
  dynamic_reconfigure
  std_msgs
  message_generation
  tf
  geometry_msgs
  perception_services
  pcl_ros
  siftgpu
  rendering
  utilities
  pose_estimation
  low_level_vision
  interface
)
# PCL
find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 2.4.8 REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4 -g -fPIC -std=c++0x -fpermissive")

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Feature.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS cv_bridge
  CATKIN_DEPENDS image_transport tf geometry_msgs message_runtime perception_services
)

# show additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f
  OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME} SOURCES
  ${FILES_LIST}
  ${PROJECT_SOURCE_DIR}/../README.md
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV2_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pc_segmentation.cpp
  src/feature_segmentation.cpp
  src/model_matcher.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  perception_services_gencfg
   ${PROJECT_NAME}_gencfg
)

add_executable(pc_segmentation
  src/pc_segmentation_main.cpp
)

target_link_libraries(pc_segmentation
  ${PROJECT_NAME}
)

add_dependencies(pc_segmentation
  perception_services_gencfg)

add_executable(feature_segmentation
  src/feature_segmentation_main.cpp
)

target_link_libraries(feature_segmentation
  ${PROJECT_NAME}
)

add_dependencies(feature_segmentation
  perception_services_gencfg)
