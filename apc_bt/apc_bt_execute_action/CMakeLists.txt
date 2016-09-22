cmake_minimum_required(VERSION 2.8.3)
project(apc_bt_execute_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  actionlib
  apc_bt_comms
  behavior_trees
  apc_manipulation
  apc_objects_detection_action
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES apc_bt_execute_action
 CATKIN_DEPENDS roscpp std_msgs std_srvs actionlib apc_bt_comms behavior_trees apc_manipulation apc_objects_detection_action
 DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(execute_action src/execute_action.cpp)
target_link_libraries(execute_action ${catkin_LIBRARIES})

add_executable(look_bin_kinect_action src/look_bin_kinect_action.cpp)
target_link_libraries(look_bin_kinect_action execute_action ${catkin_LIBRARIES})

add_executable(look_bin_ids_action src/look_bin_ids_action.cpp)
target_link_libraries(look_bin_ids_action execute_action ${catkin_LIBRARIES})

add_executable(look_tote_ids_action src/look_tote_ids_action.cpp)
target_link_libraries(look_tote_ids_action execute_action ${catkin_LIBRARIES})

add_executable(look_tote_kinect_action src/look_tote_kinect_action.cpp)
target_link_libraries(look_tote_kinect_action execute_action ${catkin_LIBRARIES})

add_executable(move_action src/move_action.cpp)
target_link_libraries(move_action execute_action ${catkin_LIBRARIES})

add_executable(move_ids_action src/move_ids_action.cpp)
target_link_libraries(move_ids_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_1_action src/set_ids_1_action.cpp)
target_link_libraries(set_ids_1_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_2_action src/set_ids_2_action.cpp)
target_link_libraries(set_ids_2_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_3_action src/set_ids_3_action.cpp)
target_link_libraries(set_ids_3_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_4_action src/set_ids_4_action.cpp)
target_link_libraries(set_ids_4_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_6_action src/set_ids_6_action.cpp)
target_link_libraries(set_ids_6_action execute_action ${catkin_LIBRARIES})

add_executable(set_ids_8_action src/set_ids_8_action.cpp)
target_link_libraries(set_ids_8_action execute_action ${catkin_LIBRARIES})

add_executable(set_memory_ids src/set_memory_ids.cpp)
target_link_libraries(set_memory_ids execute_action ${catkin_LIBRARIES})

add_executable(has_memory_ids src/has_memory_ids.cpp)
target_link_libraries(has_memory_ids execute_action ${catkin_LIBRARIES})

add_executable(set_memory_kinect src/set_memory_kinect.cpp)
target_link_libraries(set_memory_kinect execute_action ${catkin_LIBRARIES})

add_executable(has_memory_kinect src/has_memory_kinect.cpp)
target_link_libraries(has_memory_kinect execute_action ${catkin_LIBRARIES})

add_executable(set_memory src/set_memory.cpp)
target_link_libraries(set_memory execute_action ${catkin_LIBRARIES})

add_executable(has_memory src/has_memory.cpp)
target_link_libraries(has_memory execute_action ${catkin_LIBRARIES})

add_executable(move_to_home_action src/move_to_home_action.cpp)
target_link_libraries(move_to_home_action execute_action ${catkin_LIBRARIES})

add_executable(move_to_tote_action src/move_to_tote_action.cpp)
target_link_libraries(move_to_tote_action execute_action ${catkin_LIBRARIES})

add_executable(pick_bin_action src/pick_bin_action.cpp)
target_link_libraries(pick_bin_action execute_action ${catkin_LIBRARIES})

add_executable(pick_tote_action src/pick_tote_action.cpp)
target_link_libraries(pick_tote_action execute_action ${catkin_LIBRARIES})

add_executable(place_bin_action src/place_bin_action.cpp)
target_link_libraries(place_bin_action execute_action ${catkin_LIBRARIES})

add_executable(place_tote_action src/place_tote_action.cpp)
target_link_libraries(place_tote_action execute_action ${catkin_LIBRARIES})

add_executable(get_next_target_action src/get_next_target_action.cpp)
target_link_libraries(get_next_target_action execute_action ${catkin_LIBRARIES})

add_executable(success_action src/success_action.cpp)
target_link_libraries(success_action execute_action ${catkin_LIBRARIES})

add_executable(place_failure_action src/place_failure_action.cpp)
target_link_libraries(place_failure_action execute_action ${catkin_LIBRARIES})

add_executable(is_level_3 src/is_level_3.cpp)
target_link_libraries(is_level_3 execute_action ${catkin_LIBRARIES})

add_executable(is_level_4 src/is_level_4.cpp)
target_link_libraries(is_level_4 execute_action ${catkin_LIBRARIES})

add_executable(is_level_1 src/is_level_1.cpp)
target_link_libraries(is_level_1 execute_action ${catkin_LIBRARIES})

add_executable(is_level_2 src/is_level_2.cpp)
target_link_libraries(is_level_2 execute_action ${catkin_LIBRARIES})

add_executable(is_object_simtrackable src/is_object_simtrackable.cpp)
target_link_libraries(is_object_simtrackable execute_action ${catkin_LIBRARIES})

add_executable(ids_no_move_home src/ids_no_move_home.cpp)
target_link_libraries(ids_no_move_home execute_action ${catkin_LIBRARIES})

add_executable(pick_no_move_home src/pick_no_move_home.cpp)
target_link_libraries(pick_no_move_home execute_action ${catkin_LIBRARIES})

add_executable(is_place_failure src/is_place_failure.cpp)
target_link_libraries(is_place_failure execute_action ${catkin_LIBRARIES})

add_executable(load_simtrack_models src/load_simtrack_models.cpp)
target_link_libraries(load_simtrack_models execute_action ${catkin_LIBRARIES})

add_executable(unload_simtrack_models src/unload_simtrack_models.cpp)
target_link_libraries(unload_simtrack_models execute_action ${catkin_LIBRARIES})

add_executable(do_nothing_action src/do_nothing_action.cpp)
target_link_libraries(do_nothing_action execute_action ${catkin_LIBRARIES})
