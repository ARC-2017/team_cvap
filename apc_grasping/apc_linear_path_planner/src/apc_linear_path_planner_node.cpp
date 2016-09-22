#include "ros/ros.h"
#include "apc_linear_path_planner/PlanLinearPath.h"
#include "apc_linear_path_planner/APCMotionPlanner.h"
#include <string>

using namespace apc_linear_path_planner;

int main(int argc, char**argv){
  // Set up ROS
  ros::init(argc, argv, "apc_linear_path_planner_node");
  ros::NodeHandle nh;

  // Set up planner
  APCMotionPlannerParameters planner_parameters;
  // TODO: we want to get these parameters from the parameter server
  nh.param<std::string>("planning_scene_topic", planner_parameters._planning_scene_topic, "/move_group/monitored_planning_scene");
  nh.param<std::string>("debug_planning_scene_topic", planner_parameters._debug_planning_scene_topic, "/planning_scene_linear_path_planner");
  nh.param<std::string>("robot_description_param", planner_parameters._robot_description, "/robot_description");
  nh.param<std::string>("joint_states_topic", planner_parameters._joint_states_topic, "/robot/joint_states");
  nh.param<bool>("debug", planner_parameters._debug, true);
  nh.param<double>("sampling_step", planner_parameters._sampling_step, 0.01);
  
  APCMotionPlanner motion_planner(planner_parameters);
  ROS_INFO("PLANNER CREATED");

  // Advertise service
  std::string serviceName("/apc/plan_linear_path");
  std::string invertServiceName("/apc/invert_trajectory");
  std::string ikServiceName("/apc/compute_ik_solution");
  std::string cartesianPathServiceName("/apc/plan_cartesian_linear_path");
  ros::ServiceServer service = nh.advertiseService(serviceName, &APCMotionPlanner::serviceCallback, &motion_planner);
  ros::ServiceServer invertService = nh.advertiseService(invertServiceName, &APCMotionPlanner::invertTrajectoryServiceCallback, &motion_planner);
  ros::ServiceServer ikService = nh.advertiseService(ikServiceName, &APCMotionPlanner::computeIKSolutionServiceCallback, &motion_planner);
  ros::ServiceServer cartesianPathService = nh.advertiseService(cartesianPathServiceName, &APCMotionPlanner::planCartesianLinearPathCallback, &motion_planner);  
  ROS_INFO("APC linear path planner is ready; spinning");
  // spin
  ros::spin();
  return 0;
}
