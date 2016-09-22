#include "apc_linear_path_planner/APCMotionPlanner.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_state/robot_state.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <map>
#include <iostream>


using namespace apc_linear_path_planner;

bool positionsInBounds(const std::vector<std::string>& joint_names,
                       moveit::core::RobotState& robot_state) {
  bool allBoundsSatisfied = true;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const moveit::core::JointModel* jm = robot_state.getJointModel(joint_names[i]);
    bool boundsSatisfied = robot_state.satisfiesPositionBounds(jm);
    if (!boundsSatisfied) ROS_INFO_STREAM("Bounds of joint " << joint_names[i] << " not satisfied");
    allBoundsSatisfied = allBoundsSatisfied && boundsSatisfied;
  }
  return allBoundsSatisfied;
}

// Callback for state validity check (TODO: this should also be called in plan(...))
bool isStateValid(planning_scene_monitor::LockedPlanningSceneRW& planning_scene,
                  robot_state::RobotState* rs,
                  const moveit::core::JointModelGroup* jg,
                  const double* jgv)
{
  rs->setJointGroupPositions(jg, jgv);
  collision_detection::CollisionRequest req;
  req.contacts = false;
  req.group_name = jg->getName();
  collision_detection::CollisionResult res;
  planning_scene->checkCollisionUnpadded(req, res, *rs);
  return !res.collision && positionsInBounds(jg->getJointModelNames(), *rs);
}

APCMotionPlanner::APCMotionPlanner(const APCMotionPlannerParameters& params):_args(params) {
  init();
}

APCMotionPlanner::~APCMotionPlanner() {

}

bool APCMotionPlanner::serviceCallback(apc_linear_path_planner::PlanLinearPath::Request& req,
  apc_linear_path_planner::PlanLinearPath::Response& res) {
    std::vector<double> goal_config;
    bool validInput = extractJointValues(req.start.name, req.goal, goal_config);

    bool success = false;
    if (validInput) {
      success = plan(req.start.name, req.start.position, goal_config, req.move_group, res.traj);
    }
    res.success = success;
    return validInput;
}

bool APCMotionPlanner::computeIKSolutionServiceCallback(apc_linear_path_planner::ComputeIKSolution::Request& req,
        apc_linear_path_planner::ComputeIKSolution::Response& res) {
  ROS_INFO("Received an inverse kinematics request");
  // Get hold of planning scene and create state
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(_planning_scene_monitor);
  auto robot_model = planning_scene->getRobotModel();
  moveit::core::RobotState robot_state(planning_scene->getCurrentState());
  // Set state to seed
  setConfiguration(robot_state, req.seed.name, req.seed.position);

  ROS_INFO("Computing...");
  res.success = computeIKSolution(req.pose, req.move_group, planning_scene, robot_state);
  ROS_INFO("... finished!");
  if (res.success) {
    const moveit::core::JointModelGroup* arm_group = robot_model->getJointModelGroup(req.move_group);
    const std::vector<std::string>& joint_names = arm_group->getJointModelNames();
    res.iksolution.name.clear();
    res.iksolution.position.clear();
    res.iksolution.velocity.clear();
    res.iksolution.effort.clear();
    for (auto joint_name : joint_names) {
      std::cout << joint_name << std::endl;
      res.iksolution.name.push_back(joint_name);
      const double* values = robot_state.getJointPositions(joint_name);
      res.iksolution.position.push_back(values[0]);
    }
    ROS_INFO("Found a collision-free ik solution!");
  } else {
    ROS_INFO("Could not find a collision-free ik solution!");
  }
  return true;
}

bool APCMotionPlanner::invertTrajectoryServiceCallback(apc_linear_path_planner::InvertTrajectory::Request& req,
        apc_linear_path_planner::InvertTrajectory::Response& res) {
  // Get the planning scene
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(_planning_scene_monitor);
  moveit::core::RobotState robot_state(planning_scene->getCurrentState());
  robot_trajectory::RobotTrajectory traj_obj(_planning_scene_monitor->getRobotModel(), req.move_group);
  traj_obj.setRobotTrajectoryMsg(robot_state, req.traj);
  traj_obj.reverse();
  finalizeTrajectoryMsg(traj_obj, res.traj);
  return true;
}

bool APCMotionPlanner::planCartesianLinearPathCallback(apc_linear_path_planner::PlanCartesianLinearPath::Request& req,
        apc_linear_path_planner::PlanCartesianLinearPath::Response& res) {
  ROS_INFO("Planning Cartesian linear path");
  // Get hold of planning scene, robot model and get state
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(_planning_scene_monitor);
  moveit::core::RobotState& robot_state(planning_scene->getCurrentStateNonConst());
  auto robot_model = planning_scene->getRobotModel();
  // define validity callback for path planning
  moveit::core::GroupStateValidityCallbackFn validity_fn = boost::bind(isStateValid, planning_scene, _1, _2, _3);
  // get arm group
  const moveit::core::JointModelGroup* arm_group = robot_model->getJointModelGroup(req.move_group);
  const moveit::core::LinkModel* link_model = robot_model->getLinkModel(req.link_name);
  if (!arm_group) {
    ROS_ERROR_STREAM("Move group invalid:" << req.move_group);
    return false;
  }
  if (!link_model) {
    ROS_ERROR_STREAM("Link name invalid:" << req.link_name);
    return false;
  }

  // set start state
  Eigen::VectorXd start_state;
  toEigen(req.start_config.position, start_state);
  setConfiguration(robot_state, req.start_config.name, start_state);
  // Get the direction to move
  Eigen::Affine3d link_frame = robot_state.getFrameTransform(req.link_name);
  Eigen::Vector3d goal_pos(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  Eigen::Vector3d direction = goal_pos - link_frame.translation();
  double distance = direction.norm();
  if (distance > 0.0) {
    direction.normalize();
    // compute path
    std::vector<moveit::core::RobotStatePtr> vec_traj;
    //TODO: do we need a jump detection? In that case replace the value 0.0 by sth larger.
    double cartesian_path_length = robot_state.computeCartesianPath(arm_group, vec_traj, link_model,
                                                        direction, true, distance, req.step_size,
                                                        0.0, validity_fn);

    res.success = cartesian_path_length > distance - 0.01;
    res.cartesian_path_length = cartesian_path_length;
    robot_trajectory::RobotTrajectory traj_obj(_planning_scene_monitor->getRobotModel(), req.move_group);
    for (auto state : vec_traj) {
      traj_obj.addSuffixWayPoint(*state, _args._sampling_step / distance);
    }
    finalizeTrajectoryMsg(traj_obj, res.traj);
    return true;
  }
  // Else empty path
  res.success = true;
  return true;
}

bool APCMotionPlanner::plan(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values_start,
        const std::vector<double>& joint_values_goal, const std::string& move_group_name, moveit_msgs::RobotTrajectory& traj_msg) {

  ROS_INFO("Planning linear path");
  ROS_DEBUG("from config: ");
  printConfiguration(joint_names, joint_values_start);
  ROS_DEBUG("to goal: ");
  printConfiguration(joint_names, joint_values_goal);
  // Check whether we have a sane input
  if ((joint_names.size() != joint_values_start.size())
    || (joint_names.size() != joint_values_goal.size())) {
    ROS_ERROR("At least on of the given configurations has invalid dimension (n=%u, start=%u, goal=%u)",
              uint(joint_names.size()), uint(joint_values_start.size()), uint(joint_values_goal.size()));
    return false;
  }

  // Get the planning scene
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(_planning_scene_monitor);
  moveit::core::RobotState& robot_state(planning_scene->getCurrentStateNonConst());
  robot_trajectory::RobotTrajectory traj_obj(_planning_scene_monitor->getRobotModel(), move_group_name);

  // planning_scene->printKnownObjects(std::cout);

  // Do linear interpolation
  Eigen::VectorXd start_config;
  Eigen::VectorXd goal_config;
  toEigen(joint_values_start, start_config);
  toEigen(joint_values_goal, goal_config);

  Eigen::VectorXd dir_config = goal_config - start_config;
  double length = dir_config.norm();
  if (length < 0.001) {
    ROS_INFO("We are already at our goal! Making a trivial trajectory.");
    traj_obj.addSuffixWayPoint(robot_state, 0.0);
    traj_obj.addSuffixWayPoint(robot_state, 0.1);
    finalizeTrajectoryMsg(traj_obj, traj_msg);
    return true;
  }
  dir_config.normalize();

  Eigen::VectorXd current_config = start_config;
  int num_steps = floor(length / _args._sampling_step);

  collision_detection::CollisionRequest req;
  req.contacts = false;
  req.group_name = move_group_name;
  collision_detection::CollisionResult res;

  for (int i = 0; i <= num_steps; ++i) {
    current_config = start_config + double(i) * _args._sampling_step * dir_config;
    setConfiguration(robot_state, joint_names, current_config);
    // ROS_INFO_STREAM(current_config);
    planning_scene->checkCollisionUnpadded(req, res, robot_state);
    bool valid = checkBounds(joint_names, robot_state) && !res.collision;
    if (!valid) {
      ROS_INFO("Linear path planning failed!");
      if (res.collision) ROS_INFO("The trajectory results in a collision.");
      if (!checkBounds(joint_names, robot_state)) ROS_INFO("The trajectory is out of bounds.");
      return false;
    }
    traj_obj.addSuffixWayPoint(robot_state, _args._sampling_step / length);
  }

  // Finally, check the goal state for validity
  setConfiguration(robot_state, joint_names, goal_config);
  planning_scene->checkCollisionUnpadded(req, res, robot_state);
  bool valid = checkBounds(joint_names, robot_state) && !res.collision;
  if (!valid) {
    ROS_INFO("Linear path planning failed! Goal configuration is invalid.");
    return false;
  }
  traj_obj.addSuffixWayPoint(robot_state, _args._sampling_step / length);

  finalizeTrajectoryMsg(traj_obj, traj_msg);
  ROS_INFO("Linear path planning successful!");

  return true;
}


void APCMotionPlanner::init() {
  _planning_scene_monitor = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(_args._robot_description);
  _planning_scene_monitor->startSceneMonitor(_args._planning_scene_topic);
  _planning_scene_monitor->startStateMonitor(_args._joint_states_topic);
  if (_args._debug) {
    _planning_scene_monitor->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE,
        _args._debug_planning_scene_topic);
  }
}

void APCMotionPlanner::toEigen(const std::vector<double>& joint_values, Eigen::VectorXd& vector) {
  vector.resize(joint_values.size());
  for (unsigned int i = 0; i < joint_values.size(); ++i) {
    vector[i] = joint_values[i];
  }
}

void APCMotionPlanner::fromEigen(std::vector<double>& joint_values, const Eigen::VectorXd& vector) {
  joint_values.resize(vector.size());
  for (unsigned int i = 0; i < joint_values.size(); ++i) {
    joint_values[i] = vector[i];
  }
}

void APCMotionPlanner::setConfiguration(moveit::core::RobotState& robot_state,
      const std::vector<std::string>& joint_names,
      const std::vector<double>& joint_values) {
  std::vector<double> value(1);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    value[0] = joint_values[i];
    robot_state.setJointPositions(joint_names[i], value);
    // const double* written_value = robot_state.getJointPositions(joint_names[i]);
    // ROS_INFO_STREAM("Position of joint " << joint_names[i] << " is " << written_value[0]);
  }
}

void APCMotionPlanner::setConfiguration(moveit::core::RobotState& robot_state,
      const std::vector<std::string>& joint_names,
      const Eigen::VectorXd& joint_values) {
  std::vector<double> value(1);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    value[0] = joint_values[i];
    robot_state.setJointPositions(joint_names[i], value);
    // const double* written_value = robot_state.getJointPositions(joint_names[i]);
    // ROS_INFO_STREAM("Position of joint " << joint_names[i] << " is " << written_value[0]);
    // written_value = robot_state.getJointVelocities(joint_names[i]);
    // ROS_INFO_STREAM("Velocity of joint " << joint_names[i] << " is " << written_value[0]);
  }
}

void APCMotionPlanner::printConfiguration(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) {
  for (size_t i = 0; i < joint_names.size(); ++i) {
    ROS_DEBUG_STREAM(joint_names[i] << ": " << joint_values[i]);
  }
}

bool APCMotionPlanner::extractJointValues(const std::vector<std::string>& joint_names,
                                          const sensor_msgs::JointState js,
                                          std::vector<double>& values) {
  std::map<std::string, double> mymap;
  for (size_t i = 0; i < js.name.size(); ++i) {
    mymap[js.name[i]] = js.position[i];
  }
  values.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    auto it = mymap.find(joint_names[i]);
    if (it == mymap.end()) {
      return false;
    }
    values[i] = it->second;
  }
  return true;
}

bool APCMotionPlanner::checkBounds(const std::vector<std::string>& joint_names, moveit::core::RobotState& robot_state) const {
  return positionsInBounds(joint_names, robot_state);
}

void APCMotionPlanner::finalizeTrajectoryMsg(robot_trajectory::RobotTrajectory& traj_obj, moveit_msgs::RobotTrajectory& traj_msg) {
    // Retime the trajectory.
  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  time_param.computeTimeStamps(traj_obj);
  // Convert the retimed trajectory into a message
  traj_obj.getRobotTrajectoryMsg(traj_msg);
  // // PRINT TRAJECTORY MESSAGE
  // std::cout << "Trajectory:" << std::endl;
  // for (unsigned int i = 0; i < traj_msg.joint_trajectory.points.size(); ++i) {
  //   trajectory_msgs::JointTrajectoryPoint jtpoint = traj_msg.joint_trajectory.points[i];
  //   std::cout << "timestamp: " << jtpoint.time_from_start << " ";
  //   for (unsigned int j = 0; j < jtpoint.positions.size(); ++j) {
  //     std::cout << jtpoint.positions[j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
}

bool APCMotionPlanner::computeIKSolution(const geometry_msgs::Pose& global_pose,
      const std::string& move_group_name,
      planning_scene_monitor::LockedPlanningSceneRW& planning_scene,
      moveit::core::RobotState& robot_state)
{
  auto robot_model = planning_scene->getRobotModel();
  const moveit::core::JointModelGroup* arm_group = robot_model->getJointModelGroup(move_group_name);
  moveit::core::GroupStateValidityCallbackFn validity_fn = boost::bind(isStateValid, planning_scene, _1, _2, _3);
  kinematics::KinematicsQueryOptions options;
  options.lock_redundant_joints = false;
  options.return_approximate_solution = false;
  bool has_ik = robot_state.setFromIK(arm_group, global_pose, 10, 0.01, validity_fn, options);
  return has_ik;
}
