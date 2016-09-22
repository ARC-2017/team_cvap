#ifndef APC_MOTION_PLANNER_H
#define APC_MOTION_PLANNER_H
// STL includes
#include <string>
#include <vector>
// Moveit includes
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
// Eigen includes
#include <Eigen/Core>
// ROS includes
#include "ros/ros.h"
#include "apc_linear_path_planner/PlanLinearPath.h"
#include "apc_linear_path_planner/InvertTrajectory.h"
#include "apc_linear_path_planner/ComputeIKSolution.h"
#include "apc_linear_path_planner/PlanCartesianLinearPath.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

namespace apc_linear_path_planner {

  struct APCMotionPlannerParameters {
    std::string _planning_scene_topic;
    std::string _joint_states_topic;
    std::string _debug_planning_scene_topic;
    std::string _robot_description;
    bool _debug;
    double _sampling_step;
  };

  class APCMotionPlanner {
  public:
    APCMotionPlanner(const APCMotionPlannerParameters& params);
    ~APCMotionPlanner();

    bool serviceCallback(apc_linear_path_planner::PlanLinearPath::Request& req, 
        apc_linear_path_planner::PlanLinearPath::Response& res);

    bool computeIKSolutionServiceCallback(apc_linear_path_planner::ComputeIKSolution::Request& req,
        apc_linear_path_planner::ComputeIKSolution::Response& res);

	  bool invertTrajectoryServiceCallback(apc_linear_path_planner::InvertTrajectory::Request& req,
        apc_linear_path_planner::InvertTrajectory::Response& res);

    bool planCartesianLinearPathCallback(apc_linear_path_planner::PlanCartesianLinearPath::Request& req,
        apc_linear_path_planner::PlanCartesianLinearPath::Response& res);
    /**
      Attempts to compute a trajectory from the first given configuration to the second given configuration.
      @param joint_names - list of joint names of the first configuration
      @param joint_values_start - list of joint values (same order as joint names) of the first config
      @param joint_values_goal - list of joint values (same order as joint names) of the second config
      @return a serialized trajectory (with length 0 if no path was found)
    */
    bool plan(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values_start,
        const std::vector<double>& joint_values_goal, const std::string& move_group_name, moveit_msgs::RobotTrajectory& traj_msg);

  private:
    boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> _planning_scene_monitor;
    // ros::Publisher _scene_publisher;
    APCMotionPlannerParameters _args;
    // void publishDebugScene();
    void init();
    void setConfiguration(moveit::core::RobotState& robot_state,
      const std::vector<std::string>& joint_names,
      const std::vector<double>& joint_values);
    void setConfiguration(moveit::core::RobotState& robot_state,
      const std::vector<std::string>& joint_names,
      const Eigen::VectorXd& joint_values);
    void toEigen(const std::vector<double>& joint_values, Eigen::VectorXd& vector);
    void fromEigen(std::vector<double>& joint_values, const Eigen::VectorXd& vector);
    void printConfiguration(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);
    // Extract the joint values of joints joint_names from the given joint state message js and store them in values.
    // Return true if successful, else false
    bool extractJointValues(const std::vector<std::string>& joint_names, const sensor_msgs::JointState js, std::vector<double>& values);
    bool checkBounds(const std::vector<std::string>& joint_names, moveit::core::RobotState& robot_state) const;
    void finalizeTrajectoryMsg(robot_trajectory::RobotTrajectory& traj_obj, moveit_msgs::RobotTrajectory& traj_msg);
    bool computeIKSolution(const geometry_msgs::Pose& global_pose, const std::string& move_group_name,
        planning_scene_monitor::LockedPlanningSceneRW& planning_scene,
        moveit::core::RobotState& robot_state);
  };
}
#endif
