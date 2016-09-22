#include <apc_manipulation/MoveArm.h>

namespace apc
{
MoveArm::MoveArm(std::string action_name)
  : action_server_(nh_, action_name, boost::bind(&MoveArm::callback, this, _1), false), action_name_(action_name)
{
  action_server_.start();
}

void MoveArm::callback(const apc_manipulation::MoveArmGoalConstPtr &actionGoal)
{
  Configuration goal_conf;
  Eigen::Affine3d goal_pose;
  moveit::planning_interface::MoveGroup::Plan planned_path;
  MoveResult result;

  std::string goal_name, arm_name;

  bool move_to_configuration = false;
  bool move_to_pose = false;

  goal_name = actionGoal->goalBin;
  arm_name = actionGoal->arm;

  try
  {
    interface_.acquireLock();

    if (goal_name == "HOME")
    {
      move_to_configuration = true;
      goal_conf = interface_.getNamedConfiguration(goal_name, arm_name);
    }
    else
    {
      move_to_pose = true;
      goal_pose = interface_.getEigenTransform(goal_name);
    }

    // TODO: what if the goal is unvalid? We cannot set goal_x to None
    // if (??)
    // {
    //   interface_.releaseLock();
    //   throw ManipulationException(std::string("Unknown goal"));
    // }

    if (move_to_pose)
    {
      result = interface_.moveToPose(goal_pose, arm_name, planned_path);
    }

    if (move_to_configuration)
    {
      result = interface_.moveToConfiguration(goal_conf, arm_name, planned_path);
    }

    returnSuccess();
  }
  catch (ManipulationException &e)
  {
    ROS_ERROR("Could not move arm because: %s", e.what());
    returnError();
  }
}

void MoveArm::returnSuccess()
{
  ROS_INFO("MoveArm executed successfully.");
  result_.success = true;
  action_server_.setSucceeded(result_);
}

void MoveArm::returnError()
{
  result_.success = false;
  action_server_.setAborted(result_);
}

void MoveArm::returnFail()
{
  ROS_INFO("MoveArm executed properly, but failed.");
  result_.success = false;
  action_server_.setAborted(result_);
}
}

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "move_action");
//   apc::MoveArm action(ros::this_node::getName(), "/apc/manipulation/move_arm");
//   ros::spin();
//   return 0;
// }
