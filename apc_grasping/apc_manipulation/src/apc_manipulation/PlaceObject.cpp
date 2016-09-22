#include <apc_manipulation/PlaceObject.h>

namespace apc
{
PlaceObject::PlaceObject(std::string action_name)
  : action_server_(nh_, action_name, boost::bind(&PlaceObject::callback, this, _1), false), action_name_(action_name)
{
  action_server_.start();
}

void PlaceObject::callback(const apc_manipulation::PlaceObjectGoalConstPtr &actionGoal)
{
  BaxterGripperPtr gripper;

  std::string goal_name, arm_name, gripper_name;

  goal_name = actionGoal->bin_id;
  arm_name = actionGoal->arm;

  try
  {
    interface_.acquireLock();
    gripper_name = interface_.getGripperName(arm_name);
    gripper = interface_.getGripper(gripper_name);
    gripper->open();
    interface_.releaseLock();
    returnSuccess();
  }
  catch (ManipulationException &e)
  {
    ROS_ERROR("Could not place object because: %s", e.what());
    returnError();
  }
}

void PlaceObject::returnSuccess()
{
  ROS_INFO("PlaceObject executed successfully.");
  result_.success = true;
  action_server_.setSucceeded(result_);
}

void PlaceObject::returnError()
{
  result_.success = false;
  action_server_.setAborted(result_);
}

void PlaceObject::returnFail()
{
  ROS_INFO("PlaceObject executed properly, but failed.");
  result_.success = false;
  action_server_.setAborted(result_);
}
}
