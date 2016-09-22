#ifndef __MOVE_ARM__
#define __MOVE_ARM__
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <apc_manipulation/MoveArmAction.h>
#include <apc_manipulation/RobotInterface.h>
#include <apc_manipulation/ManipulationException.h>

namespace apc
{
/*
    Class that implements the move arm actionlib server
*/
class MoveArm
{
public:
  MoveArm(std::string action_name);
  void callback(const apc_manipulation::MoveArmGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  std::string action_name_;
  RobotInterface interface_;
  apc_manipulation::MoveArmResult result_;
  actionlib::SimpleActionServer<apc_manipulation::MoveArmAction> action_server_;

  void returnSuccess();
  void returnError();
  void returnFail();
};
}
#endif
