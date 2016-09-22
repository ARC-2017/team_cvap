#ifndef __PLACE_OBJECT__
#define __PLACE_OBJECT__
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <apc_manipulation/PlaceObjectAction.h>
#include <apc_manipulation/RobotInterface.h>
#include <apc_manipulation/ManipulationException.h>

namespace apc
{
/*
    Class that implements the move arm actionlib server
*/
class PlaceObject
{
public:
  PlaceObject(std::string action_name);
  void callback(const apc_manipulation::PlaceObjectGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  std::string action_name_;
  RobotInterface interface_;
  apc_manipulation::PlaceObjectResult result_;
  actionlib::SimpleActionServer<apc_manipulation::PlaceObjectAction> action_server_;

  void returnSuccess();
  void returnError();
  void returnFail();
};
}
#endif
