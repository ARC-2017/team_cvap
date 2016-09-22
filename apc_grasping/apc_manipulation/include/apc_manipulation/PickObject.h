#ifndef __PICK_OBJECT__
#define __PICK_OBJECT__
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <apc_manipulation/RobotInterface.h>
#include <apc_manipulation/BaxterGripper.h>
#include <apc_manipulation/PickObjectAction.h>
#include <apc_manipulation/ManipulationException.h>

namespace apc
{
/*
    Class that implements the move arm actionlib server
*/
class PickObject
{
public:
  PickObject(std::string action_name);
  void callback(const apc_manipulation::PickObjectGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  std::string action_name_;
  RobotInterface interface_;
  apc_manipulation::PickObjectResult result_;
  actionlib::SimpleActionServer<apc_manipulation::PickObjectAction> action_server_;

  void returnSuccess();
  void returnError();
  void returnFail();

  /*
    Returns approach poses for the given grasp poses.
    @param graspPoses - a list of poses ([x,y,z,r,p,y])
    @param approachDist - distance to offset along eef z axis
    @param objectFrameName - name of the object frame
    @return list of approach poses as PoseStamped
  */
  std::vector<geometry_msgs::PoseStamped>
  computeApproachPoses(const std::vector<geometry_msgs::PoseStamped> &grasp_poses, const double approach_dist,
                       const std::string &object_frame_name);
};
}
#endif
