#include <apc_manipulation/PickObject.h>

namespace apc
{
PickObject::PickObject(std::string action_name)
  : action_server_(nh_, action_name, boost::bind(&PickObject::callback, this, _1), false), action_name_(action_name)
{
  action_server_.start();
}

void PickObject::callback(const apc_manipulation::PickObjectGoalConstPtr &actionGoal)
{
  BaxterGripperPtr gripper;
  MoveGroupPtr move_group;
  std::vector<std::string> objects_to_add;
  std::vector<geometry_msgs::PoseStamped> approach_poses, grasp_samples;
  std::string arm_name, gripper_name, gripper_frame_name, object_frame_name;

  double approach_dist;

  bool move_to_configuration = false;
  bool move_to_pose = false;

  arm_name = actionGoal->arm;

  try
  {
    interface_.acquireLock();
    gripper_name = interface_.getGripperName(arm_name);
    gripper_frame_name = interface_.getGripperFrameName(arm_name);
    object_frame_name = interface_.getObjectFrameName(actionGoal->targetObject);
    gripper = interface_.getGripper(gripper_name);
    move_group = interface_.getMoveGroup(arm_name);
    objects_to_add = actionGoal->obstacles;

    interface_.addObjectsToPlanningScene(objects_to_add);

    // loadTSRs(actionGoal.targetObject, tsr_list_, tsr_weights_);
    // sampleTSRs(tsr_list_, tsr_weights_, grasp_samples_, post_tolerance_, rot_tolerance_);

    gripper->open();
    interface_.moveHeadAside(arm_name);
    approach_dist = 0.08;
    approach_poses = computeApproachPoses(grasp_samples, approach_dist, object_frame_name);
    // transformPoseArray(approach_poses, "/base");
    move_group->setPoseTargets(approach_poses);
  }
  catch (ManipulationException &e)
  {
    ROS_ERROR("Could not pick because: %s", e.what());
    returnFail();
  }
}

std::vector<geometry_msgs::PoseStamped>
PickObject::computeApproachPoses(const std::vector<geometry_msgs::PoseStamped> &grasp_poses, const double approach_dist,
                                 const std::string &object_frame_name)
{
  std::vector<geometry_msgs::PoseStamped> approach_poses;

  return grasp_poses;  // Need TSR
}

void PickObject::returnSuccess()
{
  ROS_INFO("PickObject executed successfully.");
  result_.success = true;
  action_server_.setSucceeded(result_);
}

void PickObject::returnError()
{
  result_.success = false;
  action_server_.setAborted(result_);
}

void PickObject::returnFail()
{
  ROS_INFO("PickObject executed properly, but failed.");
  result_.success = false;
  action_server_.setAborted(result_);
}
}

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "pick_action");
//   apc::PickObject action(ros::this_node::getName(), "/apc/manipulation/pick_object");
//   ros::spin();
//   return 0;
// }
