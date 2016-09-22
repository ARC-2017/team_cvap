#include <apc_bt_execute_action/look_for_object_action.h>

namespace apc
{
bool BTLookForObjectAction::fillGoal(apc_objects_detection_action::LookForObjectGoal &goal)
{
  std::vector<std::string> list;
  std::vector<std::string> rgbd_objects;
  std::string target;
  std::string bin;
  std::string mode;

  if (!fillParameter("/apc/task_manager/target_object", target) ||
      !fillParameter("/apc/task_manager/target_bin", bin) ||
      !fillParameter("/apc/task_manager/rgbd_objects", rgbd_objects))
  {
    return false;
  }

  fillParameter("/apc/task_manager/non_target_bin_items", list);


  if (nameIsInList(rgbd_objects, target))
  {
    goal.targetObj = target;
    goal.objectslist = list;
    goal.cameraID = "kinect_head";
    goal.methodID = "rgbd";
    goal.globalSearch = false;
    goal.binID = bin;
    return true;
  }
  return false;
}

bool BTLookForObjectAction::processResult(const apc_objects_detection_action::LookForObjectResultConstPtr &result)
{
    if (result->trueTF)
    {
        setParameter("/apc/bt/true_tf", true);
    }
    else
    {
        setParameter("/apc/bt/true_tf", false);
    }
    return true;
}

double BTLookForObjectAction::getTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/look_action/timeout_kinect", 30.0, timeout);

  return timeout;
}

double BTLookForObjectAction::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/look_action/preempt_timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "look_bin_kinect");
  apc::BTLookForObjectAction action(ros::this_node::getName(), "lookforobject", "look_bin_kinect_action_baxter");
  ros::spin();
  return 0;
}
