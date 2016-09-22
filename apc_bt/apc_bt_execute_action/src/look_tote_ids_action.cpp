#include <apc_bt_execute_action/look_for_object_action.h>

namespace apc
{
bool BTLookForObjectAction::fillGoal(apc_objects_detection_action::LookForObjectGoal &goal)
{
  std::vector<std::string> list;
  std::string target;
  std::string bin;
  std::string mode;
  std::string arm;

  if (!fillParameter("/apc/task_manager/target_object", target) ||
      !fillParameter("/apc/task_manager/target_bin", bin) || !fillParameter("/apc/bt/ids", arm))
  {
    return false;
  }

  fillParameter("/apc/task_manager/non_target_bin_items", list);

  goal.targetObj = target;
  goal.objectslist = list;

  if (arm == "left_arm")
  {
    goal.cameraID = "ids_left";
  }
  else
  {
    goal.cameraID = "ids_right";
  }

  goal.globalSearch = false;
  goal.binID = "tote";
  return true;
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
  fillParameter("/apc/bt/look_action/timeout", 30.0, timeout);

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
  ros::init(argc, argv, "look_tote_ids");
  apc::BTLookForObjectAction action(ros::this_node::getName(), "lookforobject", "look_tote_ids_action_baxter");
  ros::spin();
  return 0;
}
