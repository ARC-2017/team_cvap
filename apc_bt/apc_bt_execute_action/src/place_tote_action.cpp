#include <apc_bt_execute_action/place_action.h>

namespace apc
{
bool BTPlaceAction::fillGoal(apc_manipulation::PlaceObjectGoal &goal)
{
  if (!fillParameter("/apc/task_manager/target_object", goal.targetObject))
  {
    return false;
  }

  goal.bin_id = "tote";

  if (!fillParameter("/apc/task_manager/arm", goal.arm))
  {
    return false;
  }
  // TODO : Fill in possible obstacles
  return true;
}

double BTPlaceAction::getTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/place_action/timeout", 30.0, timeout);

  return timeout;
}

double BTPlaceAction::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/place_action/preempt_timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "place_tote_action");
  apc::BTPlaceAction action(ros::this_node::getName(), "/apc/manipulation/place_object", "place_tote_action_"
                                                                                         "baxter");
  ros::spin();
  return 0;
}
