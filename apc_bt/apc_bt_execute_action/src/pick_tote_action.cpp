#include <apc_bt_execute_action/pick_action.h>

namespace apc
{
bool BTPickAction::fillGoal(apc_manipulation::PickObjectGoal &goal)
{
  bool reliable;
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
  fillParameter("/apc/bt/true_tf", true, reliable);
  goal.is_tf_reliable = reliable;
  
  return true;
}

double BTPickAction::getTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/pick_action/timeout", 30.0, timeout);

  return timeout;
}

double BTPickAction::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/pick_action/preempt_timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_tote_action");
  apc::BTPickAction action(ros::this_node::getName(), "/apc/manipulation/pick_object", "pick_tote_action_"
                                                                                       "baxter");
  ros::spin();
  return 0;
}
