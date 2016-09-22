#include <apc_bt_execute_action/move_action.h>

namespace apc
{
bool BTMoveAction::fillGoal(apc_manipulation::MoveArmGoal &goal)
{
  goal.goalBin = home;
  goal.arm = both_arms;
  return true;
}

double BTMoveAction::getTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/move_action/timeout", 30.0, timeout);

  return timeout;
}

double BTMoveAction::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/move_action/preempt_timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_home_action");
  apc::BTMoveAction action(ros::this_node::getName(), "/apc/manipulation/move_arm", "move_to_home_action_"
                                                                                    "baxter");
  ros::spin();
  return 0;
}
