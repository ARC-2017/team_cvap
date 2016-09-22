#include <apc_bt_execute_action/manager_action.h>

namespace apc
{
bool BTManagerAction::fillGoal(apc_bt_comms::ManagerGoal &goal)
{
  goal.pop = true;
  goal.success = false;
  return true;
}

double BTManagerAction::getTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/get_next_target/timeout", 3.0, timeout);

  return timeout;
}

double BTManagerAction::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/get_next_target/preempt_timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_next_target_action");
  apc::BTManagerAction action(ros::this_node::getName(), "/apc/task_manager", "get_next_target_action_baxter");
  ros::spin();
  return 0;
}
