#include <apc_bt_test_server/apc_bt_test_pick.h>

namespace apc
{
bool BTTestPick::parseGoal(const apc_manipulation::PickObjectGoalConstPtr &goal)
{
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_action_test");
  apc::BTTestPick action(ros::this_node::getName(), "/apc/manipulation/pick_object");
  ros::spin();
  return 0;
}
