#include <apc_bt_test_server/apc_bt_test_move.h>

namespace apc
{
bool BTTestMove::parseGoal(const apc_manipulation::MoveArmGoalConstPtr &goal)
{
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_action_test");
  apc::BTTestMove action(ros::this_node::getName(), "/apc/manipulation/move_arm");
  ros::spin();
  return 0;
}
