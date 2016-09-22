#include <apc_bt_test_server/apc_bt_test_place.h>

namespace apc
{
bool BTTestPlace::parseGoal(const apc_manipulation::PlaceObjectGoalConstPtr &goal)
{
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "place_action_test");
  apc::BTTestPlace action(ros::this_node::getName(), "/apc/manipulation/place_object");
  ros::spin();
  return 0;
}
