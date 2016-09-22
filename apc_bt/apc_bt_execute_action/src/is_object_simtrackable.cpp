#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int Condition::executeCB(ros::Duration dt)
{
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  // INSERT CODE TO CHECK IF OBJECT IS SIMTRACKABLE

  set_feedback(SUCCESS);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_object_simtrackable");
  apc::Condition action(ros::this_node::getName(), "is_object_simtrackable_baxter");
  ros::spin();
  return 0;
}
