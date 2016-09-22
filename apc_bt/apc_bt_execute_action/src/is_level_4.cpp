#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int Condition::executeCB(ros::Duration dt)
{
  Desperation desperation_mode;
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  fillParameter("/apc/task_manager/desperation", desperation_mode);

  if (desperation_mode == level_4)
  {
    set_feedback(SUCCESS);
  }
  else
  {
    set_feedback(FAILURE);
  }

  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "is_level_4");
  apc::Condition action(ros::this_node::getName(), "is_level_4_baxter");
  ros::spin();
  return 0;
}
