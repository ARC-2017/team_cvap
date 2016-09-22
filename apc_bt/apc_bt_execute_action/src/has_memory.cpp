#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int Condition::executeCB(ros::Duration dt)
{
  bool has_memory;
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  fillParameter("/apc/task_manager/memory/", false,  has_memory);

  if (has_memory)
  {
    setParameter("/apc/task_manager/memory/", false);
    set_feedback(SUCCESS);
    return 1;
  }

  set_feedback(FAILURE);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "has_memory");
  apc::Condition action(ros::this_node::getName(), "has_memory_baxter");
  ros::spin();
  return 0;
}
