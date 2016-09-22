#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int Condition::executeCB(ros::Duration dt)
{
  std::string object_name, ids_arm, ids_bin;
  bool has_memory;
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  fillParameter("/apc/task_manager/target_object", object_name);
  fillParameter("/apc/task_manager/memory/" + object_name, false,  has_memory);

  if (has_memory)
  {
    fillParameter("/apc/task_manager/memory/ids_arm", ids_arm);
    fillParameter("/apc/task_manager/memory/ids_bin", ids_bin);

    setParameter("/apc/bt/ids", ids_arm);
    setParameter("/apc/bt/ids_bin", ids_bin);
    setParameter("/apc/task_manager/memory/" + object_name, false);
    set_feedback(SUCCESS);
    return 1;
  }

  set_feedback(FAILURE);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "has_memory_ids");
  apc::Condition action(ros::this_node::getName(), "has_memory_ids_baxter");
  ros::spin();
  return 0;
}
