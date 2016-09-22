#include <apc_bt_execute_action/set_ids_action.h>

namespace apc
{
SetIDSAction::SetIDSAction(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int SetIDSAction::executeCB(ros::Duration dt)
{
  set_feedback(RUNNING);

  setParameter("/apc/task_manager/memory/", true);

  set_feedback(SUCCESS);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_memory");
  apc::SetIDSAction action(ros::this_node::getName(), "set_memory_baxter");
  ros::spin();
  return 0;
}
