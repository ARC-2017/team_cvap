#include <apc_bt_execute_action/set_ids_action.h>

namespace apc
{
SetIDSAction::SetIDSAction(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int SetIDSAction::executeCB(ros::Duration dt)
{
  std::string object_name;
  std::string ids_arm;
  std::string ids_bin;
  set_feedback(RUNNING);


  fillParameter("/apc/task_manager/target_object", object_name);
  fillParameter("/apc/bt/ids", ids_arm);
  fillParameter("/apc/bt/ids_bin", ids_bin);
  setParameter("/apc/task_manager/memory/" + object_name, true);
  setParameter("/apc/task_manager/memory/ids_arm", ids_arm);
  setParameter("/apc/task_manager/memory/ids_bin", ids_bin);


  set_feedback(SUCCESS);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_memory_ids");
  apc::SetIDSAction action(ros::this_node::getName(), "set_memory_ids_baxter");
  ros::spin();
  return 0;
}
