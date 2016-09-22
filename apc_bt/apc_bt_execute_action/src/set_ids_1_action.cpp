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

  std::string bin, ids_bin, old_arm, arm, task;

  fillParameter("/apc/task", task);

  if (task != "stow")
  {
      if (!fillParameter("/apc/task_manager/target_bin", bin))
      {
          set_feedback(FAILURE);
          return 1;
      }
  }
  else
  {
      bin = "tote";
  }

  ids_bin = bin + "_1";
  arm = left_arm;

  fillParameter("/apc/bt/ids", arm, old_arm); // get old ids arm, or use the current one, if not set

  setParameter("/apc/bt/old_ids", old_arm);
  setParameter("/apc/bt/ids", arm);
  setParameter("/apc/bt/ids_bin", ids_bin);

  set_feedback(SUCCESS);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_ids_1_action");
  apc::SetIDSAction action(ros::this_node::getName(), "set_ids_1_action_baxter");
  ros::spin();
  return 0;
}
