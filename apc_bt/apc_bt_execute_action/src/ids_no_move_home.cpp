#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

/*
    The IDS no move home will return true if the arm we want to move for the IDS pose
    corresponds to the last arm we used
*/
int Condition::executeCB(ros::Duration dt)
{
  set_feedback(RUNNING);
  std::string old_arm, new_arm;

  if (!isSystemActive())
  {
    return 1;
  }

  if (!fillParameter("/apc/bt/old_ids", old_arm) || !fillParameter("/apc/bt/ids", new_arm))
  {
      set_feedback(FAILURE);
      return 1;
  }

  if (old_arm == new_arm) // no need to move
  {
      set_feedback(SUCCESS);
      return 1;
  }

  set_feedback(FAILURE);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ids_no_move_home");
  apc::Condition action(ros::this_node::getName(), "ids_no_move_home_baxter");
  ros::spin();
  return 0;
}
