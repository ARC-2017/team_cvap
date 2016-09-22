#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

/*
    The pick no move home will return true if the arm we want to move for the pick pose
    corresponds to the last arm we used for moving the IDS cameras
*/
int Condition::executeCB(ros::Duration dt)
{
  set_feedback(RUNNING);
  std::string old_arm, new_arm;

  if (!isSystemActive())
  {
    return 1;
  }

  if (!fillParameter("/apc/bt/ids", old_arm) || !fillParameter("/apc/task_manager/arm", new_arm))
  {
      set_feedback(FAILURE);
      return 1;
  }

  if (old_arm == new_arm) // no need to move
  {
      ROS_INFO("WE ARE GOING TO ATTEMPT PICK WITH THE SAME ARM AS FOR LOOKING. ALL GOOD");
      set_feedback(SUCCESS);
      return 1;
  }

  ROS_INFO("WE ARE GOING TO ATTEMPT PICK WITH A DIFFERENT ARM AS FOR LOOKING. NEED TO MOVE");
  set_feedback(FAILURE);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_no_move_home");
  apc::Condition action(ros::this_node::getName(), "pick_no_move_home_baxter");
  ros::spin();
  return 0;
}
