#include <apc_bt_execute_action/condition.h>

namespace apc
{
Condition::Condition(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int Condition::executeCB(ros::Duration dt)
{
  bool place_failure;
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  fillParameter("/apc/task_manager/place_failure", false, place_failure);

  if(place_failure)
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
  ros::init(argc, argv, "is_place_failure");
  apc::Condition action(ros::this_node::getName(), "is_place_failure_baxter");
  ros::spin();
  return 0;
}
